#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

#pragma pack(push, 1)
struct robot_status_t {
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management; 
};

struct robot_pos_t {
  float x;
  float y;
  float angle;
};

struct game_robot_HP_t {
  uint16_t ally_1_robot_HP;
  uint16_t ally_2_robot_HP;
  uint16_t ally_3_robot_HP;
  uint16_t ally_4_robot_HP;
  uint16_t reserved;
  uint16_t ally_7_robot_HP;
  uint16_t ally_outpost_HP;
  uint16_t ally_base_HP;
};
#pragma pack(pop)

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  // tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  io::ROS2 ros2;
  
  std::mutex referee_mutex;
  robot_status_t robot_status = {0};
  robot_pos_t robot_pos = {1.0f, 1.0f, 0.0f}; // 初始位置假设为(1, 1)
  game_robot_HP_t game_robot_hp = {0};

  // io::CBoard cboard(config_path);
  io::Gimbal gimbal(config_path, nullptr, [&](uint16_t cmd_id, const uint8_t* data, uint16_t len){
    std::lock_guard<std::mutex> lock(referee_mutex);
    if (cmd_id == 0x0201 && len >= sizeof(robot_status_t)) {
      std::memcpy(&robot_status, data, sizeof(robot_status_t));
    } else if (cmd_id == 0x0203 && len >= sizeof(robot_pos_t)) {
      std::memcpy(&robot_pos, data, sizeof(robot_pos_t));
    } else if (cmd_id == 0x0003 && len >= sizeof(game_robot_HP_t)) {
      std::memcpy(&game_robot_hp, data, sizeof(game_robot_HP_t));
    }
  }, nullptr);
  io::Camera camera(config_path);
  io::DM_IMU dm_imu{};
  // io::Camera back_camera("configs/camera.yaml");
  // io::USBCamera usbcam1("video0", config_path);
  // io::USBCamera usbcam2("video2", config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  std::mutex nav_mutex;
  io::NavData current_nav_data;

  auto NavThread = std::thread([&]() { 
    enum class State {
      MOVING_TO_TARGET,
      RETREATING,
      RECOVERING
    } state = State::MOVING_TO_TARGET;
    
    // 假设初始位置为 (1, 1)，沿 x 轴走 6m 到达 (7, 1)
    float target_x = 1.0f;
    float target_y = 7.0f;
    float home_x = 1.0f;
    float home_y = 1.0f;

    auto start_time = std::chrono::steady_clock::now();
    while (!exiter.exit()) {
      auto now = std::chrono::steady_clock::now();
      
      float current_x, current_y;
      uint16_t current_hp, max_hp;
      
      {
        std::lock_guard<std::mutex> lock(referee_mutex);
        current_x = robot_pos.x;
        current_y = robot_pos.y;
        
        // 优先使用 0x0201 的状态，如果为空则尝试使用 0x0003 的状态
        current_hp = robot_status.current_HP > 0 ? robot_status.current_HP : game_robot_hp.ally_7_robot_HP;
        max_hp = robot_status.maximum_HP > 0 ? robot_status.maximum_HP : 600; 
      }

      float error_x = 0;
      float error_y = 0;

      // 状态机判断
      if (state == State::MOVING_TO_TARGET) {
        if (current_hp > 0 && current_hp < max_hp * 0.3) {  // 低于 30% 血量视作低血量
          state = State::RETREATING;
        } else {
          error_x = target_x - current_x;
          error_y = target_y - current_y;
        }
      } else if (state == State::RETREATING) {
        error_x = home_x - current_x;
        error_y = home_y - current_y;
        // 判断是否回到原点附近 (误差半径 0.2m)
        if (std::abs(error_x) < 0.2f && std::abs(error_y) < 0.2f) {
          state = State::RECOVERING;
        }
      } else if (state == State::RECOVERING) {
        // 血量回满或达到 90%
        if (current_hp >= max_hp * 0.9) { 
          state = State::MOVING_TO_TARGET;
        }
        error_x = 0; 
        error_y = 0;
      }

      io::NavData data = {0};
      
      // 简单 P 控制转换误差为速度，并截断至 [-0.5, 0.5]
      float kp = 0.5f;
      float vx = kp * error_x;
      float vy = kp * error_y;
      
      if (vx > 0.5f) vx = 0.5f; else if (vx < -0.5f) vx = -0.5f;
      if (vy > 0.5f) vy = 0.5f; else if (vy < -0.5f) vy = -0.5f;

      data.linear_x = vx;
      data.linear_y = vy;
      data.angular_z = 0; 
      
      {
        std::lock_guard<std::mutex> lock(nav_mutex);
        current_nav_data = data;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 模拟50Hz的频率
    }
  });

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    //Eigen::Quaterniond q = dm_imu.imu_at(timestamp);
    auto q = gimbal.q(timestamp);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    // decider.get_invincible_armor(ros2.subscribe_enemy_status()); // 相关接口尚屏蔽

    // decider.armor_filter(armors);

    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    // decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    static auto last_seen_time = timestamp;
    if (tracker.state() != "lost") {
      last_seen_time = timestamp;
    }

    /// 全向感知逻辑
    if (tracker.state() == "lost") {
      if (std::chrono::duration<double>(timestamp - last_seen_time).count() > 1.0) {
        // command = decider.decide(yolo, gimbal_pos, camera);
        static auto t0 = timestamp;
        double t = std::chrono::duration<double>(timestamp - t0).count();
        
        command.control = true;
        // 使云台yaw以1.5弧度/秒不断旋转并映射至[-PI, PI)
        command.yaw = 0.5 * std::sin(t * 1.5) * M_PI;  // 1.5 rad/s的正弦波，幅值为PI
        // if (command.yaw > M_PI) command.yaw -= 2 * M_PI;
        
        // pitch以2 rad/s的速度，在-0.2到+0.2弧度上下摆动
        command.pitch = 0.2 * std::sin(t * 3.0);
        command.shoot = false;
      }
    } else {
      command = aimer.aim(targets, timestamp, gimbal.state().bullet_speed);
    }

    /// 发射逻辑
    if (tracker.state() != "lost") {
      Eigen::Quaterniond current_q = gimbal.q(timestamp);
      Eigen::Vector3d gimbal_pos_shoot = tools::eulers(current_q, 2, 1, 0);
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos_shoot);
    } else {
      command.shoot = false;
    }
    command.shoot = false;
    
    io::NavData nav_data;
    {
      std::lock_guard<std::mutex> lock(nav_mutex);
      nav_data = current_nav_data;
    }
    
    gimbal.send(command);
    gimbal.send_imu_forward(dm_imu);
    gimbal.send_cmd_vel(nav_data);

    /// ROS2通信
    // Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    // ros2.publish(target_info);
  }
  if (NavThread.joinable()) NavThread.join();
  return 0;
}