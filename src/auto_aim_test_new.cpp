#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard_uart.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/test2.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // Use CBoardUART for sending commands
  io::CBoardUART cboard(config_path);
  
  // Use DM_IMU for receiving quaternion
  // io::DM_IMU imu;

  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  
  // Wait for initial data (optional, but good practice)
  std::cout << "Waiting for devices..." << std::endl;
  std::this_thread::sleep_for(1s);

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();

    while (!quit) {
      auto target = target_queue.front();
      
      // Get bullet speed from cboard
      double bullet_speed = cboard.bullet_speed;
      
      auto plan = planner.plan(target, bullet_speed);

      // Construct io::Command
      io::Command cmd;
      cmd.control = plan.control;
      cmd.shoot = plan.fire;
      cmd.yaw = plan.yaw;  // Adjust for any gimbal offset
      cmd.pitch = plan.pitch;
      // Other fields in cmd (like horizon_distance) are default 0 or ignored if not used by firmware
      
      cboard.send(cmd);

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      // Note: We don't have full gimbal state feedback here like in auto_aim_debug_mpc (position, velocity)
      // unless we trust cboard.bullet_speed updates and maybe if we had position feedback.
      // But CBoardUART mainly reads quaternion and simple state. 
      // If we want to plot current yaw/pitch, we would need to read it from somewhere.
      // The DM_IMU gives us quaternion (orientation), but not necessarily gimbal joint angles unless we calculate them.
      // auto_aim_debug_mpc used `gimbal.state()` which returns joint angles.
      // CBoardUART::imu_at returns world orientation (quaternion).
      // If we need joint angles for plotting or closed loop, we might be missing them if CBoardUART doesn't provide them.
      // However, the user said "quaternion from dm_imu", implying the solver needs world orientation.
      // The planner outputs absolute target yaw/pitch (or relative? need to check).
      
      // Usually Planner outputs ABSOLUTE yaw/pitch in world frame or gimbal frame?
      // auto_aim_debug_mpc: gimbal.send(plan.yaw, ...) 
      // cboard.send uses command.yaw. 
      // If planner works in absolute coordinates, and cboard expects relative or absolute... 
      // Typically auto-aim works in absolute or relative-to-gimbal.
      // Let's assume the logic is consistent with auto_aim_debug_mpc.

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw * 57.3;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch * 57.3;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 10 : 0;
      data["control"] = cmd.control ? 10 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    
    // Get quaternion from DM_IMU
    // 由于IMU横着装，需要应用旋转变换来校正坐标系
    // 如果IMU X轴指向右侧，使用 +M_PI / 2
    // 如果IMU X轴指向左侧，使用 -M_PI / 2
    Eigen::Quaterniond q = cboard.imu_at(t);
    // Eigen::Quaterniond q_adjusted = q * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ());
    // q_adjusted.normalize();

    auto eulers = tools::eulers(q, 2, 1, 0);  // For debugging
    nlohmann::json data;
    data["imu_roll"] = eulers[0] * 57.3;
    data["imu_pitch"] = eulers[1] * 57.3;
    data["imu_yaw"] = eulers[2] * 57.3;
    plotter.plot(data);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {255, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  
  // Stop gimbal
  io::Command stop_cmd; // default is all false/0
  cboard.send(stop_cmd);

  return 0;
}
