#include "sentry_decision.hpp"

#include <algorithm>
#include <iostream>

namespace sentry_decision {

SentryDecider::SentryDecider() {
}

SentryDecider::~SentryDecider() {
  stop_nav_thread();
}

void SentryDecider::referee_callback(uint16_t cmd_id, const uint8_t* data, uint16_t len) {
  std::lock_guard<std::mutex> lock(referee_mutex);
  if (cmd_id == 0x0201 && len >= sizeof(robot_status_t)) {
    std::memcpy(&robot_status, data, sizeof(robot_status_t));
  } else if (cmd_id == 0x0203 && len >= sizeof(robot_pos_t)) {
    std::memcpy(&robot_pos, data, sizeof(robot_pos_t));
  } else if (cmd_id == 0x0003 && len >= sizeof(game_robot_HP_t)) {
    std::memcpy(&game_robot_hp, data, sizeof(game_robot_HP_t));
  } else if (cmd_id == 0x0001 && len >= sizeof(game_status_t)) {
    std::memcpy(&game_status, data, sizeof(game_status_t));
  }
}

void SentryDecider::start_nav_thread(io::DM_IMU* imu) {
  if (!is_running) {
    imu_ = imu;
    is_running = true;
    nav_thread = std::thread(&SentryDecider::nav_loop, this);
  }
}

void SentryDecider::stop_nav_thread() {
  if (is_running) {
    is_running = false;
    if (nav_thread.joinable()) {
      nav_thread.join();
    }
  }
}

io::NavData SentryDecider::get_nav_data() {
  std::lock_guard<std::mutex> lock(nav_mutex);
  return current_nav_data;
}

uint8_t SentryDecider::get_game_progress() {
  std::lock_guard<std::mutex> lock(referee_mutex);
  return game_status.game_progress;
}

void SentryDecider::nav_loop() {
  enum class State {
    INIT,
    MOVING_OUT,
    RETURNING_HOME,
    WAITING_NEXT_DIR,
    RETREAT_TO_HOME,
    RECOVERING,
    STOP
  } state = 
#ifndef NDEBUG
    State::MOVING_OUT;
#else
    State::INIT;
#endif

  float pos_x = 0;
  float pos_y = 0;
  float vel_x = 0;
  float vel_y = 0;

  float initial_yaw = 0;
  bool is_initialized = false;
  // 方向序列定义在初始化坐标系中: +X前, -Y右, -X后, +Y左
  const float dirs[4][2] = {{1.0f, 0.0f}, {0.0f, -1.0f}, {-1.0f, 0.0f}, {0.0f, 1.0f}};
  // 用户要求优先 IMU x 轴左侧(当前安装对应云台初始方向右侧) -> 右侧方向(-Y)
  int current_dir_idx = 1;

  const float out_distance_m = 8.0f;
  const float slow_down_distance_m = 6.0f;

  State resume_state = State::MOVING_OUT;

  auto setup_time = std::chrono::steady_clock::now();
  auto last_time = std::chrono::steady_clock::now();
  auto wait_start_time = std::chrono::steady_clock::now();

  while (is_running) {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;
    if (dt <= 0.0f) dt = 0.02f;

    io::NavData data = {0};
    
    io::IMU_Data imu_data = {0};
    if (imu_ != nullptr) {
      imu_data = imu_->current_data();
    }

    uint16_t current_hp = 0;
    uint16_t max_hp = 600;
    {
      std::lock_guard<std::mutex> lock(referee_mutex);
      // 保留裁判系统血量逻辑：优先使用 0x0201，无数据时回退到 0x0003。
      current_hp = robot_status.current_HP > 0 ? robot_status.current_HP : game_robot_hp.ally_7_robot_HP;
      max_hp = robot_status.maximum_HP > 0 ? robot_status.maximum_HP : 600;
    }

    if (!is_initialized) {
      if (std::chrono::duration<double>(now - setup_time).count() > 1.5) {
        initial_yaw = imu_data.yaw;
        is_initialized = true;
        state = State::MOVING_OUT;
        pos_x = 0;
        pos_y = 0;
        vel_x = 0;
        vel_y = 0;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
    }

    // 转换加速度到起点的全局坐标系，带死区控制降低静态漂移
    float yaw_rad = (imu_data.yaw - initial_yaw) * M_PI / 180.0f;
    float acc_x_local = imu_data.accx * 9.8f;
    float acc_y_local = imu_data.accy * 9.8f;
    
    // 假设死区过滤极小漂移
    if (std::abs(acc_x_local) < 0.2f) acc_x_local = 0;
    if (std::abs(acc_y_local) < 0.2f) acc_y_local = 0;

    // 旋转到初始化坐标系 (X为前, Y为左)
    float acc_x_global = acc_x_local * std::cos(yaw_rad) - acc_y_local * std::sin(yaw_rad);
    float acc_y_global = acc_x_local * std::sin(yaw_rad) + acc_y_local * std::cos(yaw_rad);

    vel_x += acc_x_global * dt;
    vel_y += acc_y_global * dt;
    
    // 给速度添加阻尼，防止无限积分和随时间过度漂移
    vel_x *= 0.95f;
    vel_y *= 0.95f;

    pos_x += vel_x * dt;
    pos_y += vel_y * dt;

    // 撞墙检测 (例如加速度突增超过1.2g)
    bool hit_wall = std::hypot(imu_data.accx, imu_data.accy) > 1.2f;

    // 低血量时按原逻辑返航，回到原点后等待恢复再继续。
    bool hp_available = current_hp > 0 && max_hp > 0;
    bool low_hp = hp_available && current_hp < static_cast<uint16_t>(max_hp * 0.3f);
    bool hp_recovered = hp_available && current_hp >= static_cast<uint16_t>(max_hp * 0.9f);
    bool in_mission_stage =
      state == State::MOVING_OUT ||
      state == State::RETURNING_HOME ||
      state == State::WAITING_NEXT_DIR;

    if (in_mission_stage && low_hp) {
      resume_state = state;
      state = State::RETREAT_TO_HOME;
    }

    if (state == State::MOVING_OUT) {
      float dir_x = dirs[current_dir_idx][0];
      float dir_y = dirs[current_dir_idx][1];
      float progress = pos_x * dir_x + pos_y * dir_y;

      float speed = progress >= slow_down_distance_m ? 0.2f : 0.5f;
      data.linear_x = speed * dir_x;
      data.linear_y = speed * dir_y;

      if (hit_wall || progress >= out_distance_m) {
        state = State::RETURNING_HOME;
        vel_x = 0;
        vel_y = 0;
      }
    } else if (state == State::RETURNING_HOME) {
      float err_x = -pos_x;
      float err_y = -pos_y;
      float kp_home = 0.4f;
      data.linear_x = std::clamp(kp_home * err_x, -0.4f, 0.4f);
      data.linear_y = std::clamp(kp_home * err_y, -0.4f, 0.4f);

      if (std::abs(err_x) < 0.3f && std::abs(err_y) < 0.3f) {
        state = State::WAITING_NEXT_DIR;
        wait_start_time = now;
        vel_x = 0;
        vel_y = 0;
        pos_x = 0;
        pos_y = 0;
      }
    } else if (state == State::WAITING_NEXT_DIR) {
      data.linear_x = 0;
      data.linear_y = 0;

      if (std::chrono::duration<double>(now - wait_start_time).count() > 1.0) {
        current_dir_idx = (current_dir_idx + 1) % 4;
        state = State::MOVING_OUT;
      }
    } else if (state == State::RETREAT_TO_HOME) {
      // 低血量返航：回到起点(0,0)。
      float err_x = -pos_x;
      float err_y = -pos_y;
      float retreat_kp = 0.4f;
      data.linear_x = std::clamp(retreat_kp * err_x, -0.4f, 0.4f);
      data.linear_y = std::clamp(retreat_kp * err_y, -0.4f, 0.4f);

      if (std::abs(err_x) < 0.3f && std::abs(err_y) < 0.3f) {
        state = State::RECOVERING;
        vel_x = 0;
        vel_y = 0;
      }
    } else if (state == State::RECOVERING) {
      data.linear_x = 0;
      data.linear_y = 0;

      // 离线模式没有血量时，不会进此状态；在线模式恢复到90%继续执行原任务。
      if (!hp_available || hp_recovered) {
        state = resume_state;
      }
    } else if (state == State::STOP) {
      data.linear_x = 0;
      data.linear_y = 0;
    }

    {
      std::lock_guard<std::mutex> lock(nav_mutex);
      current_nav_data = data;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 模拟50Hz的频率
  }
}

} // namespace sentry_decision
