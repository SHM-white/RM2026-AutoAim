#include "sentry_decision.hpp"
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

void SentryDecider::start_nav_thread() {
  if (!is_running) {
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
    EXPLORING,
    RETURN_HOME_FOR_NEXT_DIR,
    WAITING_NEXT_DIR,
    MOVING_TO_TARGET,
    RETREATING,
    RECOVERING
  } state = State::INIT;
  
  float target_x = 0;
  float target_y = 0;
  float home_x = 0;
  float home_y = 0;
  
  int current_dir_idx = 0;
  // 0: +x, 1: -y, 2: -x, 3: +y 
  float dirs[4][2] = {{1, 0}, {0, -1}, {-1, 0}, {0, 1}};

  auto setup_time = std::chrono::steady_clock::now();
  auto stuck_check_start = std::chrono::steady_clock::now();
  auto wait_start_time = std::chrono::steady_clock::now();
  float stuck_check_x = 0;
  float stuck_check_y = 0;
  bool is_initialized = false;

  auto start_time = std::chrono::steady_clock::now();
  
  while (is_running) {
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

    if (!is_initialized) {
      if (std::chrono::duration<double>(now - setup_time).count() > 1.0) {
        home_x = current_x;
        home_y = current_y;
        is_initialized = true;
        state = State::EXPLORING;
        stuck_check_start = now;
        stuck_check_x = current_x;
        stuck_check_y = current_y;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
    }

    // 状态机判断
    if (state == State::EXPLORING) {
      float try_target_x = home_x + dirs[current_dir_idx][0] * 3.0f;
      float try_target_y = home_y + dirs[current_dir_idx][1] * 3.0f;
      
      error_x = try_target_x - current_x;
      error_y = try_target_y - current_y;

      // 判断是否到达3m无障碍
      if (std::abs(error_x) < 0.2f && std::abs(error_y) < 0.2f) {
         target_x = home_x + dirs[current_dir_idx][0] * 6.0f;
         target_y = home_y + dirs[current_dir_idx][1] * 6.0f;
         state = State::MOVING_TO_TARGET;
      } else {
         // 检查是否受阻(卡住)
         if (std::chrono::duration<double>(now - stuck_check_start).count() > 2.0) {
           float dist_moved = std::hypot(current_x - stuck_check_x, current_y - stuck_check_y);
           if (dist_moved < 0.2f) { // 2秒移动低于0.2m被视为卡住
             state = State::RETURN_HOME_FOR_NEXT_DIR;
           }
           stuck_check_start = now;
           stuck_check_x = current_x;
           stuck_check_y = current_y;
         }
      }
    } else if (state == State::RETURN_HOME_FOR_NEXT_DIR) {
      error_x = home_x - current_x;
      error_y = home_y - current_y;
      if (std::abs(error_x) < 0.2f && std::abs(error_y) < 0.2f) {
          state = State::WAITING_NEXT_DIR;
          wait_start_time = now;
      }
    } else if (state == State::WAITING_NEXT_DIR) {
      error_x = 0;
      error_y = 0;
      if (std::chrono::duration<double>(now - wait_start_time).count() > 2.0) {
          current_dir_idx = (current_dir_idx + 1) % 4; // 尝试下一个方向
          state = State::EXPLORING;
          stuck_check_start = now;
          stuck_check_x = current_x;
          stuck_check_y = current_y;
      }
    } else if (state == State::MOVING_TO_TARGET) {
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
}

} // namespace sentry_decision
