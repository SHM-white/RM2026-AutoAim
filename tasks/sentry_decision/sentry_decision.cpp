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
    STAYING_AT_POSITION,
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

  const float out_speed_mps = 1.0f;
  const float home_speed_mps = -1.0f;
  const float move_duration_s = 7.0f;
  const float stay_duration_s = 10.0f;
  const float wait_duration_s = 10.0f;

  State resume_state = State::MOVING_OUT;

  auto state_start_time = std::chrono::steady_clock::now();
  auto wait_start_time = std::chrono::steady_clock::now();

  while (is_running) {
    auto now = std::chrono::steady_clock::now();

    io::NavData data = {0};

    uint16_t current_hp = 0;
    uint16_t max_hp = 600;
    {
      std::lock_guard<std::mutex> lock(referee_mutex);
      // 保留裁判系统血量逻辑：优先使用 0x0201，无数据时回退到 0x0003。
      current_hp = robot_status.current_HP > 0 ? robot_status.current_HP : game_robot_hp.ally_7_robot_HP;
      max_hp = robot_status.maximum_HP > 0 ? robot_status.maximum_HP : 600;
    }

    // 低血量时按原逻辑返航，回到原点后等待恢复再继续。
    bool hp_available = current_hp > 0 && max_hp > 0;
    bool low_hp = hp_available && current_hp < static_cast<uint16_t>(max_hp * 0.3f);
    bool hp_recovered = hp_available && current_hp >= static_cast<uint16_t>(max_hp * 0.9f);
    bool in_mission_stage =
      state == State::MOVING_OUT ||
      state == State::STAYING_AT_POSITION ||
      state == State::RETURNING_HOME ||
      state == State::WAITING_NEXT_DIR;

    if (in_mission_stage && low_hp) {
      resume_state = state;
      state = State::RETREAT_TO_HOME;
      state_start_time = now;
    }

    if (state == State::MOVING_OUT) {
      data.linear_x = out_speed_mps;
      data.linear_y = 0;

      if (std::chrono::duration<double>(now - state_start_time).count() >= move_duration_s) {
        state = State::STAYING_AT_POSITION;
        state_start_time = now;
      }
    } else if (state == State::RETURNING_HOME) {
      data.linear_x = home_speed_mps;
      data.linear_y = 0;

      if (std::chrono::duration<double>(now - state_start_time).count() >= move_duration_s) {
        state = State::WAITING_NEXT_DIR;
        wait_start_time = now;
      }
    } else if (state == State::WAITING_NEXT_DIR) {
      data.linear_x = 0;
      data.linear_y = 0;

      if (std::chrono::duration<double>(now - wait_start_time).count() >= wait_duration_s) {
        state = State::MOVING_OUT;
        state_start_time = now;
      }
    } else if (state == State::RETREAT_TO_HOME) {
      // 返航状态统一发送负向速度。
      data.linear_x = home_speed_mps;
      data.linear_y = 0;

      if (std::chrono::duration<double>(now - state_start_time).count() >= move_duration_s) {
        state = State::RECOVERING;
      }
    } else if (state == State::RECOVERING) {
      data.linear_x = 0;
      data.linear_y = 0;

      // 离线模式没有血量时，不会进此状态；在线模式恢复到90%继续执行原任务。
      if (!hp_available || hp_recovered) {
        state = resume_state;
      }
    } else if (state == State::STAYING_AT_POSITION) {
      data.linear_x = 0;
      data.linear_y = 0;

      if (std::chrono::duration<double>(now - state_start_time).count() >= stay_duration_s) {
        state = State::RETURNING_HOME;
        state_start_time = now;
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
