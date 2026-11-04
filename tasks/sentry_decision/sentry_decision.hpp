#pragma once

#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>
#include <cstring>
#include <atomic>

#include "io/cboard.hpp"
#include "io/ros2/ros2.hpp"
#include "io/dm_imu/dm_imu.hpp"

namespace sentry_decision {

    
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

struct game_status_t {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
};
#pragma pack(pop)

class SentryDecider {
public:
  SentryDecider();
  ~SentryDecider();

  void referee_callback(uint16_t cmd_id, const uint8_t* data, uint16_t len);

  void start_nav_thread(io::DM_IMU* imu);
  void stop_nav_thread();

  io::NavData get_nav_data();
  uint8_t get_game_progress();

private:
  void nav_loop();

  std::mutex referee_mutex;
  robot_status_t robot_status = {0};
  robot_pos_t robot_pos = {1.0f, 1.0f, 0.0f}; // 初始位置假设为(1, 1)
  game_robot_HP_t game_robot_hp = {0};
  game_status_t game_status = {0};

  std::mutex nav_mutex;
  io::NavData current_nav_data;

  std::thread nav_thread;
  std::atomic<bool> is_running{false};
  
  io::DM_IMU* imu_{nullptr};
};

} // namespace sentry_decision
