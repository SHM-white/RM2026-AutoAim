#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <optional>
#include <vector>

#include "io/reconnecting_serial.hpp"
#include "tools/thread_safe_queue.hpp"
#include "io/command.hpp"
#include "io/vision_protocol.hpp"
#include "io/dm_imu/dm_imu.hpp"


namespace io
{
enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  // 裁判系统回调函数类型：参数依次为 cmd_id, 数据指针, 数据长度
  using RefereeCallback = std::function<void(uint16_t cmd_id, const uint8_t* data, uint16_t len)>;
  using NavRefereeCallback = std::function<void(uint16_t cmd_id, const std::vector<uint8_t>& data)>;
  using AfterSendGimbalData = std::function<void(void)>;

  Gimbal(const std::string & config_path);
  Gimbal(const std::string & config_path, AfterSendGimbalData after_send_gimbal_data, RefereeCallback referee_callback, NavRefereeCallback nav_referee_callback);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(const io::Command & cmd);
  void send(io::VisionToGimbal VisionToGimbal);
  void send_imu_forward(const DM_IMU & imu);

private:
  ReconnectingSerial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  VisionToGimbal tx_data_gimbal;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  std::chrono::steady_clock::time_point last_feedback_{};
  uint64_t last_feedback_generation_ = 0;
  bool feedback_fresh() const;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void parse_referee_data(uint16_t cmd_id, const uint8_t* data, uint16_t len);
  void send_gimbal_data();

  RefereeCallback referee_callback_;
  NavRefereeCallback nav_referee_callback_;
  AfterSendGimbalData after_send_gimbal_data_;
};

}  // namespace io

#endif  // IO__GIMBAL_HPP