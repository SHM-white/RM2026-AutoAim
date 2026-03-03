#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
#pragma pack(push, 1)
struct GimbalToVision
{
  uint8_t head[2] = {'A', 'B'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(GimbalToVision) <= 64);

#pragma pack(push, 1)
struct VisionToGimbal
{
  const int8_t head[2] = {'A', 'B'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(VisionToGimbal) <= 64);

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

#pragma pack(push, 1)
struct NavData{
  double linear_x;
  double linear_y;
  double linear_z;
  double angular_x;
  double angular_y;
  double angular_z;
};
#pragma pack(pop)

class GimbalWithNav
{
public:
  GimbalWithNav(const std::string & config_path);

  ~GimbalWithNav();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

  std::tuple<NavData, std::chrono::steady_clock::time_point> nav_data();

private:
  serial::Serial serial_;

  std::thread thread_;
  std::thread nav_tcp_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  int nav_server_fd_ = -1;
  int nav_client_fd_ = -1;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  tools::ThreadSafeQueue<std::tuple<NavData, std::chrono::steady_clock::time_point>>
    nav_queue_{1000};
  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void read_nav_tcp_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP