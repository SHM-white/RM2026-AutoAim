#ifndef IO__GIMBAL_WITH_NAV_HPP
#define IO__GIMBAL_WITH_NAV_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <optional>

#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
constexpr uint8_t nav_struct_header[2] = {'C', 'D'};

#pragma pack(push, 1)
struct NavData
{
  uint8_t head[2] = {nav_struct_header[0], nav_struct_header[1]};
  double linear_x;
  double linear_y;
  double linear_z;
  double angular_x;
  double angular_y;
  double angular_z;
  uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(NavData) <= 64);

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

  void send(const io::VisionToGimbal & VisionToGimbal);

  void send_cmd_vel(const std::optional<const NavData> & nav_data);

  std::tuple<NavData, std::chrono::steady_clock::time_point> nav_data();

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_gimbal;
  NavData tx_data_nav;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_WITH_NAV_HPP