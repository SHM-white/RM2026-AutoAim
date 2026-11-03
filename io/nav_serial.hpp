#pragma once

#include <mutex>
#include <string>

#include "io/vision_protocol.hpp"
#include "io/reconnecting_serial.hpp"

namespace io
{
// Navigation's dedicated serial port. No shared serial state or writes with
// auto-aim Gimbal. nav_com_port is intentionally distinct from com_port.
class NavSerial
{
public:
  explicit NavSerial(const std::string & config_path);
  bool send(const NavData & command);

private:
  ReconnectingSerial serial_;
  std::mutex tx_mutex_;
};
}  // namespace io
