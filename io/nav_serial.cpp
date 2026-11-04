#include "nav_serial.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
std::string nav_port_for(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  if (!yaml["nav_com_port"] || yaml["nav_com_port"].as<std::string>().empty())
    throw std::invalid_argument("Missing nav_com_port: navigation requires a separate serial port");
  auto nav_port = yaml["nav_com_port"].as<std::string>();
  auto gimbal_port = yaml["com_port"] ? yaml["com_port"].as<std::string>() : "";
  if (nav_port == gimbal_port)
    throw std::invalid_argument("nav_com_port must differ from com_port");
  return nav_port;
}
}  // namespace

NavSerial::NavSerial(const std::string & config_path)
: serial_(nav_port_for(config_path), 115200, 80, "NavSerial")
{
  // Missing hardware is non-fatal: send() will retry the stable path.
  serial_.ensure_open();
}

bool NavSerial::send(const NavData & command)
{
  const auto packet = seal_nav_frame(command);
  std::lock_guard<std::mutex> lock(tx_mutex_);
  try {
    return serial_.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  } catch (const std::exception & e) {
    tools::logger()->warn("[NavSerial] Unexpected write failure: {}", e.what());
    return false;
  }
}
}  // namespace io
