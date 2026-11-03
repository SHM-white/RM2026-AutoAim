#include "nav_serial.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
NavSerial::NavSerial(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  if (!yaml["nav_com_port"] || yaml["nav_com_port"].as<std::string>().empty())
    throw std::invalid_argument("Missing nav_com_port: navigation requires a separate serial port");

  const auto nav_port = yaml["nav_com_port"].as<std::string>();
  const auto gimbal_port = yaml["com_port"] ? yaml["com_port"].as<std::string>() : "";
  if (nav_port == gimbal_port)
    throw std::invalid_argument("nav_com_port must differ from com_port");

  serial_.setPort(nav_port);
  serial_.setBaudrate(115200);
  serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
  serial_.open();
  tools::logger()->info("[NavSerial] Opened {}", nav_port);
}

bool NavSerial::send(const NavData & command)
{
  const auto packet = seal_nav_frame(command);
  std::lock_guard<std::mutex> lock(tx_mutex_);
  try {
    return serial_.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) ==
           sizeof(packet);
  } catch (const std::exception & e) {
    tools::logger()->warn("[NavSerial] Write failed: {}", e.what());
    return false;
  }
}
}  // namespace io
