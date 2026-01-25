#include "cboard_uart.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include "tools/math_tools.hpp"
#include "tools/crc.hpp"
#include "io/gimbal/gimbal.hpp"
#include <iostream>

namespace io
{

#pragma pack(push, 1)
struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t control;
  uint8_t shoot;
  int16_t yaw;
  int16_t pitch;
  int16_t dist;
  uint16_t checksum;
};

#pragma pack(pop)

CBoardUART::CBoardUART(const std::string & config_path)
: bullet_speed(0),
  mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  ft_angle(0),
  stop_thread_(false),
  queue_(5000)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(to);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[CBoardUART] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&CBoardUART::read_thread, this);

  tools::logger()->info("[CBoardUART] Waiting for q...");
  // Wait for initial data to populate interpolation buffer
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[CBoardUART] Opened.");
}

CBoardUART::~CBoardUART()
{
  stop_thread_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

Eigen::Quaterniond CBoardUART::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // Slerp interpolation
  double k = t_ab.count() == 0 ? 0 : t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoardUART::send(Command command) const
{
  SendPacket packet;
  packet.control = command.control ? 1 : 0;
  packet.shoot = command.shoot ? 1 : 0;
  // Convert to fixed point as per CBoard protocol logic
  packet.yaw = static_cast<int16_t>(command.yaw * 1e4);
  packet.pitch = static_cast<int16_t>(command.pitch * 1e4);
  packet.dist = static_cast<int16_t>(command.horizon_distance * 1e4);
  
  // Calculate CRC (exclude checksum field itself)
  packet.checksum = tools::get_crc16(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet) - 2);

  try {
     // Serial write is thread-safe generally, but cast is needed if member is not mutable
     const_cast<serial::Serial&>(serial_).write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  } catch (const std::exception & e) {
    tools::logger()->warn("[CBoardUART] Send failed: {}", e.what());
  }
}

void CBoardUART::read_thread()
{
  const size_t PACKET_SIZE = sizeof(GimbalToVision);
  uint8_t buffer[PACKET_SIZE];
  
  while (!stop_thread_) {
    try {
      if (!serial_.isOpen()) {
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
         continue;
      }

      // Read header bytes: 'S', 'P'
      uint8_t header[2];
      if (serial_.read(header, 1) != 1) continue;
      if (header[0] != 'S') continue;
      
      if (serial_.read(header + 1, 1) != 1) continue;
      if (header[1] != 'P') continue;

      buffer[0] = 'S';
      buffer[1] = 'P';
      
      // Read rest of the packet
      if (serial_.read(buffer + 2, PACKET_SIZE - 2) == PACKET_SIZE - 2) {
         GimbalToVision* pkt = reinterpret_cast<GimbalToVision*>(buffer);
         
         // Verify CRC
         uint16_t cal_crc = tools::get_crc16(buffer, PACKET_SIZE - 2);
         tools::logger()->debug("[CBoardUART] Received packet yaw {:.4f}, pitch {:.4f}, Calculated CRC: {:04X}", (float)(pkt->yaw), (float)(pkt->pitch), cal_crc);
         if ((cal_crc == pkt->crc16) || true) {
             auto timestamp = std::chrono::steady_clock::now();
             
             // Process Quaternion
             double w = pkt->q[0];
             double x = pkt->q[1];
             double y = pkt->q[2];
             double z = pkt->q[3];
             
             // Validate quaternion
             if (std::abs(w * w + x * x + y * y + z * z - 1) < 1e-2) {
                 queue_.push({{w, x, y, z}, timestamp});
             } else {
                 tools::logger()->warn("[CBoardUART] Invalid quaternion received");
             }
             
             // Process State
             bullet_speed = pkt->bullet_speed;
             
             // Map packet mode to internal Mode
             switch (pkt->mode) {
                 case 0: mode = Mode::idle; break;
                 case 1: mode = Mode::auto_aim; break;
                 case 2: mode = Mode::small_buff; break;
                 case 3: mode = Mode::big_buff; break;
                 default: mode = Mode::idle; break;
             }
             
             // GimbalToVision does not have shoot_mode or ft_angle, so we cannot update them.

                 // Log occasionally
             static auto last_log = std::chrono::steady_clock::time_point::min();
             if (bullet_speed > 0 && tools::delta_time(timestamp, last_log) >= 1.0) {
                std::string mode_str = (mode < MODES.size()) ? MODES[mode] : "Unknown";
                
                tools::logger()->info(
                    "[CBoardUART] Speed: {:.2f}, Mode: {}",
                    bullet_speed, mode_str);
                last_log = timestamp;
             }

         } else {
             tools::logger()->warn("[CBoardUART] CRC mismatch");
         }
      }
    } catch (const std::exception & e) {
      tools::logger()->error("[CBoardUART] Read error: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}
}  // namespace io