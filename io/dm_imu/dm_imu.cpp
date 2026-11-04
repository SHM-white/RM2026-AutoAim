#include "dm_imu.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io
{
namespace
{
std::string imu_port(const std::string & explicit_port)
{
  if (!explicit_port.empty()) return explicit_port;
  const char * env_port = std::getenv("SKYWALKER_IMU_PORT");
  return (env_port && *env_port) ? std::string(env_port) : "/dev/ttyACM0";
}
}  // namespace

DM_IMU::DM_IMU(QTransform q_transform, const std::string & port)
: serial_(imu_port(port), 921600, 30, "DM_IMU"),
  queue_(5000), q_transform_(std::move(q_transform))
{
  // Do not block construction on missing hardware.
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  tools::logger()->info("[DM_IMU] background reconnect enabled");
}

DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) rec_thread_.join();
  // Never issue a reboot command during unplug/normal shutdown.
  serial_.close();
}

bool DM_IMU::connected() const
{
  if (!serial_.connected()) return false;
  const auto generation = serial_.generation();
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return last_packet_generation_ == generation &&
         last_valid_packet_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - last_valid_packet_ <
           std::chrono::milliseconds(500);
}

bool DM_IMU::init_serial()
{
  if (!serial_.ensure_open()) return false;
  const auto generation = serial_.generation();
  if (generation == configured_generation_) return true;

  // Replay setup only on a new connection. Zeroing during a live reconnect
  // would abruptly shift the robot's reference frame, so only do it at boot.
  if (!was_initialized_ && !write_command(IMU_COMMAND::ZERO_ANGLE, std::nullopt)) return false;
  if (!write_command(IMU_COMMAND::ENTER_SET_MODE, std::nullopt)) return false;
  if (!write_command(IMU_COMMAND::ENABLE_TEMP_CONTROL, std::nullopt)) return false;
  if (!write_command(IMU_COMMAND::SET_TEMP, 25)) return false;
  if (!write_command(IMU_COMMAND::ENTER_NORMAL_MODE, std::nullopt)) return false;

  was_initialized_ = true;
  configured_generation_ = generation;
  queue_.clear();
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_packet_generation_ = 0;
  }
  tools::logger()->info("[DM_IMU] initialized UART generation {}", generation);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  return true;
}

void DM_IMU::get_imu_data_thread()
{
  uint8_t header[4];
  while (!stop_thread_) {
    if (!init_serial()) {
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_packet_generation_ = 0;
      }
      queue_.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    try {
      // 逐字节读取寻找帧头 0x55
      if (serial_.read(header, 1) != 1) continue;
      if (header[0] != 0x55) continue;

      // 读取后续3字节校验头部
      if (serial_.read(header + 1, 3) != 3) continue;

      if (header[1] != 0xAA /*|| header[2] != 0x9B*/ || header[3] != 0x01) {
        continue;
      }

      // 头部校验通过，填充结构体
      receive_data.FrameHeader1 = header[0];
      receive_data.flag1 = header[1];
      receive_data.slave_id1 = header[2];
      receive_data.reg_acc = header[3];

      // 读取剩余数据 (57 - 4 = 53 bytes)
      if (serial_.read((uint8_t *)(&receive_data.accx_u32), 53) != 53) continue;

      uint8_t current_idx = data_idx_.load(std::memory_order_relaxed);
      IMU_Data temp_data = data_buffer_[current_idx];

      if (tools::get_imu_crc16((uint8_t *)(&receive_data.FrameHeader1), 16) == receive_data.crc1) {
        temp_data.accx = *((float *)(&receive_data.accx_u32));
        temp_data.accy = *((float *)(&receive_data.accy_u32));
        temp_data.accz = *((float *)(&receive_data.accz_u32));
      }
      if (tools::get_imu_crc16((uint8_t *)(&receive_data.FrameHeader2), 16) == receive_data.crc2) {
        temp_data.gyrox = *((float *)(&receive_data.gyrox_u32));
        temp_data.gyroy = *((float *)(&receive_data.gyroy_u32));
        temp_data.gyroz = *((float *)(&receive_data.gyroz_u32));
      }
      const bool euler_valid =
        tools::get_imu_crc16((uint8_t *)(&receive_data.FrameHeader3), 16) == receive_data.crc3;
      if (euler_valid) {
        temp_data.roll = *((float *)(&receive_data.roll_u32));
        temp_data.pitch = *((float *)(&receive_data.pitch_u32));
        temp_data.yaw = *((float *)(&receive_data.yaw_u32));
      }
      
      if (!euler_valid || !std::isfinite(temp_data.yaw) ||
          !std::isfinite(temp_data.pitch) || !std::isfinite(temp_data.roll) ||
          !serial_.connected() || serial_.generation() != configured_generation_)
        continue;

      uint8_t next_idx = 1 - current_idx;
      data_buffer_[next_idx] = temp_data;
      data_idx_.store(next_idx, std::memory_order_release);

      auto timestamp = std::chrono::steady_clock::now();
      Eigen::Quaterniond q = Eigen::AngleAxisd(temp_data.yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(temp_data.pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(temp_data.roll * M_PI / 180, Eigen::Vector3d::UnitX());
      q.normalize();
      q = q_transform_(q);
      q.normalize();
      queue_.push({q, timestamp});
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_valid_packet_ = timestamp;
        last_packet_generation_ = configured_generation_;
      }
    } catch (const std::exception & e) { 
      tools::logger()->error("[DM_IMU] error: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (!connected()) return Eigen::Quaterniond::Identity();
  std::lock_guard<std::mutex> lock(interpolation_mutex_);
  IMUData a{}, b{};
  if (!queue_.try_front(b)) return Eigen::Quaterniond::Identity();
  if (timestamp <= b.timestamp) return b.q.normalized();

  while (queue_.try_pop_if_more_than_one(a)) {
    if (!queue_.try_front(b)) return a.q.normalized();
    if (timestamp <= b.timestamp) {
      const double duration = std::chrono::duration<double>(b.timestamp - a.timestamp).count();
      if (duration <= 0.0) return b.q.normalized();
      const double k = std::clamp(
        std::chrono::duration<double>(timestamp - a.timestamp).count() / duration, 0.0, 1.0);
      return a.q.slerp(k, b.q).normalized();
    }
  }
  return b.q.normalized();
}

IMU_Data DM_IMU::current_data() const {
  uint8_t read_idx = data_idx_.load(std::memory_order_acquire);
  return data_buffer_[read_idx]; 
}

bool DM_IMU::write_command(IMU_COMMAND command, std::optional<uint8_t> param)
{
  uint8_t bytes[4] = {0xAA, 0x00, 0x00, 0x0D};
  const uint16_t value = static_cast<uint16_t>(command);
  bytes[1] = (value >> 8) & 0xFF;
  bytes[2] = (value & 0xFF) | param.value_or(0);
  return serial_.write(bytes, sizeof(bytes));
}

void DM_IMU::send_command(IMU_COMMAND command, std::optional<uint8_t> param)
{
  write_command(command, param);
}

void DM_IMU::forward_data(const serial::Serial & serial) const {
  if (!connected()) return;
  IMU_Forward_Frame forward_frame;
  // 直接从当前最新的索引中读取数据，无需加锁，不阻塞写入线程
  uint8_t read_idx = data_idx_.load(std::memory_order_acquire);
  forward_frame.data = data_buffer_[read_idx]; 
  
  forward_frame.crc16 = tools::get_crc16((uint8_t *)(&forward_frame.header), sizeof(IMU_Forward_Frame) - sizeof(uint16_t));
  const_cast<serial::Serial &>(serial).write(reinterpret_cast<const uint8_t *>(&forward_frame), sizeof(IMU_Forward_Frame));
}

}  // namespace io
