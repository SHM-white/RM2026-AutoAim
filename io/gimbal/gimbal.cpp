#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#ifndef NDEBUG
#include "tools/plotter.hpp"
#endif

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
: Gimbal(config_path, nullptr, nullptr, nullptr)
{}

Gimbal::Gimbal(const std::string & config_path, AfterSendGimbalData after_send_gimbal_data, RefereeCallback referee_callback, NavRefereeCallback nav_referee_callback)
: serial_(tools::read<std::string>(tools::load(config_path), "com_port"), 115200, 80, "Gimbal"),
  after_send_gimbal_data_(std::move(after_send_gimbal_data)), referee_callback_(std::move(referee_callback)), nav_referee_callback_(std::move(nav_referee_callback))
{
  // Start even when the USB cable is absent; read_thread retries it.
  serial_.ensure_open();
  thread_ = std::thread(&Gimbal::read_thread, this);
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

bool Gimbal::feedback_fresh() const
{
  if (!serial_.connected()) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  return last_feedback_generation_ == serial_.generation() &&
         last_feedback_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - last_feedback_ < std::chrono::milliseconds(500);
}

GimbalMode Gimbal::mode() const
{
  if (!feedback_fresh()) return GimbalMode::IDLE;
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  if (!feedback_fresh()) return {};
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  // Never block control on a disconnected or silent serial port.
  if (!feedback_fresh()) return Eigen::Quaterniond::Identity();
  std::lock_guard<std::mutex> lock(q_mutex_);

  using Sample = std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>;
  Sample a{}, b{};
  if (!queue_.try_front(b)) return Eigen::Quaterniond::Identity();
  if (t <= std::get<1>(b)) return std::get<0>(b).normalized();

  while (queue_.try_pop_if_more_than_one(a)) {
    if (!queue_.try_front(b)) return std::get<0>(a).normalized();
    const auto ta = std::get<1>(a), tb = std::get<1>(b);
    if (t <= tb) {
      const double delta = std::chrono::duration<double>(tb - ta).count();
      if (delta <= 0.0) return std::get<0>(b).normalized();
      const double k = std::clamp(std::chrono::duration<double>(t - ta).count() / delta,
                                  0.0, 1.0);
      return std::get<0>(a).slerp(k, std::get<0>(b)).normalized();
    }
  }
  return std::get<0>(b).normalized();
}

void Gimbal::send(const io::Command & cmd) {
  send(cmd.control, cmd.shoot, cmd.yaw, 0, 0, cmd.pitch, 0, 0);
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_gimbal.mode = VisionToGimbal.mode;
  tx_data_gimbal.yaw = VisionToGimbal.yaw;
  tx_data_gimbal.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_gimbal.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_gimbal.pitch = VisionToGimbal.pitch;
  tx_data_gimbal.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_gimbal.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_gimbal.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_gimbal), sizeof(tx_data_gimbal) - sizeof(tx_data_gimbal.crc16));
  send_gimbal_data();
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_gimbal.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_gimbal.yaw = yaw;
  tx_data_gimbal.yaw_vel = yaw_vel;
  tx_data_gimbal.yaw_acc = yaw_acc;
  tx_data_gimbal.pitch = pitch;
  tx_data_gimbal.pitch_vel = pitch_vel;
  tx_data_gimbal.pitch_acc = pitch_acc;
  tx_data_gimbal.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_gimbal), sizeof(tx_data_gimbal) - sizeof(tx_data_gimbal.crc16));
  send_gimbal_data();
}

void Gimbal::send_imu_forward(const DM_IMU & imu) {
  if (!feedback_fresh() || !imu.connected()) return;
  IMU_Forward_Frame frame{};
  frame.data = imu.current_data();
  frame.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t *>(&frame),
                                sizeof(frame) - sizeof(frame.crc16));
  serial_.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    if (!serial_.connected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return false;
    }
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Unexpected read failure: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  uint64_t active_generation = 0;

#ifndef NDEBUG
  auto plotter = tools::Plotter{};
#endif

  while (!quit_) {
    if (!serial_.ensure_open()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = GimbalMode::IDLE;
        state_ = {};
        last_feedback_generation_ = 0;
      }
      queue_.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    if (active_generation != serial_.generation()) {
      active_generation = serial_.generation();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = GimbalMode::IDLE;
        state_ = {};
        last_feedback_generation_ = 0;
      }
      queue_.clear();
    }

    uint8_t head_byte[2];
    if (!read(head_byte, 1)) continue;

#ifndef NDEBUG
    tools::logger()->debug("[Gimbal] Read first byte: 0x{:02X}", head_byte[0]);
    // if (head_byte[0] == gimbal_struct_header[0]) {
    //   tools::logger()->debug("[Gimbal] Detected potential GimbalToVision header byte.");
    // } else if (head_byte[0] == nav_struct_header[0]) {
    //   tools::logger()->debug("[Gimbal] Detected potential NavData header byte.");
    // } else if (head_byte[0] == 0xA5) {
    //   tools::logger()->debug("[Gimbal] Detected referee system packet header.");
    // }
#endif

    if (head_byte[0] == gimbal_struct_header[0]) {
      // ---- GimbalToVision packet ('A', 'B') ----
      if (!read(head_byte + 1, 1)) continue;
      if (head_byte[1] != gimbal_struct_header[1]) continue;

      auto t = std::chrono::steady_clock::now();

      // Use a local buffer to avoid member-variable aliasing across iterations
      GimbalToVision rx_pkt{};
      rx_pkt.head[0] = head_byte[0];
      rx_pkt.head[1] = head_byte[1];

      if (!read(
            reinterpret_cast<uint8_t *>(&rx_pkt) + sizeof(rx_pkt.head),
            sizeof(rx_pkt) - sizeof(rx_pkt.head))) {
        continue;
      }
      if (active_generation != serial_.generation()) continue;
      auto crc16_passed = tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_pkt), sizeof(rx_pkt));
      tools::logger()->debug(
        "[Gimbal] Received GimbalToVision packet: yaw={:.2f}, pitch={:.2f}, crc16=0x{:04X}, received_crc16=0x{:04X}, crc16_passed={}",
        rx_pkt.yaw, rx_pkt.pitch, tools::get_crc16(reinterpret_cast<uint8_t *>(&rx_pkt), sizeof(rx_pkt) - sizeof(rx_pkt.crc16)), rx_pkt.crc16, crc16_passed);

      if (!crc16_passed) {
        tools::logger()->debug("[Gimbal] CRC16 check failed for GimbalToVision packet.");
        continue;
      }

      // Eigen::Quaterniond q(rx_pkt.q[0], rx_pkt.q[1], rx_pkt.q[2], rx_pkt.q[3]);
      // queue_.push({q, t});
             // Process Quaternion from yaw and pitch (roll = 0)
      double pitch = rx_pkt.pitch;
      double yaw = rx_pkt.yaw;
      
      double cy = std::cos(yaw * 0.5);
      double sy = std::sin(yaw * 0.5);
      double cp = std::cos(pitch * 0.5);
      double sp = std::sin(pitch * 0.5);
      
      double w = cp * cy;
      double x = -sp * sy;
      double y = sp * cy;
      double z = cp * sy;
      
      // Validate quaternion
      if (std::abs(w * w + x * x + y * y + z * z - 1) < 1e-2) {
          queue_.push({{w, x, y, z}, t});
      } else {
          tools::logger()->warn("[CBoardUART] Invalid quaternion received");
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_feedback_ = std::chrono::steady_clock::now();
        last_feedback_generation_ = active_generation;
        state_.yaw = rx_pkt.yaw;
        state_.yaw_vel = rx_pkt.yaw_vel;
        state_.pitch = rx_pkt.pitch;
        state_.pitch_vel = rx_pkt.pitch_vel;
        state_.bullet_speed = rx_pkt.bullet_speed;
        state_.bullet_count = rx_pkt.bullet_count;

        switch (rx_pkt.mode) {
          case 0:
            mode_ = GimbalMode::IDLE;
            break;
          case 1:
            mode_ = GimbalMode::AUTO_AIM;
            break;
          case 2:
            mode_ = GimbalMode::SMALL_BUFF;
            break;
          case 3:
            mode_ = GimbalMode::BIG_BUFF;
            break;
          default:
            mode_ = GimbalMode::IDLE;
            tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_pkt.mode);
            break;
        }
#ifndef NDEBUG
        nlohmann::json data;
        data["mode"] = str(mode_);
        data["yaw"] = state_.yaw * 57.3;
        data["yaw_vel"] = state_.yaw_vel * 57.3;
        data["pitch"] = state_.pitch * 57.3;
        data["pitch_vel"] = state_.pitch_vel * 57.3;
        data["bullet_speed"] = state_.bullet_speed;
        data["bullet_count"] = state_.bullet_count;
        plotter.plot(data);
#endif
      }
    } 
    else if (head_byte[0] == 0xA5) {
      // ---- Referee system packet (0xA5 header) ----
      uint8_t header[4];  // length(2), seq(1), crc8(1)
      if (!read(header, 4)) {
        continue;
      }

      uint16_t data_len = *reinterpret_cast<uint16_t *>(header);
      if (data_len > 256) {
        continue;  // 长度超限
      }

      // 校验帧头 CRC8
      uint8_t crc_header[5] = {0xA5, header[0], header[1], header[2], header[3]};
      if (!tools::check_crc8(crc_header, 5)) continue;

      uint8_t cmd_buff[2];
      if (!read(cmd_buff, 2)) {
        continue;
      }
      uint16_t cmd_id = *reinterpret_cast<uint16_t *>(cmd_buff);

      std::vector<uint8_t> data_buff(data_len);
      if (data_len > 0) {
        if (!read(data_buff.data(), data_len)) {
            continue;
        }
      }

      uint8_t tail[2];  // CRC16
      if (!read(tail, 2)) {
        continue;
      }

      // 验证全包 CRC16
      std::vector<uint8_t> full_packet(1 + 4 + 2 + data_len);
      full_packet[0] = 0xA5;
      std::copy(header, header + 4, full_packet.begin() + 1);
      std::copy(cmd_buff, cmd_buff + 2, full_packet.begin() + 5);
      if (data_len > 0) {
        std::copy(data_buff.data(), data_buff.data() + data_len, full_packet.begin() + 7);
      }
      uint16_t crc_calculated = tools::get_crc16(full_packet.data(), full_packet.size());
      uint16_t crc_received = *reinterpret_cast<uint16_t *>(tail);
      if (crc_calculated != crc_received) {
        tools::logger()->debug("[Gimbal] Full packet CRC16 check failed for referee packet.");
        continue;
      }
      parse_referee_data(cmd_id, data_buff.data(), data_len);
    }
  }
  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::parse_referee_data(uint16_t cmd_id, const uint8_t* data, uint16_t len)
{
  switch (cmd_id) {
    case 0x0201: // 机器人性能体系状态
    case 0x0202: // 实时功率热量数据
      if (referee_callback_) {
        referee_callback_(cmd_id, data, len);
      }
      // break;
    // case 0x0203: // 机器人位置数据
    // case 0x0204: // 机器人增益数据
    // case 0x0206: // 伤害状态数据
    // case 0x0101: // 场地事件数据
    // case 0x0102: // 补给站动作标识数据
    default: // 默认转发给导航决策器
      if (nav_referee_callback_) {
        std::vector<uint8_t> data_vec(data, data + len);
        nav_referee_callback_(cmd_id, data_vec);
      }
      break;
  }
}

void Gimbal::send_gimbal_data() {
  // If telemetry is stale, send an explicit NO_CONTROL/NO_FIRE packet
  // whenever the UART is still alive. Never replay a pre-disconnect aim.
  if (!feedback_fresh()) {
    tx_data_gimbal.mode = 0;
    tx_data_gimbal.yaw = tx_data_gimbal.yaw_vel = tx_data_gimbal.yaw_acc = 0.0f;
    tx_data_gimbal.pitch = tx_data_gimbal.pitch_vel = tx_data_gimbal.pitch_acc = 0.0f;
    tx_data_gimbal.crc16 = tools::get_crc16(
      reinterpret_cast<const uint8_t *>(&tx_data_gimbal),
      sizeof(tx_data_gimbal) - sizeof(tx_data_gimbal.crc16));
  }
  if (!serial_.write(reinterpret_cast<const uint8_t *>(&tx_data_gimbal),
                     sizeof(tx_data_gimbal))) return;
  if (after_send_gimbal_data_) {
    after_send_gimbal_data_();
  }
}

} // namespace io
