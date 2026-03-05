#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
: Gimbal(config_path, [](uint16_t, const uint8_t*, uint16_t) {})
{}

Gimbal::Gimbal(const std::string & config_path, RefereeCallback referee_callback)
: referee_callback_(std::move(referee_callback))
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
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
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
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

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_gimbal), sizeof(tx_data_gimbal));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
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

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_gimbal), sizeof(tx_data_gimbal));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send_cmd_vel(const std::optional<const NavData> & nav_data)
{
  if (!nav_data.has_value()) return;

  tx_data_nav.linear_x = nav_data->angular_x;
  tx_data_nav.linear_y = nav_data->linear_y;
  tx_data_nav.linear_z = nav_data->linear_z;
  tx_data_nav.angular_x = nav_data->angular_x;
  tx_data_nav.angular_y = nav_data->angular_y;
  tx_data_nav.angular_z = nav_data->angular_z;
  tx_data_nav.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_nav), sizeof(tx_data_nav) - sizeof(tx_data_nav.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_nav), sizeof(tx_data_nav));
  } catch (const std::exception & e) {
    tools::logger()->warn("[GimbalWithNav] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  size_t read_bytes = 0;
  int timeout_count = 0;
  while (read_bytes < size && !quit_) {
    try {
      size_t n = serial_.read(buffer + read_bytes, size - read_bytes);
      if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        timeout_count++;
        // 50ms没收到后续数据视为本包超时，避免死锁
        if (timeout_count > 50) return false;
      } else {
        read_bytes += n;
        timeout_count = 0;
      }
    } catch (const std::exception & e) {
      return false;
    }
  }
  return read_bytes == size && !quit_;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    uint8_t first_byte;
    if (!read(&first_byte, 1)) {
      // 超时没读到首字节说明单纯没数据或链路空闲，不视为出错断解
      continue;
    }

    if (first_byte == gimbal_struct_header[0]) {
      uint8_t second_byte;
      if (!read(&second_byte, 1)) {
        error_count++;
        continue;
      }
      if (second_byte != gimbal_struct_header[1]) {
        continue;
      }

      rx_data_.head[0] = first_byte;
      rx_data_.head[1] = second_byte;

      auto t = std::chrono::steady_clock::now();

      if (!read(
            reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
            sizeof(rx_data_) - sizeof(rx_data_.head))) {
        error_count++;
        continue;
      }

      if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
        tools::logger()->debug("[Gimbal] CRC16 check failed.");
        continue;
      }

      error_count = 0;
      Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
      queue_.push({q, t});

      std::lock_guard<std::mutex> lock(mutex_);

      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;

      switch (rx_data_.mode) {
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
          tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
          break;
      }
    } else if (first_byte == 0xA5) {
      // 比赛裁判系统协议解析
      uint8_t header[4]; // length(2), seq(1), crc8(1)
      if (!read(header, 4)) {
        error_count++;
        continue;
      }
      
      uint16_t data_len = *reinterpret_cast<uint16_t*>(header);
      if (data_len > 256) {
        continue; // 长度超限
      }

      // 可选：校验crc8
      uint8_t crc_header[5] = {0xA5, header[0], header[1], header[2], header[3]};
      if (!tools::check_crc8(crc_header, 5)) continue;

      uint8_t cmd_buff[2];
      if (!read(cmd_buff, 2)) {
        error_count++;
        continue;
      }
      uint16_t cmd_id = *reinterpret_cast<uint16_t*>(cmd_buff);

      std::vector<uint8_t> data_buff(data_len);
      if (data_len > 0) {
        if (!read(data_buff.data(), data_len)) {
          error_count++;
          continue;
        }
      }

      uint8_t tail[2]; // crc16
      if (!read(tail, 2)) {
        error_count++;
        continue;
      }

      // 可选：验证全包crc16
      std::vector<uint8_t> full_packet(1 + 4 + 2 + data_len); // header + cmd + data
      full_packet[0] = 0xA5;
      std::copy(header, header + 4, full_packet.begin() + 1);
      std::copy(cmd_buff, cmd_buff + 2, full_packet.begin() + 5);
      if (data_len > 0) {
        std::copy(data_buff.data(), data_buff.data() + data_len, full_packet.begin() + 7);
      }
      uint16_t crc_calculated = tools::get_crc16(full_packet.data(), full_packet.size());
      uint16_t crc_received = *reinterpret_cast<uint16_t*>(tail);
      if (crc_calculated != crc_received) {
        tools::logger()->debug("[Gimbal] Full packet CRC16 check failed.");
        continue;
      }
      error_count = 0;
      // 分类并回调裁判系统系统接口
      parse_referee_data(cmd_id, data_buff.data(), data_len);
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}


void Gimbal::parse_referee_data(uint16_t cmd_id, const uint8_t* data, uint16_t len)
{
  switch (cmd_id) {
    case 0x0101: // 场地事件数据
    case 0x0102: // 补给站动作标识数据
    case 0x0201: // 机器人性能体系状态
    case 0x0202: // 实时功率热量数据
    case 0x0203: // 机器人位置数据
    case 0x0204: // 机器人增益数据
    case 0x0206: // 伤害状态数据
      if (nav_referee_callback_) {
        std::vector<uint8_t> data_vec(data, data + len);
        nav_referee_callback_(cmd_id, data_vec);
      }
      break;
    default:
      if (referee_callback_) {
        referee_callback_(cmd_id, data, len);
      }
      break;
  }
}

} // namespace io
