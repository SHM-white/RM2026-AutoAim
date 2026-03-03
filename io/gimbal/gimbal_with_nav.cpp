#include "gimbal_with_nav.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr int NAV_TCP_PORT = 16667;

bool recv_full(int fd, uint8_t * buffer, size_t size)
{
  size_t received = 0;
  while (received < size) {
    auto ret = recv(fd, buffer + received, size - received, 0);
    if (ret <= 0) return false;
    received += static_cast<size_t>(ret);
  }
  return true;
}
}  // namespace

namespace io
{
GimbalWithNav::GimbalWithNav(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[GimbalWithNav] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&GimbalWithNav::read_thread, this);
  nav_tcp_thread_ = std::thread(&GimbalWithNav::read_nav_tcp_thread, this);

  queue_.pop();
  tools::logger()->info("[GimbalWithNav] First q received.");
}

GimbalWithNav::~GimbalWithNav()
{
  quit_ = true;

  if (nav_client_fd_ >= 0) {
    shutdown(nav_client_fd_, SHUT_RDWR);
    close(nav_client_fd_);
    nav_client_fd_ = -1;
  }

  if (nav_server_fd_ >= 0) {
    shutdown(nav_server_fd_, SHUT_RDWR);
    close(nav_server_fd_);
    nav_server_fd_ = -1;
  }

  if (thread_.joinable()) thread_.join();
  if (nav_tcp_thread_.joinable()) nav_tcp_thread_.join();
  serial_.close();
}

GimbalMode GimbalWithNav::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState GimbalWithNav::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string GimbalWithNav::str(GimbalMode mode) const
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

Eigen::Quaterniond GimbalWithNav::q(std::chrono::steady_clock::time_point t)
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

void GimbalWithNav::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[GimbalWithNav] Failed to write serial: {}", e.what());
  }
}

void GimbalWithNav::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[GimbalWithNav] Failed to write serial: {}", e.what());
  }
}

std::tuple<NavData, std::chrono::steady_clock::time_point> GimbalWithNav::nav_data()
{
  return nav_queue_.pop();
}

bool GimbalWithNav::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[GimbalWithNav] Failed to read serial: {}", e.what());
    return false;
  }
}

void GimbalWithNav::read_thread()
{
  tools::logger()->info("[GimbalWithNav] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[GimbalWithNav] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != tx_data_.head[0] || rx_data_.head[1] != tx_data_.head[1]) continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[GimbalWithNav] CRC16 check failed.");
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
        tools::logger()->warn("[GimbalWithNav] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[GimbalWithNav] read_thread stopped.");
}

void GimbalWithNav::read_nav_tcp_thread()
{
  tools::logger()->info("[GimbalWithNav] nav tcp thread started on 0.0.0.0:{}", NAV_TCP_PORT);

  nav_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (nav_server_fd_ < 0) {
    tools::logger()->error("[GimbalWithNav] nav socket() failed: {}", std::strerror(errno));
    return;
  }

  int opt = 1;
  setsockopt(nav_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(NAV_TCP_PORT);

  if (bind(nav_server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    tools::logger()->error("[GimbalWithNav] nav bind() failed: {}", std::strerror(errno));
    close(nav_server_fd_);
    nav_server_fd_ = -1;
    return;
  }

  if (listen(nav_server_fd_, 1) < 0) {
    tools::logger()->error("[GimbalWithNav] nav listen() failed: {}", std::strerror(errno));
    close(nav_server_fd_);
    nav_server_fd_ = -1;
    return;
  }

  while (!quit_) {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int client_fd = accept(nav_server_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
    if (client_fd < 0) {
      if (quit_) break;
      tools::logger()->warn("[GimbalWithNav] nav accept() failed: {}", std::strerror(errno));
      continue;
    }

    nav_client_fd_ = client_fd;
    tools::logger()->info("[GimbalWithNav] nav client connected: {}", inet_ntoa(client_addr.sin_addr));

    while (!quit_) {
      NavData nav{};
      if (!recv_full(client_fd, reinterpret_cast<uint8_t *>(&nav), sizeof(NavData))) {
        tools::logger()->warn("[GimbalWithNav] nav client disconnected.");
        break;
      }
      nav_queue_.push({nav, std::chrono::steady_clock::now()});
    }

    close(client_fd);
    nav_client_fd_ = -1;
  }

  tools::logger()->info("[GimbalWithNav] nav tcp thread stopped.");
}

void GimbalWithNav::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[GimbalWithNav] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[GimbalWithNav] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[GimbalWithNav] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io