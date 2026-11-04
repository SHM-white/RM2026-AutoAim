#pragma once

// A small reconnecting serial transport for Linux character devices.
// Keep a stable udev symlink (/dev/serial/by-id, /dev/serial/by-path or a
// custom /dev/skywalker/* rule) in configuration. Dynamic /dev/ttyUSBn
// names cannot identify a physical device across reconnects.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <sys/stat.h>
#include <utility>

#include "serial/serial.h"

namespace io
{
class ReconnectingSerial
{
public:
  ReconnectingSerial(std::string port, uint32_t baudrate, uint32_t timeout_ms,
                     std::string label = "serial")
  : port_(std::move(port)), label_(std::move(label))
  {
    if (port_.rfind("/dev/ttyUSB", 0) == 0 ||
        port_.rfind("/dev/ttyACM", 0) == 0) {
      std::fprintf(stderr,
                   "[%s] WARNING: %s is a dynamic Linux tty name. "
                   "Use /dev/serial/by-id, /dev/serial/by-path or a udev alias.\n",
                   label_.c_str(), port_.c_str());
    }
    serial_.setPort(port_);
    serial_.setBaudrate(baudrate);
    auto timeout = serial::Timeout::simpleTimeout(timeout_ms);
    serial_.setTimeout(timeout);
  }

  ~ReconnectingSerial() { close(); }
  ReconnectingSerial(const ReconnectingSerial &) = delete;
  ReconnectingSerial & operator=(const ReconnectingSerial &) = delete;

  // Automatically opens when the configured identity becomes available.
  // No blind scan of ttyUSB*, since that could attach to the wrong MCU.
  bool ensure_open()
  {
    {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      if (live_locked()) return true;
    }

    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (live_locked()) return true;
    if (serial_.isOpen()) {
      std::fprintf(stderr, "[%s] Device disconnected/replaced: %s\n",
                   label_.c_str(), port_.c_str());
      close_locked();
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < next_retry_) return false;
    next_retry_ = now + std::chrono::milliseconds(250);

    DeviceIdentity current{};
    if (!identity_of(port_, current)) return false;
    try {
      serial_.open();
      identity_ = current;
      ++generation_;
      std::fprintf(stderr, "[%s] Opened: %s (generation %llu)\n",
                   label_.c_str(), port_.c_str(),
                   static_cast<unsigned long long>(generation_));
      return true;
    } catch (const std::exception & e) {
      // A failed open must never terminate the whole vision process.
      // Rate-limit diagnostics while a cable/driver is temporarily absent.
      if (now - last_error_log_ >= std::chrono::seconds(5)) {
        std::fprintf(stderr, "[%s] Open failed for %s: %s\n",
                     label_.c_str(), port_.c_str(), e.what());
        last_error_log_ = now;
      }
      return false;
    }
  }

  bool connected() const
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return live_locked();
  }

  uint64_t generation() const
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return generation_;
  }

  size_t read(uint8_t * data, size_t size)
  {
    if (!ensure_open()) return 0;
    try {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      if (!live_locked()) return 0;
      return serial_.read(data, size);
    } catch (const std::exception & e) {
      fault(e.what());
      return 0;
    }
  }

  bool write(const uint8_t * data, size_t size)
  {
    if (!ensure_open()) return false;
    try {
      std::shared_lock<std::shared_mutex> lock(mutex_);
      if (!live_locked()) return false;
      if (serial_.write(data, size) == size) return true;
    } catch (const std::exception & e) {
      fault(e.what());
      return false;
    }
    fault("short write");
    return false;
  }

  void close() noexcept
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    close_locked();
  }

private:
  struct DeviceIdentity
  {
    dev_t filesystem_device = 0;
    ino_t inode = 0;
    dev_t terminal_device = 0;
    bool operator==(const DeviceIdentity & other) const
    {
      return filesystem_device == other.filesystem_device &&
             inode == other.inode && terminal_device == other.terminal_device;
    }
  };

  static bool identity_of(const std::string & port, DeviceIdentity & result)
  {
    struct stat metadata {};
    // stat() follows a stable udev symlink to its current terminal.
    if (::stat(port.c_str(), &metadata) != 0 || !S_ISCHR(metadata.st_mode))
      return false;
    result = {metadata.st_dev, metadata.st_ino, metadata.st_rdev};
    return true;
  }

  bool live_locked() const
  {
    DeviceIdentity current{};
    return serial_.isOpen() && identity_of(port_, current) && current == identity_;
  }

  void close_locked() noexcept
  {
    if (serial_.isOpen()) {
      try { serial_.close(); }
      catch (const std::exception & e) {
        std::fprintf(stderr, "[%s] Close failed: %s\n", label_.c_str(), e.what());
      }
    }
    identity_ = {};
  }

  void fault(const char * reason)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (serial_.isOpen())
      std::fprintf(stderr, "[%s] I/O error (%s), closing: %s\n",
                   label_.c_str(), reason, port_.c_str());
    close_locked();
    next_retry_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  }

  std::string port_, label_;
  serial::Serial serial_;
  mutable std::shared_mutex mutex_;
  DeviceIdentity identity_{};
  uint64_t generation_ = 0;
  std::chrono::steady_clock::time_point next_retry_{};
  std::chrono::steady_clock::time_point last_error_log_{};
};
}  // namespace io
