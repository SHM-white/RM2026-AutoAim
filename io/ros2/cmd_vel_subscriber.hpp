#pragma once

#include <functional>
#include <mutex>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tools/crc.hpp"
#include "tools/thread_safe_queue.hpp"
#include "io/gimbal/gimbal.hpp"

namespace io
{

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber();

  ~CmdVelSubscriber() = default;

  void start() { rclcpp::spin(this->shared_from_this()); }
  std::optional<NavData> last_data();

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  tools::ThreadSafeQueue<std::tuple<NavData, std::chrono::steady_clock::time_point>> nav_queue_{1000};
};

}  // namespace io