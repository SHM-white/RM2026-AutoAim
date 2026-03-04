#pragma once

#include <functional>
#include <mutex>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tools/crc.hpp"
#include "tools/thread_safe_queue.hpp"
#include "io/gimbal/gimbal_with_nav.hpp"

namespace io
{

class CmdVelSubscriber : public rclcpp::Node
{
public:
  explicit CmdVelSubscriber()
  : Node("cmd_vel_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/red_standard_robot1/cmd_vel", 10,
      std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "已订阅话题");
    RCLCPP_INFO(this->get_logger(), "等待接收数据");
  }
  ~CmdVelSubscriber() = default;

  std::optional<NavData> last_data();
  bool has_new_data() const
  {
    return !(nav_queue_.empty());
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  tools::ThreadSafeQueue<std::tuple<NavData, std::chrono::steady_clock::time_point>> nav_queue_{1000};
};

}  // namespace io