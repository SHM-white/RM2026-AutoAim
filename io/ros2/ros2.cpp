#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  cmd_vel_subscriber_ = std::make_shared<CmdVelSubscriber>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });

  cmd_vel_subscriber_spin_thread_ = std::make_unique<std::thread>([this]() { cmd_vel_subscriber_->start(); });

  nav_referee_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("nav_referee_pub_node", "nav_referee_data", 10);
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
  cmd_vel_subscriber_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}

std::optional<NavData> ROS2::get_last_cmd_vel_data()
{
  return cmd_vel_subscriber_->last_data();
}


void ROS2::publish_nav_referee_data(uint16_t cmd_id, const std::vector<uint8_t> & data)
{
  if (nav_referee_pub_) {
    std_msgs::msg::UInt8MultiArray msg;
    // 首两位字节放cmd_id
    msg.data.push_back(cmd_id & 0xFF);
    msg.data.push_back((cmd_id >> 8) & 0xFF);
    msg.data.insert(msg.data.end(), data.begin(), data.end());
    nav_referee_pub_->publish(msg);
  }
}
} // namespace io
