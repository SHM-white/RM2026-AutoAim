#include <iostream>
#include <thread>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cmd_vel_publisher_test");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    "/red_standard_robot1/cmd_vel", 10);

  std::cout << "Publisher initialized. Broadcasting random Twist messages at 0.2Hz..." << std::endl;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  rclcpp::WallRate rate(0.2); // 0.2 Hz means once every 5 seconds

  while (rclcpp::ok()) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = dis(gen);
    msg.linear.y = dis(gen);
    msg.linear.z = dis(gen);
    msg.angular.x = dis(gen);
    msg.angular.y = dis(gen);
    msg.angular.z = dis(gen);

    std::cout << "\nPublishing Random NavData:"
              << "\n  linear:  [" << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z << "]"
              << "\n  angular: [" << msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << "]"
              << std::endl;

    publisher->publish(msg);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}