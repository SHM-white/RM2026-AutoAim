#include <iostream>
#include <thread>
#include <chrono>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"

int main()
{
  std::cout << "ROS2 top-level subscriber test started." << std::endl;
  std::cout << "Waiting for data on /red_standard_robot1/cmd_vel..." << std::endl;

  io::ROS2 ros2_node;
  tools::Exiter exiter;

  while (!exiter.exit()) {
    auto data_opt = ros2_node.get_last_cmd_vel_data();
    if (data_opt.has_value()) {
      auto data = data_opt.value();
      std::cout << "Received NavData:"
                << "\n  linear:  [" << data.linear_x << ", " << data.linear_y << ", " << data.linear_z << "]"
                << "\n  angular: [" << data.angular_x << ", " << data.angular_y << ", " << data.angular_z << "]"
                << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  return 0;
}