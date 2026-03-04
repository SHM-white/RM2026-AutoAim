#include "cmd_vel_subscriber.hpp"

void io::CmdVelSubscriber::topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    NavData nav_data;
    nav_data.linear_x = msg->linear.x;
    nav_data.linear_y = msg->linear.y;
    nav_data.linear_z = msg->linear.z;
    nav_data.angular_x = msg->angular.x;
    nav_data.angular_y = msg->angular.y;
    nav_data.angular_z = msg->angular.z;

    nav_queue_.push({nav_data, std::chrono::steady_clock::now()});
}

std::optional<io::NavData> io::CmdVelSubscriber::last_data()
{
    if (nav_queue_.empty()) {
        return std::nullopt;
    }

    auto [latest, timestamp] = nav_queue_.pop();
    (void)timestamp;

    while (!nav_queue_.empty()) {
        auto [current, current_timestamp] = nav_queue_.pop();
        (void)current_timestamp;
        latest = current;
    }
    return latest;
}
