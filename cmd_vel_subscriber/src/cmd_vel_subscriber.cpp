#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

// 定义结构体
#pragma pack(push, 1) 
struct TwistData {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
    
    // 从Twist消息更新结构体
    void updateFromMsg(const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_x = msg->linear.x;
        linear_y = msg->linear.y;
        linear_z = msg->linear.z;
        angular_x = msg->angular.x;
        angular_y = msg->angular.y;
        angular_z = msg->angular.z;
    }
};
#pragma pack(pop)
static_assert(sizeof(TwistData) <= 64);

class CmdVelSubscriber : public rclcpp::Node {
public:
    CmdVelSubscriber() : Node("cmd_vel_subscriber") {
        // 创建订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/red_standard_robot1/cmd_vel", 
            10,
            std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1)
        );
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        try_connect();
        RCLCPP_INFO(this->get_logger(), "已订阅话题");
        RCLCPP_INFO(this->get_logger(), "等待接收数据");
    }
    ~CmdVelSubscriber(){
        close(fd_);
    }
private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 将数据保存到结构体
        twist_data_.updateFromMsg(msg);

        while (send(fd_, reinterpret_cast<const char*>(&twist_data_), sizeof(twist_data_), 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "发送数据失败，正在重试...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            try_connect();
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    TwistData twist_data_;  // 保存数据的结构体
    int fd_;
    bool try_connect(){
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(16667);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        while (connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            static int retry_count = 0;
            if (retry_count++ > 5) {
                RCLCPP_ERROR(this->get_logger(), "无法连接到服务器");
                throw std::runtime_error("无法连接到服务器");
                return false;
            }
            RCLCPP_WARN(this->get_logger(), "连接服务器失败，正在重试...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return true;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelSubscriber>());
    rclcpp::shutdown();
    return 0;
}
