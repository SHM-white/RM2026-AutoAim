#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#pragma pack(push, 1) 
struct TwistData {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
};
#pragma pack(pop)
static_assert(sizeof(TwistData) <= 64);

bool send_full(int fd, const void * data, size_t size)
{
    const auto * ptr = reinterpret_cast<const uint8_t *>(data);
    size_t sent = 0;
    while (sent < size) {
        auto ret = send(fd, ptr + sent, size - sent, 0);
        if (ret <= 0) return false;
        sent += static_cast<size_t>(ret);
    }
    return true;
}

int try_connect()
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(16667);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

int main(int argc, char * argv[]) {
    (void)argc;
    (void)argv;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> linear_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> angular_dist(-3.14, 3.14);

    int fd = -1;
    std::cout << "[nav_to_gimbal_test] start random send at 100Hz" << std::endl;

    while (true) {
        if (fd < 0) {
            fd = try_connect();
            if (fd < 0) {
                std::cerr << "[nav_to_gimbal_test] connect failed, retry in 1s" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            std::cout << "[nav_to_gimbal_test] connected to 127.0.0.1:16667" << std::endl;
        }

        TwistData data{};
        data.linear_x = linear_dist(rng);
        data.linear_y = linear_dist(rng);
        data.linear_z = linear_dist(rng);
        data.angular_x = angular_dist(rng);
        data.angular_y = angular_dist(rng);
        data.angular_z = angular_dist(rng);

        if (!send_full(fd, &data, sizeof(data))) {
            std::cerr << "[nav_to_gimbal_test] send failed, reconnecting..." << std::endl;
            close(fd);
            fd = -1;
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }

    return 0;
}
