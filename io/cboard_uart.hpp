#include "cboard.hpp"
#include "io/serial/include/serial/serial.h"
#include "tools/thread_safe_queue.hpp"
#include <atomic>
#include <thread>


namespace io
{
class CBoardUART
{
public:
    double bullet_speed;
    Mode mode;
    ShootMode shoot_mode;
    double ft_angle;

    CBoardUART(const std::string & config_path);
    ~CBoardUART();

    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
    void send(Command command) const;

private:
    serial::Serial serial_;
    std::thread thread_;
    std::atomic<bool> stop_thread_;

    struct IMUData
    {
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
    };

    tools::ThreadSafeQueue<IMUData> queue_;
    IMUData data_ahead_;
    IMUData data_behind_;

    void read_thread();
};
}  // namespace io