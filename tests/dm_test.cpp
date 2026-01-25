#include <chrono>
#include <thread>

#include "io/dm_imu/dm_imu.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;
  io::DM_IMU imu;
  tools::Plotter plotter;

  auto start_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = imu.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
    data["t"] = tools::delta_time(timestamp, start_time);
    data["imu_yaw"] = eulers[0];
    data["imu_pitch"] = eulers[1];
    data["imu_roll"] = eulers[2];
    plotter.plot(data);
  } 

  return 0;
}