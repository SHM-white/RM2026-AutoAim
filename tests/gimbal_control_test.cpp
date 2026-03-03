#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>

#include "io/cboard_uart.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "io/gimbal/gimbal.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | usage}"
  "{@config-path   | configs/test2.yaml | config path}";

struct TrajectoryState
{
  double pos;
  double vel;
  double acc;
};

TrajectoryState yaw_cal(double t)
{
  // double A = 120;  // Amplitude (degree)
  double T = 4;  // Period (seconds)
  double w = 2 * M_PI / T;

  // return {
  //   A * std::sin(w * t),
  //   A * w * std::cos(w * t),
  //   -A * w * w * std::sin(w * t)
  // };

  // circle motion
  
  return{ 
    std::fmod((360.0 / T) * t, 360.0) - 180.0, 
    360.0 / T, 
    0 
  };

}

TrajectoryState pitch_cal(double t)
{
  double A = 30;
  double T = 5.0;
  double w = 2 * M_PI / T;
  return {
    A * std::sin(w * t),
    A * w * std::cos(w * t),
    -A * w * w * std::sin(w * t)
  };
}

bool shoot_cal(double t)
{
  double shoot_interval = 5.0;  // seconds
  return std::fmod(t, shoot_interval) < 2;
}

int main(int argc, char * argv[])
{
  auto plotter = tools::Plotter{};
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);

  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  // // Initialize CBoardUART
  // io::CBoardUART cboard(config_path);
  io::Gimbal gimbal(config_path);

  std::cout << "Waiting for gimbal to zero..." << std::endl;
  io::Command init_command{true, false, 0, 0};
  gimbal.send(true, false, 0, 0, 0, 0, 0, 0);
  std::this_thread::sleep_for(3s);

  std::cout << "Starting head shaking..." << std::endl;

  io::Command command;
  command.control = true;
  command.shoot = false;
  auto start_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    double t = tools::delta_time(now, start_time);
    auto yaw_traj = yaw_cal(t);
    auto pitch_traj = pitch_cal(t);
    // Calculate yaw
    command.control = true;
    command.yaw = yaw_traj.pos / 57.3;
    double yaw_vel = yaw_traj.vel / 57.3;
    double yaw_acc = yaw_traj.acc / 57.3;
    // command.pitch = pitch_cal(t) / 57.3;
    // command.yaw = 0 / 57.3;
    command.pitch = pitch_traj.pos / 57.3;
    double pitch_vel = pitch_traj.vel / 57.3;
    double pitch_acc = pitch_traj.acc / 57.3;

    command.shoot = shoot_cal(t);
    
    // Plot command
    nlohmann::json json;
    json["send_yaw"] = command.yaw * 57.3;
    json["send_yaw_vel"] = yaw_vel * 57.3;
    json["send_yaw_acc"] = yaw_acc * 57.3;
    json["send_pitch"] = command.pitch * 57.3;
    json["send_pitch_vel"] = pitch_vel * 57.3;
    json["send_pitch_acc"] = pitch_acc * 57.3;  
    json["send_shoot"] = command.shoot ? 1 : 0;
    plotter.plot(json);
//tools::logger()->debug("t: {:.2f}, shoot: {}", t, command.shoot);
    // Send command
    gimbal.send(command.control, command.shoot, command.yaw, yaw_vel, yaw_acc, command.pitch, pitch_vel, pitch_acc);


    std::this_thread::sleep_for(10ms);
  }

  return 0;
}
