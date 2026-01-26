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

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | usage}"
  "{@config-path   | configs/test2.yaml | config path}";

double yaw_cal(double t)
{
  double A = 120;  // Amplitude (radians)
  double T = 2.5;  // Period (seconds)

  return A * std::sin(2 * M_PI * t / T);
}

double pitch_cal(double t)
{
  double A = 45;
  double T = 5.0;
  return A * std::sin(2 * M_PI * t / T);
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

  // Initialize CBoardUART
  io::CBoardUART cboard(config_path);

  std::cout << "Waiting for gimbal to zero..." << std::endl;
  io::Command init_command{true, false, 0, 0};
  cboard.send(init_command);
  std::this_thread::sleep_for(3s);

  std::cout << "Starting head shaking..." << std::endl;

  io::Command command;
  command.control = true;
  command.shoot = false;

  auto start_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    double t = tools::delta_time(now, start_time);

    // Calculate yaw
    command.control = true;
    command.yaw = 10 / 57.3;
    command.pitch = 10 / 57.3;
    command.shoot = shoot_cal(t);
    //tools::logger()->debug("t: {:.2f}, shoot: {}", t, command.shoot);
    // Send command
    cboard.send(command);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}
