#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include "serial/serial.h"

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/cboard_uart.hpp"
#include "io/dm_imu/dm_imu.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{@config-path   | configs/test2.yaml    | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  // 1. 读取配置文件
  auto yaml = tools::load(config_path);
  // auto com_port = tools::read<std::string>(yaml, "com_port");
  auto com_port = std::string("/dev/ttyUSB0");

  tools::logger()->info("Target serial port: {}", com_port);

  // 2. 初始化串口
  serial::Serial serial_;
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(to);
    serial_.open();
  } catch (const serial::IOException& e) {
    tools::logger()->error("Failed to open serial: {}", e.what());
    return -1;
  }

  if (!serial_.isOpen()) {
    tools::logger()->error("Serial port not open.");
    return -1;
  }
  
  tools::logger()->info("Successfully opened port: {}. Listening for data...", com_port);

  // 3. 循环监听串口数据
  tools::Exiter exiter;
  uint8_t buffer[43];

  while (!exiter.exit()) {
    if (serial_.available()) {
      size_t bytes_read = serial_.read(buffer, sizeof(buffer));
      if (bytes_read > 0) {
        // 以 Hex 格式拼接输出
        std::string hex_str;
        for (size_t i = 0; i < bytes_read; ++i) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", buffer[i]);
          hex_str += hex;
        }
        tools::logger()->info("Read {} bytes: {}", bytes_read, hex_str);
      }
    } else {
      std::this_thread::sleep_for(1ms);
    }
  }

  tools::logger()->info("Closing serial port...");
  serial_.close();
  return 0;
}
