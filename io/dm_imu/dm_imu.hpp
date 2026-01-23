#ifndef IO__Dm_Imu_HPP
#define IO__Dm_Imu_HPP

#include <math.h>
#include <serial/serial.h>

#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io
{
// IMU数据接收帧格式
struct __attribute__((packed)) IMU_Receive_Frame
{
  uint8_t FrameHeader1;   // 0x55
  uint8_t flag1;          // 0xAA
  uint8_t slave_id1;      // 0x01
  uint8_t reg_acc;        // 0x01
  uint32_t accx_u32;      // 加速度X
  uint32_t accy_u32;      // 加速度Y
  uint32_t accz_u32;      // 加速度Z
  uint16_t crc1;          // CRC16校验码
  uint8_t FrameEnd1;      // 0xAA

  uint8_t FrameHeader2;   // ?
  uint8_t flag2;
  uint8_t slave_id2;
  uint8_t reg_gyro;
  uint32_t gyrox_u32;     // 角速度X
  uint32_t gyroy_u32;     // 角速度Y
  uint32_t gyroz_u32;     // 角速度Z
  uint16_t crc2;          // 128bit长度 len=16B
  uint8_t FrameEnd2;

  uint8_t FrameHeader3;
  uint8_t flag3;
  uint8_t slave_id3;
  uint8_t reg_euler;      // r-p-y
  uint32_t roll_u32;      // 欧拉角roll
  uint32_t pitch_u32;     // 欧拉角pitch
  uint32_t yaw_u32;       // 欧拉角yaw
  uint16_t crc3;
  uint8_t FrameEnd3;
};

typedef struct
{
  float accx;             // 加速度X
  float accy;             // 加速度Y
  float accz;             // 加速度Z
  float gyrox;            // 角速度X
  float gyroy;            // 角速度Y
  float gyroz;            // 角速度Z
  float roll;             // 欧拉角roll
  float pitch;            // 欧拉角pitch
  float yaw;              // 欧拉角yaw
} IMU_Data;

class DM_IMU
{
public:
  DM_IMU();
  ~DM_IMU();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void init_serial();
  void get_imu_data_thread();

  serial::Serial serial_;
  std::thread rec_thread_;

  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_, data_behind_;

  std::atomic<bool> stop_thread_{false};
  IMU_Receive_Frame receive_data{};  //receive data frame
  IMU_Data data{};
};

}  // namespace io

#endif
