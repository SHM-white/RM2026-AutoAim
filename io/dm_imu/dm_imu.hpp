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
#include <optional>

#include "tools/thread_safe_queue.hpp"

namespace io
{
// IMU数据接收帧格式
struct __attribute__((packed)) IMU_Receive_Frame
{
  uint8_t FrameHeader1;  // 0x55
  uint8_t flag1;         // 0xAA
  uint8_t slave_id1;     // 0x01
  uint8_t reg_acc;       // 0x01
  uint32_t accx_u32;     // 加速度X
  uint32_t accy_u32;     // 加速度Y
  uint32_t accz_u32;     // 加速度Z
  uint16_t crc1;         // CRC16校验码
  uint8_t FrameEnd1;     // 0xAA

  uint8_t FrameHeader2;  // ?
  uint8_t flag2;
  uint8_t slave_id2;
  uint8_t reg_gyro;
  uint32_t gyrox_u32;  // 角速度X
  uint32_t gyroy_u32;  // 角速度Y
  uint32_t gyroz_u32;  // 角速度Z
  uint16_t crc2;       // 128bit长度 len=16B
  uint8_t FrameEnd2;

  uint8_t FrameHeader3;
  uint8_t flag3;
  uint8_t slave_id3;
  uint8_t reg_euler;   // r-p-y
  uint32_t roll_u32;   // 欧拉角roll
  uint32_t pitch_u32;  // 欧拉角pitch
  uint32_t yaw_u32;    // 欧拉角yaw
  uint16_t crc3;
  uint8_t FrameEnd3;
};

typedef struct
{
  float accx;   // 加速度X
  float accy;   // 加速度Y
  float accz;   // 加速度Z
  float gyrox;  // 角速度X
  float gyroy;  // 角速度Y
  float gyroz;  // 角速度Z
  float roll;   // 欧拉角roll
  float pitch;  // 欧拉角pitch
  float yaw;    // 欧拉角yaw
} IMU_Data;

class DM_IMU
{
public:
  DM_IMU();
  ~DM_IMU();

  enum class IMU_COMMAND
  {
    RESTART,                    // 重启IMU 
    DISABLE_RS485_ACTIVE,       // 关闭485主动模式
    DISABLE_ACC_OUTPUT,         // 关闭加速度输出
    DISABLE_GYRO_OUTPUT,        // 关闭角速度输出
    DISABLE_EULER_OUTPUT,       // 关闭欧拉角输出
    DISABLE_QUAT_OUTPUT,        // 关闭四元数输出
    DISABLE_CAN_ACTIVE,         // 关闭CAN主动模式
    ENABLE_RS485_ACTIVE,        // 开启485主动模式
    ENABLE_ACC_OUTPUT,          // 开启加速度输出
    ENABLE_GYRO_OUTPUT,         // 开启角速度输出
    ENABLE_EULER_OUTPUT,        // 开启欧拉角输出
    ENABLE_QUAT_OUTPUT,         // 开启四元数输出
    ENABLE_CAN_ACTIVE,          // 开启CAN主动模式
    SAVE_PARAMS,                // 保存参数
    START_GYRO_CALIBRATE,       // 启动陀螺静态校准
    START_ACC_CALIBRATE,        // 启动加计六面校准
    DISABLE_TEMP_CONTROL,       // 关闭温度控制
    ENABLE_TEMP_CONTROL,        // 开启温度控制
    SET_TEMP,                   // 设置控制温度
    ENTER_NORMAL_MODE,          // 进入正常模式
    ENTER_SET_MODE,             // 进入设置模式
    SET_CAN_ID,                 // 设置CANID
    SET_MST_ID,                 // 设置MSTID
    SET_OUTPUT_INTERFACE,       // 设置输出接口
    RESTORE_FACTORY,            // 恢复出厂设置
    ZERO_ANGLE                  // 角度置零
  };
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // IMU operations
  void send_command(IMU_COMMAND command, std::optional<uint8_t> param = std::nullopt);

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
