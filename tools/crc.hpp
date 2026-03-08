#ifndef TOOLS__CRC_HPP
#define TOOLS__CRC_HPP

#include <cstdint>


namespace tools
{
// len不包括crc8
uint8_t get_crc8(const uint8_t * data, uint16_t len);

// len包括crc8
bool check_crc8(const uint8_t * data, uint16_t len);

// len不包括crc16
uint16_t get_crc16(const uint8_t * data, uint32_t len);

// len包括crc16
bool check_crc16(const uint8_t * data, uint32_t len);

// len不包括crc16，专门针对IMU数据帧的CRC计算
uint16_t get_imu_crc16(const uint8_t * ptr, uint16_t len);

// len包括crc16，专门针对IMU数据帧的CRC校验
bool check_imu_crc16(const uint8_t * data, uint16_t len);
}  // namespace tools

#endif  // TOOLS__CRC_HPP
