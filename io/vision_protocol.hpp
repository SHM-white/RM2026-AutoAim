#pragma once

#include <cstdint>
#include <type_traits>

#include "tools/crc.hpp"

namespace io
{
// 2027 protocol: two independent UARTs, each with the same 'AB' frame header.
// The MCU must use the same packed layouts, IEEE-754 floats/doubles, and
// little-endian field encoding. Separate ports distinguish message types.
inline constexpr uint8_t gimbal_struct_header[2] = {'A', 'B'};
inline constexpr uint8_t nav_struct_header[2] = {'A', 'B'};

#pragma pack(push, 1)
struct NavData
{
  uint8_t head[2] = {nav_struct_header[0], nav_struct_header[1]};
  uint8_t mode = 0;
  uint8_t empty1 = 0;
  uint8_t empty2 = 0;
  double linear_x = 0, linear_y = 0, linear_z = 0;
  double angular_x = 0, angular_y = 0, angular_z = 0;
  uint16_t crc16 = 0;
};

struct GimbalToVision
{
  uint8_t head[2] = {gimbal_struct_header[0], gimbal_struct_header[1]};
  uint8_t mode;
  float q[4];
  float yaw, yaw_vel, pitch, pitch_vel, bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
};

struct VisionToGimbal
{
  const int8_t head[2] = {'A', 'B'};
  uint8_t mode;
  float yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc;
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(NavData) == 55, "NavData wire format changed");
static_assert(sizeof(GimbalToVision) == 43, "Gimbal RX wire format changed");
static_assert(sizeof(VisionToGimbal) == 29, "Gimbal TX wire format changed");
static_assert(std::is_standard_layout<NavData>::value, "Wire packet must be POD layout");

// A dedicated function keeps navigation framing away from Gimbal UART.
inline NavData seal_nav_frame(const NavData & data)
{
  NavData packet = data;
  packet.head[0] = nav_struct_header[0];
  packet.head[1] = nav_struct_header[1];
  packet.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t *>(&packet),
                                   sizeof(packet) - sizeof(packet.crc16));
  return packet;
}
}  // namespace io
