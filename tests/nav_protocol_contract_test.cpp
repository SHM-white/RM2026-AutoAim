#include "io/vision_protocol.hpp"

#include <cassert>
#include <cmath>
#include <cstring>

int main()
{
  io::NavData command{};
  command.linear_x = 1.25;
  command.angular_z = -0.5;
  command.mode = 1;

  // The serial packet must always use AB even if the caller provides an
  // uninitialized, zeroed, or old-season header.
  command.head[0] = 'C';
  command.head[1] = 'D';
  auto packet = io::seal_nav_frame(command);
  assert(packet.head[0] == 'A' && packet.head[1] == 'B');
  assert(packet.mode == 1);
  assert(std::abs(packet.linear_x - 1.25) < 1e-12);
  assert(std::abs(packet.angular_z + 0.5) < 1e-12);
  assert(tools::check_crc16(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)));
  auto bytes = reinterpret_cast<uint8_t *>(&packet);
  bytes[8] ^= 0x01;
  assert(!tools::check_crc16(bytes, sizeof(packet)));
  return 0;
}
