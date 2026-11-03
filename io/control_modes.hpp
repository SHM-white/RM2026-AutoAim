#pragma once
#include <string>
#include <vector>

namespace io
{
// Shared control modes, independent from the retired C-board CAN transport.
enum Mode { idle, auto_aim, small_buff, big_buff, outpost };
inline const std::vector<std::string> MODES = {
  "idle", "auto_aim", "small_buff", "big_buff", "outpost"
};

enum ShootMode { left_shoot, right_shoot, both_shoot };
inline const std::vector<std::string> SHOOT_MODES = {
  "left_shoot", "right_shoot", "both_shoot"
};
}  // namespace io
