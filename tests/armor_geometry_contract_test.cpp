#include "tasks/auto_aim/armor_geometry.hpp"
#include <cassert>
#include <cmath>

int main()
{
  auto legacy = auto_aim::ArmorGeometry::from_yaml(YAML::Node());
  auto small = auto_aim::ArmorGeometry::object_points(legacy.small_width_m, legacy.small_height_m);
  assert(small.size() == 4);
  assert(std::abs(small[0].y - 0.0675f) < 1e-5);
  assert(small[0].z > 0 && small[3].z < 0);
  auto newer = YAML::Load("small_width_m: 0.12\nsmall_height_m: 0.07\n");
  auto configured = auto_aim::ArmorGeometry::from_yaml(newer);
  assert(std::abs(configured.small_height_m - 0.07) < 1e-9);
  bool rejected = false;
  try { auto_aim::ArmorGeometry::from_yaml(YAML::Load("big_width_m: -1")); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  return 0;
}
