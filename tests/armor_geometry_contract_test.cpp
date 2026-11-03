#include "tasks/auto_aim/armor_geometry.hpp"
#include <cassert>
#include <cmath>

int main()
{
  const auto & small = auto_aim::armor_object_points(auto_aim::ArmorType::small, auto_aim::ArmorName::two);
  const auto & big = auto_aim::armor_object_points(auto_aim::ArmorType::big, auto_aim::ArmorName::one);
  assert(small.size() == 4 && big.size() == 4);
  assert(std::abs(small[0].y - 0.0675f) < 1e-5f);
  assert(std::abs(big[0].y - 0.115f) < 1e-5f);
  assert(small[0].z > 0 && small[3].z < 0);
  return 0;
}
