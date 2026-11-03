#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include "armor.hpp"

namespace auto_aim
{
// SEASON UPDATE POINT: replace ONLY this function when the physical target
// feature geometry changes. The 3D points MUST match the detector's four
// observed 2D keypoints in TL/TR/BR/BL order, in meters.
// These are 2026 lightbar endpoints, not verified 2027 display corners.
inline const std::vector<cv::Point3f> & armor_object_points(ArmorType type, ArmorName name)
{
  (void)name;  // Season-specific names may require distinct geometry.
  static const std::vector<cv::Point3f> small = {
    {0.0f,  0.0675f,  0.028f},
    {0.0f, -0.0675f,  0.028f},
    {0.0f, -0.0675f, -0.028f},
    {0.0f,  0.0675f, -0.028f},
  };
  static const std::vector<cv::Point3f> big = {
    {0.0f,  0.115f,  0.028f},
    {0.0f, -0.115f,  0.028f},
    {0.0f, -0.115f, -0.028f},
    {0.0f,  0.115f, -0.028f},
  };
  return type == ArmorType::big ? big : small;
}
}  // namespace auto_aim
