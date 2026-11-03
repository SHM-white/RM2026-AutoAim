#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace auto_aim
{
// The defaults preserve the *old lightbar-endpoint* PnP correspondences, not
// necessarily the new season's panel/screen corners. Measure before firing.
struct ArmorGeometry
{
  double small_width_m = 0.135;
  double small_height_m = 0.056;
  double big_width_m = 0.230;
  double big_height_m = 0.056;

  static ArmorGeometry from_yaml(const YAML::Node & node)
  {
    ArmorGeometry result;
    if (!node) return result;
    auto read = [&](const char * name, double & value) {
      if (!node[name]) return;
      value = node[name].as<double>();
      if (!std::isfinite(value) || value <= 0.0 || value >= 1.0)
        throw std::invalid_argument(std::string("Invalid armor_geometry.") + name);
    };
    read("small_width_m", result.small_width_m);
    read("small_height_m", result.small_height_m);
    read("big_width_m", result.big_width_m);
    read("big_height_m", result.big_height_m);
    return result;
  }

  static std::vector<cv::Point3f> object_points(double width, double height)
  {
    if (!std::isfinite(width) || !std::isfinite(height) || width <= 0 || height <= 0)
      throw std::invalid_argument("Armor dimensions must be positive and finite");
    // Image point order: top-left, top-right, bottom-right, bottom-left.
    return {{0.0f, static_cast<float>(width / 2), static_cast<float>(height / 2)},
            {0.0f, static_cast<float>(-width / 2), static_cast<float>(height / 2)},
            {0.0f, static_cast<float>(-width / 2), static_cast<float>(-height / 2)},
            {0.0f, static_cast<float>(width / 2), static_cast<float>(-height / 2)}};
  }
};
}  // namespace auto_aim
