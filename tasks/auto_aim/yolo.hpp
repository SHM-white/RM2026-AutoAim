#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
// Model adapters must convert their own outputs to Armor and provide four
// *measured* physical keypoints. A detection bounding box is not a PnP target.
class YOLOBase
{
public:
  virtual ~YOLOBase() = default;
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;
  virtual std::list<Armor> postprocess(double, cv::Mat &, const cv::Mat &, int)
  {
    throw std::logic_error("This model adapter does not expose raw postprocess()");
  }

  // YOLOE-style prompts are opt-in: existing YOLOv5/v8/11 do not support them.
  virtual bool supports_text_prompts() const { return false; }
  virtual void set_text_prompts(const std::vector<std::string> &)
  {
    throw std::logic_error("Text prompts are not implemented for this detector");
  }
};

// Single model-selection seam: replace create_season_detector() in yolo.cpp
// to swap this season's detector implementation. No registry or plugin config.
class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);
  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);
  std::list<Armor> postprocess(double scale, cv::Mat & output,
                               const cv::Mat & bgr_img, int frame_count);
  bool supports_text_prompts() const;
  void set_text_prompts(const std::vector<std::string> & prompts);

private:
  std::unique_ptr<YOLOBase> yolo_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
