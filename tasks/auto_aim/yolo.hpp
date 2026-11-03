#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <functional>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
// Detector/model interface. Geometry conversion belongs to the model adapter:
// arbitrary YOLO detection boxes are NOT valid PnP keypoints.
class YOLOBase
{
public:
  virtual ~YOLOBase() = default;
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  // Kept for existing OpenVINO adapters; new backends may only implement detect().
  virtual std::list<Armor> postprocess(
    double, cv::Mat &, const cv::Mat &, int)
  {
    throw std::logic_error("This model adapter does not expose raw postprocess()");
  }
  virtual bool supports_text_prompts() const { return false; }
  virtual void set_text_prompts(const std::vector<std::string> &)
  {
    throw std::logic_error("This model does not support text prompts");
  }
};

class YOLO
{
public:
  using Factory = std::function<std::unique_ptr<YOLOBase>(const std::string &, bool)>;

  // Register a custom detector (e.g. YOLOE-26 with a verified keypoint adapter)
  // before constructing YOLO. Existing models remain available by name.
  static void register_model(const std::string & name, Factory factory);

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
