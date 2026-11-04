#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

namespace auto_aim
{
namespace
{
// SEASON UPDATE POINT: a new detector only changes the implementation
// returned by this function. Legacy yolo_name is retained for existing apps;
// no per-feature/year configuration or runtime registration is introduced.
std::unique_ptr<YOLOBase> create_season_detector(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  const auto name = yaml["yolo_name"].as<std::string>();
  if (name == "yolov8") return std::make_unique<YOLOV8>(config_path, debug);
  if (name == "yolo11") return std::make_unique<YOLO11>(config_path, debug);
  if (name == "yolov5") return std::make_unique<YOLOV5>(config_path, debug);
  throw std::runtime_error("Unknown YOLO adapter: " + name);
}
}  // namespace

YOLO::YOLO(const std::string & config_path, bool debug)
: yolo_(create_season_detector(config_path, debug))
{
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

bool YOLO::supports_text_prompts() const { return yolo_->supports_text_prompts(); }

void YOLO::set_text_prompts(const std::vector<std::string> & prompts)
{
  yolo_->set_text_prompts(prompts);
}
}  // namespace auto_aim
