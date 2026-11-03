#include "yolo.hpp"

#include <yaml-cpp/yaml.h>
#include <mutex>
#include <unordered_map>

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

namespace auto_aim
{
namespace
{
std::unordered_map<std::string, YOLO::Factory> & registry()
{
  static std::unordered_map<std::string, YOLO::Factory> instance;
  return instance;
}

std::mutex & registry_mutex()
{
  static std::mutex mutex;
  return mutex;
}
}

void YOLO::register_model(const std::string & name, Factory factory)
{
  if (name.empty() || !factory) throw std::invalid_argument("Invalid model factory");
  std::lock_guard<std::mutex> lock(registry_mutex());
  if (!registry().emplace(name, std::move(factory)).second)
    throw std::invalid_argument("Model already registered: " + name);
}

YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  } else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  } else if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  } else {
    Factory factory;
    {
      std::lock_guard<std::mutex> lock(registry_mutex());
      const auto it = registry().find(yolo_name);
      if (it == registry().end()) throw std::runtime_error("Unknown yolo adapter: " + yolo_name);
      factory = it->second;
    }
    yolo_ = factory(config_path, debug);
    if (!yolo_) throw std::runtime_error("Model factory returned null: " + yolo_name);
  }
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
