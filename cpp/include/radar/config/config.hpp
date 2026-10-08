#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
namespace radar {
struct DetectionConfig {
  float conf_thres = 0.25f;
  float iou_thres = 0.45f;
  int max_det = 300;
  int img_size = 640;
};
struct AppConfig {
  std::string state = "B";
  std::string camera_mode = "test";
  bool multi_car_recognition = true;
  std::string infer_backend = "auto";
  bool show_ui = true;
  cv::Size map_size{2800, 1500};
  std::string test_image_path;
  std::string test_video_path;
  bool test_loop = false;
  double playback_speed = 1.0;
  double exposure_time = 16000.0;
  double gain = 17.9;
  std::string device_serial;
  DetectionConfig car, armor;
  cv::Size canvas_size{640, 640};
  int canvas_padding = 10;
  std::string project_root = "..";
  std::string calib_red, calib_blue, calib_red_yaml, calib_blue_yaml;
  std::string map_red, map_blue, map_mask, map_backup;
  std::string car_engine, armor_engine, car_onnx, armor_onnx;
  std::string car_names_yaml, armor_names_yaml;
  int bench_warmup = 10;
  int bench_measure = 100;
  std::string bench_report_path = "cpp/tools/bench_report.md";
  std::string Resolve(const std::string& rel) const;
};
bool LoadConfig(const std::string& path, AppConfig& out, std::string* err = nullptr);
std::vector<std::string> LoadClassNames(const std::string& path);
}  // namespace radar
