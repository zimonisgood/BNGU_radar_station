#include "radar/config/config.hpp"
#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>
namespace radar {
std::string AppConfig::Resolve(const std::string& rel) const {
  if (rel.empty()) return rel;
  if (rel[0] == '/') return rel;
  if (project_root.empty() || project_root == ".") return rel;
  return project_root.back() == '/' ? project_root + rel : project_root + "/" + rel;
}
static std::string S(const YAML::Node& n, const std::string& d="") {
  if (!n || n.IsNull()) return d;
  try { return n.as<std::string>(); } catch (...) { return d; }
}
bool LoadConfig(const std::string& path, AppConfig& o, std::string* err) {
  try {
    auto root = YAML::LoadFile(path);
    auto g = root["global"];
    o.state = S(g["state"], "B");
    o.camera_mode = S(g["camera_mode"], "test");
    o.multi_car_recognition = g["multi_car_recognition"] ? g["multi_car_recognition"].as<bool>() : true;
    o.infer_backend = S(g["infer_backend"], "auto");
    o.show_ui = g["show_ui"] ? g["show_ui"].as<bool>() : true;
    if (g["map_size"] && g["map_size"].size() >= 2)
      o.map_size = cv::Size(g["map_size"][0].as<int>(), g["map_size"][1].as<int>());
    if (auto t = g["test"]) {
      o.test_image_path = S(t["image_path"]);
      o.test_video_path = S(t["video_path"]);
      o.test_loop = t["loop"] ? t["loop"].as<bool>() : false;
      o.playback_speed = t["playback_speed"] ? t["playback_speed"].as<double>() : 1.0;
    }
    if (auto c = root["camera_params"]) {
      o.exposure_time = c["exposure_time"] ? c["exposure_time"].as<double>() : 16000.0;
      o.gain = c["gain"] ? c["gain"].as<double>() : 17.9;
      o.device_serial = S(c["device_serial"]);
    }
    if (auto d = root["detection"]) {
      if (auto car = d["car"]) {
        o.car.conf_thres = car["conf_thres"] ? car["conf_thres"].as<float>() : 0.1f;
        o.car.iou_thres = car["iou_thres"] ? car["iou_thres"].as<float>() : 0.5f;
        o.car.max_det = car["max_det"] ? car["max_det"].as<int>() : 14;
        o.car.img_size = car["img_size"] ? car["img_size"].as<int>() : 640;
      }
      if (auto a = d["armor"]) {
        o.armor.conf_thres = a["conf_thres"] ? a["conf_thres"].as<float>() : 0.5f;
        o.armor.iou_thres = a["iou_thres"] ? a["iou_thres"].as<float>() : 0.2f;
        o.armor.max_det = a["max_det"] ? a["max_det"].as<int>() : 10;
        o.armor.img_size = a["img_size"] ? a["img_size"].as<int>() : 640;
      }
      if (d["canvas_size"] && d["canvas_size"].size() >= 2)
        o.canvas_size = cv::Size(d["canvas_size"][0].as<int>(), d["canvas_size"][1].as<int>());
      o.canvas_padding = d["canvas_padding"] ? d["canvas_padding"].as<int>() : 10;
    }
    if (auto p = root["paths"]) {
      o.project_root = S(p["project_root"], "..");
      if (auto cal = p["calibration"]) {
        o.calib_red = S(cal["red"]); o.calib_blue = S(cal["blue"]);
        o.calib_red_yaml = S(cal["red_yaml"]); o.calib_blue_yaml = S(cal["blue_yaml"]);
      }
      if (auto m = p["map_images"]) {
        o.map_red = S(m["red"]); o.map_blue = S(m["blue"]);
        o.map_mask = S(m["mask"]); o.map_backup = S(m["backup"]);
      }
      if (auto md = p["models"]) {
        o.car_engine = S(md["car_engine"]); o.armor_engine = S(md["armor_engine"]);
        o.car_onnx = S(md["car_onnx"]); o.armor_onnx = S(md["armor_onnx"]);
      }
      if (auto n = p["class_names"]) {
        o.car_names_yaml = S(n["car"]); o.armor_names_yaml = S(n["armor"]);
      }
    }
    if (auto b = root["bench"]) {
      o.bench_warmup = b["warmup_frames"] ? b["warmup_frames"].as<int>() : 10;
      o.bench_measure = b["measure_frames"] ? b["measure_frames"].as<int>() : 100;
      o.bench_report_path = S(b["report_path"], o.bench_report_path);
    }
    return true;
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
}
std::vector<std::string> LoadClassNames(const std::string& path) {
  std::vector<std::string> names;
  try {
    auto root = YAML::LoadFile(path);
    auto n = root["names"];
    if (!n) return names;
    if (n.IsSequence()) for (const auto& i : n) names.push_back(i.as<std::string>());
    else if (n.IsMap()) {
      std::map<int, std::string> tmp;
      for (auto it = n.begin(); it != n.end(); ++it) tmp[it->first.as<int>()] = it->second.as<std::string>();
      for (auto& kv : tmp) names.push_back(kv.second);
    }
  } catch (const std::exception& e) {
    std::cerr << "LoadClassNames: " << e.what() << "\n";
  }
  return names;
}
}  // namespace radar
