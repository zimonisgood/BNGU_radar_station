#include "radar/geo/calibration.hpp"
#include <fstream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
namespace radar {
namespace {
bool ReadNpy(const std::string& path, cv::Mat& m0, cv::Mat& m1, std::string* err) {
  std::ifstream f(path, std::ios::binary);
  if (!f) { if (err) *err = "open npy fail"; return false; }
  char magic[6]; f.read(magic, 6);
  if (std::string(magic, 6) != "\x93NUMPY") { if (err) *err = "not npy"; return false; }
  uint8_t major=0, minor=0; f.read((char*)&major,1); f.read((char*)&minor,1);
  uint32_t hl=0;
  if (major==1) { uint16_t x=0; f.read((char*)&x,2); hl=x; } else f.read((char*)&hl,4);
  std::string header(hl,'\0'); f.read(&header[0], hl);
  if (header.find("(2, 3, 3)")==std::string::npos && header.find("(2,3,3)")==std::string::npos) {
    if (err) *err = "shape!=(2,3,3)"; return false;
  }
  std::vector<double> data(18);
  f.read((char*)data.data(), (std::streamsize)(18*sizeof(double)));
  if (!f) { if (err) *err="npy payload"; return false; }
  m0=cv::Mat(3,3,CV_64F); m1=cv::Mat(3,3,CV_64F);
  for (int i=0;i<9;++i) m0.at<double>(i/3,i%3)=data[i];
  for (int i=0;i<9;++i) m1.at<double>(i/3,i%3)=data[9+i];
  return true;
}
cv::Mat Load3x3(const YAML::Node& n) {
  cv::Mat m(3,3,CV_64F);
  for (int i=0;i<3;++i) for (int j=0;j<3;++j) m.at<double>(i,j)=n[i][j].as<double>();
  return m;
}
}  // namespace
bool LoadCalibrationNpy(const std::string& path, Calibration& out, std::string* err) {
  return ReadNpy(path, out.M_ground, out.M_height_r, err);
}
bool LoadCalibrationYaml(const std::string& path, Calibration& out, std::string* err) {
  try {
    auto root = YAML::LoadFile(path);
    if (!root["M_ground"] || !root["M_height_r"]) { if (err) *err="missing keys"; return false; }
    out.M_ground = Load3x3(root["M_ground"]);
    out.M_height_r = Load3x3(root["M_height_r"]);
    return true;
  } catch (const std::exception& e) { if (err) *err=e.what(); return false; }
}
bool LoadCalibrationAuto(const std::string& yaml_path, const std::string& npy_path, Calibration& out, std::string* err) {
  if (!yaml_path.empty()) {
    std::ifstream f(yaml_path);
    if (f.good() && LoadCalibrationYaml(yaml_path, out, err)) return true;
  }
  return LoadCalibrationNpy(npy_path, out, err);
}
MapPoint MapCameraPoint(const cv::Point2f& camera_pt, const Calibration& calib, const cv::Mat& mask) {
  MapPoint mp;
  std::vector<cv::Point2f> src{camera_pt}, dst;
  cv::perspectiveTransform(src, dst, calib.M_ground);
  int x=std::max(0,(int)dst[0].x), y=std::max(0,(int)dst[0].y);
  if (!mask.empty()) {
    x=std::min(x,mask.cols-1); y=std::min(y,mask.rows-1);
    auto c=mask.at<cv::Vec3b>(y,x);
    if (!(c[0]==0 && c[1]==0 && c[2]==0)) {
      cv::perspectiveTransform(src, dst, calib.M_height_r);
      x=std::max(0,(int)dst[0].x); y=std::max(0,(int)dst[0].y);
      x=std::min(x,mask.cols-1); y=std::min(y,mask.rows-1);
      mp.used_height = true;
    }
  }
  mp.x=x; mp.y=y; return mp;
}
cv::Point2f ProjectToMapDisplay(const std::string& state, cv::Point2f xy, cv::Size map_size) {
  if (state=="R") return {(float)map_size.width - xy.y, xy.x};
  return {xy.y, (float)map_size.height - xy.x};
}
}  // namespace radar
