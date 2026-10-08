#pragma once
#include <string>
#include <opencv2/core.hpp>
namespace radar {
struct Calibration { cv::Mat M_ground; cv::Mat M_height_r; };
bool LoadCalibrationNpy(const std::string& path, Calibration& out, std::string* err = nullptr);
bool LoadCalibrationYaml(const std::string& path, Calibration& out, std::string* err = nullptr);
bool LoadCalibrationAuto(const std::string& yaml_path, const std::string& npy_path, Calibration& out,
                         std::string* err = nullptr);
struct MapPoint { int x = 0, y = 0; bool used_height = false; };
MapPoint MapCameraPoint(const cv::Point2f& pt, const Calibration& calib, const cv::Mat& mask);
cv::Point2f ProjectToMapDisplay(const std::string& state, cv::Point2f xy, cv::Size map_size);
}  // namespace radar
