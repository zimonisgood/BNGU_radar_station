#pragma once
#include <utility>
#include <vector>
#include <opencv2/core.hpp>
namespace radar {
struct PackedRoi { int roi_idx = 0; int rx = 0, ry = 0, rw = 0, rh = 0; int ox = 0, oy = 0; };
std::pair<std::vector<cv::Mat>, std::vector<std::vector<PackedRoi>>>
PackRoisToCanvas(const std::vector<cv::Mat>& rois, cv::Size canvas_size = {640, 640}, int padding = 10);
}  // namespace radar
