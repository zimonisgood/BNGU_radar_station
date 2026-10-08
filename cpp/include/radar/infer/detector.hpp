#pragma once
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "radar/config/config.hpp"
namespace radar {
struct Detection {
  std::string cls;
  int cls_id = -1;
  cv::Rect xywh;
  float conf = 0.f;
};
struct LetterboxInfo {
  float ratio = 1.f;
  float pad_w = 0.f, pad_h = 0.f;
  int new_unpad_w = 0, new_unpad_h = 0;
  int out_w = 640, out_h = 640;
};
cv::Mat Letterbox(const cv::Mat& bgr, int new_shape, LetterboxInfo& info, bool auto_mode = false, int stride = 32);
void ScaleBoxes(const LetterboxInfo& info, const cv::Size& orig, std::vector<cv::Rect2f>& boxes_xyxy);
std::vector<Detection> NmsYoloV5(const float* pred, int num_pred, int num_attrs,
                                 float conf_thres, float iou_thres, int max_det,
                                 const std::vector<std::string>& names, const LetterboxInfo& lb,
                                 const cv::Size& orig);
class Detector {
 public:
  virtual ~Detector() = default;
  virtual bool Init(const std::string& weights, const std::vector<std::string>& names,
                    const DetectionConfig& cfg, std::string* err) = 0;
  virtual std::vector<Detection> Predict(const cv::Mat& bgr) = 0;
  virtual std::string BackendName() const = 0;
};
std::unique_ptr<Detector> CreateDetector(const std::string& backend_pref, std::string* chosen,
                                         std::string* err);
}  // namespace radar
