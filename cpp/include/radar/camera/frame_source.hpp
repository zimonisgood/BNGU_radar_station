#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/core.hpp>
#include "radar/config/config.hpp"
namespace radar {
class FrameSource {
 public:
  virtual ~FrameSource() = default;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool GetLatest(cv::Mat& out_bgr) = 0;
  virtual std::string Name() const = 0;
};
class ImageSource : public FrameSource {
 public:
  explicit ImageSource(std::string image_path);
  bool Start() override; void Stop() override; bool GetLatest(cv::Mat& out_bgr) override;
  std::string Name() const override { return "image"; }
 private:
  std::string path_; cv::Mat image_;
};
class VideoSource : public FrameSource {
 public:
  VideoSource(std::string path, bool loop, double speed);
  explicit VideoSource(int camera_index);
  bool Start() override; void Stop() override; bool GetLatest(cv::Mat& out_bgr) override;
  std::string Name() const override { return camera_index_ >= 0 ? "video_usb" : "video_file"; }
 private:
  void Loop();
  std::string path_; int camera_index_ = -1; bool loop_ = false; double speed_ = 1.0;
  std::atomic<bool> running_{false}; std::thread worker_; std::mutex mu_; cv::Mat latest_;
};
#ifdef RADAR_WITH_MVS
class HikCamera : public FrameSource {
 public:
  HikCamera(std::string serial, double exposure_us, double gain);
  ~HikCamera() override;
  bool Start() override; void Stop() override; bool GetLatest(cv::Mat& out_bgr) override;
  std::string Name() const override { return "hik"; }
 private:
  void Loop(); bool OpenDevice(); void CloseDevice();
  static cv::Mat ConvertFrame(const unsigned char* data, int w, int h, unsigned int pixel_type);
  std::string serial_; double exposure_us_ = 16000.0; double gain_ = 17.9; void* handle_ = nullptr;
  std::atomic<bool> running_{false}; std::thread worker_; std::mutex mu_; cv::Mat latest_;
  unsigned int payload_size_ = 0;
};
#endif
std::unique_ptr<FrameSource> CreateFrameSource(const AppConfig& cfg, std::string* err);
}  // namespace radar
