#include "radar/camera/frame_source.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace radar {

ImageSource::ImageSource(std::string image_path) : path_(std::move(image_path)) {}

bool ImageSource::Start() {
  image_ = cv::imread(path_);
  if (image_.empty()) {
    std::cerr << "imread fail " << path_ << "\n";
    return false;
  }
  return true;
}

void ImageSource::Stop() {}

bool ImageSource::GetLatest(cv::Mat& out_bgr) {
  if (image_.empty()) return false;
  out_bgr = image_.clone();
  return true;
}

VideoSource::VideoSource(std::string path, bool loop, double speed)
    : path_(std::move(path)), loop_(loop), speed_(speed <= 0 ? 1.0 : speed) {}

VideoSource::VideoSource(int camera_index) : camera_index_(camera_index) {}

bool VideoSource::Start() {
  running_ = true;
  worker_ = std::thread([this] { Loop(); });
  return true;
}

void VideoSource::Stop() {
  running_ = false;
  if (worker_.joinable()) worker_.join();
}

bool VideoSource::GetLatest(cv::Mat& out_bgr) {
  std::lock_guard<std::mutex> lk(mu_);
  if (latest_.empty()) return false;
  out_bgr = latest_.clone();
  return true;
}

void VideoSource::Loop() {
  cv::VideoCapture cap;
  if (camera_index_ >= 0) {
    cap.open(camera_index_);
  } else {
    cap.open(path_);
  }
  if (!cap.isOpened()) {
    std::cerr << "VideoSource open fail\n";
    running_ = false;
    return;
  }
  const double fps = cap.get(cv::CAP_PROP_FPS);
  const int delay = fps > 1 ? static_cast<int>(1000.0 / (fps * speed_)) : 30;
  while (running_) {
    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
      if (loop_ && camera_index_ < 0) {
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      }
      if (camera_index_ >= 0) {
        cap.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        cap.open(camera_index_);
        continue;
      }
      break;
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      latest_ = std::move(frame);
    }
    if (delay > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  }
}

std::unique_ptr<FrameSource> CreateFrameSource(const AppConfig& cfg, std::string* err) {
  if (cfg.camera_mode == "test") {
    if (!cfg.test_video_path.empty()) {
      auto s = std::make_unique<VideoSource>(cfg.Resolve(cfg.test_video_path), cfg.test_loop, cfg.playback_speed);
      if (!s->Start()) {
        if (err) *err = "test video fail";
        return nullptr;
      }
      return s;
    }
    auto s = std::make_unique<ImageSource>(cfg.Resolve(cfg.test_image_path));
    if (!s->Start()) {
      if (err) *err = "test image fail";
      return nullptr;
    }
    return s;
  }
  if (cfg.camera_mode == "video") {
    auto s = std::make_unique<VideoSource>(0);
    if (!s->Start()) {
      if (err) *err = "usb fail";
      return nullptr;
    }
    return s;
  }
  if (cfg.camera_mode == "hik") {
#ifdef RADAR_WITH_MVS
    auto s = std::make_unique<HikCamera>(cfg.device_serial, cfg.exposure_time, cfg.gain);
    if (!s->Start()) {
      if (err) *err = "hik fail";
      return nullptr;
    }
    return s;
#else
    if (err) *err = "built without MVS";
    return nullptr;
#endif
  }
  if (err) *err = "unknown camera_mode";
  return nullptr;
}

}  // namespace radar
