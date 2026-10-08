#include "radar/camera/frame_source.hpp"

#ifdef RADAR_WITH_MVS

#include <cstring>
#include <iostream>
#include <vector>

#include <MvCameraControl.h>
#include <opencv2/imgproc.hpp>

namespace radar {

namespace {

// Hikvision PixelType.h common values (same as Python hik_camera.image_control)
constexpr unsigned int kMono8 = 0x01080001;       // 17301505
constexpr unsigned int kBayerRG8 = 0x01080009;    // 17301513
constexpr unsigned int kBayerGB8 = 0x0108000A;    // 17301514
constexpr unsigned int kBayerGR8 = 0x01080008;    // 17301512
constexpr unsigned int kBayerBG8 = 0x0108000B;    // 17301515
constexpr unsigned int kRGB8 = 0x02180014;        // 35127316
constexpr unsigned int kBGR8 = 0x02180015;        // 35127317
constexpr unsigned int kYUV422_YUYV = 0x0210003B; // 34603039 (approx / legacy)

std::string SerialOf(MV_CC_DEVICE_INFO* info) {
  if (!info) return {};
  char buf[64] = {0};
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    std::memcpy(buf, info->SpecialInfo.stGigEInfo.chSerialNumber,
                sizeof(info->SpecialInfo.stGigEInfo.chSerialNumber));
  } else if (info->nTLayerType == MV_USB_DEVICE) {
    std::memcpy(buf, info->SpecialInfo.stUsb3VInfo.chSerialNumber,
                sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
  }
  return std::string(buf);
}

int SelectIdx(MV_CC_DEVICE_INFO_LIST& list, const std::string& want) {
  if (!list.nDeviceNum) return -1;
  if (want.empty()) return 0;
  for (unsigned i = 0; i < list.nDeviceNum; ++i) {
    if (SerialOf(list.pDeviceInfo[i]) == want) return static_cast<int>(i);
  }
  return -1;
}

}  // namespace

HikCamera::HikCamera(std::string serial, double exposure_us, double gain)
    : serial_(std::move(serial)), exposure_us_(exposure_us), gain_(gain) {}

HikCamera::~HikCamera() { Stop(); }

bool HikCamera::Start() {
  if (!OpenDevice()) return false;
  running_.store(true);
  worker_ = std::thread([this] { Loop(); });
  return true;
}

void HikCamera::Stop() {
  // 先置位再 StopGrabbing，打断 GetOneFrameTimeout，避免 join 永久卡住（SDK 常见坑）
  running_.store(false);
  if (handle_) {
    MV_CC_StopGrabbing(handle_);
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  CloseDevice();
}

bool HikCamera::GetLatest(cv::Mat& out_bgr) {
  std::lock_guard<std::mutex> lk(mu_);
  if (latest_.empty()) return false;
  out_bgr = latest_.clone();
  return true;
}

bool HikCamera::OpenDevice() {
  // MVS 4.x 官方要求：枚举前先 Initialize（可重复调用）
  int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] MV_CC_Initialize failed ret=0x" << std::hex << ret << std::dec << "\n";
    return false;
  }

  MV_CC_DEVICE_INFO_LIST list{};
  // 与 Python hik_camera.enum_devices 对齐：网口+USB+未知+1394+CameraLink
  const unsigned int tlayer =
      MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE;
  ret = MV_CC_EnumDevices(tlayer, &list);
  if (ret != MV_OK || !list.nDeviceNum) {
    std::cerr << "[HikCamera] EnumDevices failed ret=0x" << std::hex << ret << std::dec
              << " count=" << list.nDeviceNum << "\n"
              << "  hint: SDK OK but no camera found. Check:\n"
              << "    - GigE: eth cable plugged? `ip -br link` show NO-CARRIER?\n"
              << "    - USB3: `lsusb` show Hikvision?\n"
              << "    - power on camera; close MVS Client if open\n";
    return false;
  }
  std::cout << "[HikCamera] found " << list.nDeviceNum << " device(s):\n";
  for (unsigned i = 0; i < list.nDeviceNum; ++i) {
    std::cout << "  [" << i << "] serial=" << SerialOf(list.pDeviceInfo[i]) << "\n";
  }

  const int idx = SelectIdx(list, serial_);
  if (idx < 0) {
    std::cerr << "[HikCamera] serial not found: '" << serial_ << "'\n";
    return false;
  }
  std::cout << "[HikCamera] open index=" << idx << " serial=" << SerialOf(list.pDeviceInfo[idx]) << "\n";

  ret = MV_CC_CreateHandle(&handle_, list.pDeviceInfo[idx]);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] CreateHandle failed 0x" << std::hex << ret << std::dec << "\n";
    return false;
  }
  ret = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] OpenDevice failed 0x" << std::hex << ret << std::dec
              << " (camera busy in MVS Client?)\n";
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  // 曝光/增益失败不致命（部分机型节点名不同）
  ret = MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_us_));
  if (ret != MV_OK) std::cerr << "[HikCamera] set ExposureTime warn 0x" << std::hex << ret << std::dec << "\n";
  ret = MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_));
  if (ret != MV_OK) std::cerr << "[HikCamera] set Gain warn 0x" << std::hex << ret << std::dec << "\n";

  MVCC_INTVALUE_EX payload{};
  ret = MV_CC_GetIntValueEx(handle_, "PayloadSize", &payload);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] Get PayloadSize failed\n";
    CloseDevice();
    return false;
  }
  payload_size_ = static_cast<unsigned int>(payload.nCurValue);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] StartGrabbing failed 0x" << std::hex << ret << std::dec << "\n";
    CloseDevice();
    return false;
  }
  std::cout << "[HikCamera] grabbing, payload=" << payload_size_ << "\n";
  return true;
}

void HikCamera::CloseDevice() {
  if (handle_) {
    // StopGrabbing 可能已在 Stop() 调过，重复调用一般可接受
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  MV_CC_Finalize();
}

cv::Mat HikCamera::ConvertFrame(const unsigned char* data, int w, int h, unsigned int pixel_type) {
  if (pixel_type == kMono8) {
    cv::Mat g(h, w, CV_8UC1, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(g, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  if (pixel_type == kBayerRG8) {
    cv::Mat b(h, w, CV_8UC1, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(b, bgr, cv::COLOR_BayerRG2BGR);
    return bgr;
  }
  if (pixel_type == kBayerGB8) {
    cv::Mat b(h, w, CV_8UC1, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(b, bgr, cv::COLOR_BayerGB2BGR);
    return bgr;
  }
  if (pixel_type == kBayerGR8) {
    cv::Mat b(h, w, CV_8UC1, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(b, bgr, cv::COLOR_BayerGR2BGR);
    return bgr;
  }
  if (pixel_type == kBayerBG8) {
    cv::Mat b(h, w, CV_8UC1, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(b, bgr, cv::COLOR_BayerBG2BGR);
    return bgr;
  }
  if (pixel_type == kRGB8) {
    cv::Mat rgb(h, w, CV_8UC3, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  if (pixel_type == kBGR8) {
    return cv::Mat(h, w, CV_8UC3, const_cast<unsigned char*>(data)).clone();
  }
  if (pixel_type == kYUV422_YUYV || pixel_type == 34603039u) {
    cv::Mat yuv(h, w, CV_8UC2, const_cast<unsigned char*>(data));
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);
    return bgr;
  }
  return {};
}

void HikCamera::Loop() {
  std::vector<unsigned char> buf(payload_size_ > 0 ? payload_size_ : 1);
  MV_FRAME_OUT_INFO_EX info{};
  int unsupported_logs = 0;
  while (running_.load()) {
    std::memset(&info, 0, sizeof(info));
    // 短超时，便于响应 Stop()
    const int ret = MV_CC_GetOneFrameTimeout(handle_, buf.data(), payload_size_, &info, 200);
    if (ret != MV_OK) continue;
    if (!running_.load()) break;

    cv::Mat f = ConvertFrame(buf.data(), static_cast<int>(info.nWidth), static_cast<int>(info.nHeight),
                             static_cast<unsigned int>(info.enPixelType));
    if (f.empty()) {
      if (unsupported_logs < 5) {
        std::cerr << "[HikCamera] unsupported pixelType=0x" << std::hex << info.enPixelType << std::dec
                  << " (" << info.enPixelType << ") size=" << info.nWidth << "x" << info.nHeight << "\n";
        ++unsupported_logs;
      }
      continue;
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      latest_ = std::move(f);
    }
  }
}

}  // namespace radar

#endif
