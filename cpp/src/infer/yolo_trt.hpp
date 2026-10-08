#pragma once
#include "radar/infer/detector.hpp"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/imgproc.hpp>
namespace radar {
class TrtLogger : public nvinfer1::ILogger {
  void log(Severity s, const char* msg) noexcept override {
    if (s <= Severity::kWARNING) std::cerr << "[TRT] " << msg << "\n";
  }
};
struct TrtDel { template <class T> void operator()(T* p) const { delete p; } };
class YoloTrt : public Detector {
 public:
  ~YoloTrt() override { Free(); }
  bool Init(const std::string& weights, const std::vector<std::string>& names, const DetectionConfig& cfg,
            std::string* err) override {
    names_ = names; cfg_ = cfg;
    std::ifstream f(weights, std::ios::binary);
    if (!f) { if (err) *err = "cannot open engine"; return false; }
    f.seekg(0, std::ios::end); size_t sz = (size_t)f.tellg(); f.seekg(0);
    std::vector<char> blob(sz); f.read(blob.data(), (std::streamsize)sz);
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) { if (err) *err = "createInferRuntime"; return false; }
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), sz));
    if (!engine_) { if (err) *err = "deserializeCudaEngine"; return false; }
    context_.reset(engine_->createExecutionContext());
    if (!context_) { if (err) *err = "createExecutionContext"; return false; }
#if defined(NV_TENSORRT_MAJOR) && (NV_TENSORRT_MAJOR >= 10)
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char* n = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(n) == nvinfer1::TensorIOMode::kINPUT) in_name_ = n;
      else out_name_ = n;
    }
#else
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
      if (engine_->bindingIsInput(i)) { in_idx_ = i; in_name_ = engine_->getBindingName(i); }
      else { out_idx_ = i; out_name_ = engine_->getBindingName(i); }
    }
#endif
    const int img = cfg_.img_size, nc = (int)names_.size();
    in_n_ = (size_t)3 * img * img; out_n_ = (size_t)25200 * (5 + nc);
    hin_.assign(in_n_, 0); hout_.assign(out_n_, 0);
    if (cudaMalloc(&din_, in_n_ * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&dout_, out_n_ * sizeof(float)) != cudaSuccess ||
        cudaStreamCreate(&stream_) != cudaSuccess) {
      if (err) *err = "cuda alloc"; Free(); return false;
    }
    return true;
  }
  std::vector<Detection> Predict(const cv::Mat& bgr) override {
    LetterboxInfo lb;
    cv::Mat lbimg = Letterbox(bgr, cfg_.img_size, lb, false);
    cv::Mat rgb; cv::cvtColor(lbimg, rgb, cv::COLOR_BGR2RGB);
    const int h = cfg_.img_size, w = cfg_.img_size;
    for (int c = 0; c < 3; ++c)
      for (int y = 0; y < h; ++y) {
        auto* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < w; ++x) hin_[(size_t)c * h * w + (size_t)y * w + x] = row[x][c] / 255.f;
      }
    cudaMemcpyAsync(din_, hin_.data(), in_n_ * sizeof(float), cudaMemcpyHostToDevice, stream_);
#if defined(NV_TENSORRT_MAJOR) && (NV_TENSORRT_MAJOR >= 10)
    context_->setTensorAddress(in_name_.c_str(), din_);
    context_->setTensorAddress(out_name_.c_str(), dout_);
    context_->enqueueV3(stream_);
#else
    void* bindings[2] = {}; bindings[in_idx_] = din_; bindings[out_idx_] = dout_;
    context_->enqueueV2(bindings, stream_, nullptr);
#endif
    cudaMemcpyAsync(hout_.data(), dout_, out_n_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    const int nc = (int)names_.size(), na = 5 + nc, np = (int)(out_n_ / na);
    return NmsYoloV5(hout_.data(), np, na, cfg_.conf_thres, cfg_.iou_thres, cfg_.max_det, names_, lb, bgr.size());
  }
  std::string BackendName() const override { return "tensorrt"; }
 private:
  void Free() {
    if (stream_) { cudaStreamDestroy(stream_); stream_ = nullptr; }
    if (din_) { cudaFree(din_); din_ = nullptr; }
    if (dout_) { cudaFree(dout_); dout_ = nullptr; }
  }
  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDel> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDel> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDel> context_;
  std::string in_name_ = "images", out_name_ = "output0";
  int in_idx_ = 0, out_idx_ = 1;
  std::vector<std::string> names_; DetectionConfig cfg_;
  std::vector<float> hin_, hout_; size_t in_n_ = 0, out_n_ = 0;
  void* din_ = nullptr, *dout_ = nullptr; cudaStream_t stream_ = nullptr;
};
}  // namespace radar
