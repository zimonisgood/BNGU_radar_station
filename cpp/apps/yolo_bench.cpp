#include "radar/config/config.hpp"
#include "radar/infer/detector.hpp"
#include "radar/util/timer.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
int main(int argc, char** argv) {
  std::string cfg_path = argc>1?argv[1]:"configs/config.yaml";
  radar::AppConfig cfg; std::string err;
  if (!radar::LoadConfig(cfg_path, cfg, &err)) { std::cerr<<err<<"\n"; return 1; }
  auto names = radar::LoadClassNames(cfg.Resolve(cfg.car_names_yaml));
  std::string chosen;
  auto det = radar::CreateDetector(cfg.infer_backend, &chosen, &err);
  if (!det) { std::cerr<<err<<"\n"; return 1; }
  std::string weights = (chosen=="tensorrt") ? cfg.Resolve(cfg.car_engine) : cfg.Resolve(cfg.car_onnx);
  if (!det->Init(weights, names, cfg.car, &err)) { std::cerr<<err<<"\n"; return 1; }
  std::cout << "backend=" << det->BackendName() << " weights=" << weights << "\n";
  cv::Mat img = cv::imread(cfg.Resolve(cfg.test_image_path));
  if (img.empty()) { std::cerr << "empty image\n"; return 1; }
  radar::PerfStats stats;
  for (int i=0;i<cfg.bench_warmup;++i) (void)det->Predict(img);
  auto t0=std::chrono::steady_clock::now();
  for (int i=0;i<cfg.bench_measure;++i) {
    double ms=0; { radar::ScopedTimer t(&ms); auto d=det->Predict(img); if(i==0) std::cout<<"dets="<<d.size()<<"\n"; }
    stats.Add("infer", ms); stats.Tick();
  }
  double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
  std::cout << stats.Summary(wall);
  return 0;
}
