#include "radar/camera/frame_source.hpp"
#include "radar/config/config.hpp"
#include "radar/geo/calibration.hpp"
#include "radar/infer/detector.hpp"
#include "radar/pipeline/roi_packer.hpp"
#include "radar/util/timer.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
int main(int argc, char** argv) {
  std::string cfg_path = "configs/config.yaml";
  bool bench = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--bench") bench = true;
    else if (!a.empty() && a[0] != '-') cfg_path = a;
  }
  radar::AppConfig cfg; std::string err;
  if (!radar::LoadConfig(cfg_path, cfg, &err)) { std::cerr << err << "\n"; return 1; }
  auto src = radar::CreateFrameSource(cfg, &err);
  if (!src) { std::cerr << err << "\n"; return 1; }
  auto car_names = radar::LoadClassNames(cfg.Resolve(cfg.car_names_yaml));
  auto armor_names = radar::LoadClassNames(cfg.Resolve(cfg.armor_names_yaml));
  std::string backend;
  auto car_det = radar::CreateDetector(cfg.infer_backend, &backend, &err);
  auto armor_det = radar::CreateDetector(cfg.infer_backend, &backend, &err);
  if (!car_det || !armor_det) { std::cerr << err << "\n"; return 1; }
  bool use_trt = (backend == "tensorrt");
  if (!car_det->Init(cfg.Resolve(use_trt ? cfg.car_engine : cfg.car_onnx), car_names, cfg.car, &err) ||
      !armor_det->Init(cfg.Resolve(use_trt ? cfg.armor_engine : cfg.armor_onnx), armor_names, cfg.armor, &err)) {
    std::cerr << err << "\n"; return 1;
  }
  std::cout << "backend=" << backend << " source=" << src->Name()
            << " multi_car=" << cfg.multi_car_recognition << "\n";
  radar::Calibration calib;
  bool is_red = (cfg.state == "R");
  if (!radar::LoadCalibrationAuto(cfg.Resolve(is_red ? cfg.calib_red_yaml : cfg.calib_blue_yaml),
                                  cfg.Resolve(is_red ? cfg.calib_red : cfg.calib_blue), calib, &err)) {
    std::cerr << "calib warn: " << err << "\n";
  }
  cv::Mat map_img = cv::imread(cfg.Resolve(is_red ? cfg.map_red : cfg.map_blue));
  cv::Mat mask = cv::imread(cfg.Resolve(cfg.map_mask));
  cv::Mat map_backup = cv::imread(cfg.Resolve(cfg.map_backup));
  if (map_backup.empty()) map_backup = map_img.clone();
  radar::PerfStats stats;
  int measured = 0;
  int target = bench ? (cfg.bench_warmup + cfg.bench_measure) : 1000000;
  auto wall0 = std::chrono::steady_clock::now();
  while (measured < target) {
    cv::Mat frame; double capture_ms = 0;
    { radar::ScopedTimer t(&capture_ms);
      if (!src->GetLatest(frame) || frame.empty()) { if (cv::waitKey(5)==27) break; continue; } }
    double car_ms=0; std::vector<radar::Detection> cars;
    { radar::ScopedTimer t(&car_ms); cars = car_det->Predict(frame); }
    std::vector<cv::Mat> rois; std::vector<cv::Rect> positions;
    std::vector<std::pair<float,int>> entries;
    for (int i=0;i<(int)cars.size();++i) {
      if (cars[i].cls != "car") continue;
      cv::Rect r = cars[i].xywh & cv::Rect(0,0,frame.cols,frame.rows);
      if (r.width<=1||r.height<=1) continue;
      entries.push_back({cars[i].conf,i});
    }
    std::sort(entries.begin(), entries.end(), [](auto&a,auto&b){return a.first>b.first;});
    if (!cfg.multi_car_recognition && !entries.empty()) entries.resize(1);
    for (auto& e: entries) {
      cv::Rect r = cars[e.second].xywh & cv::Rect(0,0,frame.cols,frame.rows);
      rois.push_back(frame(r).clone()); positions.push_back(r);
    }
    cv::Mat vis=frame.clone(); cv::Mat map_vis = map_backup.empty()?cv::Mat():map_backup.clone();
    double armor_ms=0, map_ms=0;
    if (!rois.empty()) {
      auto pack = radar::PackRoisToCanvas(rois, cfg.canvas_size, cfg.canvas_padding);
      radar::ScopedTimer t(&armor_ms);
      for (size_t ci=0; ci<pack.first.size(); ++ci) {
        auto dets = armor_det->Predict(pack.first[ci]);
        for (const auto& pr : pack.second[ci]) {
          const radar::Detection* best=nullptr;
          for (const auto& d: dets) {
            float cx=d.xywh.x+d.xywh.width*0.5f, cy=d.xywh.y+d.xywh.height*0.5f;
            if (cx>=pr.rx && cx<pr.rx+pr.rw && cy>=pr.ry && cy<pr.ry+pr.rh)
              if (!best || d.conf>best->conf) best=&d;
          }
          if (!best) continue;
          const cv::Rect& pos = positions[pr.roi_idx];
          int x0=best->xywh.x-pr.rx+pos.x, y0=best->xywh.y-pr.ry+pos.y;
          int w=best->xywh.width, h=best->xywh.height;
          cv::rectangle(vis, cv::Rect(x0,y0,w,h), {255,0,255}, 2);
          cv::putText(vis, best->cls+" "+cv::format("%.2f", best->conf), {x0,std::max(0,y0-5)},
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,0,255}, 2);
          if (!calib.M_ground.empty() && !map_vis.empty()) {
            radar::ScopedTimer tm(&map_ms);
            float px=std::min(x0+0.5f*w,(float)frame.cols-1.f);
            float py=std::min(y0+1.5f*h,(float)frame.rows-1.f);
            auto mp = radar::MapCameraPoint({px,py}, calib, mask);
            auto disp = radar::ProjectToMapDisplay(cfg.state, {(float)mp.x,(float)mp.y}, cfg.map_size);
            cv::Scalar color = (!best->cls.empty() && best->cls[0]=='R') ? cv::Scalar(0,0,255) : cv::Scalar(255,0,0);
            cv::circle(map_vis, {(int)disp.x,(int)disp.y}, 12, color, -1);
            cv::putText(map_vis, best->cls, {(int)disp.x-5,(int)disp.y+5}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {255,255,255}, 2);
          }
        }
      }
    }
    for (auto& c: cars) if (c.cls=="car") cv::rectangle(vis, c.xywh, {0,255,0}, 2);
    if (!bench || measured >= cfg.bench_warmup) {
      stats.Add("capture", capture_ms); stats.Add("car", car_ms);
      stats.Add("armor", armor_ms); stats.Add("map", map_ms); stats.Tick();
    }
    ++measured;
    if (cfg.show_ui && !bench) {
      cv::imshow("radar_main", vis);
      if (!map_vis.empty()) { cv::Mat s; cv::resize(map_vis,s,{},0.35,0.35); cv::imshow("map", s); }
      int key=cv::waitKey(1); if (key==27||key=='q') break;
    }
  }
  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now()-wall0).count();
  std::cout << stats.Summary(wall);
  if (bench) {
    std::ofstream ofs(cfg.Resolve(cfg.bench_report_path));
    ofs << "# C++ Radar MVP Bench\n\n- backend: `" << backend << "`\n- source: `" << src->Name() << "`\n\n```\n"
        << stats.Summary(wall) << "```\n";
    std::cout << "wrote " << cfg.Resolve(cfg.bench_report_path) << "\n";
  }
  src->Stop();
  return 0;
}
