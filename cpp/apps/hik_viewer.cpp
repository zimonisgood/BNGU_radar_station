#include "radar/camera/frame_source.hpp"
#include "radar/config/config.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
  std::string cfg_path = argc > 1 ? argv[1] : "configs/config.yaml";
  radar::AppConfig cfg;
  std::string err;
  if (!radar::LoadConfig(cfg_path, cfg, &err)) {
    std::cerr << "config: " << err << "\n";
    return 1;
  }
  if (argc > 2) cfg.camera_mode = argv[2];
  cfg.show_ui = true;

  auto src = radar::CreateFrameSource(cfg, &err);
  if (!src) {
    std::cerr << "source: " << err << "\n";
    return 1;
  }
  std::cout << "source=" << src->Name() << "  (press Esc to quit)\n";

  int empty_rounds = 0;
  auto t0 = std::chrono::steady_clock::now();
  while (true) {
    cv::Mat frame;
    if (!src->GetLatest(frame) || frame.empty()) {
      ++empty_rounds;
      if (empty_rounds == 50) {
        std::cerr << "[hik_viewer] still waiting for frames... "
                     "(check pixel format logs / MVS Client not occupying camera)\n";
      }
      const int key = cv::waitKey(10);
      if (key == 27) break;
      continue;
    }
    empty_rounds = 0;
    cv::imshow("hik_viewer", frame);
    const int key = cv::waitKey(1);
    if (key == 27) break;

    static int shown = 0;
    if (++shown == 1) {
      const auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
              .count();
      std::cout << "[hik_viewer] first frame " << frame.cols << "x" << frame.rows << " after " << ms
                << " ms\n";
    }
  }

  std::cout << "[hik_viewer] stopping...\n";
  src->Stop();
  std::cout << "[hik_viewer] stopped ok\n";
  return 0;
}
