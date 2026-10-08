# PFA Radar C++ MVP

Linux rewrite of the radar core pipeline:

**camera → dual YOLO → perspective map → bench**

## Features

- Frame sources: test image/video, USB camera, Hikvision MVS (`RADAR_WITH_MVS`)
- Detectors: TensorRT (`.engine`) when available; OpenCV DNN ONNX fallback
- Multi-car ROI canvas packing (same idea as Python `pack_rois_to_canvas`)
- Calibration load from `.npy` or YAML; ground / height homography + mask
- Perf timers and `--bench` report

## Build

```bash
cd cpp
cmake -S . -B build -DRADAR_WITH_TENSORRT=ON -DRADAR_WITH_MVS=ON
cmake --build build -j
```

If TensorRT/CUDA are missing, CMake falls back to OpenCV DNN only.

Dependencies: OpenCV 4, yaml-cpp, optional CUDA+TensorRT, optional Hikvision MVS (`/opt/MVS`).

## Config

Edit `configs/config.yaml`. Paths are relative to `paths.project_root` (default `..` = repo root).

Export calibration YAML from Python `.npy`:

```bash
python3 tools/export_calib_to_yaml.py ../arrays_test_blue.npy -o configs/calib_blue.yaml
python3 tools/export_calib_to_yaml.py ../arrays_test_red.npy -o configs/calib_red.yaml
```

## Run

```bash
# test image pipeline
./build/radar_main configs/config.yaml

# benchmark
./build/radar_main configs/config.yaml --bench

# camera preview
./build/hik_viewer configs/config.yaml hik

# single-model bench
./build/yolo_bench configs/config.yaml
```

For production: `camera_mode: hik`, `infer_backend: tensorrt`, engines under `models/`.

**Note:** stock `models/*.onnx` may be FP16. OpenCV DNN needs FP32 (`models/*_fp32.onnx`). Prefer TensorRT on GPU machines.

## Phase 5+

See `tools/ROADMAP_PHASE5.md` (filters, blind-zone, serial, double vulnerability, recording).
