#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_pcd.py — 订阅 ROS 2 的 PointCloud2，按帧保存为 PCD，并可合并成一个 PCD。

推荐用法（wrapper 会自动 source ROS 和工作空间）:
  ./scripts/record_pcd.sh
  ./scripts/record_pcd.sh --config scripts/record_robot.yaml
  ./scripts/record_pcd.sh --duration 10 --mode merged

直接调用 Python（必须用系统 Python，避免 conda 覆盖 ROS 的 Python）:
  /usr/bin/python3 scripts/record_pcd.py --config scripts/record_pcd.yaml

配置文件:
  默认读取脚本同目录的 record_pcd.yaml；
  --config 可指定其他 YAML；
  命令行参数优先级高于配置文件。

输出:
  <output-dir>/
    frames/frame_000000_<sec>_<nsec>.pcd   # --mode frames/both
    merged.pcd                             # --mode merged/both
    metadata.json                          # 帧号、时间戳、frame_id、点数

默认字段: x y z intensity
  --keep-extra 会额外保存 tag line（如果消息里有）
  --keep-nans  保留 NaN/Inf（默认跳过）
  --ascii      输出 ASCII PCD（默认 binary）

注意:
  --mode merged/both 会在内存里累积所有点；长时间录制建议用
  --mode frames，或设置 --max-frames 限制长度。
"""

import argparse
import json
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


# ---------------------------------------------------------------------------
# PCD 字段定义
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PcdField:
    name: str
    size: int          # 字节数
    type_char: str     # PCD TYPE: F=float, U=unsigned, I=signed
    count: int         # COUNT
    dtype: str         # numpy dtype（小端）


FIELD_XYZ = [
    PcdField("x", 4, "F", 1, "<f4"),
    PcdField("y", 4, "F", 1, "<f4"),
    PcdField("z", 4, "F", 1, "<f4"),
]
FIELD_INTENSITY = PcdField("intensity", 4, "F", 1, "<f4")
FIELD_TAG = PcdField("tag", 1, "U", 1, "u1")
FIELD_LINE = PcdField("line", 2, "U", 1, "<u2")


def build_fields(include_intensity: bool, extra_names):
    """按 PCD FIELDS 顺序返回字段列表。"""
    fields = list(FIELD_XYZ)
    if include_intensity:
        fields.append(FIELD_INTENSITY)
    for name in extra_names:
        if name == "tag":
            fields.append(FIELD_TAG)
        elif name == "line":
            fields.append(FIELD_LINE)
    return fields


def build_array(points, fields):
    """把 {字段名: 一维数组} 转成 PCD 需要的结构化 numpy 数组。"""
    n = len(points["x"])
    dtype = np.dtype([(f.name, f.dtype) for f in fields])
    arr = np.empty(n, dtype=dtype)
    for f in fields:
        arr[f.name] = points[f.name]
    return np.ascontiguousarray(arr)


def write_pcd(path: Path, arr: np.ndarray, fields, binary: bool):
    """写 PCD（binary 或 ascii）。"""
    n = len(arr)
    header_lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS " + " ".join(f.name for f in fields),
        "SIZE " + " ".join(str(f.size) for f in fields),
        "TYPE " + " ".join(f.type_char for f in fields),
        "COUNT " + " ".join(str(f.count) for f in fields),
        f"WIDTH {n}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {n}",
        "DATA " + ("binary" if binary else "ascii"),
        "",
    ]
    header = "\n".join(header_lines).encode("ascii")

    if binary:
        with open(path, "wb") as f:
            f.write(header)
            arr.tofile(f)
    else:
        # ASCII：按列写，浮点 %f，整数 %d
        fmt = " ".join("%.6f" if f.type_char == "F" else "%d" for f in fields)
        cols = [arr[f.name] for f in fields]
        data = np.column_stack(cols)
        with open(path, "w", encoding="ascii") as f:
            f.write(header.decode("ascii"))
            np.savetxt(f, data, fmt=fmt)


# ---------------------------------------------------------------------------
# 从 PointCloud2 提取点
# ---------------------------------------------------------------------------

def extract_points(msg: PointCloud2, include_intensity: bool,
                   keep_extra: bool, keep_nans: bool):
    """
    返回 (points_dict, extra_names)。
    points_dict 至少包含 x/y/z；include_intensity=True 时包含 intensity。
    """
    available = {f.name for f in msg.fields}
    missing = [n for n in ("x", "y", "z") if n not in available]
    if missing:
        raise RuntimeError(
            f"点云缺少字段 {missing}；实际字段: {sorted(available)}"
        )

    names = ["x", "y", "z"]
    extra_names = []
    if include_intensity and "intensity" in available:
        names.append("intensity")
    if keep_extra:
        for name in ("tag", "line"):
            if name in available:
                names.append(name)
                extra_names.append(name)

    # sensor_msgs_py 会处理字段偏移、point_step、endianness 和 NaN
    data = pc2.read_points(
        msg,
        field_names=names,
        skip_nans=not keep_nans,
    )

    points = {name: data[name] for name in names}
    if include_intensity and "intensity" not in points:
        # 消息里没有 intensity 时补 0，保持 PCD 字段固定
        points["intensity"] = np.zeros(len(data), dtype=np.float32)
    return points, extra_names


# ---------------------------------------------------------------------------
# 录制节点
# ---------------------------------------------------------------------------

class PcdRecorder(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.args = args
        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.frames_dir = self.output_dir / "frames"
        if args.mode in ("frames", "both"):
            self.frames_dir.mkdir(parents=True, exist_ok=True)

        self.include_intensity = not args.no_intensity
        self.frame_idx = 0
        self.total_points = 0
        self.frames = []
        self.merged_chunks = []
        self.extra_names = None
        self.schema_fields = None
        self.stop_requested = False
        self.start_monotonic = time.monotonic()
        self.start_wall = datetime.now()
        self.last_save_monotonic = 0.0

        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        if args.qos == "sensor":
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            qos.reliability = ReliabilityPolicy.RELIABLE

        self.sub = self.create_subscription(
            PointCloud2, args.topic, self.cloud_callback, qos
        )

        self.get_logger().info(
            f"PCD 录制启动: topic={args.topic} qos={args.qos} "
            f"mode={args.mode} output={self.output_dir}"
        )
        self.get_logger().info(
            "字段: x y z"
            + (" intensity" if self.include_intensity else "")
            + (" + tag/line(若存在)" if args.keep_extra else "")
        )
        if args.duration > 0:
            self.get_logger().info(f"时长限制: {args.duration:.1f} s")
        if args.max_frames > 0:
            self.get_logger().info(f"帧数限制: {args.max_frames}")
        self.get_logger().info("按 Ctrl+C 结束并写出 metadata.json（merged 模式还会写 merged.pcd）")

    # ------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2):
        if self.stop_requested:
            return

        now = time.monotonic()
        if self.args.duration > 0 and (now - self.start_monotonic) >= self.args.duration:
            self.stop_requested = True
            return
        if self.args.min_interval > 0 and (now - self.last_save_monotonic) < self.args.min_interval:
            return

        try:
            points, extra_names = extract_points(
                msg,
                include_intensity=self.include_intensity,
                keep_extra=self.args.keep_extra,
                keep_nans=self.args.keep_nans,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"解析点云失败: {exc}")
            return

        n = len(points["x"])
        if n == 0:
            self.get_logger().warn("收到空点云，跳过")
            return

        # 固定 schema：以第一帧为准，避免合并时字段顺序不一致
        if self.extra_names is None:
            self.extra_names = extra_names
            self.schema_fields = build_fields(self.include_intensity, self.extra_names)
        elif extra_names != self.extra_names:
            self.get_logger().warn(
                f"点云字段与首帧不一致，忽略本次额外字段: {extra_names}"
            )

        fields = self.schema_fields
        arr = build_array(points, fields)

        stamp = msg.header.stamp
        stamp_sec = int(stamp.sec)
        stamp_nsec = int(stamp.nanosec)
        frame_name = f"frame_{self.frame_idx:06d}_{stamp_sec:010d}_{stamp_nsec:09d}.pcd"

        if self.args.mode in ("frames", "both"):
            write_pcd(
                self.frames_dir / frame_name,
                arr,
                fields,
                binary=not self.args.ascii,
            )

        if self.args.mode in ("merged", "both"):
            self.merged_chunks.append(arr)

        self.frames.append({
            "index": self.frame_idx,
            "stamp": {"sec": stamp_sec, "nanosec": stamp_nsec},
            "frame_id": msg.header.frame_id,
            "points": n,
            "file": frame_name if self.args.mode in ("frames", "both") else None,
        })
        self.frame_idx += 1
        self.total_points += n
        self.last_save_monotonic = now

        if self.frame_idx == 1 or self.frame_idx % 10 == 0:
            self.get_logger().info(
                f"已保存 {self.frame_idx} 帧, 本帧 {n} 点, 累计 {self.total_points} 点"
            )

        if self.args.max_frames > 0 and self.frame_idx >= self.args.max_frames:
            self.stop_requested = True

    # ------------------------------------------------------------------
    def finalize(self):
        merged_file = None
        merged_points = 0

        if self.args.mode in ("merged", "both") and self.merged_chunks:
            merged_arr = np.concatenate(self.merged_chunks)
            merged_points = len(merged_arr)
            merged_path = self.output_dir / self.args.merged_name
            write_pcd(
                merged_path,
                np.ascontiguousarray(merged_arr),
                self.schema_fields,
                binary=not self.args.ascii,
            )
            merged_file = merged_path.name
            self.get_logger().info(
                f"合并 PCD 已写出: {merged_path} ({merged_points} 点)"
            )
        elif self.args.mode in ("merged", "both"):
            self.get_logger().warn("没有收到任何点云，未写出 merged.pcd")

        metadata = {
            "config_file": getattr(self.args, "config_used", None),
            "topic": self.args.topic,
            "qos": self.args.qos,
            "mode": self.args.mode,
            "binary": not self.args.ascii,
            "include_intensity": self.include_intensity,
            "keep_extra": self.args.keep_extra,
            "extra_fields": self.extra_names or [],
            "fields": [
                {"name": f.name, "size": f.size, "type": f.type_char, "count": f.count}
                for f in (self.schema_fields or [])
            ],
            "frame_count": self.frame_idx,
            "total_points": self.total_points,
            "merged_file": merged_file,
            "merged_points": merged_points,
            "started_at": self.start_wall.isoformat(timespec="seconds"),
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "output_dir": str(self.output_dir),
            "frames": self.frames,
        }
        meta_path = self.output_dir / "metadata.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
        self.get_logger().info(
            f"录制结束: {self.frame_idx} 帧, {self.total_points} 点, 元数据: {meta_path}"
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def default_output_dir():
    return f"pcd_recordings/{datetime.now().strftime('%Y%m%d_%H%M%S')}"


DEFAULT_CONFIG_PATH = Path(__file__).with_name("record_pcd.yaml")
CONFIG_KEYS = {
    "topic", "output_dir", "mode", "max_frames", "duration",
    "min_interval", "qos", "keep_nans", "keep_extra", "ascii",
    "no_intensity", "merged_name", "node_name",
}


def load_config(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"配置文件不存在: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"配置文件必须是 YAML 字典: {path}")
    unknown = set(data) - CONFIG_KEYS
    if unknown:
        raise ValueError(f"配置文件包含未知字段: {sorted(unknown)}")

    # YAML 里可能写成字符串，这里统一转成脚本期望的类型
    if "duration" in data:
        data["duration"] = float(data["duration"])
    if "min_interval" in data:
        data["min_interval"] = float(data["min_interval"])
    if "max_frames" in data:
        data["max_frames"] = int(data["max_frames"])
    for key in ("keep_nans", "keep_extra", "ascii", "no_intensity"):
        if key in data:
            data[key] = bool(data[key])
    return data


def parse_args(argv=None):
    # 先单独解析 --config，读取 YAML，再用它作为 argparse 的默认值
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", default=None)
    pre_args, _ = pre_parser.parse_known_args(argv)

    if pre_args.config:
        config_path = Path(pre_args.config).expanduser()
    elif DEFAULT_CONFIG_PATH.exists():
        config_path = DEFAULT_CONFIG_PATH
    else:
        config_path = None

    try:
        config = load_config(config_path) if config_path else {}
    except Exception as exc:  # noqa: BLE001
        pre_parser.error(f"读取配置文件失败: {exc}")

    parser = argparse.ArgumentParser(
        description="订阅 ROS 2 PointCloud2，保存为 PCD（支持逐帧和合并）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--config", default=str(config_path) if config_path else None,
        help="YAML 配置文件；默认读取脚本同目录的 record_pcd.yaml")
    parser.add_argument("--topic", default="/livox/lidar",
                        help="PointCloud2 话题")
    parser.add_argument("-o", "--output-dir", default=None,
                        help="输出目录；默认 pcd_recordings/<时间戳>")
    parser.add_argument("--mode", choices=["frames", "merged", "both"],
                        default="both",
                        help="frames=每帧一个 PCD；merged=最后合并一个；both=两者")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="最多保存多少帧；0=不限制")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="录制时长（秒）；0=不限制，Ctrl+C 结束")
    parser.add_argument("--min-interval", type=float, default=0.0,
                        help="两帧之间至少间隔多少秒；0=每帧都存")
    parser.add_argument("--qos", choices=["reliable", "sensor"],
                        default="reliable",
                        help="reliable=Livox 驱动默认；sensor=best-effort")
    parser.add_argument("--keep-nans", action="store_true",
                        help="保留 NaN/Inf（默认跳过）")
    parser.add_argument("--keep-extra", action="store_true",
                        help="额外保存 tag/line 字段（如果消息里有）")
    parser.add_argument("--ascii", action="store_true",
                        help="输出 ASCII PCD（默认 binary）")
    parser.add_argument("--no-intensity", action="store_true",
                        help="不保存 intensity 字段")
    parser.add_argument("--merged-name", default="merged.pcd",
                        help="合并 PCD 的文件名")
    parser.add_argument("--node-name", default="pcd_recorder",
                        help="ROS 节点名")

    # 配置文件作为默认值；命令行参数会覆盖它
    parser.set_defaults(**config)
    args = parser.parse_args(argv)
    args.config_used = args.config

    if args.output_dir is None:
        args.output_dir = default_output_dir()
    if args.mode not in ("frames", "merged", "both"):
        parser.error(f"--mode 只能是 frames/merged/both，当前: {args.mode}")
    if args.qos not in ("reliable", "sensor"):
        parser.error(f"--qos 只能是 reliable/sensor，当前: {args.qos}")
    if args.max_frames < 0:
        parser.error("--max-frames 不能为负数")
    if args.duration < 0:
        parser.error("--duration 不能为负数")
    if args.min_interval < 0:
        parser.error("--min-interval 不能为负数")
    return args


def main(argv=None):
    args = parse_args(argv)
    if getattr(args, "config_used", None):
        print(f"使用配置文件: {args.config_used}")

    # 这个环境里 conda 的 Python 可能覆盖 ROS 2 的 Python；给出明确提示
    if "miniconda" in sys.executable or "conda" in sys.executable:
        print(
            "警告: 当前 Python 可能不是 ROS 2 使用的 Python。\n"
            "      建议: source /opt/ros/jazzy/setup.bash && "
            "/usr/bin/python3 scripts/record_pcd.py ...",
            file=sys.stderr,
        )

    rclpy.init()
    recorder = PcdRecorder(args)
    try:
        while rclpy.ok() and not recorder.stop_requested:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if (args.duration > 0 and
                    time.monotonic() - recorder.start_monotonic >= args.duration):
                recorder.stop_requested = True
    except KeyboardInterrupt:
        recorder.get_logger().info("收到 Ctrl+C，正在收尾...")
    finally:
        recorder.finalize()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
