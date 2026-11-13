#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_bag.py — 录制 ROS 2 bag，录完后再用 bag_to_pcd.py 转成 PCD。

推荐用法（wrapper 会自动 source ROS 和工作空间）:
  ./scripts/record_bag.sh
  ./scripts/record_bag.sh --config scripts/record_bag_robot.yaml
  ./scripts/record_bag.sh --duration 10 -o bags/quick_test

配置文件:
  默认读取脚本同目录的 record_bag.yaml；
  --config 可指定其他 YAML；
  命令行参数优先级高于配置文件。

录完之后:
  ./scripts/bag_to_pcd.sh bags/empty_scene -o pcd_recordings/empty_scene_merged.pcd

说明:
  - 本脚本调用 ROS 2 自带的 ros2 bag record；
  - duration > 0 时，到时间会自动发 SIGINT 停止并收尾；
  - duration = 0 时，一直录到 Ctrl+C。
"""

import argparse
import signal
import subprocess
import sys
from pathlib import Path

import yaml


DEFAULT_CONFIG_PATH = Path(__file__).with_name("record_bag.yaml")
CONFIG_KEYS = {
    "topic", "output_dir", "duration", "storage",
    "compression_mode", "compression_format", "node_name",
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
    if "duration" in data:
        data["duration"] = float(data["duration"])
    return data


def parse_args(argv=None):
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
        description="录制 ROS 2 bag（默认 /livox/lidar），录完用 bag_to_pcd 转 PCD",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--config", default=str(config_path) if config_path else None,
        help="YAML 配置文件；默认读取脚本同目录的 record_bag.yaml")
    parser.add_argument("--topic", default="/livox/lidar",
                        help="要录制的 PointCloud2 话题")
    parser.add_argument("-o", "--output-dir", default="bags/empty_scene",
                        help="bag 输出目录（必须不存在）")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="录制时长（秒）；0=Ctrl+C 结束")
    parser.add_argument("--storage", choices=["mcap", "sqlite3"], default="mcap",
                        help="bag 存储格式")
    parser.add_argument("--compression-mode", choices=["none", "file", "message"],
                        default="none", help="压缩模式；none=不压缩")
    parser.add_argument("--compression-format", default="zstd",
                        help="压缩格式（compression-mode 不为 none 时使用）")
    parser.add_argument("--node-name", default="livox_bag_recorder",
                        help="ros2 bag record 的节点名")

    parser.set_defaults(**config)
    args = parser.parse_args(argv)
    args.config_used = args.config

    if args.duration < 0:
        parser.error("--duration 不能为负数")
    return args


def check_topic(topic: str):
    """只做提示，不阻止录制；驱动可能比录制脚本晚启动。"""
    try:
        listed = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            capture_output=True, text=True, timeout=5,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"警告: 无法执行 ros2 topic list: {exc}", file=sys.stderr)
        return

    names = {
        line.split()[0]
        for line in listed.stdout.splitlines()
        if line.strip()
    }
    if topic not in names:
        print(
            f"警告: ros2 topic list 里没有 {topic}。\n"
            "      bag 可能是 0 条消息；请先启动 Livox 驱动，"
            "并确认 ros2 topic hz 有数据。",
            file=sys.stderr,
        )
        return

    try:
        info = subprocess.run(
            ["ros2", "topic", "info", topic, "--verbose"],
            capture_output=True, text=True, timeout=5,
        )
        if "sensor_msgs/msg/PointCloud2" not in info.stdout:
            print(
                f"警告: {topic} 可能不是 sensor_msgs/msg/PointCloud2，"
                "bag_to_pcd.py 只能转换 PointCloud2。",
                file=sys.stderr,
            )
    except Exception as exc:  # noqa: BLE001
        print(f"警告: 无法检查 {topic} 的类型: {exc}", file=sys.stderr)


def stop_process(proc: subprocess.Popen):
    """先 SIGINT 让 rosbag2 正常收尾，超时才 terminate/kill。"""
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        print("警告: ros2 bag record 10 秒内未退出，发送 SIGTERM", file=sys.stderr)
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("警告: 仍未退出，发送 SIGKILL", file=sys.stderr)
            proc.kill()
            proc.wait()


def main(argv=None):
    args = parse_args(argv)
    if getattr(args, "config_used", None):
        print(f"使用配置文件: {args.config_used}")
    print(f"使用话题: {args.topic}")

    check_topic(args.topic)

    output = Path(args.output_dir).expanduser()
    if output.exists():
        print(
            f"输出目录已存在: {output}\n"
            "请修改配置里的 output_dir，或先删除旧 bag。",
            file=sys.stderr,
        )
        return 1
    output.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2", "bag", "record",
        "-o", str(output),
        "-s", args.storage,
        "--node-name", args.node_name,
    ]
    if args.compression_mode != "none":
        cmd += [
            "--compression-mode", args.compression_mode,
            "--compression-format", args.compression_format,
        ]
    cmd += ["--topics", args.topic]

    print("执行:", " ".join(cmd))
    if args.duration > 0:
        print(f"录制 {args.duration:.1f} 秒后自动停止；Ctrl+C 可提前结束")
    else:
        print("录制中，按 Ctrl+C 停止")

    proc = subprocess.Popen(cmd)
    try:
        if args.duration > 0:
            try:
                proc.wait(timeout=args.duration)
            except subprocess.TimeoutExpired:
                print("时间到，停止录制...")
                stop_process(proc)
        else:
            proc.wait()
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止录制...")
        stop_process(proc)

    if proc.returncode not in (0, -signal.SIGINT, 130):
        print(f"ros2 bag record 退出码: {proc.returncode}", file=sys.stderr)

    print(f"bag 已保存: {output.resolve()}")
    print(f"检查: ros2 bag info {output}")
    print(
        "转 PCD: "
        f"./scripts/bag_to_pcd.sh {output} "
        f"-o pcd_recordings/{output.name}_merged.pcd"
    )
    return 0 if proc.returncode in (0, -signal.SIGINT, 130) else proc.returncode


if __name__ == "__main__":
    sys.exit(main())
