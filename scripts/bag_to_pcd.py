#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_to_pcd.py — 读取 ROS 2 bag 里的 PointCloud2，把所有帧合并成一个 PCD。

用法（推荐用 wrapper，自动 source ROS）:
  ./scripts/bag_to_pcd.sh bags/empty_scene
  ./scripts/bag_to_pcd.sh bags/empty_scene -o pcd_recordings/empty_scene.pcd
  ./scripts/bag_to_pcd.sh bags/empty_scene --topic /livox/lidar --max-frames 150

直接调用 Python:
  /usr/bin/python3 scripts/bag_to_pcd.py bags/empty_scene -o empty_scene.pcd

说明:
  - 只处理 sensor_msgs/msg/PointCloud2；
  - 默认保存 x y z intensity，默认跳过 NaN；
  - --keep-extra 额外保存 tag/line；
  - --no-intensity 只保存 x y z；
  - 默认输出 pcd_recordings/<bag名>_merged.pcd。
  - 省略 bag 参数时，如果 bags/ 下只有一个候选，会自动使用它。
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from record_pcd import build_array, build_fields, extract_points, write_pcd


def read_storage_id(bag_path: Path, fallback: str) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return fallback
    with metadata_path.open("r", encoding="utf-8") as f:
        metadata = yaml.safe_load(f) or {}
    info = metadata.get("rosbag2_bagfile_information", {})
    return info.get("storage_identifier", fallback)


def find_default_bag(script_dir: Path) -> Path:
    """未指定 bag 时，在 workspace 的 bags/ 下找一个 bag。

    优先顺序: 只有一个候选 -> 直接用它；多个候选 -> 报错列出，让用户明确指定。
    """
    ws_dir = script_dir.parent
    bags_dir = ws_dir / "bags"
    candidates = []
    if bags_dir.is_dir():
        for child in sorted(bags_dir.iterdir()):
            if not child.is_dir():
                continue
            if (child / "metadata.yaml").exists() or any(child.glob("*.mcap")):
                candidates.append(child)
    if not candidates:
        raise SystemExit(
            f"未指定 bag，且在 {bags_dir} 下没有找到可用的 bag。\n"
            "用法: ./scripts/bag_to_pcd.sh bags/<bag名>"
        )
    if len(candidates) > 1:
        names = "\n".join(f"  {c}" for c in candidates)
        raise SystemExit(
            "未指定 bag，且找到多个候选，请明确指定：\n" + names
        )
    return candidates[0]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="读取 ROS 2 bag 中的 PointCloud2，合并成一个 PCD",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("bag", nargs="?", default=None,
                        help="bag 目录，例如 bags/empty_scene")
    parser.add_argument("-o", "--output", default=None,
                        help="输出 PCD；默认 pcd_recordings/<bag名>_merged.pcd")
    parser.add_argument("--topic", default="/livox/lidar",
                        help="要读取的 PointCloud2 话题")
    parser.add_argument("--storage-id", default="mcap",
                        help="metadata.yaml 不存在时使用的存储插件")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="最多读取多少帧；0=全部")
    parser.add_argument("--keep-nans", action="store_true",
                        help="保留 NaN/Inf（默认跳过）")
    parser.add_argument("--keep-extra", action="store_true",
                        help="额外保存 tag/line 字段（如果消息里有）")
    parser.add_argument("--no-intensity", action="store_true",
                        help="不保存 intensity 字段")
    parser.add_argument("--ascii", action="store_true",
                        help="输出 ASCII PCD（默认 binary）")
    args = parser.parse_args(argv)

    if args.max_frames < 0:
        parser.error("--max-frames 不能为负数")
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.bag:
        bag_path = Path(args.bag).expanduser().resolve()
    else:
        bag_path = find_default_bag(Path(__file__).resolve().parent).resolve()
        print(f"未指定 bag，自动使用: {bag_path}")
    if not bag_path.is_dir():
        print(f"bag 目录不存在: {bag_path}", file=sys.stderr)
        return 1

    storage_id = read_storage_id(bag_path, args.storage_id)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topics:
        print(f"bag 里没有话题 {args.topic}", file=sys.stderr)
        print("可用话题:", file=sys.stderr)
        for name, type_name in sorted(topics.items()):
            print(f"  {name}  [{type_name}]", file=sys.stderr)
        return 1

    topic_type = topics[args.topic]
    if topic_type != "sensor_msgs/msg/PointCloud2":
        print(
            f"话题 {args.topic} 的类型是 {topic_type}，不是 "
            "sensor_msgs/msg/PointCloud2；无法直接转 PCD。",
            file=sys.stderr,
        )
        return 1

    msg_type = get_message(topic_type)
    chunks = []
    fields = None
    extra_names = None
    frames = 0
    total_points = 0

    while reader.has_next():
        topic_name, data, _timestamp = reader.read_next()
        if topic_name != args.topic:
            continue

        msg = deserialize_message(data, msg_type)
        points, names = extract_points(
            msg,
            include_intensity=not args.no_intensity,
            keep_extra=args.keep_extra,
            keep_nans=args.keep_nans,
        )
        n = len(points["x"])
        if n == 0:
            continue

        if fields is None:
            extra_names = names
            fields = build_fields(not args.no_intensity, extra_names)
        elif names != extra_names:
            print(
                f"警告: 第 {frames + 1} 帧字段与首帧不一致，按首帧 schema 处理",
                file=sys.stderr,
            )

        chunks.append(build_array(points, fields))
        frames += 1
        total_points += n
        if frames == 1 or frames % 20 == 0:
            print(f"已读取 {frames} 帧, 累计 {total_points} 点")

        if args.max_frames > 0 and frames >= args.max_frames:
            break

    if not chunks:
        print("没有读到任何非空点云。", file=sys.stderr)
        return 1

    merged = np.concatenate(chunks)
    if args.output:
        output = Path(args.output).expanduser()
    else:
        output = Path("pcd_recordings") / f"{bag_path.name}_merged.pcd"
    output.parent.mkdir(parents=True, exist_ok=True)
    write_pcd(
        output,
        np.ascontiguousarray(merged),
        fields,
        binary=not args.ascii,
    )
    print(
        f"完成: {frames} 帧, {len(merged)} 点 -> {output.resolve()}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
