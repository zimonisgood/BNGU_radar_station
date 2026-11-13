#!/usr/bin/env bash
# record_pcd.py 的薄封装：
#   - 自动 source ROS 2 和工作空间
#   - 强制使用系统 Python 3.12（避免 conda 的 Python 3.13 覆盖 ROS）
#   - 默认读取 scripts/record_pcd.yaml
#
# 用法:
#   ./scripts/record_pcd.sh
#   ./scripts/record_pcd.sh --config scripts/record_robot.yaml
#   ./scripts/record_pcd.sh --duration 10 --mode merged
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "找不到 $ROS_SETUP，请确认已安装 ROS 2 Jazzy" >&2
  exit 1
fi
if [ ! -f "$WS_SETUP" ]; then
  echo "找不到 $WS_SETUP，请先在 $WS_DIR 运行 colcon build" >&2
  exit 1
fi
if [ ! -x /usr/bin/python3 ]; then
  echo "找不到 /usr/bin/python3" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS_SETUP"

exec /usr/bin/python3 "$SCRIPT_DIR/record_pcd.py" "$@"
