#!/usr/bin/env bash
# bag_to_pcd.py 的薄封装：
#   - 自动 source ROS 2
#   - 强制使用系统 Python 3.12
#   - 工作空间 install/setup.bash 存在时也 source；不存在不影响转换
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "找不到 $ROS_SETUP，请确认已安装 ROS 2 Jazzy" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [ -f "$WS_SETUP" ]; then
  # shellcheck disable=SC1091
  source "$WS_SETUP"
fi

exec /usr/bin/python3 "$SCRIPT_DIR/bag_to_pcd.py" "$@"
