#!/bin/bash
# 把树莓派上的配置文件与 SDK 拉回本地，保持文件列表同步
# 用法: ./pull_from_robot.sh [user@host]   默认 pi@192.168.149.1
set -euo pipefail

ROBOT="${1:-pi@192.168.149.1}"
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE=/home/pi/spiderpi

rsync -avz "$ROBOT:$REMOTE/config/" "$BASE/config/"
rsync -avz \
  --exclude 'build' --exclude 'dist' --exclude '*.egg-info' --exclude '__pycache__' \
  "$ROBOT:$REMOTE/spiderpi_sdk/" "$BASE/spiderpi_sdk/"

echo "拉取完成: $ROBOT:$REMOTE -> $BASE"
