#!/bin/bash
# 把本地开发代码同步到树莓派 ~/spiderpi（目录结构与机器人一致，逐个目录 rsync）
# 用法: ./sync_to_robot.sh [user@host]   默认 pi@192.168.149.1（AP 直连地址）
set -euo pipefail

ROBOT="${1:-pi@192.168.149.1}"
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE=/home/pi/spiderpi

EXCLUDE=(--exclude '__pycache__' --exclude '*.md')

rsync -avz "${EXCLUDE[@]}" "$BASE/config/"            "$ROBOT:$REMOTE/config/"
rsync -avz "${EXCLUDE[@]}" "$BASE/agcs_lib/"          "$ROBOT:$REMOTE/agcs_lib/"
rsync -avz "${EXCLUDE[@]}" "$BASE/tasks/"             "$ROBOT:$REMOTE/tasks/"
rsync -avz "${EXCLUDE[@]}" "$BASE/functions/"         "$ROBOT:$REMOTE/functions/"
rsync -avz "${EXCLUDE[@]}" "$BASE/advanced/"          "$ROBOT:$REMOTE/advanced/"
rsync -avz "${EXCLUDE[@]}" "$BASE/kinematic_routines/" "$ROBOT:$REMOTE/kinematic_routines/"
rsync -avz "${EXCLUDE[@]}" "$BASE/communication/"     "$ROBOT:$REMOTE/communication/"

echo "同步完成: $ROBOT:$REMOTE"
