#!/bin/bash
# 把本地开发代码同步到树莓派 ~/spiderpi（目录结构与机器人一致，逐个目录 rsync）
# 用法: ./sync_to_robot.sh [user@host]   默认 pi@192.168.149.1（AP 直连地址）
set -euo pipefail

ROBOT="${1:-pi@192.168.149.1}"
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE=/home/pi/spiderpi

EXCLUDE=(--exclude '__pycache__' --exclude '*.md')

# config/、functions/、advanced/、kinematic_routines/ 树莓派上有标定/官方文件，不加 --delete；
# agcs_lib/、tasks/、communication/ 是我们专属目录，加 --delete 让树莓派和本地完全一致（本地删了树莓派也删）。
rsync -avz "${EXCLUDE[@]}" "$BASE/config/"            "$ROBOT:$REMOTE/config/"
rsync -avz --delete "${EXCLUDE[@]}" "$BASE/agcs_lib/"          "$ROBOT:$REMOTE/agcs_lib/"
rsync -avz --delete "${EXCLUDE[@]}" "$BASE/tasks/"             "$ROBOT:$REMOTE/tasks/"
rsync -avz "${EXCLUDE[@]}" "$BASE/functions/"         "$ROBOT:$REMOTE/functions/"
rsync -avz "${EXCLUDE[@]}" "$BASE/advanced/"          "$ROBOT:$REMOTE/advanced/"
rsync -avz "${EXCLUDE[@]}" "$BASE/kinematic_routines/" "$ROBOT:$REMOTE/kinematic_routines/"
rsync -avz --delete "${EXCLUDE[@]}" "$BASE/communication/"     "$ROBOT:$REMOTE/communication/"

echo "同步完成: $ROBOT:$REMOTE"
