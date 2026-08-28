#!/bin/bash
# 用 Git 把当前分支的 GroundRobot/Code 部署到树莓派 ~/spiderpi。
#
# 流程：
# 1. 检查 GroundRobot/Code 是否已提交；
# 2. git push origin <当前分支>；
# 3. 在本机用 git archive 从 origin/<当前分支> 导出各目录；
# 4. 通过 SSH 在机器人端解包，不再使用 rsync。
#
# 用法: ./sync_to_robot.sh [user@host]
set -euo pipefail

ROBOT="${1:-pi@10.194.228.89}"
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$BASE/../.." && pwd)"
REMOTE=/home/pi/spiderpi

BRANCH="$(git -C "$REPO_ROOT" branch --show-current)"
if [[ -z "$BRANCH" ]]; then
    echo "无法确定当前 Git 分支" >&2
    exit 1
fi

if [[ -n "$(git -C "$REPO_ROOT" status --porcelain --untracked-files=no -- GroundRobot/Code)" ]]; then
    echo "GroundRobot/Code 有未提交修改，请先 commit 后再同步" >&2
    exit 1
fi

git -C "$REPO_ROOT" push origin "$BRANCH"

sync_dir() {
    local dir="$1"
    local delete="$2"

    if [[ "$delete" == "1" ]]; then
        ssh "$ROBOT" "rm -rf '$REMOTE/$dir'"
    fi
    ssh "$ROBOT" "mkdir -p '$REMOTE/$dir'"

    git -C "$REPO_ROOT" archive --format=tar "origin/$BRANCH" "GroundRobot/Code/$dir" \
        | ssh "$ROBOT" "tar -x -C '$REMOTE' --strip-components=2 --exclude='*.md'"
}

# 我们专属目录：和 Git 保持一致，先清空再解包。
sync_dir agcs_lib 1
sync_dir tasks 1
sync_dir communication 1

# 机器人上有官方/标定文件的目录：只覆盖 Git 里已有的文件，不清空。
sync_dir config 0
sync_dir functions 0
sync_dir advanced 0
sync_dir kinematic_routines 0

echo "Git 同步完成: origin/$BRANCH -> $ROBOT:$REMOTE"
