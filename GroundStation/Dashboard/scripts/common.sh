#!/bin/bash
# 地面站 公共脚本模块（被各平台 start.sh source 调用）
#
# 管理两个服务（同一套命令，一起启停/重启）：
#   1) 中枢仪表盘    backend/app.py    端口 20000
#   2) 研发进度流程图 progress/app.py  端口 20010（独立网页，数据从中枢取）

# 研发进度流程图（独立端口）相关常量
PROGRESS_DIR_NAME="progress"
PROGRESS_DEFAULT_PORT=20010
PROGRESS_DIR="$PROJECT_DIR/$PROGRESS_DIR_NAME"    # 页面目录（PROJECT_DIR 由各平台入口脚本先赋值）

# 选一个「带依赖」的 Python：优先 $PYTHON_BIN / 项目 .venv，再退到能 import flask+requests 的解释器。
# 原因：有些机器的 `python` 指向一个没装 requests 的解释器，直接 nohup 会起不来、日志只剩报错。
resolve_python() {
    local c
    for c in "$PYTHON_BIN" "$PROJECT_DIR/.venv/bin/python" python3 python \
             "$HOME/.devtools/ptool/shims/python3" "/opt/homebrew/bin/python3" "/usr/bin/python3"; do
        [ -z "$c" ] && continue
        if command -v "$c" >/dev/null 2>&1 && "$c" -c "import flask, requests" >/dev/null 2>&1; then
            echo "$c"; return 0
        fi
    done
    echo "${PYTHON_BIN:-python}"   # 实在找不到就退到 python：至少能在日志里看到缺依赖的报错
}

# ============================================
# 环境安装
# ============================================
do_install() {
    echo "========================================"
    echo "  地面站 环境安装"
    echo "========================================"
    echo ""

    echo "[1/2] 安装中枢依赖..."
    "$(resolve_python)" -m pip install -r "$PROJECT_DIR/backend/requirements.txt"
    echo ""

    echo "[2/2] 安装进度流程图依赖..."
    "$(resolve_python)" -m pip install -r "$PROGRESS_DIR/requirements.txt"
    echo ""

    echo "========================================"
    echo "  安装完成！"
    echo "========================================"
    echo ""
    echo "启动: $0 start"
}

# ============================================
# 启动服务
# ============================================
do_start() {
    local port="${1:-20000}"
    local pport="${2:-$PROGRESS_DEFAULT_PORT}"

    echo "=== 地面站 启动 ==="

    # 检查是否已在运行
    if is_running; then
        echo "中枢已在运行中"
    else
        # 确保必要目录存在
        mkdir -p "$PROJECT_DIR/logs"
        mkdir -p "$PROJECT_DIR/data"

        cd "$PROJECT_DIR/backend"

        echo "中枢端口: $port"

        nohup "$(resolve_python)" app.py --port "$port" > "$PROJECT_DIR/logs/nohup.log" 2>&1 &
        echo $! > "$PROJECT_DIR/data/server.pid"
        echo "$port" > "$PROJECT_DIR/data/server.port"
        echo "中枢已启动 (PID: $!, 端口: $port)"
    fi

    # ---- 研发进度流程图（独立端口）----
    if is_progress_running; then
        echo "进度流程图已在运行中"
    else
        mkdir -p "$PROGRESS_DIR"
        cd "$PROGRESS_DIR"

        echo "进度流程图端口: ${pport}  中枢: http://127.0.0.1:${port}"

        nohup "$(resolve_python)" app.py --port "$pport" --hub "http://127.0.0.1:$port" \
            > "$PROJECT_DIR/logs/progress.log" 2>&1 &
        echo $! > "$PROJECT_DIR/data/progress.pid"
        echo "$pport" > "$PROJECT_DIR/data/progress.port"
        echo "进度流程图已启动 (PID: $!, 端口: $pport)"
    fi

    sleep 3
    do_status
}

# ============================================
# 停止服务（两个都停）
# ============================================
do_stop() {
    echo "=== 地面站 停止 ==="

    # 1. 按 PID 文件杀
    if [ -f "$PROJECT_DIR/data/server.pid" ]; then
        local pid
        pid=$(cat "$PROJECT_DIR/data/server.pid")
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null && echo "已停止中枢 (PID: $pid)"
            sleep 1
            kill -9 "$pid" 2>/dev/null || true
        fi
        rm -f "$PROJECT_DIR/data/server.pid"
    fi

    # 2. 按端口杀（处理 PID 文件丢失的情况）
    local port="${1:-}"
    if [ -z "$port" ] && [ -f "$PROJECT_DIR/data/server.port" ]; then
        port=$(cat "$PROJECT_DIR/data/server.port")
    fi
    port="${port:-20000}"
    kill_port "$port" "中枢"

    rm -f "$PROJECT_DIR/data/server.pid"
    rm -f "$PROJECT_DIR/data/server.port"

    # 3. 进度流程图：同样按 PID 文件 + 端口停
    if [ -f "$PROJECT_DIR/data/progress.pid" ]; then
        local ppid
        ppid=$(cat "$PROJECT_DIR/data/progress.pid")
        if kill -0 "$ppid" 2>/dev/null; then
            kill "$ppid" 2>/dev/null && echo "已停止进度流程图 (PID: $ppid)"
            sleep 1
            kill -9 "$ppid" 2>/dev/null || true
        fi
        rm -f "$PROJECT_DIR/data/progress.pid"
    fi
    local pport=""
    if [ -f "$PROJECT_DIR/data/progress.port" ]; then
        pport=$(cat "$PROJECT_DIR/data/progress.port")
    fi
    kill_port "${pport:-$PROGRESS_DEFAULT_PORT}" "进度流程图"
    rm -f "$PROJECT_DIR/data/progress.pid"
    rm -f "$PROJECT_DIR/data/progress.port"

    echo "服务已停止"
}

# 释放某个端口（端口 + 名称用于日志）
kill_port() {
    local port="$1" name="$2"
    if command -v lsof &>/dev/null; then
        local pids
        pids=$(lsof -ti:"$port" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo "$name 端口 $port 被占用 (PID: $pids)，强制释放..."
            kill -9 $pids 2>/dev/null || true
            sleep 1
        fi
    fi
}

# ============================================
# 查看状态（两个都看）
# ============================================
do_status() {
    echo "=== 地面站 状态 ==="

    local pid port
    pid=$(get_pid)

    if [ -z "$pid" ]; then
        echo "中枢:      未运行"
    else
        port=20000
        if [ -f "$PROJECT_DIR/data/server.port" ]; then
            port=$(cat "$PROJECT_DIR/data/server.port")
        fi
        echo "中枢:      运行中 (PID: $pid)"
        echo "           访问 http://localhost:$port"
    fi

    local ppid pport
    ppid=$(get_progress_pid)
    if [ -z "$ppid" ]; then
        echo "进度流程图: 未运行"
    else
        pport=$PROGRESS_DEFAULT_PORT
        if [ -f "$PROJECT_DIR/data/progress.port" ]; then
            pport=$(cat "$PROJECT_DIR/data/progress.port")
        fi
        echo "进度流程图: 运行中 (PID: $ppid)"
        echo "           访问 http://localhost:$pport"
    fi
    echo ""

    if [ -f "$PROJECT_DIR/logs/nohup.log" ]; then
        echo "── 中枢最近日志 ──"
        tail -8 "$PROJECT_DIR/logs/nohup.log"
    fi
    if [ -f "$PROJECT_DIR/logs/progress.log" ]; then
        echo "── 进度流程图最近日志 ──"
        tail -6 "$PROJECT_DIR/logs/progress.log"
    fi
}

# ============================================
# 重启服务（两个一起重启）
# ============================================
do_restart() {
    echo "=== 地面站 重启 ==="
    local port="${1:-20000}"
    local pport="${2:-$PROGRESS_DEFAULT_PORT}"
    do_stop "$port"
    sleep 2
    do_start "$port" "$pport"
}

# ============================================
# 更新（git pull）
# ============================================
do_update() {
    echo "=== 地面站仪表盘 更新 ==="

    if [ ! -d "$PROJECT_DIR/.git" ]; then
        echo "错误: 不是 git 仓库，无法自动更新"
        return 1
    fi

    echo "[1/3] 停止服务..."
    do_stop 2>/dev/null || true

    echo "[2/3] 拉取最新代码..."
    cd "$PROJECT_DIR"
    git pull origin ground-station || { echo "拉取失败"; return 1; }
    cd - > /dev/null

    echo "[3/3] 更新依赖..."
    pip install -r "$PROJECT_DIR/backend/requirements.txt" -q
    pip install -r "$PROGRESS_DIR/requirements.txt" -q

    echo ""
    echo "更新完成！重新启动: $0 start"
}

# ============================================
# 卸载
# ============================================
do_uninstall() {
    echo "========================================"
    echo "  地面站 卸载"
    echo "========================================"
    echo ""

    read -p "确定要卸载吗？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        return 0
    fi

    echo "[1/3] 停止服务（中枢 + 进度流程图）..."
    do_stop 2>/dev/null || true

    echo "[2/3] 清理运行时数据..."
    rm -rf "$PROJECT_DIR/data"
    rm -rf "$PROJECT_DIR/logs"

    read -p "[3/3] 卸载 Python 依赖? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip uninstall -y flask waitress requests pymavlink opencv-python numpy 2>/dev/null || true
        echo "依赖已卸载"
    else
        echo "跳过"
    fi

    echo ""
    echo "卸载完成"
}

# ============================================
# 帮助
# ============================================
do_help() {
    cat << 'EOF'
地面站 管理脚本（中枢仪表盘 + 研发进度流程图，一起启停/重启）

用法:  start.sh [命令] [参数]

命令:
  start [中枢端口] [进度页端口]   启动两个服务（默认 20000 / 20010）
  stop                            停止两个服务
  restart [中枢端口] [进度页端口] 重启两个服务
  status                          查看两个服务的运行状态
  install        环境安装（两个服务的 Python 依赖）
  update         更新到最新版本
  uninstall      卸载
  help           帮助

示例:
  start.sh                         启动（默认端口 20000 / 20010）
  start.sh start 8080 8081         指定中枢 8080、进度流程图 8081
  start.sh restart                 重启两个服务
  start.sh stop                    停止两个服务

服务:
  中枢仪表盘       http://localhost:20000   （数据采集 + 四个面板）
  研发进度流程图   http://localhost:20010   （独立网页，进度数据从中枢取）
EOF
}

# ============================================
# 内部辅助函数
# ============================================
get_pid() {
    if [ -f "$PROJECT_DIR/data/server.pid" ]; then
        local pid
        pid=$(cat "$PROJECT_DIR/data/server.pid" 2>/dev/null)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "$pid"
        fi
    fi
}

is_running() {
    [ -n "$(get_pid)" ]
}

get_progress_pid() {
    if [ -f "$PROJECT_DIR/data/progress.pid" ]; then
        local pid
        pid=$(cat "$PROJECT_DIR/data/progress.pid" 2>/dev/null)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "$pid"
        fi
    fi
}

is_progress_running() {
    [ -n "$(get_progress_pid)" ]
}
