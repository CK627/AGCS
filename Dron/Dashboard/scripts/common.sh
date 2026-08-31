#!/bin/bash
# Dashboard 公共脚本模块（被各平台 start.sh source 调用）
# 无人机电脑端网页：视频 + UDP 监控 + 下达任务

DEFAULT_PORT=20000

# Dashboard 目录（入口脚本会先设置，这里兜底）
PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BACKEND_DIR="$PROJECT_DIR/backend"
PYTHON_BIN="${PYTHON_BIN:-python3}"

# ============================================
# 环境安装
# ============================================
do_install() {
    echo "========================================"
    echo "  Dashboard 环境安装"
    echo "========================================"
    echo ""

    echo "[1/2] 检查 Python..."
    if ! command -v "$PYTHON_BIN" &>/dev/null; then
        echo "  ✗ 未检测到 Python，请先安装 Python 3.8+"
        echo "    macOS: brew install python3"
        echo "    Linux: apt install python3 python3-pip"
        return 1
    fi
    "$PYTHON_BIN" --version
    echo ""

    echo "[2/2] 检查 Python 依赖..."
    if "$PYTHON_BIN" -c "import flask, waitress, pymavlink, numpy" &>/dev/null; then
        echo "  ✓ 核心依赖已安装"
    else
        echo "  ⚠ 缺少依赖，请执行: $0 setup"
    fi
    echo ""

    echo "========================================"
    echo "  安装完成！"
    echo "========================================"
    echo ""
    echo "首次运行请先安装依赖:"
    echo "  $0 setup"
    echo ""
    echo "启动: $0 start"
}

# ============================================
# 启动服务
# ============================================
do_start() {
    local port="${1:-$DEFAULT_PORT}"
    local os_name
    os_name=$(uname -s)

    echo "=== Dashboard 启动 ==="

    # 检查 Python 是否可用
    if ! command -v "$PYTHON_BIN" &>/dev/null; then
        echo "错误: 未找到 Python"
        return 1
    fi

    # 检查端口是否已被占用（可能是本项目的旧进程）
    if is_port_in_use "$port"; then
        local existing_pid
        existing_pid=$(lsof -ti:"$port" 2>/dev/null)
        echo "端口 $port 已被占用 (PID: $existing_pid)"
        echo "如需重启: $0 restart"
        return 1
    fi

    # 确保必要目录存在
    mkdir -p "$PROJECT_DIR/data"
    mkdir -p "$PROJECT_DIR/logs"

    cd "$BACKEND_DIR"

    echo "端口: $port"
    echo "目录: $PROJECT_DIR"

    case "$os_name" in
        Darwin|Linux)
            PYTHONUNBUFFERED=1 nohup "$PYTHON_BIN" app.py --port "$port" > "$PROJECT_DIR/logs/server.log" 2>&1 &
            ;;
        *)
            "$PYTHON_BIN" app.py --port "$port" &
            ;;
    esac

    local pid=$!
    echo "$pid" > "$PROJECT_DIR/data/server.pid"
    echo "$port" > "$PROJECT_DIR/data/server.port"
    echo "服务已启动 (PID: $pid)"

    # 等待并验证启动
    sleep 1.5
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "⚠ 进程启动失败，查看日志:"
        tail -10 "$PROJECT_DIR/logs/server.log"
        rm -f "$PROJECT_DIR/data/server.pid" "$PROJECT_DIR/data/server.port"
        return 1
    fi

    do_status
}

# ============================================
# 停止服务
# ============================================
do_stop() {
    echo "=== Dashboard 停止 ==="

    local stopped=0
    local pid_file="$PROJECT_DIR/data/server.pid"
    local port_file="$PROJECT_DIR/data/server.port"

    # 读取保存的端口
    local saved_port
    saved_port=$(cat "$port_file" 2>/dev/null)

    # 1. 按 PID 文件杀
    if [ -f "$pid_file" ]; then
        local pid
        pid=$(cat "$pid_file")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null && echo "已停止 (PID: $pid)" && stopped=1
            sleep 1
            kill -9 "$pid" 2>/dev/null || true
        fi
    fi

    # 2. 按端口杀（处理 PID 文件丢失的情况）
    local port="${saved_port:-$DEFAULT_PORT}"
    if command -v lsof &>/dev/null; then
        local pids
        pids=$(lsof -ti:"$port" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo "端口 $port 仍被占用 (PID: $pids)，强制释放..."
            kill -9 $pids 2>/dev/null || true
            stopped=1
            sleep 1
        fi
    fi

    rm -f "$pid_file" "$port_file"

    if [ "$stopped" -eq 0 ]; then
        echo "服务未在运行"
    else
        echo "服务已停止"
    fi
}

# ============================================
# 查看状态
# ============================================
do_status() {
    echo "=== Dashboard 状态 ==="

    local pid port
    pid=$(get_pid)
    port=$(get_port)

    if [ -z "$pid" ]; then
        echo "状态: 未运行"
        return 1
    fi

    echo "状态: 运行中"
    echo "PID:   $pid"
    echo "端口:  ${port:-$DEFAULT_PORT}"
    echo "访问:  http://127.0.0.1:${port:-$DEFAULT_PORT}"
}

# ============================================
# 重启服务
# ============================================
do_restart() {
    echo "=== Dashboard 重启 ==="
    do_stop
    sleep 1.5
    do_start "$@"
}

# ============================================
# 安装依赖（首次运行）
# ============================================
do_setup() {
    echo "=== Dashboard 依赖安装 ==="

    if ! command -v "$PYTHON_BIN" &>/dev/null; then
        echo "错误: 未找到 Python"
        return 1
    fi

    cd "$PROJECT_DIR"
    "$PYTHON_BIN" -m pip install -r requirements.txt
}

# ============================================
# 更新（git pull）
# ============================================
do_update() {
    echo "=== Dashboard 更新 ==="

    if ! git -C "$PROJECT_DIR" rev-parse --git-dir >/dev/null 2>&1; then
        echo "错误: 不是 git 仓库，无法自动更新"
        return 1
    fi

    echo "[1/2] 停止服务..."
    do_stop 2>/dev/null || true

    echo "[2/2] 拉取最新代码..."
    cd "$PROJECT_DIR"
    git pull origin main || { echo "拉取失败"; return 1; }
    cd - > /dev/null

    echo ""
    echo "更新完成！重新启动: $0 start"
}

# ============================================
# 卸载
# ============================================
do_uninstall() {
    echo "========================================"
    echo "  Dashboard 卸载"
    echo "========================================"
    echo ""

    read -p "确定要卸载吗？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        return 0
    fi

    echo "[1/2] 停止服务..."
    do_stop 2>/dev/null || true

    echo "[2/2] 清理运行时数据..."
    rm -rf "$PROJECT_DIR/data"
    rm -rf "$PROJECT_DIR/logs"

    echo ""
    echo "卸载完成（代码未删除）"
}

# ============================================
# 帮助
# ============================================
do_help() {
    cat << 'EOF'
Dashboard 管理脚本

用法:  start.sh [命令] [参数]

命令:
  start [端口]   启动服务（默认端口 20000）
  stop           停止服务
  restart [端口] 重启服务
  status         查看运行状态
  setup          安装依赖（首次运行）
  install        环境检查（Python / 依赖）
  update         更新到最新版本
  uninstall      卸载
  help           帮助

示例:
  start.sh                   启动（默认端口 20000）
  start.sh start 8080        启动并指定端口 8080
  start.sh restart           重启服务（沿用旧端口）
  start.sh restart 8080      重启并换到 8080 端口
  start.sh stop              停止服务
  start.sh status            查看状态
  start.sh setup             安装依赖
EOF
}

# ============================================
# 辅助函数
# ============================================

get_pid() {
    local pid_file="$PROJECT_DIR/data/server.pid"

    # 优先从 PID 文件读取
    if [ -f "$pid_file" ]; then
        local pid
        pid=$(cat "$pid_file")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "$pid"
            return 0
        fi
    fi

    # 后备：按保存的端口精确查找
    local saved_port
    saved_port=$(cat "$PROJECT_DIR/data/server.port" 2>/dev/null)
    local match_port="${saved_port:-$DEFAULT_PORT}"

    local pids
    pids=$(lsof -ti:"$match_port" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        echo "$pids" | head -1
        return 0
    fi

    return 1
}

get_port() {
    # 优先从端口文件读取
    local port_file="$PROJECT_DIR/data/server.port"
    if [ -f "$port_file" ]; then
        cat "$port_file"
        return 0
    fi

    # 后备：从运行中进程解析
    local pid
    pid=$(get_pid 2>/dev/null)
    if [ -n "$pid" ] && command -v lsof &>/dev/null; then
        lsof -Pan -p "$pid" -i TCP -s TCP:LISTEN 2>/dev/null | \
            awk 'NR>1 {split($9,a,":"); print a[length(a)]}' | head -1
    fi
}

is_port_in_use() {
    local port="$1"
    if command -v lsof &>/dev/null; then
        lsof -ti:"$port" >/dev/null 2>&1
        return $?
    fi
    return 1
}

is_running() {
    local pid
    pid=$(get_pid 2>/dev/null)
    [ -n "$pid" ]
}
