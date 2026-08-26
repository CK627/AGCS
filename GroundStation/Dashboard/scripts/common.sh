#!/bin/bash
# 地面站仪表盘 公共脚本模块（被各平台 start.sh source 调用）

# ============================================
# 环境安装
# ============================================
do_install() {
    echo "========================================"
    echo "  地面站仪表盘 环境安装"
    echo "========================================"
    echo ""

    echo "[1/1] 安装 Python 依赖..."
    pip install -r "$PROJECT_DIR/backend/requirements.txt"
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
    local port="${1:-20001}"

    echo "=== 地面站仪表盘 启动 ==="

    # 检查是否已在运行
    if is_running; then
        echo "服务已在运行中"
        return 0
    fi

    # 确保必要目录存在
    mkdir -p "$PROJECT_DIR/logs"
    mkdir -p "$PROJECT_DIR/data"

    cd "$PROJECT_DIR/backend"

    echo "端口: $port"

    nohup python app.py --port "$port" > "$PROJECT_DIR/logs/nohup.log" 2>&1 &
    echo $! > "$PROJECT_DIR/data/server.pid"
    echo "$port" > "$PROJECT_DIR/data/server.port"
    echo "服务已启动 (PID: $!, 端口: $port)"

    sleep 2
    do_status
}

# ============================================
# 停止服务
# ============================================
do_stop() {
    echo "=== 地面站仪表盘 停止 ==="

    # 1. 按 PID 文件杀
    if [ -f "$PROJECT_DIR/data/server.pid" ]; then
        local pid
        pid=$(cat "$PROJECT_DIR/data/server.pid")
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null && echo "已停止 (PID: $pid)"
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
    port="${port:-20001}"
    if command -v lsof &>/dev/null; then
        local pids
        pids=$(lsof -ti:"$port" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo "端口 $port 被占用 (PID: $pids)，强制释放..."
            kill -9 $pids 2>/dev/null || true
            sleep 1
        fi
    fi

    rm -f "$PROJECT_DIR/data/server.pid"
    rm -f "$PROJECT_DIR/data/server.port"
    echo "服务已停止"
}

# ============================================
# 查看状态
# ============================================
do_status() {
    echo "=== 地面站仪表盘 状态 ==="

    local pid
    pid=$(get_pid)

    if [ -z "$pid" ]; then
        echo "状态: 未运行"
        return 1
    fi

    echo "状态: 运行中"
    echo "PID:   $pid"
    local port=20001
    if [ -f "$PROJECT_DIR/data/server.port" ]; then
        port=$(cat "$PROJECT_DIR/data/server.port")
    fi
    echo "访问:  http://localhost:$port"
    echo ""

    if [ -f "$PROJECT_DIR/logs/nohup.log" ]; then
        echo "── 最近日志 ──"
        tail -3 "$PROJECT_DIR/logs/nohup.log"
    fi
}

# ============================================
# 重启服务
# ============================================
do_restart() {
    echo "=== 地面站仪表盘 重启 ==="
    local port="${1:-20001}"
    do_stop "$port"
    sleep 2
    do_start "$port"
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
    git pull origin main || { echo "拉取失败"; return 1; }
    cd - > /dev/null

    echo "[3/3] 更新依赖..."
    pip install -r "$PROJECT_DIR/backend/requirements.txt" -q

    echo ""
    echo "更新完成！重新启动: $0 start"
}

# ============================================
# 卸载
# ============================================
do_uninstall() {
    echo "========================================"
    echo "  地面站仪表盘 卸载"
    echo "========================================"
    echo ""

    read -p "确定要卸载吗？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        return 0
    fi

    echo "[1/3] 停止服务..."
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
地面站仪表盘 管理脚本

用法:  start.sh [命令] [参数]

命令:
  start [端口]   启动服务（默认端口 20001）
  stop           停止服务
  restart [端口] 重启服务
  status         查看运行状态
  install        环境安装（Python 依赖）
  update         更新到最新版本
  uninstall      卸载
  help           帮助

示例:
  start.sh                   启动（默认端口 20001）
  start.sh start 8080        启动并指定端口 8080
  start.sh restart           重启服务
  start.sh stop              停止服务
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
