#!/bin/bash
# 地面站 管理入口 (Linux)：中枢仪表盘 + 研发进度流程图

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$PROJECT_DIR/scripts/common.sh"

case "${1:-start}" in
    start)      do_start "${2:-20000}" "${3:-20010}" ;;
    stop)       do_stop ;;
    restart)    do_restart "${2:-20000}" "${3:-20010}" ;;
    status)     do_status ;;
    install)    do_install ;;
    update)     do_update ;;
    uninstall)  do_uninstall ;;
    help)       do_help ;;
    *)          echo "未知命令: $1"; do_help; exit 1 ;;
esac
