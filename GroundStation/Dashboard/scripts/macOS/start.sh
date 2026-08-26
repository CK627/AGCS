#!/bin/bash
# 地面站仪表盘 管理入口 (macOS)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$PROJECT_DIR/scripts/common.sh"

case "${1:-start}" in
    start)      do_start "${2:-20001}" ;;
    stop)       do_stop ;;
    restart)    do_restart "${2:-20001}" ;;
    status)     do_status ;;
    install)    do_install ;;
    update)     do_update ;;
    uninstall)  do_uninstall ;;
    help)       do_help ;;
    *)          echo "未知命令: $1"; do_help; exit 1 ;;
esac
