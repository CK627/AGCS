#!/bin/bash
# Robot Dashboard 管理入口 (macOS)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source "$PROJECT_DIR/scripts/common.sh"

case "${1:-start}" in
    start)      do_start "${2:-20002}" ;;
    stop)       do_stop ;;
    restart)    do_restart "${2:-20002}" ;;
    status)     do_status ;;
    setup)      do_setup ;;
    install)    do_install ;;
    update)     do_update ;;
    uninstall)  do_uninstall ;;
    help)       do_help ;;
    *)          echo "未知命令: $1"; do_help; exit 1 ;;
esac
