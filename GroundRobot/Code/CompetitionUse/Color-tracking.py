#!/usr/bin/python3
# coding=utf8
"""比赛步骤 2.1 视觉追踪：云台 PID 跟随色块，回传地面站。

启动 task_server（5000 端口），带标注画面推给地面站 /video.mjpeg，
仪表盘状态设为 TRACKING（中枢轮询 /status 判定；2.1 是人工勾选）。

用法（先 sudo systemctl stop spiderpi）：
    cd /home/pi/spiderpi/CompetitionUse
    python3 Color-tracking.py --color red
"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import _common
from agcs_lib.logs import setup_logger

try:
    from communication import task_server
except ImportError:
    task_server = None

try:
    import progress_signal
except ImportError:
    progress_signal = None


def main():
    parser = argparse.ArgumentParser(description='2.1 视觉追踪')
    parser.add_argument('--color', default='red',
                        choices=['red', 'green', 'blue', 'yellow'])
    args = parser.parse_args()

    logger = setup_logger('competition_track')
    logger.info('视觉追踪启动：color=%s', args.color)

    if progress_signal is not None:
        progress_signal.notify_hub('robot', 1)  # 失败仅告警，不中断

    # 带标注画面推给地面站的钩子（detect 闭包内部调用 publish_frame）
    publish = task_server.publish_frame if task_server is not None else None
    rt = _common.build_runtime(args, logger, publish_frame=publish)
    if rt is None:
        return

    if task_server is not None:
        task_server.start_server()
        task_server.set_status(state='TRACKING', message='2.1 视觉追踪，目标颜色=%s' % args.color)

    from agcs_lib.tracker import ColorTracker
    from agcs_lib.sensors import show_status

    show_status(rt.display, 1)
    tracker = ColorTracker(rt.board, rt.detect)
    tracker.start()

    logger.info('追踪开始（Ctrl+C 退出）')
    try:
        while True:
            latest = tracker.latest()
            if task_server is not None:
                if latest is not None:
                    cx, cy = latest['center']
                    task_server.set_status(
                        state='TRACKING',
                        message='追踪中 中心=(%d,%d) area=%.0f' % (cx, cy, latest['area']))
                else:
                    task_server.set_status(state='TRACKING', message='追踪中 未发现目标')
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        _common.reset_gimbal(rt.board)
        _common.close_runtime(rt)
        show_status(rt.display, 0)
        if task_server is not None:
            task_server.set_status(state='IDLE', message='追踪结束')


if __name__ == '__main__':
    main()
