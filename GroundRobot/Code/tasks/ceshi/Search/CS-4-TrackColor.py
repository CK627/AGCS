#!/usr/bin/python3
# coding=utf8
"""CS-4-TrackColor：扫描找到目标 → 颜色追踪（目标锁定 + 画面居中，不走动）测试。

只调用 Restore 的摄像头启动；默认颜色 blue。
流程：
    1) Restore.start_camera() 启动摄像头 + 推流；
    2) ScanNumberTwentyOne() 扫描找目标（21号+24号）；
    3) TrackColor() 启动颜色追踪：后台 PID 持续调整 21/24，
       让目标一直保持在画面居中（机器人不走动）；
    4) 看到目标稳定居中后，回车结束（停止追踪、关闭摄像头）。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/Search/CS-4-TrackColor.py
"""
import os
import socket
import sys
import threading
import time

os.environ.setdefault('OPENCV_LOG_LEVEL', 'SILENT')   # 屏蔽 OpenCV 警告刷屏

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2

from agcs_lib import (
    make_board, make_ik, make_arm_ik, load_params,
    load_lab_data, detect_color, capture, correct_camera,
)
from agcs_lib.logs import setup_logger
from agcs_lib.restore import Restore
from agcs_lib.search import Searcher

try:
    from communication import task_server
except ImportError:
    task_server = None


def lan_ip():
    """取本机局域网 IP（不真正发包）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def _publish_loop(cam, mapx, mapy, rotate, stop_event):
    """后台线程：持续取帧 → 畸变校正 → 推给 /video.mjpeg（内部 10fps 限流）。"""
    while not stop_event.is_set():
        f = capture(cam, tries=5)
        if f is None:
            time.sleep(0.1)
            continue
        frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        if task_server is not None:
            task_server.publish_frame(frame)
        time.sleep(0.05)


def main():
    logger = setup_logger('CS-4-TrackColor')   # 终端显示流程，详细日志进文件
    params = load_params()
    color = 'blue'   # 默认颜色

    if task_server is not None:
        task_server.start_server()
        task_server.set_status(state='CS-4-TrackColor', message='颜色追踪：目标保持居中')

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(board, params)
    restore = Restore(board, ik, ak, params)

    cam, mapx, mapy, rotate = restore.start_camera()
    logger.info('[camera] 摄像头启动 OK')

    # 推流：浏览器打开 http://<IP>:5000/video.mjpeg 看扫描与追踪过程
    stop_video = threading.Event()
    threading.Thread(
        target=_publish_loop, args=(cam, mapx, mapy, rotate, stop_video), daemon=True).start()
    logger.info('[video] 推流已开启：http://%s:5000/video.mjpeg', lan_ip())

    lab = load_lab_data()

    def detect():
        f = capture(cam, tries=5)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, color, min_area=150)

    searcher = Searcher(board, ik, ak, params, detect)

    # 1) 扫描找目标
    logger.info('[scan] 开始扫描（21号+24号）…')
    r = searcher.ScanNumberTwentyOne()
    if r is None:
        logger.info('[scan] 未找到目标，直接进入追踪等待')
    else:
        logger.info('[scan] 找到目标 center=%s area=%d', r['center'], r.get('area', 0))

    # 2) 颜色追踪：目标锁定并保持画面居中（不走动）
    searcher.TrackColor()
    logger.info('[track] 目标保持居中中，请观察画面…')

    try:
        input('目标稳定居中后，按回车结束…')
    except EOFError:
        time.sleep(3)
    searcher.stop()
    stop_video.set()
    restore.close_camera()
    logger.info('[end] 结束：追踪已停止，摄像头已关闭')
    return 0


if __name__ == '__main__':
    sys.exit(main())
