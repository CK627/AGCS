#!/usr/bin/python3
# coding=utf8
"""CS-3-Scan21：21号舵机水平扫描 + 每档调用24号上下扫描 目标检测测试。

只调用 Restore 的摄像头启动；默认颜色 blue。
流程：
    1) Restore.start_camera() 启动摄像头 + 推流；
    2) 21号按 500-300-200-700-800 扫描，每档调用 24号上下扫描，
       发现目标就停下；找不到则 21号回 500、24号回 260；
    3) 看到目标确认后，回车结束（关闭摄像头）。

用法（树莓派，先 sudo systemctl stop spiderpi）：
    python3 tasks/ceshi/Search/CS-3-Scan21.py
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
    logger = setup_logger('CS-3-Scan21', console=False)   # 只写日志文件，终端不输出
    params = load_params()
    color = 'blue'   # 默认颜色

    if task_server is not None:
        task_server.start_server()
        task_server.set_status(state='CS-3-Scan21', message='21号+24号扫描目标检测')

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(params)
    restore = Restore(board, ik, ak, params)

    cam, mapx, mapy, rotate = restore.start_camera()
    logger.info('[camera] 摄像头启动 OK')

    # 推流：浏览器打开 http://<IP>:5000/video.mjpeg 看扫描过程
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
    r = searcher.ScanNumberTwentyOne()
    if r is not None:
        logger.info('[scan21] PASS 找到目标 center=%s area=%d', r['center'], r.get('area', 0))
    else:
        logger.info('[scan21] 未找到目标')

    try:
        input('看到目标确认后，按回车结束…')
    except EOFError:
        time.sleep(2)
    stop_video.set()
    restore.close_camera()
    logger.info('[end] 结束：摄像头已关闭')
    return 0


if __name__ == '__main__':
    sys.exit(main())
