#!/usr/bin/python3
# coding=utf8
"""夹取单测：只按 x,y,z 走 IK，不做微调。

用法：
    python3 CS-zq.py --color blue
"""
import argparse
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    make_board, make_ik, make_arm_ik, load_params, load_lab_data, load_block_params,
    detect_color, correct_camera, load_undistort_maps, make_ultrasonic, make_display,
    open_camera, capture, stand,
)
from agcs_lib.grab import Grabber
from agcs_lib.logs import setup_logger, action_msg


def main():
    parser = argparse.ArgumentParser(description='夹取单测：纯 x,y,z IK')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=50)
    args = parser.parse_args()

    logger = setup_logger('CS-zq')
    logger.info('[CS-zq] %s', action_msg('启动夹取单测', action='color=%s' % args.color))

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(board)
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    cam = open_camera()

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=args.min_area)

    def find_with_24():
        """只扫描 24 号找目标，找到就停（100-700，避免 1000 看天花板）。返回 (center, y_dis)。"""
        board.bus_servo_set_position(0.5, [[21, 500]])
        for y in range(100, 701, 20):
            board.bus_servo_set_position(0.05, [[24, y]])
            time.sleep(0.08)
            r = detect()
            if r is not None:
                board.bus_servo_set_position(0.3, [[24, y]])
                time.sleep(0.3)
                return r['center'], y
        return None, 0

    stand(ik)
    center, y_dis = find_with_24()
    if center is None:
        logger.info('[CS-zq] %s', action_msg('未找到目标', action='颜色=%s' % args.color))
        cam.camera_close()
        return
    grabber = Grabber(board, ik, ak, params, K, R, T, detect, display, ultrasonic=ultrasonic)
    try:
        ok = grabber.run(cy=center[1], x_dis=500, y_dis=y_dis)
        if ok:
            logger.info('[CS-zq] %s', action_msg('夹取成功', action='颜色=%s' % args.color))
        else:
            logger.info('[CS-zq] %s', action_msg('夹取失败', action='颜色=%s' % args.color))
    finally:
        cam.camera_close()


if __name__ == '__main__':
    main()
