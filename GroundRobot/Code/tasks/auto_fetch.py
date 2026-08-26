#!/usr/bin/python3
# coding=utf8
"""自动取物编排：先寻路(search)，再夹取(grab)，两个算法严格分离。"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


def main():
    import cv2
    from agcs_lib import (
        make_board, make_ik, make_arm_ik, load_params,
        load_lab_data, load_block_params, detect_color, correct_camera,
        load_undistort_maps, make_ultrasonic, make_display, open_camera, capture,
        stand,
    )
    from agcs_lib.search import Searcher
    from agcs_lib.grab import Grabber
    from agcs_lib.sensors import show_status

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    args = parser.parse_args()
    color = args.color

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    cam = open_camera()

    def detect(min_area=300):
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, color, min_area=min_area)

    stand(ik)
    # 机械臂复位（21/22/23/24 回 reset_pulses，夹爪张开）
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(2)

    searcher = Searcher(board, ik, ak, params, detect, ultrasonic, display)
    center, cy = searcher.run()
    if center is None:
        print('%s 未找到目标' % color)
        cam.camera_close()
        show_status(display, 0)
        return

    grabber = Grabber(board, ik, ak, params, K, R, T, detect, display)
    ok = grabber.run(cy=cy)

    cam.camera_close()
    if ok:
        print('完成：%s 已夹取并放下' % color)
    else:
        print('%s 夹取失败' % color)
        show_status(display, 0)
        return

    # 第三阶段码显示 5 秒后熄灭
    show_status(display, 3)
    time.sleep(5)
    show_status(display, 0)


if __name__ == '__main__':
    main()
