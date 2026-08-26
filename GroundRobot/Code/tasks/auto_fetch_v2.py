#!/usr/bin/python3
# coding=utf8
"""自动取物编排 v2：搜索(search_v2) → 夹取(grab_v2)。"""
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
        load_undistort_maps, make_ultrasonic, make_display, open_camera,
        capture, stand,
    )
    from agcs_lib.search_v2 import SearcherV2
    from agcs_lib.grab_v2 import GrabberV2
    from agcs_lib.sensors import show_status
    from agcs_lib.logs import setup_logger, action_msg

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    args = parser.parse_args()
    color = args.color

    logger = setup_logger()
    logger.info('启动 auto_fetch_v2：color=%s', color)

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
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(2)

    # 1) 搜索
    searcher = SearcherV2(board, ik, ak, params, detect, ultrasonic, display)
    center, cy = searcher.run()
    if center is None:
        logger.info('[search] %s', action_msg('未找到目标', reason='颜色=%s' % color))
        cam.camera_close()
        show_status(display, 0)
        return

    # 2) 夹取
    grabber = GrabberV2(board, ik, ak, params, K, R, T, detect, display, ultrasonic=ultrasonic)
    ok = grabber.run(center=center, cy=cy)

    cam.camera_close()
    if ok:
        logger.info('[grab] %s', action_msg('完成', action='%s 已夹取并放下' % color))
        show_status(display, 3)
        time.sleep(5)
        show_status(display, 0)
    else:
        logger.info('[grab] %s', action_msg('夹取失败', reason='颜色=%s' % color))
        show_status(display, 0)


if __name__ == '__main__':
    main()
