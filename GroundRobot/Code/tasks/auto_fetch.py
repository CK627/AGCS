#!/usr/bin/python3
# coding=utf8
"""自动取物编排：搜索(search) → 官方色块定点夹取(grab_official)。

- search：找目标 + 走路逼近（只动 1-20 + 云台 21/24）；
- grab_official：搜索到位后直接调用官方 block_fetch.py 色块定点夹取。
"""
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
    from agcs_lib.search import Searcher
    from agcs_lib.grab_official import official_color_grab
    from agcs_lib.sensors import show_status

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--detector', default='color', choices=['color', 'yolo'])
    parser.add_argument('--model', default='models/worm_best.onnx', help='yolo 时的 ONNX 模型路径')
    parser.add_argument('--conf', type=float, default=0.35, help='yolo 置信度阈值')
    args = parser.parse_args()
    color = args.color

    from agcs_lib.logs import setup_logger, action_msg
    logger = setup_logger()
    logger.info('启动 auto_fetch：color=%s', color)

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

    detector = None
    if args.detector == 'yolo':
        from functions.yolo_detect_onnx import YoloDetector
        detector = YoloDetector(args.model, conf=args.conf)

    def detect(min_area=300):
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        if detector is not None:
            return detector.detect(f, min_area=min_area)
        return detect_color(f, lab, color, min_area=min_area)

    stand(ik)
    # 机械臂复位（21/22/23/24 回 reset_pulses，夹爪张开）
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(2)

    # 1) 搜索
    searcher = Searcher(board, ik, ak, params, detect, ultrasonic, display)
    center, cy = searcher.run()
    if center is None:
        logger.info('[search] %s', action_msg('未找到目标', reason='颜色=%s' % color))
        cam.camera_close()
        show_status(display, 0)
        return

    # 寻路刚停，等机器人站稳后再进入官方夹取，避免把未停稳的数据传给 IK。
    time.sleep(1)
    ok = official_color_grab(
        board, ak, params, detect, K, R, T,
        display=display,
        min_area=int(params['vision'].get('min_area', 500)),
        attempts=int(params['grab'].get('attempts', 3)),
    )

    cam.camera_close()
    if ok:
        logger.info('[grab-official] %s', action_msg('完成', action='%s 已夹取' % color))
        show_status(display, 3)
        time.sleep(5)
        show_status(display, 0)
    else:
        logger.info('[grab] %s', action_msg('夹取失败', reason='颜色=%s' % color))
        show_status(display, 0)


if __name__ == '__main__':
    main()
