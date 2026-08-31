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
    from agcs_lib.grab import Grabber
    from agcs_lib.sensors import show_status

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--detector', default='color', choices=['color', 'yolo'])
    parser.add_argument('--model', default='models/worm_best.onnx', help='yolo 时的 ONNX 模型路径')
    parser.add_argument('--conf', type=float, default=0.35, help='yolo 置信度阈值')
    parser.add_argument('--ratio', type=float, default=0.3, help='夹取占比阈值（目标占画面 320x240 比例 >= 此值夹取）')
    args = parser.parse_args()
    color = args.color

    from agcs_lib.logs import setup_logger, action_msg
    from agcs_lib.params import summarize
    logger = setup_logger()
    logger.info('启动 auto_fetch：color=%s', color)

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik()
    params = load_params()
    params['grab']['grab_area_ratio'] = args.ratio  # 命令行 --ratio 覆盖 yaml 占比阈值
    logger.info('关键生效参数：\n%s', summarize(params))
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    # 红外(VL53L0X)初始化（寻路 + 夹取共享）
    tof = None
    try:
        import board as adafruit_board
        import busio
        import adafruit_vl53l0x
        i2c = busio.I2C(adafruit_board.SCL, adafruit_board.SDA)
        tof = adafruit_vl53l0x.VL53L0X(i2c)
        logger.info('红外初始化成功')
    except Exception as e:
        logger.info('红外初始化失败: %s' % e)
    cam = open_camera()

    detector = None
    if args.detector == 'yolo':
        from functions.yolo_detect_onnx import YoloDetector
        detector = YoloDetector(args.model, conf=args.conf)

    def detect(min_area=150):
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
    searcher = Searcher(board, ik, ak, params, detect, ultrasonic, display, tof=tof)
    center, _ = searcher.run()
    if center is None:
        logger.info('[search] %s', action_msg('未找到目标', reason='颜色=%s' % color))
        cam.camera_close()
        show_status(display, 0)
        return

    # 寻路刚停，停止追踪（夹取阶段不再调 21/24 居中，只判断面积夹取，避免 21 号抽搐）
    searcher.tracker.stop()
    time.sleep(1)
    grabber = Grabber(board, ik, ak, params, K, R, T, detect, display, ultrasonic=ultrasonic, tof=tof)
    ok = grabber.run()
    searcher.stop()  # 夹取结束，清理追踪线程 + 避障线程

    # 夹住保持，人工确认夹稳后再松开（测试阶段：回车前一直夹紧）
    if ok:
        input('已夹住，保持夹紧。敲回车松开夹爪并复位...')

    # 归位到官方初始位置（机械臂复位 + 夹爪张开）
    arm_pulses = params['arm']['reset_pulses']
    board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, int(params['arm'].get('gripper_open', 120))]])
    time.sleep(1.5)

    cam.camera_close()
    if ok:
        logger.info('[grab] %s', action_msg('完成', action='%s 已夹取' % color))
        show_status(display, 3)
        time.sleep(5)
        show_status(display, 0)
    else:
        logger.info('[grab] %s', action_msg('夹取失败', reason='颜色=%s' % color))
        show_status(display, 0)


if __name__ == '__main__':
    main()
