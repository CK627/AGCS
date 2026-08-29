#!/usr/bin/python3
# coding=utf8
"""临时测试双策略夹取：mapped（低处映射）/ fixed（高处固定点）。

用法：
    python3 CS-grab-alt.py --color blue --mode mapped
    python3 CS-grab-alt.py --color blue --mode fixed
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
from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.logs import setup_logger, action_msg


def main():
    parser = argparse.ArgumentParser(description='双策略夹取临时测试')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--mode', default='mapped', choices=['mapped', 'fixed'])
    parser.add_argument('--min-area', type=int, default=300)
    args = parser.parse_args()

    logger = setup_logger()
    logger.info('[CS-grab-alt] %s', action_msg('启动', action='mode=%s color=%s' % (args.mode, args.color)))

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

    def detect():
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=args.min_area)

    def stable(frames=60, need=5):
        st = 0
        old = None
        last = None
        for _ in range(frames):
            r = detect()
            if r is None:
                st = 0
                old = None
                time.sleep(0.05)
                continue
            c = r['center']
            if old is not None and abs(c[0] - old[0]) < 5 and abs(c[1] - old[1]) < 5:
                st += 1
            else:
                st = 0
            old = c
            last = r
            if st >= need:
                return last
            time.sleep(0.05)
        return None

    stand(ik)
    gc = params.get('grab', {})
    arm = params['arm']
    open_pulse = int(arm.get('gripper_open', 120))
    close_pulse = int(arm.get('gripper_close', 550))
    detect_pose = [float(v) for v in arm.get('detect_pose', [0, 15, 5])]
    pick_z = float(arm.get('pick_z', 5))

    if args.mode == 'mapped':
        ak.setPitchRangeMoving(tuple(detect_pose), -90, -90, 100, 2)
        time.sleep(2)
        r = stable()
        if r is None:
            logger.info('[CS-grab-alt] %s', action_msg('mapped未检测到目标'))
            cam.camera_close()
            return
        x, y = pixel_to_arm_coord(K, R, T, r['center'], initial_coord=(detect_pose[0], detect_pose[1]))
        logger.info('[CS-grab-alt] %s', action_msg('mapped坐标', action='x=%.1f y=%.1f z=%.1f' % (x, y, pick_z)))
        board.bus_servo_set_position(0.5, [[25, open_pulse]])
        ak.setPitchRangeMoving((x, y, pick_z), -90, -90, 100, 1)
        time.sleep(3)
        board.bus_servo_set_position(0.5, [[25, close_pulse]])
        time.sleep(2)
        ak.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1.5)
        time.sleep(1.5)
        ak.setPitchRangeMoving((12, 24, -5), -90, -90, 100, 1)
        time.sleep(1)
        board.bus_servo_set_position(0.5, [[25, open_pulse]])
        time.sleep(0.5)
        ak.setPitchRangeMoving(tuple(detect_pose), -90, -90, 100, 1.5)
        time.sleep(1.5)
    else:
        tilt_pose = [float(v) for v in gc.get('tilt_pose', [0, 15, 18])]
        tilt_pitch = int(gc.get('tilt_pitch', -35))
        cx_min = int(gc.get('tilt_cx_min', 250))
        cx_max = int(gc.get('tilt_cx_max', 390))
        cy_min = int(gc.get('tilt_cy_min', 330))
        cy_max = int(gc.get('tilt_cy_max', 430))
        fixed_grab = [float(v) for v in gc.get('fixed_grab', [0, 25])]
        fixed_grab_z = float(gc.get('fixed_grab_z', 5))
        ak.setPitchRangeMoving(tuple(tilt_pose), tilt_pitch, -90, 100, 2)
        time.sleep(2)
        for _ in range(10):
            r = stable(frames=30, need=3)
            if r is None:
                time.sleep(0.3)
                continue
            cx, cy = r['center']
            logger.info('[CS-grab-alt] %s', action_msg('fixed检测', action='cx=%d cy=%d' % (cx, cy)))
            if cx_min <= cx <= cx_max and cy_min <= cy <= cy_max:
                fx, fy = fixed_grab
                logger.info('[CS-grab-alt] %s', action_msg('fixed夹取', action='fx=%.1f fy=%.1f z=%.1f' % (fx, fy, fixed_grab_z)))
                board.bus_servo_set_position(0.5, [[25, open_pulse]])
                ak.setPitchRangeMoving((fx, fy + 2, fixed_grab_z), -90, -90, 100, 2)
                time.sleep(2)
                board.bus_servo_set_position(0.5, [[25, close_pulse]])
                time.sleep(0.8)
                ak.setPitchRangeMoving((fx, fy, 8), -90, -90, 100, 1)
                time.sleep(1)
                ak.setPitchRangeMoving((12, 24, -5), -90, -90, 100, 1.5)
                time.sleep(1.5)
                board.bus_servo_set_position(0.5, [[25, open_pulse]])
                time.sleep(0.5)
                ak.setPitchRangeMoving(tuple(detect_pose), -90, -90, 100, 1.5)
                time.sleep(1.5)
                break
            time.sleep(0.3)

    cam.camera_close()
    logger.info('[CS-grab-alt] %s', action_msg('完成', action='mode=%s' % args.mode))


if __name__ == '__main__':
    main()
