#!/usr/bin/python3
# coding=utf8
"""单个颜色方块：稳定识别 → 朝向对齐（可选）→ 夹取 → 放下 → 复位。

用法：
    python3 tasks/grab_block.py --color red
    python3 tasks/grab_block.py --color red --align
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
        make_board, make_arm_ik, load_params, load_lab_data, load_block_params,
        detect_color, correct_camera, load_undistort_maps, pixel_to_arm_coord,
        block_orientation, orientation_error, make_ultrasonic, dist_cm,
        open_camera, capture,
    )

    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue', 'yellow'])
    parser.add_argument('--align', action='store_true', help='夹取前先转舵机21把方块转正')
    args = parser.parse_args()
    color = args.color

    board = make_board()
    ak = make_arm_ik()
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    reach_x = float(params['walk'].get('reach_x', 8.0))
    reach_y = float(params['walk'].get('reach_y', 24.0))
    ultrasonic = make_ultrasonic()
    align_cfg = params.get('align', {})
    do_align = args.align or align_cfg.get('enabled', False)

    ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
    time.sleep(2)
    board.bus_servo_set_position(0.5, [[25, 120]])
    cam = open_camera()

    def detect(min_area=300):
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, color, min_area=min_area)

    def stable_detect(frames=80, need=5, jitter=5):
        stable = 0
        old = None
        for _ in range(frames):
            r = detect()
            if r is None:
                stable = 0
                old = None
                time.sleep(0.05)
                continue
            c = r['center']
            if old is not None and abs(c[0] - old[0]) < jitter and abs(c[1] - old[1]) < jitter:
                stable += 1
            else:
                stable = 0
            old = c
            if stable >= need:
                return c
            time.sleep(0.05)
        return None

    def align_block():
        ref = float(align_cfg.get('ref_angle', 0.0))
        ratio = float(align_cfg.get('servo21_ratio', 4.1667))
        direction = int(align_cfg.get('servo21_dir', 1))
        tol = float(align_cfg.get('angle_tol', 3.0))
        max_step = float(align_cfg.get('max_step', 60))
        max_iter = int(align_cfg.get('max_iter', 8))
        settle = float(align_cfg.get('settle_ms', 400)) / 1000.0
        min_area = int(align_cfg.get('min_area', 300))
        pulse = int(align_cfg.get('start_pulse', 500))
        for i in range(max_iter):
            r = detect(min_area=min_area)
            if r is None:
                print('[align] 第%d次未检测到目标，跳过对齐' % (i + 1))
                return False
            angle, _pts = block_orientation(r['contour'])
            if angle is None:
                print('[align] 朝向角不可用，跳过对齐')
                return False
            err = orientation_error(angle, ref)
            print('[align] #%d angle=%.1f ref=%.1f err=%+.1f pulse=%d'
                  % (i + 1, angle, ref, err, pulse))
            if abs(err) <= tol:
                print('[align] 已对齐')
                return True
            delta = direction * ratio * err
            delta = max(-max_step, min(max_step, delta))
            pulse = max(0, min(1000, pulse + int(round(delta))))
            board.bus_servo_set_position(0.3, [[21, pulse]])
            time.sleep(settle)
        print('[align] 达到最大迭代次数，未完全对齐')
        return False

    def grab(center):
        """官方 block_fetch.py move() 的夹取动作（夹取 z=5、夹爪 550）。"""
        x, y = pixel_to_arm_coord(K, R, T, center, initial_coord=(0, 15, 5))
        board.bus_servo_set_position(0.5, [[25, 120]])
        ak.setPitchRangeMoving((x, y, 5), -90, -90, 100, 1)
        time.sleep(3)
        board.bus_servo_set_position(0.5, [[25, 550]])
        time.sleep(2)
        ak.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1.5)
        time.sleep(1.5)
        ak.setPitchRangeMoving((12, 24, -5), -90, -90, 100, 1)
        time.sleep(1)
        board.bus_servo_set_position(0.5, [[25, 120]])
        time.sleep(0.5)
        ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 1.5)
        time.sleep(1.5)

    center = stable_detect()
    if center is None:
        cam.camera_close()
        print('%s 未检测到（或位置不稳定）' % color)
        return

    if do_align:
        align_block()
        center = stable_detect()
        if center is None:
            cam.camera_close()
            print('%s 对齐后未检测到目标' % color)
            return

    x, y = pixel_to_arm_coord(K, R, T, center, initial_coord=(0, 15, 5))
    print('%s 稳定像素=%s -> 夹取坐标 x=%.1f y=%.1f' % (color, center, x, y))

    r2 = detect()
    if r2 is not None:
        x, y = pixel_to_arm_coord(K, R, T, r2['center'], initial_coord=(0, 15, 5))
        print('夹前复检 x=%.1f y=%.1f' % (x, y))

    if abs(x) > reach_x or y > reach_y or y < 6:
        print('[grab] 超出可及范围 x=%.1f y=%.1f，请放近/放正后重试' % (x, y))
        cam.camera_close()
        return

    d = dist_cm(ultrasonic)
    print('[grab] 超声波实测约 %.1fcm，视觉 y=%.1fcm' % (d, y))

    grab(center)
    cam.camera_close()
    print('完成：%s 已夹取并放下' % color)


if __name__ == '__main__':
    main()
