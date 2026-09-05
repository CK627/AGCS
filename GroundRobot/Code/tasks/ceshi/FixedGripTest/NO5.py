#!/usr/bin/python3
# coding=utf8
"""NO5：摄像头色块保持直线，pick/place 手动确认，不写固定夹取脉宽。"""

import argparse
import json
import os
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import NO4

from agcs_lib import (
    make_board,
    make_ik,
    load_params,
    load_lab_data,
    load_undistort_maps,
    detect_color,
    correct_camera,
    open_camera,
    capture,
)


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400
MOVE_SPEED = 50
TURN_SPEED = 30
CENTER_TOL = 40
CORRECT_MOVE = 20


def restore_travel(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def detect_target(cam, mapx, mapy, rotate, lab, color, min_area=300):
    f = capture(cam)
    if f is None:
        return None
    frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
    return detect_color(frame, lab, color, min_area=min_area)


def align_to_color(ik, det):
    if det is None:
        return False
    cx, _ = det['center']
    frame_cx = 320
    offset = cx - frame_cx
    if abs(offset) <= CENTER_TOL:
        return True
    if offset > 0:
        ik.right_move(ik.initial_pos, 2, CORRECT_MOVE, MOVE_SPEED, 1)
        print('色块偏右，右移 %d' % CORRECT_MOVE, flush=True)
    else:
        ik.left_move(ik.initial_pos, 2, CORRECT_MOVE, MOVE_SPEED, 1)
        print('色块偏左，左移 %d' % CORRECT_MOVE, flush=True)
    time.sleep(0.05)
    return False


def straight_with_camera(ik, detector, distance_mm):
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        det = detector()
        if det is not None:
            align_to_color(ik, det)
        move = min(100, remaining)
        if forward:
            ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        else:
            ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        remaining -= move
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description='NO5 摄像头直线引导')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=300)
    args = parser.parse_args()

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    def detector():
        return detect_target(cam, mapx, mapy, rotate, lab, args.color, args.min_area)

    restore_travel(board, GRIPPER_OPEN)
    print('启动完成，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    pending_forward = 0
    pick_count = 0
    place_count = 0

    for i, act in enumerate(actions, 1):
        name = act.get('action')
        if name == 'forward':
            pending_forward += int(act.get('step', 100))
            continue
        if name == 'back':
            pending_forward -= int(act.get('step', 50))
            continue

        if pending_forward:
            print('%d/%d 摄像头引导直行 %dmm' % (i, len(actions), pending_forward), flush=True)
            straight_with_camera(ik, detector, pending_forward)
            pending_forward = 0

        if name == 'turn_left':
            ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
        elif name == 'turn_right':
            ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            if pick_count == 1:
                arm_state = NO4.pick1_prepare(board)
            else:
                arm_state = NO4.pick2_prepare(board)
            NO4.arm_fine_tune(board, arm_state, 'pick')
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            if place_count == 1:
                arm_state = NO4.place1_prepare(board)
            else:
                arm_state = dict(OFFICIAL_ARM)
            NO4.arm_fine_tune(board, arm_state, 'place')
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        straight_with_camera(ik, detector, pending_forward)

    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO5 运行结束', flush=True)


if __name__ == '__main__':
    main()
