#!/usr/bin/python3
# coding=utf8
"""NO5：分阶段清晰的固定路线运行脚本。

- 摄像头推流始终开启；
- 第一次夹取前：只走路线，不跟踪；
- 第一次夹取后到第一次放下前：启动摄像头颜色跟踪；
- 第一次放下后：关闭跟踪；
- pick/place 使用独立函数处理。
"""

import argparse
import json
import os
import re
import socket
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

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
from agcs_lib.tracker import ColorTracker

try:
    from communication import task_server
except ImportError:
    task_server = None


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}
PICK1 = {21: 840, 22: 230, 23: 645, 24: 355}
PLACE1 = {21: 830, 22: 470, 23: 295, 24: 460}
PICK2 = {21: 735, 22: 610, 23: 205, 24: 410}

GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400
MOVE_SPEED = 50
TURN_SPEED = 30


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def restore_travel(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def clamp_pulse(v):
    return max(0, min(1000, int(v)))


def set_servos(board, pulses, order):
    board.bus_servo_set_position(
        2.2, [[sid, int(pulses[sid])] for sid in order])
    time.sleep(2.2)


def parse_adjust(cmd):
    cmd = cmd.strip().lower()
    if not cmd:
        return None
    if cmd[0] in ('a', 'd'):
        return 21, (-1 if cmd[0] == 'a' else 1), (int(cmd[1:]) if cmd[1:] else 5)
    m = re.match(r'^(22|23|24)([ws])(\d*)$', cmd)
    if not m:
        return None
    return int(m.group(1)), (1 if m.group(2) == 'w' else -1), (int(m.group(3)) if m.group(3) else 5)


def arm_fine_tune(board, state, kind):
    print('机械臂微调：回车=%s，c=退出' % ('夹取' if kind == 'pick' else '放下'), flush=True)
    while True:
        print('当前 21=%d 22=%d 23=%d 24=%d'
              % (state[21], state[22], state[23], state[24]), flush=True)
        cmd = input('arm> ').strip().lower()
        if cmd == '':
            break
        if cmd == 'c':
            print('手动退出', flush=True)
            sys.exit(0)
        parsed = parse_adjust(cmd)
        if parsed is None:
            print('命令错误', flush=True)
            continue
        servo, delta, amount = parsed
        state[servo] = clamp_pulse(state[servo] + delta * amount)
        board.bus_servo_set_position(0.2, [[servo, state[servo]]])
        time.sleep(0.1)

    gripper = GRIPPER_CLOSE if kind == 'pick' else GRIPPER_OPEN
    board.bus_servo_set_position(2.0, [[25, gripper]])
    time.sleep(2.0)
    restore_travel(board, gripper)


def pick1_prepare(board):
    print('pick1：先处理 21，再移动 22-23-24', flush=True)
    set_servos(board, PICK1, [21])
    set_servos(board, PICK1, [22, 23, 24])
    return dict(PICK1)


def pick2_prepare(board):
    print('pick2：22-23-(24+100) -> 21 -> 24', flush=True)
    temp = dict(PICK2)
    temp[24] = PICK2[24] + 100
    set_servos(board, temp, [22, 23, 24])
    set_servos(board, PICK2, [21])
    set_servos(board, PICK2, [24])
    return dict(PICK2)


def place1_prepare(board):
    print('place1：21 保持官方初始位，只动 22-23-24', flush=True)
    set_servos(board, PLACE1, [22, 23, 24])
    state = dict(OFFICIAL_ARM)
    state.update({22: PLACE1[22], 23: PLACE1[23], 24: PLACE1[24]})
    return state


def open_vision(color, min_area):
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    def detector():
        f = capture(cam)
        if f is None:
            return None
        frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        result = detect_color(frame, lab, color, min_area=min_area)
        if result is not None:
            cx, cy = result['center']
            cv2.circle(frame, (cx, cy), int(result.get('radius', 20)), (0, 255, 0), 2)
            cv2.putText(frame, color, (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if task_server is not None:
            task_server.publish_frame(frame, max_fps=10.0)
            task_server.publish_lab_frame(lab_view(frame, lab, color), max_fps=10.0)
        return result

    return cam, detector


def lab_view(frame, lab, color):
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.bitwise_and(frame, frame, mask=mask)


def start_tracking(board, detector):
    tracker = ColorTracker(
        board, detector,
        dead_x=10, dead_y=60,
        start_x=500, start_y=260,
        tilt_fixed=True)
    tracker.x_pid.setKp(0.25)
    tracker.start()
    return tracker


def stop_tracking(tracker):
    if tracker is not None:
        tracker.stop()


def move_straight(ik, distance_mm):
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        move = min(100, remaining)
        if forward:
            ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        else:
            ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        remaining -= move
        time.sleep(0.05)


def do_turn(ik, act):
    name = act.get('action')
    if name == 'turn_left':
        ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
    elif name == 'turn_right':
        ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)


def do_pick(board, pick_count):
    if pick_count == 1:
        state = pick1_prepare(board)
    else:
        state = pick2_prepare(board)
    arm_fine_tune(board, state, 'pick')


def do_place(board, place_count):
    if place_count == 1:
        state = place1_prepare(board)
    else:
        state = dict(OFFICIAL_ARM)
    arm_fine_tune(board, state, 'place')


def main():
    parser = argparse.ArgumentParser(description='NO5 分阶段路线运行')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=50)
    args = parser.parse_args()

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    cam, detector = open_vision(args.color, args.min_area)

    if task_server is not None:
        print('视频推流: http://%s:5000/video.mjpeg' % lan_ip(), flush=True)
        print('LAB 推流: http://%s:5000/video_lab.mjpeg' % lan_ip(), flush=True)
        task_server.start_server()

    restore_travel(board, GRIPPER_OPEN)
    print('启动完成，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    tracker = None
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
            print('%d/%d 直行 %dmm' % (i, len(actions), pending_forward), flush=True)
            move_straight(ik, pending_forward)
            pending_forward = 0

        if name in ('turn_left', 'turn_right'):
            print('%d/%d %s' % (i, len(actions), name), flush=True)
            do_turn(ik, act)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            do_pick(board, pick_count)
            if pick_count == 1 and tracker is None:
                tracker = start_tracking(board, detector)
                print('第一次夹取完成，开始摄像头跟踪到放下点', flush=True)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            do_place(board, place_count)
            if place_count == 1 and tracker is not None:
                stop_tracking(tracker)
                tracker = None
                print('第一次放下完成，停止摄像头跟踪', flush=True)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        move_straight(ik, pending_forward)

    stop_tracking(tracker)
    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO5 运行结束', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
