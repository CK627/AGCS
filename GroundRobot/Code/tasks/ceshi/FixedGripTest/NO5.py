#!/usr/bin/python3
# coding=utf8
"""NO5：摄像头色块保持直线，pick/place 手动确认，不写固定夹取脉宽。"""

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

try:
    from communication import task_server
except ImportError:
    task_server = None

PICK1 = {21: 815, 22: 230, 23: 645, 24: 355}
PLACE1 = {21: 830, 22: 470, 23: 295, 24: 460}
PICK2 = {21: 735, 22: 610, 23: 205, 24: 410}


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400
MOVE_SPEED = 50
TURN_SPEED = 30
CENTER_TOL = 40
CORRECT_MOVE = 20


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


def detect_target(cam, mapx, mapy, rotate, lab, color, min_area=300):
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


def lab_view(frame, lab, color):
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


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
            cx, _ = det['center']
            print('检测到色块 cx=%d 偏移=%d' % (cx, cx - 320), flush=True)
            align_to_color(ik, det)
        else:
            print('未发现定位色块，持续观察中...', flush=True)
            time.sleep(0.2)
            continue
        move = min(100, remaining)
        if forward:
            ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        else:
            ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        remaining -= move
        time.sleep(0.05)
    return True


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
    if task_server is not None:
        print('视频推流: http://%s:5000/video.mjpeg' % lan_ip(), flush=True)
        print('LAB 推流: http://%s:5000/video_lab.mjpeg' % lan_ip(), flush=True)
        task_server.start_server()

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
            if not straight_with_camera(ik, detector, pending_forward):
                break
            pending_forward = 0

        if name == 'turn_left':
            ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
        elif name == 'turn_right':
            ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            if pick_count == 1:
                arm_state = pick1_prepare(board)
            else:
                arm_state = pick2_prepare(board)
            arm_fine_tune(board, arm_state, 'pick')
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            if place_count == 1:
                arm_state = place1_prepare(board)
            else:
                arm_state = dict(OFFICIAL_ARM)
            arm_fine_tune(board, arm_state, 'place')
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        straight_with_camera(ik, detector, pending_forward)

    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO5 运行结束', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
