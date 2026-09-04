#!/usr/bin/python3
# coding=utf8
"""NO4：按 JSON 分段，IMU 航向保持 + 夹取/放下微调。"""

import json
import os
import re
import sys
import threading
import time

_PKG_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import make_board, make_ik


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}

PICK1 = {21: 825, 22: 350, 23: 425, 24: 460}
PLACE1 = {21: 825, 22: 350, 23: 425, 24: 460}
PICK2 = {21: 735, 22: 610, 23: 205, 24: 410}

GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400

MOVE_SPEED = 30
TURN_STEP = 5
TURN_SPEED = 30
HEADING_TOL_DEG = 2.0
GYRO_SCALE_LEFT = 1.15
GYRO_SCALE_RIGHT = 1.22


def clamp_pulse(v):
    return max(0, min(1000, int(v)))


def read_gz(board):
    try:
        data = board.get_imu()
        if data is None:
            return None
        return float(data[5])
    except Exception:
        return None


def calibrate_gz_bias(board, samples=200):
    vals = []
    while len(vals) < samples:
        gz = read_gz(board)
        if gz is not None:
            vals.append(gz)
        time.sleep(0.005)
    return sum(vals) / len(vals)


class GyroIntegrator(threading.Thread):
    def __init__(self, board, bias):
        super().__init__(daemon=True)
        self.board = board
        self.bias = bias
        self.yaw = 0.0
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    def run(self):
        last_t = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            gz = read_gz(self.board)
            if gz is not None:
                rate = gz - self.bias
                scale = GYRO_SCALE_LEFT if rate >= 0 else GYRO_SCALE_RIGHT
                with self.lock:
                    self.yaw += rate * dt * scale
            time.sleep(0.005)

    def get_yaw(self):
        with self.lock:
            return self.yaw

    def stop(self):
        self.stop_event.set()


def angle_error(current, target):
    return (target - current + 180.0) % 360.0 - 180.0


def correct_heading(ik, gyro, target):
    while abs(angle_error(gyro.get_yaw(), target)) > HEADING_TOL_DEG:
        err = angle_error(gyro.get_yaw(), target)
        if err > 0:
            ik.turn_left(ik.initial_pos, 2, TURN_STEP, TURN_SPEED, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, TURN_STEP, TURN_SPEED, 1)
        time.sleep(0.08)


def straight_segment(ik, gyro, target_yaw, distance_mm):
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
        correct_heading(ik, gyro, target_yaw)


def turn_delta(ik, gyro, delta_deg):
    target = gyro.get_yaw() + delta_deg
    correct_heading(ik, gyro, target)


def set_servos(board, pulses, order):
    board.bus_servo_set_position(
        1.2, [[sid, int(pulses[sid])] for sid in order])
    time.sleep(1.2)


def restore_travel(board, gripper):
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


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
    board.bus_servo_set_position(0.8, [[25, gripper]])
    time.sleep(0.8)
    restore_travel(board, gripper)


def pick1_prepare(board):
    print('pick1：21保持官方，只动22-23-24', flush=True)
    set_servos(board, PICK1, [22, 23, 24])
    state = dict(OFFICIAL_ARM)
    state.update({22: PICK1[22], 23: PICK1[23], 24: PICK1[24]})
    return state


def pick2_prepare(board):
    print('pick2：22-23-(24+100) -> 21 -> 24', flush=True)
    temp = dict(PICK2)
    temp[24] = PICK2[24] + 100
    set_servos(board, temp, [22, 23, 24])
    set_servos(board, PICK2, [21])
    set_servos(board, PICK2, [24])
    return dict(PICK2)


def place1_prepare(board):
    print('place1：到固定点', flush=True)
    set_servos(board, PLACE1, [22, 23, 21, 24])
    return dict(PLACE1)


def main():
    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    board.enable_reception()
    board.bus_servo_set_position(0.8, [[25, GRIPPER_OPEN]])
    time.sleep(0.8)
    print('启动时已打开 25 号夹爪: %d' % GRIPPER_OPEN, flush=True)

    print('IMU 零漂标定...', flush=True)
    bias = calibrate_gz_bias(board)
    gyro = GyroIntegrator(board, bias)
    gyro.start()
    print('gz bias=%.4f' % bias, flush=True)

    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    pending_forward = 0
    pick_index = 0
    place_index = 0
    arm_state = dict(OFFICIAL_ARM)

    for i, act in enumerate(actions, 1):
        name = act.get('action')

        if name == 'forward':
            pending_forward += int(act.get('step', 100))
            continue
        if name == 'back':
            pending_forward -= int(act.get('step', 50))
            continue

        if pending_forward:
            print('%d/%d 直行 %dmm，保持航向 %.1f°'
                  % (i, len(actions), pending_forward, gyro.get_yaw()), flush=True)
            straight_segment(ik, gyro, gyro.get_yaw(), pending_forward)
            pending_forward = 0

        if name == 'turn_left':
            angle = int(act.get('angle', 90))
            print('%d/%d IMU左转 %d°' % (i, len(actions), angle), flush=True)
            turn_delta(ik, gyro, angle)
        elif name == 'turn_right':
            angle = int(act.get('angle', 90))
            print('%d/%d IMU右转 %d°' % (i, len(actions), angle), flush=True)
            turn_delta(ik, gyro, -angle)
        elif name == 'pick':
            pick_index += 1
            print('%d/%d pick%d' % (i, len(actions), pick_index), flush=True)
            if pick_index == 1:
                arm_state = pick1_prepare(board)
            else:
                arm_state = pick2_prepare(board)
            arm_fine_tune(board, arm_state, 'pick')
        elif name == 'place':
            place_index += 1
            print('%d/%d place%d' % (i, len(actions), place_index), flush=True)
            if place_index == 1:
                arm_state = place1_prepare(board)
            else:
                arm_state = dict(OFFICIAL_ARM)
            arm_fine_tune(board, arm_state, 'place')
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        straight_segment(ik, gyro, gyro.get_yaw(), pending_forward)

    gyro.stop()
    ik.stand(ik.initial_pos, t=500)
    print('NO4 运行结束', flush=True)


if __name__ == '__main__':
    main()
