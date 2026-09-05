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
import threading
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
CENTER_TOL = 40
CORRECT_MOVE = 20
camera_lock = threading.Lock()


def lan_ip():
    """获取本机局域网 IP，用于打印视频推流地址。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def restore_travel(board, gripper):
    """恢复 21-24 到官方初始位置，并设置 25 夹爪状态。夹取/放下后回到安全姿态时使用。"""
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def clamp_pulse(v):
    """把舵机脉宽限制在 0-1000，防止手动微调越界。"""
    return max(0, min(1000, int(v)))


def set_servos(board, pulses, order):
    """按指定顺序把多个舵机移动到目标脉宽。pick/place 准备动作使用。"""
    board.bus_servo_set_position(
        2.2, [[sid, int(pulses[sid])] for sid in order])
    time.sleep(2.2)


def parse_adjust(cmd):
    """解析机械臂微调命令，例如 a/d、22w/23s10。"""
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
    """进入机械臂手动微调；回车执行夹取/放下。用于 pick/place 前精调。"""
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
    """准备第一次夹取：先 21，再 22-23-24。"""
    print('pick1：先处理 21，再移动 22-23-24', flush=True)
    set_servos(board, PICK1, [21])
    set_servos(board, PICK1, [22, 23, 24])
    return dict(PICK1)


def pick2_prepare(board):
    """准备第二次夹取：22-23-(24+100) -> 21 -> 24。"""
    print('pick2：22-23-(24+100) -> 21 -> 24', flush=True)
    temp = dict(PICK2)
    temp[24] = PICK2[24] + 100
    set_servos(board, temp, [22, 23, 24])
    set_servos(board, PICK2, [21])
    set_servos(board, PICK2, [24])
    return dict(PICK2)


def place1_prepare(board):
    """准备第一次放下：使用已记录的 21-24 放下脉宽。"""
    print('place1：使用记录的 21-24 放下脉宽', flush=True)
    set_servos(board, PLACE1, [21, 22, 23, 24])
    return dict(PLACE1)


def open_vision(color, min_area):
    """打开摄像头和检测器，返回 (cam, detector)。detector 同时负责视频推流。"""
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    def detector():
        with camera_lock:
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


def video_loop(detector, stop_event):
    """持续取帧并推流，保证视频始终有画面。"""
    while not stop_event.is_set():
        detector()
        time.sleep(0.1)


def lab_view(frame, lab, color):
    """生成 LAB 阈值图，只保留识别到的颜色区域，供 video_lab.mjpeg 显示。"""
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.bitwise_and(frame, frame, mask=mask)


def start_tracking(board, detector):
    """第一次夹取后到第一次放下前：只让 24 号云台跟踪，21 号固定。"""
    tracker = ColorTracker(
        board, detector,
        dead_x=10, dead_y=30,
        start_x=500, start_y=260,
        pan_fixed=True, tilt_fixed=False)
    tracker.start()
    return tracker


def stop_tracking(tracker):
    if tracker is not None:
        tracker.stop()


def move_straight(ik, distance_mm):
    """纯直线移动，不做摄像头或左右微调。"""
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


def move_straight_adjust(ik, detector, distance_mm):
    """直线前进：先看色块偏移，机械足左右微调后再走。"""
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        det = detector()
        if det is not None:
            cx, _ = det['center']
            offset = cx - 320
            direction = '右偏' if offset > 0 else ('左偏' if offset < 0 else '居中')
            print('检测到色块 cx=%d %s%d' % (cx, direction, abs(offset)), flush=True)
            if abs(offset) > CENTER_TOL:
                if offset > 0:
                    ik.right_move(ik.initial_pos, 2, CORRECT_MOVE, MOVE_SPEED, 1)
                    print('色块右偏，机械足右移 %dmm' % CORRECT_MOVE, flush=True)
                else:
                    ik.left_move(ik.initial_pos, 2, CORRECT_MOVE, MOVE_SPEED, 1)
                    print('色块左偏，机械足左移 %dmm' % CORRECT_MOVE, flush=True)
                time.sleep(0.05)
        else:
            print('未发现定位色块', flush=True)
        move = min(100, remaining)
        if forward:
            ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        else:
            ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)
        remaining -= move
        time.sleep(0.05)


def do_turn(ik, act):
    """执行 JSON 中的左转/右转动作。"""
    name = act.get('action')
    if name == 'turn_left':
        ik.turn_left(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)
    elif name == 'turn_right':
        ik.turn_right(ik.initial_pos, 2, int(act.get('angle', 10)), TURN_SPEED, 1)


def do_pick(board, pick_count):
    """执行第 1/2 次夹取准备和手动微调。"""
    if pick_count == 1:
        state = pick1_prepare(board)
    else:
        state = pick2_prepare(board)
    arm_fine_tune(board, state, 'pick')


def do_place(board, place_count):
    """执行第 1/2 次放下准备和手动微调。"""
    if place_count == 1:
        state = place1_prepare(board)
    else:
        state = dict(OFFICIAL_ARM)
    arm_fine_tune(board, state, 'place')


def main():
    """主流程：加载路线，按阶段调用移动、夹取、放下和跟踪。"""
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
    video_stop = threading.Event()
    video_thread = threading.Thread(
        target=video_loop, args=(detector, video_stop), daemon=True)
    video_thread.start()

    if task_server is not None:
        print('视频推流: http://%s:5000/video.mjpeg' % lan_ip(), flush=True)
        print('LAB 推流: http://%s:5000/video_lab.mjpeg' % lan_ip(), flush=True)
        task_server.start_server()

    restore_travel(board, GRIPPER_OPEN)
    print('启动完成，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    pending_forward = 0
    pick_count = 0
    place_count = 0
    tracker = None

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
            if tracker is not None:
                move_straight(ik, pending_forward)
            else:
                move_straight_adjust(ik, detector, pending_forward)
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
                print('第一次夹取完成，开始 24 号云台跟踪', flush=True)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            do_place(board, place_count)
            if place_count == 1 and tracker is not None:
                stop_tracking(tracker)
                tracker = None
                print('第一次放下完成，停止云台跟踪', flush=True)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        if tracker is not None:
            move_straight(ik, pending_forward)
        else:
            move_straight_adjust(ik, detector, pending_forward)

    stop_tracking(tracker)
    video_stop.set()
    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO5 运行结束', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
