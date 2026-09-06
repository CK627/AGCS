#!/usr/bin/python3
# coding=utf8
"""NO6：IMU 保持航向 + 摄像头色块左右微调，分函数清晰。"""

import argparse
import json
import os
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

import NO5


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

MOVE_SPEED = 50
TURN_SPEED = 30
GYRO_SCALE_LEFT = 1.15
GYRO_SCALE_RIGHT = 1.15
HEADING_TOL_DEG = 2.0
COLOR_CENTER_TOL = 40
COLOR_CORRECT_MM = 10


def read_gz(board):
    try:
        data = board.get_imu()
        if data is None:
            return None
        return float(data[5])
    except Exception:
        return None


def init_imu(board):
    board.enable_reception()
    vals = []
    while len(vals) < 200:
        gz = read_gz(board)
        if gz is not None:
            vals.append(gz)
        time.sleep(0.005)
    bias = sum(vals) / len(vals)
    return {'bias': bias, 'yaw': 0.0, 'last_t': time.monotonic()}


def update_imu(state, board):
    now = time.monotonic()
    dt = now - state['last_t']
    state['last_t'] = now
    gz = read_gz(board)
    if gz is None:
        return
    rate = gz - state['bias']
    scale = GYRO_SCALE_LEFT if rate >= 0 else GYRO_SCALE_RIGHT
    state['yaw'] += rate * dt * scale


def angle_error(current, target):
    return (target - current + 180.0) % 360.0 - 180.0


def color_keep_center(ik, detector):
    """只做色块左右微调。"""
    det = detector()
    if det is None:
        print('未发现定位色块', flush=True)
        return
    cx = det.get('bbox_center_x', det['center'][0])
    offset = cx - 320
    if abs(offset) <= COLOR_CENTER_TOL:
        return
    if offset > 0:
        ik.right_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
        print('色块右偏，机械足右移 %dmm' % COLOR_CORRECT_MM, flush=True)
    else:
        ik.left_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
        print('色块左偏，机械足左移 %dmm' % COLOR_CORRECT_MM, flush=True)
    time.sleep(0.05)


def move_one_chunk(ik, move, forward):
    if forward:
        ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
    else:
        ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)


def move_straight_imu_color(ik, board, detector, imu_state, target_yaw, distance_mm):
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        update_imu(imu_state, board)
        if abs(angle_error(imu_state['yaw'], target_yaw)) > HEADING_TOL_DEG:
            err = angle_error(imu_state['yaw'], target_yaw)
            if err > 0:
                ik.turn_left(ik.initial_pos, 2, 1, TURN_SPEED, 1)
            else:
                ik.turn_right(ik.initial_pos, 2, 1, TURN_SPEED, 1)
            time.sleep(0.05)
        color_keep_center(ik, detector)
        move = min(100, remaining)
        move_one_chunk(ik, move, forward)
        remaining -= move
        time.sleep(0.05)


def imu_turn(ik, board, imu_state, delta_deg):
    target = imu_state['yaw'] + delta_deg
    for _ in range(40):
        update_imu(imu_state, board)
        if abs(angle_error(imu_state['yaw'], target)) <= HEADING_TOL_DEG:
            break
        err = angle_error(imu_state['yaw'], target)
        if err > 0:
            ik.turn_left(ik.initial_pos, 2, 5, TURN_SPEED, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, 5, TURN_SPEED, 1)
        time.sleep(0.08)


def main():
    parser = argparse.ArgumentParser(description='NO6 IMU+颜色路线运行')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=1)
    args = parser.parse_args()

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    imu_state = init_imu(board)
    cam, detector = NO5.open_vision(args.color, args.min_area)

    video_stop = threading.Event()
    video_thread = threading.Thread(
        target=NO5.video_loop, args=(detector, video_stop), daemon=True)
    video_thread.start()

    if NO5.task_server is not None:
        print('视频推流: http://%s:5000/video.mjpeg' % NO5.lan_ip(), flush=True)
        print('LAB 推流: http://%s:5000/video_lab.mjpeg' % NO5.lan_ip(), flush=True)
        NO5.task_server.start_server()

    NO5.restore_travel(board, NO5.GRIPPER_OPEN)
    print('NO6 启动，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    target_yaw = 0.0
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
            move_straight_imu_color(
                ik, board, detector, imu_state, target_yaw, pending_forward)
            pending_forward = 0

        if name == 'turn_left':
            angle = int(act.get('angle', 90))
            print('%d/%d IMU左转 %d' % (i, len(actions), angle), flush=True)
            imu_turn(ik, board, imu_state, angle)
        elif name == 'turn_right':
            angle = int(act.get('angle', 90))
            print('%d/%d IMU右转 %d' % (i, len(actions), angle), flush=True)
            imu_turn(ik, board, imu_state, -angle)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            NO5.do_pick(board, pick_count)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            NO5.do_place(board, place_count)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        move_straight_imu_color(
            ik, board, detector, imu_state, target_yaw, pending_forward)

    video_stop.set()
    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO6 运行结束', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
