#!/usr/bin/python3
# coding=utf8
"""NO6：IMU 航向保持 + 摄像头色块左右微调。完全独立，不依赖 NO5。"""

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

try:
    from communication import task_server
except ImportError:
    task_server = None


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}  # 机械臂官方初始脉宽

GRIPPER_CLOSE = 700  # 夹取时 25 号夹爪闭合的脉宽，越大夹得越紧
GRIPPER_OPEN = 400   # 放下时 25 号夹爪打开的脉宽，越小张得越开
MOVE_SPEED = 50      # 六足直线前进/后退的速度，越大走得越快
TURN_SPEED = 30      # 六足左转/右转的速度，越大转得越快
GYRO_SCALE_LEFT = 1.15   # IMU 左转时陀螺仪积分修正比例
GYRO_SCALE_RIGHT = 1.15  # IMU 右转时陀螺仪积分修正比例
HEADING_TOL_DEG = 2.0    # 航向误差容忍范围，单位：度；越小越严格
COLOR_CENTER_TOL = 3.0   # 色块中心允许偏差，单位：像素；偏差小于该值不调整
COLOR_CORRECT_MM = 7     # 颜色左右微调每次移动的距离，单位：毫米
COLOR_MIN_RADIUS = 0     # 色块半径小于该值时暂不进行颜色微调
COLOR_DIRECTION_SIGN = 1  # 颜色修正方向：1=默认，-1=左右指令反向后使用
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
    """恢复 21-24 到官方初始位置，并设置 25 夹爪状态。"""
    board.bus_servo_set_position(
        1.5, [[sid, OFFICIAL_ARM[sid]] for sid in [22, 23, 21, 24]])
    time.sleep(1.5)
    board.bus_servo_set_position(0.5, [[25, gripper]])
    time.sleep(0.5)


def clamp_pulse(v):
    """限制舵机脉宽在 0-1000。"""
    return max(0, min(1000, int(v)))


def set_servos(board, pulses, order):
    """按指定顺序移动多个舵机到目标脉宽。"""
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
    """机械臂手动微调，回车执行夹取/放下。"""
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
    time.sleep(0.5)
    restore_travel(board, gripper)


def pick1_prepare(board, pulses=None):
    """准备第一次夹取：先 21，再 22-23-24。"""
    p = pulses
    print('pick1：先处理 21，再移动 22-23-24', flush=True)
    set_servos(board, p, [21])
    set_servos(board, p, [22, 23, 24])
    return dict(p)


def pick2_prepare(board, pulses=None):
    """准备第二次夹取：22-23-(24+100) -> 21 -> 24。"""
    p = pulses
    print('pick2：22-23-(24+100) -> 21 -> 24', flush=True)
    temp = dict(p)
    temp[24] = p[24] + 100
    set_servos(board, temp, [22, 23, 24])
    set_servos(board, p, [21])
    set_servos(board, p, [24])
    return dict(p)


def place1_prepare(board, pulses=None):
    """准备第一次放下：使用记录的 21-24 放下脉宽。"""
    p = pulses
    print('place1：使用记录的 21-24 放下脉宽', flush=True)
    set_servos(board, p, [21, 22, 23, 24])
    return dict(p)


def open_vision(color, min_area):
    """打开摄像头，返回 (cam, detector)。detector 负责检测和推流。"""
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
        frame = cv2.GaussianBlur(frame, (7, 7), 0)
        result = detect_color(frame, lab, color, min_area=min_area)
        if result is not None:
            x, y, w, h = cv2.boundingRect(result['contour'])
            ul, ur = x, x + w
            result['bbox_center_x'] = (ul + ur) / 2.0
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
    """后台持续取帧推流，保证视频始终有画面。"""
    while not stop_event.is_set():
        detector()
        time.sleep(0.1)


def lab_view(frame, lab, color):
    """生成 LAB 阈值图，只保留识别到的颜色区域。"""
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.bitwise_and(frame, frame, mask=mask)


def read_gz(board):
    """读取 IMU 的 gz 角速度。"""
    try:
        data = board.get_imu()
        if data is None:
            return None
        return float(data[5])
    except Exception:
        return None


def init_imu(board):
    """初始化 IMU：开启接收，标定 gz 零漂。"""
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
    """更新 IMU 偏航角。"""
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
    """计算两个航向角的最小误差，范围 -180 到 180。"""
    return (target - current + 180.0) % 360.0 - 180.0


def color_keep_center(ik, board, detector, tilt, color_state):
    """只根据色块左右中心，做机械足左右微调。"""
    det = detector()
    if det is None:
        print('未发现定位色块', flush=True)
        tilt['pulse'] = max(100, tilt['pulse'] - 5)
        board.bus_servo_set_position(0.2, [[24, tilt['pulse']]])
        time.sleep(0.2)
        print('24 号下移微调 -> %d' % tilt['pulse'], flush=True)
        return
    cx = det.get('bbox_center_x', det['center'][0])
    if det.get('radius', 0) < COLOR_MIN_RADIUS:
        print('目标较远，暂不做颜色微调，依赖 IMU 保持航向', flush=True)
        return
    if color_state['ref_cx'] is None:
        color_state['ref_cx'] = cx
        print('设置颜色参考中心 cx=%.1f' % cx, flush=True)
        return
    offset = cx - color_state['ref_cx']
    if offset > 0:
        direction = '右'
    elif offset < 0:
        direction = '左'
    else:
        direction = '中'
    print('检测到色块 cx=%.1f offset=%+.1f %s' % (cx, offset, direction), flush=True)
    if abs(offset) <= COLOR_CENTER_TOL:
        return
    if offset > 0:
        if COLOR_DIRECTION_SIGN > 0:
            ik.right_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('色块右偏，机械足右移 %dmm' % COLOR_CORRECT_MM, flush=True)
        else:
            ik.left_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('色块右偏，机械足左移 %dmm（方向反向）' % COLOR_CORRECT_MM, flush=True)
    else:
        if COLOR_DIRECTION_SIGN > 0:
            ik.left_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('色块左偏，机械足左移 %dmm' % COLOR_CORRECT_MM, flush=True)
        else:
            ik.right_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('色块左偏，机械足右移 %dmm（方向反向）' % COLOR_CORRECT_MM, flush=True)
    time.sleep(0.05)


def move_one_chunk(ik, move, forward):
    """只走一小段前进或后退。"""
    if forward:
        ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
    else:
        ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)


def move_straight_imu_color(ik, board, detector, imu_state, target_yaw, distance_mm, tilt, color_enabled, color_state):
    """直线阶段：IMU 保持航向 + 颜色左右微调 + 前进。"""
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
        if color_enabled:
            color_keep_center(ik, board, detector, tilt, color_state)
        move = min(100, remaining)
        move_one_chunk(ik, move, forward)
        remaining -= move
        time.sleep(0.05)


def imu_turn(ik, board, imu_state, delta_deg):
    """IMU 闭环转弯到目标角度。"""
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


def do_pick(board, pick_count, pulses=None):
    """执行第 1/2 次夹取。"""
    if pick_count == 1:
        state = pick1_prepare(board, pulses)
    else:
        state = pick2_prepare(board, pulses)
    arm_fine_tune(board, state, 'pick')


def do_place(board, place_count, pulses=None):
    """执行第 1/2 次放下。"""
    if place_count == 1:
        state = place1_prepare(board, pulses)
    else:
        state = pulses or dict(OFFICIAL_ARM)
    arm_fine_tune(board, state, 'place')


def main():
    """主流程：按 JSON 调用移动、转弯、夹取和放下。"""
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
    print('NO6 启动，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    target_yaw = 0.0
    pending_forward = 0
    pick_count = 0
    place_count = 0
    tilt = {'pulse': 260}
    color_enabled = True
    first_place_done = False
    turns_after_first_place = 0
    color_state = {'ref_cx': None}

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
                ik, board, detector, imu_state, target_yaw, pending_forward, tilt, color_enabled, color_state)
            pending_forward = 0

        if name == 'turn_left':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 2:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            print('%d/%d IMU左转 %d' % (i, len(actions), angle), flush=True)
            imu_turn(ik, board, imu_state, angle)
            target_yaw = imu_state['yaw']
        elif name == 'turn_right':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 2:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            print('%d/%d IMU右转 %d' % (i, len(actions), angle), flush=True)
            imu_turn(ik, board, imu_state, -angle)
            target_yaw = imu_state['yaw']
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_pick(board, pick_count, pulses)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_place(board, place_count, pulses)
            if place_count == 1:
                color_enabled = False
                first_place_done = True
                turns_after_first_place = 0
                print('第一次放下完成，暂时关闭颜色识别', flush=True)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        move_straight_imu_color(
            ik, board, detector, imu_state, target_yaw, pending_forward, tilt, color_enabled, color_state)

    video_stop.set()
    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO6 运行结束', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
