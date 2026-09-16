#!/usr/bin/python3
# coding=utf8
"""自动捕获（Auto-capture）：摄像头色块左右微调 + 历史平均补偿 + 固定路线 + 夹取/放下。"""

import argparse
import json
import math
import os
import re
import socket
import sys
import threading
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}

GRIPPER_CLOSE = 700
GRIPPER_OPEN = 400
PULL_UP_22 = 450
MOVE_SPEED = 50
TURN_SPEED = 30

COLOR_CENTER_TOL = 3.0
COLOR_CORRECT_MM = 7
LEFT_CORRECT_MM = 10
COLOR_MIN_RADIUS = 0
COLOR_DIRECTION_SIGN = 1

COLOR_HISTORY_WINDOW = 10
COLOR_HISTORY_MIN_MM = 2

# 电压距离补偿：夹取/放下前，电压低就额外前进一点补步态幅度。
VOLTAGE_COMP_LOW_V = 10.9
VOLTAGE_EXTRA_MIN = 40
VOLTAGE_EXTRA_MAX = 80
VOLTAGE_EXTRA_LOW_V = 10.3

camera_lock = threading.Lock()


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def report(**kw):
    if task_server is not None:
        task_server.set_status(**kw)


def norm_heading(deg):
    return round(deg % 360.0, 1)


def advance_pose(pose, yaw_deg, dist_mm):
    yaw_rad = math.radians(yaw_deg)
    pose['x'] += (dist_mm / 1000.0) * math.sin(yaw_rad)
    pose['y'] += (dist_mm / 1000.0) * math.cos(yaw_rad)
    return pose


def pose_dict(pose):
    return {'x': round(pose['x'], 3), 'y': round(pose['y'], 3)}


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


def arm_fine_tune(board, state, kind, pull_up=False, pull_up_pulse=None):
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
    if pull_up:
        pulse = PULL_UP_22 if pull_up_pulse is None else clamp_pulse(pull_up_pulse)
        print('拔起：22 号肩舵机 %d → %d（抬升 %+d）'
              % (state[22], pulse, pulse - state[22]), flush=True)
        board.bus_servo_set_position(1.0, [[22, pulse]])
        time.sleep(1.0)
    restore_travel(board, gripper)


def pick1_prepare(board, pulses=None):
    p = pulses
    print('pick1：先处理 21，再移动 22-23-24', flush=True)
    set_servos(board, p, [21])
    set_servos(board, p, [22, 23, 24])
    return dict(p)


def pick2_prepare(board, pulses=None):
    p = pulses
    print('pick2：22-23 -> 24=500 -> 21 -> 24=JSON值', flush=True)
    set_servos(board, p, [22, 23])
    board.bus_servo_set_position(2.2, [[24, 500]])
    time.sleep(2.2)
    set_servos(board, p, [21])
    set_servos(board, p, [24])
    return dict(p)


def place1_prepare(board, pulses=None):
    p = pulses
    print('place1：使用记录的 21-24 放下脉宽', flush=True)
    set_servos(board, p, [21, 22, 23, 24])
    return dict(p)


def open_vision(color, min_area):
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
    while not stop_event.is_set():
        detector()
        time.sleep(0.1)


def lab_view(frame, lab, color):
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.bitwise_and(frame, frame, mask=mask)


def start_serial_reception(board):
    """只调 SDK 的 enable_reception，让 recv_task 独占串口。"""
    try:
        board.enable_reception()
    except Exception as exc:
        print('enable_reception 失败：%s' % exc, flush=True)
        return False

    last_exc = None
    for i in range(20):
        time.sleep(0.2)
        try:
            v = board.get_battery()
            if v is not None and int(v) > 0:
                print('串口接收已开启，试读电压 %d mV（第 %d 次）'
                      % (int(v), i + 1), flush=True)
                return True
        except Exception as exc:
            last_exc = exc
    if last_exc is not None:
        print('串口接收已开启，但试读电压一直异常：%s' % last_exc, flush=True)
    else:
        print('串口接收已开启，但试读 20 次电压均为 None', flush=True)
    print('  排查方向：', flush=True)
    print('   1) 是否有残留进程占用串口：'
          '`sudo fuser -v /dev/ttyAMA0` 看 PID，kill 掉', flush=True)
    print('   2) SpiderPi.py 是否在后台跑：'
          '`sudo systemctl list-units | grep -i spider`', flush=True)
    print('   3) SDK 的 recv_task 是否已经抛异常退出（看终端上方日志）', flush=True)
    return True


def read_battery_running(board, samples=30, interval=0.05):
    vals = []
    for _ in range(samples):
        try:
            v = board.get_battery()
            if v is not None:
                vals.append(int(v))
        except Exception:
            pass
        time.sleep(interval)
    if not vals:
        return None, None
    vals.sort()
    median = vals[len(vals) // 2]
    avg = sum(vals) / len(vals)
    return median, avg


def extra_distance_mm(voltage):
    """电压从 10.9V 到 10.3V，补偿距离从 40mm 线性增加到 80mm。"""
    if voltage >= VOLTAGE_COMP_LOW_V:
        return 0
    if voltage <= VOLTAGE_EXTRA_LOW_V:
        return VOLTAGE_EXTRA_MAX
    ratio = (VOLTAGE_COMP_LOW_V - voltage) / \
            (VOLTAGE_COMP_LOW_V - VOLTAGE_EXTRA_LOW_V) * 0.75
    return int(VOLTAGE_EXTRA_MIN + (VOLTAGE_EXTRA_MAX - VOLTAGE_EXTRA_MIN) * ratio)


def apply_voltage_compensation(board, ik):
    """夹取/放下前读电压，需要时额外前进补偿距离。"""
    median_mv, avg_mv = read_battery_running(board)
    if median_mv is None:
        print('无法读取电压，跳过电压补偿', flush=True)
        return
    voltage = median_mv / 1000.0
    extra_mm = extra_distance_mm(voltage)
    print('实时电压: 中位数 %.2fV, 平均 %.2fV（补偿阈值 %.1fV）'
          % (voltage, avg_mv / 1000.0, VOLTAGE_COMP_LOW_V), flush=True)
    if extra_mm > 0:
        print('电压补偿：额外前进 %dmm' % extra_mm, flush=True)
        ik.go_forward(ik.initial_pos, 2, extra_mm, MOVE_SPEED, 1)
    else:
        print('电压正常，不额外前进', flush=True)


# ---------------- 颜色历史修正统计 ----------------

def _signed_correction(direction, dist_mm):
    return dist_mm if direction == 'right' else -dist_mm


def _record_color_correction(color_history, direction, dist_mm):
    color_history.append(_signed_correction(direction, dist_mm))
    while len(color_history) > COLOR_HISTORY_WINDOW:
        color_history.pop(0)


def _avg_color_correction(color_history):
    if not color_history:
        return 0.0
    return sum(color_history) / len(color_history)


def _apply_signed_correction(ik, signed_mm, source):
    dist = max(1, int(round(abs(signed_mm))))
    if signed_mm > 0:
        ik.right_move(ik.initial_pos, 2, dist, MOVE_SPEED, 1)
        print('[%s] 按历史平均右移 %dmm' % (source, dist), flush=True)
    else:
        ik.left_move(ik.initial_pos, 2, dist, MOVE_SPEED, 1)
        print('[%s] 按历史平均左移 %dmm' % (source, dist), flush=True)


# ---------------- 颜色微调 ----------------

def color_keep_center(ik, board, detector, tilt, color_state, color_history):
    det = detector()
    if det is None:
        print('未发现定位色块', flush=True)
        tilt['pulse'] = max(100, tilt['pulse'] - 5)
        board.bus_servo_set_position(0.2, [[24, tilt['pulse']]])
        time.sleep(0.2)
        print('24 号下移微调 -> %d' % tilt['pulse'], flush=True)
        avg = _avg_color_correction(color_history)
        if abs(avg) >= COLOR_HISTORY_MIN_MM:
            _apply_signed_correction(ik, avg, '历史平均')
        else:
            print('[历史平均] %.1fmm 低于阈值，跳过' % avg, flush=True)
        return False
    cx = det.get('bbox_center_x', det['center'][0])
    if det.get('radius', 0) < COLOR_MIN_RADIUS:
        print('目标较远，暂不做颜色微调', flush=True)
        return False
    if color_state['ref_cx'] is None:
        color_state['ref_cx'] = cx
        print('设置颜色参考中心 cx=%.1f' % cx, flush=True)
        return True
    offset = cx - color_state['ref_cx']
    if offset > 0:
        direction = '右'
    elif offset < 0:
        direction = '左'
    else:
        direction = '中'
    print('检测到色块 cx=%.1f offset=%+.1f %s' % (cx, offset, direction), flush=True)
    if abs(offset) <= COLOR_CENTER_TOL:
        return False
    if offset > 0:
        if COLOR_DIRECTION_SIGN > 0:
            ik.right_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('色块右偏，机械足右移 %dmm' % COLOR_CORRECT_MM, flush=True)
            _record_color_correction(color_history, 'right', COLOR_CORRECT_MM)
        else:
            ik.left_move(ik.initial_pos, 2, LEFT_CORRECT_MM, MOVE_SPEED, 1)
            print('色块右偏，机械足左移 %dmm（方向反向）' % LEFT_CORRECT_MM, flush=True)
            _record_color_correction(color_history, 'left', LEFT_CORRECT_MM)
    else:
        if COLOR_DIRECTION_SIGN > 0:
            ik.left_move(ik.initial_pos, 2, LEFT_CORRECT_MM, MOVE_SPEED, 1)
            print('色块左偏，机械足左移 %dmm' % LEFT_CORRECT_MM, flush=True)
            _record_color_correction(color_history, 'left', LEFT_CORRECT_MM)
        else:
            ik.right_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('色块左偏，机械足右移 %dmm（方向反向）' % COLOR_CORRECT_MM, flush=True)
            _record_color_correction(color_history, 'right', COLOR_CORRECT_MM)
    time.sleep(0.05)
    return True


def move_one_chunk(ik, move, forward):
    if forward:
        ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
    else:
        ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)


def move_straight_color(ik, board, detector, distance_mm, tilt,
                        color_enabled, color_state, color_history):
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        if color_enabled:
            color_keep_center(ik, board, detector, tilt, color_state, color_history)
        move = min(100, remaining)
        move_one_chunk(ik, move, forward)
        remaining -= move
        time.sleep(0.05)


def fixed_turn(ik, delta_deg):
    remaining = abs(int(delta_deg))
    direction = 1 if delta_deg >= 0 else -1
    while remaining > 0:
        step = min(5, remaining)
        if direction > 0:
            ik.turn_left(ik.initial_pos, 2, step, TURN_SPEED, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, step, TURN_SPEED, 1)
        remaining -= step
        time.sleep(0.08)


def do_pick(board, pick_count, pulses=None, pull_up_pulse=None):
    if pick_count == 1:
        state = pick1_prepare(board, pulses)
    else:
        state = pick2_prepare(board, pulses)
    arm_fine_tune(board, state, 'pick', pull_up=(pick_count == 1),
                  pull_up_pulse=pull_up_pulse)


def do_place(board, place_count, pulses=None):
    if place_count == 1:
        state = place1_prepare(board, pulses)
    else:
        p = pulses or dict(OFFICIAL_ARM)
        print('place2：使用记录的 21-24 放下脉宽', flush=True)
        set_servos(board, p, [21, 22, 23, 24])
        state = dict(p)
    arm_fine_tune(board, state, 'place')


class _Tee(object):
    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for st in self.streams:
            try:
                st.write(text)
            except Exception:
                pass
        return len(text)

    def flush(self):
        for st in self.streams:
            try:
                st.flush()
            except Exception:
                pass


def start_run_log():
    try:
        now = time.localtime()
        day = '%d-%d-%d' % (now.tm_year, now.tm_mon, now.tm_mday)
        folder = os.path.join('/home/pi/spiderpi/logs', day, 'autocapture')
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, '%02d-%02d.log' % (now.tm_hour, now.tm_min))
        handle = open(path, 'w', encoding='utf-8')
    except Exception as exc:
        print('运行日志未启用（%s）' % exc, flush=True)
        return None
    sys.stdout = _Tee(sys.__stdout__, handle)
    sys.stderr = _Tee(sys.__stderr__, handle)
    print('本次运行日志：%s' % path, flush=True)
    return handle


def main():
    global COLOR_HISTORY_WINDOW, COLOR_HISTORY_MIN_MM

    parser = argparse.ArgumentParser(description='NO6 颜色路线运行（历史平均补偿）')
    parser.add_argument('--color', default='red',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=1)
    parser.add_argument('--pull-up', type=int, default=None,
                        help='第一次夹取后拔起的脉宽（22 号肩，默认 %d）' % PULL_UP_22)
    parser.add_argument('--hist-window', type=int, default=COLOR_HISTORY_WINDOW,
                        help='历史修正滑动窗口长度（默认 %d）' % COLOR_HISTORY_WINDOW)
    parser.add_argument('--hist-min-mm', type=float, default=COLOR_HISTORY_MIN_MM,
                        help='应用历史平均补偿的最小阈值 mm（默认 %.1f）'
                             % COLOR_HISTORY_MIN_MM)
    args = parser.parse_args()

    COLOR_HISTORY_WINDOW = args.hist_window
    COLOR_HISTORY_MIN_MM = args.hist_min_mm

    log_file = start_run_log()
    print('配置：颜色=%s，min_area=%d，历史窗口=%d，历史阈值=%.1fmm'
          % (args.color, args.min_area, COLOR_HISTORY_WINDOW, COLOR_HISTORY_MIN_MM),
          flush=True)
    print('  电压距离补偿阈值=%.1fV（仅夹取/放下前生效，无左转/后退补偿）'
          % VOLTAGE_COMP_LOW_V, flush=True)

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    start_serial_reception(board)

    ik = make_ik(board)
    cam, detector = open_vision(args.color, args.min_area)

    video_stop = threading.Event()
    video_thread = threading.Thread(
        target=video_loop, args=(detector, video_stop), daemon=True)
    video_thread.start()

    if task_server is not None:
        task_server.start_server()
    report(state='CAPTURE',
           position_m={'x': 0.0, 'y': 0.0},
           heading_deg=0.0,
           picked_count=0,
           last_task={'task_id': 'capture', 'color': args.color},
           last_result=None,
           message='自动捕获，颜色=%s' % args.color)

    restore_travel(board, GRIPPER_OPEN)
    print('自动捕获启动，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    nominal_yaw = 0.0
    pending_forward = 0
    pick_count = 0
    place_count = 0
    picked_count = 0
    pose = {'x': 0.0, 'y': 0.0}
    tilt = {'pulse': 260}
    color_enabled = True
    first_place_done = False
    turns_after_first_place = 0
    color_state = {'ref_cx': None}
    color_history = []

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
            # 无后退补偿，直接走
            segment_color = color_enabled and not (23 <= i <= 53)
            if color_enabled and not segment_color:
                print('当前步数 %d 在 23-53，暂停颜色微调' % i, flush=True)
            if segment_color:
                color_state['ref_cx'] = None
            dist_mm = pending_forward
            move_straight_color(
                ik, board, detector, pending_forward, tilt,
                segment_color, color_state, color_history)
            pending_forward = 0
            advance_pose(pose, nominal_yaw, dist_mm)
            report(position_m=pose_dict(pose),
                   heading_deg=norm_heading(nominal_yaw),
                   message='直行 %dmm' % dist_mm)

        if name == 'turn_left':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 6:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            # 无左转补偿，按 JSON 角度走
            print('%d/%d 左转 %d' % (i, len(actions), angle), flush=True)
            fixed_turn(ik, angle)
            nominal_yaw += angle
            report(heading_deg=norm_heading(nominal_yaw),
                   message='左转 %d°' % angle)
        elif name == 'turn_right':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 6:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            print('%d/%d 右转 %d' % (i, len(actions), angle), flush=True)
            fixed_turn(ik, -angle)
            nominal_yaw -= angle
            report(heading_deg=norm_heading(nominal_yaw),
                   message='右转 %d°' % angle)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            apply_voltage_compensation(board, ik)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_pick(board, pick_count, pulses, pull_up_pulse=args.pull_up)
            picked_count += 1
            report(picked_count=picked_count,
                   message='第 %d 次夹取完成' % picked_count)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            apply_voltage_compensation(board, ik)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_place(board, place_count, pulses)
            report(message='第 %d 次放下完成' % place_count)
            if place_count == 1:
                color_enabled = False
                first_place_done = True
                turns_after_first_place = 0
                print('第一次放下完成，暂时关闭颜色识别', flush=True)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        segment_color = color_enabled and not (23 <= len(actions) <= 53)
        if segment_color:
            color_state['ref_cx'] = None
        dist_mm = pending_forward
        move_straight_color(
            ik, board, detector, pending_forward, tilt,
            segment_color, color_state, color_history)
        advance_pose(pose, nominal_yaw, dist_mm)

    video_stop.set()
    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('自动捕获运行结束', flush=True)
    report(state='END', last_result='done',
           position_m=pose_dict(pose),
           heading_deg=norm_heading(nominal_yaw),
           picked_count=picked_count,
           message='自动捕获完成')
    time.sleep(5)

    if log_file is not None:
        log_file.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)