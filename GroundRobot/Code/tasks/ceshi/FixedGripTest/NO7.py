#!/usr/bin/python3
# coding=utf8
"""NO7：两级对准夹取。

阶段一：完全沿用 NO6 的「颜色左右微调 + IMU 航向保持」导航，把机器人带到固定夹取点附近。
阶段二：到夹取点后，用真实模型（默认 models/base.pt）检测目标矩形框，再通过标定好的
        雅可比矩阵把像素误差换算成机械臂 21-24 号舵机增量，自动做精对准。
对准结束后仍保留手动微调 + 回车夹取 / c 退出，验证流程不变。

标定（一次性，每个夹取点做一次）：
    python3 NO7.py --calibrate 1 --model models/base.pt --color red
    python3 NO7.py --calibrate 2 --model models/base.pt --color red
运行：
    python3 NO7.py --model models/base.pt --color red
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
import numpy as np

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


# ---------- 路径 ----------
ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'fixed_route.json')


def calib_path(pick_num):
    """返回第 pick_num 次夹取的雅可比标定文件路径。"""
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'calib_pick%d.json' % pick_num)


# ---------- 机械臂 ----------
OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}  # 机械臂官方初始脉宽
GRIPPER_CLOSE = 700  # 25 号夹爪闭合脉宽
GRIPPER_OPEN = 400   # 25 号夹爪张开脉宽
ARM_SERVOS = [21, 22, 23, 24]   # 机械臂 4 个舵机


# ---------- 六足运动 ----------
MOVE_SPEED = 50  # 直线前进/后退速度
TURN_SPEED = 30  # 左转/右转速度


# ---------- IMU ----------
GYRO_SCALE_LEFT = 1.177    # 左转陀螺仪积分修正比例
GYRO_SCALE_RIGHT = 1.199   # 右转陀螺仪积分修正比例
HEADING_TOL_DEG = 1.0      # 转弯后航向误差容忍，越小越严格
LEFT_TURN_TOL_DEG = 1.0    # 直线阶段允许左偏多少才左转
RIGHT_TURN_TOL_DEG = 8.0   # 直线阶段允许右偏多少才右转
IMU_STRAIGHT_STEP = 1      # 直线阶段 IMU 每次修正角度
ENABLE_IMU_STRAIGHT = True  # 直线阶段是否启用 IMU 航向修正


# ---------- 颜色（阶段一） ----------
COLOR_CENTER_TOL = 3.0     # 色块中心允许偏差，像素
COLOR_CORRECT_MM = 7       # 颜色左右微调每次移动距离，毫米
COLOR_MIN_RADIUS = 0       # 色块半径小于该值暂不微调
COLOR_DIRECTION_SIGN = 1   # 颜色修正方向：1=默认，-1=反向
IMU_DIRECTION_SIGN = 1     # IMU 转向方向：1=默认，-1=反向


# ---------- 电压补偿 ----------
LOW_VOLTAGE = 11.3         # 电压低于该值时夹取前补距离
VOLTAGE_EXTRA_MIN = 40     # 电压补偿最小距离，毫米
VOLTAGE_EXTRA_MAX = 80     # 电压补偿最大距离，毫米
VOLTAGE_EXTRA_LOW_V = 10.3  # 补偿达到最大距离时对应的电压


# ---------- 模型 + 雅可比（阶段二） ----------
DEFAULT_MODEL = 'models/base.pt'  # 默认模型路径（相对 spiderpi 根目录）
MODEL_CONF = 0.35          # 模型置信度阈值
JAC_DELTA = 20             # 标定时每个舵机的扰动脉宽
CALIB_SAMPLES = 5          # 标定时 bbox 中心平均帧数
CENTER_TOL_PX = 3.0        # 精对准中心误差阈值，像素
MAX_PULSE_STEP = 40        # 精对准单步舵机最大增量
MAX_ALIGN_ITER = 8         # 精对准最大迭代次数


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


def set_servo(board, sid, pulse, wait=0.3):
    """单个舵机移动到目标脉宽并等待。"""
    board.bus_servo_set_position(0.3, [[sid, int(pulse)]])
    time.sleep(wait)


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
    """准备第二次夹取：22-23 -> 24=500 -> 21 -> 24=JSON值。"""
    p = pulses
    print('pick2：22-23 -> 24=500 -> 21 -> 24=JSON值', flush=True)
    set_servos(board, p, [22, 23])
    board.bus_servo_set_position(2.2, [[24, 500]])
    time.sleep(2.2)
    set_servos(board, p, [21])
    set_servos(board, p, [24])
    return dict(p)


def place1_prepare(board, pulses=None):
    """准备第一次放下：使用记录的 21-24 放下脉宽。"""
    p = pulses
    print('place1：使用记录的 21-24 放下脉宽', flush=True)
    set_servos(board, p, [21, 22, 23, 24])
    return dict(p)


def lab_view(frame, lab, color):
    """生成 LAB 阈值图，只保留识别到的颜色区域。"""
    labf = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    minv = tuple(int(v) for v in lab[color]['min'])
    maxv = tuple(int(v) for v in lab[color]['max'])
    mask = cv2.inRange(labf, minv, maxv)
    return cv2.bitwise_and(frame, frame, mask=mask)


def open_vision(color, min_area):
    """打开摄像头，返回 (cam, read_frame, color_detector, publish)。"""
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    def read_frame():
        """读取并预处理一帧（去畸变 + 高斯模糊）。"""
        with camera_lock:
            f = capture(cam)
        if f is None:
            return None
        frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return cv2.GaussianBlur(frame, (7, 7), 0)

    def publish(frame):
        """把画面推流，供手机/网页查看。"""
        if task_server is not None:
            task_server.publish_frame(frame, max_fps=10.0)
            task_server.publish_lab_frame(lab_view(frame, lab, color), max_fps=10.0)

    def color_detector():
        """颜色检测（阶段一导航用），返回 dict 或 None，并推流。"""
        frame = read_frame()
        if frame is None:
            return None
        result = detect_color(frame, lab, color, min_area=min_area)
        if result is not None:
            x, y, w, h = cv2.boundingRect(result['contour'])
            result['bbox_center_x'] = (x + x + w) / 2.0
            cx, cy = result['center']
            cv2.circle(frame, (cx, cy), int(result.get('radius', 20)), (0, 255, 0), 2)
            cv2.putText(frame, color, (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        publish(frame)
        return result

    return cam, read_frame, color_detector, publish


def video_loop(detector, stop_event):
    """后台持续取帧推流，保证视频始终有画面。"""
    while not stop_event.is_set():
        detector()
        time.sleep(0.1)


class ModelDetector:
    """Ultralytics YOLO 检测器，detect() 返回 bbox dict 或 None。"""

    def __init__(self, model_path, conf, classes, read_frame, publish):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit('未安装 ultralytics，请先在树莓派执行: pip3 install ultralytics')
        self.model = YOLO(model_path)
        self.conf = conf
        self.classes = set(classes) if classes else None
        self.read_frame = read_frame
        self.publish = publish

    def detect(self):
        """检测一帧，返回 {'x','y','w','h','conf','name'} 或 None，并推流标注画面。"""
        frame = self.read_frame()
        if frame is None:
            return None
        result = None
        results = self.model.predict(frame, verbose=False, imgsz=640)[0]
        names = results.names
        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            name = names.get(cls, str(cls))
            if conf < self.conf:
                continue
            if self.classes and name not in self.classes:
                continue
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            w, h = x2 - x1, y2 - y1
            if w < 2 or h < 2:
                continue
            result = {'x': x1, 'y': y1, 'w': w, 'h': h, 'conf': conf, 'name': name}
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, '%s %.2f' % (name, conf), (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            break
        self.publish(frame)
        return result


def bbox_center(det):
    """从检测结果取矩形框中心 (cx, cy)。"""
    return det['x'] + det['w'] / 2.0, det['y'] + det['h'] / 2.0


def read_gz(board):
    """读取 IMU 的 gz 角速度。"""
    try:
        data = board.get_imu()
        if data is None:
            return None
        return float(data[5])
    except Exception:
        return None


def read_battery_running(board, samples=30, interval=0.05):
    """实时连续采样电池电压，返回 (中位数毫伏, 平均值毫伏)。"""
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
    """电压从 11.3V 到 10.3V，补偿距离从 40mm 线性增加到 80mm（整体乘 0.75）。"""
    if voltage >= LOW_VOLTAGE:
        return 0
    if voltage <= VOLTAGE_EXTRA_LOW_V:
        return VOLTAGE_EXTRA_MAX
    ratio = (LOW_VOLTAGE - voltage) / (LOW_VOLTAGE - VOLTAGE_EXTRA_LOW_V) * 0.75
    return int(VOLTAGE_EXTRA_MIN + (VOLTAGE_EXTRA_MAX - VOLTAGE_EXTRA_MIN) * ratio)


def apply_voltage_compensation(board, ik):
    """夹取前读取实时电压，需要时额外前进补偿距离。"""
    median_mv, avg_mv = read_battery_running(board)
    if median_mv is None:
        print('无法读取电压，跳过电压补偿', flush=True)
        return
    voltage = median_mv / 1000.0
    extra_mm = extra_distance_mm(voltage)
    print('实时电压: 中位数 %.2fV, 平均 %.2fV'
          % (voltage, avg_mv / 1000.0), flush=True)
    if extra_mm > 0:
        print('电压补偿：额外前进 %dmm' % extra_mm, flush=True)
        ik.go_forward(ik.initial_pos, 2, extra_mm, MOVE_SPEED, 1)
    else:
        print('电压正常，不额外前进', flush=True)


def is_low_voltage(board):
    """快速判断当前电压是否低于阈值。"""
    median_mv, _ = read_battery_running(board, samples=10, interval=0.02)
    return median_mv is not None and median_mv / 1000.0 < LOW_VOLTAGE


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
    """阶段一：只根据色块左右中心，做机械足左右微调。"""
    det = detector()
    if det is None:
        print('未发现定位色块', flush=True)
        tilt['pulse'] = max(100, tilt['pulse'] - 5)
        board.bus_servo_set_position(0.2, [[24, tilt['pulse']]])
        time.sleep(0.2)
        print('24 号下移微调 -> %d' % tilt['pulse'], flush=True)
        return False
    cx = det.get('bbox_center_x', det['center'][0])
    if det.get('radius', 0) < COLOR_MIN_RADIUS:
        print('目标较远，暂不做颜色微调，依赖 IMU 保持航向', flush=True)
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
    return True


def move_one_chunk(ik, move, forward):
    """只走一小段前进或后退。"""
    if forward:
        ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
    else:
        ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)


def move_straight_imu_color(ik, board, detector, imu_state, target_yaw,
                            distance_mm, tilt, color_enabled, color_state):
    """直线阶段：IMU 保持航向 + 颜色左右微调 + 前进。"""
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        color_adjusted = False
        if color_enabled:
            color_adjusted = color_keep_center(ik, board, detector, tilt, color_state)
            if color_adjusted:
                target_yaw = imu_state['yaw']
        update_imu(imu_state, board)
        err = angle_error(imu_state['yaw'], target_yaw)
        if ENABLE_IMU_STRAIGHT:
            if err > LEFT_TURN_TOL_DEG:
                if IMU_DIRECTION_SIGN > 0:
                    ik.turn_left(ik.initial_pos, 2, IMU_STRAIGHT_STEP, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f -> 左转%d°'
                          % (imu_state['yaw'], target_yaw, err, IMU_STRAIGHT_STEP), flush=True)
                else:
                    ik.turn_right(ik.initial_pos, 2, IMU_STRAIGHT_STEP, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f -> 右转%d°'
                          % (imu_state['yaw'], target_yaw, err, IMU_STRAIGHT_STEP), flush=True)
            elif err < -RIGHT_TURN_TOL_DEG:
                if IMU_DIRECTION_SIGN > 0:
                    ik.turn_right(ik.initial_pos, 2, IMU_STRAIGHT_STEP, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f -> 右转%d°'
                          % (imu_state['yaw'], target_yaw, err, IMU_STRAIGHT_STEP), flush=True)
                else:
                    ik.turn_left(ik.initial_pos, 2, IMU_STRAIGHT_STEP, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f -> 左转%d°'
                          % (imu_state['yaw'], target_yaw, err, IMU_STRAIGHT_STEP), flush=True)
                time.sleep(0.05)
                target_yaw = imu_state['yaw']
        move = min(100, remaining)
        move_one_chunk(ik, move, forward)
        remaining -= move
        time.sleep(0.05)


def imu_turn(ik, board, imu_state, delta_deg):
    """固定步数转弯，转完后再用 IMU 判断并只修正一次。"""
    start_yaw = imu_state['yaw']
    target = start_yaw + delta_deg
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

    time.sleep(0.2)
    for _ in range(5):
        update_imu(imu_state, board)
        time.sleep(0.02)

    err = angle_error(imu_state['yaw'], target)
    print('转弯完成 yaw=%.1f target=%.1f error=%+.1f'
          % (imu_state['yaw'], target, err), flush=True)

    if abs(err) > HEADING_TOL_DEG:
        step = IMU_STRAIGHT_STEP if err > 0 else -IMU_STRAIGHT_STEP
        if step > 0:
            ik.turn_left(ik.initial_pos, 2, abs(step), TURN_SPEED, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, abs(step), TURN_SPEED, 1)
        time.sleep(0.08)
        update_imu(imu_state, board)
        print('转弯后修正一次 yaw=%.1f' % imu_state['yaw'], flush=True)


def solve_jacobian(J, e):
    """解 J @ du = -e，用最小范数伪逆得到舵机增量 du。"""
    J = np.asarray(J, dtype=np.float64)
    e = np.asarray(e, dtype=np.float64)
    jjt = J @ J.T + np.eye(J.shape[0]) * 1e-6
    return J.T @ np.linalg.inv(jjt) @ (-e)


def clamp_step(du, max_step):
    """限制单步舵机增量向量的模长，避免跳太远。"""
    du = np.asarray(du, dtype=np.float64)
    norm = np.linalg.norm(du)
    if norm > max_step:
        du = du / norm * max_step
    return du


def sample_bbox(detector, samples):
    """连续采样多帧，返回平均 (cx, cy, w, h)，检测不到返回 None。"""
    cxs, cys, ws, hs = [], [], [], []
    for _ in range(samples):
        det = detector()
        if det is None:
            continue
        cxs.append(det['x'] + det['w'] / 2.0)
        cys.append(det['y'] + det['h'] / 2.0)
        ws.append(det['w'])
        hs.append(det['h'])
    if not cxs:
        return None
    return (sum(cxs) / len(cxs), sum(cys) / len(cys),
            sum(ws) / len(ws), sum(hs) / len(hs))


def calibrate_jacobian(board, detector, pulses):
    """逐舵机 +delta 扰动，估计 2x4 雅可比，返回 (J, anchor, ref_size)。"""
    base = sample_bbox(detector, CALIB_SAMPLES)
    if base is None:
        raise RuntimeError('标定失败：检测不到目标，请确认目标在夹取位置且模型可用')
    base_cx, base_cy, base_w, base_h = base

    J = [[0.0] * len(ARM_SERVOS) for _ in range(2)]
    for j, sid in enumerate(ARM_SERVOS):
        set_servo(board, sid, clamp_pulse(pulses[sid] + JAC_DELTA), wait=0.6)
        c = sample_bbox(detector, max(2, CALIB_SAMPLES // 2))
        if c is None:
            set_servo(board, sid, pulses[sid], wait=0.6)
            raise RuntimeError('标定失败：舵机 %d 扰动后检测不到目标' % sid)
        J[0][j] = (c[0] - base_cx) / JAC_DELTA
        J[1][j] = (c[1] - base_cy) / JAC_DELTA
        set_servo(board, sid, pulses[sid], wait=0.6)
        time.sleep(0.2)
    return J, (base_cx, base_cy), (base_w, base_h)


def fine_align(board, detector, calib, arm_state):
    """阶段二：模型检测 bbox -> 雅可比 -> 微调 21-24，循环到居中。"""
    J = calib['jacobian']
    anchor = (calib['anchor']['cx'], calib['anchor']['cy'])
    for it in range(MAX_ALIGN_ITER):
        det = detector()
        if det is None:
            arm_state[24] = clamp_pulse(arm_state[24] - 5)
            set_servo(board, 24, arm_state[24])
            continue
        cx = det['x'] + det['w'] / 2.0
        cy = det['y'] + det['h'] / 2.0
        ex = cx - anchor[0]
        ey = cy - anchor[1]
        print('精对准 iter=%d cx=%.1f cy=%.1f ex=%+.1f ey=%+.1f'
              % (it, cx, cy, ex, ey), flush=True)
        if abs(ex) <= CENTER_TOL_PX and abs(ey) <= CENTER_TOL_PX:
            return True
        du = clamp_step(solve_jacobian(J, [ex, ey]), MAX_PULSE_STEP)
        for sid, d in zip(ARM_SERVOS, du):
            arm_state[sid] = clamp_pulse(arm_state[sid] + int(round(d)))
        set_servos(board, arm_state, ARM_SERVOS)
        time.sleep(0.8)
    return False


def load_calib(pick_num):
    """读取第 pick_num 次夹取的雅可比标定，无文件返回 None。"""
    p = calib_path(pick_num)
    if not os.path.exists(p):
        return None
    with open(p, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_calib(pick_num, data):
    """保存第 pick_num 次夹取的雅可比标定。"""
    with open(calib_path(pick_num), 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def find_pick_actions(actions):
    """从路线 JSON 中提取所有 pick 动作，返回 [{'index','pulses'}, ...]。"""
    picks = []
    for a in actions:
        if a.get('action') != 'pick':
            continue
        pulses = {int(k): int(v) for k, v in a.get('pulses', {}).items()} \
            if a.get('pulses') else None
        picks.append({'index': a.get('index'), 'pulses': pulses})
    return picks


def run_calibrate(board, model_det, pick_num, actions):
    """执行一次雅可比标定并保存结果。"""
    picks = find_pick_actions(actions)
    if pick_num > len(picks):
        raise SystemExit('标定失败：JSON 里只有 %d 个 pick，没有第 %d 个' % (len(picks), pick_num))
    pulses = picks[pick_num - 1]['pulses']
    if not pulses:
        raise SystemExit('标定失败：第 %d 个 pick 没有 pulses' % pick_num)

    print('标定第 %d 次夹取，标称脉宽: %s' % (pick_num, pulses), flush=True)
    print('请确认：目标已放在正确夹取位置，机械臂将先摆到标称脉宽。', flush=True)
    set_servos(board, pulses, ARM_SERVOS)
    time.sleep(1.0)

    J, anchor, ref_size = calibrate_jacobian(board, model_det, pulses)
    data = {
        'pick_index': picks[pick_num - 1]['index'],
        'nominal_pulses': pulses,
        'anchor': {'cx': round(anchor[0], 2), 'cy': round(anchor[1], 2)},
        'ref_size': {'w': round(ref_size[0], 2), 'h': round(ref_size[1], 2)},
        'jacobian': [[round(v, 4) for v in row] for row in J],
    }
    save_calib(pick_num, data)
    print('标定完成，已保存: %s' % calib_path(pick_num), flush=True)
    print(json.dumps(data, indent=2, ensure_ascii=False), flush=True)


def do_pick(board, pick_count, pulses, model_det, calib):
    """执行第 1/2 次夹取：先摆臂，再模型精对准，最后手动确认。"""
    if pick_count == 1:
        state = pick1_prepare(board, pulses)
    else:
        state = pick2_prepare(board, pulses)
    if calib is not None:
        print('阶段二：模型 + 雅可比精对准', flush=True)
        ok = fine_align(board, model_det, calib, state)
        if not ok:
            print('精对准未完全居中，进入手动微调', flush=True)
    else:
        print('无标定数据，跳过模型精对准，进入手动微调', flush=True)
    arm_fine_tune(board, state, 'pick')


def do_place(board, place_count, pulses=None):
    """执行第 1/2 次放下。"""
    if place_count == 1:
        state = place1_prepare(board, pulses)
    else:
        state = pulses or dict(OFFICIAL_ARM)
    arm_fine_tune(board, state, 'place')


def main():
    """主流程：按 JSON 导航，到夹取点后用模型 + 雅可比精对准夹取。"""
    parser = argparse.ArgumentParser(description='NO7 两级对准夹取')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=1)
    parser.add_argument('--model', default=DEFAULT_MODEL, help='YOLO 模型路径')
    parser.add_argument('--conf', type=float, default=MODEL_CONF, help='模型置信度阈值')
    parser.add_argument('--classes', default='', help='目标类别，逗号分隔；留空=接受所有类别')
    parser.add_argument('--calibrate', type=int, choices=[1, 2], default=None,
                        help='只做第 1/2 次夹取的雅可比标定')
    args = parser.parse_args()

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    cam, read_frame, color_detector, publish = open_vision(args.color, args.min_area)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    model_det = ModelDetector(args.model, args.conf, classes, read_frame, publish)

    video_stop = threading.Event()
    video_thread = threading.Thread(
        target=video_loop, args=(color_detector, video_stop), daemon=True)
    video_thread.start()

    if task_server is not None:
        print('视频推流: http://%s:5000/video.mjpeg' % lan_ip(), flush=True)
        print('LAB 推流: http://%s:5000/video_lab.mjpeg' % lan_ip(), flush=True)
        task_server.start_server()

    restore_travel(board, GRIPPER_OPEN)
    print('NO7 启动，颜色目标=%s，模型=%s' % (args.color, args.model), flush=True)

    # 标定模式：只摆臂 + 标定，不走完整路线
    if args.calibrate is not None:
        run_calibrate(board, model_det, args.calibrate, actions)
        restore_travel(board, GRIPPER_OPEN)
        video_stop.set()
        cam.camera_close()
        return

    imu_state = init_imu(board)
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
    left_turn_compensated = False

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
            if pending_forward < 0 and is_low_voltage(board):
                pending_forward -= 10
                print('低电压后退补偿：额外多退 10mm', flush=True)
            target_yaw = imu_state['yaw']
            segment_color = color_enabled and not (23 <= i <= 53)
            if color_enabled and not segment_color:
                print('当前步数 %d 在 23-53，暂停颜色微调' % i, flush=True)
            move_straight_imu_color(
                ik, board, color_detector, imu_state, target_yaw,
                pending_forward, tilt, segment_color, color_state)
            pending_forward = 0
            left_turn_compensated = False

        if name == 'turn_left':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 2:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            if is_low_voltage(board) and not left_turn_compensated:
                angle += 5
                print('低电压左转补偿：额外多转 5°', flush=True)
                left_turn_compensated = True
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
            apply_voltage_compensation(board, ik)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} \
                if act.get('pulses') else None
            calib = load_calib(pick_count)
            do_pick(board, pick_count, pulses, model_det, calib)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} \
                if act.get('pulses') else None
            do_place(board, place_count, pulses)
            if place_count == 1:
                color_enabled = False
                first_place_done = True
                turns_after_first_place = 0
                print('第一次放下完成，暂时关闭颜色识别', flush=True)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        if pending_forward < 0 and is_low_voltage(board):
            pending_forward -= 10
            print('低电压后退补偿：额外多退 10mm', flush=True)
        target_yaw = imu_state['yaw']
        segment_color = color_enabled and not (23 <= len(actions) <= 53)
        move_straight_imu_color(
            ik, board, color_detector, imu_state, target_yaw,
            pending_forward, tilt, segment_color, color_state)

    video_stop.set()
    cam.camera_close()
    ik.stand(ik.initial_pos, t=500)
    print('NO7 运行结束', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
