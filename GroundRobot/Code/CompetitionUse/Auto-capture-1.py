#!/usr/bin/python3
# coding=utf8
"""NO7：两级对准夹取（文件名 `Auto-capture-1.py`，旧称 NO7）。

阶段一：完全沿用 NO6 的「颜色左右微调 + IMU 航向保持」导航，把机器人带到固定夹取点附近。
阶段二：到夹取点后，用真实模型（默认 models/best.onnx）检测目标矩形框，再通过标定好的
        雅可比矩阵把像素误差换算成机械臂 21-24 号舵机增量，自动做精对准。
对准结束后仍保留手动微调 + 回车夹取 / c 退出，验证流程不变。

标定（一次性，每个夹取点做一次）：
    python3 Auto-capture-1.py --calibrate 1 --model models/best.onnx --color red
    python3 Auto-capture-1.py --calibrate 2 --model models/best.onnx --color red
运行：
    python3 Auto-capture-1.py --model models/best.onnx --color red

其它模式：
    --approach      不走固定路线，直接让模型边找边靠近再夹
    --imu-straight off / --turn-tol N   现场调直线段航向修正
"""

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
import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (
    make_board,
    make_ik,
    ImuTracker,
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
# 第一次夹取后拔起：动 22 号「肩」舵机（不是 23 号肘，23 是肘）。
# 关键是基准：拔起时 22 号停在路线 JSON 的「夹取位」（当前路线 22:395），不是复位位 705。
# 所以「往上抬」= 把 22 调到比夹取位大。现场实测 785 抬得太高（395→785，+390），
# 450 是小幅抬（395→450，+55）。命令行传 --pull-up N 试值，不用改代码。
PULL_UP_22 = 450


# ---------- 六足运动 ----------
MOVE_SPEED = 50  # 直线前进/后退速度
TURN_SPEED = 30  # 左转/右转速度


# ---------- IMU ----------
GYRO_SCALE_LEFT = 1.177    # 左转陀螺仪积分修正比例
GYRO_SCALE_RIGHT = 1.199   # 右转陀螺仪积分修正比例
# 重标零漂前先等机身晃动静下来再采样。这个方法基本都在刚转完弯之后调用，六足转身时
# 整个机身还在晃，这时候采到的不是真实零漂——标错多少，后面整段直行就照着错多少积分。
IMU_SETTLE_S = 0.25
HEADING_TOL_DEG = 1.0      # 转弯后航向误差容忍，越小越严格
# 直线段航向死区：|误差| 超过它才发一次修正转向。**左右必须对称。**
# 原先写的是左 1.0 / 右 8.0，方向是反的：angle_error 是 (target - current)，
# 所以 err > 0 表示机身朝右歪（该左转）、err < 0 表示机身朝左歪（该右转）。
# 「右 8.0」实际效果是「机身朝左歪 8° 都不管」，机身长期歪着 → 装在身上的相机跟着歪
# → 画面里色块恒偏一侧 → 颜色微调一直往那一侧平移 → 直线段先直、然后一路偏出去。
# 取值要略大于六足单次转弯的最小步进，太小会左右来回抖；现场用 --turn-tol 调。
TURN_TOL_DEG = 3.0
IMU_STRAIGHT_STEP = 1      # 转弯后一次性修正的角度（imu_turn 用）
IMU_STRAIGHT_GAIN = 0.6    # 直线阶段航向修正比例：单次转「误差 × 该比例」的角度（P 控制，需现场调）
IMU_STRAIGHT_MAX = 8.0     # 直线阶段单次修正最大角度（度），防止误差大时一步转过头
ENABLE_IMU_STRAIGHT = True  # 直线阶段是否启用 IMU 航向修正


# ---------- 颜色（阶段一） ----------
COLOR_CENTER_TOL = 3.0     # 色块中心允许偏差，像素
COLOR_CORRECT_MM = 7       # 向右微调每次移动距离，毫米
LEFT_CORRECT_MM = 10       # 向左微调每次移动距离，毫米（向左力度加大）
COLOR_MIN_RADIUS = 0       # 色块半径小于该值暂不微调
COLOR_DIRECTION_SIGN = 1   # 颜色修正方向：1=默认，-1=反向
IMU_DIRECTION_SIGN = 1     # IMU 转向方向：1=默认，-1=反向


# ---------- 电压补偿 ----------
LOW_VOLTAGE = 11.3         # 电压低于该值时夹取前补距离
VOLTAGE_EXTRA_MIN = 40     # 电压补偿最小距离，毫米
VOLTAGE_EXTRA_MAX = 80     # 电压补偿最大距离，毫米
VOLTAGE_EXTRA_LOW_V = 10.3  # 补偿达到最大距离时对应的电压


# ---------- 模型 + 雅可比（阶段二） ----------
DEFAULT_MODEL = 'models/best.onnx'  # 默认模型路径（相对 spiderpi 根目录）
MODEL_CONF = 0.8           # 模型置信度阈值
JAC_DELTA = 20             # 标定时每个舵机的扰动脉宽
CALIB_SAMPLES = 5          # 标定时 bbox 中心平均帧数
CENTER_TOL_PX = 3.0        # 精对准中心误差阈值，像素
MAX_PULSE_STEP = 40        # 精对准单步舵机最大增量
MAX_ALIGN_ITER = 8         # 精对准最大迭代次数


# ---------- 模型直接靠近 ----------
MODEL_STOP_W = 140         # 目标 bbox 宽度达到该值(像素)视为够近，停止靠近
MODEL_STEP_MM = 30         # 靠近每步前进 mm
MODEL_SEARCH_TURN = 15     # 未检测到目标时左转搜索角度
MODEL_CENTER_TOL = 40      # 目标中心允许偏差(像素)，小于该值不左右移
MODEL_MAX_STEP = 80        # 靠近最多步数


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


def report(**kw):
    """上报仪表盘状态（task_server 未启用时静默跳过）。"""
    if task_server is not None:
        task_server.set_status(**kw)


def norm_heading(deg):
    """把累计 yaw 归一化到 [0, 360) 度。"""
    return round(deg % 360.0, 1)


def advance_pose(pose, yaw_deg, dist_mm):
    """按当前朝向累计里程，更新 pose（单位：米）。"""
    yaw_rad = math.radians(yaw_deg)
    pose['x'] += (dist_mm / 1000.0) * math.sin(yaw_rad)
    pose['y'] += (dist_mm / 1000.0) * math.cos(yaw_rad)
    return pose


def pose_dict(pose):
    """返回仪表盘需要的 position_m 结构。"""
    return {'x': round(pose['x'], 3), 'y': round(pose['y'], 3)}


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


def arm_fine_tune(board, state, kind, pull_up=False, pull_up_pulse=None):
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
    if pull_up:
        # 22 号肩舵机上抬，把目标从地里/网里拔出来，再恢复初始位置
        pulse = PULL_UP_22 if pull_up_pulse is None else clamp_pulse(pull_up_pulse)
        # 起点是当前实际值 state[22]（夹取位），不是复位位 705——印错起点会让人
        # 误判抬升方向（曾据此把 400 当成「往下压」）。
        print('拔起：22 号肩舵机 %d → %d（抬升 %+d）'
              % (state[22], pulse, pulse - state[22]), flush=True)
        board.bus_servo_set_position(1.0, [[22, pulse]])
        time.sleep(1.0)
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


def video_loop(detectors, stop_event):
    """后台持续取帧推流：颜色识别 + YOLO 模型检测都叠加推流显示。"""
    while not stop_event.is_set():
        for d in detectors:
            d()
        time.sleep(0.1)


class ModelDetector:
    """ONNX YOLO 检测器（onnxruntime 本地推理），detect() 返回 bbox dict 或 None。"""

    NAME = 'fake bug'  # 目标类别名
    CLASS_IDX = 15     # 目标类在模型输出里的索引（16 类模型里 fake bug 是第 15 类，输出 [4+16, 8400]）

    def __init__(self, model_path, conf, classes, read_frame, publish):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4  # Pi5 四核并行
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        self.output_name = self.sess.get_outputs()[0].name
        self.conf = conf
        self.classes = set(classes) if classes else None
        self.read_frame = read_frame
        self.publish = publish

    def _letterbox(self, img):
        """等比缩放到 640x640 补灰边，返回 (画布, 缩放比, pad_x, pad_y)。"""
        h0, w0 = img.shape[:2]
        r = min(640 / w0, 640 / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        pad_x, pad_y = (640 - new_w) // 2, (640 - new_h) // 2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(img, (new_w, new_h))
        return canvas, r, pad_x, pad_y

    def detect(self):
        """检测一帧，返回 {'x','y','w','h','conf','name'} 或 None，并推流标注画面。"""
        if self.classes and self.NAME not in self.classes:
            return None
        frame = self.read_frame()
        if frame is None:
            return None
        h0, w0 = frame.shape[:2]
        canvas, r, pad_x, pad_y = self._letterbox(frame)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run([self.output_name], {self.input_name: blob})[0][0]  # [4+nc, 8400]

        best = None  # (x1, y1, x2, y2, score)
        for i in range(out.shape[1]):
            score = float(out[4 + self.CLASS_IDX, i])
            if score < self.conf:
                continue
            cx, cy, w, h = out[0, i], out[1, i], out[2, i], out[3, i]
            x1 = (cx - w / 2 - pad_x) / r
            y1 = (cy - h / 2 - pad_y) / r
            x2 = (cx + w / 2 - pad_x) / r
            y2 = (cy + h / 2 - pad_y) / r
            x1 = max(0, min(w0, x1))
            y1 = max(0, min(h0, y1))
            x2 = max(0, min(w0, x2))
            y2 = max(0, min(h0, y2))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            if best is None or score > best[4]:
                best = (x1, y1, x2, y2, score)

        result = None
        if best is not None:
            x1, y1, x2, y2, score = best
            result = {'x': int(x1), 'y': int(y1), 'w': int(x2 - x1), 'h': int(y2 - y1),
                      'conf': float(score), 'name': self.NAME}
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(frame, '%s %.2f' % (self.NAME, score), (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        self.publish(frame)
        return result


def bbox_center(det):
    """从检测结果取矩形框中心 (cx, cy)。"""
    return det['x'] + det['w'] / 2.0, det['y'] + det['h'] / 2.0


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
    """初始化 IMU：开启接收，起后台积分线程，标定 gz 零漂。

    航向积分交给 `ImuTracker` 线程连续做（约 105Hz）。**不能**再靠「要用的时候读
    一个样本」——官方 SDK 的 imu_queue 是 maxsize=1，两次读取之间的样本全被丢掉，
    而这里两次读取之间夹着一整段 100mm 行走（1.5~1.9 秒），那一个瞬时样本落在步态
    周期的哪个相位纯属偶然，乘上 1.9 秒就是一个随机方向的假转角。
    详见 `agcs_lib/imu.py` 与 `CompetitionUse/imu_probe.py` 的实测数据。
    """
    board.enable_reception()
    state = {'bias': 0.0, 'yaw': 0.0, 'last_rate': 0.0, 'last_dt': 0.0}
    tracker = ImuTracker(board, state,
                         scale_left=GYRO_SCALE_LEFT, scale_right=GYRO_SCALE_RIGHT)
    tracker.start()
    time.sleep(0.5)                       # 等队列里开始有数据
    bias, n = tracker.calibrate(1.0, settle=0.0)   # 开机时是静止的，不用沉降
    tracker.reset()
    state['tracker'] = tracker
    print('IMU 零漂 %+.3f°/s（%d 个样本），后台采样已启动' % (bias, n), flush=True)
    return state


def reset_imu(board, imu_state):
    """转弯后重新标定 gz 零漂并清零航向积分（消除累积漂移导致的误纠）。"""
    tracker = imu_state.get('tracker')
    if tracker is None:
        return
    tracker.calibrate(0.5, settle=IMU_SETTLE_S)
    tracker.reset()


def update_imu(state, board):
    """刷新日志用的「上一段平均角速度 / 时长」。

    航向积分已经由后台线程连续完成，这里不再积分——保留这个函数只是为了不动主循环
    结构，并给现场留一个对照值：`rate` 是这一段直行的**平均**角速度，机器人走得直
    它就应该接近 0；若常在 ±0.5°/s 以上，说明零漂标定不准。
    """
    tracker = state.get('tracker')
    if tracker is None:
        return
    state['last_rate'], state['last_dt'] = tracker.since_last()


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
            ik.left_move(ik.initial_pos, 2, LEFT_CORRECT_MM, MOVE_SPEED, 1)
            print('色块右偏，机械足左移 %dmm（方向反向）' % LEFT_CORRECT_MM, flush=True)
    else:
        if COLOR_DIRECTION_SIGN > 0:
            ik.left_move(ik.initial_pos, 2, LEFT_CORRECT_MM, MOVE_SPEED, 1)
            print('色块左偏，机械足左移 %dmm' % LEFT_CORRECT_MM, flush=True)
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
        if color_enabled:
            # 颜色微调是「平移」，不改变航向基准 —— 这里**不能**把 target_yaw 重设成
            # 当前 yaw。那样等于每做一次微调就把这一小段已攒下的航向误差一笔勾销，
            # 误差永远不收敛，机身会一路朝同一边偏下去（原实现就是这样）。
            color_keep_center(ik, board, detector, tilt, color_state)
        update_imu(imu_state, board)
        err = angle_error(imu_state['yaw'], target_yaw)
        if ENABLE_IMU_STRAIGHT:
            # dt/rate 是这次积分用的时长与平均角速度。看 rate：机器人站在地上不动时
            # 它应该接近 0，若常在 ±0.5°/s 以上，说明那次转弯后的零漂没标定准，
            # yaw 里就混进了「假漂移」，修正循环会照着假误差转。
            dbg = '(dt=%.1fs rate=%+.2f°/s)' % (
                imu_state.get('last_dt', 0.0), imu_state.get('last_rate', 0.0))
            corrected = False
            if err > TURN_TOL_DEG:
                step = max(1, int(round(min(err, IMU_STRAIGHT_MAX) * IMU_STRAIGHT_GAIN)))
                if IMU_DIRECTION_SIGN > 0:
                    ik.turn_left(ik.initial_pos, 2, step, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f %s -> 左转%d°'
                          % (imu_state['yaw'], target_yaw, err, dbg, step), flush=True)
                else:
                    ik.turn_right(ik.initial_pos, 2, step, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f %s -> 右转%d°'
                          % (imu_state['yaw'], target_yaw, err, dbg, step), flush=True)
                corrected = True
            elif err < -TURN_TOL_DEG:
                step = max(1, int(round(min(-err, IMU_STRAIGHT_MAX) * IMU_STRAIGHT_GAIN)))
                if IMU_DIRECTION_SIGN > 0:
                    ik.turn_right(ik.initial_pos, 2, step, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f %s -> 右转%d°'
                          % (imu_state['yaw'], target_yaw, err, dbg, step), flush=True)
                else:
                    ik.turn_left(ik.initial_pos, 2, step, TURN_SPEED, 1)
                    print('IMU yaw=%.1f target=%.1f error=%+.1f %s -> 左转%d°'
                          % (imu_state['yaw'], target_yaw, err, dbg, step), flush=True)
                corrected = True
            if corrected:
                time.sleep(0.05)
                # 这里**同样不能**重设 target_yaw。修正是「把 yaw 拉回本段目标」，
                # 修完就把目标挪到当前 yaw，等于宣告「刚才的偏差不算数」——控制器
                # 从此只修「距上次修正之后的新偏差」，永远清不掉已有的偏置，目标会
                # 跟着漂移一路爬。现场日志就是这么跑的：target 3.0→6.6→9.6→12.3，
                # 每块涨 3° 左右，和幻影漂移同步，于是每块都发一次右转 → 「右转严重」。
                # 本段目标就是进入本段时的航向，整段不动。
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

    time.sleep(0.2)   # 等机身晃动静下来；积分由后台线程连续做，不用再手动补采
    update_imu(imu_state, board)

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
        print('转弯后修正一次 %d°' % abs(step), flush=True)


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


def model_approach(board, ik, model_det, stop_w=MODEL_STOP_W):
    """用 YOLO 模型直接检测目标并慢慢靠近：左右居中 + 前进，直到 bbox 宽度够大。

    不依赖固定路线。检测不到就左转搜索；检测到就按目标中心左右平移、再前进一小步。
    """
    for i in range(MODEL_MAX_STEP):
        det = model_det.detect()
        if det is None:
            print('未检测到目标，左转搜索', flush=True)
            ik.turn_left(ik.initial_pos, 2, MODEL_SEARCH_TURN, TURN_SPEED, 1)
            time.sleep(0.4)
            continue
        cx = det['x'] + det['w'] / 2.0
        w = det['w']
        print('模型检测 #%d cx=%.1f w=%d conf=%.2f' % (i, cx, w, det.get('conf', 0)),
              flush=True)
        if w >= stop_w:
            print('目标已够近(w=%d)，停止靠近' % w, flush=True)
            return det
        if cx < 320 - MODEL_CENTER_TOL:
            ik.left_move(ik.initial_pos, 2, LEFT_CORRECT_MM, MOVE_SPEED, 1)
            print('目标偏左，左移 %dmm' % LEFT_CORRECT_MM, flush=True)
        elif cx > 320 + MODEL_CENTER_TOL:
            ik.right_move(ik.initial_pos, 2, COLOR_CORRECT_MM, MOVE_SPEED, 1)
            print('目标偏右，右移 %dmm' % COLOR_CORRECT_MM, flush=True)
        ik.go_forward(ik.initial_pos, 2, MODEL_STEP_MM, MOVE_SPEED, 1)
        time.sleep(0.1)
    print('靠近步数用尽，未到目标', flush=True)
    return None


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


def do_pick(board, pick_count, pulses, model_det, calib, pull_up_pulse=None):
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
    arm_fine_tune(board, state, 'pick', pull_up=(pick_count == 1),
                  pull_up_pulse=pull_up_pulse)


def do_place(board, place_count, pulses=None):
    """执行第 1/2 次放下。"""
    if place_count == 1:
        state = place1_prepare(board, pulses)
    else:
        p = pulses or dict(OFFICIAL_ARM)
        print('place2：使用记录的 21-24 放下脉宽', flush=True)
        set_servos(board, p, [21, 22, 23, 24])
        state = dict(p)
    arm_fine_tune(board, state, 'place')


class _Tee(object):
    """把 stdout/stderr 同时抄一份到日志文件。"""

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
    """把本次运行的完整输出抄到 logs/<日期>/autocapture/<时-分>.log。

    跑一次路线好几分钟，出问题全靠翻终端；而 print() 只进终端、不落盘（官方日志
    只收 action_msg），几次现场排查都因为「没留下完整日志」只能靠猜。落一份盘，
    事后可以直接完整回看整段 yaw/误差/颜色微调序列。
    本地跑（没有 /home/pi）时静默跳过，不影响脚本。
    """
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
    """主流程：按 JSON 导航，到夹取点后用模型 + 雅可比精对准夹取。"""
    global ENABLE_IMU_STRAIGHT, TURN_TOL_DEG
    parser = argparse.ArgumentParser(description='NO7 两级对准夹取')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=1)
    parser.add_argument('--model', default=DEFAULT_MODEL, help='YOLO 模型路径')
    parser.add_argument('--conf', type=float, default=MODEL_CONF, help='模型置信度阈值')
    parser.add_argument('--classes', default='', help='目标类别，逗号分隔；留空=接受所有类别')
    parser.add_argument('--calibrate', type=int, choices=[1, 2], default=None,
                        help='只做第 1/2 次夹取的雅可比标定')
    parser.add_argument('--approach', action='store_true',
                        help='直接启用模型检测并慢慢靠近(不走固定路线)')
    parser.add_argument('--pull-up', type=int, default=None,
                        help='第一次夹取后拔起的脉宽（22 号肩，默认 %d）；'
                             '幅度不合适现场试值' % PULL_UP_22)
    parser.add_argument('--imu-straight', default='on', choices=['on', 'off'],
                        help='直线阶段是否用 IMU 修正航向（默认 on）。'
                             '想单独看「不做 IMU 修正会不会更直」就传 off 做对照')
    parser.add_argument('--turn-tol', type=float, default=TURN_TOL_DEG,
                        help='直线段航向死区（度，默认 %.1f）。左右对称。'
                             '机器人若左右来回抖就调大，偏出去不修就调小' % TURN_TOL_DEG)
    args = parser.parse_args()

    log_file = start_run_log()

    ENABLE_IMU_STRAIGHT = (args.imu_straight == 'on')
    TURN_TOL_DEG = args.turn_tol
    if not ENABLE_IMU_STRAIGHT:
        print('直线阶段 IMU 航向修正已关闭（--imu-straight off）', flush=True)
    print('直线段航向死区 ±%.1f°' % TURN_TOL_DEG, flush=True)

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    cam, read_frame, color_detector, publish = open_vision(args.color, args.min_area)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    # 相对路径按 spiderpi 根目录解析（模型/地图在 ~/spiderpi/models/ 下，不在 CompetitionUse/ 下）
    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    model_det = ModelDetector(model_path, args.conf, classes, read_frame, publish)

    video_stop = threading.Event()
    video_thread = threading.Thread(
        target=video_loop, args=([color_detector, model_det.detect], video_stop), daemon=True)
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
    print('NO7 启动，颜色目标=%s，模型=%s' % (args.color, args.model), flush=True)

    # 标定模式：只摆臂 + 标定，不走完整路线
    if args.calibrate is not None:
        run_calibrate(board, model_det, args.calibrate, actions)
        restore_travel(board, GRIPPER_OPEN)
        video_stop.set()
        cam.camera_close()
        return

    # 模型直接靠近模式：不走固定路线，直接启用模型检测并慢慢靠近，够近后夹取
    if args.approach:
        print('模型直接靠近模式', flush=True)
        ik.stand(ik.initial_pos, t=500)
        time.sleep(0.5)
        det = model_approach(board, ik, model_det)
        if det is not None:
            picks = find_pick_actions(actions)
            pulses = picks[0]['pulses'] if picks else None
            calib = load_calib(1)
            do_pick(board, 1, pulses, model_det, calib, pull_up_pulse=args.pull_up)
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
    picked_count = 0
    pose = {'x': 0.0, 'y': 0.0}
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
            if segment_color:
                color_state['ref_cx'] = None  # 每个直行段开头重新取「一开始检测到的色块」作固定参考点
            dist_mm = pending_forward
            move_straight_imu_color(
                ik, board, color_detector, imu_state, target_yaw,
                pending_forward, tilt, segment_color, color_state)
            pending_forward = 0
            left_turn_compensated = False
            advance_pose(pose, imu_state['yaw'], dist_mm)
            report(position_m=pose_dict(pose),
                   heading_deg=norm_heading(imu_state['yaw']),
                   message='直行 %dmm' % dist_mm)

        if name == 'turn_left':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 6:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            if is_low_voltage(board) and not left_turn_compensated:
                angle += 5
                print('低电压左转补偿：额外多转 5°', flush=True)
                left_turn_compensated = True
            print('%d/%d IMU左转 %d' % (i, len(actions), angle), flush=True)
            imu_turn(ik, board, imu_state, angle)
            reset_imu(board, imu_state)
            target_yaw = imu_state['yaw']
            report(heading_deg=norm_heading(imu_state['yaw']),
                   message='左转 %d°' % angle)
        elif name == 'turn_right':
            if first_place_done:
                turns_after_first_place += 1
                if turns_after_first_place >= 6:
                    color_enabled = True
                    color_state['ref_cx'] = None
            angle = int(act.get('angle', 90))
            print('%d/%d IMU右转 %d' % (i, len(actions), angle), flush=True)
            imu_turn(ik, board, imu_state, -angle)
            reset_imu(board, imu_state)
            target_yaw = imu_state['yaw']
            report(heading_deg=norm_heading(imu_state['yaw']),
                   message='右转 %d°' % angle)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            apply_voltage_compensation(board, ik)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} \
                if act.get('pulses') else None
            calib = load_calib(pick_count)
            do_pick(board, pick_count, pulses, model_det, calib, pull_up_pulse=args.pull_up)
            picked_count += 1
            report(picked_count=picked_count,
                   message='第 %d 次夹取完成' % picked_count)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            apply_voltage_compensation(board, ik)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} \
                if act.get('pulses') else None
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
        if pending_forward < 0 and is_low_voltage(board):
            pending_forward -= 10
            print('低电压后退补偿：额外多退 10mm', flush=True)
        target_yaw = imu_state['yaw']
        segment_color = color_enabled and not (23 <= len(actions) <= 53)
        if segment_color:
            color_state['ref_cx'] = None
        dist_mm = pending_forward
        move_straight_imu_color(
            ik, board, color_detector, imu_state, target_yaw,
            pending_forward, tilt, segment_color, color_state)
        advance_pose(pose, imu_state['yaw'], dist_mm)

    video_stop.set()
    cam.camera_close()
    tracker = imu_state.get('tracker')
    if tracker is not None:
        tracker.stop()
    ik.stand(ik.initial_pos, t=500)
    report(state='END', last_result='done',
           position_m=pose_dict(pose),
           heading_deg=norm_heading(imu_state['yaw']),
           picked_count=picked_count,
           message='自动捕获完成')
    print('NO7 运行结束', flush=True)
    time.sleep(5)  # END 状态停留 5 秒，供中枢轮询确认

    if log_file is not None:
        log_file.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
