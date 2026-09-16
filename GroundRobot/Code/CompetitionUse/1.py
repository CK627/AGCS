#!/usr/bin/python3
# coding=utf8
"""1.py —— 融合导航 + JSON 路线（独立脚本，不依赖 NO6/NO7）。

直线段走融合导航（LaneFusion + LaneController：相机写状态、IMU 管执行、
先航向后横向），夹取/放下/路线读 fixed_route.json 的固定脉宽，不碰 YOLO。
YOLO 夹取那套在 2.py。
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
from agcs_lib.heading_fusion import (
    LaneFusion,
    LaneController,
    range_from_radius_px,
)

try:
    from communication import task_server
except ImportError:
    task_server = None


ROUTE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'json1.json')

OFFICIAL_ARM = {21: 500, 22: 705, 23: 90, 24: 330}  # 机械臂官方初始脉宽

GRIPPER_CLOSE = 700  # 夹取时 25 号夹爪闭合的脉宽，越大夹得越紧
GRIPPER_OPEN = 400   # 放下时 25 号夹爪打开的脉宽，越小张得越开
# 第一次夹取后拔起：动 22 号「肩」舵机（不是 23 号肘，23 是肘）。
# 关键是基准：拔起时 22 号停在路线 JSON 的「夹取位」（当前路线 22:395），不是复位位 705。
# 所以「往上抬」= 把 22 调到比夹取位大。现场实测 785 抬得太高（395→785，+390），
# 450 是小幅抬（395→450，+55）。命令行传 --pull-up N 试值，不用改代码。
PULL_UP_22 = 450
MOVE_SPEED = 50      # 六足直线前进/后退的速度，越大走得越快
TURN_SPEED = 30      # 六足左转/右转的速度，越大转得越快
GYRO_SCALE_LEFT = 1.177   # IMU 左转时陀螺仪积分修正比例
GYRO_SCALE_RIGHT = 1.199  # IMU 右转时陀螺仪积分修正比例
# 重标零漂前先等机身晃动静下来再采样。这个方法基本都在刚转完弯之后调用，六足转身时
# 整个机身还在晃，这时候采到的不是真实零漂——标错多少，后面整段直行就照着错多少积分。
IMU_SETTLE_S = 0.25
HEADING_TOL_DEG = 1.0    # 航向误差容忍范围，单位：度；越小越严格
IMU_STRAIGHT_STEP = 1    # 转弯后一次性修正的角度（imu_turn 用）
# --- 融合导航（默认启用）---
# 内参单位是 detect_color 缩放后的像素（内部 resize 到 320×240，
# 而 result['contour'] 也来自这张 320 图，所以 bbox_center_x 同属 320 坐标系）。
# f_px / cx0 / f_px_full 都要实测标定，下面只是「320 宽、约 60° 水平视场」的占位值。
FUSION_F_PX = 277.0
FUSION_CX0 = 160.0
FUSION_F_PX_FULL = 554.0   # 原始分辨率下的焦距，用于从色块视半径反推距离
FUSION_HEAD_GAIN = 0.9     # 航向 P 增益：转「误差 × 该比例」度
FUSION_CROSS_GAIN = 0.35   # 横向 P 增益：横移「偏差 × 该比例」mm
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


def reset_imu(board, imu_state, reset_yaw=True):
    """转弯后重新标定 gz 零漂，并（可选）清零航向积分。

    **融合模式下 `reset_yaw` 必须传 False**：零漂是传感器属性，重标没问题；
    航向是**状态**，清零等于把已经攒下来的可观测性丢掉 —— 相机那边若同时重取
    参考点，系统就再没有任何东西知道「我到底歪了多少」。
    """
    tracker = imu_state.get('tracker')
    if tracker is None:
        return
    tracker.calibrate(0.5, settle=IMU_SETTLE_S)
    if reset_yaw:
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


def move_one_chunk(ik, move, forward):
    """只走一小段前进或后退。"""
    if forward:
        ik.go_forward(ik.initial_pos, 2, move, MOVE_SPEED, 1)
    else:
        ik.back(ik.initial_pos, 2, move, MOVE_SPEED, 1)


def move_straight_fusion(ik, board, detector, imu_state, tracker, distance_mm,
                         fusion, ctrl, marker_range_mm, marker_size_mm):
    """融合模式的直线段：相机写状态、IMU 管执行，单一控制器先航向后横向。

    与 `move_straight_imu_color` 的本质区别：
    - 相机不再「平移去消掉像素偏移」，而是更新 (航向误差, 横向偏差) 这个估计；
    - 控制器同一小块里**只发一种指令**：航向没进死区就只转，进了才横移；
    - `fusion` 的状态全程不归零（归零 = 把误差从数字搬到机器人身上）。
    """
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    last_ds = 0.0
    while remaining > 0:
        det = detector()
        cx = None
        if det is not None:
            cx = det.get('bbox_center_x', det['center'][0])
            if marker_size_mm > 0 and det.get('radius'):
                # radius 是映射回原始分辨率的像素，所以用 f_px_full 反推距离
                r = range_from_radius_px(det['radius'], marker_size_mm, FUSION_F_PX_FULL)
                if r is not None:
                    marker_range_mm = max(80.0, r)
        if cx is not None:
            # 第一次看到色块时，把它此刻的像素当成基准 cx0（保持初始方位，不怼画面中心）。
            # 之前 cx0 写死 160（画面中心），等于「朝色块走」；色块放在路线旁边时会把路线带偏。
            if fusion.updates == 0:
                fusion.cx0 = cx
                print('融合基准 cx0 → %.1f（色块初始像素）' % cx, flush=True)
            fusion.update_bearing(cx, marker_range_mm)

        turn, lateral = ctrl.decide(fusion.heading_error, fusion.cross_error)
        if turn != 0.0:
            if turn < 0:
                ik.turn_left(ik.initial_pos, 2, abs(turn), TURN_SPEED, 1)
            else:
                ik.turn_right(ik.initial_pos, 2, abs(turn), TURN_SPEED, 1)
            print('融合 e=%+.2f° cross=%+.0fmm -> 转%+.1f°'
                  % (fusion.heading_error, fusion.cross_error, turn), flush=True)
        elif lateral != 0.0:
            if lateral < 0:
                ik.left_move(ik.initial_pos, 2, abs(lateral), MOVE_SPEED, 1)
            else:
                ik.right_move(ik.initial_pos, 2, abs(lateral), MOVE_SPEED, 1)
            print('融合 e=%+.2f° cross=%+.0fmm -> 横移%+.0fmm'
                  % (fusion.heading_error, fusion.cross_error, lateral), flush=True)

        # IMU 增量：since_last 给的是「自上次调用以来」的平均角速度与时长
        rate, dt = tracker.since_last()
        # 约定：imu_state['yaw'] 是「左正」，本模块用「右正」，故取负
        d_theta_right = -rate * dt
        fusion.predict(d_theta_right, ds_mm=last_ds, lateral_mm=lateral)

        move = min(100, remaining)
        move_one_chunk(ik, move, forward)
        last_ds = move if forward else -move
        remaining -= move
        time.sleep(0.05)
    return marker_range_mm


def feed_turn_to_fusion(imu_state, tracker, fusion, yaw_before, cmd_right_deg):
    """转弯后把「实际转了多少 − 命令转了多少」喂给估计器。

    这样 `fusion` 里的 e 全程表示「相对理想路线的航向偏差」，转弯本身不会把它撑爆，
    转弯**残差**才会 —— 而残差正是要被相机看到并修掉的东西。

    yaw_before：转弯前的 imu_state['yaw']（左正）
    cmd_right_deg：本次命令的转角，右正（JSON 的 turn_left 90 应传 -90）
    """
    d_yaw_left = imu_state['yaw'] - yaw_before
    residual = -d_yaw_left - cmd_right_deg      # 右正：实际 − 命令
    fusion.predict(residual, ds_mm=0.0)
    # 把 since_last 的基准挪到「此刻」，否则直线段第一次取值会把转弯尾巴再积一遍
    tracker.since_last()
    print('转弯残差 %+.2f°（IMU 实测 %+.2f° / 命令 %+.1f°）-> e=%+.2f°'
          % (residual, -d_yaw_left, cmd_right_deg, fusion.heading_error), flush=True)
    return residual


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


def do_pick(board, pick_count, pulses=None, pull_up_pulse=None):
    """执行第 1/2 次夹取。"""
    if pick_count == 1:
        state = pick1_prepare(board, pulses)
    else:
        state = pick2_prepare(board, pulses)
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
    """主流程：按 JSON 调用移动、转弯、夹取和放下。"""
    parser = argparse.ArgumentParser(description='融合导航 + JSON 路线运行')
    parser.add_argument('--color', default='red',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=1)
    parser.add_argument('--pull-up', type=int, default=None,
                        help='第一次夹取后拔起的脉宽（22 号肩，默认 %d）；'
                             '幅度不合适现场试值' % PULL_UP_22)
    parser.add_argument('--f-px', type=float, default=FUSION_F_PX,
                        help='320 坐标系下的等效焦距像素（需标定）')
    parser.add_argument('--cx0', type=float, default=FUSION_CX0,
                        help='画面主点横坐标，即「色块正对机身」时的像素（需标定）')
    parser.add_argument('--f-px-full', type=float, default=FUSION_F_PX_FULL,
                        help='原始分辨率下的焦距像素，用色块视半径反推距离')
    parser.add_argument('--marker-size-mm', type=float, default=0.0,
                        help='色块真实半径（mm）；>0 时按视半径自动估距离，'
                             '否则用 --marker-range 的定值')
    parser.add_argument('--marker-range', type=float, default=1000.0,
                        help='到色块的前向距离（mm），--marker-size-mm=0 时生效')
    parser.add_argument('--fusion-head-gain', type=float, default=FUSION_HEAD_GAIN)
    parser.add_argument('--fusion-cross-gain', type=float, default=FUSION_CROSS_GAIN)
    args = parser.parse_args()

    log_file = start_run_log()

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
        task_server.start_server()
    report(state='CAPTURE',
           position_m={'x': 0.0, 'y': 0.0},
           heading_deg=0.0,
           picked_count=0,
           last_task={'task_id': 'capture', 'color': args.color},
           last_result=None,
           message='自动捕获，颜色=%s' % args.color)

    fusion = LaneFusion(f_px=args.f_px, cx0=args.cx0)
    ctrl = LaneController(head_gain=args.fusion_head_gain,
                          cross_gain=args.fusion_cross_gain)
    marker_range = args.marker_range
    print('融合导航已开启：相机写状态 / IMU 管执行 / 状态不归零 '
          '(f_px=%.0f cx0=%.0f)' % (args.f_px, args.cx0), flush=True)

    restore_travel(board, GRIPPER_OPEN)
    print('自动捕获启动，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    target_yaw = 0.0
    pending_forward = 0
    pick_count = 0
    place_count = 0
    picked_count = 0
    pose = {'x': 0.0, 'y': 0.0}

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
            target_yaw = imu_state['yaw']
            dist_mm = pending_forward
            # 融合导航：相机写状态、IMU 管执行，状态不归零；丢帧卡尔曼自己扛得住
            marker_range = move_straight_fusion(
                ik, board, detector, imu_state, imu_state['tracker'],
                pending_forward, fusion, ctrl, marker_range, args.marker_size_mm)
            pending_forward = 0
            advance_pose(pose, imu_state['yaw'], dist_mm)
            report(position_m=pose_dict(pose),
                   heading_deg=norm_heading(imu_state['yaw']),
                   message='直行 %dmm' % dist_mm)

        if name == 'turn_left':
            angle = int(act.get('angle', 90))
            print('%d/%d IMU左转 %d' % (i, len(actions), angle), flush=True)
            yaw_before = imu_state['yaw']
            imu_turn(ik, board, imu_state, angle)
            feed_turn_to_fusion(imu_state, imu_state['tracker'],
                                fusion, yaw_before, -angle)
            reset_imu(board, imu_state, reset_yaw=False)
            target_yaw = imu_state['yaw']
            report(heading_deg=norm_heading(imu_state['yaw']),
                   message='左转 %d°' % angle)
        elif name == 'turn_right':
            angle = int(act.get('angle', 90))
            print('%d/%d IMU右转 %d' % (i, len(actions), angle), flush=True)
            yaw_before = imu_state['yaw']
            imu_turn(ik, board, imu_state, -angle)
            feed_turn_to_fusion(imu_state, imu_state['tracker'],
                                fusion, yaw_before, angle)
            reset_imu(board, imu_state, reset_yaw=False)
            target_yaw = imu_state['yaw']
            report(heading_deg=norm_heading(imu_state['yaw']),
                   message='右转 %d°' % angle)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_pick(board, pick_count, pulses, pull_up_pulse=args.pull_up)
            picked_count += 1
            report(picked_count=picked_count,
                   message='第 %d 次夹取完成' % picked_count)
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_place(board, place_count, pulses)
            report(message='第 %d 次放下完成' % place_count)
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        target_yaw = imu_state['yaw']
        dist_mm = pending_forward
        move_straight_fusion(
            ik, board, detector, imu_state, imu_state['tracker'],
            pending_forward, fusion, ctrl, marker_range, args.marker_size_mm)
        advance_pose(pose, imu_state['yaw'], dist_mm)

    video_stop.set()
    cam.camera_close()
    tracker = imu_state.get('tracker')
    if tracker is not None:
        tracker.stop()
    ik.stand(ik.initial_pos, t=500)
    print('自动捕获运行结束', flush=True)
    report(state='END', last_result='done',
           position_m=pose_dict(pose),
           heading_deg=norm_heading(imu_state['yaw']),
           picked_count=picked_count,
           message='自动捕获完成')
    time.sleep(5)  # END 状态停留 5 秒，供中枢轮询确认

    if log_file is not None:
        log_file.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户中断，已退出', flush=True)
