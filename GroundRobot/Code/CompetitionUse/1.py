#!/usr/bin/python3
# coding=utf8
"""1.py —— 融合导航 + JSON 路线 + YOLO/深度夹取（独立脚本，只依赖 agcs_lib）。

走路：直线段走融合导航（相机写状态、IMU 管执行、先航向后横向）。默认 LaneFusion
（--fusion reference 可换 ReferenceFusion）。颜色检测只用于导航「保证不跑歪」。

夹取（pick）：路线把机身开到定点后，先只转 21 观测目标；观测到 → 靠近夹取：22/23 渐进前伸，
21 水平居中，24 不锁 JSON 角度、每步把目标往画面中间拉（保证目标一直看得见）。
判距和 AutonomousCrawling 同一套：用 bbox 框高估前方距离（焦距 × 目标高度 ÷ 框高），
估到 ≤STOP_DIST_CM 就夹（深度 ≤STOP_DEPTH_CM 也算，但 Astra Pro 近端 0.6m 是盲区，基本不触发）；
一直估不到够近就伸到 JSON 夹取位 +REACH_EXTRA 那个标定位姿再夹。
**收尾的唯一判据是「看不见了」，不是「估距够近了」**：只要模型还能认出目标就继续靠近，
直到①目标丢失且锁定估距 ≤LOST_NEAR_CM ②框顶快被画面上边裁掉
③手臂伸到 JSON 夹取位 +REACH_EXTRA 的终点 ④估距真的 ≤STOP_DIST_CM。
前两种进收尾：24 交还给路线 JSON 的夹取角（24 和夹爪同轴，一路跟着目标走会把夹爪带歪
→ 夹不到），22/23 按锁定的估距盲走完剩下的路再夹（见 finish_blind）。
只有还在远处就丢目标才算真丢，才原地摆动 24 找回、找不回退回固定脉宽。
观测不到 → 摆 22/23/24 固定脉宽夹取（兜底）。
place 仍读 json1.json 固定脉宽。全程默认全自动，无 input 阻塞。
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
from agcs_lib.heading_fusion import (
    LaneFusion,
    LaneController,
    ReferenceFusion,
    range_from_radius_px,
)
from agcs_lib.depth import DepthCamera

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
PULL_UP_MAX_DELTA = 80  # 拔起时 22 相对**实际夹取位**最多动这么多（判早了也不会一撸到底）
                        # 判据正常收敛时实际夹取位≈405，450-405=+45，在这个上限内
PICK1_RESTORE_24 = 260  # 第一次夹取结束后恢复时 24 号腕俯仰的脉宽（不是复位位 330）；
                        # 160 太低（相机朝下看不到目标、还可能照到夹爪里的方块），现场改 260。
# ---------- YOLO + 深度夹取（pick 用，走路部分不碰） ----------
DEFAULT_MODEL = 'models/v8n.onnx'  # YOLO 模型路径（相对 spiderpi 根目录）
MODEL_CONF = 0.8                   # 置信度阈值
# 距离判定：和 AutonomousCrawling 同一套 —— 用 bbox 框高按针孔模型估前方距离
#   dist_cm = 焦距 × 目标物理高度 ÷ 框高
# 目标高度取固定值（虫子/虫子模型按 5cm 算），框高随靠近变大、估出的距离就变小，
# 小于阈值就夹。不用 bbox 面积。
DIST_F_PX = 838.0                  # 640 分辨率下的焦距像素（同 AutonomousCrawling 的 F_PX）
BUG_HEIGHT_CM = 5.0                # 目标物理高度（cm），目标换了要跟着改
STOP_DIST_CM = 9.0                 # 估距 ≤ N cm 就夹（调小=更近）
                                   # 估距下限：框最高只能到 480px，838×5/480≈8.7cm，所以 9 基本
                                   # 就是「框快占满画面」；框被画面裁掉时按速率外推（同 AutonomousCrawling）
RELIABLE_MAX_H = 450               # 框高超过它=框被裁了，距离不可靠，改用速率外推
STOP_DEPTH_CM = 5.0                # 深度 ≤ N cm 也算够近（Astra Pro 近端 0.6m 内是盲区，基本不触发）
REACH_EXTRA = 45                   # 22/23 在 JSON 夹取位上额外前伸的量（--reach-extra 可调）
REACH_EXTRA_MAX = 90               # --reach-extra 的硬上限：再往前手臂太低，夹爪会杵到地面
                                   # 现场轨迹：20（收尾后停在 22=410）→「差一点夹到」→ 45
                                   # 注意：这个值同时是收尾盲走的终点，估距不够近时靠它兜底
GRAB_HOLD = 3                      # 判据要连续 N 帧成立才夹（单帧检测抖一下不能夹）
OBSERVE_TIMEOUT_S = 3.0            # 转 21 后观测目标的最长时间（秒）
# ---------- 目标丢失 = 「已经贴脸」的信号（盲走收尾） ----------
# 模型是远距离样本训的，近到 12~13cm 就认不出来了（现场实测每次都在这个距离丢）。
# 所以「丢目标」不是故障，是「够近了」的信号：那一刻把估距和角度**锁定**住，
# 不再试图重新识别，按当时的角度把剩下的距离（锁定估距 - STOP_DIST_CM）走完就夹。
LOST_CONFIRM = 3                   # 连续丢 N 帧才算真丢（单帧抖动不算）
LOST_NEAR_CM = 15.0                # 丢目标时锁定估距已 ≤ 它 → 判「近到认不出」，走盲走收尾
BLIND_MIN_STEPS = 2                # 盲走收尾最少步数
BLIND_MAX_STEPS = 25               # 盲走收尾最多步数（护栏，速率估飞了也不会一直走）
CM_PER_STEP_FALLBACK = 0.25        # 盲走速率还没测出来时的兜底（cm/步）
CM_PER_STEP_MAX = 0.35             # 每步距离下降速率的上限（cm/步）。实测健康值 0.15~0.25
                                   # （22 走 5 脉宽 ≈ 0.18cm），框高噪声会把速率带飞
                                   # （现场飘到 0.51、1.09），速率虚高 → 收尾盲走的步数算少
                                   # → 手臂没伸到位就停了（现场「差一点夹到」就是这么来的）
STICKY_TOL_CM = 0.5                # 「只许越来越近」的容差（cm）：框高抖 ±10px 就是 ±0.3cm
                                   # 的假波动，卡太死会把噪声一路棘轮下去
# ---------- 收尾：24 交还给路线 JSON 的夹取角 ----------
# 24 和夹爪装在同一块（同轴），靠近时靠它把目标留在画面里，但这一跟就把夹爪的俯仰角
# 一起带跑了：现场 24 一路 270 → 174，比路线 JSON 的夹取角 290 低了 116，
# 夹爪是歪着伸过去的，所以「看不到也夹不到」。一进收尾就把 24 还给 JSON。
#
# **没有「估距够近了就收尾」这条**（原来的 FINISH_CM=14 已于 2026-09-21 删掉）：
# 现场 13.3cm 时模型还认得出（conf 0.93、框顶 59 还在画面里），却被 14cm 判成收尾，
# 22/23 只伸到 490/305 就停了（终点是 380/340），白丢 22 步行程 → 差一点夹不到。
# 既然还看得见就该继续靠近，所以收尾一律等「看不见了」才触发：
#   ① 丢目标且锁定估距 ≤LOST_NEAR_CM  ② 框顶快被画面上边裁掉  ③ 手臂伸到终点
#   （④ 估距真到 STOP_DIST_CM 是正常的夹取判据，不算收尾）
FINISH_EXTRA_CM = 2.0              # 收尾盲走**多走**这么多（现场「差一点夹到，再伸过去一点」：
                                   # 1cm 还差一点，2026-09-21 加到 2cm）。注意它受 22/23 终点
                                   # 限制：终点不够前时会被终点截住，这时要调 --reach-extra
                                   # 把终点推出去，光加这个数没用
# ---------- 靠近夹取（22/23 渐进前伸，21 水平居中，24 只管把目标留在画面里） ----------
APPROACH_D = 5                     # 每步 22/23 朝目标脉宽靠近的最大量（越小越稳）
APPROACH_STEPS = 90                # 靠近最多步数
APPROACH_SLEEP = 0.15              # 每步间隔（秒），要留够一次识别的时间
IMG_CX = 320                       # 画面中心 x（640×480），21 水平跟踪用
TRACK_P = 0.1                      # 21 跟踪 P 增益
TRACK_DEAD_X = 40                  # 21 水平死区（像素）
# 24 号靠近时**不锁 JSON 角度**，只跟着目标走，保证它一直在画面里。
# 22/23 前伸会把相机整个甩出去，24 不跟的话目标一步就飘出画面 → 后面再也认不到，
# 所以这里必须每步都跟（P 控制），而不是只在画面边缘才动手。
CAM24_MIN = 160                    # 24 下限（再小=太朝下，会照到自己的夹爪）
CAM24_MAX = 360                    # 24 上限（再大=太朝上，目标跑出画面底部）
# 靠近到最后目标框会长到 300~400px 高，还一味按「框中心对 190」往下压 24，框顶就顶出
# 画面上边（框被裁 → 模型认不出 → 现场丢目标就是这么来的）。所以瞄准行加个下限：
# 框顶至少留 CAM24_TOP_MARGIN 像素，框越大瞄准行自动越低，24 就不会一直往下推。
CAM24_TOP_MARGIN = 40              # 框顶离画面上边至少留这么多像素（再小就顶出去了）
CAM24_STEP = 6                     # 丢目标后找回时每次摆动的脉宽
CAM24_AIM_CY = 190                 # 24 把目标**框中心**往画面这一行拉（越靠上=相机越朝下）。
                                   # 相机装在夹爪**上面**，看的是目标偏上的位置，所以这一行
                                   # 要比画面正中（240）更靠上。实际瞄准行会被框高顶下去
                                   # （见 CAM24_TOP_MARGIN），框越大越接近画面中部
CAM24_DEAD_Y = 25                  # 俯仰死区（像素）：目标在瞄准行 ±25 内不动 24
                                   # （靠近末段框长得快，死区大了跟不上，框顶就顶出去了）
CAM24_P = 0.06                     # 俯仰 P 增益（像素→脉宽），越大跟得越急
MOVE_SPEED = 50      # 六足直线前进/后退的速度，越大走得越快
TURN_SPEED = 30      # 六足左转/右转的速度，越大转得越快
STRIDE_SCALE = 0.95   # 前进/后退名义步幅的缩放系数；现场实测 0.946 短一点、0.96 又过了，0.95 折中
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
# f_px 已现场标定（probe_f_px.py，2026-09-17）；cx0 运行时自动抓色块初始像素，不用填。
FUSION_F_PX = 419.0
FUSION_CX0 = 160.0
FUSION_F_PX_FULL = 838.0   # 原始 640 分辨率焦距 = 2×FUSION_F_PX，色块视半径反推距离用
FUSION_HEAD_GAIN = 0.9     # 航向 P 增益：转「误差 × 该比例」度
FUSION_CROSS_GAIN = 0.35   # 横向 P 增益：横移「偏差 × 该比例」mm
MARKER_SIZE_MM = 62.5      # 红色方块真实半径（mm，现场量直径 12.5cm → 半径 6.25cm=62.5mm）；
                           # >0 时按视半径动态估距离，不用再管 marker_range 那个固定值（距离随接近一直变）
STRAIGHT_MIN_MM = 400      # 直行段长度 >= 它才算「真直线」（重新开启颜色识别）；
                           # 弯道里那些 80/150mm 的小直行不算，颜色识别保持关闭
# --- 相机俯仰跟踪（接近时把 24 往下压，保证方块一直留在画面里不丢）---
CAM_TRACK_MIN_24 = 160     # 24 号往下压的下限（太小=太朝下）
CAM_TRACK_MAX_24 = 360     # 24 号往上抬的上限
CAM_TRACK_STEP = 6         # 每块俯仰调整步长（脉宽）
CAM_TRACK_CY_LOW = 185     # 方块中心 cy 超过它（快出画面底部）→ 往下压 24
CAM_TRACK_CY_HIGH = 55     # 方块中心 cy 低于它（快出画面顶部）→ 往上抬 24
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


def read_battery_v(board, samples=20):
    """读电池电压（V），连续采样取中位数。只用于记录分析，不参与控制。失败返回 None。"""
    try:
        vs = []
        for _ in range(samples):
            v = board.get_battery()
            if v is not None:
                vs.append(float(v))
            time.sleep(0.03)
        if not vs:
            return None
        vs.sort()
        return vs[len(vs) // 2] / 1000.0
    except Exception:
        return None


def log_battery(board, tag=''):
    """把电池电压写进日志（print 会被 start_run_log 的 _Tee 抄进日志文件）。"""
    v = read_battery_v(board)
    if v is None:
        print('电池电压%s读取失败' % (' ' + tag if tag else ''), flush=True)
    else:
        print('电池电压%s %.2f V' % (' ' + tag if tag else '', v), flush=True)
    return v


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


def restore_travel(board, gripper, s24=None):
    """恢复 21-24 到官方初始位置（s24 非 None 时覆盖 24 号脉宽），并设置 25 夹爪状态。"""
    p = dict(OFFICIAL_ARM)
    if s24 is not None:
        p[24] = int(s24)
    board.bus_servo_set_position(1.5, [[sid, p[sid]] for sid in [22, 23, 21, 24]])
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


def arm_fine_tune(board, state, kind, pull_up=False, pull_up_pulse=None, restore_s24=None):
    """机械臂手动微调，回车执行夹取/放下。restore_s24 非 None 时恢复阶段 24 号用它。"""
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
    restore_travel(board, gripper, s24=restore_s24)


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
    """打开摄像头，返回 (cam, read_frame, detector, publish)。

    read_frame：读一帧并去畸变（颜色导航和 YOLO 共用这一路，只开一个 /dev/video0）。
    detector：颜色检测（融合导航用），内部调 read_frame + 推流。
    publish：推流（YOLO 检测也用它推标注画面）。
    """
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    def read_frame():
        with camera_lock:
            f = capture(cam)
        if f is None:
            return None
        return cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)

    def publish(frame):
        if task_server is not None:
            task_server.publish_frame(frame, max_fps=10.0)
            task_server.publish_lab_frame(lab_view(frame, lab, color), max_fps=10.0)

    def detector():
        frame = read_frame()
        if frame is None:
            return None
        result = detect_color(frame, lab, color, min_area=min_area)
        if result is not None:
            x, y, w, h = cv2.boundingRect(result['contour'])
            ul, ur = x, x + w
            result['bbox_center_x'] = (ul + ur) / 2.0
            cx, cy = result['center']
            cv2.circle(frame, (cx, cy), int(result.get('radius', 20)), (0, 255, 0), 2)
            cv2.putText(frame, color, (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        publish(frame)
        return result

    return cam, read_frame, detector, publish


def depth_cm_at(depth, cx, cy, rotate):
    """读深度相机在彩色像素 (cx,cy) 处的距离(cm)；无有效读数返回 None。"""
    try:
        d = depth.read_depth(timeout_ms=200)
        if d is None:
            return None
        if rotate:
            d = correct_camera(d, rotate)
        h, w = d.shape
        dx = min(w - 1, max(0, int(round(cx))))
        dy = min(h - 1, max(0, int(round(cy))))
        z_mm = int(d[dy, dx])
        if z_mm <= 0:
            return None
        return z_mm / 10.0
    except Exception:
        return None


class ModelDetector:
    """ONNX YOLO 检测器（onnxruntime 本地推理），detect() 返回 {'x','y','w','h','conf'} 或 None。

    从 Auto-capture-1.py 拷过来，让 1.py 独立自包含（只依赖 agcs_lib），将来好直接顶替 NO6/NO7。
    """

    NAME = 'fake bug'  # 目标类别名

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
        shp = self.sess.get_inputs()[0].shape
        self.in_h, self.in_w = int(shp[2]), int(shp[3])

    def _letterbox(self, img):
        """等比缩放到模型输入尺寸补灰边，返回 (画布, 缩放比, pad_x, pad_y)。"""
        h0, w0 = img.shape[:2]
        ih, iw = self.in_h, self.in_w
        r = min(iw / w0, ih / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        pad_x, pad_y = (iw - new_w) // 2, (ih - new_h) // 2
        canvas = np.full((ih, iw, 3), 114, dtype=np.uint8)
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

        nc = out.shape[0] - 4   # 类别数（单类=1，多类=16）
        best = None  # (x1, y1, x2, y2, score)
        for i in range(out.shape[1]):
            scores = out[4:4 + nc, i]
            cls = int(scores.argmax())
            score = float(scores[cls])
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


def video_loop(detectors, stop_event):
    """后台持续取帧推流：颜色 + YOLO 都叠加推流显示（同 Auto-capture-1.py）。"""
    while not stop_event.is_set():
        for d in detectors:
            d()
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
    """只走一小段前进或后退（按 STRIDE_SCALE 缩放实际步幅）。"""
    m = int(round(move * STRIDE_SCALE))
    if forward:
        ik.go_forward(ik.initial_pos, 2, m, MOVE_SPEED, 1)
    else:
        ik.back(ik.initial_pos, 2, m, MOVE_SPEED, 1)


def move_straight_fusion(ik, board, detector, imu_state, tracker, distance_mm,
                         fusion, ctrl, marker_range_mm, marker_size_mm, cam_state,
                         color_enabled=True):
    """融合模式的直线段：相机写状态、IMU 管执行，单一控制器先航向后横向。

    与 `move_straight_imu_color` 的本质区别：
    - 相机不再「平移去消掉像素偏移」，而是更新 (航向误差, 横向偏差) 这个估计；
    - 控制器同一小块里**只发一种指令**：航向没进死区就只转，进了才横移；
    - `fusion` 的状态全程不归零（归零 = 把误差从数字搬到机器人身上）。

    cam_state：相机 24 号俯仰跟踪状态 {'pulse': 当前脉宽}。接近时方块往下掉出画面，
    就往把 24 往下压，保证方块一直留在画面里，融合才不会丢参照。
    color_enabled：False 时跳过颜色检测（弯道里用），整段只靠 IMU 走。
    """
    remaining = abs(int(distance_mm))
    forward = distance_mm >= 0
    while remaining > 0:
        det = detector() if color_enabled else None
        cx = None
        if det is not None:
            cx = det.get('bbox_center_x', det['center'][0])
            # 相机俯仰跟踪：方块快出画面底部就往下压 24，快出顶部就往上抬。
            cy = det['center'][1]
            if cy > CAM_TRACK_CY_LOW and cam_state['pulse'] > CAM_TRACK_MIN_24:
                cam_state['pulse'] = max(CAM_TRACK_MIN_24, cam_state['pulse'] - CAM_TRACK_STEP)
                board.bus_servo_set_position(0.15, [[24, cam_state['pulse']]])
            elif cy < CAM_TRACK_CY_HIGH and cam_state['pulse'] < CAM_TRACK_MAX_24:
                cam_state['pulse'] = min(CAM_TRACK_MAX_24, cam_state['pulse'] + CAM_TRACK_STEP)
                board.bus_servo_set_position(0.15, [[24, cam_state['pulse']]])
            if marker_size_mm > 0 and det.get('radius'):
                # radius 是映射回原始分辨率的像素，所以用 f_px_full 反推距离
                r = range_from_radius_px(det['radius'], marker_size_mm, FUSION_F_PX_FULL)
                if r is not None:
                    marker_range_mm = max(80.0, r)
        if cx is not None:
            if not isinstance(fusion, ReferenceFusion) and fusion.updates == 0:
                # LaneFusion 旧行为：把色块初始像素冻成基准（会「朝方块走」）。
                # ReferenceFusion 内部自己跟踪方块位置 (bx,by)，不需要这个 hack。
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
        move = min(100, remaining)
        fusion.predict(d_theta_right, ds_mm=(move if forward else -move),
                       lateral_mm=lateral)
        move_one_chunk(ik, move, forward)
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
    if isinstance(fusion, ReferenceFusion):
        # 方块位置按「命令转角」推进（直行的命令转角=0 由 predict 缺省处理），
        # e 只累积残差 —— 这样方块参照系不会跟着转弯残差歪掉。
        fusion.predict(residual, ds_mm=0.0, d_theta_cmd_deg=cmd_right_deg)
    else:
        fusion.predict(residual, ds_mm=0.0)
        # LaneFusion 用冻结的 cx0 当基准，转弯后方块相对机身的方位变了，把 cx0 按
        # 「反方向转角」转过去，否则融合会把转弯当成漂移、又把机器人转回直线。
        fusion.rotate_reference(-cmd_right_deg)
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

    # 反复用 IMU 修正，直到误差进死区（最多 5 次）。现场每次转弯都往外多转 1~3°，
    # 单次 1° 修正不够，残差会累加到十几度把机器人带出弯道。
    for _ in range(5):
        err = angle_error(imu_state['yaw'], target)
        if abs(err) <= HEADING_TOL_DEG:
            break
        step = IMU_STRAIGHT_STEP if err > 0 else -IMU_STRAIGHT_STEP
        if step > 0:
            ik.turn_left(ik.initial_pos, 2, abs(step), TURN_SPEED, 1)
        else:
            ik.turn_right(ik.initial_pos, 2, abs(step), TURN_SPEED, 1)
        time.sleep(0.2)
        print('转弯后修正 %+d°（当前误差 %+.1f°）' % (step, err), flush=True)


def _step_toward(cur, target, step):
    """把 cur 朝 target 移动一步（最多 step），不越过 target。"""
    if cur < target:
        return min(target, cur + step)
    if cur > target:
        return max(target, cur - step)
    return cur


def _clamp24(pulse):
    """把 24 号脉宽夹到靠近时可用的范围内。"""
    return max(CAM24_MIN, min(CAM24_MAX, clamp_pulse(pulse)))


def reacquire(board, model_det, y24, pick_count, step):
    """靠近途中目标丢了：先停住别伸，小幅摆动 24 把它找回来。

    22/23 一伸，目标就从画面里滑出去；不找回来越伸越瞎，所以丢帧时宁可原地
    上下扫几下。试探位是相对当前值的 -1/-2/+1/+2 个 CAM24_STEP（都在原位附近），
    找到就停在找到的角度；四个都试过还没有就摆回原位、返回 None。

    返回 (det, y24)。
    """
    for k, d in enumerate((-CAM24_STEP, -2 * CAM24_STEP, CAM24_STEP, 2 * CAM24_STEP)):
        p = _clamp24(y24 + d)
        board.bus_servo_set_position(0.25, [[24, p]])
        time.sleep(0.25)
        det = model_det.detect()
        print('pick%d 靠近 #%d 目标丢失，找目标 %d/4（24=%d）%s'
              % (pick_count, step, k + 1, p, '找到了' if det is not None else ''),
              flush=True)
        if det is not None:
            return det, p
    board.bus_servo_set_position(0.25, [[24, y24]])
    time.sleep(0.25)
    return None, y24


def do_pick(board, ik, model_det, depth, rotate, pick_count, pulses,
            pull_up_pulse=None, no_depth=False, manual=False, s24_now=None,
            stop_dist=None, reach_extra=None):
    """执行第 1/2 次夹取。默认全自动：转 21 观测 → 靠近夹取 / 固定夹取。

    先只转 21 号到夹取方向再观测（不动 22/23/24）：
    - 观测到目标 → 靠近夹取（22/23 渐进前伸，21 水平居中，24 每步把目标往画面中间拉；
      框高估距 ≤STOP_DIST_CM 就夹；近到模型认不出（≈12~13cm）就锁定记忆目标盲走收尾；
      还在远处丢目标才摆 24 找回，找不回则退回固定脉宽夹取）；
    - 观测不到 → 摆 22/23/24 到 JSON 固定脉宽夹取（兜底）。
    --manual 退回原来的手动回车微调。
    """
    if manual:
        if pick_count == 1:
            state = pick1_prepare(board, pulses)
        else:
            state = pick2_prepare(board, pulses)
        arm_fine_tune(board, state, 'pick', pull_up=(pick_count == 1),
                      pull_up_pulse=pull_up_pulse,
                      restore_s24=(PICK1_RESTORE_24 if pick_count == 1 else None))
        return

    state = dict(pulses)
    stop_cm = STOP_DIST_CM if stop_dist is None else float(stop_dist)
    reach = REACH_EXTRA if reach_extra is None else int(reach_extra)
    reach = max(0, min(REACH_EXTRA_MAX, reach))     # 硬上限：再往前夹爪会杵到地面
    # 1. 先只转 21 到夹取方向（不动 22/23/24），再观测目标（给足时间让模型识别）
    board.bus_servo_set_position(1.0, [[21, state[21]]])
    time.sleep(1.0)
    observed = False
    deadline = time.time() + OBSERVE_TIMEOUT_S
    while time.time() < deadline:
        if model_det.detect() is not None:
            observed = True
            break
        time.sleep(0.2)

    if not observed:
        # 2a. 观测不到 → 固定夹取：摆 22/23/24 到 JSON 位
        print('pick%d 观测不到目标，固定夹取' % pick_count, flush=True)
        if pick_count == 1:
            set_servos(board, state, [22, 23, 24])
        else:
            set_servos(board, state, [22, 23])
            board.bus_servo_set_position(2.2, [[24, 500]])
            time.sleep(2.2)
            set_servos(board, state, [24])
        cur_22 = state[22]
    else:
        # 2b. 观测到了 → 靠近夹取：22/23 渐进前伸，21 水平居中，24 跟目标（保证它一直在画面里）。
        #     **只要还看得见就一直靠近**，不看「估距够近了没」。等真的看不见了（丢目标 /
        #     框顶贴边）或手臂伸到终点才收尾：24 交还给 JSON 夹取角，22/23 按锁定的估距
        #     盲走完剩下的路再夹。
        print('pick%d 观测到目标，进入靠近夹取' % pick_count, flush=True)
        w22 = OFFICIAL_ARM[22]   # 705
        z23 = OFFICIAL_ARM[23]   # 90
        # 24 从**当前实际值**起步，不要拉回复位位 330：走路段的颜色跟踪会把 24 压低
        # （CAM_TRACK_MIN_24=160），这里一拉回 330 相机就猛地往上翘一下，然后才开始
        # 靠近——现场看到的就是这个「翘一下」。观测阶段只动 21，所以 24 还在走路留下的位置。
        y24 = _clamp24(OFFICIAL_ARM[24] if s24_now is None else s24_now)
        s21 = state[21]          # 21 跟踪起点（转 21 后的实际值）
        t22 = state[22] - reach        # 22 终点：比 JSON 更低、更前伸
        t23 = state[23] + reach        # 23 终点：比 JSON 更伸展
        hit = 0            # 判据连续成立的帧数
        miss = 0           # 连续丢帧数
        last_dist = None   # 最近一次可靠估距（cm）
        mem_cm = None      # 记忆距离：上一帧算出来的估距（含外推），收尾那一刻就是它
        mem_cx = None      # 记忆横向：最后一次看到目标时的中心 x
        sticky = False     # 进过外推区：之后不许估距再「变远」（框重新变完整≠真的变远）
        dist_rate = 0.0    # 每步距离下降速率（cm/步），实测自适应
        since_rel = 0      # 距上次可靠估距过了多少步

        def finish_blind(reason, mem_cm, s21, w22, z23, y24):
            """收尾：24 交还给路线 JSON 的夹取角，22/23 按锁定的估距盲走完剩下的路再夹。

            24 是相机的眼睛（找目标、量距离），可它和夹爪同轴，一路跟着目标走会把
            夹爪的俯仰角一起带跑——现场 24 从 270 一路收到 174，比路线 JSON 的夹取角
            290 低了 116，夹爪是歪着伸过去夹的，所以夹不到。进了收尾 24 就还给 JSON，
            后面只靠「锁定的估距 + 最后一眼的 21 角度」，相机的活到此为止。
            """
            y24 = clamp_pulse(state[24])
            board.bus_servo_set_position(0.6, [[24, y24]])
            time.sleep(0.6)
            if mem_cm is None:
                rate, need_cm, n_blind = 0.0, 0.0, BLIND_MIN_STEPS
            else:
                rate = min(dist_rate if dist_rate > 0.05 else CM_PER_STEP_FALLBACK,
                           CM_PER_STEP_MAX)
                need_cm = max(0.0, mem_cm - stop_cm) + FINISH_EXTRA_CM
                n_blind = min(BLIND_MAX_STEPS, max(BLIND_MIN_STEPS, int(need_cm / rate) + 1))
            print('pick%d 收尾（%s）：24 回路线 JSON 夹取角 %d（再跟着目标走夹爪会歪），'
                  '锁定估距 %s → %.1fcm，剩 %.1fcm，按 %.2fcm/步 走 %d 步'
                  % (pick_count, reason, y24,
                     '%.1fcm' % mem_cm if mem_cm is not None else '--',
                     stop_cm, need_cm, rate, n_blind), flush=True)
            # 按最后一眼的横向偏差对正 21；在死区内就别动，保持当时的角度直着走
            if mem_cx is not None and abs(mem_cx - IMG_CX) >= TRACK_DEAD_X:
                s21 = clamp_pulse(s21 + int(TRACK_P * (IMG_CX - mem_cx)))
                board.bus_servo_set_position(0.3, [[21, s21]])
                time.sleep(0.3)
            for _k in range(n_blind):
                w22 = _step_toward(w22, t22, APPROACH_D)
                z23 = _step_toward(z23, t23, APPROACH_D)
                board.bus_servo_set_position(APPROACH_SLEEP, [[22, w22], [23, z23], [24, y24]])
                time.sleep(APPROACH_SLEEP)
                if w22 == t22 and z23 == t23:
                    print('pick%d 收尾：22/23 已到路线 JSON 终点（%d/%d 步），手臂伸不动了，'
                          '就地闭夹爪' % (pick_count, _k + 1, n_blind), flush=True)
                    break
            print('pick%d 收尾结束 21=%d 22=%d 23=%d 24=%d，闭夹爪'
                  % (pick_count, s21, w22, z23, y24), flush=True)
            return s21, w22, z23, y24

        for step in range(APPROACH_STEPS):
            det = model_det.detect()
            if det is None:
                miss += 1
                if miss < LOST_CONFIRM:
                    # 单帧抖动：原地等一帧，不伸也不扫（摸黑伸出去只会越走越瞎）
                    print('pick%d 靠近 #%d 目标丢失 %d/%d，先等一帧'
                          % (pick_count, step, miss, LOST_CONFIRM), flush=True)
                    time.sleep(APPROACH_SLEEP)
                    continue
                if mem_cm is not None and mem_cm <= LOST_NEAR_CM:
                    # 近到模型认不出 —— 这是信号不是故障，直接收尾
                    print('pick%d 靠近 #%d 目标丢失（模型只在远处训练过，%.1fcm 认不出）= 已贴脸'
                          % (pick_count, step, mem_cm), flush=True)
                    s21, w22, z23, y24 = finish_blind(
                        '目标认不出（锁定 %.1fcm）' % mem_cm, mem_cm, s21, w22, z23, y24)
                    break
                # 还在远处就丢，那才是真丢 → 原地摆 24 找回来
                det, y24 = reacquire(board, model_det, y24, pick_count, step)
                if det is None:
                    print('pick%d 靠近中目标丢失且找不回（锁定估距 %s），改用 JSON 固定脉宽夹取'
                          % (pick_count, '%.1fcm' % mem_cm if mem_cm is not None else '--'),
                          flush=True)
                    set_servos(board, state, [22, 23, 24])
                    w22 = state[22]
                    break
                miss = 0
                continue
            miss = 0
            cx = det['x'] + det['w'] / 2.0
            cy = det['y'] + det['h'] / 2.0
            box_h = det['h']
            # 框碰到画面上下边 = 被画面裁了，这帧框高不可信（靠近到最后目标会长到出画）
            clipped = det['y'] <= 1 or (det['y'] + box_h) >= 479
            # 24 的瞄准行：框小的时候是偏上的那一行；框长大到快顶出画面上边就让开
            aim_cy = max(CAM24_AIM_CY, CAM24_TOP_MARGIN + box_h / 2.0)
            # 距离：焦距 × 目标高度 ÷ 框高（同 AutonomousCrawling）
            if not clipped and 0 < box_h < RELIABLE_MAX_H:
                est_cm = DIST_F_PX * BUG_HEIGHT_CM / box_h
                cur_cm = est_cm
                if sticky and mem_cm is not None and est_cm > mem_cm + STICKY_TOL_CM:
                    # 只许越来越近：进过外推区之后再看到「又变远了一大截」的读数（框重新
                    # 变完整，其实识别质量已经掉了），不能让它把估计值拉回去、把判据清零。
                    # 容差留着放框高抖动，不然噪声会被一路棘轮往下推。
                    cur_cm = mem_cm
                if last_dist is not None and cur_cm < last_dist:
                    rate = min(last_dist - cur_cm, CM_PER_STEP_MAX)
                    dist_rate = rate if dist_rate <= 0 else 0.7 * dist_rate + 0.3 * rate
                last_dist = cur_cm
                since_rel = 0
            else:
                # 框被裁：按前面测到的下降速率外推，别让距离卡住不降（同 AutonomousCrawling）
                since_rel += 1
                sticky = True
                est_cm = None
                cur_cm = None if last_dist is None else max(0.0, last_dist - dist_rate * since_rel)
            if cur_cm is not None:
                mem_cm = cur_cm      # 记忆距离：丢目标那一刻就按它算剩下的路
            mem_cx = cx              # 记忆横向：最后一次看到目标的位置
            dep_cm = None if (no_depth or depth is None) else depth_cm_at(depth, cx, cy, rotate)
            close = (cur_cm is not None and cur_cm <= stop_cm) or \
                    (dep_cm is not None and dep_cm < STOP_DEPTH_CM)
            hit = hit + 1 if close else 0
            print('pick%d 靠近 #%d conf=%.2f 中心=(%.0f,%.0f) 框=%dx%d 框顶=%d 瞄准=%.0f 距离=%s%s 深度=%s 22=%d 23=%d 24=%d%s'
                  % (pick_count, step, det['conf'], cx, cy, det['w'], box_h, det['y'], aim_cy,
                     '%.1fcm' % cur_cm if cur_cm is not None else '--',
                     '(外推)' if est_cm is None else ('(锁定)' if sticky else ''),
                     '%.1fcm' % dep_cm if dep_cm is not None else '--',
                     w22, z23, y24,
                     ' -> 到距离 %d/%d' % (hit, GRAB_HOLD) if close else ''), flush=True)
            if hit >= GRAB_HOLD:
                break
            # 这里**故意没有**「估距够近就收尾」那一条（2026-09-21 删）：还看得见就继续靠近，
            # 上面 hit>=GRAB_HOLD（估距真到 STOP_DIST_CM）才是够近的正常出口。
            # 收尾（看得见的最后一步）：框顶快贴到画面上边了 —— 再跟下去就要被裁掉、模型认不出
            if det['y'] <= CAM24_TOP_MARGIN:
                s21, w22, z23, y24 = finish_blind(
                    '框顶 %d 快被画面上边裁掉' % det['y'],
                    cur_cm if cur_cm is not None else mem_cm, s21, w22, z23, y24)
                break
            # 21：水平跟踪，把目标拉回画面中心
            if abs(cx - IMG_CX) >= TRACK_DEAD_X:
                s21 = clamp_pulse(s21 + int(TRACK_P * (IMG_CX - cx)))
                board.bus_servo_set_position(0.02, [[21, s21]])
            # 24：不锁 JSON 角度，每步把目标往画面偏上拉（保证它一直在画面里）。
            # 瞄准行 = max(偏上的那一行, 框顶留够边距时框中心能到的最高行)：
            # 框小的时候就是 190（偏上），框长到 300~400px 后自动往下让，
            # 免得框顶被画面上边裁掉——一裁模型就认不出，那就是现场丢目标的原因。
            if abs(cy - aim_cy) >= CAM24_DEAD_Y:
                y24 = _clamp24(y24 + int(CAM24_P * (aim_cy - cy)))
            # 22/23 朝终点前伸一步（不越过终点）
            w22 = _step_toward(w22, t22, APPROACH_D)
            z23 = _step_toward(z23, t23, APPROACH_D)
            board.bus_servo_set_position(APPROACH_SLEEP, [[22, w22], [23, z23], [24, y24]])
            time.sleep(APPROACH_SLEEP)
            if w22 == t22 and z23 == t23:
                break
        # 一次收尾都没触发就到了终点（手臂伸到头）/ 步数用尽：24 也还给 JSON 夹取角。
        # 24 和夹爪同轴，夹之前必须把它掰回标定过的角度，否则夹爪是歪的。
        if abs(y24 - state[24]) >= 10:
            y24 = clamp_pulse(state[24])
            board.bus_servo_set_position(0.6, [[24, y24]])
            time.sleep(0.6)
            print('pick%d 靠近结束（没触发收尾）：24 回路线 JSON 夹取角 %d，21=%d 22=%d 23=%d'
                  % (pick_count, y24, s21, w22, z23), flush=True)
        cur_22 = w22

    # 3. 夹取：闭 25 + 抬 22（仅第一次）+ 复位
    board.bus_servo_set_position(2.0, [[25, GRIPPER_CLOSE]])
    time.sleep(2.0)
    time.sleep(0.5)
    if pick_count == 1:
        pulse = PULL_UP_22 if pull_up_pulse is None else clamp_pulse(pull_up_pulse)
        # 上限保护：pulse 是按「正常夹取位≈405」定的绝对值，判早了（22 还很高）直接跳过去
        # 会一撸到底。这里限制相对**实际夹取位**最多动 PULL_UP_MAX_DELTA。
        pulse = clamp_pulse(max(cur_22 - PULL_UP_MAX_DELTA,
                                min(cur_22 + PULL_UP_MAX_DELTA, pulse)))
        print('拔起：22 号肩舵机 %d → %d（抬升 %+d）%s'
              % (cur_22, pulse, pulse - cur_22,
                 '（已限幅 ±%d）' % PULL_UP_MAX_DELTA if abs(pulse - cur_22) >= PULL_UP_MAX_DELTA else ''),
              flush=True)
        board.bus_servo_set_position(1.0, [[22, pulse]])
        time.sleep(1.0)
    restore_travel(board, GRIPPER_CLOSE,
                   s24=(PICK1_RESTORE_24 if pick_count == 1 else None))


def do_place(board, place_count, pulses=None, manual=False):
    """执行第 1/2 次放下。默认全自动，--manual 退回手动回车微调。"""
    if place_count == 1:
        state = place1_prepare(board, pulses)
    else:
        p = pulses or dict(OFFICIAL_ARM)
        print('place2：使用记录的 21-24 放下脉宽', flush=True)
        set_servos(board, p, [21, 22, 23, 24])
        state = dict(p)
    if manual:
        arm_fine_tune(board, state, 'place')
        return
    board.bus_servo_set_position(2.0, [[25, GRIPPER_OPEN]])
    time.sleep(2.0)
    time.sleep(0.5)
    restore_travel(board, GRIPPER_OPEN)


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
    global STRIDE_SCALE
    parser = argparse.ArgumentParser(description='融合导航 + JSON 路线运行')
    parser.add_argument('--color', default='red',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=1)
    parser.add_argument('--pull-up', type=int, default=None,
                        help='第一次夹取后拔起的脉宽（22 号肩，默认 %d）；'
                             '幅度不合适现场试值' % PULL_UP_22)
    parser.add_argument('--stride-scale', type=float, default=STRIDE_SCALE,
                        help='前进/后退步幅缩放（实际步幅偏大就 <1，如 0.946 抵消多走的 5.7%%）')
    parser.add_argument('--f-px', type=float, default=FUSION_F_PX,
                        help='320 坐标系下的等效焦距像素（需标定）')
    parser.add_argument('--cx0', type=float, default=FUSION_CX0,
                        help='画面主点横坐标，即「色块正对机身」时的像素（需标定）')
    parser.add_argument('--f-px-full', type=float, default=FUSION_F_PX_FULL,
                        help='原始分辨率下的焦距像素，用色块视半径反推距离')
    parser.add_argument('--marker-size-mm', type=float, default=MARKER_SIZE_MM,
                        help='色块真实半径（mm）；>0 时按视半径动态估距离，'
                             '否则用 --marker-range 的定值（固定值不推荐）')
    parser.add_argument('--marker-range', type=float, default=610.0,
                        help='到色块的前向距离（mm），--marker-size-mm=0 时生效')
    parser.add_argument('--fusion-head-gain', type=float, default=FUSION_HEAD_GAIN)
    parser.add_argument('--fusion-cross-gain', type=float, default=FUSION_CROSS_GAIN)
    parser.add_argument('--fusion-head-deadzone', type=float, default=1.0,
                        help='航向死区（度）：|e| 小于它就不转。横移会踢航向，调大到 2 可避免「修航向修出外偏」')
    parser.add_argument('--fusion-cross-deadzone', type=float, default=8.0,
                        help='横向死区（mm）：|cross| 小于它就不横移。调大到 20 可少发横移、少踢航向')
    parser.add_argument('--fusion', default='lane', choices=['lane', 'reference'],
                        help='直线段融合算法：lane=旧的 LaneFusion（默认，稳定），'
                             'reference=按路线走、方块当参照（B 方案，试验中）')
    parser.add_argument('--model', default=DEFAULT_MODEL, help='YOLO 模型路径（pick 用）')
    parser.add_argument('--conf', type=float, default=MODEL_CONF, help='YOLO 置信度阈值')
    parser.add_argument('--classes', default='', help='YOLO 目标类别，逗号分隔；留空=接受所有')
    parser.add_argument('--no-depth', action='store_true',
                        help='关掉深度相机，只用「目标中心→夹爪像素」判够近')
    parser.add_argument('--stop-dist', type=float, default=STOP_DIST_CM,
                        help='框高估距 ≤ N cm 就夹（默认 %(default)s）')
    parser.add_argument('--reach-extra', type=int, default=REACH_EXTRA,
                        # 注意：help 里既有 %(default)d（argparse 后面会格式化一次），
                        # 又有上限值要拼进去 —— 只能用 + 拼字符串，不能再套一层 % 格式化
                        help='22/23 越过路线 JSON 夹取位再前伸的脉宽（默认 %(default)d，'
                             '上限 ' + str(REACH_EXTRA_MAX) +
                             '）。夹不到、差一点就把它调大；太大会让夹爪杵到地面')
    parser.add_argument('--manual', action='store_true',
                        help='夹取/放下恢复手动回车微调（调试用）')
    args = parser.parse_args()

    STRIDE_SCALE = args.stride_scale

    log_file = start_run_log()

    with open(ROUTE_PATH, 'r', encoding='utf-8') as f:
        actions = json.load(f)

    board = make_board()
    ik = make_ik(board)
    imu_state = init_imu(board)
    cam, read_frame, detector, publish = open_vision(args.color, args.min_area)
    log_battery(board, tag='启动')   # 只记录分析，不参与控制

    # YOLO 检测器（pick 用）：独立自包含，跟颜色导航共用 read_frame/publish
    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    model_det = ModelDetector(model_path, args.conf, classes, read_frame, publish)

    # 深度相机（pick 判够近用）：Astra Pro，走 OpenNI2
    depth = None
    if not args.no_depth:
        try:
            depth = DepthCamera()
            depth.open()
            depth.start_depth()
            print('深度相机已打开', flush=True)
        except Exception as e:
            print('深度相机打开失败（%s），退回 bbox 面积判近' % e, flush=True)
            depth = None
    rotate = load_params()['vision'].get('camera_rotate', 0)

    video_stop = threading.Event()
    video_thread = threading.Thread(
        target=video_loop, args=([detector, model_det.detect], video_stop), daemon=True)
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

    if args.fusion == 'reference':
        fusion = ReferenceFusion(f_px=args.f_px, cx0=args.cx0)
    else:
        fusion = LaneFusion(f_px=args.f_px, cx0=args.cx0)
    ctrl = LaneController(head_gain=args.fusion_head_gain,
                          cross_gain=args.fusion_cross_gain,
                          head_deadzone=args.fusion_head_deadzone,
                          cross_deadzone=args.fusion_cross_deadzone)
    marker_range = args.marker_range
    print('融合导航已开启（%s）：相机写状态 / IMU 管执行 / 状态不归零 '
          '(f_px=%.0f cx0=%.0f)' % (args.fusion, args.f_px, args.cx0), flush=True)
    print('步幅缩放 stride_scale=%.3f' % STRIDE_SCALE, flush=True)

    restore_travel(board, GRIPPER_OPEN)
    cam_state = {'pulse': OFFICIAL_ARM[24]}   # 24 号腕俯仰跟踪状态（相机上下）
    print('自动捕获启动，颜色目标=%s' % args.color, flush=True)
    ik.stand(ik.initial_pos, t=500)
    time.sleep(0.5)

    target_yaw = 0.0
    pending_forward = 0
    pick_count = 0
    place_count = 0
    picked_count = 0
    pose = {'x': 0.0, 'y': 0.0}
    color_enabled = True   # 颜色识别开关：弯道里关掉，过了弯道再开

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
            # 长直行才算「真直线」，重新开启颜色识别；弯道里的小直行保持关闭
            if abs(pending_forward) >= STRAIGHT_MIN_MM:
                color_enabled = True
            # 融合导航：相机写状态、IMU 管执行，状态不归零；丢帧卡尔曼自己扛得住
            marker_range = move_straight_fusion(
                ik, board, detector, imu_state, imu_state['tracker'],
                pending_forward, fusion, ctrl, marker_range, args.marker_size_mm,
                cam_state, color_enabled)
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
            color_enabled = False   # 进入弯道，关闭颜色识别
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
            color_enabled = False   # 进入弯道，关闭颜色识别
            report(heading_deg=norm_heading(imu_state['yaw']),
                   message='右转 %d°' % angle)
        elif name == 'pick':
            pick_count += 1
            print('%d/%d pick%d' % (i, len(actions), pick_count), flush=True)
            log_battery(board, tag='夹取前')   # 只记录分析，不参与控制
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_pick(board, ik, model_det, depth, rotate, pick_count, pulses,
                    pull_up_pulse=args.pull_up, no_depth=args.no_depth,
                    manual=args.manual, s24_now=cam_state['pulse'],
                    stop_dist=args.stop_dist, reach_extra=args.reach_extra)
            picked_count += 1
            report(picked_count=picked_count,
                   message='第 %d 次夹取完成' % picked_count)
            # 夹取时手臂大幅摆动会带动机身，IMU 会积出假转角；重设基准丢掉这段，
            # 否则下一段直线的 e 会被灌进几十度假误差（现场 e 跳到 35° 就是这里）。
            imu_state['tracker'].since_last()
            if isinstance(fusion, ReferenceFusion):
                fusion.reset()   # 方块被抓走，参照失效，等下一个方块重新初始化
            cam_state['pulse'] = PICK1_RESTORE_24 if pick_count == 1 else OFFICIAL_ARM[24]
        elif name == 'place':
            place_count += 1
            print('%d/%d place%d' % (i, len(actions), place_count), flush=True)
            pulses = {int(k): int(v) for k, v in act.get('pulses', {}).items()} if act.get('pulses') else None
            do_place(board, place_count, pulses, manual=args.manual)
            report(message='第 %d 次放下完成' % place_count)
            imu_state['tracker'].since_last()
            if isinstance(fusion, ReferenceFusion):
                fusion.reset()   # 方块已放下，参照失效，等下一个方块重新初始化
            cam_state['pulse'] = OFFICIAL_ARM[24]
        elif name == 'stand':
            ik.stand(ik.initial_pos, t=500)

    if pending_forward:
        target_yaw = imu_state['yaw']
        dist_mm = pending_forward
        if abs(pending_forward) >= STRAIGHT_MIN_MM:
            color_enabled = True
        move_straight_fusion(
            ik, board, detector, imu_state, imu_state['tracker'],
            pending_forward, fusion, ctrl, marker_range, args.marker_size_mm,
            cam_state, color_enabled)
        advance_pose(pose, imu_state['yaw'], dist_mm)

    video_stop.set()
    cam.camera_close()
    if depth is not None:
        try:
            depth.close()
        except Exception:
            pass
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
