#!/usr/bin/python3
# coding=utf8
"""画面占比判断前伸 + 夹取 定点测试（目标放固定位置，不跑寻路）。

原理：机械臂水平前伸（alpha=0），摄像头朝前下方看。摄像头在夹爪前方，目标
在地面出现在画面下部。前伸靠近目标时，目标在画面下部越来越大（摄像头能朝下
看，目标不会消失）；目标占画面比例达标 = 夹爪已到目标上方 = 可以夹。

用法：
    python3 CS-grab-tof.py --color blue --ratio 0.3
        --ratio 目标占画面(320x240)比例阈值（0~1），占比 >= 此值停止前伸夹取

流程：
1. 复位 + 夹爪张开，前伸到扫描起始位置（alpha=0 末端朝前）；
2. 动态前伸：占比离阈值越远步长越大、越近越小；占比达标 → 停止前伸 → 闭合夹爪；
3. 前伸到 IK 极限仍不够 → 直接夹取；
4. 夹住后原地保持夹紧（不抬起到 RAISE_POSE），敲回车才松开并复位；
5. 无论成功失败 / Ctrl+C，finally 里复位到官方初始状态。
"""
import argparse
import math
import time

import cv2
import numpy as np

import board as adafruit_board
import busio
import adafruit_vl53l0x

from common import misc
from common import yaml_handle
from calibration.camera import Camera
from calibration.CalibrationConfig import calibration_param_path
from common.ros_robot_controller_sdk import Board
import arm_ik.arm_move_ik as AMK


# ---- 前伸 + 下降参数（现场标定）----
REACH_Y_START = 28.0     # 前伸起始 y cm（alpha=0 有解下限）
REACH_Y_MAX = 45.0       # 前伸最大 y cm（要能越过目标）
REACH_GAIN = 20.0        # 前伸增益 cm/占比：步长 = (ratio - 当前占比) * gain
REACH_STEP_MIN = 0.5     # 最小前伸步长 cm（占比接近阈值时的小步）
REACH_STEP_MAX = 3.0     # 最大前伸步长 cm（看不到/目标远时的大步）
COARSE_Z = 15.0          # 前伸/下降末端高度 cm
FAST_SWITCH_MM = 200     # 红外 > 此值快速下降，<= 此值切慢速点动
FAST_STEP_CM = 1.0       # 快速下降步长 cm
SLOW_STEP_CM = 0.2       # 慢速下降步长 cm
MOVE_MS = 400            # 每次 IK 移动时间 ms（慢）
SETTLE_MS = 300          # 每次移动后稳定等待 ms
TOF_SAMPLES = 5          # 红外中位数采样次数
LOST_FRAMES_TO_GRAB = 2  # 连续丢帧数（目标消失判定）

SIZE = (320, 240)
RESET_PULSES = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_OPEN = 120
GRIPPER_CLOSE = 550
RAISE_POSE = (12, 24, 5)


def his_hqul_color(img):
    """官方 color_track 直方图均衡。"""
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    return cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)


def get_area_max_contour(contours):
    contour_area_max = 0
    area_max_contour = None
    max_area = 0
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:
                area_max_contour = c
                max_area = contour_area_temp
    return area_max_contour, max_area


def detect_area(img, lab_data, color):
    """检测目标，返回面积（无目标返回 0）。"""
    img_h, img_w = img.shape[:2]
    img_his = his_hqul_color(img)
    frame_resize = cv2.resize(img_his, SIZE, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    frame_mask = cv2.inRange(
        frame_lab,
        (lab_data[color]['min'][0], lab_data[color]['min'][1], lab_data[color]['min'][2]),
        (lab_data[color]['max'][0], lab_data[color]['max'][1], lab_data[color]['max'][2]))
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    _, area = get_area_max_contour(contours)
    return int(area)


def tof_mm(tof, samples=TOF_SAMPLES):
    """红外中位数测距 mm。"""
    vals = []
    for _ in range(samples):
        try:
            d = tof.range
            if 0 < d < 8000:
                vals.append(d)
        except Exception:
            pass
        time.sleep(0.01)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def move(ak, x, y, z, move_ms=MOVE_MS, settle_ms=SETTLE_MS):
    """末端移到 (x,y,z)，alpha=0（夹持器水平，红外朝下）。返回 servos dict 或 False。"""
    res = ak.setPitchRangeMoving((x, y, z), 0, -90, 100, move_ms / 1000.0)
    time.sleep(settle_ms / 1000.0)
    if res is False:
        return False
    if abs(res[1]) > 0.5:
        return False
    return res[0]  # {"servo21":.., "servo22":.., "servo23":.., "servo24":..}


def reset_arm(board):
    board.bus_servo_set_position(1.0, [[sid, RESET_PULSES[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(0.5, [[25, GRIPPER_OPEN]])


def capture_frame(camera, mapx, mapy):
    """取一帧并畸变校正，失败返回 None。"""
    img = camera.frame
    if img is None:
        return None
    return cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='blue', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--ratio', type=float, default=0.3, help='目标占画面(320x240)比例阈值（0~1），占比 >= 此值停止前伸夹取')
    args = parser.parse_args()

    board = Board()
    ak = AMK.ArmIK()
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    # 相机畸变校正
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    i2c = busio.I2C(adafruit_board.SCL, adafruit_board.SDA)
    tof = adafruit_vl53l0x.VL53L0X(i2c)
    print('红外初始化成功', flush=True)

    camera = Camera()
    camera.camera_open()

    reset_arm(board)
    time.sleep(2)
    last_servos = move(ak, 0, REACH_Y_START, COARSE_Z, move_ms=800, settle_ms=500)

    try:
        print('=== 前伸扫描（目标占比 >= %.0f%% 即夹）===' % (args.ratio * 100), flush=True)
        grabbed = False
        y = REACH_Y_START
        while True:
            frame = capture_frame(camera, mapx, mapy)
            area = detect_area(frame, lab_data, args.color) if frame is not None else 0
            ratio = area / (SIZE[0] * SIZE[1])
            if area > 0 and ratio >= args.ratio:
                # 目标占画面比例达标 = 夹爪已到目标上方，直接夹取
                print('>>> 目标占比 %.1f%% >= %.1f%% → 停止前伸夹取 y=%.1fcm' % (ratio * 100, args.ratio * 100, y), flush=True)
                board.bus_servo_set_position(0.5, [[25, GRIPPER_CLOSE]])
                time.sleep(1.0)
                if last_servos:
                    board.bus_servo_set_position(0.5, [[22, last_servos["servo22"] + 50]])
                    time.sleep(0.5)
                print('>>> 夹取执行完成', flush=True)
                grabbed = True
                break
            # 根据画面占比动态前伸：占比离阈值越远步长越大、越近越小
            gap = max(0.0, args.ratio - ratio)
            step = max(REACH_STEP_MIN, min(REACH_STEP_MAX, gap * REACH_GAIN))
            print('前伸扫描 y=%.1fcm 面积=%d 占比=%.1f%% → 前伸 %.1fcm' % (y, area, ratio * 100, step), flush=True)
            y += step
            res = move(ak, 0, y, COARSE_Z)
            if res is False:
                # 前伸到极限，已经最靠近目标，直接夹取
                print('>>> 前伸到极限 y=%.1fcm，直接夹取' % y, flush=True)
                board.bus_servo_set_position(0.5, [[25, GRIPPER_CLOSE]])
                time.sleep(1.0)
                if last_servos:
                    board.bus_servo_set_position(0.5, [[22, last_servos["servo22"] + 50]])
                    time.sleep(0.5)
                print('>>> 夹取执行完成', flush=True)
                grabbed = True
                break
            last_servos = res

        if not grabbed:
            print('前伸扫描未定位到目标', flush=True)
        else:
            # 夹住保持，人工确认夹稳后再松开（测试阶段：回车前一直夹紧）
            input('>>> 已夹住，保持夹紧。敲回车松开夹爪并复位...')
    finally:
        reset_arm(board)
        camera.camera_close()
        print('=== 已复位到官方初始状态 ===', flush=True)


if __name__ == '__main__':
    main()
