#!/usr/bin/python3
# coding=utf8
"""官方 color_track.py 二开 + VL53L0X 红外测距夹取。

用法：
    python3 CS-grab-servo.py --color blue --distance 150

流程：
1. 官方初始姿态、夹爪张开、初始化 VL53L0X 红外；
2. 官方 color_track.py 的检测（320x240 + 高斯5x5 + min_area50）+ PID 追踪调 21/24；
3. 目标居中 + 红外测距（机身到目标 mm）< 阈值 → 夹爪闭合夹取；
4. 夹取后抬机械臂、松夹爪、复位。
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
from common.pid import PID
from common import yaml_handle
from calibration.camera import Camera
from calibration.CalibrationConfig import calibration_param_path
from common.ros_robot_controller_sdk import Board
import arm_ik.arm_move_ik as AMK

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
}


def his_hqul_color(img):
    """官方 color_track 直方图均衡。"""
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    img_eq = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    return img_eq


def get_area_max_contour(contours):
    """官方 color_track 最大轮廓（面积 > 50）。"""
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


SIZE = (320, 240)


def detect_color_frame(img, lab_data, color):
    """官方 color_track 的检测：返回 (area, center) 或 (0, None)。"""
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
    area_max_contour, area_max = get_area_max_contour(contours)
    if area_max_contour is None:
        return 0, None

    (centerX, centerY), radius = cv2.minEnclosingCircle(area_max_contour)
    centerX = int(misc.map(centerX, 0, SIZE[0], 0, img_w))
    centerY = int(misc.map(centerY, 0, SIZE[1], 0, img_h))
    return area_max, (centerX, centerY)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='blue', choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--distance', type=int, default=150, help='夹取距离阈值 mm（机身到目标）')
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

    # 官方 color_track 的 PID
    x_pid = PID(P=0.1, I=0.001, D=0.008)
    y_pid = PID(P=0.1, I=0.02, D=0.008)
    x_dis = 500
    y_dis = 260

    # VL53L0X 红外初始化（机身前方朝前）
    i2c = busio.I2C(adafruit_board.SCL, adafruit_board.SDA)
    tof = adafruit_vl53l0x.VL53L0X(i2c)

    # 初始姿态（朝前下方看平地/垫高目标）+ 夹爪张开
    ak.setPitchRangeMoving((0, 15, 15), -45, -90, 100, 1)
    time.sleep(1)
    board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
    time.sleep(0.5)
    board.bus_servo_set_position(0.5, [[25, 120]])
    time.sleep(0.5)

    camera = Camera()
    camera.camera_open()
    print('=== 开始：追踪目标，红外距离 < %dmm 就夹取，ESC 退出 ===' % args.distance, flush=True)

    try:
        while True:
            img = camera.frame
            if img is None:
                time.sleep(0.01)
                continue

            frame = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
            area, center = detect_color_frame(frame, lab_data, args.color)

            # 读红外距离（机身到目标 mm）
            try:
                distance_mm = tof.range
            except Exception:
                distance_mm = -1

            if center is not None:
                cx, cy = center
                # 官方 PID 追踪
                x_pid.SetPoint = frame.shape[1] / 2
                x_pid.update(cx)
                x_dis += int(x_pid.output)
                x_dis = 0 if x_dis < 0 else x_dis
                x_dis = 1000 if x_dis > 1000 else x_dis

                y_pid.SetPoint = frame.shape[0] / 2
                y_pid.update(cy)
                y_dis += int(y_pid.output)
                y_dis = 0 if y_dis < 0 else y_dis
                y_dis = 1000 if y_dis > 1000 else y_dis

                board.bus_servo_set_position(0.02, [[24, y_dis], [21, x_dis]])
                time.sleep(0.02)

                print('追踪 cx=%d cy=%d 21=%d 24=%d 距离=%dmm' % (cx, cy, x_dis, y_dis, distance_mm), flush=True)

                # 红外距离阈值夹取（且目标居中）
                if 0 < distance_mm < args.distance and abs(cx - frame.shape[1] / 2) < 40 and abs(cy - frame.shape[0] / 2) < 60:
                    print('>>> 距离到阈值，夹取！', flush=True)
                    board.bus_servo_set_position(0.5, [[25, 550]])  # 夹爪闭合
                    time.sleep(1)
                    ak.setPitchRangeMoving((12, 24, 5), -90, -90, 100, 1.5)  # 抬机械臂
                    time.sleep(1.5)
                    board.bus_servo_set_position(0.5, [[25, 120]])  # 松夹爪
                    time.sleep(0.5)
                    ak.setPitchRangeMoving((0, 15, 15), -45, -90, 100, 1.5)  # 复位
                    time.sleep(1.5)
                    print('>>> 夹取完成', flush=True)
                    break

            key = cv2.waitKey(1)
            if key == 27:
                break
    finally:
        camera.camera_close()
        cv2.destroyAllWindows()
        print('=== 结束 ===', flush=True)


if __name__ == '__main__':
    main()
