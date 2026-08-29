#!/usr/bin/python3
# coding=utf8
"""视觉封装：颜色检测、轮廓、坐标变换、朝向角、畸变校正。

封装官方 calibration / common 提供的相机标定、颜色阈值读取。
"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2
import math
import numpy as np
from common import misc
from common import yaml_handle

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


def load_lab_data():
    """读取 ~/spiderpi/config/lab_config.yaml 颜色阈值。"""
    return yaml_handle.get_yaml_data(yaml_handle.lab_file_path)


def load_block_params():
    """读取相机标定 block_params，返回 K, R, T。"""
    camera_cal = yaml_handle.get_yaml_data(yaml_handle.camera_file_path)['block_params']
    K = np.array(camera_cal['K'], dtype=np.float64).reshape(3, 3)
    R = np.array(camera_cal['R'], dtype=np.float64).reshape(3, 1)
    T = np.array(camera_cal['T'], dtype=np.float64).reshape(3, 1)
    return K, R, T


def get_area_max_contour(contours, min_area=100):
    """返回最大轮廓及其面积；面积不足 min_area 视为无效，返回 (None, 0)。"""
    contour_area_max = 0
    area_max_contour = None
    max_area = 0
    for c in contours:
        contour_area_temp = abs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= min_area:
                area_max_contour = c
                max_area = contour_area_temp
    return area_max_contour, max_area


def detect_color(img, lab_data, color='red', size=(320, 240), min_area=50):
    """在 img 中检测指定颜色目标（LAB 阈值），返回 dict(center, radius, area, color, contour)。"""
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    # 官方 color_track 直方图均衡（Y 通道），增强弱光/环境下的颜色识别
    _ycrcb = cv2.cvtColor(img_copy, cv2.COLOR_BGR2YCR_CB)
    _ch = cv2.split(_ycrcb)
    cv2.equalizeHist(_ch[0], _ch[0])
    cv2.merge(_ch, _ycrcb)
    img_copy = cv2.cvtColor(_ycrcb, cv2.COLOR_YCR_CB2BGR)
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    frame_mask = cv2.inRange(
        frame_lab,
        (lab_data[color]['min'][0], lab_data[color]['min'][1], lab_data[color]['min'][2]),
        (lab_data[color]['max'][0], lab_data[color]['max'][1], lab_data[color]['max'][2]))
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    area_max_contour, area_max = get_area_max_contour(contours, min_area)
    if area_max_contour is None:
        return None

    ((centerX, centerY), radius) = cv2.minEnclosingCircle(area_max_contour)
    centerX = int(misc.map(centerX, 0, size[0], 0, img_w))
    centerY = int(misc.map(centerY, 0, size[1], 0, img_h))
    radius = int(misc.map(radius, 0, size[0], 0, img_w))

    cv2.circle(img, (centerX, centerY), radius, range_rgb.get(color, (0, 255, 0)), 2)
    cv2.putText(img, "Color: %s" % color, (10, img.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb.get(color, (0, 255, 0)), 2)
    return {'center': (centerX, centerY), 'radius': radius,
            'area': area_max, 'color': color, 'contour': area_max_contour}


def camera_to_world(cam_mtx, r, t, img_points):
    """像素坐标转平面世界坐标（mm），算法与官方 block_fetch.py 一致。"""
    inv_k = np.asmatrix(cam_mtx).I
    r_mat = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(r, r_mat)
    inv_r = np.asmatrix(r_mat).I
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)
        worldPtPlane = np.dot(inv_r, worldPtCam)
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt


def pixel_to_arm_coord(K, R, T, center, initial_coord=(0, 15, 5)):
    """像素坐标 → 机械臂目标坐标 (x, y)，单位 cm。"""
    center = np.array(center, dtype=np.float64).reshape((1, 1, 2))
    w = camera_to_world(K, R, T, center)[0][0]
    wx = int(-w[0]) / 10.0
    wy = int(-w[1]) / 10.0
    return initial_coord[0] + wx, initial_coord[1] + wy


def load_undistort_maps(size=(640, 480)):
    """加载相机畸变校正映射，返回 (mapx, mapy)。"""
    from calibration.CalibrationConfig import calibration_param_path
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, size, 0, size)
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, size, 5)
    return mapx, mapy


def correct_camera(img, rotate=180):
    """校正摄像头画面方向。"""
    if rotate == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if rotate == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if rotate == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img
