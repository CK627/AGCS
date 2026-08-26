#!/usr/bin/python3
# coding=utf8
"""颜色识别（视觉识别第一步）。

可独立运行（树莓派 VNC 终端）：
    python3 ~/spiderpi/functions/color_detect.py --color red

同时提供官方 RPC 风格接口 init/start/stop/exit/run，
可被 SpiderPi.py 直接加载（本文件同步后会替换官方同名文件，
官方原版保留在资料包 3 源码资料/SpiderPi_Pro.zip 中）。
"""
import os
import sys
import time
import argparse

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2
from calibration.camera import Camera
from functions.robot_config import load_params
from functions.vision_utils import load_lab_data, detect_color, load_undistort_maps, correct_camera

__isRunning = False
_lab_data = None
_target_color = 'red'
_min_area = 500
_camera_rotate = 0


def init():
    global _lab_data, _target_color, _min_area, _camera_rotate
    params = load_params()
    # params 是"嵌套字典"：params['vision'] 先取 vision 这一组，
    # 再取 ['target_color'] 拿到组里的值 → 对应 YAML 的 vision.target_color
    _target_color = params['vision']['target_color']   # 目标颜色（YAML: vision.target_color）
    _min_area = params['vision']['min_area']           # 有效目标最小面积（YAML: vision.min_area）
    _camera_rotate = params['vision'].get('camera_rotate', 0)
    _lab_data = load_lab_data()
    print('ColorDetect Init')


def start():
    global __isRunning
    __isRunning = True
    print('ColorDetect Start')


def stop():
    global __isRunning
    __isRunning = False
    print('ColorDetect Stop')


def exit():
    stop()
    print('ColorDetect Exit')


def run(img):
    """RPC 主循环回调：检测并绘制，返回处理后的 img。"""
    if not __isRunning or _lab_data is None:
        return img
    detect_color(img, _lab_data, _target_color, min_area=_min_area)
    return img


def main():
    parser = argparse.ArgumentParser(description='颜色识别')
    parser.add_argument('--color', default=None, help='目标颜色 red/green/blue')
    args = parser.parse_args()

    init()
    global _target_color
    if args.color:
        _target_color = args.color

    mapx, mapy = load_undistort_maps()
    camera = Camera()
    camera.camera_open()
    try:
        while True:
            img = camera.frame
            if img is None:
                time.sleep(0.01)
                continue
            img = correct_camera(img, _camera_rotate)
            frame = cv2.remap(img.copy(), mapx, mapy, cv2.INTER_LINEAR)
            run(frame)
            cv2.imshow('ColorDetect', frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC 退出
                break
    finally:
        camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
