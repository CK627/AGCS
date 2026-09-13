#!/usr/bin/python3
# coding=utf8
"""相机封装：打开相机、取帧。"""
import time


def open_camera():
    """打开相机并返回 Camera 对象。"""
    from calibration.camera import Camera
    import cv2
    cam = Camera()
    cam.camera_open()
    # 官方 Camera 默认 SATURATION=40，Astra Pro 会被洗淡颜色（红块检测不到），恢复 128
    if cam.cap is not None:
        cam.cap.set(cv2.CAP_PROP_SATURATION, 128)
    return cam


def capture(cam, tries=20):
    """取一帧，失败返回 None。"""
    for _ in range(tries):
        f = cam.frame
        if f is not None:
            return f
        time.sleep(0.1)
    return None
