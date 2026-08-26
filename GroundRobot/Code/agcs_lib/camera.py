#!/usr/bin/python3
# coding=utf8
"""相机封装：打开相机、取帧。"""
import time


def open_camera():
    """打开相机并返回 Camera 对象。"""
    from calibration.camera import Camera
    cam = Camera()
    cam.camera_open()
    return cam


def capture(cam, tries=20):
    """取一帧，失败返回 None。"""
    for _ in range(tries):
        f = cam.frame
        if f is not None:
            return f
        time.sleep(0.1)
    return None
