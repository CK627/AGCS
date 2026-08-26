#!/usr/bin/python3
# coding=utf8
"""传感器封装：超声波测距、点阵显示。"""
import time


def make_ultrasonic():
    """创建超声波对象（I2C 0x77），失败返回 None。"""
    try:
        from sensor.ultrasonic_sensor import Ultrasonic
        return Ultrasonic()
    except Exception:
        return None


def dist_cm(ultrasonic, samples=3):
    """读取超声波距离（cm）：多次采样取最近有效值，忽略超量程，失败返回 -1.0。"""
    if ultrasonic is None:
        return -1.0
    vals = []
    for _ in range(samples):
        try:
            d = ultrasonic.getDistance() / 10.0
            if 0 < d < 500.0:
                vals.append(d)
        except Exception:
            pass
        time.sleep(0.02)
    if not vals:
        return -1.0
    return min(vals)


def make_display(clk=8, dio=7):
    """创建点阵 TM1640，失败返回 None。"""
    try:
        from sensor.dot_matrix_sensor import TM1640
        display = TM1640(clk=clk, dio=dio)
        display.clear()
        return display
    except Exception:
        return None


def show_status(display, v):
    """在点阵上显示状态数字，失败静默。"""
    if display is None:
        return
    try:
        display.write_int(v, pos=0, len=8)
    except Exception:
        pass
