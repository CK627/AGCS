#!/usr/bin/python3
# coding=utf8
import sys
import time

from common.ros_robot_controller_sdk import Board
from sensor.ultrasonic_sensor import Ultrasonic

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

board = Board()
ultrasonic = Ultrasonic()


def _turn_off_front_lights():
    """开机时把超声波模块前面的两颗 RGB 灯关掉。

    超声波模块刚上电时 I2C 可能还没就绪，直接写会抛
    OSError: [Errno 121] Remote I/O error。这里做几次重试，
    失败也不让 SpiderPi 主流程崩溃。
    """
    for attempt in range(5):
        try:
            ultrasonic.setRGBMode(0)
            ultrasonic.setRGB(1, (0, 0, 0))
            ultrasonic.setRGB(2, (0, 0, 0))
            return
        except Exception as e:
            print('turn off front lights failed, retry %d/5: %s'
                  % (attempt + 1, e))
            time.sleep(1.0)
    print('turn off front lights still failed after 5 attempts')


# 初始位置(initial position)
def initMove():
    _turn_off_front_lights()
    #board.bus_servo_set_position(0.5, [[1, 1500], [2, 1500]])


def reset():
    return None


def init():
    initMove()
    print("RemoteControl Init")
    return None


def start():
    print("RemoteControl Start")
    return None


def stop():
    print("RemoteControl Stop")
    return None


def exit():
    print("RemoteControl Exit")
    return None


def run(img):
    return img
