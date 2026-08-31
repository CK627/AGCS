#!/usr/bin/python3
# coding=utf8
"""Restore：恢复初始状态 + 摄像头启动/关闭（独立类，供各模块调用）。"""
import time

import cv2

from agcs_lib.camera import open_camera, capture
from agcs_lib.logs import get_logger
from agcs_lib.motion import stand
from agcs_lib.vision import load_undistort_maps, correct_camera


class Restore:
    def __init__(self, board, ik, ak, params):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.log = get_logger('restore')
        self._cam = None
        self._mapx = self._mapy = None
        self._rotate = 0

    def restore_initial_state(self):
        """立正 + 机械臂复位(reset_pulses) + 云台回中(21=500/24=260) + 夹爪张开。"""
        arm = self.params['arm']
        stand(self.ik, t=500)
        arm_pulses = arm['reset_pulses']
        self.board.bus_servo_set_position(1.5, [[sid, arm_pulses[sid]] for sid in [21, 22, 23, 24]])
        self.board.bus_servo_set_position(1.0, [[25, int(arm.get('gripper_open', 120))]])
        time.sleep(1.5)
        self.board.bus_servo_set_position(0.5, [[24, 260], [21, 500]])
        time.sleep(0.5)
        self.log.info('[restore] 恢复初始状态完成：立正+机械臂复位+云台21=500/24=260+夹爪张开')

    def start_camera(self):
        """打开摄像头并取帧自检，返回 (cam, mapx, mapy, rotate)；失败抛 RuntimeError。"""
        self._rotate = int(self.params['vision'].get('camera_rotate', 0))
        self._mapx, self._mapy = load_undistort_maps()
        self._cam = open_camera()
        f = capture(self._cam, tries=10)
        if f is None:
            self._cam.camera_close()
            self._cam = None
            raise RuntimeError('摄像头取不到帧（可能被 spiderpi/CS-video 等进程占用）')
        frame = cv2.remap(correct_camera(f, self._rotate), self._mapx, self._mapy, cv2.INTER_LINEAR)
        self.log.info('[camera] 摄像头启动完成，shape=%s', frame.shape)
        return self._cam, self._mapx, self._mapy, self._rotate

    def close_camera(self):
        """关闭摄像头（未打开时静默）。"""
        if self._cam is not None:
            self._cam.camera_close()
            self._cam = None
            self.log.info('[camera] 摄像头已关闭')
