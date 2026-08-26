#!/usr/bin/python3
# coding=utf8
"""机械臂抓取封装（机械臂抓取模块）。

基于官方 arm_ik.arm_move_ik.ArmIK 逆运动学 + 25 号舵机夹爪。
坐标单位 cm，俯仰角 deg，movetime 单位秒（与官方 SDK 一致）。
"""
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from functions.robot_config import load_params


class ArmPicker:
    """机械臂抓取控制：移动到目标 → 夹取 → 抬起 → 松开。"""

    def __init__(self, board, ak, params=None):
        self.board = board
        self.ak = ak
        self.params = params if params is not None else load_params()
        # 取 YAML 里 arm 这一整组参数（机械臂相关），
        # 后面 self.p['gripper_open'] 就是"取 arm 组里的 gripper_open 键"
        self.p = self.params['arm']

    def _gripper_pulse(self, pulse):
        # 本机夹爪25方向反，发指令前绕中位镜像（arm.flip_gripper）
        if self.p.get('flip_gripper', False):
            return 1000 - pulse
        return pulse

    def open_gripper(self, movetime=0.5):
        # arm.gripper_open：夹爪张开时 25 号舵机的脉宽
        self.board.bus_servo_set_position(
            movetime, [[25, self._gripper_pulse(self.p['gripper_open'])]])

    def close_gripper(self, movetime=0.5):
        # arm.gripper_close：夹爪闭合时 25 号舵机的脉宽
        self.board.bus_servo_set_position(
            movetime, [[25, self._gripper_pulse(self.p['gripper_close'])]])

    def move_to(self, x, y, z, movetime=1.0):
        """逆运动学移动到 (x, y, z) cm；无解返回 False。"""
        # arm.pitch / arm.alpha1 / arm.alpha2：俯仰角及其搜索范围
        return self.ak.setPitchRangeMoving(
            (x, y, z), self.p['pitch'], self.p['alpha1'], self.p['alpha2'], movetime)

    def reset_pose(self):
        """复位：按现场标定的脉宽把 5 个舵机摆到检测位。"""
        pulses = self.p['reset_pulses']
        self.board.bus_servo_set_position(
            1.5, [[sid, pulses[sid]] for sid in [21, 22, 23, 24, 25]])
        return True

    def pick_at(self, x, y):
        """移动到目标 (x, y) 上方 → 下降 → 夹取 → 抬起。

        返回 True 表示完成抓取动作，False 表示逆运动学无解。
        """
        if self.move_to(x, y, self.p['pick_z']) is False:   # arm.pick_z 抓取高度
            print('pick_at: 逆运动学无解 x=%.1f y=%.1f' % (x, y))
            return False
        time.sleep(0.5)
        self.close_gripper(0.5)
        time.sleep(0.5)
        raise_pose = self.p['raise_pose']   # arm.raise_pose 抓取后抬起位置
        self.move_to(raise_pose[0], raise_pose[1], raise_pose[2], 1.5)
        return True

    def release(self, place=True):
        """移动到放置位置并松开夹爪。"""
        if place:
            pose = self.p['release_pose']   # arm.release_pose 松开/放置位置
            self.move_to(pose[0], pose[1], pose[2], 1)
            time.sleep(0.3)
        self.open_gripper(0.5)
        time.sleep(0.3)
