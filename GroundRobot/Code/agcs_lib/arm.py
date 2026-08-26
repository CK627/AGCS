#!/usr/bin/python3
# coding=utf8
"""机械臂封装：make_arm_ik（封装官方 arm_ik + 方向校正）。

夹取动作直接用官方 block_fetch.py 的 arm_ik + 夹爪，不在此重写。
"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.params import load_params


def make_arm_ik(params=None):
    """创建机械臂 IK 对象，并应用本机舵机方向校正（arm.flip_servos）。"""
    import arm_ik.arm_move_ik as AMK
    p = (params or load_params())['arm']
    flip = p.get('flip_servos', [])

    class _CalibratedArmIK(AMK.ArmIK):
        def servosMove(self, servos, movetime=None):
            # servos 顺序为 (24, 23, 22, 21)
            order = {24: 0, 23: 1, 22: 2, 21: 3}
            s = list(servos)
            for sid in flip:
                s[order[sid]] = 1000 - s[order[sid]]
            return AMK.ArmIK.servosMove(self, tuple(s), movetime)

    return _CalibratedArmIK()
