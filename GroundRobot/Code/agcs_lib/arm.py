#!/usr/bin/python3
# coding=utf8
"""机械臂封装：make_arm_ik（封装官方 arm_ik + 方向校正）。

官方 arm_ik.arm_move_ik 模块顶层会执行 `board = Board()`，再次打开舵机串口，
会打断当前进程已开启的串口读取线程（recv_task 报 multiple access on port）。
这里导入前先用空壳替换 Board，避免二次打开串口；真实 board 通过参数传入，
_CalibratedArmIK 的 servosMove 直接使用它。
"""
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib.params import load_params


def make_arm_ik(board=None, params=None):
    """创建机械臂 IK 对象，并应用本机舵机方向校正（arm.flip_servos）。

    board：控制板对象（传入已创建的 board，避免二次打开串口）；
           不传时内部创建一个。
    """
    import common.ros_robot_controller_sdk as sdk
    from agcs_lib.hardware import make_board
    if board is None:
        board = make_board()
    p = (params or load_params())['arm']
    flip = p.get('flip_servos', [])

    class _NoPortBoard:
        """空壳：仅用于替换官方模块导入时自动创建的 Board，不打开串口。"""

        def __init__(self, *args, **kwargs):
            pass

    orig_board = sdk.Board
    sdk.Board = _NoPortBoard
    try:
        import arm_ik.arm_move_ik as AMK
    finally:
        sdk.Board = orig_board

    class _CalibratedArmIK(AMK.ArmIK):
        def __init__(self, board, flip):
            self.board = board
            self.flip = flip
            AMK.ArmIK.__init__(self)

        def servosMove(self, servos, movetime=None):
            # servos 顺序为 (24, 23, 22, 21)
            order = {24: 0, 23: 1, 22: 2, 21: 3}
            s = list(servos)
            for sid in self.flip:
                s[order[sid]] = 1000 - s[order[sid]]
            time.sleep(0.02)
            if movetime is None:
                movetime = 500  # 官方用读 PWM 舵机算时长，这里直接用默认值
            self.board.bus_servo_set_position(
                movetime, [[24, s[0]], [23, s[1]], [22, s[2]], [21, s[3]]])
            return movetime

    return _CalibratedArmIK(board, flip)
