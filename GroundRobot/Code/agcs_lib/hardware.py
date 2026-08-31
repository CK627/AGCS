#!/usr/bin/python3
# coding=utf8
"""硬件封装：控制板 Board。"""


def make_board():
    """创建控制板 Board 对象，并开启串口接收（官方用法）。

    不调用 enable_reception() 时 enable_recv=False，
    bus_servo_read_* 全部返回 None（读不到舵机实际位置）。
    """
    from common.ros_robot_controller_sdk import Board
    board = Board()
    board.enable_reception()
    return board
