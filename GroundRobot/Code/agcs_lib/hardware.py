#!/usr/bin/python3
# coding=utf8
"""硬件封装：控制板 Board。"""


def make_board():
    """创建控制板 Board 对象。"""
    from common.ros_robot_controller_sdk import Board
    return Board()
