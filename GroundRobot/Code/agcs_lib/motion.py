#!/usr/bin/python3
# coding=utf8
"""六足步态封装：封装官方 kinematics.IK。"""


def make_ik(board):
    from common import kinematics
    return kinematics.IK(board)


def stand(ik, t=500):
    """立正，回到初始站立姿态（体态高度复位）。"""
    ik.stand(ik.initial_pos, t=t)


def move_body(ik, dz, speed=300):
    """升降体态（仅夹取阶段使用），dz 为高度偏移 mm，正=升高、负=降低。

    相对初始站姿的绝对偏移，dz=0 即复位。返回实际偏移。
    """
    dz = int(dz)
    ik.moveBody(ik.initial_pos, [0, 0, dz], [0, 0, 0], speed)
    return dz


def move_body_xyz(ik, dx=0, dy=0, dz=0, speed=100):
    """六足 IK 精确平移（相对初始站姿的绝对偏移，单位 mm）。

    坐标：dx 负=前进、dx 正=后退；dy 正=右移；dz 正=升高。
    比步态走（go_forward）快且精确，适合夹取前的 mm 级微调。
    """
    ik.moveBody(ik.initial_pos, [int(dx), int(dy), int(dz)], [0, 0, 0], speed)


def turn_left(ik, angle=10, speed=60):
    ik.turn_left(ik.initial_pos, 2, angle, speed, 1)


def turn_right(ik, angle=10, speed=60):
    ik.turn_right(ik.initial_pos, 2, angle, speed, 1)


def go_forward(ik, step=15, speed=50, times=1):
    ik.go_forward(ik.initial_pos, 2, step, speed, times)


def go_back(ik, step=15, speed=50):
    ik.back(ik.initial_pos, 2, step, speed, 1)
