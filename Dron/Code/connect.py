#!/usr/bin/python3
# coding=utf8
"""MAVLink 连接封装与常用工具。

用法：
    python3 connect.py          # 只测试能否连上飞控

被其他脚本 import：
    from connect import connect, is_armed, set_px4_mode
"""
import sys
from pymavlink import mavutil
from drone_config import CONNECTION, TIMEOUT, TARGET_SYSTEM, TARGET_COMPONENT


def connect(conn=CONNECTION, timeout=TIMEOUT):
    """建立 MAVLink 连接并等待飞控心跳，返回 master 对象。

    master 是 pymavlink 的 mavfile 对象：
      - master.messages['ATTITUDE']  最近一帧姿态
      - master.recv_match(type=...)  阻塞接收指定消息
      - master.mav.xxx_send(...)     发送指令
    """
    print('[连接] 目标: %s' % conn)
    master = mavutil.mavlink_connection(conn)
    print('[连接] 等待飞控心跳（超时 %s 秒）...' % timeout)
    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise SystemExit('连接失败：没有收到心跳。检查连接方式/端口/波特率。')
    type_name = mavutil.mavlink.enums['MAV_TYPE'][hb.type].name
    autopilot = mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name
    print('[连接] 成功：type=%s autopilot=%s flightmode=%s' % (
        type_name, autopilot, master.flightmode))
    return master


def wait_message(master, msg_type, timeout=5):
    """等待指定类型的一条消息，超时返回 None。"""
    return master.recv_match(type=msg_type, blocking=True, timeout=timeout)


def is_armed(master):
    """是否已解锁：HEARTBEAT 的 base_mode 里 SAFETY_ARMED 位。"""
    hb = master.messages.get('HEARTBEAT')
    if hb is None:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def px4_custom_mode(main_mode, sub_mode=0):
    """把 PX4 主/子模式编码成 custom_mode：
    主模式放 16-23 位，子模式放 24-31 位。"""
    return (main_mode << 16) | (sub_mode << 24)


def set_px4_mode(master, main_mode, sub_mode=0):
    """切换 PX4 飞行模式（如 AUTO + MISSION、OFFBOARD）。"""
    custom_mode = px4_custom_mode(main_mode, sub_mode)
    master.mav.set_mode_send(
        TARGET_SYSTEM,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode)
    print('[模式] 已发送: main=%s sub=%s' % (main_mode, sub_mode))


if __name__ == '__main__':
    m = connect()
    print('测试完成。飞行模式: %s，解锁状态: %s' % (m.flightmode, is_armed(m)))
