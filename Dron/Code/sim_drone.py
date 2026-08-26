#!/usr/bin/python3
# coding=utf8
"""假无人机模拟器：真机到货前，用它练习 Python ↔ 无人机互通。

原理：用 pymavlink 以 udpout 方式向地面站监听端口 14550 发送 MAVLink 数据，
模拟一架正在定点悬停的 PX4 四旋翼（姿态缓慢摆动、位置缓慢移动、电量缓慢下降）。

用法（两个终端）：
    终端1: python3 sim_drone.py
    终端2: python3 read_drone.py     # 或 view_data.py / backend_exercise.py
"""
import argparse
import math
import time

from pymavlink import mavutil

# 模拟一架 PX4 四旋翼：AUTO.LOITER（定点模式）
# base_mode 带上 AUTO/GUIDED/STABILIZE/CUSTOM_MODE 标志，
# 地面站才能把 custom_mode 解析成 "LOITER"（对应真机 PX4 的心跳行为）
BASE_MODE = (mavutil.auto_mode_flags |
             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
CUSTOM_MODE = ((mavutil.PX4_CUSTOM_MAIN_MODE_AUTO << 16) |
               (mavutil.PX4_CUSTOM_SUB_MODE_AUTO_LOITER << 24))


def main(host='127.0.0.1', port=14550):
    conn = mavutil.mavlink_connection(
        'udpout:%s:%d' % (host, port), source_system=1, source_component=1)
    t0 = time.time()
    print('假无人机已启动 → %s:%d（Ctrl+C 停止）' % (host, port))

    while True:
        t = time.time() - t0

        # 心跳：模拟飞控持续广播"我还活着 + 当前模式/解锁状态"
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
            BASE_MODE, CUSTOM_MODE,
            mavutil.mavlink.MAV_STATE_ACTIVE)

        # 姿态：悬停微动（roll/pitch/yaw 弧度）
        roll = 0.05 * math.sin(0.4 * t)
        pitch = 0.04 * math.cos(0.3 * t)
        yaw = 0.8 + 0.1 * math.sin(0.2 * t)
        conn.mav.attitude_send(int(t * 1000), roll, pitch, yaw, 0, 0, 0)

        # 位置：缓慢漂移（经纬度 1e7 缩放，高度 mm）
        lat = 31.2304 + 1e-5 * math.sin(0.05 * t)
        lon = 121.4737 + 1e-5 * math.cos(0.05 * t)
        conn.mav.global_position_int_send(
            int(t * 1000), int(lat * 1e7), int(lon * 1e7),
            int(10 * 1000), int(5 * 1000), 0, 0, 0, int(yaw * 100))
        conn.mav.local_position_ned_send(
            int(t * 1000), math.sin(0.05 * t), math.cos(0.05 * t), -5.0, 0, 0, 0)

        # GPS：fix=3（已定位），15 颗星
        conn.mav.gps_raw_int_send(
            int(t * 1e6), 3, int(lat * 1e7), int(lon * 1e7),
            int(10 * 1000), 100, 100, 0, 0, 15)

        # 电量：从 16.0V / 100% 缓慢下降（SYS_STATUS 的电压单位 mV）
        volt = max(14.0, 16.0 - 0.0002 * t)
        pct = max(0, int(100 - t / 60))
        conn.mav.sys_status_send(
            0, 0, 0, 0, int(volt * 1000), -1, pct, 0, 0, 0, 0, 0, 0)

        # 航向/地速/高度（对应 HUD 仪表）
        conn.mav.vfr_hud_send(1.2, 1.2, int(yaw * 57.2958), 35, 5.0, 0.1)

        time.sleep(0.25)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='假无人机（MAVLink 数据模拟器）')
    parser.add_argument('--host', default='127.0.0.1', help='发送目标地址')
    parser.add_argument('--port', type=int, default=14550, help='发送目标端口')
    args = parser.parse_args()
    main(args.host, args.port)
