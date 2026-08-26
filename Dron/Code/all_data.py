#!/usr/bin/python3
# coding=utf8
"""总参数读取器：一条命令把所有"小纸条"读全。

这是闯关第 4 关的总和代码，把无人机能给的参数一次打印：
模式/解锁、姿态、位置、GPS、电池、飞行速度、局部位置。

用法:
    python3 all_data.py --tlog test_data/flight_sample.tlog   # 离线（推荐先跑）
    python3 all_data.py                                       # 在线（模拟器/真机）

只读安全：只收数据，不发任何指令。
"""
import argparse
import math
import time

from pymavlink import mavutil

from drone_config import CONNECTION


def deg(rad):
    """把弧度换算成度（就像把分换算成元）。"""
    return math.degrees(rad)


def snapshot(master):
    """把公告栏（master.messages）里最新的纸条拼成一行可读文字。"""
    m = master.messages
    parts = []

    hb = m.get('HEARTBEAT')   # 报平安：模式 + 是否解锁
    if hb is not None:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        parts.append('状态=模式:%s 解锁:%s' % (master.flightmode, armed))

    att = m.get('ATTITUDE')   # 姿态：左右/前后/转向的角度
    if att is not None:
        parts.append('姿态=roll %.1f° pitch %.1f° yaw %.1f°' % (
            deg(att.roll), deg(att.pitch), deg(att.yaw)))

    gpi = m.get('GLOBAL_POSITION_INT')   # 位置：经纬度 + 相对高度
    if gpi is not None:
        parts.append('位置=lat %.7f lon %.7f 高度 %.2fm' % (
            gpi.lat / 1e7, gpi.lon / 1e7, gpi.relative_alt / 1000.0))

    gps = m.get('GPS_RAW_INT')   # GPS：定位质量、卫星数、精度
    if gps is not None:
        parts.append('GPS=fix %s 星数 %s 精度 %scm' % (
            gps.fix_type, gps.satellites_visible, gps.eph))

    bat = m.get('BATTERY_STATUS') or m.get('SYS_STATUS')   # 电池
    if bat is not None:
        parts.append('电池=%.2fV 剩余 %s%%' % (
            bat.voltage_battery / 1000.0, bat.battery_remaining))

    vfr = m.get('VFR_HUD')   # 飞行仪表：航向、地速、爬升、油门
    if vfr is not None:
        parts.append('飞行=航向 %.0f° 地速 %.1fm/s 爬升 %.1fm/s 油门 %s%%' % (
            vfr.heading, vfr.groundspeed, vfr.climb, vfr.throttle))

    lpn = m.get('LOCAL_POSITION_NED')   # 局部位置：NED → ENU
    if lpn is not None:
        parts.append('局部位置=东 %.2fm 北 %.2fm 上 %.2fm' % (lpn.y, lpn.x, -lpn.z))

    return ' | '.join(parts) if parts else '(还没有收到纸条)'


def main():
    parser = argparse.ArgumentParser(description='总参数读取器')
    parser.add_argument('--tlog', default=None, help='离线读 test_data 日志')
    args = parser.parse_args()

    if args.tlog:
        master = mavutil.mavlink_connection(args.tlog)
        print('离线读取: %s' % args.tlog)
    else:
        master = mavutil.mavlink_connection(CONNECTION)
        print('等待飞控报平安...')
        master.wait_heartbeat(timeout=15)

    while True:
        # 离线读日志用非阻塞（读完自然结束）；在线保持阻塞等新纸条
        msg = master.recv_match(blocking=not args.tlog, timeout=2)
        if msg is None:
            if args.tlog:
                print('日志读完')
                break
            continue
        # 用"姿态"纸条当节拍：每收到一张姿态，就打印一次当前所有参数
        if msg.get_type() == 'ATTITUDE':
            print(snapshot(master))
            if not args.tlog:
                time.sleep(0.5)


if __name__ == '__main__':
    main()
