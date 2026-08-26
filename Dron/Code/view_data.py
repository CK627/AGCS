#!/usr/bin/python3
# coding=utf8
"""在电脑上实时查看无人机数据（只读，安全）。

用法:
    python3 view_data.py                                     # 在线（模拟器/真机）
    python3 view_data.py --tlog test_data/flight_sample.tlog  # 离线：读测试日志

显示：飞行模式、解锁状态、姿态、经纬度、高度、GPS、电池、航向/地速。
"""
import argparse
import math
import time
from pymavlink import mavutil

from connect import connect, is_armed


def snapshot(master):
    """把 master.messages 里的最新数据拼成一行可读文本。"""
    msgs = master.messages
    hb = msgs.get('HEARTBEAT')
    att = msgs.get('ATTITUDE')
    gpi = msgs.get('GLOBAL_POSITION_INT')
    gps = msgs.get('GPS_RAW_INT')
    # 电量：真机/模拟器可能用 BATTERY_STATUS 或 SYS_STATUS，两者都兼容
    bat = msgs.get('BATTERY_STATUS') or msgs.get('SYS_STATUS')
    vfr = msgs.get('VFR_HUD')

    parts = []
    if hb is not None:
        parts.append('模式=%s' % master.flightmode)
        parts.append('解锁=%s' % ('是' if is_armed(master) else '否'))
    if att is not None:
        parts.append('姿态r/p/y=%.1f/%.1f/%.1f°' % (
            math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw)))
    if gpi is not None:
        parts.append('位置%.6f,%.6f 高度%.1fm' % (
            gpi.lat / 1e7, gpi.lon / 1e7, gpi.relative_alt / 1000.0))
    if gps is not None:
        parts.append('GPS fix=%s 星=%s eph=%scm' % (
            gps.fix_type, gps.satellites_visible, gps.eph))
    if bat is not None:
        pct = bat.battery_remaining
        parts.append('电池%.2fV %s%%' % (
            bat.voltage_battery / 1000.0, pct if pct >= 0 else '-'))
    if vfr is not None:
        parts.append('航向%.0f° 地速%.1fm/s 爬升%.1fm/s' % (
            vfr.heading, vfr.groundspeed, vfr.climb))
    return ' | '.join(parts) if parts else '(等待数据...)'


def main():
    parser = argparse.ArgumentParser(description='查看无人机数据（可离线）')
    parser.add_argument('--tlog', default=None,
                        help='从 test_data 的 tlog 日志读取（离线练习）')
    args = parser.parse_args()

    if args.tlog:
        master = mavutil.mavlink_connection(args.tlog)
        print('离线读取: %s' % args.tlog)
    else:
        master = connect()
        print('实时数据（Ctrl+C 退出）...')
    try:
        while True:
            # 阻塞收一帧：收到的消息会存进 master.messages，供 snapshot 读取
            # 离线读日志用非阻塞：读到末尾返回 None，直接结束
            msg = master.recv_match(blocking=not args.tlog, timeout=2)
            if msg is None:
                if args.tlog:
                    print('日志读取完毕')
                    break
                print(snapshot(master))
                continue
            print(snapshot(master))
            if not args.tlog:   # 离线回放不等待，快速读完；在线保持 0.5s 节奏
                time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n已退出')


if __name__ == '__main__':
    main()
