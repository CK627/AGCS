#!/usr/bin/python3
# coding=utf8
"""最小 MAVLink 读取脚本（对应《MAVLink 与 pymavlink》教学文档里的 read_drone.py）。

用法:
    python3 read_drone.py                                     # 在线（模拟器/真机）
    python3 read_drone.py --tlog test_data/flight_sample.tlog  # 离线：读测试日志

只读安全：只打印数据，不发送任何指令。
"""
import argparse

from pymavlink import mavutil

from drone_config import CONNECTION


def main():
    parser = argparse.ArgumentParser(description='读取无人机数据（可离线）')
    parser.add_argument('--tlog', default=None,
                        help='从 test_data 的 tlog 日志读取（离线练习）')
    args = parser.parse_args()

    if args.tlog:
        conn = mavutil.mavlink_connection(args.tlog)
        print('离线读取: %s' % args.tlog)
    else:
        conn = mavutil.mavlink_connection(CONNECTION)
        print('等待飞控心跳...')
        conn.wait_heartbeat(timeout=15)
        print('已连接，目标系统 %d' % conn.target_system)

    while True:
        # 离线读日志用非阻塞：读到末尾返回 None，直接结束；
        # 在线保持阻塞等待实时消息
        msg = conn.recv_match(blocking=not args.tlog, timeout=1.0)
        if msg is None:
            if args.tlog:
                print('日志读取完毕')
                break
            continue
        mtype = msg.get_type()
        if mtype == 'LOCAL_POSITION_NED':
            # NED（北-东-下）→ ENU（东-北-上）
            print('position ENU(m): x=%.2f y=%.2f z=%.2f' % (msg.y, msg.x, -msg.z))
        elif mtype == 'GLOBAL_POSITION_INT':
            print('GPS: lat=%.7f lon=%.7f alt=%.2fm' %
                  (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0))
        elif mtype == 'ATTITUDE':
            print('yaw=%.1fdeg' % (msg.yaw * 57.2958))
        elif mtype == 'HEARTBEAT':
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print('mode=%s armed=%s' % (mavutil.mode_string_v10(msg), armed))
        elif mtype == 'SYS_STATUS':
            print('battery: %.2fV %d%%' % (msg.voltage_battery / 1000.0,
                                           msg.battery_remaining))


if __name__ == '__main__':
    main()
