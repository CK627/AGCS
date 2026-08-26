#!/usr/bin/python3
# coding=utf8
"""航线任务：上传 + 启动自主飞行（PX4）。

!!! 安全警告（实飞前必读）!!!
1. 默认只上传任务并打印预览，不会起飞；加 --start 才会执行
2. 本代码不会自动解锁；解锁请用遥控器 5 通道
3. 示例航点是占位坐标，实飞前必须用 --waypoints 改成你的场地
4. 起飞条件：GPS fix>=3 且星数>=10、电量充足、场地开阔、遥控器在手

用法:
    python3 mission.py --waypoints "31.2304,121.4737,10;31.2305,121.4738,12" --takeoff-alt 8
    python3 mission.py --waypoints "..." --preview   # 离线：只打印任务预览，不连接
    python3 mission.py --waypoints "..." --start   # 预检+人工确认后起飞
"""
import argparse
import time

from pymavlink import mavutil

from connect import connect, is_armed, set_px4_mode
from drone_config import TARGET_SYSTEM, TARGET_COMPONENT

# ---- MAVLink / PX4 常量 ----
FRAME = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
CMD_TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
CMD_WAYPOINT = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
CMD_RTL = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
CMD_MISSION_START = mavutil.mavlink.MAV_CMD_MISSION_START
MAIN_AUTO = mavutil.PX4_CUSTOM_MAIN_MODE_AUTO
SUB_MISSION = mavutil.PX4_CUSTOM_SUB_MODE_AUTO_MISSION


def build_mission(waypoints, takeoff_alt):
    """生成任务列表：[起飞] + 各航点 + [返航]。"""
    items = []
    # 0. 起飞（x/y=0 表示当前位置起飞）
    items.append(dict(seq=0, command=CMD_TAKEOFF, current=1, autocontinue=1,
                      p1=0.0, p2=0.0, p3=0.0, p4=float('nan'),
                      x=0, y=0, z=float(takeoff_alt)))
    # 1..n. 航点（经纬度单位 1e7，高度 m）
    for i, (lat, lon, alt) in enumerate(waypoints, start=1):
        items.append(dict(seq=i, command=CMD_WAYPOINT, current=0, autocontinue=1,
                          p1=1.0, p2=0.5, p3=0.0, p4=float('nan'),
                          x=int(round(lat * 1e7)), y=int(round(lon * 1e7)), z=float(alt)))
    # 最后一个：返航
    items.append(dict(seq=len(waypoints) + 1, command=CMD_RTL, current=0, autocontinue=1,
                      p1=0.0, p2=0.0, p3=0.0, p4=0.0, x=0, y=0, z=0.0))
    return items


def print_mission(items):
    """打印任务预览，起飞前人工核对。"""
    names = {CMD_TAKEOFF: '起飞', CMD_WAYPOINT: '航点', CMD_RTL: '返航'}
    print('[任务] 预览（起飞前请人工核对）：')
    for it in items:
        if it['command'] == CMD_WAYPOINT:
            desc = '%.7f, %.7f @ %.1fm' % (it['x'] / 1e7, it['y'] / 1e7, it['z'])
        elif it['command'] == CMD_TAKEOFF:
            desc = '原地起飞到 %.1fm' % it['z']
        else:
            desc = '返回起飞点'
        print('  #%d %s: %s' % (it['seq'], names.get(it['command'], '?'), desc))


def upload_mission(master, items):
    """按 MAVLink 协议上传任务：count → 逐条应答 → ack。"""
    ts, tc = TARGET_SYSTEM, TARGET_COMPONENT
    # 先清空旧任务（丢弃清空的 ACK）
    master.mav.mission_clear_all_send(ts, tc)
    master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    # 告知任务数量
    master.mav.mission_count_send(ts, tc, len(items))
    # 飞控会逐个 MISSION_REQUEST_INT(seq) 索要，我们按序号应答
    for item in items:
        req = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
            blocking=True, timeout=15)
        if req is None:
            raise RuntimeError('等待 MISSION_REQUEST 超时（第 %s 个任务项）' % item['seq'])
        it = items[req.seq]
        master.mav.mission_item_int_send(
            ts, tc, req.seq, FRAME, it['command'], it['current'], it['autocontinue'],
            it['p1'], it['p2'], it['p3'], it['p4'], it['x'], it['y'], it['z'])
    # 最后确认
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=15)
    if ack is None:
        raise RuntimeError('未收到 MISSION_ACK')
    result = mavutil.mavlink.enums['MAV_MISSION_RESULT'][ack.type].name
    if ack.type != 0:  # MAV_MISSION_ACCEPTED == 0
        raise RuntimeError('任务上传被拒绝: %s' % result)
    print('[任务] 上传成功（%s）' % result)


def preflight(master):
    """起飞前安全检查：解锁状态 / GPS / 电量。"""
    for _ in range(20):  # 先收几帧，让 messages 缓存数据
        master.recv_match(blocking=True, timeout=1)
        time.sleep(0.1)
    if is_armed(master):
        raise SystemExit('飞行器已解锁！请先上锁（遥控器 5 通道）再运行')
    gps = master.messages.get('GPS_RAW_INT')
    if gps is None:
        raise SystemExit('没有 GPS 数据，请到室外等待定位')
    if gps.fix_type < 3 or gps.satellites_visible < 10:
        raise SystemExit('GPS 不满足条件 fix=%s 星=%s' % (gps.fix_type, gps.satellites_visible))
    bat = master.messages.get('BATTERY_STATUS')
    if bat is not None and bat.battery_remaining >= 0 and bat.battery_remaining < 30:
        raise SystemExit('电池电量过低: %s%%' % bat.battery_remaining)
    print('[预检] 通过：GPS fix=%s 星=%s' % (gps.fix_type, gps.satellites_visible))


def start_mission(master):
    """切 AUTO.MISSION 并发 MISSION_START，然后只做监控。"""
    set_px4_mode(master, MAIN_AUTO, SUB_MISSION)
    time.sleep(1.0)
    master.mav.command_long_send(
        TARGET_SYSTEM, TARGET_COMPONENT, CMD_MISSION_START, 0,
        0, 0, 0, 0, 0, 0, 0)
    print('[任务] 已切换 AUTO.MISSION 并发送 MISSION_START')
    print('[任务] 请用遥控器 5 通道解锁；监控中（Ctrl+C 停止监控，任务仍由飞控执行）')
    while True:
        master.recv_match(type=['HEARTBEAT', 'VFR_HUD'], blocking=True, timeout=2)
        print('  模式=%s 解锁=%s' % (master.flightmode, is_armed(master)))


def parse_waypoints(text):
    """把 "lat,lon,alt;lat,lon,alt" 解析成 [(lat, lon, alt), ...]。"""
    wps = []
    for seg in text.split(';'):
        seg = seg.strip()
        if not seg:
            continue
        lat, lon, alt = (float(x) for x in seg.split(','))
        wps.append((lat, lon, alt))
    if not wps:
        raise SystemExit('航点格式错误，示例: 31.2304,121.4737,10;31.2305,121.4738,12')
    return wps


def main():
    parser = argparse.ArgumentParser(description='航线任务上传/启动（PX4）')
    parser.add_argument('--waypoints', default=None,
                        help='"lat,lon,alt;lat,lon,alt" 航点列表')
    parser.add_argument('--takeoff-alt', type=float, default=8.0, help='起飞高度 m')
    parser.add_argument('--start', action='store_true', help='预检通过后启动任务（高危）')
    parser.add_argument('--preview', action='store_true',
                        help='只打印任务预览，不连接飞控（离线练习）')
    args = parser.parse_args()

    if args.waypoints:
        wps = parse_waypoints(args.waypoints)
    else:
        print('!! 警告：使用示例占位航点，实飞前必须用 --waypoints 指定实际场地 !!')
        wps = [(31.2304, 121.4737, 10.0), (31.2305, 121.4738, 12.0)]

    items = build_mission(wps, args.takeoff_alt)
    print_mission(items)

    if args.preview:
        print('[任务] 预览模式结束（未连接飞控）')
        return

    master = connect()
    upload_mission(master, items)

    if not args.start:
        print('[任务] 仅上传完成，未启动。确认无误后加 --start 执行。')
        return

    preflight(master)
    ans = input('确认航点与场地无误？输入 YES 继续: ')
    if ans.strip() != 'YES':
        print('已取消')
        return
    try:
        start_mission(master)
    except KeyboardInterrupt:
        print('\n[任务] 停止监控（任务仍由飞控执行，可用遥控器接管）')


if __name__ == '__main__':
    main()
