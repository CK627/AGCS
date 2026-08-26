#!/usr/bin/python3
# coding=utf8
"""无人机电脑端网页服务（视频 + UDP 监控 + 下达任务）。

三块能力：
1. 视频页面：把 RTSP 摄像头转成网页能显示的 MJPEG 流（/video_feed）
2. UDP 监控：后台线程读 MAVLink（UDP 14550），网页轮询 /api/telemetry 显示
3. 下达任务：网页填航点，后端转成 MAVLink 任务上传到飞控

启动:
    python3 app.py
然后浏览器打开 http://127.0.0.1:20000

安全红线：本页只上传任务，不自动解锁、不自动起飞；解锁仍用遥控器 5 通道。
"""
import argparse
import math
import os
import threading
import time

from flask import Flask, jsonify, render_template, request, Response
from pymavlink import mavutil

from config import MAVLINK_CONN, RTSP_URL, VIDEO_ENABLED, FLASK_HOST, FLASK_PORT

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'frontend')
app = Flask(__name__, template_folder=FRONTEND_DIR)

# ---- 共享数据：后台线程写，API 读，用 LOCK 保护 ----
HISTORY_MAX = 300
DATA = {
    'connected': False,
    'error': '',
    'mode': '-', 'armed': False,
    'roll': None, 'pitch': None, 'yaw': None,
    'lat': None, 'lon': None, 'alt': None,
    'fix': None, 'sats': None, 'eph': None,
    'volt': None, 'battery': None,
    'heading': None, 'groundspeed': None, 'climb': None,
    'history': {'t': [], 'roll': [], 'pitch': [], 'yaw': [], 'alt': [], 'battery': []},
}
LOCK = threading.Lock()
master = None  # 全局连接对象，读者线程和任务上传共用


def reader_loop():
    """后台线程：收 MAVLink 消息，更新监控数据。"""
    global master
    try:
        master = mavutil.mavlink_connection(MAVLINK_CONN)
        heartbeat = master.wait_heartbeat(timeout=15)
    except Exception as e:
        with LOCK:
            DATA['connected'] = False
            DATA['error'] = str(e)
        return

    if heartbeat is None:
        # wait_heartbeat 超时只返回 None，不抛异常；这里需要显式标记为未连接，
        # 否则页面会在没有飞控数据时误显示“已连接”。
        with LOCK:
            DATA['connected'] = False
            DATA['error'] = '未收到飞控心跳（检查数传/端口 14550）'
        master.close()
        master = None
        return

    with LOCK:
        DATA['connected'] = True
        DATA['error'] = ''
    t0 = time.time()

    while True:
        try:
            master.recv_match(blocking=True, timeout=2)
        except Exception:
            break
        msgs = master.messages
        with LOCK:
            hb = msgs.get('HEARTBEAT')
            if hb is not None:
                DATA['mode'] = master.flightmode
                DATA['armed'] = bool(
                    hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            att = msgs.get('ATTITUDE')
            if att is not None:
                DATA['roll'] = math.degrees(att.roll)
                DATA['pitch'] = math.degrees(att.pitch)
                DATA['yaw'] = math.degrees(att.yaw)
            gpi = msgs.get('GLOBAL_POSITION_INT')
            if gpi is not None:
                DATA['lat'] = gpi.lat / 1e7
                DATA['lon'] = gpi.lon / 1e7
                DATA['alt'] = gpi.relative_alt / 1000.0
            gps = msgs.get('GPS_RAW_INT')
            if gps is not None:
                DATA['fix'] = gps.fix_type
                DATA['sats'] = gps.satellites_visible
                DATA['eph'] = gps.eph
            bat = msgs.get('BATTERY_STATUS') or msgs.get('SYS_STATUS')
            if bat is not None:
                DATA['volt'] = bat.voltage_battery / 1000.0
                DATA['battery'] = bat.battery_remaining
            vfr = msgs.get('VFR_HUD')
            if vfr is not None:
                DATA['heading'] = vfr.heading
                DATA['groundspeed'] = vfr.groundspeed
                DATA['climb'] = vfr.climb
            if att is not None:
                h = DATA['history']
                h['t'].append(time.time() - t0)
                h['roll'].append(DATA['roll'])
                h['pitch'].append(DATA['pitch'])
                h['yaw'].append(DATA['yaw'])
                h['alt'].append(DATA['alt'] if DATA['alt'] is not None else float('nan'))
                h['battery'].append(DATA['battery'] if DATA['battery'] is not None else float('nan'))
                for key in h:
                    if len(h[key]) > HISTORY_MAX:
                        del h[key][: len(h[key]) - HISTORY_MAX]

    with LOCK:
        DATA['connected'] = False
        DATA['error'] = '连接断开'


# ---------------- 视频：RTSP → MJPEG ----------------
def gen_frames():
    import cv2  # 延迟导入，没装 OpenCV 也不影响监控/任务功能
    cap = cv2.VideoCapture(RTSP_URL)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            ok, buf = cv2.imencode('.jpg', frame)
            if ok:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.05)
    finally:
        cap.release()


@app.route('/video_feed')
def video_feed():
    if not VIDEO_ENABLED:
        return 'video disabled', 503
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ---------------- 页面与监控数据 ----------------
@app.route('/')
def index():
    return render_template('index.html', video_enabled=VIDEO_ENABLED)


@app.route('/api/telemetry')
def telemetry():
    with LOCK:
        return jsonify({k: v for k, v in DATA.items() if k != 'history'})


@app.route('/api/history')
def history():
    with LOCK:
        return jsonify(DATA['history'])


# ---------------- 下达任务 ----------------
def build_mission(waypoints, takeoff_alt):
    items = [{'seq': 0, 'command': 'NAV_TAKEOFF', 'z': float(takeoff_alt)}]
    for i, wp in enumerate(waypoints, start=1):
        items.append({'seq': i, 'command': 'NAV_WAYPOINT',
                      'lat': float(wp['lat']), 'lon': float(wp['lon']),
                      'alt': float(wp['alt'])})
    items.append({'seq': len(waypoints) + 1, 'command': 'NAV_RETURN_TO_LAUNCH'})
    return items


@app.route('/api/task/preview', methods=['POST'])
def task_preview():
    payload = request.get_json(force=True)
    wps = payload.get('waypoints', [])
    takeoff_alt = payload.get('takeoff_alt', 8.0)
    items = build_mission(wps, takeoff_alt)
    return jsonify({'ok': True, 'items': items})


@app.route('/api/task/upload', methods=['POST'])
def task_upload():
    global master
    with LOCK:
        connected = DATA['connected']
    if master is None or not connected:
        return jsonify({'ok': False, 'error': '还没连接无人机（无心跳）'}), 400
    payload = request.get_json(force=True)
    wps = payload.get('waypoints', [])
    takeoff_alt = payload.get('takeoff_alt', 8.0)
    items = build_mission(wps, takeoff_alt)
    ts, tc = 1, 1
    try:
        # 上传协议：清空 → 报数量 → 一问一答 → 收确认
        master.mav.mission_clear_all_send(ts, tc)
        master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        master.mav.mission_count_send(ts, tc, len(items))
        for item in items:
            req = master.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
                blocking=True, timeout=15)
            if req is None:
                return jsonify({'ok': False, 'error': '等待 MISSION_REQUEST 超时'}), 408
            seq = req.seq
            it = items[seq]
            if it['command'] == 'NAV_TAKEOFF':
                master.mav.mission_item_int_send(
                    ts, tc, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1 if seq == 0 else 0, 1,
                    0, 0, 0, float('nan'), 0, 0, it['z'])
            elif it['command'] == 'NAV_WAYPOINT':
                master.mav.mission_item_int_send(
                    ts, tc, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                    1, 0.5, 0, float('nan'),
                    int(it['lat'] * 1e7), int(it['lon'] * 1e7), it['alt'])
            else:
                master.mav.mission_item_int_send(
                    ts, tc, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1,
                    0, 0, 0, 0, 0, 0, 0)
        ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=15)
        if ack is None:
            return jsonify({'ok': False, 'error': '未收到 MISSION_ACK'}), 408
        if ack.type != 0:
            result = mavutil.mavlink.enums['MAV_MISSION_RESULT'][ack.type].name
            return jsonify({'ok': False, 'error': '上传被拒绝: ' + result}), 400
        return jsonify({'ok': True, 'message': '任务上传成功（未启动，解锁仍需遥控器）'})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


def main():
    parser = argparse.ArgumentParser(description='无人机网页端')
    parser.add_argument('--host', default=FLASK_HOST)
    parser.add_argument('--port', type=int, default=FLASK_PORT)
    args = parser.parse_args()
    threading.Thread(target=reader_loop, daemon=True).start()
    print('网页地址: http://%s:%d' % (args.host, args.port))
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=8)
    except ImportError:
        # 没装 waitress 时退回 Flask 开发服务器（会打印开发服务器警告）
        app.run(host=args.host, port=args.port, threaded=True)


if __name__ == '__main__':
    main()
