#!/usr/bin/python3
# coding=utf8
"""无人机电脑端网页服务（视频 + 无人机数据监控）。

三块能力：
1. 视频页面：EWRF 图传接收机（USB 摄像头）→ 后台线程独占采集 → /video_feed MJPEG 流
   （接收方式同 外接图传录制程序/capture_external_camera.py；摄像头序号改 Dorn/Dashboard/data/config.yaml）
   启动时自检“摄像头设备在不在、画面有没有真实信号”，状态见 /api/camera/status
2. 无人机数据：优先用 SDK（串口 helloFly）读位置/姿态/电压，MAVLink 作为备用源，
   网页轮询 /api/telemetry 显示
3. 自动任务由 OpenFly 图形化编程处理，本页不再提供任务上传

启动:
    python3 app.py
然后浏览器打开 http://127.0.0.1:20000

安全红线：本页只上传任务，不自动解锁、不自动起飞；解锁仍用遥控器 5 通道。
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time

from flask import Flask, jsonify, render_template, request, Response, send_file

import config
import drone_link
import sdk_telemetry

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'frontend')
app = Flask(__name__, template_folder=FRONTEND_DIR)


@app.route('/common.css')
def common_css():
    """统一的通用样式文件（无人机/地面站共用）。"""
    return send_file(os.path.join(FRONTEND_DIR, 'common.css'), mimetype='text/css')

# ---- 共享数据：后台线程写，API 读，用 LOCK 保护 ----
HISTORY_MAX = 300
DATA = {
    'connected': False,
    'source': 'none',            # none / mavlink / sdk
    'error': '',
    'mode': '-', 'armed': False,
    'roll': None, 'pitch': None, 'yaw': None,
    'lat': None, 'lon': None, 'alt': None,
    'loc_x': None, 'loc_y': None, 'loc_z': None,   # SDK 位置（厘米）
    'err_x': None, 'err_y': None, 'err_z': None,   # 定位误差（厘米）
    'obs_f': None, 'obs_b': None, 'obs_l': None, 'obs_r': None,   # 避障距离（cm）
    'key_press': None,                                # 遥控器按键
    'role_news': None, 'role_news_id': None, 'timer': None,   # 消息 / 计时器
    'fix': None, 'sats': None, 'eph': None,
    'volt': None,
    'sdk': {'serial': '', 'error': '', 'checked_at': None},
    'history': {'t': [], 'roll': [], 'pitch': [], 'yaw': [], 'alt': []},
}
LOCK = threading.Lock()
master = None  # 全局连接对象，读者线程和任务上传共用
sdk_thread = None  # SDK 遥测线程（sdk_telemetry.SdkTelemetry）
_HISTORY_T0 = time.time()
_FAKE_T0 = time.time()   # 假数据（无真实数据时的演示曲线）时间基准
_REAL_SEEN = False       # 是否已收到过真实数据；一旦为 True 就不再生成假数据


def _sdk_connected():
    """SDK 遥测是否已连接（优先作为无人机数据源）。"""
    if sdk_thread is None:
        return False
    return bool(sdk_thread.snapshot().get('connected'))


def _sdk_on_update(snap):
    """SDK 遥测线程回调：把位置/姿态/电压合并进共享数据。"""
    global _REAL_SEEN
    with LOCK:
        if snap['connected']:
            DATA['connected'] = True
            DATA['source'] = 'sdk'
            DATA['error'] = ''
            for key in ('loc_x', 'loc_y', 'loc_z', 'roll', 'pitch', 'yaw', 'volt',
                        'err_x', 'err_y', 'err_z',
                        'obs_f', 'obs_b', 'obs_l', 'obs_r',
                        'key_press', 'role_news', 'role_news_id', 'timer'):
                DATA[key] = snap.get(key)
            DATA['sdk'] = {
                'serial': snap.get('serial', ''),
                'error': snap.get('error', ''),
                'checked_at': snap.get('checked_at'),
            }
            # 追加历史曲线（高度用 loc_z 厘米换算成米，与 MAVLink 的 alt 同单位）
            h = DATA['history']
            h['t'].append(time.time() - _HISTORY_T0)
            h['roll'].append(DATA['roll'])
            h['pitch'].append(DATA['pitch'])
            h['yaw'].append(DATA['yaw'])
            h['alt'].append(DATA['loc_z'] / 100.0 if DATA['loc_z'] is not None else float('nan'))
            for key in h:
                if len(h[key]) > HISTORY_MAX:
                    del h[key][: len(h[key]) - HISTORY_MAX]
            if any(DATA[k] is not None for k in ('roll', 'pitch', 'yaw', 'loc_z')):
                _REAL_SEEN = True
        else:
            if DATA.get('source') == 'sdk':
                DATA['source'] = 'none'
            DATA['sdk'] = {
                'serial': snap.get('serial', ''),
                'error': snap.get('error', ''),
                'checked_at': snap.get('checked_at'),
            }


def reader_loop():
    """后台线程：收 MAVLink 消息，更新监控数据。"""
    global master, _REAL_SEEN
    try:
        from pymavlink import mavutil
    except ImportError as e:
        if _sdk_connected():
            return  # SDK 已提供数据，MAVLink 缺库不再报错
        with LOCK:
            DATA['connected'] = False
            DATA['error'] = '缺少 pymavlink: %s' % e
        return
    try:
        master = mavutil.mavlink_connection(config.MAVLINK_CONN)
        heartbeat = master.wait_heartbeat(timeout=15)
    except Exception as e:
        if _sdk_connected():
            return
        with LOCK:
            DATA['connected'] = False
            DATA['error'] = str(e)
        return

    if heartbeat is None:
        # wait_heartbeat 超时只返回 None，不抛异常；这里需要显式标记为未连接，
        # 否则页面会在没有飞控数据时误显示“已连接”。
        if _sdk_connected():
            master.close()
            master = None
            return
        with LOCK:
            DATA['connected'] = False
            DATA['error'] = '未收到飞控心跳（检查数传/端口 14550）'
        master.close()
        master = None
        return

    with LOCK:
        if not _sdk_connected():
            DATA['connected'] = True
            DATA['error'] = ''
            DATA['source'] = 'mavlink'
    t0 = time.time()

    while True:
        try:
            master.recv_match(blocking=True, timeout=2)
        except Exception:
            break
        if _sdk_connected():
            continue  # SDK 优先：位置/姿态/电压由 SDK 数据源负责
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
            if att is not None:
                h = DATA['history']
                h['t'].append(time.time() - t0)
                h['roll'].append(DATA['roll'])
                h['pitch'].append(DATA['pitch'])
                h['yaw'].append(DATA['yaw'])
                h['alt'].append(DATA['alt'] if DATA['alt'] is not None else float('nan'))
                for key in h:
                    if len(h[key]) > HISTORY_MAX:
                        del h[key][: len(h[key]) - HISTORY_MAX]
                _REAL_SEEN = True

    with LOCK:
        if not _sdk_connected():
            DATA['connected'] = False
            DATA['error'] = '连接断开'


# ---------------- 视频：EWRF 图传接收机（USB 摄像头）→ MJPEG ----------------
latest_jpeg = None
latest_jpeg_lock = threading.Lock()

# 摄像头/链路状态：后台采集线程写，/api/camera/status 与网页读，用锁保护
CAMERA_STATUS = {
    'state': 'checking',          # disabled / device_missing / no_signal / ok
    'message': '正在检测摄像头…',
    'camera_index': config.CAMERA_INDEX,
    'video_enabled': config.VIDEO_ENABLED,
    'device_ok': False,           # 设备能打开、能读出帧
    'camera_on': False,           # 画面里有真实内容（图传信号在）
    'width': None,
    'height': None,
    'fps': None,
    'signal_mean': None,
    'checked_at': None,
}
CAMERA_LOCK = threading.Lock()


def _set_camera_status(**kwargs):
    """更新摄像头状态（自动带上检查时间）。"""
    with CAMERA_LOCK:
        CAMERA_STATUS.update(kwargs)
        CAMERA_STATUS['checked_at'] = time.time()


def _frame_signal(frame):
    """判断一帧有没有真实画面内容。

    图传接收机在发射端没开机时，画面通常是：
    - 全黑（亮度极低）；
    - 纯蓝屏/纯色屏（如 EWRF 的 NO SIGNAL 蓝屏：单一颜色通道压倒性占优，
      且整帧几乎不变）；
    - 静帧（整帧几乎没有纹理变化）。
    以上情况都视为“无信号”。
    """
    import cv2
    b, g, r = cv2.split(frame)
    bm, gm, rm = float(b.mean()), float(g.mean()), float(r.mean())
    max_std = max(float(b.std()), float(g.std()), float(r.std()))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_mean = float(gray.mean())
    gray_std = float(gray.std())

    # 全黑
    if gray_mean < 10.0:
        return False, gray_mean
    # 纯色屏（蓝屏等）：整帧几乎不变 + 某个颜色通道压倒性占优
    sorted_means = sorted((bm, gm, rm))
    if max_std < 20.0 and sorted_means[2] - sorted_means[1] > 50.0:
        return False, gray_mean
    # 静帧 / 无纹理
    if gray_std < 6.0:
        return False, gray_mean
    return True, gray_mean


def _sample_signal(cap, count=5):
    """连读几帧判断图传信号：半数以上帧有内容就算有信号。

    返回 (camera_on, 平均亮度)；读帧失败返回 (False, None)。
    """
    results = []
    means = []
    for _ in range(count):
        try:
            ok, frame = cap.read()
        except Exception:
            return False, None
        if not ok:
            return False, None
        on, mean = _frame_signal(frame)
        results.append(on)
        means.append(mean)
    on = sum(results) >= (count + 1) // 2
    return on, sum(means) / len(means)


def _status_frame(lines):
    """生成黑底黄字状态卡：摄像头打不开时也照常推流，链路通不通、问题在哪一眼可见。"""
    import cv2
    import numpy as np
    img = np.zeros((480, 720, 3), dtype=np.uint8)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (30, 130 + i * 55), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (250, 204, 21), 2)
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    return buf.tobytes() if ok else None


def _open_receiver(index):
    """打开 EWRF 接收机对应的 USB 摄像头；Windows 走 DirectShow（同录制程序）。"""
    import cv2
    if sys.platform == 'win32':
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    # 图传接收机标称 720x480（同 capture_external_camera.py）
    for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, 720),
                        (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                        (cv2.CAP_PROP_FPS, 30)):
        try:
            cap.set(prop, value)
        except Exception:
            pass  # 部分设备不支持设置时忽略，不影响读取
    return cap


def camera_loop():
    """后台线程：独占 USB 摄像头持续采集，最新一帧压 JPEG 存共享缓冲。

    - 全进程只有一个线程持有摄像头，浏览器多人同时看也不会重复占设备
    - 摄像头序号在 Dorn/Dashboard/data/config.yaml 里热改后自动重开设备
    - 打不开（没插/无权限/序号错）或画面无信号时推状态卡并自动重试/复检，
      接收机或飞机摄像头后开都不用重启
    """
    global latest_jpeg
    import cv2
    cap = None
    using_index = None
    interval = 1.0 / 10.0   # 推流上限 10fps，别占满带宽
    last_pub = 0.0
    signal_interval = 2.0   # 每 2 秒复检一次画面信号
    last_signal_check = 0.0
    while True:
        config.reload_if_changed()
        if not config.VIDEO_ENABLED:
            if cap is not None:
                cap.release()
                cap = None
                using_index = None
            _set_camera_status(
                state='disabled',
                message='视频已关闭（drone.video=false）',
                device_ok=False, camera_on=False,
                width=None, height=None, fps=None)
            time.sleep(1.0)
            continue
        if cap is None or config.CAMERA_INDEX != using_index:
            if cap is not None:
                cap.release()
            using_index = config.CAMERA_INDEX
            _set_camera_status(camera_index=using_index, video_enabled=True)
            try:
                cap = _open_receiver(using_index)
            except Exception as e:
                print('[视频] 打开摄像头 %s 异常：%s' % (using_index, e))
                cap = None
            if cap is None:
                print('[视频] 打不开摄像头 %s（序号错/没插/无权限），3 秒后自动重试' % using_index)
                _set_camera_status(
                    state='device_missing',
                    message='未检测到图传接收机（摄像头 %s）' % using_index,
                    device_ok=False, camera_on=False,
                    width=None, height=None, fps=None)
                with latest_jpeg_lock:
                    latest_jpeg = _status_frame([
                        'EWRF receiver not found (camera %s)' % using_index,
                        'check USB cable / camera index',
                        'edit Dorn/Dashboard/data/config.yaml -> drone.camera',
                    ])
                time.sleep(3.0)
                continue
            _set_camera_status(device_ok=True)
        try:
            ok, frame = cap.read()
        except Exception:
            ok = False
            frame = None
        if not ok:
            cap.release()
            cap = None
            print('[视频] 读取画面失败，重新打开摄像头')
            _set_camera_status(
                state='device_missing',
                message='摄像头读取失败，正在重试',
                device_ok=False, camera_on=False)
            time.sleep(1.0)
            continue
        if config.VIDEO_CROP_TOP > 0:
            frame = frame[config.VIDEO_CROP_TOP:, :]  # 去掉画面顶部 OSD 文字
        now = time.time()
        if now - last_signal_check >= signal_interval:
            last_signal_check = now
            on, mean = _sample_signal(cap)
            _set_camera_status(
                camera_on=on,
                signal_mean=mean,
                state='ok' if on else 'no_signal',
                message='图传信号正常' if on else '摄像头无信号（蓝屏/黑屏），请确认飞机摄像头已开机',
                width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                height=int(frame.shape[0]),
                fps=round(float(cap.get(cv2.CAP_PROP_FPS)) or 0, 1))
            if not on:
                with latest_jpeg_lock:
                    latest_jpeg = _status_frame([
                        'No signal (blue/black screen)',
                        'power on drone camera / transmitter',
                        'check EWRF receiver input',
                    ])
        if now - last_pub >= interval:
            last_pub = now
            ok_jpg, buf = cv2.imencode('.jpg', frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok_jpg:
                with latest_jpeg_lock:
                    latest_jpeg = buf.tobytes()
        time.sleep(0.02)


@app.route('/video_feed')
def video_feed():
    config.reload_if_changed()
    if not config.VIDEO_ENABLED:
        return 'video disabled', 503

    def gen():
        while True:
            with latest_jpeg_lock:
                jpg = latest_jpeg
            if jpg is not None:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpg + b'\r\n')
            else:
                yield (b'--frame\r\nContent-Type: text/plain\r\n\r\n'
                       b'no video\r\n\r\n')
            time.sleep(0.1)

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ---------------- 摄像头/链路状态 ----------------
_last_serial_check = 0.0
_last_serial_result = []


# ---------------- 局域网广播（让地面站自动发现本机） ----------------
BEACON_PORT = 20003


def _lan_ips():
    """返回本机所有局域网 IP。

    服务监听 0.0.0.0，任何网卡地址都能访问，所以不做网卡过滤，
    全部广播出去，客户端通过哪个网卡能通就用哪个。
    """
    try:
        ips = socket.gethostbyname_ex(socket.gethostname())[2]
    except Exception:
        return []
    cands = [ip for ip in ips
             if not ip.startswith('127.')
             and ip.startswith(('10.', '192.168.', '172.'))]

    def key(ip):
        return (0 if ip.startswith('10.') else
                1 if ip.startswith('192.168.') else 2)
    return sorted(cands, key=key)


def _beacon_loop():
    """每 2 秒向局域网广播无人机仪表盘地址（UDP 20003），供地面站自动发现。"""
    ips = _lan_ips()
    if not ips:
        print('[广播] 找不到局域网 IP，无法广播无人机地址')
        return
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print('[广播] 每 2 秒向局域网广播无人机地址（UDP %d）：%s'
          % (BEACON_PORT, ', '.join('http://%s:%d' % (ip, config.FLASK_PORT) for ip in ips)))
    while True:
        config.reload_if_changed()
        port = config.FLASK_PORT
        targets = {'255.255.255.255'}
        for ip in ips:
            targets.add('.'.join(ip.split('.')[:-1]) + '.255')  # 每个网段的子网广播
        for target in targets:
            try:
                payload = json.dumps({
                    'type': 'agcs_drone',
                    'name': 'drone-dashboard',
                    'port': port,
                }).encode('utf-8')
                sock.sendto(payload, (target, BEACON_PORT))
            except Exception as e:
                print('[广播] 发送到 %s 失败：%s' % (target, e))
        time.sleep(2.0)


def _drone_link_info():
    """无人机串口（遥控器链路）状态，2 秒缓存一次，避免频繁枚举串口。"""
    global _last_serial_check, _last_serial_result
    now = time.time()
    if now - _last_serial_check >= 2.0:
        _last_serial_check = now
        _last_serial_result = drone_link.find_drone_serial()
    return {
        'serial_available': drone_link.serial_available(),
        'devices': list(_last_serial_result),
    }


@app.route('/api/camera/status')
def camera_status():
    """启动自检 + 实时状态：摄像头设备/信号/分辨率 + 无人机串口链路。"""
    config.reload_if_changed()
    with CAMERA_LOCK:
        data = dict(CAMERA_STATUS)
    data['drone_link'] = _drone_link_info()
    return jsonify(data)


# ---------------- 页面与监控数据 ----------------
@app.route('/')
def index():
    config.reload_if_changed()
    return render_template('index.html', video_enabled=config.VIDEO_ENABLED)


@app.route('/api/telemetry')
def telemetry():
    """无人机遥测：只返回当前数据源真实可获取的字段。"""
    with LOCK:
        data = {k: v for k, v in DATA.items() if k != 'history'}
    if sdk_thread is not None:
        snap = sdk_thread.snapshot()
        if snap.get('connected'):
            # SDK 是首选数据源：把最新 SDK 数值覆盖到返回值里
            for key in ('loc_x', 'loc_y', 'loc_z', 'roll', 'pitch', 'yaw', 'volt'):
                if snap.get(key) is not None:
                    data[key] = snap[key]
            data['source'] = 'sdk'
            data['connected'] = True
            data['error'] = ''
            data['sdk'] = {
                'serial': snap.get('serial', ''),
                'error': snap.get('error', ''),
                'checked_at': snap.get('checked_at'),
            }
        elif snap.get('error') and data.get('source') != 'sdk':
            data['error'] = snap['error']
    source = data.get('source')
    if source == 'mavlink':
        keep = ('connected', 'source', 'error', 'mode', 'armed', 'roll', 'pitch', 'yaw',
                'lat', 'lon', 'alt', 'fix', 'sats', 'eph', 'volt', 'battery',
                'heading', 'groundspeed', 'climb')
        return jsonify({k: data.get(k) for k in keep})
    # SDK 无人机可获取的数据（厘米、度、伏）
    keep = ('connected', 'source', 'error', 'roll', 'pitch', 'yaw',
            'loc_x', 'loc_y', 'loc_z', 'err_x', 'err_y', 'err_z',
            'volt',
            'obs_f', 'obs_b', 'obs_l', 'obs_r',
            'key_press', 'role_news', 'role_news_id', 'timer',
            'sdk')
    return jsonify({k: data.get(k) for k in keep})


@app.route('/api/history')
def history():
    with LOCK:
        _seed_fake_history()
        return jsonify(DATA['history'])


def _seed_fake_history():
    """没有任何真实数据时，给曲线图生成演示数据，让坐标轴/线条一打开就有。

    规则：
    - 一旦收到过真实数据（_REAL_SEEN），停止生成假数据；
    - 已生成的假数据不会立即消失：真实数据持续入队时，会把假数据逐个挤出
      HISTORY_MAX 窗口，随轮询一点点淡出；
    - 没有任何真实数据时，每次轮询补 1 个点，曲线持续推进。
    调用方必须已持有 LOCK。
    """
    global _REAL_SEEN
    if _REAL_SEEN:
        return
    h = DATA['history']
    if not h['t']:
        # 首次：先铺满一屏（60 个点），页面一打开就能看到曲线
        for i in range(60):
            _append_fake_point(h, i)
    else:
        _append_fake_point(h, len(h['t']))
    for key in h:
        if len(h[key]) > HISTORY_MAX:
            del h[key][: len(h[key]) - HISTORY_MAX]


def _append_fake_point(h, i):
    """追加一个形态接近真实遥测的演示点（小幅波动 + 缓慢漂移）。"""
    h['t'].append(time.time() - _FAKE_T0)
    h['roll'].append(round(4.0 * math.sin(i / 12.0) + 0.6 * math.sin(i / 3.0), 2))
    h['pitch'].append(round(3.5 * math.cos(i / 15.0) + 0.5 * math.sin(i / 4.0), 2))
    h['yaw'].append(round(30.0 + 8.0 * math.sin(i / 30.0), 2))
    h['alt'].append(round(1.2 + 0.5 * math.sin(i / 12.0) + 0.3 * math.sin(i / 5.0), 2))


def startup_check():
    """启动自检：无人机串口链路 + 图传摄像头（设备与信号）。"""
    config.reload_if_changed()
    print('=== 启动自检 ===')

    # 1) 无人机串口链路（遥控器 USB 是否插上）
    if drone_link.serial_available():
        serials = drone_link.find_drone_serial()
        if serials:
            print('无人机串口（遥控器链路）：%s' % ', '.join(serials))
        else:
            print('无人机串口（遥控器链路）：未检测到（请确认遥控器已连电脑）')
    else:
        print('未安装 pyserial，跳过串口检测（python -m pip install pyserial）')

    # 2) 图传摄像头：设备/信号检测由后台 camera_loop 持续进行（自动重试），
    #    这里只打印摄像头序号，避免设备卡死拖住服务启动
    print('摄像头序号：%s（可用 capture_external_camera.py --list 查看；后台自动检测/重试）'
          % config.CAMERA_INDEX)


def main():
    parser = argparse.ArgumentParser(description='无人机网页端')
    parser.add_argument('--host', default=config.FLASK_HOST)
    parser.add_argument('--port', type=int, default=config.FLASK_PORT)
    args = parser.parse_args()
    startup_check()
    global sdk_thread
    sdk_thread = sdk_telemetry.SdkTelemetry(drone_id=0, on_update=_sdk_on_update)
    sdk_thread.start()
    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()
    threading.Thread(target=_beacon_loop, daemon=True).start()
    print('网页地址: http://%s:%d' % (args.host, args.port))
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=8)
    except ImportError:
        # 没装 waitress 时退回 Flask 开发服务器（会打印开发服务器警告）
        app.run(host=args.host, port=args.port, threaded=True)


if __name__ == '__main__':
    main()
