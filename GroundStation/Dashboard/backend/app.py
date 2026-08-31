#!/usr/bin/python3
# coding=utf8
"""地面站仪表盘（中枢）：无人机/机器人/YOLO 模型统一视图，并支持下发任务。

机器人数据流：
    树莓派 autonomous_pick.py / CS-video.py（每帧 publish_frame 压缩 JPEG）
      → task_server.py /video.mjpeg（MJPEG 流，端口 5000）
      → 本后端 /robot_video_feed 代理转发 + /api/robot/status 状态代理
      → 浏览器页面实时显示画面与参数

启动（Windows 地面站，无需 ROS）：
    python app.py
浏览器打开 http://localhost:20001

依赖：python -m pip install flask requests pymavlink
（视频预览需要 opencv-python，已随 YOLO 环境装好；无人机数据用 pymavlink
读 MAVLink，替代 ROS MAVROS）
"""
import math
import os
import argparse
import json
import socket
import threading
import time

import requests
from flask import Flask, jsonify, render_template, request, Response, send_file

import config

# frontend 目录：backend/ 的上一级的 frontend/
FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'frontend')
app = Flask(__name__, template_folder=FRONTEND_DIR)


@app.route('/common.css')
def common_css():
    """统一的通用样式文件（无人机/地面站共用）。"""
    return send_file(os.path.join(FRONTEND_DIR, 'common.css'), mimetype='text/css')


# ---------------- 无人机状态（pymavlink 读 MAVLink，后台线程） ----------------
drone_status = {'online': False, 'message': '等待无人机数据…'}

# 通过局域网广播自动发现的无人机仪表盘
DISCOVERED_DRONE = {'url': '', 'name': '', 'ip': '', 'last_seen': 0}
BEACON_PORT = 20003


def _discovery_loop():
    """监听无人机仪表盘的 UDP 广播（20003），自动记下它的地址。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', BEACON_PORT))
    except Exception as e:
        print('[发现] 监听 UDP %d 失败（可能被占用）：%s' % (BEACON_PORT, e))
        return
    print('[发现] 正在监听无人机广播（UDP %d），地面站会自动找到无人机仪表盘…' % BEACON_PORT)
    last_logged = {}  # url -> 上次打印时间（每个地址最多 30 秒打印一次，避免刷屏）
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            info = json.loads(data.decode('utf-8'))
            if info.get('type') == 'agcs_drone':
                # 用“收到广播的源 IP”拼地址：无人机有几张网卡也不怕选错
                port = int(info.get('port') or 20000)
                url = 'http://%s:%d' % (addr[0], port)
                DISCOVERED_DRONE.update({
                    'url': url,
                    'name': info.get('name', ''),
                    'ip': addr[0],
                    'last_seen': time.time(),
                })
                if time.time() - last_logged.get(url, 0) >= 30.0:
                    last_logged[url] = time.time()
                    print('[发现] 无人机仪表盘：%s（%s）' % (url, addr[0]))
        except Exception:
            continue


def _effective_drone_url():
    """实际使用的无人机仪表盘地址：配置里写了局域网 IP 就用配置；
    配置还是 127.0.0.1/空 时，用广播自动发现的地址。"""
    base = (config.DRONE_URL or '').rstrip('/')
    host = ''
    if base:
        host = base.split('://')[-1].split(':')[0]
    disc = (DISCOVERED_DRONE.get('url') or '').rstrip('/')
    if disc and (not base or host in ('127.0.0.1', 'localhost')):
        return disc
    return base or disc


def _drone_monitor():
    """无人机数据监控（后台线程）。

    优先从无人机电脑端仪表盘拉 /api/telemetry（SDK 数据：位置/姿态/电压）；
    无人机电脑端不可达时才回退到 pymavlink 读 MAVLink。
    """
    while True:
        config.reload_if_changed()
        if _pull_drone_dashboard():
            time.sleep(1.0)
            continue
        _mavlink_fallback()


def _pull_drone_dashboard():
    """从无人机电脑端仪表盘拉遥测；返回 True 表示仪表盘可达（无论无人机是否连接）。"""
    global drone_status
    try:
        url = _effective_drone_url().rstrip('/') + '/api/telemetry'
        resp = requests.get(url, timeout=3)
        data = resp.json()
    except Exception as e:
        drone_status = {
            'online': False,
            'message': '无人机电脑端不可达（检查 drone.url 与 20000 服务是否在跑）: %s' % e,
        }
        return False

    if not data.get('connected'):
        drone_status = {
            'online': False,
            'source': data.get('source', 'none'),
            'message': data.get('error') or '无人机电脑端未连接无人机',
        }
        return True

    pose = {}
    if data.get('loc_x') is not None:
        # SDK 位置单位是厘米，中枢统一按米显示
        pose['x'] = round(data['loc_x'] / 100.0, 2)
        pose['y'] = round(data['loc_y'] / 100.0, 2)
        pose['z'] = round(data['loc_z'] / 100.0, 2)
    if data.get('yaw') is not None:
        pose['yaw_deg'] = round(float(data['yaw']) % 360.0, 1)

    sdk = data.get('sdk') or {}
    source = data.get('source', 'sdk')
    msg = ('数据源：SDK（串口 %s）' % (sdk.get('serial') or '?')) if source == 'sdk' else '数据源：MAVLink'

    drone_status = {
        'online': True,
        'source': source,
        'message': msg,
        'drone_url': _effective_drone_url(),
        'pose': pose or None,
        'battery': {'voltage': round(float(data['volt']), 2)} if data.get('volt') is not None else None,
        'obstacles': {'front': data.get('obs_f'), 'back': data.get('obs_b'),
                      'left': data.get('obs_l'), 'right': data.get('obs_r')}
                     if any(data.get(k) is not None for k in ('obs_f', 'obs_b', 'obs_l', 'obs_r')) else None,
        'key_press': data.get('key_press'),
        'role_news': data.get('role_news'),
    }
    return True


def _mavlink_fallback():
    """回退数据源：pymavlink 读 MAVLink；连不上或断开后回到主循环。"""
    global drone_status
    try:
        from pymavlink import mavutil
    except ImportError as e:
        drone_status = {'online': False, 'message': '缺少 pymavlink: %s' % e}
        return

    try:
        conn = mavutil.mavlink_connection(config.DRONE_MAVLINK)
        heartbeat = conn.wait_heartbeat(timeout=5)
    except Exception as e:
        drone_status = {'online': False, 'message': 'MAVLink 未接入（检查数传/端口）: %s' % e}
        return

    if heartbeat is None:
        # wait_heartbeat 超时只返回 None，不抛异常；需要显式标记为未连接
        drone_status = {'online': False, 'message': '未收到飞控心跳（检查数传/端口 14550）'}
        try:
            conn.close()
        except Exception:
            pass
        return

    drone_status = {'online': True, 'source': 'mavlink', 'message': '数据源：MAVLink'}
    try:
        while True:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                # NED（北-东-下）→ ENU（东-北-上）：东=ned.y，北=ned.x，上=-ned.z
                drone_status['pose'] = {
                    'x': round(msg.y, 2),
                    'y': round(msg.x, 2),
                    'z': round(-msg.z, 2),
                    'yaw_deg': None,
                }
                drone_status['online'] = True
                drone_status['message'] = 'OK'
            elif mtype == 'ATTITUDE':
                # NED 航向：北为 0，顺时针为正（度）
                pose = drone_status.get('pose') or {}
                pose['yaw_deg'] = round(math.degrees(msg.yaw) % 360.0, 1)
                drone_status['pose'] = pose
                drone_status['online'] = True
            elif mtype == 'GLOBAL_POSITION_INT':
                drone_status['global'] = {
                    'lat': round(msg.lat / 1e7, 7),
                    'lon': round(msg.lon / 1e7, 7),
                    'alt': round(msg.alt / 1000.0, 2),
                }
                drone_status['online'] = True
            elif mtype == 'HEARTBEAT':
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    mode = mavutil.mode_string_v10(msg)
                except Exception:
                    mode = str(msg.custom_mode)
                drone_status['state'] = {'mode': mode, 'armed': armed}
                drone_status['online'] = True
            elif mtype == 'SYS_STATUS':
                drone_status['battery'] = {
                    'voltage': round(msg.voltage_battery / 1000.0, 2),
                    'percent': msg.battery_remaining,
                }
                drone_status['online'] = True
    except Exception as e:
        drone_status = {'online': False, 'message': 'MAVLink 连接断开（回退到无人机电脑端）: %s' % e}
        try:
            conn.close()
        except Exception:
            pass


# ---------------- YOLO 模型状态（后台检测线程） ----------------
model_stats = {'loaded': False, 'message': '模型未加载'}
latest_jpeg = None
latest_jpeg_lock = threading.Lock()
_offline_jpg = None


def _offline_frame():
    """黑底黄字状态卡：无人机电脑端不可达时也推一张看得见的图，别让页面干等。"""
    global _offline_jpg
    if _offline_jpg is None:
        import cv2
        import numpy as np
        img = np.zeros((480, 720, 3), dtype=np.uint8)
        for i, text in enumerate(['Drone dashboard unreachable',
                                  'check GroundStation/Dashboard/data/config.yaml -> drone.url',
                                  'is Dron/Dashboard running?']):
            cv2.putText(img, text, (30, 130 + i * 55), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (250, 204, 21), 2)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        _offline_jpg = buf.tobytes() if ok else b''
    return _offline_jpg or None


def _iter_mjpeg_frames(url):
    """从 MJPEG HTTP 流逐帧取出 JPEG 字节（按 JPEG 头尾标记切分，兼容各种 boundary 写法）。"""
    resp = requests.get(url, stream=True, timeout=5)
    buf = b''
    try:
        for chunk in resp.iter_content(chunk_size=4096):
            buf += chunk
            while True:
                start = buf.find(b'\xff\xd8')            # JPEG SOI
                if start < 0:
                    if len(buf) > (1 << 20):
                        buf = b''
                    break
                end = buf.find(b'\xff\xd9', start + 2)   # JPEG EOI
                if end < 0:
                    buf = buf[start:]
                    break
                yield buf[start:end + 2]
                buf = buf[end + 2:]
    finally:
        resp.close()


def _model_monitor():
    """拉取无人机电脑端（Dron/Dashboard 的 /video_feed）图传画面并转发到 /video.mjpeg。

    画面源是 EWRF 图传接收机（USB 摄像头，由 Dron/Dashboard 独占采集）：
    - 有 YOLO 环境就在地面站推理画框并统计；
    - 没有（或缺模型）就直接转发原始画面，保证"先能看到画面"；
    - 拉流失败/断开自动重连，drone.url 在 GroundStation/Dashboard/data/config.yaml 热改后自动切新地址。
    """
    global model_stats, latest_jpeg
    import cv2
    import numpy as np

    model = None
    try:
        from ultralytics import YOLO
        model = YOLO(config.MODEL_INFO['path'])
        model_stats = {'loaded': True, 'model': config.MODEL_INFO['name'],
                       'message': 'OK（未收到画面）'}
    except Exception as e:
        model_stats = {'loaded': False, 'model': config.MODEL_INFO['name'],
                       'message': 'YOLO 未启用（%s），原始画面转发' % e}

    counts = {}
    total = 0
    last_det = []
    prev = time.time()
    interval = 1.0 / max(config.VIDEO_FPS_LIMIT, 1)

    while True:
        config.reload_if_changed()
        if not config.DRONE_VIDEO_ENABLED:
            time.sleep(1.0)
            continue
        url = config.DRONE_URL.rstrip('/') + '/video_feed'
        try:
            for jpg in _iter_mjpeg_frames(url):
                config.reload_if_changed()
                if not config.DRONE_VIDEO_ENABLED:
                    break
                if config.DRONE_URL.rstrip('/') + '/video_feed' != url:
                    break  # drone.url 热改了，断开去连新地址
                now = time.time()
                if now - prev < interval:
                    continue
                fps = 1.0 / max(now - prev, 1e-6)
                prev = now
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                if model is not None:
                    dets = []
                    try:
                        res = model.predict(frame, conf=config.MODEL_INFO['conf'],
                                            imgsz=config.MODEL_INFO['imgsz'],
                                            verbose=False)[0]
                        for box in res.boxes:
                            cls = res.names[int(box.cls[0])]
                            conf = float(box.conf[0])
                            counts[cls] = counts.get(cls, 0) + 1
                            total += 1
                            dets.append({'class': cls, 'conf': round(conf, 2)})
                        frame = res.plot()
                        last_det = dets[-10:]
                    except Exception as e:
                        model_stats = {'loaded': True, 'model': config.MODEL_INFO['name'],
                                       'message': '检测出错: %s' % e}
                ok_jpg, buf = cv2.imencode('.jpg', frame,
                                           [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok_jpg:
                    with latest_jpeg_lock:
                        latest_jpeg = buf.tobytes()
                model_stats = {
                    'loaded': model is not None,
                    'model': config.MODEL_INFO['name'],
                    'classes': config.MODEL_INFO['classes'],
                    'conf': config.MODEL_INFO['conf'] if model is not None else None,
                    'fps': round(fps, 1),
                    'total_detections': total if model is not None else None,
                    'counts': counts if model is not None else None,
                    'last_detections': last_det if model is not None else None,
                    'message': 'OK' if model is not None else '原始画面转发（YOLO 未启用）',
                }
        except Exception as e:
            model_stats = {
                'loaded': model is not None,
                'model': config.MODEL_INFO['name'],
                'message': '无人机电脑端拉流失败（检查 drone.url / Dron Dashboard 是否运行）: %s' % e,
            }
            time.sleep(2.0)
        else:
            time.sleep(1.0)


# ---------------- 路由 ----------------

@app.route('/')
def index():
    config.reload_if_changed()
    return render_template('index.html',
                           robot_video_enabled=config.ROBOT_VIDEO_ENABLED,
                           drone_video_enabled=config.DRONE_VIDEO_ENABLED)


@app.route('/api/drone/status')
def api_drone():
    data = dict(drone_status)
    data['discovery'] = {
        'url': DISCOVERED_DRONE.get('url', ''),
        'name': DISCOVERED_DRONE.get('name', ''),
        'ip': DISCOVERED_DRONE.get('ip', ''),
        'last_seen': DISCOVERED_DRONE.get('last_seen', 0),
    }
    return jsonify(data)


@app.route('/api/model/status')
def api_model():
    return jsonify(model_stats)


@app.route('/api/robot/status')
def api_robot_status():
    """代理机器人 GET /status（仪表盘轮询）。"""
    config.reload_if_changed()
    try:
        r = requests.get(config.ROBOT_URL + '/status', timeout=2)
        data = r.json()
        data['online'] = True
        return jsonify(data)
    except Exception as e:
        return jsonify({'online': False, 'error': str(e)})


@app.route('/robot_video_feed')
def robot_video_feed():
    """机器人摄像头画面 MJPEG 代理（转发树莓派 task_server /video.mjpeg）。"""
    config.reload_if_changed()
    if not config.ROBOT_VIDEO_ENABLED:
        return 'robot video disabled', 503
    try:
        upstream = requests.get(config.ROBOT_URL + '/video.mjpeg',
                                stream=True, timeout=5)
    except Exception as e:
        return '机器人视频不可用（检查 ROBOT_URL / autonomous_pick 或 CS-video 是否在跑）: %s' % e, 502

    def gen():
        try:
            for chunk in upstream.iter_content(chunk_size=1024):
                if chunk:
                    yield chunk
        finally:
            upstream.close()

    ctype = upstream.headers.get(
        'Content-Type', 'multipart/x-mixed-replace; boundary=frame')
    return Response(gen(), mimetype=ctype)


@app.route('/video.mjpeg')
def video_mjpeg():
    """无人机画面 MJPEG 预览（YOLO 检测线程提供帧）。"""
    config.reload_if_changed()
    if not config.DRONE_VIDEO_ENABLED:
        return 'drone video disabled', 503

    def gen():
        while True:
            with latest_jpeg_lock:
                jpg = latest_jpeg
            if jpg is None:
                jpg = _offline_frame()
            if jpg is not None:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpg + b'\r\n')
            else:
                yield b'--frame\r\nContent-Type: text/plain\r\n\r\nno video\r\n\r\n'
            time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='地面站仪表盘')
    parser.add_argument('--host', default=config.DASHBOARD_HOST)
    parser.add_argument('--port', type=int, default=config.DASHBOARD_PORT)
    args = parser.parse_args()

    threading.Thread(target=_drone_monitor, daemon=True).start()
    threading.Thread(target=_model_monitor, daemon=True).start()
    threading.Thread(target=_discovery_loop, daemon=True).start()
    print('地面站仪表盘: http://%s:%d' % (args.host, args.port), flush=True)
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=8)
    except ImportError:
        app.run(host=args.host, port=args.port, threaded=True)
