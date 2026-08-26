#!/usr/bin/python3
# coding=utf8
"""地面站仪表盘：显示无人机/机器人/YOLO 模型信息，并支持下发任务。

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
import threading
import time

import requests
from flask import Flask, jsonify, render_template, request, Response

import config

# frontend 目录：backend/ 的上一级的 frontend/
FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'frontend')
app = Flask(__name__, template_folder=FRONTEND_DIR)


# ---------------- 无人机状态（pymavlink 读 MAVLink，后台线程） ----------------
drone_status = {'online': False, 'message': 'pymavlink 未接入'}


def _drone_monitor():
    """用 pymavlink 读飞控 MAVLink 并缓存最新值；未连接时显示 offline。"""
    global drone_status
    try:
        from pymavlink import mavutil
    except ImportError as e:
        drone_status = {'online': False, 'message': '缺少 pymavlink: %s' % e}
        return

    try:
        conn = mavutil.mavlink_connection(config.DRONE_MAVLINK)
        heartbeat = conn.wait_heartbeat(timeout=15)
    except Exception as e:
        drone_status = {'online': False, 'message': '未收到飞控心跳（检查数传/端口）: %s' % e}
        return

    if heartbeat is None:
        # wait_heartbeat 超时只返回 None，不抛异常；需要显式标记为未连接
        drone_status = {'online': False, 'message': '未收到飞控心跳（检查数传/端口 14550）'}
        try:
            conn.close()
        except Exception:
            pass
        return

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


# ---------------- YOLO 模型状态（后台检测线程） ----------------
model_stats = {'loaded': False, 'message': '模型未加载'}
latest_jpeg = None
latest_jpeg_lock = threading.Lock()


def _model_monitor():
    """拉取无人机 RTSP 流跑 YOLO，更新模型信息/检测统计/预览帧。"""
    global model_stats, latest_jpeg
    try:
        import cv2
        from ultralytics import YOLO
    except ImportError as e:
        model_stats = {'loaded': False, 'message': '缺少依赖（opencv/ultralytics）: %s' % e}
        return

    try:
        model = YOLO(config.MODEL_INFO['path'])
    except Exception as e:
        model_stats = {'loaded': False, 'message': '模型加载失败: %s' % e}
        return

    cap = cv2.VideoCapture(config.DRONE_RTSP)
    counts = {}
    total = 0
    last_det = []
    prev = time.time()
    interval = 1.0 / max(config.VIDEO_FPS_LIMIT, 1)

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(1)
            continue
        t0 = time.time()
        fps = 1.0 / max(t0 - prev, 1e-6)
        prev = t0
        dets = []
        try:
            res = model.predict(frame, conf=config.MODEL_INFO['conf'],
                                imgsz=config.MODEL_INFO['imgsz'],
                                verbose=False, device=0)[0]
            for box in res.boxes:
                cls = res.names[int(box.cls[0])]
                conf = float(box.conf[0])
                counts[cls] = counts.get(cls, 0) + 1
                total += 1
                dets.append({'class': cls, 'conf': round(conf, 2)})
            frame = res.plot()
        except Exception as e:
            model_stats = {'loaded': True, 'message': '检测出错: %s' % e}
        last_det = dets[-10:]

        with latest_jpeg_lock:
            latest_jpeg = None
            try:
                ok_jpg, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok_jpg:
                    latest_jpeg = buf.tobytes()
            except Exception:
                pass

        model_stats = {
            'loaded': True,
            'model': config.MODEL_INFO['name'],
            'classes': config.MODEL_INFO['classes'],
            'conf': config.MODEL_INFO['conf'],
            'fps': round(fps, 1),
            'total_detections': total,
            'counts': counts,
            'last_detections': last_det,
            'message': 'OK',
        }
        time.sleep(max(0.0, interval - (time.time() - t0)))


# ---------------- 路由 ----------------

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/drone/status')
def api_drone():
    return jsonify(drone_status)


@app.route('/api/model/status')
def api_model():
    return jsonify(model_stats)


@app.route('/api/robot/status')
def api_robot_status():
    """代理机器人 GET /status（仪表盘轮询）。"""
    try:
        r = requests.get(config.ROBOT_URL + '/status', timeout=2)
        data = r.json()
        data['online'] = True
        return jsonify(data)
    except Exception as e:
        return jsonify({'online': False, 'error': str(e)})


@app.route('/api/robot/task', methods=['POST'])
def api_robot_task():
    """把任务转发给机器人（仪表盘页面上的"下发任务"按钮）。"""
    try:
        task = request.get_json(force=True)
        r = requests.post(config.ROBOT_URL + '/task', json=task, timeout=5)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({'status': 'error', 'reason': str(e)}), 502


@app.route('/video.mjpeg')
def video_mjpeg():
    """无人机画面 MJPEG 预览（YOLO 检测线程提供帧）。"""
    def gen():
        while True:
            with latest_jpeg_lock:
                jpg = latest_jpeg
            if jpg is not None:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
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
    print('地面站仪表盘: http://%s:%d' % (args.host, args.port), flush=True)
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=8)
    except ImportError:
        app.run(host=args.host, port=args.port, threaded=True)
