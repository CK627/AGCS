#!/usr/bin/python3
# coding=utf8
"""任务接收服务：地面站 ↔ 机器人通信的机器人端。

地面站通过 HTTP POST /task 下发目标坐标，本模块把任务放进队列，
autonomous_pick.py 的 NAV 状态从队列取任务执行；
GET /status 供地面站仪表盘轮询机器人状态；
/video.mjpeg 把 autonomous_pick 喂进来的摄像头画面推成网页可看的 MJPEG 流。

用法：
    集成模式：autonomous_pick.py 启动时自动在后台线程启动本服务；
    独立调试：python3 communication/task_server.py
"""
import json
import queue
import threading
import time

TASK_FILE = '/tmp/task.json'
task_queue = queue.Queue()

# 机器人状态共享区（由 autonomous_pick 更新，GET /status 读取）
robot_status = {
    'online': True,
    'state': 'IDLE',
    'position_m': {'x': 0.0, 'y': 0.0},
    'heading_deg': 0.0,
    'picked_count': 0,
    'last_task': None,
    'last_result': None,
    'message': '等待任务',
    'time': 0.0,
}

# ---------------- 视频回传（autonomous_pick 喂帧，/video.mjpeg 推流） ----------------
VIDEO_JPEG_QUALITY = 60   # JPEG 质量（0-100），越小越省带宽
latest_jpeg = None
latest_jpeg_lock = threading.Lock()
_last_publish_time = 0.0


def publish_frame(frame, max_fps=10.0):
    """主程序每帧调用：把最新一帧压缩成 JPEG 存入共享缓冲，供网页推流。

    - 按 max_fps 限流（默认 10 FPS），避免编码占用树莓派 CPU 和 WiFi 带宽
    - 无摄像头 / 编码失败时静默跳过，绝不影响主程序
    """
    global latest_jpeg, _last_publish_time
    interval = 1.0 / max(max_fps, 1.0)
    now = time.time()
    with latest_jpeg_lock:
        if now - _last_publish_time < interval:
            return
        _last_publish_time = now
    try:
        import cv2
        ok, buf = cv2.imencode(
            '.jpg', frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), VIDEO_JPEG_QUALITY])
        if not ok:
            return
    except Exception:
        return
    with latest_jpeg_lock:
        latest_jpeg = buf.tobytes()


def set_status(**kwargs):
    """更新机器人状态（autonomous_pick 每帧/事件时调用）。"""
    robot_status.update(kwargs)
    robot_status['time'] = time.time()


def get_next_task(timeout=0.1):
    """取下一个任务；没有则返回 None。"""
    try:
        return task_queue.get(timeout=timeout)
    except queue.Empty:
        return None


def _create_app():
    """创建 Flask 应用（延迟导入，Flask 未安装时不影响主程序启动）。"""
    from flask import Flask, request, jsonify, Response

    app = Flask(__name__)

    @app.route('/task', methods=['POST'])
    def receive_task():
        try:
            task = request.get_json(force=True)
        except Exception:
            return jsonify({'status': 'rejected', 'reason': 'invalid json'}), 400
        t = task.get('target')
        if not t or 'x' not in t or 'y' not in t:
            return jsonify({'status': 'rejected', 'reason': 'missing target.x/y'}), 400
        task_queue.put(task)
        with open(TASK_FILE, 'w') as f:
            json.dump(task, f)
        set_status(last_task=task, message='收到任务 %s' % task.get('task_id', '?'))
        print('[TASK] 收到任务:', task)
        return jsonify({'status': 'accepted'})

    @app.route('/status', methods=['GET'])
    def status():
        return jsonify(robot_status)

    @app.route('/video.mjpeg')
    def video_mjpeg():
        """机器人摄像头画面 MJPEG 流（autonomous_pick 通过 publish_frame 喂帧）。"""
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

    return app


def start_server(host='0.0.0.0', port=5000):
    """在后台线程启动 HTTP 服务（autonomous_pick 启动时调用一次）。"""
    try:
        app = _create_app()
    except ImportError:
        print('[TASK] Flask 未安装，跳过 HTTP 服务（需 pip3 install flask）')
        return None
    t = threading.Thread(
        target=app.run,
        # threaded=True：视频流会长期占用一个连接，必须支持并发处理 /status、/task
        kwargs={'host': host, 'port': port, 'use_reloader': False, 'threaded': True},
        daemon=True,
    )
    t.start()
    print('[TASK] HTTP 服务已启动: http://%s:%d  (POST /task, GET /status, GET /video.mjpeg)' % (host, port))
    return t


if __name__ == '__main__':
    start_server()
    print('[TASK] 独立调试模式，等待任务...')
    while True:
        task = get_next_task(timeout=1.0)
        if task:
            print('[TASK] 取到任务:', task)
        time.sleep(1)
