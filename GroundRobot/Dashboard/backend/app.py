#!/usr/bin/python3
# coding=utf8
"""地面机器人独立仪表盘：视频画面回传 + 状态监控 + 下发任务。

与 GroundStation/Dashboard（地面站中枢）分开：
- 中枢仪表盘管全局（无人机 + 机器人 + YOLO 统一视图）
- 本仪表盘只服务地面机器人自己，页面直接看到机器人摄像头回传画面

数据流：
   机器人 autonomous_pick.py 每帧 publish_frame()
       → task_server.py /video.mjpeg（MJPEG 流）
       → 本后端 /video_feed 代理转发
       → 浏览器 <img> 实时显示

启动（地面站电脑，无需 ROS）：
    python app.py
浏览器打开 http://127.0.0.1:20002
"""
import argparse
import os

import requests
from flask import Flask, jsonify, render_template, request, Response

import config

# frontend 目录：backend/ 的上一级的 frontend/
FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'frontend')
app = Flask(__name__, template_folder=FRONTEND_DIR)


@app.route('/')
def index():
    config.reload_if_changed()
    return render_template('index.html', video_enabled=config.VIDEO_ENABLED)


@app.route('/api/status')
def api_status():
    """机器人状态（代理机器人 task_server 的 GET /status，页面轮询）。"""
    config.reload_if_changed()
    try:
        r = requests.get(config.ROBOT_URL + '/status', timeout=2)
        data = r.json()
        data['online'] = True
        return jsonify(data)
    except Exception as e:
        return jsonify({'online': False, 'error': str(e)})


@app.route('/api/task', methods=['POST'])
def api_task():
    """把任务转发给机器人（POST /task）。"""
    config.reload_if_changed()
    try:
        task = request.get_json(force=True)
        r = requests.post(config.ROBOT_URL + '/task', json=task, timeout=5)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({'status': 'error', 'reason': str(e)}), 502


@app.route('/video_feed')
def video_feed():
    """机器人摄像头画面 MJPEG 代理（转发 task_server /video.mjpeg）。"""
    config.reload_if_changed()
    if not config.VIDEO_ENABLED:
        return 'video disabled', 503
    try:
        upstream = requests.get(config.ROBOT_URL + '/video.mjpeg',
                                stream=True, timeout=5)
    except Exception as e:
        return '机器人视频不可用（检查 ROBOT_URL / autonomous_pick 是否在跑）: %s' % e, 502

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


def main():
    parser = argparse.ArgumentParser(description='地面机器人独立仪表盘')
    parser.add_argument('--host', default=config.DASHBOARD_HOST)
    parser.add_argument('--port', type=int, default=config.DASHBOARD_PORT)
    args = parser.parse_args()
    print('机器人仪表盘: http://%s:%d' % (args.host, args.port))
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=8)
    except ImportError:
        # 没装 waitress 时退回 Flask 开发服务器（会打印开发服务器警告）
        app.run(host=args.host, port=args.port, threaded=True)


if __name__ == '__main__':
    main()
