# -*- coding: utf-8 -*-
"""地面站·研发进度流程图（独立网页 / 独立端口）

和「地面站中枢」（GroundStation/Dashboard，默认 20000）分开跑：
中枢负责采集数据与维护进度状态，这个服务只做一件事 ——
把中枢的研发进度画成流程图页面（带流动动画）。

启动：
    python app.py                          # 默认 0.0.0.0:20010，中枢取 127.0.0.1:20000
    python app.py --port 20010 --hub http://127.0.0.1:20000

接口：
    GET  /                    流程图页面
    GET  /api/progress/flow   代理中枢的同名接口（中枢挂了就返回离线结构）
    GET  /status              轻量健康检查（启动脚本用）

平时不用手动跑：用 GroundStation/Dashboard/scripts 下的启动脚本，
会和中枢一起被启动 / 停止 / 重启（见 scripts/common.sh）。
"""

import argparse
import os

import requests
from flask import Flask, jsonify, send_from_directory

HERE = os.path.dirname(os.path.abspath(__file__))
DASHBOARD_DIR = os.path.dirname(HERE)                 # GroundStation/Dashboard
INDEX = 'index.html'

DEFAULT_PORT = 20010
DEFAULT_HUB = 'http://127.0.0.1:20000'

HUB_URL = DEFAULT_HUB
START_AT = None

app = Flask(__name__, static_folder=None)


# ---------------- 页面 ----------------
@app.route('/')
def index():
    return send_from_directory(HERE, INDEX)


@app.route('/images/<path:filename>')
def images(filename):
    """复用中枢的 logo / 图标（同属地面站，标题栏保持一致）。"""
    return send_from_directory(os.path.join(DASHBOARD_DIR, 'images'), filename)


# ---------------- 接口 ----------------
@app.route('/api/progress/flow')
def api_progress_flow():
    """代理中枢的进度接口。

    中枢不可达时返回空结构 + hub_online=false，页面显示"中枢未连接"而不是白屏。
    """
    try:
        resp = requests.get(HUB_URL.rstrip('/') + '/api/progress/flow', timeout=3)
        data = resp.json()
        data['hub_online'] = True
        data['hub_url'] = HUB_URL
        return jsonify(data)
    except Exception as exc:                     # noqa: BLE001
        return jsonify({
            'modules': [],
            'total_percent': 0,
            'hub_online': False,
            'hub_url': HUB_URL,
            'error': '中枢不可达：%s' % exc,
        })


@app.route('/status')
def status():
    """轻量健康检查（启动脚本判断这个服务是否在跑）。"""
    import time
    up = None if START_AT is None else round(time.time() - START_AT, 1)
    return jsonify({'ok': True, 'service': 'progress', 'port': PORT,
                    'hub_url': HUB_URL, 'uptime_sec': up})


PORT = DEFAULT_PORT

if __name__ == '__main__':
    import time
    parser = argparse.ArgumentParser(description='地面站·研发进度流程图')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--hub', default=os.environ.get('HUB_URL', DEFAULT_HUB),
                        help='中枢地址（默认 %s）' % DEFAULT_HUB)
    args = parser.parse_args()

    PORT = args.port
    HUB_URL = args.hub.rstrip('/')
    START_AT = time.time()
    print('研发进度流程图: http://%s:%d  (中枢 %s)' % (args.host, PORT, HUB_URL), flush=True)
    try:
        from waitress import serve
        serve(app, host=args.host, port=PORT, threads=6)
    except ImportError:
        app.run(host=args.host, port=PORT, threaded=True)
