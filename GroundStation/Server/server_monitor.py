#!/usr/bin/python3
# coding=utf8
"""服务器监控（部署在“服务器”那台机器上，供地面站中枢拉系统信息）。

提供：
    GET /api/system   完整系统信息（CPU / 内存 / 磁盘 / 网络 / 运行时间 / 主机名）
    GET /status       轻量在线探测（中枢用它算延迟，尽量少干活）

启动：
    python3 server_monitor.py                 # 默认 0.0.0.0:20006
    python3 server_monitor.py --port 20006
    python3 server_monitor.py --host 0.0.0.0 --port 20006

依赖：
    python -m pip install -r requirements.txt   （flask + psutil [+ waitress]）
    没装 psutil 时会自动降级（CPU/内存/磁盘为空，但在线探测照常可用）。

对应中枢配置（GroundStation/Dashboard/data/config.yaml）：
    server:
      url: http://192.168.124.7:20006
      name: 服务器
      enabled: true
"""
import argparse
import os
import platform
import socket
import time

from flask import Flask, jsonify

try:
    import psutil
except ImportError:      # 没装 psutil 也能跑，只是系统指标为空
    psutil = None

app = Flask(__name__)
_START = time.time()
MB = 1048576.0
GB = 1073741824.0


def _cpu_info():
    """CPU 使用率、核心数与主频。"""
    info = {'count': os.cpu_count()}
    if psutil is not None:
        info['percent'] = round(psutil.cpu_percent(interval=0.2), 1)
        info['count_logical'] = psutil.cpu_count(logical=True)
        info['count_physical'] = psutil.cpu_count(logical=False)
        try:
            f = psutil.cpu_freq()
            if f and f.current:
                info['freq_mhz'] = int(round(f.current))
        except Exception:
            pass
    if hasattr(os, 'getloadavg'):
        try:
            info['load_avg'] = [round(x, 2) for x in os.getloadavg()]
        except Exception:
            pass
    return info


def _cpu_name():
    """处理器型号（Linux 读 /proc/cpuinfo，macOS 读 sysctl；读不到返回 None）。"""
    try:
        if os.path.exists('/proc/cpuinfo'):
            with open('/proc/cpuinfo', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    if line.lower().startswith('model name'):
                        return line.split(':', 1)[1].strip()
    except Exception:
        pass
    try:
        import subprocess
        out = subprocess.run(['sysctl', '-n', 'machdep.cpu.brand_string'],
                             capture_output=True, timeout=2)
        name = out.stdout.decode('utf-8', 'ignore').strip()
        if name:
            return name
    except Exception:
        pass
    return None


def _swap_info():
    """交换分区占用（MB）。"""
    if psutil is None:
        return {}
    try:
        s = psutil.swap_memory()
    except Exception:
        return {}
    return {
        'percent': round(s.percent, 1),
        'total_mb': round(s.total / MB, 1),
        'used_mb': round(s.used / MB, 1),
    }


def _proc_count():
    """当前进程数（拿不到返回 None）。"""
    if psutil is None:
        return None
    try:
        return len(psutil.pids())
    except Exception:
        return None


def _mem_info():
    """内存占用（MB）。"""
    if psutil is None:
        return {}
    m = psutil.virtual_memory()
    return {
        'percent': round(m.percent, 1),
        'total_mb': round(m.total / MB, 1),
        'used_mb': round(m.used / MB, 1),
        'available_mb': round(m.available / MB, 1),
    }


def _disk_info():
    """磁盘占用（GB）。"""
    if psutil is None:
        return {}
    try:
        d = psutil.disk_usage(os.path.abspath(os.sep))
    except Exception:
        return {}
    return {
        'percent': round(d.percent, 1),
        'total_gb': round(d.total / GB, 1),
        'used_gb': round(d.used / GB, 1),
        'free_gb': round(d.free / GB, 1),
    }


def _net_info():
    """网卡累计流量（MB，拿不到就省略）。"""
    if psutil is None:
        return {}
    try:
        n = psutil.net_io_counters()
        return {'sent_mb': round(n.bytes_sent / MB, 1),
                'recv_mb': round(n.bytes_recv / MB, 1)}
    except Exception:
        return {}


def _uptime():
    """机器已运行秒数（拿不到返回 None）。"""
    if psutil is None:
        return None
    try:
        return round(time.time() - psutil.boot_time(), 1)
    except Exception:
        return None


@app.route('/api/system')
def api_system():
    """完整系统信息（中枢「服务器信息」面板用）。"""
    return jsonify({
        'ok': True,
        'host': socket.gethostname(),
        'platform': platform.platform(),
        'cpu_name': _cpu_name(),
        'python': platform.python_version(),
        'time': time.time(),
        'agent_uptime_sec': round(time.time() - _START, 1),   # 本监控服务已运行
        'uptime_sec': _uptime(),                              # 机器已运行
        'proc_count': _proc_count(),
        'cpu': _cpu_info(),
        'mem': _mem_info(),
        'swap': _swap_info(),
        'disk': _disk_info(),
        'net': _net_info(),
    })


@app.route('/status')
def status():
    """轻量在线探测（中枢算延迟用）。"""
    return jsonify({'ok': True, 'host': socket.gethostname(), 'time': time.time()})


@app.route('/')
def index():
    return ('<meta charset="utf-8"><h3>服务器监控在运行</h3>'
            '<p>主机：%s</p>'
            '<p>接口：<code>/api/system</code>　<code>/status</code></p>'
            '<p><a href="/api/system">查看系统信息</a></p>' % socket.gethostname())


def main():
    p = argparse.ArgumentParser(description='服务器监控（供地面站中枢拉系统信息）')
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=20006)
    args = p.parse_args()
    print('服务器监控已启动：http://%s:%d  （/api/system、/status）' % (args.host, args.port))
    print('请在中枢 data/config.yaml 里把 server.url 指到本机地址，例如 http://%s:%d'
          % (socket.gethostname(), args.port))
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=4)
    except ImportError:
        app.run(host=args.host, port=args.port, threaded=True)


if __name__ == '__main__':
    main()
