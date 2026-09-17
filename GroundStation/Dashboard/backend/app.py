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
浏览器打开 http://localhost:20000

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

# Windows 静默运行：pythonw 无控制台时 stdout/stderr 为 None，重定向到日志避免 print 崩溃
import sys as _sys
if _sys.stdout is None or _sys.stderr is None:
    import os as _os
    _log = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'logs', 'server.log')
    _os.makedirs(_os.path.dirname(_log), exist_ok=True)
    _f = open(_log, 'a', encoding='utf-8', buffering=1)
    if _sys.stdout is None:
        _sys.stdout = _f
    if _sys.stderr is None:
        _sys.stderr = _f

import config

# frontend 目录：backend/ 的上一级的 frontend/
FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'frontend')
IMAGES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'images')
app = Flask(__name__, template_folder=FRONTEND_DIR)


@app.route('/common.css')
def common_css():
    """统一的通用样式文件（无人机/地面站共用）。"""
    return send_file(os.path.join(FRONTEND_DIR, 'common.css'), mimetype='text/css')


@app.route('/images/<path:filename>')
def images(filename):
    """仪表盘图片资源（logo 等，来自 images/ 目录）。"""
    return send_file(os.path.join(IMAGES_DIR, filename))

@app.route('/api/config', methods=['GET', 'POST'])
def api_config():
    """读取/保存 data/config.yaml（页面右上角「配置」按钮用，表单形式）。"""
    import yaml
    if request.method == 'GET':
        try:
            with open(config._CONFIG_YAML, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return jsonify(data)
        except Exception as e:
            return jsonify({'error': str(e)}), 500
    data = request.get_json(force=True)
    if not data:
        return jsonify({'status': 'error', 'reason': '配置为空，未保存'}), 400
    try:
        with open(config._CONFIG_YAML, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        config._last_mtime = None   # 强制热重载
        config.reload_if_changed()
        return jsonify({'status': 'ok'})
    except Exception as e:
        return jsonify({'status': 'error', 'reason': str(e)}), 500



# ---------------- 无人机状态（pymavlink 读 MAVLink，后台线程） ----------------
drone_status = {'online': False, 'message': '等待无人机数据…'}
robot_status = {'online': False}  # 机器人连接状态（机器人监听线程更新）

# 通过局域网广播自动发现的无人机仪表盘 / YOLO 模型仪表盘
DISCOVERED_DRONE = {'url': '', 'name': '', 'ip': '', 'last_seen': 0}
DISCOVERED_YOLO = {'url': '', 'name': '', 'ip': '', 'last_seen': 0}
BEACON_PORT = 20004


# ---------------- 任务进度：按步骤持久化 + 各自计时 ----------------
# 每个模块 6 步，每步独立：是否激活、开始时间（可清除）、时长（分钟）、编号顺序。
# 进度 = 最高激活步 × 步宽 + 该步按自身时长推进的百分比。
# 持久化到 data/progress.json（start 存绝对时间戳，重启后接着走，停机时间也算）。
STEP_NAMES = {
    # 第一阶段验证可行性（手动链路）
    'phase1': ['手动操控无人机巡检', '人工传输巡检信息', '手动操控机器人捕获'],
    # 第二阶段研发自动化系统 —— 四个模块（与《研发工单》流程图逐字一致）
    'drone': ['自动直线飞行', 'S型提高巡检效率', '多机共检：修理飞机'],
    'yolo': ['拍照采样、数据标注', '欠拟合模型训练', '正常模型训练', '过拟合模型训练'],
    'hub': ['网络连线', '网络配置', '架设平台', '安装系统', '配置环境', '部署软件'],
    'robot': ['开发稳压电路板', '视觉追踪', '自主寻路', '自动抓取'],
}
DEFAULT_DURATION_MIN = 0  # 每步默认时长（分钟）——已取消倒计时：全部即时完成
# 各模块步骤的触发方式覆盖（无倒计时：手动勾选 / 触发关键字到达即完成）
STEP_OVERRIDES = {
    # 全部无倒计时：勾选 / 触发关键字到达即完成
    'phase1': {
        0: {'duration_min': 0}, 1: {'duration_min': 0}, 2: {'duration_min': 0},
    },
    'drone': {
        0: {'duration_min': 0},                            # 自动直线飞行
        1: {'duration_min': 0},                            # S型提高巡检效率
        2: {'trigger': 'flight', 'duration_min': 0},       # 多机共检：修理飞机：位置变动触发即完成
    },
    'yolo': {
        0: {'duration_min': 0}, 1: {'duration_min': 0},    # 拍照采样、数据标注 / 欠拟合模型训练
        2: {'duration_min': 0},                            # 正常模型训练
        3: {'duration_min': 0},                            # 过拟合模型训练：模型脚本跑完即勾
    },
    'hub': {
        0: {'duration_min': 0}, 1: {'duration_min': 0}, 2: {'duration_min': 0},   # 连线/配置/架设平台
        3: {'duration_min': 0}, 4: {'duration_min': 0},                           # 安装系统/配置环境
        5: {'trigger': 'all_connected', 'duration_min': 0},                       # 部署软件：三端全连接即完成
    },
    'robot': {
        0: {'duration_min': 0}, 1: {'duration_min': 0}, 2: {'duration_min': 0},   # 电路板/视觉追踪/自主寻路
        3: {'trigger': 'CAPTURE', 'duration_min': 0, 'end_keyword': 'END'},       # 自动抓取：CAPTURE 触发、END 结束
    },
}
PROGRESS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '..', 'data', 'progress.json')
STEPS_LOCK = threading.Lock()


def _default_steps():
    out = {}
    for m, names in STEP_NAMES.items():
        out[m] = []
        for i, n in enumerate(names):
            ov = STEP_OVERRIDES.get(m, {}).get(i, {})
            out[m].append({
                'order': i,
                'name': n,
                'duration_min': ov.get('duration_min', DEFAULT_DURATION_MIN),
                'delay_min': ov.get('delay_min', 0),
                'trigger': ov.get('trigger', 'manual'),
                'end_keyword': ov.get('end_keyword'),
                'active': False,
                'start': False,
                'done': False,
            })
    return out


def _load_progress():
    """读取 data/progress.json。

    trigger / duration_min / delay_min 这些「步骤定义」始终用代码默认（STEP_OVERRIDES），
    只从文件恢复运行时状态 active / start / done，避免改代码后旧 JSON 覆盖新配置。
    """
    steps = _default_steps()
    try:
        with open(PROGRESS_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if isinstance(data, dict):
            for m, saved_list in data.items():
                if m not in steps or not isinstance(saved_list, list):
                    continue
                by_order = {s['order']: s for s in saved_list if isinstance(s, dict)}
                for s in steps[m]:
                    saved = by_order.get(s['order'])
                    if saved:
                        s['active'] = bool(saved.get('active', False))
                        s['start'] = saved.get('start') or False
                        s['done'] = bool(saved.get('done', False))
    except Exception as e:
        print('[进度] 读取 progress.json 失败，用默认值：%s' % e, flush=True)
    return steps


def _save_progress():
    """把各步骤状态写回 progress.json（start 为绝对时间戳，可随时清除）。"""
    try:
        os.makedirs(os.path.dirname(PROGRESS_FILE), exist_ok=True)
        with open(PROGRESS_FILE, 'w', encoding='utf-8') as f:
            json.dump(STEPS, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print('[进度] 保存失败：%s' % e, flush=True)


STEPS = _load_progress()


def _activate_step(module, order):
    """激活第 order 步（只标记该步，之后的清空；之前的步骤不动，由进度推导为已完成）。

    是否立即完成由该步的 duration_min 决定（0 = 立即完成，>0 = 跑计时器）。
    """
    now = time.time()
    with STEPS_LOCK:
        for s in STEPS[module]:
            if s['order'] == order:
                s['active'] = True
                s['done'] = False
                s['start'] = now
            elif s['order'] > order:
                s['active'] = False
                s['start'] = False
                s['done'] = False
        _save_progress()


def _deactivate_step(module, order):
    """取消第 order 步：该步及其之后的步骤清空（回退）。"""
    with STEPS_LOCK:
        for s in STEPS[module]:
            if s['order'] >= order:
                s['active'] = False
                s['start'] = False
                s['done'] = False
        _save_progress()


def _reset_all():
    """一键初始化：所有模块所有步骤清空，进度全部归零。"""
    with STEPS_LOCK:
        for m in STEPS:
            for s in STEPS[m]:
                s['active'] = False
                s['start'] = False
                s['done'] = False
        _save_progress()


def _mark_done(s):
    """标记步骤完成并落盘（auto 步为派生，不落盘 done）。"""
    if s.get('trigger') == 'auto':
        return
    with STEPS_LOCK:
        s['done'] = True
        _save_progress()


END_TIMEOUT_FRAC = 0.94  # 有 end_keyword 的步骤超时后停在 99%（最后一格 frac 0.94 ≈ 99%）


def _step_frac(s, eff_start=None):
    """某步已推进的百分比（0~1）。

    先读 done：done=1 直接返回 1。done=0 时算剩余 y = 时长 - 已过时长；
    y<=0（时长 0 或已到点）就主动把 done 改成 1，后续按完成处理。
    auto 步的开始时间由上一步派生（eff_start）。
    有 end_keyword 的步骤：超时未收到结束信号时停在 99%，不标记完成。
    """
    if s.get('done'):
        return 1.0
    dur = float(s.get('duration_min') or 0)
    if dur <= 0:
        if s.get('end_keyword'):
            return 0.0  # 时长 0 但等结束信号，先停 0
        if not eff_start:
            eff_start = s.get('start')
        if eff_start and time.time() < eff_start:
            return 0.0  # 延迟(delay)还没到，停在 0
        _mark_done(s)
        return 1.0
    if not eff_start:
        eff_start = s.get('start')
    if not eff_start:
        return 0.0
    frac = (time.time() - eff_start) / (dur * 60.0)
    if frac < 0.0:
        return 0.0  # 延迟(delay)还没到，停在 0
    if frac >= 1.0:
        if s.get('end_keyword'):
            return END_TIMEOUT_FRAC  # 超时未收到 END：停在 99%
        _mark_done(s)
        return 1.0
    return frac


def _effective_start(module, s):
    """auto 步的有效开始时间 = 上一步完成时间 + 自身延迟(delay_min)；其余用自身 start。

    首次派生后把 start 记录并落盘，之后直接用记录的 start，
    保证进度「从开始时间推过去」而不会因上一步 start 变化而反复回退。
    """
    if s.get('trigger') == 'auto' and s['order'] > 0:
        if s.get('start'):
            return s.get('start')  # 已记录，直接推过去
        prev = STEPS[module][s['order'] - 1]
        pstart = prev.get('start')
        if pstart:
            delay = float(s.get('delay_min') or 0) * 60.0
            eff = pstart + float(prev.get('duration_min') or 0) * 60.0 + delay
            with STEPS_LOCK:
                if not s.get('start'):
                    s['start'] = eff
                    _save_progress()
            return eff
    return s.get('start')


def _is_step_complete(s, eff_start):
    """某步是否已完成（done 标记，或按有效开始时间已到时）。"""
    if s.get('done'):
        return True
    dur = float(s.get('duration_min') or 0)
    if dur <= 0:
        if eff_start:
            return time.time() >= eff_start  # 延迟(delay)到了才算完成
        return True
    if not eff_start:
        return False
    return time.time() - eff_start >= dur * 60.0


def _current_order(module):
    """当前有效推进到的 step 编号（沿 auto 链派生），无则 -1。"""
    steps = STEPS[module]
    n = len(steps)
    base = -1
    for i in range(n - 1, -1, -1):
        if steps[i]['active']:
            base = i
            break
    if base < 0:
        return -1
    cur = base
    while True:
        s = steps[cur]
        eff_start = _effective_start(module, s)
        if _is_step_complete(s, eff_start):
            nxt = cur + 1
            if nxt < n and steps[nxt].get('trigger') == 'auto':
                cur = nxt
                continue
        break
    return cur


def _module_percent(module):
    """某模块进度：找当前有效推进到的那一步，按自身时长推进。"""
    steps = STEPS[module]
    step_width = 100.0 / len(steps)
    cur = _current_order(module)
    if cur < 0:
        return 0.0
    s = steps[cur]
    frac = _step_frac(s, _effective_start(module, s))
    return min(100.0, cur * step_width + frac * step_width)


# ---------------- 机器人状态监听：自动触发 status 型步骤 ----------------
_status_seen = {}  # (module, order) -> 该关键字当前是否已命中（避免反复重置计时）


def _robot_status_text(data):
    """拼接机器人状态文本，供 trigger 关键字匹配。"""
    return ' '.join(str(data.get(k) or '') for k in ('state', 'last_result', 'message'))


def _trigger_status_steps(data):
    """机器人状态命中某 status 步骤关键字时，激活该步骤（只在出现瞬间触发一次）。

    有 end_keyword 的步骤：命中结束关键字（如 END）时直接标记完成。
    """
    text = _robot_status_text(data)
    for m in STEPS:
        for s in STEPS[m]:
            end_kw = s.get('end_keyword')
            if end_kw and end_kw in text:
                if s.get('active') and not s.get('done'):
                    _mark_done(s)
                continue
            trig = s.get('trigger')
            if not trig or trig in ('manual', 'auto', 'video', 'flight', 'network', 'all_connected'):
                continue
            key = (m, s['order'])
            matched = trig in text
            if matched and not _status_seen.get(key):
                _status_seen[key] = True
                _activate_step(m, s['order'])
            elif not matched:
                _status_seen[key] = False


def _robot_status_listener():
    """后台监听机器人状态，命中关键字时自动激活对应步骤（自主抓取/自动捕获等）。"""
    global robot_status
    while True:
        config.reload_if_changed()
        try:
            r = requests.get(config.ROBOT_URL + '/status', timeout=2)
            data = r.json()
        except Exception:
            robot_status = {'online': False}
            time.sleep(2)
            continue
        robot_status = dict(data)
        robot_status['online'] = True
        _trigger_status_steps(data)
        time.sleep(1)


# ---------------- 无人机监听：有画面 / S形飞行 ----------------
def _drone_video_available():
    """无人机是否正在回传画面（能读到 JPEG 帧头即认为有画面）。"""
    url = _drone_video_url().rstrip('/') + '/video_feed'
    try:
        resp = requests.get(url, stream=True, timeout=3)
    except Exception:
        return False
    try:
        buf = b''
        for chunk in resp.iter_content(chunk_size=2048):
            buf += chunk
            if b'\xff\xd8' in buf:   # JPEG SOI
                return True
            if len(buf) > 200000:
                return False
        return False
    except Exception:
        return False
    finally:
        try:
            resp.close()
        except Exception:
            pass


POSITION_CHANGE_THRESHOLD = 0.5  # 位置变动阈值（米）：x/y/z 任一轴变化超过它算起飞
POSITION_CALM_THRESHOLD = 0.1    # 平静阈值（米）：位置变化低于它算落地


def _position_span(hist):
    """窗口内 x/y/z 任一轴的最大变化（米）。"""
    if len(hist) < 3:
        return 0.0
    span = 0.0
    for i in (1, 2, 3):
        vals = [r[i] for r in hist]
        span = max(span, max(vals) - min(vals))
    return span


def _order_done(module, order):
    """第 order 步是否已完成（含 auto 步的派生完成）。order < 0 视为完成。"""
    if order < 0:
        return True
    s = STEPS[module][order]
    if s.get('active'):
        return _is_step_complete(s, _effective_start(module, s))
    if s.get('trigger') == 'auto':
        eff = _effective_start(module, s)
        return bool(eff) and _is_step_complete(s, eff)
    return False


def _next_ready_order(module, trig):
    """返回 module 中 trigger==trig、前一步已完成且自身未激活的最小 order；无则 None。"""
    for s in STEPS[module]:
        if s.get('trigger') == trig and not s.get('active') and _order_done(module, s['order'] - 1):
            return s['order']
    return None


def _activate_by_trigger(trig):
    """激活所有 trigger == trig 的步骤（无人机 video/flight 用）。"""
    for m in STEPS:
        for s in STEPS[m]:
            if s.get('trigger') == trig:
                _activate_step(m, s['order'])


def _drone_listener():
    """监听无人机：位置变动(起飞) → S形飞行(2.3)/自动S形飞行(2.5)；有画面 → 数图联传(2.4)。"""
    video_seen = False
    armed = True  # 是否平静（可触发下一次起飞）
    hist = []  # (time, x, y, z)
    while True:
        config.reload_if_changed()
        # 位置变动(起飞) → 激活就绪的 flight 步（2.3 S形飞行 或 2.5 自动S形飞行）
        order = _next_ready_order('drone', 'flight')
        if order is not None:
            st = dict(drone_status)
            pose = st.get('pose') or {}
            x, y, z = pose.get('x'), pose.get('y'), pose.get('z')
            if st.get('online') and x is not None and y is not None and z is not None:
                now = time.time()
                hist.append((now, x, y, z))
                hist = [r for r in hist if now - r[0] < 10]
                span = _position_span(hist)
                if span >= POSITION_CHANGE_THRESHOLD and armed:
                    armed = False
                    _activate_step('drone', order)
                elif span < POSITION_CALM_THRESHOLD:
                    armed = True  # 位置平静，重新武装
            else:
                armed = True
                hist = []
        else:
            armed = True
            hist = []

        # 有画面：S型提高巡检效率完成后，画面首次出现即触发（当前步骤表里没有 video 步，天然空转）
        if not _order_done('drone', 1):
            video_seen = False
        elif not video_seen and _drone_video_available():
            video_seen = True
            _activate_by_trigger('video')
        time.sleep(2)


# ---------------- 中枢监听：网络扫描 / 三端连接 ----------------
def _local_ip():
    """获取本机局域网出口 IP（WLAN 网卡）。"""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))  # UDP connect 不实际发包，只取本机出口 IP
        return s.getsockname()[0]
    except Exception:
        pass
    finally:
        s.close()
    try:
        return socket.gethostbyname(socket.gethostname())
    except Exception:
        return None


def _count_network_ips():
    """用 socket 并发 TCP connect 扫描本机 /24 子网，返回可达设备(IP)数量（>=3 判为联通）。"""
    from concurrent.futures import ThreadPoolExecutor
    local_ip = _local_ip()
    if not local_ip:
        return 0
    prefix = local_ip.rsplit('.', 1)[0]
    found = set([local_ip])  # 自己算一个
    lock = threading.Lock()
    ports = (80, 443, 22, 445, 5000, 20000, 20001, 20002, 20003)

    def probe(ip):
        for port in ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.15)
            try:
                if s.connect_ex((ip, port)) == 0:
                    with lock:
                        found.add(ip)
                    return
            except Exception:
                pass
            finally:
                s.close()

    ips = ['%s.%d' % (prefix, i) for i in range(1, 255)
           if '%s.%d' % (prefix, i) != local_ip]
    with ThreadPoolExecutor(max_workers=64) as ex:
        list(ex.map(probe, ips))
    return len(found)


def _all_connected():
    """机器人 + 无人机 + YOLO 模型全部连接/加载。"""
    return (robot_status.get('online')
            and drone_status.get('online')
            and model_stats.get('loaded'))


def _hub_listener():
    """监听中枢：网络扫描(IP>=3) → 架设平台(2)；三端全连接 → 部署软件(5)。"""
    network_seen = False
    connected_seen = False
    while True:
        config.reload_if_changed()
        # 架设平台（IP>=3）：网络配置(1)完成后，首次检测到就触发；触发后不再重复
        if not _order_done('hub', 1):
            network_seen = False
        elif not network_seen and _count_network_ips() >= 3:
            network_seen = True
            _activate_by_trigger('network')

        # 部署软件（三端全连接）：配置环境(4)完成后，首次全连接就触发；触发后不再重复
        if not _order_done('hub', 4):
            connected_seen = False
        elif not connected_seen and _all_connected():
            connected_seen = True
            _activate_by_trigger('all_connected')
        time.sleep(3)



def _discovery_loop():
    """监听无人机仪表盘 / YOLO 仪表盘的 UDP 广播（20004），自动记下它们的地址。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', BEACON_PORT))
    except Exception as e:
        print('[发现] 监听 UDP %d 失败（可能被占用）：%s' % (BEACON_PORT, e))
        return
    print('[发现] 正在监听广播（UDP %d），地面站会自动找到无人机 / YOLO 仪表盘…' % BEACON_PORT)
    last_logged = {}  # url -> 上次打印时间（每个地址最多 30 秒打印一次，避免刷屏）
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            info = json.loads(data.decode('utf-8'))
            if info.get('type') == 'agcs_drone':
                # 用“收到广播的源 IP”拼地址：无人机有几张网卡也不怕选错
                port = int(info.get('port') or 20002)
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
            elif info.get('type') == 'agcs_yolo':
                port = int(info.get('port') or 20003)
                url = 'http://%s:%d' % (addr[0], port)
                DISCOVERED_YOLO.update({
                    'url': url,
                    'name': info.get('name', ''),
                    'ip': addr[0],
                    'last_seen': time.time(),
                    'loaded': info.get('loaded'),
                    'model': info.get('model'),
                    'message': info.get('message'),
                })
                if time.time() - last_logged.get(url, 0) >= 30.0:
                    last_logged[url] = time.time()
                    state = '已加载' if info.get('loaded') else '未加载'
                    print('[发现] YOLO 仪表盘：%s（%s）模型%s（%s）'
                          % (url, addr[0], state, info.get('model') or '-'))
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


def _drone_video_url():
    """无人机视频流地址（独立视频端口，区别于仪表盘端口的遥测）。"""
    base = _effective_drone_url()
    host = base.split('://')[-1].split(':')[0]
    return 'http://%s:%d' % (host, config.DRONE_VIDEO_PORT)


def _effective_yolo_url():
    """实际使用的 YOLO 仪表盘地址：配置里写了局域网 IP 就用配置；
    配置还是 127.0.0.1/空 时，用广播自动发现的地址。"""
    base = (config.YOLO_URL or '').rstrip('/')
    host = ''
    if base:
        host = base.split('://')[-1].split(':')[0]
    disc = (DISCOVERED_YOLO.get('url') or '').rstrip('/')
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
        # 兜底路径也要歇一下：上游一挂就立即返回（例如没装 pymavlink），
        # 不 sleep 的话这里会空转打满一个 CPU 核。
        time.sleep(0.5)


def _pull_drone_dashboard():
    """从无人机电脑端仪表盘拉遥测；返回 True 表示仪表盘可达（无论无人机是否连接）。

    上游（Dron/Dashboard）支持编队多机：/api/telemetry 返回 drones 数组。
    这里逐台解析放进 drone_status['drones']；顶层同时保留第 1 台，兼容旧调用。
    """
    global drone_status
    try:
        url = _effective_drone_url().rstrip('/') + '/api/telemetry'
        resp = requests.get(url, timeout=3)
        data = resp.json()
    except Exception as e:
        drone_status = {
            'online': False,
            'message': '无人机电脑端不可达（检查 drone.url 与 20002 服务是否在跑）: %s' % e,
        }
        return False

    # 上游是多机仪表盘（drones 数组）；旧版单机则当成一台处理
    raw = data.get('drones')
    if not isinstance(raw, list) or not raw:
        raw = [data]
    drones = [_parse_drone_item(d, i) for i, d in enumerate(raw)]
    first = drones[0]
    any_online = any(d['connected'] for d in drones)

    if not data.get('connected') and not any_online:
        drone_status = {
            'online': False,
            'source': data.get('source', 'none'),
            'message': data.get('error') or '无人机电脑端未连接无人机',
            'drones': drones,
        }
        return True

    sdk = data.get('sdk') or {}
    source = data.get('source', 'sdk')
    msg = ('数据源：SDK（串口 %s）' % (sdk.get('serial') or '?')) if source == 'sdk' else '数据源：MAVLink'

    drone_status = {
        'online': True,
        'source': source,
        'message': msg,
        'drone_url': _effective_drone_url(),
        'pose': first['pose'],          # 顶层保留第 1 台，兼容旧调用方
        'battery': first['battery'],
        'obstacles': first['obstacles'],
        'key_press': first['key_press'],
        'role_news': first['role_news'],
        'drones': drones,               # 逐台数据，前端分开显示
    }
    return True


# 无人机仪表盘 /api/telemetry 里每台机可用、且中枢要原样透传给前端的字段。
# 前端按这些字段渲染与仪表盘一致的 9 张数据卡片（数据源/姿态/位置/定位误差/
# 电池/避障/遥控器按键/计时器/消息），所以必须原样带过来，不能只留换算后的值。
_DRONE_PASSTHROUGH = (
    'source', 'error', 'sdk',
    'roll', 'pitch', 'yaw',
    'loc_x', 'loc_y', 'loc_z',
    'err_x', 'err_y', 'err_z',
    'volt',
    'obs_f', 'obs_b', 'obs_l', 'obs_r',
    'key_press', 'role_news', 'role_news_id', 'timer',
)


def _parse_drone_item(d, idx):
    """整理一台无人机的遥测。

    - 仪表盘原始字段（厘米/度/伏等）原样透传：前端要按仪表盘那 9 张卡片显示，
      不能在中枢里换成别的单位，否则两边显示会不一致；
    - 另附 pose / battery / obstacles 换算值，方便其它调用方（米/伏）直接使用。
    """
    pose = {}
    if d.get('loc_x') is not None:
        pose['x'] = round(d['loc_x'] / 100.0, 2)
        pose['y'] = round(d['loc_y'] / 100.0, 2)
        pose['z'] = round(d['loc_z'] / 100.0, 2)
    if d.get('yaw') is not None:
        pose['yaw_deg'] = round(float(d['yaw']) % 360.0, 1)
    if d.get('roll') is not None:
        pose['roll_deg'] = round(float(d['roll']), 1)
        pose['pitch_deg'] = round(float(d['pitch']), 1)
    obs = {'front': d.get('obs_f'), 'back': d.get('obs_b'),
           'left': d.get('obs_l'), 'right': d.get('obs_r')}
    item = {
        'index': idx,
        'connected': bool(d.get('connected')),
        'pose': pose or None,
        'battery': ({'voltage': round(float(d['volt']), 2)}
                    if d.get('volt') is not None else None),
        'obstacles': obs if any(v is not None for v in obs.values()) else None,
        'key_press': d.get('key_press'),
        'role_news': d.get('role_news'),
        'timer': d.get('timer'),
    }
    for key in _DRONE_PASSTHROUGH:
        item[key] = d.get(key)
    return item


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

# 服务器监控状态（GroundStation/Server：CPU/内存/磁盘等；后台线程写，/api/server/status 读）
server_status = {'online': False, 'message': '等待服务器信息…'}
SERVER_LOCK = threading.Lock()
_offline_jpg = None


def _offline_frame():
    """黑底黄字状态卡：YOLO 仪表盘不可达时也推一张看得见的图，别让页面干等。

    没装 opencv/numpy 时（例如中枢部署在没有这些依赖的机器上）返回 None，
    由调用方退化成文字帧 —— 这里绝不能抛异常，否则 /video.mjpeg 会 500。
    """
    global _offline_jpg
    if _offline_jpg is None:
        try:
            import cv2
            import numpy as np
        except ImportError as e:
            print('[视频] 缺少 opencv/numpy（%s），离线状态卡退化为文字帧' % e, flush=True)
            _offline_jpg = b''          # 记成空，避免每帧都重复打印
        else:
            img = np.zeros((480, 720, 3), dtype=np.uint8)
            for i, text in enumerate(['检测画面暂不可用（等待视频流）',
                                      'local 模式：检查 drone.url / Dron Dashboard 是否在跑',
                                      'remote 模式：检查 yolo.url / YOLOModel Dashboard 是否在跑']):
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


_yolo_last_log = 0.0


def _log_yolo_connect(url, e):
    """YOLO 连接失败的日志（30 秒限频，完整异常只进日志，不进前端）。"""
    global _yolo_last_log
    if time.time() - _yolo_last_log >= 30.0:
        _yolo_last_log = time.time()
        print('[YOLO] 连接失败 %s: %s' % (url, e), flush=True)


def _pull_yolo_stats(url):
    """从 YOLO 仪表盘拉检测统计，写入 model_stats。"""
    global model_stats
    try:
        resp = requests.get(url + '/api/detections', timeout=3)
        data = resp.json()
        data['yolo_url'] = url
        model_stats = data
    except Exception as e:
        _log_yolo_connect(url, e)
        model_stats = {'loaded': False,
                       'message': 'YOLO 仪表盘未连接（检查 yolo.url / YOLOModel Dashboard 是否运行）'}


def _yolo_monitor():
    """拉 YOLO 模型仪表盘的标注画面转发到 /video.mjpeg（只负责视频）。"""
    global latest_jpeg, model_stats
    try:
        import cv2
        import numpy as np
    except ImportError as e:
        # 别让线程直接崩掉（崩了是静默的，看不出原因）：给出可读的提示后退出
        model_stats = {'loaded': False,
                       'message': '缺少 opencv/numpy（%s），YOLO 画面转发已停用' % e}
        print('[YOLO] %s' % model_stats['message'], flush=True)
        return

    while True:
        config.reload_if_changed()
        if not config.YOLO_VIDEO_ENABLED:
            time.sleep(1.0)
            continue
        url = _effective_yolo_url().rstrip('/')
        try:
            for jpg in _iter_mjpeg_frames(url + '/video_feed'):
                config.reload_if_changed()
                if not config.YOLO_VIDEO_ENABLED:
                    break
                if _effective_yolo_url().rstrip('/') != url:
                    break  # yolo.url 热改了，断开去连新地址
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                ok_jpg, buf = cv2.imencode('.jpg', frame,
                                           [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok_jpg:
                    with latest_jpeg_lock:
                        latest_jpeg = buf.tobytes()
        except Exception as e:
            _log_yolo_connect(url, e)
            time.sleep(2.0)
        else:
            time.sleep(1.0)


def _yolo_stats_monitor():
    """定时拉 YOLO 检测统计（独立于视频流，保证状态始终更新，即使没有画面）。"""
    while True:
        config.reload_if_changed()
        if config.YOLO_VIDEO_ENABLED:
            _pull_yolo_stats(_effective_yolo_url().rstrip('/'))
        time.sleep(2.0)


# ---------------- 服务器监控（本机 local / 远程 http） ----------------
# 采集命令：分节输出便于解析（Linux 通用工具；非 Linux 会缺节，解析时自动跳过）。
# local 模式在本机执行它；http 模式则由服务器上的 GroundStation/Server 采集后中枢去拉。
_COLLECT_CMD = (
    'echo "##HOST"; hostname 2>/dev/null; '
    'echo "##PLAT"; (uname -srm 2>/dev/null || echo unknown); '
    'echo "##UP"; (cut -d" " -f1 /proc/uptime 2>/dev/null); '
    'echo "##NCPU"; (nproc 2>/dev/null || grep -c ^processor /proc/cpuinfo 2>/dev/null); '
    'echo "##CPUINFO"; (grep -m1 "model name" /proc/cpuinfo 2>/dev/null '
        '|| sysctl -n machdep.cpu.brand_string 2>/dev/null); '
    'echo "##STAT1"; grep "^cpu " /proc/stat 2>/dev/null; '
    'sleep 0.4; '
    'echo "##STAT2"; grep "^cpu " /proc/stat 2>/dev/null; '
    'echo "##MEM"; (free -m 2>/dev/null | sed -n 2p); '
    'echo "##SWAP"; (free -m 2>/dev/null | sed -n 3p); '
    'echo "##DISK"; (df -m / 2>/dev/null | tail -1); '
    'echo "##LOAD"; (cat /proc/loadavg 2>/dev/null); '
    'echo "##PROC"; (ps -e 2>/dev/null | wc -l); '
    'echo "##TIME"; date +%s 2>/dev/null; '
    'echo "##NET"; (cat /proc/net/dev 2>/dev/null | tail -n +3)'
)


def _to_float(v):
    try:
        return float(str(v).strip())
    except Exception:
        return None


def _cpu_ticks(lines):
    """从 /proc/stat 的 cpu 行取 (idle, total)。"""
    if not lines:
        return None
    parts = lines[0].split()
    if len(parts) < 5:
        return None
    nums = [_to_float(x) or 0.0 for x in parts[1:]]
    idle = nums[3] + (nums[4] if len(nums) > 4 else 0.0)   # idle + iowait
    return idle, sum(nums)


def _cpu_percent(t1, t2):
    """两次 /proc/stat 采样算 CPU 使用率。"""
    if not t1 or not t2:
        return None
    idle_d, total_d = t2[0] - t1[0], t2[1] - t1[1]
    if total_d <= 0:
        return None
    return round((1.0 - idle_d / total_d) * 100.0, 1)


def _load_avg(text):
    parts = (text or '').split()[:3]
    if not parts:
        return None
    try:
        return [round(float(x), 2) for x in parts]
    except Exception:
        return None


def _parse_sysinfo(text):
    """解析 SSH 采集的分节输出（##节名 换行 内容…）。"""
    sec, cur = {}, None
    for line in text.splitlines():
        if line.startswith('##'):
            cur = line[2:].strip()
            sec[cur] = []
        elif cur is not None:
            sec[cur].append(line)

    def first(k):
        v = sec.get(k) or []
        return v[0].strip() if v else ''

    info = {
        'host': first('HOST'),
        'platform': first('PLAT'),
        'cpu_name': first('CPUINFO').split(':', 1)[-1].strip() or None,
        'uptime_sec': _to_float(first('UP')),
        'server_time': _to_float(first('TIME')),
        'proc_count': int(_to_float(first('PROC')) or 0) or None,
        'cpu': {
            'count': int(_to_float(first('NCPU')) or 0) or None,
            'percent': _cpu_percent(_cpu_ticks(sec.get('STAT1')), _cpu_ticks(sec.get('STAT2'))),
            'load_avg': _load_avg(first('LOAD')),
        },
        'mem': {}, 'swap': {}, 'disk': {}, 'net': {},
    }
    # free -m 第 2 行：Mem: total used free shared buff/cache available
    mem = (sec.get('MEM') or [''])[0].split()
    if len(mem) >= 3 and mem[0].startswith('Mem'):
        total, used = _to_float(mem[1]), _to_float(mem[2])
        info['mem'] = {
            'total_mb': total, 'used_mb': used,
            'available_mb': _to_float(mem[6]) if len(mem) > 6 else None,
            'percent': round(used / total * 100.0, 1) if total else None,
        }
    # free -m 第 3 行：Swap: total used free
    swap = (sec.get('SWAP') or [''])[0].split()
    if len(swap) >= 3 and swap[0].startswith('Swap'):
        s_total, s_used = _to_float(swap[1]), _to_float(swap[2])
        info['swap'] = {
            'total_mb': s_total, 'used_mb': s_used,
            'percent': round(s_used / s_total * 100.0, 1) if s_total else 0.0,
        }
    # df -m /：Filesystem 1M-blocks Used Available Use% Mounted
    disk = ((sec.get('DISK') or [''])[0] or '').split()
    if len(disk) >= 4:
        total_mb, used_mb = _to_float(disk[1]), _to_float(disk[2])
        free_mb = _to_float(disk[3])
        info['disk'] = {
            'total_gb': round(total_mb / 1024.0, 1) if total_mb else None,
            'used_gb': round(used_mb / 1024.0, 1) if used_mb else None,
            'free_gb': round(free_mb / 1024.0, 1) if free_mb else None,
            'percent': _to_float(disk[4].replace('%', '')) if len(disk) > 4 else None,
        }
    # /proc/net/dev 逐网卡累加（第 1 列=收字节，第 9 列=发字节）
    rx = tx = 0.0
    for line in sec.get('NET') or []:
        if ':' not in line:
            continue
        cols = line.split(':', 1)[1].split()
        if len(cols) >= 9:
            rx += _to_float(cols[0]) or 0.0
            tx += _to_float(cols[8]) or 0.0
    info['net'] = {'recv_mb': round(rx / 1048576.0, 1), 'sent_mb': round(tx / 1048576.0, 1)}
    return info


def _local_ips():
    """本机所有可识别 IP / 主机名（判断 server.url 是否指向本机用）。"""
    ips = {'127.0.0.1', 'localhost', '::1', '0.0.0.0'}
    try:
        name = socket.gethostname()
        ips.add(name)
        for ip in socket.gethostbyname_ex(name)[2]:
            ips.add(ip)
    except Exception:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))          # 只为拿本机出口 IP，不会真的发包
        ips.add(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    return ips


def _is_local_url(url):
    """server.url 是否指向本机（中枢和服务器是同一台电脑）。"""
    if not url:
        return False
    try:
        host = url.split('://')[-1].split('/')[0].split(':')[0].strip()
    except Exception:
        return False
    return host in _local_ips()


def _local_collect_psutil(p):
    """用 psutil 读本机指标（跨平台，Windows/Linux/macOS 都能用）。"""
    import platform as _pf
    try:
        percent = round(p.cpu_percent(interval=0.3), 1)
    except Exception:
        percent = None
    vm = p.virtual_memory()
    du = p.disk_usage(os.path.abspath(os.sep))
    try:
        nio = p.net_io_counters()
        net = {'recv_mb': round(nio.bytes_recv / 1048576.0, 1),
               'sent_mb': round(nio.bytes_sent / 1048576.0, 1)}
    except Exception:
        net = {}
    try:
        load = [round(x, 2) for x in os.getloadavg()]
    except Exception:
        load = None
    try:
        uptime = round(time.time() - p.boot_time(), 1)
    except Exception:
        uptime = None
    cpu = {'count': p.cpu_count(logical=True), 'percent': percent, 'load_avg': load}
    try:
        cpu['count_physical'] = p.cpu_count(logical=False)
    except Exception:
        pass
    try:
        freq = p.cpu_freq()
        if freq and freq.current:
            cpu['freq_mhz'] = int(round(freq.current))
    except Exception:
        pass
    # 交换分区 / 进程数（拿不到就留空，前端显示 "-"）
    try:
        sw = p.swap_memory()
        swap = {'percent': round(sw.percent, 1),
                'total_mb': round(sw.total / 1048576.0, 1),
                'used_mb': round(sw.used / 1048576.0, 1)}
    except Exception:
        swap = {}
    try:
        proc_count = len(p.pids())
    except Exception:
        proc_count = None
    cpu_name = None
    try:                       # 处理器型号（Linux 可读 /proc/cpuinfo；其它平台交给 psutil 兜底）
        if os.path.exists('/proc/cpuinfo'):
            with open('/proc/cpuinfo', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    if line.lower().startswith('model name'):
                        cpu_name = line.split(':', 1)[1].strip()
                        break
    except Exception:
        pass
    return {
        'host': socket.gethostname(),
        'platform': _pf.platform(),
        'cpu_name': cpu_name,
        'uptime_sec': uptime,
        'server_time': time.time(),
        'proc_count': proc_count,
        'cpu': cpu,
        'mem': {'percent': round(vm.percent, 1),
                'total_mb': round(vm.total / 1048576.0, 1),
                'used_mb': round(vm.used / 1048576.0, 1),
                'available_mb': round(vm.available / 1048576.0, 1)},
        'swap': swap,
        'disk': {'percent': round(du.percent, 1),
                 'total_gb': round(du.total / 1073741824.0, 1),
                 'used_gb': round(du.used / 1073741824.0, 1),
                 'free_gb': round(du.free / 1073741824.0, 1)},
        'net': net,
    }


def _local_collect(timeout=8):
    """本机采集：优先 psutil（跨平台），没装则退回 shell 命令（Linux/macOS）。

    返回 (info, latency)；本机无网络往返，延迟不适用（None）。
    """
    try:
        import psutil
        return _local_collect_psutil(psutil), None
    except ImportError:
        pass
    import subprocess
    out = subprocess.run(['/bin/sh', '-c', _COLLECT_CMD],
                         capture_output=True, timeout=timeout)
    text = out.stdout.decode('utf-8', 'ignore')
    return _parse_sysinfo(text), None


def _probe_server_local():
    """本机采集并组装成面板状态。"""
    info, _latency = _local_collect()
    return {
        'online': True,
        'mode': 'local',
        'url': '本机',
        'latency_ms': None,
        'host': info.get('host') or socket.gethostname(),
        'platform': info.get('platform'),
        'cpu_name': info.get('cpu_name'),
        'uptime_sec': info.get('uptime_sec'),
        'server_time': info.get('server_time'),
        'proc_count': info.get('proc_count'),
        'cpu': info.get('cpu') or {},
        'mem': info.get('mem') or {},
        'swap': info.get('swap') or {},
        'disk': info.get('disk') or {},
        'net': info.get('net') or {},
        'message': '本机采集（中枢与服务器是同一台）',
    }


def _probe_server_http():
    """HTTP 兜底：拉服务器上 GroundStation/Server 的 /api/system（拿不到就退 /status）。"""
    url = (config.SERVER_URL or '').rstrip('/')
    started = time.time()
    try:
        data = requests.get(url + '/api/system', timeout=3).json()
        return {
            'online': bool(data.get('ok', True)),
            'mode': 'http',
            'url': url,
            'latency_ms': round((time.time() - started) * 1000.0, 1),
            'host': data.get('host'),
            'platform': data.get('platform'),
            'cpu_name': data.get('cpu_name'),
            'uptime_sec': data.get('uptime_sec'),
            'server_time': data.get('time'),
            'proc_count': data.get('proc_count'),
            'cpu': data.get('cpu') or {},
            'mem': data.get('mem') or {},
            'swap': data.get('swap') or {},
            'disk': data.get('disk') or {},
            'net': data.get('net') or {},
            'message': 'HTTP 已连接',
        }
    except Exception:
        t0 = time.time()
        try:
            requests.get(url + '/status', timeout=2)
            return {'online': True, 'mode': 'http', 'url': url,
                    'latency_ms': round((time.time() - t0) * 1000.0, 1),
                    'message': 'HTTP 在线（未提供 /api/system，无系统指标）'}
        except Exception as e:
            return {'online': False, 'mode': 'http', 'url': url,
                    'message': '服务器不可达：%s' % e}


def _server_monitor():
    """后台线程：采集服务器信息。

    模式（config.SERVER_MODE）：
      local → 读本机（中枢和服务器同一台）；http → 拉 server.url；
      auto  → server.url 指向本机就用 local，否则 http。
    """
    global server_status
    while True:
        config.reload_if_changed()
        name = config.SERVER_NAME
        url = (config.SERVER_URL or '').rstrip('/')
        mode = (config.SERVER_MODE or 'auto').lower()
        if mode not in ('auto', 'local', 'http'):
            mode = 'auto'
        if mode == 'auto':
            mode = 'local' if _is_local_url(url) else 'http'
        interval = 2.0
        if not config.SERVER_ENABLED:
            status = {'online': False, 'enabled': False, 'name': name,
                      'message': '服务器面板已关闭（config.yaml 的 server.enabled=false）'}
        elif mode == 'local':
            try:
                status = _probe_server_local()
            except Exception as e:
                status = {'online': False, 'mode': 'local', 'url': '本机',
                          'message': '本机采集失败：%s' % e}
                interval = 3.0
        elif url:
            status = _probe_server_http()
        else:
            status = {'online': False, 'mode': 'http',
                      'message': '未配置 server.url（http 模式需要）'}
            interval = 3.0
        status['name'] = status.get('name') or name
        status['enabled'] = config.SERVER_ENABLED
        status['checked_at'] = time.time()
        _add_net_rate(status)
        with SERVER_LOCK:
            server_status = status
        time.sleep(interval)


# 上一次的网卡累计流量，用来算实时速率（三种采集方式通用，不用服务器端支持）
_NET_PREV = {'t': 0.0, 'recv_mb': None, 'sent_mb': None}
_AGENT_T0 = time.time()          # 中枢开始监控服务器的时刻


def _add_net_rate(status):
    """用相邻两次采样的差值算网卡实时速率（KB/s），并附上中枢已监控时长。"""
    net = status.get('net') or {}
    now = time.time()
    recv_mb, sent_mb = net.get('recv_mb'), net.get('sent_mb')
    prev = _NET_PREV
    if (recv_mb is not None and sent_mb is not None
            and prev['recv_mb'] is not None and prev['sent_mb'] is not None):
        dt = now - prev['t']
        if dt >= 0.5:
            net['recv_kbps'] = round(max(0.0, (recv_mb - prev['recv_mb']) * 1024.0 / dt), 1)
            net['sent_kbps'] = round(max(0.0, (sent_mb - prev['sent_mb']) * 1024.0 / dt), 1)
    prev.update({'t': now, 'recv_mb': recv_mb, 'sent_mb': sent_mb})
    status['net'] = net
    status['agent_uptime_sec'] = round(now - _AGENT_T0, 1)
    return status


# ---------------- 路由 ----------------

@app.route('/')
def index():
    config.reload_if_changed()
    # 独立端口的「研发进度流程图」网页地址（同机不同端口，由同一套脚本一起启停）
    host = (request.host or '').split(':')[0] or '127.0.0.1'
    return render_template('index.html',
                           robot_video_enabled=config.ROBOT_VIDEO_ENABLED,
                           drone_video_enabled=config.DRONE_VIDEO_ENABLED,
                           yolo_video_enabled=config.YOLO_VIDEO_ENABLED,
                           progress_url='http://%s:%d' % (host, config.PROGRESS_PORT))


@app.route('/api/drone/status')
def api_drone():
    data = dict(drone_status)
    # 统一形状：无论哪种情况（SDK/MAVLink/不可达）drones 都是数组，前端不用再判空
    data['drones'] = data.get('drones') or []
    data['discovery'] = {
        'url': DISCOVERED_DRONE.get('url', ''),
        'name': DISCOVERED_DRONE.get('name', ''),
        'ip': DISCOVERED_DRONE.get('ip', ''),
        'last_seen': DISCOVERED_DRONE.get('last_seen', 0),
    }
    return jsonify(data)


@app.route('/api/drone/history')
def api_drone_history():
    """代理无人机仪表盘的 /api/history：每台机的姿态/高度曲线。

    无人机面板的曲线图和仪表盘画的是同一份数据，所以直接透传上游结构
    （{'drones': [{'t': [...], 'roll': [...], 'pitch': [...], 'yaw': [...], 'alt': [...]}]}）。
    """
    try:
        url = _effective_drone_url().rstrip('/') + '/api/history'
        data = requests.get(url, timeout=3).json()
    except Exception as e:
        return jsonify({'drones': [], 'error': str(e)})
    if isinstance(data, dict) and isinstance(data.get('drones'), list):
        return jsonify(data)
    # 旧版单机仪表盘：整份当成第 1 台
    return jsonify({'drones': [data] if data else []})


@app.route('/api/model/status')
def api_model():
    return jsonify(model_stats)


@app.route('/api/server/status')
def api_server_status():
    """服务器监控状态（「服务器信息」面板轮询）。"""
    config.reload_if_changed()
    with SERVER_LOCK:
        data = dict(server_status)
    data['name'] = data.get('name') or config.SERVER_NAME
    data['url'] = data.get('url') or config.SERVER_URL
    data['enabled'] = config.SERVER_ENABLED
    return jsonify(data)


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


@app.route('/drone_video_feed')
def drone_video_feed():
    """无人机图传画面 MJPEG 代理（转发 Dron/Dashboard 的 /video_feed?cam=N）。

    ?cam=0 / ?cam=1 分别对应无人机仪表盘的两路画面；不带参数默认第 1 路。
    """
    config.reload_if_changed()
    if not config.DRONE_VIDEO_ENABLED:
        return 'drone video disabled', 503
    try:
        cam = int(request.args.get('cam', 0))
    except ValueError:
        cam = 0
    cam = max(0, cam)
    upstream_url = '%s/video_feed?cam=%d' % (_drone_video_url().rstrip('/'), cam)
    try:
        upstream = requests.get(upstream_url, stream=True, timeout=5)
    except Exception as e:
        return '无人机视频不可用（检查 drone.url / Dron Dashboard 是否运行）: %s' % e, 502

    def gen():
        try:
            for chunk in upstream.iter_content(chunk_size=1024):
                if chunk:
                    yield chunk
        finally:
            upstream.close()

    ctype = upstream.headers.get('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
    return Response(gen(), mimetype=ctype)


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
    """无人机画面 MJPEG 预览（YOLO 仪表盘转发已标注帧）。"""
    config.reload_if_changed()
    if not config.YOLO_VIDEO_ENABLED:
        return 'yolo video disabled', 503

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
            time.sleep(1.0 / max(config.VIDEO_FPS_LIMIT, 1))
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')




@app.route('/api/progress/event', methods=['POST'])
def api_progress_event():
    """脚本发来的进度信号：激活对应步骤（step 为 0 起的编号顺序），返回确认。"""
    data = request.get_json(force=True)
    module = data.get('module')
    if module not in STEPS:
        return jsonify({'status': 'error', 'reason': '未知模块'}), 400
    order = int(data.get('step', 0))
    _activate_step(module, order)
    print('[进度] %s 进入第 %d 步' % (module, order), flush=True)
    return jsonify({'status': 'ok'})


@app.route('/api/progress/status')
def api_progress_status():
    """各模块当前进度百分比（前端轮询）。"""
    result = {m: round(_module_percent(m), 1) for m in STEPS}
    return jsonify(result)


# 流程图（研发工单）：第一阶段（手动链路）→ 第二阶段的四个模块，顺序即开发流程图里的顺序
FLOW_MODULES = (
    ('phase1', '第一阶段验证可行性'),
    ('drone', '无人机自动巡检'),
    ('yolo', '模型训练'),
    ('hub', '中枢互通'),
    ('robot', '机器人自动捕获'),
)


@app.route('/api/progress/flow')
def api_progress_flow():
    """研发进度流程图数据：每个模块的每一步状态（done / active / pending）+ 模块进度。

    前端把这份数据画成流程图（方框 + 连线），并按状态上色/加流动动画。
    """
    now = time.time()
    modules = []
    for key, name in FLOW_MODULES:
        steps = STEPS.get(key) or []
        cur = _current_order(key)
        items = []
        for i, s in enumerate(steps):
            if s.get('done') or (cur >= 0 and i < cur):
                state = 'done'
            elif cur >= 0 and i == cur:
                state = 'active'
            else:
                state = 'pending'
            items.append({
                'order': i,
                'name': s.get('name'),
                'state': state,
                'trigger': s.get('trigger'),
                'duration_min': float(s.get('duration_min') or 0),
            })
        timing = None
        if 0 <= cur < len(steps):
            s = steps[cur]
            eff = _effective_start(key, s)
            dur = float(s.get('duration_min') or 0)
            delay = float(s.get('delay_min') or 0)
            if not eff:
                txt, remain = '等待开始', delay + dur
            elif now < eff:
                txt, remain = '延迟中', (eff - now) / 60.0 + dur
            elif dur > 0 and now < eff + dur * 60.0:
                txt, remain = '倒计时中', (eff + dur * 60.0 - now) / 60.0
            else:
                txt, remain = '已完成', 0.0
            timing = {'state': txt, 'remaining_min': round(remain, 2),
                      'duration_min': dur, 'delay_min': delay}
        modules.append({
            'key': key,
            'name': name,
            'percent': round(_module_percent(key), 1),
            'current': cur,
            'current_name': (steps[cur].get('name') if 0 <= cur < len(steps) else '-'),
            'timing': timing,
            'steps': items,
        })
    total = round(sum(m['percent'] for m in modules) / len(modules), 1) if modules else 0.0
    return jsonify({'modules': modules, 'total_percent': total, 'checked_at': now})


@app.route('/api/progress/detail')
def api_progress_detail():
    """各模块当前步骤的计时详情（前端控制台打印倒计时调试用）。"""
    result = {}
    now = time.time()
    for m in STEPS:
        cur = _current_order(m)
        if cur < 0:
            result[m] = {'order': -1, 'name': '-', 'state': '未开始', 'percent': round(_module_percent(m), 1)}
            continue
        s = STEPS[m][cur]
        eff = _effective_start(m, s)
        dur = float(s.get('duration_min') or 0)
        delay = float(s.get('delay_min') or 0)
        if not eff:
            state = '等待开始'
            remaining = delay + dur
        elif now < eff:
            state = '延迟中'
            remaining = (eff - now) / 60.0 + dur
        elif dur > 0 and now < eff + dur * 60.0:
            state = '倒计时中'
            remaining = (eff + dur * 60.0 - now) / 60.0
        else:
            state = '已完成'
            remaining = 0.0
        result[m] = {
            'order': cur,
            'name': s.get('name'),
            'trigger': s.get('trigger'),
            'delay_min': delay,
            'duration_min': dur,
            'state': state,
            'remaining_min': round(remaining, 2),
            'percent': round(_module_percent(m), 1),
        }
    return jsonify(result)


@app.route('/api/progress/skip', methods=['POST'])
def api_progress_skip():
    """临时跳过所有 auto 步（把 start 拨到过去，立即完成；触发类步骤仍按触发）。"""
    now = time.time()
    skipped = []
    for m in STEPS:
        for s in STEPS[m]:
            if s.get('trigger') == 'auto':
                dur = float(s.get('duration_min') or 0)
                delay = float(s.get('delay_min') or 0)
                s['start'] = now - (delay + dur) * 60.0 - 10
                skipped.append(m + ' → ' + s.get('name', ''))
    _save_progress()
    return jsonify({'status': 'ok', 'skipped': skipped})


@app.route('/api/hub/gate')
def api_hub_gate():
    """中枢是否已到达 2.3（搭建网络）—— 控制显示无人机/机器人/YOLO 三画面。"""
    return jsonify({'show_three': _current_order('hub') >= 3})


@app.route('/api/checklist')
def api_checklist():
    """返回各模块已激活的步骤编号（前端恢复清单勾选）。"""
    return jsonify({m: [s['order'] for s in STEPS[m] if s['active']] for m in STEPS})


@app.route('/api/checklist/toggle', methods=['POST'])
def api_checklist_toggle():
    """勾选/取消清单项：激活/取消对应步骤，并立刻推进/回退进度条。"""
    data = request.get_json(force=True)
    module = data.get('module')
    if module not in STEPS:
        return jsonify({'status': 'error', 'reason': '未知模块'}), 400
    order = int(data.get('order', 0))
    checked = bool(data.get('checked', True))
    if checked:
        _activate_step(module, order)  # 是否立即完成由该步 duration_min 决定（0 = 立即）
    else:
        _deactivate_step(module, order)
    return jsonify({'status': 'ok',
                    'checklist': {m: [s['order'] for s in STEPS[m] if s['active']] for m in STEPS}})


@app.route('/api/checklist/reset', methods=['POST'])
def api_checklist_reset():
    """一键初始化：清空所有步骤，进度条全部归零。"""
    _reset_all()
    return jsonify({'status': 'ok'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='地面站仪表盘')
    parser.add_argument('--host', default=config.DASHBOARD_HOST)
    parser.add_argument('--port', type=int, default=config.DASHBOARD_PORT)
    args = parser.parse_args()

    threading.Thread(target=_drone_monitor, daemon=True).start()
    threading.Thread(target=_yolo_monitor, daemon=True).start()
    threading.Thread(target=_server_monitor, daemon=True).start()
    threading.Thread(target=_yolo_stats_monitor, daemon=True).start()
    threading.Thread(target=_discovery_loop, daemon=True).start()
    threading.Thread(target=_robot_status_listener, daemon=True).start()
    threading.Thread(target=_drone_listener, daemon=True).start()
    threading.Thread(target=_hub_listener, daemon=True).start()
    print('地面站仪表盘: http://%s:%d' % (args.host, args.port), flush=True)
    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port, threads=8)
    except ImportError:
        app.run(host=args.host, port=args.port, threaded=True)
