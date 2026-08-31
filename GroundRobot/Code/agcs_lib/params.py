#!/usr/bin/python3
# coding=utf8
"""参数加载：封装 config/robot_params.yaml。

- load_params：加载 + 文本级重复键校验（PyYAML 会静默取最后一个值，
  重复键是实机行为异常的常见来源，必须显式报错）；
- summarize：生成当前算法关键生效参数摘要，供启动时打印确认。
"""
import os
import re
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import yaml

_DEFAULT_PATH = os.path.join(_PKG_ROOT, 'config', 'robot_params.yaml')

# 顶层段：无缩进的 "section:"；段内键：恰好 2 空格缩进的 "key:"
_TOP_RE = re.compile(r'^([A-Za-z_][\w]*):\s*(?:#.*)?$')
_KEY_RE = re.compile(r'^  ([A-Za-z_][\w]*):')


def _check_duplicate_keys(text, path):
    """扫描同段内重复键，发现即报错（避免 PyYAML 静默取最后一个）。"""
    section = None
    seen = {}
    for i, line in enumerate(text.splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        m = _TOP_RE.match(line)
        if m and not line.startswith(' '):
            section = m.group(1)
            key = ('<top>', section)
        elif section is not None and line.startswith('  ') and not line.startswith('    '):
            m2 = _KEY_RE.match(line)
            if m2:
                key = (section, m2.group(1))
            else:
                continue
        else:
            continue
        seen.setdefault(key, []).append(i)

    dups = {k: v for k, v in seen.items() if len(v) > 1}
    if dups:
        detail = '; '.join(
            '%s.%s 出现在第 %s 行' % (sec, k, ','.join(map(str, lines)))
            for (sec, k), lines in dups.items())
        raise ValueError(
            '配置文件 %s 存在重复键（PyYAML 会静默取最后一个值，容易改错、'
            '导致实机行为异常）：\n  %s' % (path, detail))


def load_params(path=_DEFAULT_PATH):
    """读取机器人行为参数，返回 dict。

    发现同段重复键时抛 ValueError（安全优先，不允许静默覆盖）。
    """
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    _check_duplicate_keys(text, path)
    return yaml.safe_load(text)


def summarize(params):
    """生成当前算法关键生效参数的摘要（供启动时打印确认）。"""
    v = params.get('vision', {})
    s = params.get('search', {})
    g = params.get('grab', {})
    o = params.get('obstacle', {})
    arm = params.get('arm', {})
    gf = params.get('gimbal_fetch', {})
    lines = [
        'vision.min_area=%s(320x240) camera_rotate=%s'
        % (v.get('min_area'), v.get('camera_rotate')),
        'search.pan_pulses=%s tilt_pulses=%s' % (s.get('pan_pulses'), s.get('tilt_pulses')),
        'obstacle.threshold=%scm target_radius_gate=%s'
        % (o.get('threshold'), o.get('target_radius_gate')),
        'gimbal.pan_band=%s pan_band_fine=%s track_dead=%s/%s max_approach=%s'
        % (gf.get('pan_band', 80), gf.get('pan_band_fine', 20),
           gf.get('track_dead_cx', 40), gf.get('track_dead_cy', 60), gf.get('max_approach', 12)),
        'gimbal.scan_move=%sms/settle=%sms pan_move=%sms/settle=%sms'
        % (gf.get('scan_move_ms'), gf.get('scan_settle_ms'),
           gf.get('pan_move_ms'), gf.get('pan_settle_ms')),
        'gimbal.tilt_sign=%s track_settle=%sms lost_limit=%s帧'
        % (gf.get('tilt_sign'), gf.get('track_settle_ms'), gf.get('lost_limit_frames')),
        'gimbal.pan_sign=%s track_p_gain=%s walk_mm_small=%smm center_wait=%sms'
        % (gf.get('pan_sign'), gf.get('track_p_gain'), gf.get('walk_mm_small'), gf.get('center_wait_ms')),
        'grab.approach_area_threshold=%s(320x240) grab_area_ratio=%s'
        % (g.get('approach_area_threshold'), g.get('grab_area_ratio')),
        'grab.reach_y_start=%s reach_y_max=%s max_reach_steps=%s'
        % (g.get('reach_y_start'), g.get('reach_y_max'), g.get('max_reach_steps')),
        'grab.coarse_z=%s min_z=%s descend_max_steps=%s tof_grab_cm=%s'
        % (g.get('coarse_z'), g.get('min_z'), g.get('descend_max_steps'), g.get('tof_grab_cm')),
        'arm.gripper_open=%s gripper_close=%s'
        % (arm.get('gripper_open'), arm.get('gripper_close')),
    ]
    return '\n'.join('[params] %s' % line for line in lines)
