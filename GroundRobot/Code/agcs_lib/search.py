#!/usr/bin/python3
# coding=utf8
"""寻路算法（二开）：先 24 号舵机上下扫描，未找到再逐点转动 21 号舵机，每次转动后 24 号上下扫描一遍。

扫描顺序：
- 先检测初始位(500, 260)；
- 未找到时，21 号舵机脉宽固定，先让 24 号舵机上下扫描一遍；
- 仍未找到时，21 号舵机逐点转动，每动一次脉宽就做一次 24 号舵机上下扫描；
- 找到后 PID 让摄像头一直锁定目标，避免目标进入盲区导致连续丢失/重扫。
"""
import math
import time

from common.pid import PID
from agcs_lib.motion import turn_left, turn_right, go_forward, go_back
from agcs_lib.sensors import show_status
from agcs_lib.logs import get_logger, action_msg


class Searcher:
    def __init__(self, board, ik, ak, params, detect, ultrasonic=None, display=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.detect = detect
        self.ultrasonic = ultrasonic
        self.display = display
        self.log = get_logger()

        gf = params.get('gimbal_fetch', {})
        self.pan_min = int(gf.get('pan_min', 0))
        self.pan_max = int(gf.get('pan_max', 1000))
        self.pan_step = int(gf.get('pan_step', 40))
        self.settle = float(gf.get('settle_ms', 280)) / 1000.0
        self.edge_margin = int(gf.get('edge_margin', 30))
        self.px_per_deg = float(gf.get('px_per_deg', 10.7))
        self.turn_deg = int(gf.get('turn_deg', 10))
        self.turn_sign = int(gf.get('turn_sign', 1))
        self.area_k = float(gf.get('area_k', 1650.0))
        self.near_cm = float(gf.get('near_cm', 18.0))
        self.max_approach = int(gf.get('max_approach', 20))
        self.scan_rounds = int(gf.get('scan_rounds', 3))
        self.fine_cm = float(gf.get('fine_cm', 12.0))
        self.fast_walk_mm = int(gf.get('fast_walk_mm', 70))
        self.fast_speed = int(gf.get('fast_speed', 90))
        self.walk_mm = int(gf.get('walk_mm', 40))
        self.walk_speed = int(gf.get('walk_speed', 50))

        # 相机舵机（与官方 color_track 一致）
        self.x_dis = 500          # 舵机21 水平
        self.y_dis = 260          # 舵机24 俯仰
        self.x_pid = PID(P=0.1, I=0.001, D=0.008)
        self.y_pid = PID(P=0.1, I=0.02, D=0.008)
        self.v_scan = [80, 140, 200, 260, 320, 380, 440]  # 24舵机俯仰：单调递增扫描，避免上下抖动

    def _status(self, v):
        show_status(self.display, v)

    def _cam(self):
        self.board.bus_servo_set_position(0.02, [[24, int(self.y_dis)], [21, int(self.x_dis)]])

    def _set_cam(self, x, y):
        old_x, old_y = self.x_dis, self.y_dis
        self.x_dis = max(self.pan_min, min(self.pan_max, int(x)))
        self.y_dis = max(0, min(1000, int(y)))
        self._cam()
        time.sleep(self.settle)
        if (old_x, old_y) != (self.x_dis, self.y_dis):
            self.log.debug('[search] %s', action_msg(
                '云台调整', action='舵机21 %d->%d脉宽，舵机24 %d->%d脉宽'
                % (old_x, self.x_dis, old_y, self.y_dis)))

    def _confirm(self, tries=5, need_hits=2):
        hits = 0
        last = None
        for _ in range(tries):
            r = self.detect()
            if r is not None:
                cx, cy = r['center']
                if (self.edge_margin <= cx <= 640 - self.edge_margin and
                        self.edge_margin <= cy <= 480 - self.edge_margin):
                    hits += 1
                    last = r
            time.sleep(0.08)
        return last if hits >= need_hits else None

    def _turn_body(self, angle):
        if angle == 0:
            return
        self.log.info('[search] %s', action_msg('转身', action='%+.0f°' % angle))
        if self.turn_sign > 0:
            (turn_left if angle > 0 else turn_right)(self.ik, abs(angle), 60)
        else:
            (turn_right if angle > 0 else turn_left)(self.ik, abs(angle), 60)

    def _track(self, center):
        """PID 让舵机21/24 把目标锁到画面中心，返回是否已居中。"""
        cx, cy = center
        self.x_pid.SetPoint = 320
        self.x_pid.update(cx)
        self.y_pid.SetPoint = 240
        self.y_pid.update(cy)
        self._set_cam(self.x_dis + self.x_pid.output, self.y_dis + self.y_pid.output)
        return abs(cx - 320) < 30 and abs(cy - 240) < 30

    def _close(self, r, dist=None, best_dist=None, start_dist=None):
        """到位判定：目标贴底（即将出画面）或估算距离进入 near_cm 以内 → 到位。"""
        cy = r['center'][1]
        radius = max(int(r.get('radius', 20)), 1)
        # 目标贴底（即将出画面）→ 到位：目标已在机器人脚下附近，交给 grab 像素转坐标夹取
        if cy + radius >= 480 - self.edge_margin:
            return True
        if dist is None:
            dist = self.area_k / math.sqrt(max(int(r.get('area', 1)), 1))
        if dist <= self.near_cm:
            return True
        return False

    def _vertical_sweep(self, x):
        """固定 21 号舵机脉宽为 x，让 24 号舵机上下扫描一遍；找到目标返回 dict，否则返回 None。"""
        for y in self.v_scan:
            self._set_cam(x, y)
            r = self.detect()
            if r is not None:
                self.log.info('[search] %s', action_msg(
                    '检测到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽' % (x, y)))
                # 检测到目标后先 PID 锁定到画面中心，不再继续扫描
                for _ in range(4):
                    self._track(r['center'])
                    time.sleep(0.1)
                    r2 = self.detect()
                    if r2 is not None:
                        r = r2
                    else:
                        break
                cr = self._confirm()
                if cr is not None:
                    return cr
                # 确认失败也返回当前目标，交给上层追踪处理
                return r
        return None

    def search(self):
        """第一次扫描：先 24 号舵机上下扫描；未找到再逐点转动 21 号舵机，每次转动后 24 号上下扫描一遍。"""
        self.board.bus_servo_set_position(0.5, [[25, 120]])
        time.sleep(0.5)
        self._set_cam(500, 260)
        r = self.detect()
        if r is not None:
            cr = self._confirm()
            if cr is not None:
                return cr

        # 第一步：21 号舵机脉宽固定在中位(500)，仅 24 号舵机上下扫描
        self.log.info('[search] %s', action_msg('开始扫描', action='21号固定500，24号上下扫描'))
        r = self._vertical_sweep(500)
        if r is not None:
            return r

        # 第二步：21 号舵机逐点转动，每动一次脉宽就做一次 24 号舵机上下扫描
        pans = [400, 300, 250, 600, 700, 750]  # 21舵机水平扫描：先左后右，范围250~750，步进100
        for x in pans:
            self.log.info('[search] %s', action_msg('扫描', action='21号脉宽=%d，24号上下扫描' % x))
            r = self._vertical_sweep(x)
            if r is not None:
                return r
        return None

    def run(self):
        self._status(1)
        for round_no in range(self.scan_rounds):
            det = self.search()
            if det is None:
                self.log.info('[search] %s', action_msg('第%d轮未找到目标' % (round_no + 1)))
                continue

            cx, cy = det['center']
            self.log.info('[search] %s', action_msg(
                '找到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 面积=%d像素'
                % (self.x_dis, self.y_dis, cx, cy, det['area'])))

            # 按云台角度转身朝向一次
            target_dir = (self.x_dis - 500) / 4.1667 + (320 - cx) / self.px_per_deg
            deg = target_dir * self.turn_sign
            remaining = abs(deg)
            while remaining > 3:
                step = min(self.turn_deg, remaining)
                self._turn_body(step if deg > 0 else -step)
                remaining -= step
                time.sleep(0.5)
            self._set_cam(500, self.y_dis)

            # 追踪 + 逼近
            lost = 0
            start_dist = None
            best_dist = None
            for step in range(self.max_approach):
                r = self.detect()
                if r is None:
                    lost += 1
                    if lost >= 3:
                        self.log.info('[search] %s', action_msg('连续丢失目标'))
                        return None, None
                    self.log.debug('[search] %s', action_msg('目标丢失，后退找回', action='后退 15mm'))
                    go_back(self.ik, 15, 50)
                    time.sleep(0.4)
                    continue
                lost = 0
                cx, cy = r['center']
                area = max(int(r.get('area', 1)), 1)
                dist = self.area_k / math.sqrt(area)
                if start_dist is None:
                    start_dist = dist
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                centered = self._track(r['center'])
                self.log.info('[search] %s', action_msg(
                    '追踪 #%d' % (step + 1),
                    action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 距离=%.1fcm 最近距离=%.1fcm'
                    % (self.x_dis, self.y_dis, cx, cy, dist, best_dist)))

                # 云台水平偏太多则转身并回正
                if self.x_dis > 700:
                    self._turn_body(-10)
                    self._set_cam(500, self.y_dis)
                elif self.x_dis < 300:
                    self._turn_body(10)
                    self._set_cam(500, self.y_dis)

                if self._close(r, dist=dist, best_dist=best_dist, start_dist=start_dist):
                    return r['center'], cy

                if dist > self.fine_cm:
                    self.log.debug('[search] %s', action_msg(
                        '接近中', reason='距离 %.1fcm > %.1fcm' % (dist, self.fine_cm),
                        action='前进 %dmm' % self.fast_walk_mm))
                    go_forward(self.ik, self.fast_walk_mm, self.fast_speed)
                else:
                    self.log.debug('[search] %s', action_msg(
                        '接近中', reason='距离 %.1fcm <= %.1fcm' % (dist, self.fine_cm),
                        action='前进 %dmm' % self.walk_mm))
                    go_forward(self.ik, self.walk_mm, self.walk_speed)
                time.sleep(0.3)

            r = self.detect()
            if r is not None and self._close(r, best_dist=best_dist, start_dist=start_dist):
                return r['center'], r['center'][1]

        return None, None
