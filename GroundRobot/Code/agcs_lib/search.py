#!/usr/bin/python3
# coding=utf8
"""寻路算法（二开）：21+24 双舵机搜索 + PID 持续锁定 + 步态逼近。

参照官方 color_track.py：
- 舵机21 水平、舵机24 俯仰，搜索时两轴都扫；
- 找到后 PID 让摄像头一直锁定目标，避免目标进入盲区导致连续丢失/重扫。
"""
import math
import time

from common.pid import PID
from agcs_lib.motion import turn_left, turn_right, go_forward, go_back
from agcs_lib.sensors import show_status


class Searcher:
    def __init__(self, board, ik, ak, params, detect, ultrasonic=None, display=None):
        self.board = board
        self.ik = ik
        self.ak = ak
        self.params = params
        self.detect = detect
        self.ultrasonic = ultrasonic
        self.display = display

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
        self.v_scan = [260, 200, 320, 140, 380, 80, 440]

    def _status(self, v):
        show_status(self.display, v)

    def _cam(self):
        self.board.bus_servo_set_position(0.02, [[24, int(self.y_dis)], [21, int(self.x_dis)]])

    def _set_cam(self, x, y):
        self.x_dis = max(self.pan_min, min(self.pan_max, int(x)))
        self.y_dis = max(0, min(1000, int(y)))
        self._cam()
        time.sleep(self.settle)

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

    def _close(self, r):
        cy = r['center'][1]
        radius = max(int(r.get('radius', 20)), 1)
        if cy + radius >= 480 - self.edge_margin:
            return True
        dist = self.area_k / math.sqrt(max(int(r.get('area', 1)), 1))
        return dist <= self.near_cm

    def search(self):
        """21+24 双轴扫描，返回目标 dict 或 None。"""
        self.board.bus_servo_set_position(0.5, [[25, 120]])
        time.sleep(0.5)
        self._set_cam(500, 260)
        r = self.detect()
        if r is not None:
            cr = self._confirm()
            if cr is not None:
                return cr

        pans = list(range(500 + self.pan_step, self.pan_max + 1, self.pan_step)) + \
               list(range(self.pan_max, self.pan_min - 1, -self.pan_step)) + \
               list(range(self.pan_min, 500, self.pan_step))
        for y in self.v_scan:
            print('[search][扫描] 俯仰 y_dis=%d 开始水平扫描' % y, flush=True)
            self._set_cam(500, y)
            for x in pans:
                self._set_cam(x, y)
                r = self.detect()
                if r is not None:
                    print('[search][扫描] 检测到目标 x_dis=%d y_dis=%d' % (x, y), flush=True)
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

    def run(self):
        self._status(1)
        for round_no in range(self.scan_rounds):
            det = self.search()
            if det is None:
                print('[search] 第%d轮未找到目标' % (round_no + 1))
                continue

            cx, cy = det['center']
            print('[search] 找到目标 x_dis=%d y_dis=%d cx=%d cy=%d area=%d'
                  % (self.x_dis, self.y_dis, cx, cy, det['area']), flush=True)

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
            for step in range(self.max_approach):
                r = self.detect()
                if r is None:
                    lost += 1
                    if lost >= 3:
                        print('[search] 连续丢失目标')
                        return None, None
                    go_back(self.ik, 15, 50)
                    time.sleep(0.4)
                    continue
                lost = 0
                cx, cy = r['center']
                area = max(int(r.get('area', 1)), 1)
                dist = self.area_k / math.sqrt(area)
                centered = self._track(r['center'])
                print('[search][追踪] #%d x_dis=%d y_dis=%d cx=%d cy=%d dist=%.1fcm'
                      % (step + 1, self.x_dis, self.y_dis, cx, cy, dist), flush=True)

                # 云台水平偏太多则转身并回正
                if self.x_dis > 700:
                    self._turn_body(-10)
                    self._set_cam(500, self.y_dis)
                elif self.x_dis < 300:
                    self._turn_body(10)
                    self._set_cam(500, self.y_dis)

                if self._close(r):
                    return r['center'], cy

                if dist > self.fine_cm:
                    go_forward(self.ik, self.fast_walk_mm, self.fast_speed)
                else:
                    go_forward(self.ik, self.walk_mm, self.walk_speed)
                time.sleep(0.3)

            r = self.detect()
            if r is not None and self._close(r):
                return r['center'], r['center'][1]

        return None, None
