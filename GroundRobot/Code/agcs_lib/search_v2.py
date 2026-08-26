#!/usr/bin/python3
# coding=utf8
"""寻路算法 v2：云台抬头 PID 追踪 + 超声波避障 + 距离判断 + 目标丢失保底。

与 v1 的区别：
- 保持抬头追踪（云台 24 不低头），避免高处目标贴底；
- 距离判断用超声波 dist_cm（不靠面积估距），并做避障；
- 目标丢失有「恢复云台位 → 重扫 → 后退 → 转身」的兜底链。
"""
import math
import time

from common.pid import PID
from agcs_lib.motion import turn_left, turn_right, go_forward, go_back
from agcs_lib.sensors import show_status, dist_cm
from agcs_lib.logs import get_logger, action_msg


class SearcherV2:
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
        self.near_cm = float(gf.get('near_cm', 15.0))
        self.obstacle_stop = float(gf.get('obstacle_stop', 12.0))
        self.cam_raise_limit = int(gf.get('cam_raise_limit', 120))
        self.timeout = float(gf.get('timeout', 60.0))
        self.scan_rounds = int(gf.get('scan_rounds', 3))
        self.fast_walk_mm = int(gf.get('fast_walk_mm', 70))
        self.fast_speed = int(gf.get('fast_speed', 90))
        self.walk_mm = int(gf.get('walk_mm', 40))
        self.walk_speed = int(gf.get('walk_speed', 50))

        # 云台（与官方 color_track 一致）
        self.x_dis = 500
        self.y_dis = 260
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

    def _turn_body(self, angle):
        if angle == 0:
            return
        if self.turn_sign > 0:
            (turn_left if angle > 0 else turn_right)(self.ik, abs(angle), 60)
        else:
            (turn_right if angle > 0 else turn_left)(self.ik, abs(angle), 60)

    def _track(self, center):
        """云台 PID 追踪，让目标锁定到画面中心（cx→320, cy→240）。"""
        cx, cy = center
        self.x_pid.SetPoint = 320
        self.x_pid.update(cx)
        self.y_pid.SetPoint = 240
        self.y_pid.update(cy)
        self._set_cam(self.x_dis + self.x_pid.output, self.y_dis + self.y_pid.output)
        return abs(cx - 320) < 30 and abs(cy - 240) < 30

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

    def _vertical_sweep(self, x):
        for y in self.v_scan:
            self._set_cam(x, y)
            r = self.detect()
            if r is not None:
                self.log.info('[search] %s', action_msg('检测到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽' % (x, y)))
                for _ in range(4):
                    self._track(r['center'])
                    time.sleep(0.1)
                    r2 = self.detect()
                    if r2 is not None:
                        r = r2
                    else:
                        break
                cr = self._confirm()
                return cr if cr is not None else r
        return None

    def _search(self):
        self.board.bus_servo_set_position(0.5, [[25, 120]])
        time.sleep(0.5)
        self._set_cam(500, 260)
        r = self.detect()
        if r is not None:
            cr = self._confirm()
            if cr is not None:
                return cr
        r = self._vertical_sweep(500)
        if r is not None:
            return r
        for x in [400, 300, 250, 600, 700, 750]:
            r = self._vertical_sweep(x)
            if r is not None:
                return r
        return None

    def _rescan(self):
        pans = list(range(500 + self.pan_step, 1001, self.pan_step)) + \
               list(range(1000, -1, -self.pan_step)) + \
               list(range(0, 500, self.pan_step))
        for y in self.v_scan:
            self._set_cam(500, y)
            for x in pans:
                self._set_cam(x, y)
                r = self.detect()
                if r is not None:
                    return r
        return None

    def _recover(self):
        """目标丢失保底：恢复云台位 → 重扫 → 后退 → 转身。"""
        r = self.detect()
        if r is not None:
            return r
        self._set_cam(500, self.y_dis)
        r = self.detect()
        if r is not None:
            return r
        r = self._rescan()
        if r is not None:
            return r
        go_back(self.ik, 15, 50)
        time.sleep(0.4)
        return self.detect()

    def run(self):
        self._status(1)
        det = None
        for round_no in range(self.scan_rounds):
            det = self._search()
            if det is not None:
                break
            self.log.info('[search] %s', action_msg('第%d轮未找到目标' % (round_no + 1)))
        if det is None:
            return None, None

        cx, cy = det['center']
        self.log.info('[search] %s', action_msg(
            '找到目标', action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 面积=%d像素'
            % (self.x_dis, self.y_dis, cx, cy, det['area'])))

        lost = 0
        step = 0
        start_time = time.time()
        while True:
            # 超时兜底（用时间，不用轮数）
            if time.time() - start_time > self.timeout:
                self.log.info('[search] %s', action_msg('逼近超时', reason='超过 %.0f 秒' % self.timeout))
                return None, None

            step += 1
            r = self.detect()
            if r is None:
                lost += 1
                if lost >= 3:
                    r = self._recover()
                    if r is None:
                        self.log.info('[search] %s', action_msg('连续丢失目标'))
                        return None, None
                    lost = 0
                else:
                    continue
            else:
                lost = 0

            cx, cy = r['center']
            # 云台持续追踪（多次迭代 + 重新检测，让目标真正居中，摄像头一直盯着）
            for _ in range(3):
                self._track(r['center'])
                r2 = self.detect()
                if r2 is None:
                    break
                r = r2
            cx, cy = r['center']
            ultra = dist_cm(self.ultrasonic) if self.ultrasonic else -1.0
            self.log.info('[search] %s', action_msg(
                '追踪 #%d' % (step + 1),
                action='云台水平=%d脉宽 云台俯仰=%d脉宽 中心x=%dpx 中心y=%dpx 超声波=%.1fcm'
                % (self.x_dis, self.y_dis, cx, cy, ultra)))

            # 距离判断（超声波）
            if 0 < ultra <= self.near_cm:
                self.log.info('[search] %s', action_msg('够近', action='超声波=%.1fcm' % ultra))
                return (cx, cy), cy
            if 0 < ultra <= self.obstacle_stop:
                self.log.info('[search] %s', action_msg('前方有障碍物/已贴近', action='超声波=%.1fcm 停止前进' % ultra))
                return (cx, cy), cy
            # 云台抬头到极限（目标太高）→ 停止前进，交给抬头夹取
            if self.y_dis <= self.cam_raise_limit:
                self.log.info('[search] %s', action_msg(
                    '目标过高', reason='云台俯仰=%d脉宽 抬头到下限' % self.y_dis, action='停止前进'))
                return (cx, cy), cy

            # 先对准再走：目标横向偏 → 六足转身；上下偏 → 云台俯仰；居中才前进
            if abs(cx - 320) > 40:
                deg = (320 - cx) / self.px_per_deg * self.turn_sign
                deg = max(-self.turn_deg, min(self.turn_deg, deg))
                self._turn_body(deg)
                time.sleep(0.4)
            elif abs(cy - 240) > 60:
                # 上下偏，交给云台俯仰（下一轮 _track 继续调），暂不走路
                self.log.debug('[search] %s', action_msg('上下未居中', reason='中心y=%dpx' % cy))
                time.sleep(0.1)
            else:
                # 目标居中，才前进（超声波远才前进，避障）
                if ultra < 0 or ultra > self.obstacle_stop:
                    go_forward(self.ik, self.walk_mm, self.walk_speed, 1)
                    time.sleep(0.1)
                else:
                    self.log.debug('[search] %s', action_msg('暂停前进', reason='超声波=%.1fcm' % ultra))

        return None, None
