#!/usr/bin/python3
# coding=utf8
"""自主抓取：搜索寻路逼近 → 视觉伺服前伸夹取。

完整流程：
  1. 云台21/24平滑扫描找目标（小步渐进，不跳跃）
  2. 平滑PID追踪锁定目标居中（每周期限幅，不抖）
  3. 转身对准（让21号回500）
  4. 小步逼近（走一步等追踪拉回居中）
  5. 面积达标 → 视觉伺服前伸夹取

三维坐标函数写进去了但当前不稳定，实际夹取靠视觉识别。

用法：
    python3 CS-auto-grab.py --color blue
    python3 CS-auto-grab.py --color red --ratio 0.25
"""
import argparse
import os
import sys
import time
import threading

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2

from agcs_lib import (
    make_board, make_ik, make_arm_ik, load_params, load_lab_data, load_block_params,
    detect_color, correct_camera, load_undistort_maps, make_ultrasonic, make_display,
    open_camera, capture, stand,
)
from agcs_lib.vision import pixel_to_arm_coord
from agcs_lib.tracker import ColorTracker
from agcs_lib.grab import Grabber
from agcs_lib.logs import setup_logger, action_msg
from agcs_lib.motion import turn_left, turn_right, go_forward
from agcs_lib.sensors import show_status, dist_cm


# =====================================================================
# 红外 TOF（写入供演示，当前不用于实际夹取）
# =====================================================================
def init_tof():
    try:
        import board as adafruit_board
        import busio
        import adafruit_vl53l0x
        i2c = busio.I2C(adafruit_board.SCL, adafruit_board.SDA)
        return adafruit_vl53l0x.VL53L0X(i2c)
    except Exception:
        return None


def tof_height_cm(tof, samples=5):
    """Z轴：红外离地高度cm。当前不稳定，暂不用于实际夹取。"""
    if tof is None:
        return None
    vals = []
    for _ in range(samples):
        try:
            d = tof.range
            if 0 < d < 8000:
                vals.append(d)
        except Exception:
            pass
        time.sleep(0.01)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2] / 10.0


def solve_grasp_coord(K, R, T, center, tof, params, initial_coord=(0, 15, 5)):
    """三维坐标(x,y,z)——写入供演示，不用于实际夹取。"""
    x, y = pixel_to_arm_coord(K, R, T, center, initial_coord=initial_coord)
    tof_height_cm(tof)
    pick_z = float(params.get('arm', {}).get('pick_z', -4))
    return x, y, pick_z


# =====================================================================
# 平滑舵机工具
# =====================================================================
def servo_smooth_move(board, servo_id, from_pulse, to_pulse,
                      step=20, move_ms=0.1, settle_ms=0.15):
    """平滑移动单个舵机，每步step脉宽渐进，每步后等待settle_ms。"""
    if from_pulse == to_pulse:
        return to_pulse
    cur = int(from_pulse)
    target = int(to_pulse)
    step = int(step)
    direction = 1 if target > cur else -1
    while cur != target:
        nxt = cur + direction * step
        if (direction > 0 and nxt > target) or (direction < 0 and nxt < target):
            nxt = target
        board.bus_servo_set_position(move_ms, [[servo_id, nxt]])
        time.sleep(settle_ms)
        cur = nxt
    return cur


# =====================================================================
# 平滑追踪器：在 ColorTracker 上加每周期限幅 + 低通滤波
# =====================================================================
class SmoothTracker:
    """包装 ColorTracker，对云台21/24每周期最大变化量限幅，
    防止追踪时舵机抖动过大。

    限幅方式：
      - 每个追踪周期，PID 算出的脉宽变化量 clamp 到 [-max_step, +max_step]
      - 同时对目标 center 做指数移动平均（EMA），减少噪声抖动
    """

    def __init__(self, board, detect, params, logger,
                 pan_max_step=12, tilt_max_step=8,
                 ema_alpha=0.4,
                 dead_x=40, dead_y=60,
                 start_x=500, start_y=260,
                 interval=0.05):
        self.board = board
        self.detect = detect
        self.log = logger
        self.pan_max_step = int(pan_max_step)
        self.tilt_max_step = int(tilt_max_step)
        self.ema_alpha = float(ema_alpha)
        self.dead_x = int(dead_x)
        self.dead_y = int(dead_y)
        self.interval = float(interval)

        self.x_dis = int(start_x)
        self.y_dis = int(start_y)
        self.pan_min = int(params.get('gimbal_fetch', {}).get('pan_min', 100))
        self.pan_max = int(params.get('gimbal_fetch', {}).get('pan_max', 900))
        self.tilt_min = int(params.get('gimbal_fetch', {}).get('tilt_min', 100))
        self.tilt_max = int(params.get('gimbal_fetch', {}).get('tilt_max', 800))

        # PID（纯P控制，增益取自配置或默认0.12）
        from common.pid import PID
        p_gain = float(params.get('gimbal_fetch', {}).get('track_p_gain', 0.12))
        self.x_pid = PID(P=p_gain, I=0.0, D=0.0)
        self.y_pid = PID(P=p_gain, I=0.0, D=0.0)

        self._lock = threading.Lock()
        self._latest = None
        self._lost_frames = 0
        self._stop = threading.Event()
        self._thread = None

        # EMA 状态
        self._ema_cx = None
        self._ema_cy = None

    def _clamp_pulse_pan(self, v):
        return max(self.pan_min, min(self.pan_max, int(v)))

    def _clamp_pulse_tilt(self, v):
        return max(self.tilt_min, min(self.tilt_max, int(v)))

    def _update(self, r):
        cx_raw, cy_raw = r['center']
        a = self.ema_alpha

        # EMA 平滑目标坐标
        if self._ema_cx is None:
            self._ema_cx = float(cx_raw)
            self._ema_cy = float(cy_raw)
        else:
            self._ema_cx = a * cx_raw + (1.0 - a) * self._ema_cx
            self._ema_cy = a * cy_raw + (1.0 - a) * self._ema_cy
        cx = self._ema_cx
        cy = self._ema_cy

        old_x = self.x_dis
        old_y = self.y_dis

        # 21号水平追踪
        if abs(cx - 320) < self.dead_x:
            self.x_pid.clear()
        else:
            self.x_pid.SetPoint = 320
            self.x_pid.update(cx)
            raw_delta = self.x_pid.output
            delta = max(-self.pan_max_step, min(self.pan_max_step, int(raw_delta)))
            self.x_dis = self._clamp_pulse_pan(self.x_dis + delta)

        # 24号俯仰追踪
        if abs(cy - 240) < self.dead_y:
            self.y_pid.clear()
        else:
            self.y_pid.SetPoint = 240
            self.y_pid.update(cy)
            raw_delta = self.y_pid.output
            delta = max(-self.tilt_max_step, min(self.tilt_max_step, int(raw_delta)))
            self.y_dis = self._clamp_pulse_tilt(self.y_dis + delta)

        if old_x != self.x_dis or old_y != self.y_dis:
            self.log.debug('[smooth-track] 21: %d→%d  24: %d→%d',
                           old_x, self.x_dis, old_y, self.y_dis)

        self.board.bus_servo_set_position(0.03,
            [[24, self.y_dis], [21, self.x_dis]])

        with self._lock:
            self._latest = {
                'center': (int(cx_raw), int(cy_raw)),
                'area': r.get('area', 0),
                'radius': r.get('radius', 20),
                'x_dis': self.x_dis,
                'y_dis': self.y_dis,
            }
            self._lost_frames = 0

    def _run(self):
        while not self._stop.is_set():
            r = self.detect()
            if r is None:
                with self._lock:
                    self._latest = None
                    self._lost_frames += 1
            else:
                self._update(r)
            time.sleep(self.interval)

    def start(self, x_dis=None, y_dis=None):
        if self._thread is not None and self._thread.is_alive():
            return
        if x_dis is not None:
            self.x_dis = int(x_dis)
        if y_dis is not None:
            self.y_dis = int(y_dis)
        self._ema_cx = None
        self._ema_cy = None
        self._latest = None
        self._lost_frames = 0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='smooth-track', daemon=True)
        self._thread.start()
        self.log.info('[smooth-track] 启动 21=%d 24=%d pan_max_step=%d tilt_max_step=%d',
                      self.x_dis, self.y_dis, self.pan_max_step, self.tilt_max_step)

    def latest(self):
        with self._lock:
            if self._latest is None:
                return None
            return dict(self._latest)

    def lost_frames(self):
        with self._lock:
            return self._lost_frames

    def stop(self, timeout=1.0):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout)
        self.log.info('[smooth-track] 停止')


# =====================================================================
# 寻路：平滑扫描 + 追踪 + 转身 + 逼近
# =====================================================================
def scan_gimbal(board, detect, params, logger):
    """21/24 云台平滑扫描找目标。

    不再跳跃式移动24号（200→400→600太猛），
    改为：21号逐步移到每个位置，24号从低到高小步渐扫，
    每步仅 tilt_scan_step(20) 脉宽，settle 350ms 稳定后再检测。
    """
    gf = params.get('gimbal_fetch', {})
    sc = params.get('search', {})

    pan_positions = sc.get('pan_pulses', [500, 300, 200, 700, 800])
    tilt_lo = int(sc.get('tilt_lo', 150))
    tilt_hi = int(sc.get('tilt_hi', 800))
    tilt_scan_step = int(sc.get('tilt_scan_step', 20))
    tilt_step_ms = float(sc.get('tilt_step_ms', 350)) / 1000.0
    pan_step_ms = float(sc.get('pan_step_ms', 350)) / 1000.0
    pan_scan_step = int(gf.get('pan_scan_step', 20))
    detect_after_settle = float(sc.get('detect_settle_ms', 80)) / 1000.0

    cur_21 = 500
    cur_24 = tilt_lo

    for pan_target in pan_positions:
        # 平滑移动21号到目标位置
        cur_21 = servo_smooth_move(board, 21, cur_21, pan_target,
                                   step=pan_scan_step, move_ms=0.1, settle_ms=0.15)
        time.sleep(0.1)

        # 24号从低到高渐扫
        tilt = tilt_lo
        while tilt <= tilt_hi:
            cur_24 = servo_smooth_move(board, 24, cur_24, tilt,
                                       step=tilt_scan_step, move_ms=0.1, settle_ms=0.12)
            time.sleep(detect_after_settle)

            r = detect()
            if r is not None:
                logger.info('[scan] 发现 21=%d 24=%d center=%s', cur_21, cur_24, r['center'])
                return r

            tilt += tilt_scan_step

        # 24号从高到低渐扫（回程也检测）
        tilt = tilt_hi - tilt_scan_step
        while tilt >= tilt_lo:
            cur_24 = servo_smooth_move(board, 24, cur_24, tilt,
                                       step=tilt_scan_step, move_ms=0.1, settle_ms=0.12)
            time.sleep(detect_after_settle)

            r = detect()
            if r is not None:
                logger.info('[scan] 发现 21=%d 24=%d center=%s', cur_21, cur_24, r['center'])
                return r

            tilt -= tilt_scan_step

    return None


def approach(board, ik, detect, params, logger, ultrasonic=None, display=None):
    """追踪+转身+逼近，面积达标返回(center,cy)，失败返回(None,None)。

    用 SmoothTracker（每周期限幅 + EMA），不直接用 ColorTracker。
    """
    gf = params.get('gimbal_fetch', {})
    sc = params.get('search', {})
    obs = params.get('obstacle', {})

    track_dead_x = int(gf.get('track_dead_cx', 40))
    track_dead_y = int(gf.get('track_dead_cy', 60))
    approach_area_threshold = float(
        params.get('grab', {}).get('approach_area_threshold', 2800.0))
    max_approach = int(gf.get('max_approach', 12))
    walk_mm = int(gf.get('walk_mm', 40))
    walk_speed = int(gf.get('walk_speed', 50))
    center_wait = float(gf.get('center_wait_ms', 800)) / 1000.0
    lost_limit = int(gf.get('lost_limit_frames', 15))
    pan_band = int(gf.get('pan_band', 80))
    pan_band_fine = int(gf.get('pan_band_fine', 20))
    pan_turn_deg = int(gf.get('pan_turn_deg', 5))
    turn_sign = int(gf.get('turn_sign', 1))
    body_turn_speed = int(sc.get('body_turn_speed', 80))
    obstacle_threshold = float(obs.get('threshold', 35.0))
    obstacle_disable_radius = float(obs.get('target_radius_gate', 30.0))
    pan_max_step = int(gf.get('smooth_pan_max_step', 12))
    tilt_max_step = int(gf.get('smooth_tilt_max_step', 8))

    # 启动平滑追踪线程
    tracker = SmoothTracker(
        board, detect, params, logger,
        pan_max_step=pan_max_step, tilt_max_step=tilt_max_step,
        ema_alpha=0.4,
        dead_x=track_dead_x, dead_y=track_dead_y,
        start_x=500, start_y=260,
        interval=0.05)
    tracker.start()
    logger.info('[approach] 平滑追踪启动')

    stop_event = threading.Event()

    def obstacle_monitor():
        blocked = 0
        while not stop_event.is_set():
            latest = tracker.latest()
            if latest is not None and int(latest.get('radius', 0)) >= obstacle_disable_radius:
                blocked = 0
                time.sleep(0.05)
                continue
            d = dist_cm(ultrasonic)
            if 0 < d < obstacle_threshold:
                blocked += 1
                if blocked >= 2:
                    stop_event.set()
                    return
            else:
                blocked = 0
            time.sleep(0.1)

    obs_thread = threading.Thread(target=obstacle_monitor, daemon=True)
    obs_thread.start()

    def _turn_body(angle):
        if angle == 0:
            return
        if turn_sign > 0:
            (turn_left if angle > 0 else turn_right)(ik, abs(angle), body_turn_speed)
        else:
            (turn_right if angle > 0 else turn_left)(ik, abs(angle), body_turn_speed)

    def _wait_target(timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if stop_event.is_set():
                return None
            r = tracker.latest()
            if r is not None:
                return r
            time.sleep(0.03)
        return None

    def _wait_centered(timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if stop_event.is_set():
                return None
            r = tracker.latest()
            if r is not None:
                cx, cy = r['center']
                if abs(int(r['x_dis']) - 500) <= pan_band and 120 <= cy <= 380:
                    return r
            time.sleep(0.03)
        return None

    try:
        # 1) 转身对准
        logger.info('[approach] 转身对准')
        for i in range(max_approach):
            if stop_event.is_set():
                return None, None
            r = tracker.latest()
            if r is None:
                time.sleep(0.05)
                continue
            dx = int(r['x_dis']) - 500
            if abs(dx) <= pan_band:
                logger.info('[approach] 对准完成 21偏移=%+d', dx)
                break
            logger.info('[approach] 转身 21偏移=%+d', dx)
            _turn_body(pan_turn_deg if dx > 0 else -pan_turn_deg)
            time.sleep(0.4)

        # 2) 逼近
        logger.info('[approach] 开始逼近')
        for step in range(max_approach * 3):
            if stop_event.is_set():
                return None, None

            r = _wait_target(center_wait)
            if r is None:
                lost = tracker.lost_frames()
                logger.info('[approach] 目标丢失 %d帧', lost)
                return None, None

            cx, cy = r['center']
            dx = int(r['x_dis']) - 500

            if abs(dx) > pan_band:
                logger.info('[approach] 偏移%+d，转身', dx)
                _turn_body(pan_turn_deg if dx > 0 else -pan_turn_deg)
                time.sleep(0.4)
                continue

            if not (120 <= cy <= 380):
                r2 = _wait_centered(center_wait)
                if r2 is None:
                    logger.info('[approach] 目标未回中部')
                    return None, None
                r = r2
                cx, cy = r['center']
                dx = int(r['x_dis']) - 500

            area = int(r.get('area', 0))
            radius = int(r.get('radius', 20))

            if area >= approach_area_threshold:
                logger.info('[approach] 面积%d>=%d 到位', area, int(approach_area_threshold))
                for _ in range(max_approach):
                    r2 = tracker.latest()
                    if r2 is None:
                        break
                    dx2 = int(r2['x_dis']) - 500
                    if abs(dx2) <= pan_band_fine:
                        break
                    _turn_body(pan_turn_deg if dx2 > 0 else -pan_turn_deg)
                    time.sleep(0.4)
                r_final = tracker.latest()
                if r_final is not None:
                    return r_final['center'], r_final['center'][1]
                return r['center'], cy

            logger.info('[approach] #%d 面积=%d 半径=%d 21偏=%+d', step+1, area, radius, dx)
            go_forward(ik, walk_mm, walk_speed, 1)
            time.sleep(0.05)

        logger.info('[approach] 最大步数')
        return None, None

    finally:
        stop_event.set()
        tracker.stop()
        if obs_thread.is_alive():
            obs_thread.join(timeout=1.0)


# =====================================================================
# 主程序
# =====================================================================
def main():
    parser = argparse.ArgumentParser(description='自主抓取：寻路逼近→视觉夹取')
    parser.add_argument('--color', default='blue',
                        choices=['red', 'green', 'blue', 'yellow', 'cz1'])
    parser.add_argument('--min-area', type=int, default=150)
    parser.add_argument('--ratio', type=float, default=0.30, help='画面占比阈值')
    args = parser.parse_args()

    logger = setup_logger('CS-auto-grab')
    logger.info('[main] 启动 color=%s', args.color)

    board = make_board()
    ik = make_ik(board)
    ak = make_arm_ik(board)
    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    lab = load_lab_data()
    mapx, mapy = load_undistort_maps()
    K, R, T = load_block_params()
    ultrasonic = make_ultrasonic()
    display = make_display()
    tof = init_tof()
    cam = open_camera()

    arm = params['arm']
    open_pulse = int(arm.get('gripper_open', 120))
    close_pulse = int(arm.get('gripper_close', 550))
    reset_pulses = arm.get('reset_pulses', {21: 500, 22: 705, 23: 90, 24: 330})
    params['grab']['grab_area_ratio'] = args.ratio

    def detect(min_area=None):
        min_area = min_area or args.min_area
        f = capture(cam)
        if f is None:
            return None
        f = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)
        return detect_color(f, lab, args.color, min_area=min_area)

    stand(ik)
    board.bus_servo_set_position(1.5,
        [[sid, reset_pulses[sid]] for sid in [21, 22, 23, 24]])
    board.bus_servo_set_position(1.0, [[25, open_pulse]])
    time.sleep(2)

    grabbed = False

    try:
        # 1) 扫描
        logger.info('[scan] 云台平滑扫描')
        r = scan_gimbal(board, detect, params, logger)
        if r is None:
            for rnd in range(3):
                logger.info('[scan] 第%d轮转身再扫', rnd + 1)
                turn_left(ik, angle=30, speed=60)
                r = scan_gimbal(board, detect, params, logger)
                if r is not None:
                    break
        if r is None:
            logger.info('[scan] 未找到目标')
            return
        logger.info('[scan] 发现目标 center=%s', r['center'])

        # 2) 追踪+逼近
        logger.info('[approach] 平滑追踪逼近')
        center, cy = approach(board, ik, detect, params, logger,
                              ultrasonic=ultrasonic, display=display)
        if center is None:
            logger.info('[approach] 逼近失败')
            return
        logger.info('[approach] 到位 center=%s', str(center))

        # 3) 三维坐标（演示）
        detect_pose = [float(v) for v in arm.get('detect_pose', [0, 15, 5])]
        coord_x, coord_y, coord_z = solve_grasp_coord(
            K, R, T, center, tof, params,
            initial_coord=(detect_pose[0], detect_pose[1]))
        logger.info('[coord] x=%.1f y=%.1f z=%.1f (演示)', coord_x, coord_y, coord_z)

        # 4) 视觉伺服夹取
        logger.info('[grab] 视觉伺服夹取')
        grabber = Grabber(board, ik, ak, params, K, R, T, detect,
                          display, ultrasonic=ultrasonic, tof=tof)
        ok = grabber.run()
        grabbed = ok
        if ok:
            logger.info('[grab] 夹取成功')
            try:
                input('已夹住，敲回车松开复位...')
            except EOFError:
                time.sleep(2)
        else:
            logger.info('[grab] 夹取失败')

    except KeyboardInterrupt:
        logger.info('[main] 中断')
    finally:
        board.bus_servo_set_position(1.5,
            [[sid, reset_pulses[sid]] for sid in [21, 22, 23, 24]])
        board.bus_servo_set_position(1.0, [[25, open_pulse]])
        time.sleep(1.5)
        stand(ik)
        cam.camera_close()
        logger.info('[main] %s', '成功' if grabbed else '未完成')


if __name__ == '__main__':
    main()
