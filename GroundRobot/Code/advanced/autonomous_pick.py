#!/usr/bin/python3
# coding=utf8
"""自主行走 + 视觉识别 + 机械臂抓取 整合程序（第二步~第四步的合体演示）。

运行（树莓派 VNC 终端，先 sudo systemctl stop spiderpi）：
    python3 ~/spiderpi/advanced/autonomous_pick.py --color red
    python3 ~/spiderpi/advanced/autonomous_pick.py --detector yolo --model /home/pi/yolov8n.pt

状态机：
    NAV      等待地面站下发任务（HTTP /task）→ 转向目标 → 避障前进到粗定位区域
    SEARCH   原地扫描寻找目标，一圈未发现则前进一段
    APPROACH 对准目标中心并逐步接近（带超声波避障）
    PICK     进入机械臂可及范围后抓取，完成后后退继续找下一个

启动后自动在后台启动 HTTP 服务（POST /task 收任务，GET /status 上报状态），
主程序自主判断并接收地面站消息。按 ESC 退出，程序会复位机械臂并站定。
"""
import os
import sys
import time
import argparse
import math

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2
from common import kinematics
from common.ros_robot_controller_sdk import Board
from sensor.ultrasonic_sensor import Ultrasonic
from calibration.camera import Camera
import arm_ik.arm_move_ik as AMK

from functions.robot_config import load_params
from functions.vision_utils import (load_lab_data, load_block_params, detect_color,
                                    pixel_to_arm_coord, load_undistort_maps)
from functions.obstacle_avoidance import ObstacleAvoidance
from functions.yolo_detect import YoloDetector
from kinematic_routines.arm_pick import ArmPicker
try:
    from communication import task_server
except ImportError:
    task_server = None

STATE_SEARCH = 'SEARCH'
STATE_APPROACH = 'APPROACH'
STATE_PICK = 'PICK'
STATE_NAV = 'NAV'


class AutonomousPick:
    """自主抓取状态机。"""

    def __init__(self, color='red', detector='color',
                 model_path=None, target_classes=('pod',), wait_task=True):
        self.params = load_params()
        self.color = color

        self.board = Board()
        self.ik = kinematics.IK(self.board)
        self.ultrasonic = Ultrasonic()
        self.ak = AMK.ArmIK()

        self.walker = ObstacleAvoidance(self.board, self.ik, self.ultrasonic, self.params)
        self.picker = ArmPicker(self.board, self.ak, self.params)

        self.lab_data = load_lab_data()
        self.K, self.R, self.T = load_block_params()
        self.mapx, self.mapy = load_undistort_maps()

        self.detector_name = detector
        self.yolo = None
        if detector == 'yolo':
            self.yolo = YoloDetector(model_path=model_path,
                                     target_classes=target_classes)

        # 默认先进入 NAV：等地面站下发任务；--no-wait-task 可恢复旧行为
        self.state = STATE_NAV if wait_task else STATE_SEARCH
        self.turn_count = 0
        self.lost_streak = 0
        self.pick_count = 0

        # NAV 状态：位置估算（以启动点为原点，x 向前、y 向左，单位米）
        self.nav = self.params.get('nav', {})
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0           # 当前朝向（度，0=初始朝向，逆时针为正）
        self.current_task = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.arrive_radius = self.nav.get('arrive_radius', 0.8)
        self.walked_m = 0.0

        if task_server is not None:
            task_server.start_server()
            task_server.set_status(state=self.state, message='等待地面站下发任务')
        else:
            print('[NAV] 未找到 communication/task_server，无法接收任务')

    def reset_pose(self):
        """机械臂回到检测姿态，机器人立正。"""
        self.picker.reset_pose()
        self.ik.stand(self.ik.initial_pos, t=500)

    def _detect(self, img):
        """按检测器类型返回 dict(center, area, ...) 或 None。"""
        if self.detector_name == 'yolo':
            return self.yolo.detect(img)
        # 嵌套字典取值：先取 vision 组，再取组里的 min_area（有效目标最小面积）
        return detect_color(img, self.lab_data, self.color,
                            min_area=self.params['vision']['min_area'])

    def _step_search(self, img):
        det = self._detect(img)
        if det is not None:
            self.turn_count = 0
            return STATE_APPROACH
        w = self.params['walk']   # 取 walk 组参数（步幅/转向角/阈值），w['xxx'] 就是取组里的键
        self.walker.turn(left=True, angle=w['turn_angle'])   # walk.turn_angle 扫描转向角
        self.turn_count += 1
        if self.turn_count * w['turn_angle'] >= 360:
            self.turn_count = 0
            print('[SEARCH] 扫描一圈未发现目标，前进一段')
            self.walker.walk_forward()
        return STATE_SEARCH

    def _step_approach(self, img):
        det = self._detect(img)
        if det is None:
            self.lost_streak += 1
            if self.lost_streak > 10:
                print('[APPROACH] 目标丢失，回到扫描')
                self.lost_streak = 0
                return STATE_SEARCH
            time.sleep(0.05)
            return STATE_APPROACH
        self.lost_streak = 0
        w = self.params['walk']   # 同上：walk 组参数
        img_cx = img.shape[1] / 2.0
        offset = det['center'][0] - img_cx
        if abs(offset) > w['align_tolerance']:   # walk.align_tolerance 对准容差
            # 目标偏右→右转；偏左→左转
            self.walker.turn(left=offset < 0, angle=w['turn_angle'])
            return STATE_APPROACH
        if det['area'] >= w['approach_area']:   # walk.approach_area 判定"够近可抓"的面积
            print('[APPROACH] 目标足够近 area=%d' % det['area'])
            return STATE_PICK
        self.walker.walk_forward()
        return STATE_APPROACH

    def _step_pick(self, img):
        det = self._detect(img)
        if det is None:
            return STATE_SEARCH
        wx, wy = pixel_to_arm_coord(self.K, self.R, self.T, det['center'])
        w = self.params['walk']   # 同上：walk 组参数
        if abs(wx) > w['reach_x'] or wy > w['reach_y']:   # walk.reach_x/y 机械臂可及范围
            print('[PICK] 超出机械臂可及范围 (%.1f, %.1f) cm，继续接近' % (wx, wy))
            return STATE_APPROACH

        # 超声波"够近"确认：夹取前读一次前方距离，没到阈值就小步接近，避免空抓
        pick_dist = self.params['arm'].get('pick_distance', 20.0)
        dist = self.walker.distance_cm()
        if dist > pick_dist:
            print('[PICK] 前方距离 %.1f cm > 阈值 %.1f cm，小步接近' % (dist, pick_dist))
            self.ik.go_forward(self.ik.initial_pos, 2, 30, 40, 1)
            return STATE_PICK

        print('[PICK] 抓取目标 (%.1f, %.1f) cm，前方距离 %.1f cm' % (wx, wy, dist))
        if self.picker.pick_at(wx, wy):
            self.pick_count += 1
            self.picker.release()
            self.picker.reset_pose()
            print('[PICK] 完成第 %d 个目标' % self.pick_count)
            if task_server is not None:
                task_server.set_status(last_result='done',
                                       message='抓取成功，已抓 %d 个' % self.pick_count)
            # 后退一步，避免机械臂/身体遮挡下一个目标
            self.ik.back(self.ik.initial_pos, 2, 60, 50, 1)
        else:
            self.picker.reset_pose()
            print('[PICK] 抓取失败，重新对准')
            if task_server is not None:
                task_server.set_status(last_result='failed', message='抓取失败')
        return STATE_SEARCH

    def _step_nav(self, img):
        """等任务 → 转向目标 → 避障前进 → 进入粗定位半径后切 SEARCH。"""
        if self.current_task is None:
            if task_server is None:
                return STATE_SEARCH
            self.current_task = task_server.get_next_task()
            if self.current_task is None:
                return STATE_NAV            # 没任务就保持等待
            t = self.current_task.get('target', {})
            self.target_x = float(t.get('x', 0.0))
            self.target_y = float(t.get('y', 0.0))
            self.arrive_radius = float(self.current_task.get(
                'arrive_radius', self.nav.get('arrive_radius', 0.8)))
            self.walked_m = 0.0
            print('[NAV] 任务 %s 目标 (%.2f, %.2f) m，到达半径 %.2f m' % (
                self.current_task.get('task_id', '?'),
                self.target_x, self.target_y, self.arrive_radius))

        # 到达判定：进入粗定位半径 → 交给近景视觉精定位
        dist = math.hypot(self.target_x - self.pos_x, self.target_y - self.pos_y)
        if dist <= self.arrive_radius:
            print('[NAV] 已到达目标区域 (%.2fm)，开始近景搜索' % dist)
            self.current_task = None
            return STATE_SEARCH

        if self.walked_m >= self.nav.get('max_walk_m', 15.0):
            print('[NAV] 超出最大行走距离，上报失败')
            if task_server is not None:
                task_server.set_status(last_result='failed', message='导航超时')
            self.current_task = None
            return STATE_NAV

        # 1) 转到目标方向（负反馈，和 APPROACH 同一个思路）
        bearing = math.degrees(math.atan2(self.target_y - self.pos_y,
                                          self.target_x - self.pos_x))
        rel = (bearing - self.heading + 180.0) % 360.0 - 180.0   # 归一化到 [-180,180]
        tol = self.nav.get('heading_tolerance', 5.0)
        if abs(rel) > tol:
            step = min(abs(rel), self.params['walk']['turn_angle'])
            self.walker.turn(left=rel > 0, angle=step)
            self.heading = (self.heading + (step if rel > 0 else -step) + 360.0) % 360.0
            return STATE_NAV

        # 2) 对准了 → 前进（自带超声波避障）
        moved = self.walker.walk_forward()
        if moved:
            step_m = self.params['walk']['stride'] / 1000.0
            self.walked_m += step_m
            self.pos_x += step_m * math.cos(math.radians(self.heading))
            self.pos_y += step_m * math.sin(math.radians(self.heading))
        return STATE_NAV

    def _sync_status(self):
        """把运行状态同步给 task_server，供地面站仪表盘 GET /status 轮询。"""
        if task_server is None:
            return
        task_server.set_status(
            state=self.state,
            position_m={'x': round(self.pos_x, 2), 'y': round(self.pos_y, 2)},
            heading_deg=round(self.heading, 1),
            picked_count=self.pick_count,
            last_task=self.current_task,
        )

    def run(self):
        camera = Camera()
        camera.camera_open()
        try:
            while True:
                img = camera.frame
                if img is None:
                    time.sleep(0.01)
                    continue
                frame = cv2.remap(img.copy(), self.mapx, self.mapy, cv2.INTER_LINEAR)
                frame = cv2.flip(frame, 0)  # 摄像头画面上下颠倒，垂直翻正后再检测/转向
                if self.state == STATE_SEARCH:
                    self.state = self._step_search(frame)
                elif self.state == STATE_APPROACH:
                    self.state = self._step_approach(frame)
                elif self.state == STATE_PICK:
                    self.state = self._step_pick(frame)
                elif self.state == STATE_NAV:
                    self.state = self._step_nav(frame)
                self._sync_status()

                cv2.putText(frame, 'State: %s' % self.state, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, 'Picked: %d' % self.pick_count, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                # 把带状态标注的画面喂给 HTTP 服务，机器人独立仪表盘即可看到回传画面
                if task_server is not None:
                    task_server.publish_frame(frame)
                cv2.imshow('AutonomousPick', frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC 退出
                    break
        finally:
            camera.camera_close()
            self.reset_pose()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='自主行走+视觉识别+机械臂抓取')
    parser.add_argument('--color', default='red', help='颜色检测目标 (red/green/blue)')
    parser.add_argument('--detector', default='color', choices=['color', 'yolo'],
                        help='检测器类型')
    parser.add_argument('--model', default=None, help='YOLO 模型路径（detector=yolo 时使用）')
    parser.add_argument('--classes', default='pod', help='YOLO 目标类别，逗号分隔')
    parser.add_argument('--no-wait-task', action='store_true',
                        help='启动后不等任务，直接开始扫描（旧行为）')
    args = parser.parse_args()

    target_classes = tuple(c.strip() for c in args.classes.split(',') if c.strip())
    app = AutonomousPick(color=args.color, detector=args.detector,
                         model_path=args.model, target_classes=target_classes,
                         wait_task=not args.no_wait_task)
    app.reset_pose()
    print('AutonomousPick Start (color=%s, detector=%s, wait_task=%s)，ESC 退出'
          % (args.color, args.detector, not args.no_wait_task))
    app.run()


if __name__ == '__main__':
    main()