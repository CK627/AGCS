#!/usr/bin/python3
# coding=utf8
"""超声波自主避障行走（自主行走模块）。

可独立运行（树莓派终端）：
    python3 ~/spiderpi/functions/obstacle_avoidance.py

也可作为模块被 autonomous_pick.py 调用。
"""
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from functions.robot_config import load_params


class ObstacleAvoidance:
    """基于 kinematics.IK 与超声波传感器的六足避障行走控制。"""

    def __init__(self, board, ik, ultrasonic, params=None):
        self.board = board
        self.ik = ik
        self.ultrasonic = ultrasonic
        self.params = params if params is not None else load_params()
        # 取 YAML 里 walk 这一整组参数（一个字典），
        # 后面 self.walk['stride'] 这种写法就是"取 walk 组里的 stride 键"
        self.walk = self.params['walk']
        # 同理取出 obstacle 组（避障阈值、后退步幅等）
        self.obs = self.params['obstacle']
        self._dist_window = []

    def distance_cm(self):
        """返回滤波后的前方距离（cm）。"""
        d = self.ultrasonic.getDistance() / 10.0
        self._dist_window.append(d)
        n = self.obs['filter_window']  # obstacle 组里的滤波窗口大小（YAML: obstacle.filter_window）
        if len(self._dist_window) > n:
            self._dist_window.pop(0)
        return sum(self._dist_window) / float(len(self._dist_window))

    def blocked(self):
        """前方距离小于阈值即视为被阻挡。"""
        # obstacle.threshold：距离小于该值（cm）视为有障碍
        return 0 < self.distance_cm() < self.obs['threshold']

    def turn(self, left=True, angle=None, speed=None):
        """原地转向，angle 单位度。"""
        angle = self.walk['turn_angle'] if angle is None else angle   # walk.turn_angle 默认转向角
        speed = self.walk['turn_speed'] if speed is None else speed   # walk.turn_speed 默认转向速度
        if left:
            self.ik.turn_left(self.ik.initial_pos, 2, angle, speed, 1)
        else:
            self.ik.turn_right(self.ik.initial_pos, 2, angle, speed, 1)

    def walk_forward(self, stride=None, speed=None):
        """前进一步；受阻则后退并左转 45 度。返回是否成功前进。"""
        stride = self.walk['stride'] if stride is None else stride   # walk.stride 直行步幅 mm
        speed = self.walk['speed'] if speed is None else speed       # walk.speed 直行速度 mm/s
        if self.blocked():
            self.ik.back(self.ik.initial_pos, 2, self.obs['back_stride'], 50, 1)  # obstacle.back_stride 后退步幅
            self.turn(left=True, angle=45)
            return False
        self.ik.go_forward(self.ik.initial_pos, 2, stride, speed, 1)
        return True


def main():
    from common.ros_robot_controller_sdk import Board
    from common import kinematics
    from sensor.ultrasonic_sensor import Ultrasonic

    board = Board()
    ik = kinematics.IK(board)
    ultrasonic = Ultrasonic()
    walker = ObstacleAvoidance(board, ik, ultrasonic)

    ik.stand(ik.initial_pos, t=500)
    print('ObstacleAvoidance Start，Ctrl+C 退出')
    try:
        while True:
            moved = walker.walk_forward()
            print('前方距离 %.1f cm, %s' % (walker.distance_cm(), '前进' if moved else '避障'))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ik.stand(ik.initial_pos, t=500)


if __name__ == '__main__':
    main()
