# -*- coding: utf-8 -*-
"""无人机 SDK 遥测（数据源与 外接图传录制程序/capture_external_camera.py 一致）。

通过 lib/ 里的 OpenFly SDK（helloFly）直接读无人机的：
- 位置 loc_x / loc_y / loc_z（厘米）
- 姿态 rol / pit / yaw（度）
- 电压 vol（伏特）
- 定位误差 err_x / err_y / err_z
- 避障距离（前后左右）
- 遥控器按键、消息内容、计时器

pymavlink / MAVLink 不可用或收不到心跳时，仪表盘回退到这个数据源，
保证“无人机数据”照样能显示。
"""

import os
import sys
import threading
import time

# 把随 backend 自带的 SDK 目录加进搜索路径（backend/lib）
LIB_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'lib'))
if LIB_DIR not in sys.path:
    sys.path.insert(0, LIB_DIR)


# getFlySensor 可读的全部字段（key -> 文档里的 type）
FLY_SENSOR_TYPES = (
    ('loc_x', 'loc_x'), ('loc_y', 'loc_y'), ('loc_z', 'loc_z'),
    ('roll', 'rol'), ('pitch', 'pit'), ('yaw', 'yaw'),
    ('volt', 'vol'),
    ('err_x', 'err_x'), ('err_y', 'err_y'), ('err_z', 'err_z'),
)


class SdkTelemetry(threading.Thread):
    """后台线程：连接 OpenFly SDK 并持续轮询传感器数据。"""

    def __init__(self, drone_id=0, on_update=None):
        super().__init__(daemon=True)
        self.drone_id = drone_id
        self.on_update = on_update          # 回调（线程外更新共享数据）
        self.lock = threading.Lock()
        self.data = {
            'connected': False,
            'error': '',
            'serial': '',
        }
        self._drone = None

    # ---------------- 供外部读取 ----------------
    def snapshot(self):
        with self.lock:
            return dict(self.data)

    # ---------------- 后台线程 ----------------
    def run(self):
        while True:
            try:
                if self._drone is None:
                    self._connect()
                self._poll()
            except SystemExit:
                self._fail('SDK 连接失败：找不到无人机串口，或 OpenFly 未正确连接')
            except Exception as exc:  # noqa: BLE001
                self._fail('SDK 读取异常：%s' % exc)
            time.sleep(0.5)

    def _connect(self):
        import helloFly
        import drone_link

        # 只做识别拿到串口名（不占用串口），真正连接交给 helloFly
        serials = drone_link.find_drone_serial()
        serial_name = serials[0] if serials else ''
        drone = helloFly.fly(maxNum=1)
        self._drone = drone
        with self.lock:
            self.data.update({
                'connected': True,
                'error': '',
                'serial': serial_name,
            })
        print('[SDK] 无人机 SDK 连接成功（串口 %s）' % (serial_name or '未知'))
        if self.on_update:
            self.on_update(self.snapshot())

    def _poll(self):
        d = self._drone
        if d is None:
            return
        row = {}
        for key, sensor in FLY_SENSOR_TYPES:
            row[key] = self._safe(d.getFlySensor, self.drone_id, sensor)
        for key, direction in (('obs_f', 0), ('obs_b', 1), ('obs_l', 2), ('obs_r', 3)):
            row[key] = self._safe(d.getObsDistance, self.drone_id, direction)
        row['key_press'] = self._safe(d.getKeyPress, self.drone_id)
        row['role_news'] = self._safe(d.getRoleNews, self.drone_id, 'details')
        row['role_news_id'] = self._safe(d.getRoleNews, self.drone_id, 'id')
        row['timer'] = self._safe(d.getTimer)
        with self.lock:
            self.data.update(row)
            self.data['connected'] = True
            self.data['error'] = ''
            self.data['checked_at'] = time.time()
        if self.on_update:
            self.on_update(self.snapshot())

    def _fail(self, message):
        with self.lock:
            self.data.update({'connected': False, 'error': message})
        print('[SDK] %s，10 秒后重试' % message)
        self._drone = None
        if self.on_update:
            self.on_update(self.snapshot())
        time.sleep(10)

    @staticmethod
    def _safe(fn, *args):
        try:
            return fn(*args)
        except Exception:  # noqa: BLE001
            return None
