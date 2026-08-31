#!/usr/bin/python3
# coding=utf8
"""无人机电脑端仪表盘配置。

临时可变的配置（图传摄像头序号 / 视频开关 / 端口）集中在仓库根目录的
Dorn/Dashboard/data/config.yaml，改那个文件即可（保存即热重载，端口除外）；
本文件只放默认值和硬件绑定项（MAVLink 连接串）。
"""
import os

# ---------------- 默认值（config.yaml 缺失或未写的项用这里） ----------------

# 无人机 MAVLink 数据来源（电脑监听 UDP 14550，收飞控遥测）
MAVLINK_CONN = 'udpin:0.0.0.0:14550'

# EWRF 图传接收机的 USB 摄像头序号（yaml drone.camera 热调整）
CAMERA_INDEX = 0

# 视频开关：接收机没插/不想看时设 False（yaml drone.video 热调整）
VIDEO_ENABLED = True

# 画面顶部裁剪像素数：去掉图传画面左上角 OSD 文字（如 0,0,0）；0 = 不裁剪
VIDEO_CROP_TOP = 0

# 网页服务地址与端口：0.0.0.0 允许地面站中枢从局域网拉流（端口走 yaml dashboard.drone_port）
FLASK_HOST = '0.0.0.0'
FLASK_PORT = 20000

# ---------------- 本仪表盘 data/config.yaml 热重载 ----------------
# backend 上一级进 data = Dorn/Dashboard/data
_CONFIG_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'config.yaml'))
_last_mtime = None


def reload_if_changed():
    """本仪表盘 data/config.yaml 有改动时重新加载覆盖项。

    仪表盘请求/视频线程循环时调用，所以改 yaml 保存即生效，无需重启进程；
    端口在启动时绑定，改端口仍需重启。摄像头序号变化由视频线程自动重开设备。
    """
    global _last_mtime
    try:
        mtime = os.path.getmtime(_CONFIG_YAML)
    except OSError:
        return  # 文件不存在：用默认值，之后创建了也会被自动加载
    if mtime == _last_mtime:
        return
    _last_mtime = mtime
    try:
        import yaml
        with open(_CONFIG_YAML, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception as e:
        print('[config] %s 读取失败，沿用当前配置: %s' % (_CONFIG_YAML, e))
        return

    global CAMERA_INDEX, VIDEO_ENABLED, VIDEO_CROP_TOP, FLASK_PORT
    drone = data.get('drone') or {}
    if drone.get('camera') is not None:
        CAMERA_INDEX = int(drone['camera'])
    if 'video' in drone:
        VIDEO_ENABLED = bool(drone['video'])
    if drone.get('video_crop_top') is not None:
        VIDEO_CROP_TOP = int(drone['video_crop_top'])
    dash = data.get('dashboard') or {}
    if dash.get('drone_port'):
        FLASK_PORT = int(dash['drone_port'])
    print('[config] 已加载 %s (drone.camera=%s drone.video=%s)'
          % (_CONFIG_YAML, CAMERA_INDEX, VIDEO_ENABLED))


reload_if_changed()
