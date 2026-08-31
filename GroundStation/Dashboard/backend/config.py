# coding=utf8
"""地面站仪表盘配置。

临时可变的配置（机器人 IP / 无人机电脑地址 / 视频开关 / 端口）集中在
GroundStation/Dashboard/data/config.yaml，改那个文件即可（保存即热重载，端口除外）；
本文件只放默认值和与硬件绑定的固定项。
"""
import os

# ---------------- 默认值（config.yaml 缺失或未写的项用这里） ----------------

# 机器人任务/状态/视频服务（树莓派 task_server.py，autonomous_pick / CS-video 启动时自动开启）
ROBOT_URL = 'http://10.194.228.89:5000'

# 无人机电脑端仪表盘（Dron/Dashboard，独占 EWRF 图传接收机的那个服务）
# 中枢从这里拉图传画面；不在同一台电脑时改成那台电脑的局域网 IP
DRONE_URL = 'http://127.0.0.1:20000'

# 无人机 MAVLink 数据源（pymavlink 连接串，替代 ROS MAVROS）
# - 数传默认端口：QGC 用 8080，pymavlink 建议监听 14550（数传端需把地面站 IP+14550 加为目标）
# - SITL 模拟调试：'udpout:127.0.0.1:14550'
DRONE_MAVLINK = 'udpin:0.0.0.0:14550'

# 视频开关
ROBOT_VIDEO_ENABLED = True
DRONE_VIDEO_ENABLED = True

# 仪表盘自身 HTTP 地址与端口（20001=地面站中枢，20000=无人机端，20002=机器人独立仪表盘）
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 20001

# 视频预览帧率上限（预览够用即可，别占满带宽）
VIDEO_FPS_LIMIT = 10

# YOLO 模型信息（显示用；检测线程实际加载此路径的模型。
# 未装 ultralytics 或路径无效时自动退化为原始画面转发，不影响看画面）
MODEL_INFO = {
    'name': 'pod_pest_v4 (best.pt)',
    'path': r'D:\yolo\runs\detect\runs\pod_pest_v4\weights\best.pt',
    'classes': ['worm'],
    'conf': 0.45,
    'imgsz': 1280,
}

# ---------------- 本仪表盘 data/config.yaml 热重载 ----------------
# backend 上一级进 data = GroundStation/Dashboard/data
_CONFIG_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'config.yaml'))
_last_mtime = None


def reload_if_changed():
    """本仪表盘 data/config.yaml 有改动时重新加载覆盖项。

    仪表盘请求时调用（app.py 各代理路由入口），所以改 yaml 保存即生效，
    无需重启进程；端口在启动时绑定，改端口仍需重启。
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

    global ROBOT_URL, DRONE_URL, ROBOT_VIDEO_ENABLED, DRONE_VIDEO_ENABLED, DASHBOARD_PORT
    robot = data.get('robot') or {}
    if robot.get('url'):
        ROBOT_URL = str(robot['url']).rstrip('/')
    if 'video' in robot:
        ROBOT_VIDEO_ENABLED = bool(robot['video'])
    drone = data.get('drone') or {}
    if drone.get('url'):
        DRONE_URL = str(drone['url']).rstrip('/')
    if 'video' in drone:
        DRONE_VIDEO_ENABLED = bool(drone['video'])
    dash = data.get('dashboard') or {}
    if dash.get('hub_port'):
        DASHBOARD_PORT = int(dash['hub_port'])
    print('[config] 已加载 %s (robot.url=%s drone.url=%s robot.video=%s drone.video=%s)'
          % (_CONFIG_YAML, ROBOT_URL, DRONE_URL, ROBOT_VIDEO_ENABLED, DRONE_VIDEO_ENABLED))


reload_if_changed()
