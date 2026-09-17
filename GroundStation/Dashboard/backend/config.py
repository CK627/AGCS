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
DRONE_URL = 'http://127.0.0.1:20002'
DRONE_VIDEO_PORT = 20005   # 无人机图传视频流独立端口（Dron/Dashboard 的 drone.video_port）

# 无人机 MAVLink 数据源（pymavlink 连接串，替代 ROS MAVROS）
# - 数传默认端口：QGC 用 8080，pymavlink 建议监听 14550（数传端需把地面站 IP+14550 加为目标）
# - SITL 模拟调试：'udpout:127.0.0.1:14550'
DRONE_MAVLINK = 'udpin:0.0.0.0:14550'

# 视频开关
ROBOT_VIDEO_ENABLED = True
DRONE_VIDEO_ENABLED = True
YOLO_VIDEO_ENABLED = True

# 服务器监控：CPU / 内存 / 磁盘 / 运行时间等系统信息
#   local：中枢就跑在服务器这台机器上 → 直接读本机（无需网络，最省事）
#   http ：服务器上另跑了 GroundStation/Server → 中枢去拉它的 /api/system
#   auto ：server.url 指向本机时用 local，否则用 http
SERVER_URL = 'http://192.168.124.7:20006'
SERVER_NAME = '服务器'
SERVER_ENABLED = True
SERVER_MODE = 'auto'

# YOLO 模型仪表盘（YOLOModel/Dashboard，拉无人机图传做检测并广播自己）
# 中枢从这里拉检测数据与标注画面；不在同一台电脑时改成那台电脑的局域网 IP
YOLO_URL = 'http://127.0.0.1:20003'

# 仪表盘自身 HTTP 地址与端口（20000=地面站中枢，20001=地面机器人，20002=无人机，20003=YOLO 模型）
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 20000

# 研发进度流程图（独立网页，独立端口；由同一套启动脚本一起启停）
# 页面在 Dashboard/progress/，进度数据从本中枢 /api/progress/flow 取
PROGRESS_PORT = 20010

# 视频预览帧率上限（预览够用即可，别占满带宽）
VIDEO_FPS_LIMIT = 10    # /video.mjpeg 转发帧率（yolo.fps 可配置）

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

    global ROBOT_URL, DRONE_URL, YOLO_URL, DRONE_VIDEO_PORT, ROBOT_VIDEO_ENABLED, DRONE_VIDEO_ENABLED, YOLO_VIDEO_ENABLED, DASHBOARD_PORT, PROGRESS_PORT, SERVER_URL, SERVER_NAME, SERVER_ENABLED, SERVER_MODE
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
    if drone.get('video_port') is not None:
        DRONE_VIDEO_PORT = int(drone['video_port'])
    yolo = data.get('yolo') or {}
    if yolo.get('url'):
        YOLO_URL = str(yolo['url']).rstrip('/')
    if 'video' in yolo:
        YOLO_VIDEO_ENABLED = bool(yolo['video'])
    if yolo.get('fps') is not None:
        VIDEO_FPS_LIMIT = int(yolo['fps'])
    server = data.get('server') or {}
    if server.get('url'):
        SERVER_URL = str(server['url']).rstrip('/')
    if server.get('name'):
        SERVER_NAME = str(server['name'])
    if 'enabled' in server:
        SERVER_ENABLED = bool(server['enabled'])
    if server.get('mode'):
        SERVER_MODE = str(server['mode']).strip().lower()
    dash = data.get('dashboard') or {}
    if dash.get('hub_port'):
        DASHBOARD_PORT = int(dash['hub_port'])
    if dash.get('progress_port'):
        PROGRESS_PORT = int(dash['progress_port'])
    print('[config] 已加载 %s (robot.url=%s drone.url=%s yolo.url=%s server.url=%s)'
          % (_CONFIG_YAML, ROBOT_URL, DRONE_URL, YOLO_URL, SERVER_URL))


reload_if_changed()
