# coding=utf8
"""地面机器人独立仪表盘配置。

临时可变的配置（机器人 IP / 视频开关 / 端口）集中在仓库根目录的
AGCS/config.yaml，改那个文件即可（保存即热重载，端口除外）；
本文件只放默认值。无人机 MAVLink / RTSP / YOLO 模型等地面站专属配置
在 GroundStation/Dashboard/backend/config.py。
"""
import os

# ---------------- 默认值（config.yaml 缺失或未写的项用这里） ----------------

# 机器人任务/状态/视频服务地址（task_server.py，autonomous_pick / CS-video 启动时自动开启）
# 仪表盘跑在地面站电脑上就填机器人当前局域网 IP；直接在机器人本机打开则填 127.0.0.1
ROBOT_URL = 'http://10.194.228.89:5000'

# 视频开关：机器人端画面没开/不想看视频时设为 False，避免页面一直转圈
VIDEO_ENABLED = True

# 仪表盘自身监听地址与端口（默认 20002，与无人机端 20000、地面站中枢 20001 区分）
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 20002

# ---------------- AGCS/config.yaml 热重载 ----------------
# Dashboard/backend 向上三级 = AGCS 仓库根目录
_CONFIG_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'config.yaml'))
_last_mtime = None


def reload_if_changed():
    """AGCS/config.yaml 有改动时重新加载覆盖项。

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

    global ROBOT_URL, VIDEO_ENABLED, DASHBOARD_PORT
    robot = data.get('robot') or {}
    if robot.get('url'):
        ROBOT_URL = str(robot['url']).rstrip('/')
    if 'video' in robot:
        VIDEO_ENABLED = bool(robot['video'])
    dash = data.get('dashboard') or {}
    if dash.get('robot_port'):
        DASHBOARD_PORT = int(dash['robot_port'])
    print('[config] 已加载 %s (robot.url=%s robot.video=%s)'
          % (_CONFIG_YAML, ROBOT_URL, VIDEO_ENABLED))


reload_if_changed()
