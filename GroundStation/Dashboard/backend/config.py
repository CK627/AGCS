# coding=utf8
"""地面站仪表盘配置（改这里的地址即可，不用改代码）。"""

# 机器人任务/状态服务（树莓派 task_server.py，autonomous_pick 启动时自动开启）
ROBOT_URL = 'http://10.194.228.87:5000'

# 无人机 MAVLink 数据源（pymavlink 连接串，替代 ROS MAVROS）
# - 数传默认端口：QGC 用 8080，pymavlink 建议监听 14550（数传端需把地面站 IP+14550 加为目标）
# - SITL 模拟调试：'udpout:127.0.0.1:14550'
DRONE_MAVLINK = 'udpin:0.0.0.0:14550'

# 无人机图传 RTSP（机载摄像头，经 Minihomer 数传可达）
DRONE_RTSP = 'rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=1.sdp?'

# 仪表盘自身 HTTP 地址与端口（网络部门如需跨网段访问，按 7.8 转发这个端口）
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 20001

# 视频预览帧率上限（预览够用即可，别占满带宽）
VIDEO_FPS_LIMIT = 10

# YOLO 模型信息（显示用；检测线程实际加载此路径的模型）
MODEL_INFO = {
    'name': 'pod_pest_v4 (best.pt)',
    'path': r'D:\yolo\runs\detect\runs\pod_pest_v4\weights\best.pt',
    'classes': ['worm'],
    'conf': 0.45,
    'imgsz': 1280,
}
