#!/usr/bin/python3
# coding=utf8
"""地面机器人独立仪表盘配置：改这里即可，不用改代码。"""

# 机器人任务/状态/视频服务地址（task_server.py，autonomous_pick 启动时自动开启）
# 仪表盘跑在地面站电脑上就填机器人当前局域网 IP；直接在机器人本机打开则填 127.0.0.1
ROBOT_URL = 'http://10.194.228.89:5000'

# 视频开关：机器人端画面没开/不想看视频时设为 False，避免页面一直转圈
VIDEO_ENABLED = True

# 仪表盘自身监听地址与端口（默认 20002，与无人机端 20000、地面站中枢 20001 区分）
DASHBOARD_HOST = '0.0.0.0'
DASHBOARD_PORT = 20002
