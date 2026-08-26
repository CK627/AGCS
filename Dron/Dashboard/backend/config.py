#!/usr/bin/python3
# coding=utf8
"""无人机网页端配置：改这里即可。"""

# 无人机 MAVLink 数据来源（电脑监听 UDP 14550，收飞控"小纸条"）
MAVLINK_CONN = "udpin:0.0.0.0:14550"

# 摄像头 RTSP 地址（官方手册默认值）
RTSP_URL = "rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=1.sdp?"

# 视频开关：没有摄像头/不想开视频时先关掉，避免启动报错
VIDEO_ENABLED = False

# 网页服务地址与端口
FLASK_HOST = "127.0.0.1"
FLASK_PORT = 20000
