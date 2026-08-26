#!/usr/bin/python3
# coding=utf8
"""无人机连接配置：改这个文件即可切换连接方式。"""

# 连接字符串（二选一）：
#   数传 UDP（推荐）：udpin:0.0.0.0:14550
#     说明：QGC 占用 8080，pymavlink 用 MAVLink 默认地面站端口 14550；
#     需要在数传端把"地面站 IP + 14550"加为额外发送目标（或改数传目标端口）。
#   USB 直连（Windows）：serial:COM3,57600  （设备管理器查 COM 号）
#   USB 直连（Linux）  ：serial:/dev/ttyUSB0,57600
#   SITL 仿真（练习）  ：udpout:127.0.0.1:14550
CONNECTION = "udpin:0.0.0.0:14550"

# 串口波特率（USB 直连用；UDP 方式用不到）
BAUD = 57600

# 等待心跳超时（秒）
TIMEOUT = 10

# 飞控的 system/component id，PX4 默认都是 1
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1
