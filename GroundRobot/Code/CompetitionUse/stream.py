#!/usr/bin/python3
# coding=utf8
"""stream.py —— 把机器人彩色相机推到 /video.mjpeg，只推流、不动舵机、不检测。

复用 SourceCode/vision_common.py 的 open_vision（取帧 + 去畸变 + 推流都封装在里面），
不调用 detector、不碰舵机。

跑法（在机器人上）：
    cd ~/spiderpi/CompetitionUse
    python3 stream.py
本地浏览器/程序打开：http://<机器人IP>:5000/video.mjpeg
"""

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

_SOURCE_DIR = os.path.join(_HERE, 'SourceCode')
if _SOURCE_DIR not in sys.path:
    sys.path.insert(0, _SOURCE_DIR)
from vision_common import open_vision, task_server

cam, read_frame, _detector, publish = open_vision('red', 1)

if task_server is not None:
    task_server.start_server()
    print('推流已启动: http://<机器人IP>:5000/video.mjpeg', flush=True)
else:
    print('!! 未安装 Flask / task_server，无法推流', flush=True)

try:
    while True:
        f = read_frame()
        if f is not None:
            publish(f)
        time.sleep(0.03)
except KeyboardInterrupt:
    print('退出', flush=True)
finally:
    cam.camera_close()
