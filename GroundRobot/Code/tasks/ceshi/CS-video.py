#!/usr/bin/python3
# coding=utf8
"""临时测试摄像头回传链路：只推流不动机器人。

验证链路：
    本脚本（唯一占摄像头的进程，每帧 publish_frame）
      → task_server /video.mjpeg（5000 端口）
      → 地面站 Dashboard/backend/app.py /video_feed
      → 浏览器 http://127.0.0.1:20002

树莓派上运行（同时只能跑一个占摄像头的程序）：
    python3 CS-video.py   # 有桌面（VNC）开预览窗口 ESC 退出；SSH 无桌面自动纯推流 Ctrl+C 退出

启动后按打印的 IP 改 Dashboard/backend/config.py 的 ROBOT_URL，
再在地面站跑 app.py、浏览器开 http://127.0.0.1:20002 看画面。
也可先用浏览器直接开 http://<机器人IP>:5000/video.mjpeg 单独验证机器人端。
"""
import argparse
import os
import socket
import sys
import time

import cv2

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import load_params, correct_camera, load_undistort_maps, open_camera

try:
    from communication import task_server
except ImportError:
    task_server = None


def lan_ip():
    """取本机局域网 IP（不真正发包，只为让系统选出走局域网的地址）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def main():
    parser = argparse.ArgumentParser(description='摄像头回传链路临时测试')
    parser.add_argument('--no-show', action='store_true', help='强制不开本地预览窗口')
    parser.add_argument('--fps', type=float, default=10.0, help='推流帧率上限（默认 10）')
    args = parser.parse_args()

    # SSH 无桌面时 DISPLAY 未设置，cv2.imshow 会让 Qt 直接 abort 整个进程，
    # 因此按 DISPLAY 自动降级为纯推流；VNC 桌面下 DISPLAY 有值，正常开预览窗口
    show = not args.no_show and bool(os.environ.get('DISPLAY'))

    if task_server is None:
        print('[CS-video] 未找到 communication/task_server，无法推流')
        return
    if task_server.start_server() is None:
        print('[CS-video] Flask 未安装，HTTP 服务没起来：pip3 install flask')
        return

    params = load_params()
    rotate = params['vision'].get('camera_rotate', 0)
    mapx, mapy = load_undistort_maps()
    cam = open_camera()

    ip = lan_ip()
    print('[CS-video] 仪表盘 config.py 的 ROBOT_URL 填: http://%s:5000' % ip)
    if not show:
        print('[CS-video] 未检测到桌面（SSH 环境），已自动跳过本地预览窗口，Ctrl+C 退出')
    print('[CS-video] 推流中...')

    frames = 0
    t0 = time.time()
    fps = 0.0
    last_status = 0.0
    try:
        while True:
            f = cam.frame
            if f is None:
                time.sleep(0.01)
                continue
            # 与 auto_fetch.py 检测管线一致：畸变矫正 + 方向校正，仪表盘看到的就是机器人看到的
            frame = cv2.remap(correct_camera(f, rotate), mapx, mapy, cv2.INTER_LINEAR)

            frames += 1
            now = time.time()
            if now - t0 >= 1.0:
                fps = frames / (now - t0)
                frames = 0
                t0 = now
            cv2.putText(frame, 'CS-video %.1f fps' % fps, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            task_server.publish_frame(frame, max_fps=args.fps)
            if now - last_status >= 1.0:
                last_status = now
                task_server.set_status(state='VIDEO_TEST',
                                       message='摄像头回传测试 %.1f fps' % fps)

            if not show:
                time.sleep(0.01)
            else:
                cv2.imshow('CS-video', frame)
                if cv2.waitKey(1) == 27:  # ESC
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cam.camera_close()
        if show:
            cv2.destroyAllWindows()
        print('[CS-video] 已退出，摄像头已释放')


if __name__ == '__main__':
    main()
