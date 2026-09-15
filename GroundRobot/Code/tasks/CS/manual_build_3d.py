#!/usr/bin/python3
# coding=utf8
"""手动遥控走 + 采 RGB-D 建 3D 模型（键盘 WASD/QE 开机器人）。

跟 rotate_capture_rgbd.py（原地自动转一圈）的区别：这个是你自己开着走 —— 房间大、
有遮挡、想重点扫某块就用这个，转向用 IMU 闭环保证准，走一步自动存一帧彩色+深度。

存出来的目录直接喂给本机（Mac）的 reconstruct_rgbd.py 做 TSDF 融合：
    scp -r pi@<IP>:/tmp/rgbd3d ./
    ~/open3d_env/bin/python reconstruct_rgbd.py --in ./rgbd3d --out map.ply
位姿也一起存进 poses.json（reconstruct_rgbd.py 目前只用 RGB-D 里程计，没接
poses.json；里程计跑飞了再让它读这个文件，是按录的位姿直接融合的备选）。

一次动作（走一步 + 等站稳 + 拍一帧）约 1.4 秒。**按键是「轻点走一步、按住连续走」**：
每读完一个键会把终端里排队的自动重复键丢掉，所以不会出现「只点一下却一直走」。

用法（先 sudo systemctl stop spiderpi，串口 /dev/ttyAMA0 同一时刻只能一个进程占）：
    python3 manual_build_3d.py --out /tmp/rgbd3d --step 40 --angle 20

    --out 目录里已有上次采集时会拒绝启动（帧从 depth_00000 编号，重跑会把两次的帧
    混在一起），换目录或加 --force 清掉旧的。

按键：
    w 前进      s 后退      a 左横移    d 右横移
    q 左转 --angle°   e 右转 --angle°（IMU 闭环，误差 <=1°）
    c 补采一帧   r 立正      y 航向清零   空格 急停   x 退出
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty

import cv2
import numpy as np

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from agcs_lib import (DepthCamera, make_board, make_ik, stand,
                      turn_left, turn_right, go_forward, go_back)
from agcs_lib.pcl import rot_y

POLL_S = 0.05  # 主循环节奏；每轮都积分 IMU，别让航向漏积分
GYRO_GAIN = 1.18  # 角速度标度补偿（跟 rotate_capture_rgbd.py 一致）


def getch_timeout(timeout=POLL_S):
    """带超时读一个键，超时返回 None（不阻塞，好让 IMU 一直积分）。

    用 os.read 直接读 fd，不走 sys.stdin：Python 的文本层会一次预读一整块进它
    自己的缓冲，预读进去的字符 select 看不见，下面 drain_input 就会漏掉它们。
    """
    r, _, _ = select.select([sys.stdin.fileno()], [], [], timeout)
    if not r:
        return None
    try:
        b = os.read(sys.stdin.fileno(), 1)
    except OSError:
        return None
    return b.decode('utf-8', 'ignore') if b else None


def drain_input(max_bytes=8192):
    """丢掉输入缓冲里排队的按键，返回丢掉的个数。

    终端按住不放会自动重复，一秒能塞进来二三十个字符；而本脚本一次动作要 ~1.4 秒
    （走一步 + 等站稳 + 拍一帧），这段时间攒下的重复键会在你松手之后一个接一个地
    执行 —— 表现就是「只点了一下，却一直走很多步」。所以每读完一个键就把剩下的
    排队键丢掉：轻点 = 走一步，按住 = 连续走（松手最多多走一步）。
    """
    n = 0
    fd = sys.stdin.fileno()
    while n < max_bytes:
        r, _, _ = select.select([fd], [], [], 0)
        if not r:
            break
        try:
            b = os.read(fd, 256)
        except OSError:
            break
        if not b:
            break
        n += len(b)
    return n


def init_imu(board):
    """标定 gz 零漂，返回 imu_state（需要机器人静止）。"""
    board.enable_reception()
    vals = []
    deadline = time.monotonic() + 3.0
    while len(vals) < 200 and time.monotonic() < deadline:
        try:
            data = board.get_imu()
        except Exception:
            data = None
        if data is not None:
            vals.append(float(data[5]))
        time.sleep(0.005)
    if not vals:
        return None
    st = {'bias': sum(vals) / len(vals), 'yaw': 0.0, 'last_t': time.monotonic()}
    print('IMU 零漂标定完成（%d 样本，bias=%.4f）' % (len(vals), st['bias']), flush=True)
    return st


def update_yaw(st, board):
    """按 dt 积分 gz 得到航向（度，左正右负），返回当前航向。"""
    now = time.monotonic()
    dt = now - st['last_t']
    st['last_t'] = now
    try:
        data = board.get_imu()
    except Exception:
        data = None
    if data is not None:
        st['yaw'] += (float(data[5]) - st['bias']) * dt * GYRO_GAIN
    return st['yaw']


def turn_by(ik, board, st, delta_deg, tol=1.0, timeout=8.0, speed=60):
    """IMU 闭环转 delta_deg（正=左转），返回实际转过的角度（度）。

    目标从「当前实际朝向」算起，转多了/转少了自动补步，所以连按 q 十次也不会越转
    越偏（开环固定步数会，六足每步都有转角偏差）。
    """
    start = st['yaw']
    target = start + delta_deg
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        yaw = update_yaw(st, board)
        err = target - yaw
        if abs(err) <= tol:
            break
        step = max(1, min(5, int(round(abs(err)))))
        if err > 0:
            turn_left(ik, angle=step, speed=speed)
        else:
            turn_right(ik, angle=step, speed=speed)
        time.sleep(0.15)
    return st['yaw'] - start


def main():
    parser = argparse.ArgumentParser(description='手动遥控走 + 采 RGB-D 建 3D')
    parser.add_argument('--out', default='/tmp/rgbd3d')
    parser.add_argument('--step', type=int, default=40, help='直行/横移一步 mm')
    parser.add_argument('--angle', type=float, default=20.0, help='q/e 每次转角 deg')
    parser.add_argument('--speed', type=int, default=50, help='行走速度')
    parser.add_argument('--settle', type=float, default=0.6, help='走完等站稳(秒)再拍')
    parser.add_argument('--no-auto', action='store_true', help='只在按 c 时采，不每步自动采')
    parser.add_argument('--force', action='store_true',
                        help='输出目录里已有上次采集时，清掉旧的再采（默认拒绝，防止两次混在一起）')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    # 帧是从 depth_00000 开始编号的：直接重跑会把新帧覆盖到旧帧上，比新帧多的旧帧
    # 还留在目录里，融图时两次的帧会混在一起。所以旧没清干净就不给跑。
    old = sorted(f for f in os.listdir(args.out)
                 if f.startswith(('depth_', 'color_')) or f == 'poses.json')
    if old:
        if not args.force:
            print('FAIL：%s 里已经有上次采集的 %d 个文件（%s ...）。直接跑会从 '
                  'depth_00000 开始覆盖，新帧和旧帧混在一起，融出来的图是错的。'
                  % (args.out, len(old), old[0]), flush=True)
            print('换个目录（--out /tmp/rgbd3d2），或加 --force 清掉旧的重新采。',
                  flush=True)
            return 1
        print('--force：清掉 %s 里 %d 个旧文件' % (args.out, len(old)), flush=True)
        for f in old:
            os.remove(os.path.join(args.out, f))

    cam = DepthCamera()
    cam.open()
    cam.start_depth()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(5):
        cap.read()

    fx, fy, cx, cy = cam.get_depth_intrinsics()
    with open(os.path.join(args.out, 'intrinsic.json'), 'w') as f:
        json.dump({'width': 640, 'height': 480, 'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy},
                  f, indent=2)
    print('深度内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f' % (fx, fy, cx, cy), flush=True)

    board = make_board()
    ik = make_ik(board)
    stand(ik)
    time.sleep(1.5)
    imu = init_imu(board)
    if imu is None:
        print('IMU 读不到数据：确认已 sudo systemctl stop spiderpi（串口被占），'
              '且板子 enable_reception', flush=True)
        return 1

    R = np.eye(3, dtype=np.float32)   # 机体系→世界：t 是机体系原点在世界系的位置
    t = np.zeros(3, dtype=np.float32)
    poses = []
    idx = 0

    def capture():
        nonlocal idx
        d = cam.read_depth(timeout_ms=2000)
        ok, bgr = cap.read()
        if d is None or not ok:
            print('  采集失败（深度或彩色读不到）', flush=True)
            return
        cv2.imwrite(os.path.join(args.out, 'depth_%05d.png' % idx), d)
        cv2.imwrite(os.path.join(args.out, 'color_%05d.jpg' % idx),
                    cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), [cv2.IMWRITE_JPEG_QUALITY, 90])
        poses.append({'idx': idx, 'R': R.tolist(), 't': t.tolist(),
                      'yaw': imu['yaw']})
        print('  已存第 %d 帧  位姿 t=(%.0f, %.0f, %.0f)mm yaw=%.1f°'
              % (idx, t[0], t[1], t[2], imu['yaw']), flush=True)
        idx += 1

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    print('=== 手动建 3D 模式 ===', flush=True)
    print('w前进 s后退 a左横移 d右横移  q左转%.0f° e右转%.0f°  c补采 r立正 y航向清零 空格急停 x退出'
          % (args.angle, args.angle), flush=True)
    print('轻点走一步、按住连续走（每次动作约 1.4s，排队的自动重复键会被丢掉）', flush=True)
    if not args.no_auto:
        print('每走一步自动采一帧；走一圈回到起点后按 x 退出', flush=True)

    try:
        tty.setcbreak(fd)  # 逐字符读、不等回车；Ctrl-C 仍然有效
        while True:
            ch = getch_timeout()
            update_yaw(imu, board)  # 每轮都积分，别漏
            if ch is None:
                continue
            ch = ch.lower()
            # 先丢掉排队/自动重复的按键，再执行这一个 —— 轻点就只走一步
            dropped = drain_input()
            if dropped:
                print('  丢掉 %d 个排队按键（按住不放的自动重复）' % dropped, flush=True)
            moved = False
            t0 = time.monotonic()
            if ch == 'w':
                go_forward(ik, step=args.step, speed=args.speed, times=1)
                t = t + R @ np.array([0, 0, args.step], dtype=np.float32)
                moved = True
                print('  前进 %dmm' % args.step, flush=True)
            elif ch == 's':
                go_back(ik, step=args.step, speed=args.speed)
                t = t + R @ np.array([0, 0, -args.step], dtype=np.float32)
                moved = True
                print('  后退 %dmm' % args.step, flush=True)
            elif ch == 'a':
                t = t + R @ np.array([-args.step, 0, 0], dtype=np.float32)
                ik.left_move(ik.initial_pos, 2, args.step, args.speed, 1)
                moved = True
                print('  左横移 %dmm' % args.step, flush=True)
            elif ch == 'd':
                t = t + R @ np.array([args.step, 0, 0], dtype=np.float32)
                ik.right_move(ik.initial_pos, 2, args.step, args.speed, 1)
                moved = True
                print('  右横移 %dmm' % args.step, flush=True)
            elif ch in ('q', 'e'):
                want = args.angle if ch == 'q' else -args.angle
                got = turn_by(ik, board, imu, want, speed=60)
                # 用 IMU 实测转角，不用命令值（六足每步都有转角偏差，开环会越转越偏）。
                # ⚠ 方向约定要和 yaw 的正负号对上：turn_by 里 yaw 变大 = 左转（err>0 就
                # 叫 turn_left），而相机系是 x 右 / y 上 / z 前，rot_y(+θ) 把 +z 转向 +x
                # 就是右转 —— 按这个推导，左转（got>0）该配 rot_y(-got)，现在这行是反的。
                # 待现场确认：按一次 q，机器人确实往左转、上面打印的「实测」是正数，
                # 就把这行改成 R = R @ rot_y(-got)。
                R = rot_y(got).astype(np.float32) @ R
                moved = True
                print('  %s转 命令%+.0f° 实测%+.1f°（航向 %+.1f°）'
                      % ('左' if ch == 'q' else '右', want, got, imu['yaw']), flush=True)
            elif ch == 'c':
                capture()
            elif ch == 'r':
                stand(ik)
            elif ch == ' ':
                # 急停：一次动作要 1.4 秒，中途发现要走过头就按空格
                try:
                    ik.stopMove()
                    print('  急停', flush=True)
                except Exception as exc:
                    print('  急停不可用: %s' % exc, flush=True)
            elif ch == 'y':
                imu['yaw'] = 0.0
                print('  航向清零', flush=True)
            elif ch in ('x', '\x03'):
                print('退出采集', flush=True)
                break
            else:
                print('未识别按键: %r' % ch, flush=True)

            if moved:
                time.sleep(args.settle)
                if not args.no_auto:
                    capture()
                # 一次按键的总耗时；正常 ~1.4s（走一步 0.5s + 站稳 + 拍一帧）。
                # 若某一行的耗时是几十秒，说明那一次调用本身在循环，是另一类问题。
                print('  用时 %.1fs' % (time.monotonic() - t0), flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        # 一帧都没采到就别写 poses.json：以前这里无条件写，一次空跑会把上一次
        # 采好的位姿覆盖成 []，帧还在、位姿没了，数据就废了。
        if poses:
            with open(os.path.join(args.out, 'poses.json'), 'w') as f:
                json.dump(poses, f, indent=1)
        else:
            print('本次一帧都没采到，不动已有的 poses.json', flush=True)
        cap.release()
        cam.close()
        stand(ik)

    print('采集完成，共 %d 帧 -> %s' % (idx, args.out), flush=True)
    print('拉回本机融合：', flush=True)
    print('  scp -r pi@<本机IP>:%s ./rgbd3d' % args.out, flush=True)
    print('  ~/open3d_env/bin/python reconstruct_rgbd.py --in ./rgbd3d --out map.ply',
          flush=True)


if __name__ == '__main__':
    sys.exit(main())  # 失败时 main 返回 1，别把退出码吞掉
