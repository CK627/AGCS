#!/usr/bin/python3
# coding=utf8
"""IMU 体检工具：查采样能力（只读）+ 判转向符号（会动 20°）。

## 为什么需要它

官方 SDK `ros_robot_controller_sdk.py`：

    self.imu_queue = queue.Queue(maxsize=1)      # 只留一个样本
    def packet_report_imu(self, data):
        try:    self.imu_queue.put_nowait(data)
        except queue.Full: pass                  # 没人取就直接丢

所以 `get_imu()` 拿到的是「上一次被取走之后到达的第一个样本」。**两次调用之间的所有
样本都被丢掉了**——而 `Auto-capture.py` 的 `update_imu()` 每 1.9 秒才调一次，中间夹着
一整段 100mm 行走和一次修正转弯。结果是：那 1.9 秒里真发生的旋转，只按「窗口最开头
那一瞬间的角速度」折算，转弯本身完全没被积分进去。

现场表现：控制器看不见自己刚发出的修正转了多少 → 误差永远不收敛 → 一直朝同一个方向
发指令 → 机器人真的朝那个方向转飞。

## 修法

后台线程以 ~100Hz 连续取队列并用「距上次取到的间隔」积分。单样本间隔只有几毫秒，
坏样本乘出来的误差可以忽略。`get_imu()` 只读队列（串口接收由 SDK 自己的 recv_task
线程负责），不碰 `buf_write` 的串口写路径，所以**不会跟舵机指令抢 /dev/ttyAMA0**。

## 用法

    python3 imu_probe.py            # 只读：采样率 / 噪声 / 静止积分漂移
    python3 imu_probe.py --sign     # 会动：左转 20°、右转 20°，判定 yaw 符号
"""

import argparse
import sys
import threading
import time

sys.path.insert(0, '/home/pi/spiderpi')

SCALE = 1.188  # 与 Auto-capture.py 的 GYRO_SCALE_LEFT/RIGHT 同量级


class ImuSampler(threading.Thread):
    """后台 ~100Hz 采样并积分 yaw。

    get_imu() 是纯队列读取，不写串口，因此与主线程的舵机指令无冲突。
    """

    def __init__(self, board, scale=SCALE):
        super().__init__(daemon=True)
        self.board = board
        self.scale = scale
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bias_sum = 0.0
        self._bias_n = 0
        self._calibrating = False
        self.yaw = 0.0
        self.bias = 0.0
        self.peak_rate = 0.0   # 标定后见过的最大 |rate|，用来确认陀螺确实测到了转动
        self._last_t = None

    def run(self):
        while not self._stop.is_set():
            d = self.board.get_imu()
            if d is None:
                time.sleep(0.001)
                continue
            now = time.monotonic()
            gz = float(d[5])
            with self._lock:
                if self._last_t is not None:
                    rate = gz - self.bias
                    self.yaw += rate * (now - self._last_t) * self.scale
                    if abs(rate) > abs(self.peak_rate):
                        self.peak_rate = rate
                self._last_t = now
                if self._calibrating:
                    self._bias_sum += gz
                    self._bias_n += 1
            time.sleep(0.001)

    def calibrate(self, seconds=1.5):
        """静止采样若干秒，用均值当零漂。"""
        with self._lock:
            self._bias_sum = 0.0
            self._bias_n = 0
            self._calibrating = True
        time.sleep(seconds)
        with self._lock:
            self._calibrating = False
            if self._bias_n:
                self.bias = self._bias_sum / self._bias_n
            return self.bias, self._bias_n

    def reset(self):
        with self._lock:
            self.yaw = 0.0
            self.peak_rate = 0.0

    def snapshot(self):
        with self._lock:
            return self.yaw

    def stop(self):
        self._stop.set()


def sample_report():
    """只读：量采样率、噪声、静止积分漂移。不发任何舵机指令。"""
    from agcs_lib import make_board

    board = make_board()
    board.enable_reception()
    print('等待 IMU 数据流稳定…', flush=True)
    time.sleep(1.0)

    t0 = time.monotonic()
    stamps, vals = [], []
    while time.monotonic() - t0 < 2.0:
        d = board.get_imu()
        if d is not None:
            stamps.append(time.monotonic())
            vals.append(float(d[5]))
        time.sleep(0.001)

    print('=' * 60)
    print('2 秒内取到样本：%d 个' % len(vals))
    if len(vals) < 2:
        print('样本太少——检查 enable_reception 与串口占用')
        return

    gaps = sorted(stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1))
    mid = gaps[len(gaps) // 2]
    print('样本间隔：中位 %.2f ms  最小 %.2f ms  最大 %.2f ms'
          % (mid * 1000, gaps[0] * 1000, gaps[-1] * 1000))
    print('等效采样率：约 %.1f Hz' % (1.0 / mid if mid > 0 else 0.0))

    same = sum(1 for i in range(1, len(vals)) if vals[i] == vals[i - 1])
    print('相邻两次读数完全相同：%d / %d 次' % (same, len(vals) - 1))

    sv = sorted(vals)
    med = sv[len(sv) // 2]
    print('静止 gz：中位 %.4f  最小 %.4f  最大 %.4f  极差 %.4f'
          % (med, sv[0], sv[-1], sv[-1] - sv[0]))
    print('（gz 单位是 °/s：静止零漂约 %.2f°/s，噪声 ±%.2f°/s）'
          % (med, (sv[-1] - sv[0]) / 2))

    yaw = 0.0
    for i in range(1, len(stamps)):
        yaw += (vals[i] - med) * (stamps[i] - stamps[i - 1])
    print('静止 2 秒、按真实样本间隔积分：%+.3f 度（理想≈0）' % yaw)
    span = stamps[-1] - stamps[0]
    print('对照·旧做法「最后那个样本 × 整段 dt」：%+.2f 度'
          % ((vals[-1] - med) * span))
    print('=' * 60)


def sign_test(angle=20, speed=30):
    """会动：左转 angle°、右转 angle°，用后台积分判定 yaw 符号与量纲。"""
    from agcs_lib import make_board, make_ik

    board = make_board()
    board.enable_reception()
    ik = make_ik(board)
    s = ImuSampler(board)
    s.start()
    time.sleep(0.5)

    bias, n = s.calibrate(1.5)
    print('=' * 60)
    print('零漂标定：%.4f °/s（%d 个样本）' % (bias, n))

    results = {}
    for name, fn in (('左转', ik.turn_left), ('右转', ik.turn_right)):
        s.reset()
        time.sleep(0.3)
        y0 = s.snapshot()
        fn(ik.initial_pos, 2, angle, speed, 1)   # 与 Auto-capture.py 同样的调用
        time.sleep(1.2)                          # 转完再积分一会儿，吃掉残余摆动
        y1 = s.snapshot()
        results[name] = (y1 - y0, s.peak_rate)
        print('%s %d°：yaw %+.2f → %+.2f，变化 %+.2f（过程中峰值角速度 %+.2f°/s）'
              % (name, angle, y0, y1, y1 - y0, s.peak_rate))
        time.sleep(0.5)

    s.stop()
    dl, dr = results['左转'][0], results['右转'][0]
    print('-' * 60)
    ratio = abs(dl) / angle if angle else 0.0
    print('量纲判定：物理转了 %d°，积分得到 %.2f → 比值 %.2f' % (angle, abs(dl), ratio))
    if 0.3 <= ratio <= 3.0:
        print('  → yaw 是「度」，阈值 1.0/8.0 这些数字有意义')
    elif ratio < 0.3:
        print('  → 积分远小于物理角度：要么 gz 不是 °/s，要么采样仍没覆盖转动')
    else:
        print('  → 积分远大于物理角度：比例系数或采样有问题')

    print('符号判定：左转 Δyaw=%+.2f，右转 Δyaw=%+.2f' % (dl, dr))
    if dl > 0 and dr < 0:
        print('  → 左转=+yaw，与代码约定一致（IMU_DIRECTION_SIGN=1 正确）')
    elif dl < 0 and dr > 0:
        print('  → 左转=-yaw，与代码约定相反！IMU_DIRECTION_SIGN 应设为 -1')
    else:
        print('  → 两次同号，转动没被积到，采样方式仍有问题')
    print('=' * 60)


def walk_test(chunks=3, mm=100, speed=50):
    """会动：量「走路时」的航向漂移——站着标零漂量不到的就是这一段。

    静止时零漂标得很准（现场实测 −1.913°/s，0.5~30s 纹丝不动），可一走起来 gz
    就整体偏了约 +1.8°/s。这有两种可能，站着区分不出来，只能让它走一段、人盯着看：
      (a) 六足步态真的把机身转过去了 —— IMU 修正是对的，该修；
      (b) 腿部振动把陀螺零偏带跑了 —— 积分出来的是幻影，越修越偏。
    判据只有一个：**走路积分出来的角度，跟你眼睛看到的是否一致。**

    前进/后退交替，净位移接近 0，不会把机器人带跑。
    """
    from agcs_lib import make_board, make_ik

    board = make_board()
    board.enable_reception()
    ik = make_ik(board)

    # 先把机械臂收回来（同 Auto-capture.py 的 restore_travel），免得走路时拖地
    board.bus_servo_set_position(1.5, [[22, 705], [23, 90], [21, 500], [24, 330]])
    time.sleep(1.5)

    s = ImuSampler(board)
    s.start()
    time.sleep(0.5)
    bias, n = s.calibrate(1.5)
    print('=' * 60)
    print('零漂标定：%+.4f °/s（%d 个样本）' % (bias, n))
    print('接下来前/后各走 %d 段、每段 %dmm。**请盯着机器人，看它走路时歪不歪。**'
          % (chunks, mm))
    print('-' * 60)

    s.reset()
    time.sleep(1.2)
    print('静止对照 1.2s：Δyaw %+.3f°（这段应该≈0）' % s.snapshot())

    tot_f = tot_b = 0.0
    for i in range(chunks):
        for label, fn in (('前进', ik.go_forward), ('后退', ik.back)):
            s.reset()
            y0 = s.snapshot()
            fn(ik.initial_pos, 2, mm, speed, 1)
            time.sleep(0.3)
            d = s.snapshot() - y0
            print('%s %dmm：Δyaw %+.2f°（峰值角速度 %+.2f°/s）'
                  % (label, mm, d, s.peak_rate))
            if label == '前进':
                tot_f += d
            else:
                tot_b += d
            time.sleep(0.2)

    s.stop()
    print('-' * 60)
    print('前进共 %dmm：Δyaw %+.2f°　　后退共 %dmm：Δyaw %+.2f°'
          % (chunks * mm, tot_f, chunks * mm, tot_b))
    print('=' * 60)
    print('怎么读：')
    print('  · 积分到明显角度，但你看着它**是直着走的**')
    print('    → 幻影(b)。振动把陀螺零偏带跑了。此时 IMU 直线修正是有害的，')
    print('      越修越歪，应该 --imu-straight off 关掉，另想办法。')
    print('  · 积分到的角度跟你**看到它歪的方向、幅度对得上**')
    print('    → 真的(a)。步态本身有系统性偏转。IMU 修正方向是对的，')
    print('      要保证控制器守的是「本段固定目标」，不能跟着漂移跑。')


def main():
    ap = argparse.ArgumentParser(description='IMU 体检')
    ap.add_argument('--sign', action='store_true',
                    help='做转向符号测试（机器人会左转、右转各 20°）')
    ap.add_argument('--angle', type=int, default=20, help='符号测试的转角，默认 20')
    ap.add_argument('--walk', action='store_true',
                    help='量「走路时」的航向漂移（机器人会前/后各走几段，净位移≈0）')
    ap.add_argument('--chunks', type=int, default=3, help='--walk 的前/后段数，默认 3')
    ap.add_argument('--mm', type=int, default=100, help='--walk 每段距离(mm)，默认 100')
    args = ap.parse_args()
    if args.sign:
        sign_test(args.angle)
    elif args.walk:
        walk_test(args.chunks, args.mm)
    else:
        sample_report()


if __name__ == '__main__':
    main()
