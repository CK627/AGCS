#!/usr/bin/python3
# coding=utf8
"""自动抓取（夹爪逐步靠近 + 模型引导对准）。

流程：
    状态 Grab → 恢复官方初始位置（夹爪张开）→ 相机朝前下看
    → 夹爪分步下降靠近：22/23 从复位位插值到夹取位，每步模型微调 21/24 居中
      + 22/23 高度/前后补偿（框高估距离）
    → 慢慢闭合夹爪 → 保持 → 恢复机械臂初始位置（夹爪保持闭合）。

完全独立：只 import 标准库 + pip 库（cv2/flask/onnxruntime）+ 官方 SDK（common）。
上报 /status + 视频流 /video.mjpeg（http://<IP>:5000/video.mjpeg）。

用法（先 sudo systemctl stop spiderpi）：
    python3 AutonomousCrawling.py
    python3 AutonomousCrawling.py --model ""   # 不检测，纯固定脉宽夹取
"""
import os
import sys
import time
import ctypes
import threading
import argparse

import cv2
import numpy as np

from common.ros_robot_controller_sdk import Board

# eye-in-hand 坐标转换（同目录，纯数学，不依赖 SDK）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import eye_in_hand as EIH
except Exception as _e:  # 打包成 exe 时若未收集，退回框高估距
    EIH = None
    print('eye_in_hand 导入失败：%s，退回框高估距' % _e, flush=True)

# spiderpi 根目录（模型在 ~/spiderpi/models/ 下）
if getattr(sys, 'frozen', False):
    _PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[0])))
else:
    _PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# 目标舵机脉宽（21/22/23/24）
GRAB = {21: 500, 22: 400, 23: 500, 24: 250}   # 21 向左 +15 脉宽（485->500）
# 官方初始位置（复位，取自 robot_params.yaml arm.reset_pulses）
RESET = {21: 500, 22: 705, 23: 90, 24: 330}
GRIPPER_OPEN = 120    # 25 号张开
GRIPPER_CLOSE = 700   # 25 号闭合（拉满）
HOLD_SEC = 1.0        # 夹住保持时长（秒）
# 用框高估距离：距离(cm) = F_PX_FULL * 虫子实际高度(cm) / 框高(px)
F_PX_FULL = 838.0     # 原始 640 分辨率焦距像素（与 1.py 标定一致）
BUG_HEIGHT_CM = 5.0   # 虫子模型实际高度(cm)，现场量一次
# 前后（深度）补偿：框高度越小目标越远。NOMINAL_BBOX_H 是目标在正确夹取距离时的框高度
# （像素），现场标定；REACH_GAIN / HEIGHT_GAIN 是补偿增益（正负决定方向，现场调）。0 = 关闭补偿。
NOMINAL_BBOX_H = 0.0  # 目标在正确距离时的框高度（像素），0 表示关闭前后/高度补偿
REACH_GAIN = 0.3      # 框高度每差 1 像素，23（肘）调多少脉宽（前后）
HEIGHT_GAIN = 0.3     # 框高度每差 1 像素，22（肩）调多少脉宽（高度）
# 夹爪逐步靠近参数
APPROACH_STEPS = 30   # 夹爪从复位位分多少步下降到夹取位（步数越多越慢越平滑）
K_PAN = 0.2           # 21 横转增益（让目标在画面水平居中）
K_TILT = 0.2          # 24 俯仰增益（以「夹爪水平」为基准，比例微调让目标竖直居中，不累加）
LEVEL_SUM = 1125      # 夹爪水平时 22+23+24 = 1125（alpha=0）


STATUS = {'state': 'Grab', 'message': '自动抓取', 'last_result': None}
_LATEST_JPEG = None
_JPEG_LOCK = threading.Lock()


def set_status(**kw):
    STATUS.update(kw)


def publish_frame(frame):
    """把一帧压成 JPEG 存共享区，供 /video.mjpeg 推流。"""
    global _LATEST_JPEG
    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    if ok:
        with _JPEG_LOCK:
            _LATEST_JPEG = jpg.tobytes()


def start_server():
    """内联 Flask：/status 状态 + /video.mjpeg 视频流。"""
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    import flask.cli
    flask.cli.show_server_banner = lambda *a, **k: None
    from flask import Flask, Response, jsonify
    app = Flask(__name__)

    @app.route('/status')
    def status():
        return jsonify(STATUS)

    @app.route('/video.mjpeg')
    def video():
        def gen():
            while True:
                with _JPEG_LOCK:
                    jpg = _LATEST_JPEG
                if jpg is None:
                    time.sleep(0.05)
                    continue
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                time.sleep(0.1)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000,
                                             'debug': False, 'threaded': True},
                     daemon=True).start()


class Camera:
    """后台线程连续取帧（存最新帧）+ 检测画框 + 推流（10fps 限流）。"""

    def __init__(self, cap, model_det=None):
        self.cap = cap
        self.frame = None
        self.model_det = model_det
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_pub = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok:
                if self.model_det is not None:
                    self.model_det.detect(f)  # 画识别框（约 65ms）
                with self._lock:
                    self.frame = f
                now = time.time()
                if now - self._last_pub >= 0.1:  # 10 fps
                    self._last_pub = now
                    publish_frame(f)
            time.sleep(0.01)

    def read(self):
        with self._lock:
            return self.frame

    def stop(self):
        self._stop.set()


def lan_ip():
    import socket
    import subprocess
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        pass
    # 直连(AP)模式没有外网，退回从网卡枚举本机 IP
    try:
        out = subprocess.check_output(['hostname', '-I']).decode().strip()
        ips = [x for x in out.split()
               if x.startswith(('192.168', '10.', '172.'))]
        return ips[0] if ips else (out.split()[0] if out else '127.0.0.1')
    except Exception:
        return '127.0.0.1'


def move(board, servos, sec):
    """按给定脉宽移动舵机；sec 为移动时长（秒），阻塞到移动完成。"""
    board.bus_servo_set_position(sec, [[sid, p] for sid, p in servos])
    time.sleep(sec + 0.1)


def compute_grab_servos(target_xyz, alpha1=-90.0, alpha2=100.0):
    """用官方 IK 解目标坐标 (X右,Y前,Z高 cm) 对应的 21/22/23/24 脉宽。

    返回 {'21':..,'22':..,'23':..,'24':..} 或 None（无解/超范围）。
    导入官方 arm_ik 前先用空壳替换 Board，避免二次打开串口。
    """
    import common.ros_robot_controller_sdk as sdk

    class _NoPortBoard:
        def __init__(self, *a, **k):
            pass

    orig_board = sdk.Board
    sdk.Board = _NoPortBoard
    try:
        import arm_ik.arm_move_ik as AMK
    finally:
        sdk.Board = orig_board

    ik = AMK.ArmIK()
    r = ik.setPitchRange(tuple(target_xyz), alpha1, alpha2)
    if r is False:
        return None
    servos, _alpha = r
    return {'21': servos['servo21'], '22': servos['servo22'],
            '23': servos['servo23'], '24': servos['servo24']}


# ---------------- 深度相机（OpenNI2，内联最小版，与 probe_bug_depth.py 一致）----------------
class _OniFrame(ctypes.Structure):
    _fields_ = [
        ('dataSize', ctypes.c_int), ('data', ctypes.c_void_p),
        ('sensorType', ctypes.c_int), ('timestamp', ctypes.c_uint64),
        ('frameIndex', ctypes.c_int), ('width', ctypes.c_int), ('height', ctypes.c_int),
        ('videoMode_pixelFormat', ctypes.c_int), ('videoMode_resX', ctypes.c_int),
        ('videoMode_resY', ctypes.c_int), ('videoMode_fps', ctypes.c_int),
        ('croppingEnabled', ctypes.c_int), ('cropOriginX', ctypes.c_int),
        ('cropOriginY', ctypes.c_int), ('stride', ctypes.c_int),
    ]


class _OniDeviceInfo(ctypes.Structure):
    _fields_ = [
        ('uri', ctypes.c_char * 256), ('vendor', ctypes.c_char * 256),
        ('name', ctypes.c_char * 256), ('usbVendorId', ctypes.c_uint16),
        ('usbProductId', ctypes.c_uint16),
    ]


class DepthCam:
    def __init__(self, lib='/home/pi/orbbec_sdk/libOpenNI2.so'):
        self.lib = ctypes.CDLL(lib)
        self._device = ctypes.c_void_p()
        self._stream = ctypes.c_void_p()
        L = self.lib
        L.oniInitialize.argtypes = [ctypes.c_int]; L.oniInitialize.restype = ctypes.c_int
        L.oniShutdown.argtypes = []; L.oniShutdown.restype = None
        L.oniGetDeviceList.argtypes = [ctypes.POINTER(ctypes.POINTER(_OniDeviceInfo)),
                                       ctypes.POINTER(ctypes.c_int)]
        L.oniGetDeviceList.restype = ctypes.c_int
        L.oniReleaseDeviceList.argtypes = [ctypes.POINTER(_OniDeviceInfo)]
        L.oniReleaseDeviceList.restype = ctypes.c_int
        L.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
        L.oniDeviceOpen.restype = ctypes.c_int
        L.oniDeviceClose.argtypes = [ctypes.c_void_p]; L.oniDeviceClose.restype = ctypes.c_int
        L.oniDeviceCreateStream.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                            ctypes.POINTER(ctypes.c_void_p)]
        L.oniDeviceCreateStream.restype = ctypes.c_int
        L.oniStreamStart.argtypes = [ctypes.c_void_p]; L.oniStreamStart.restype = ctypes.c_int
        L.oniStreamStop.argtypes = [ctypes.c_void_p]; L.oniStreamStop.restype = None
        L.oniStreamDestroy.argtypes = [ctypes.c_void_p]; L.oniStreamDestroy.restype = None
        L.oniStreamReadFrame.argtypes = [ctypes.c_void_p,
                                         ctypes.POINTER(ctypes.POINTER(_OniFrame))]
        L.oniStreamReadFrame.restype = ctypes.c_int
        L.oniFrameRelease.argtypes = [ctypes.POINTER(_OniFrame)]
        L.oniFrameRelease.restype = None
        L.oniCoordinateConverterDepthToWorld.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                                         ctypes.c_float, ctypes.c_float,
                                                         ctypes.POINTER(ctypes.c_float),
                                                         ctypes.POINTER(ctypes.c_float),
                                                         ctypes.POINTER(ctypes.c_float)]
        L.oniCoordinateConverterDepthToWorld.restype = ctypes.c_int

    def open(self):
        self.lib.oniInitialize(2002)
        devs = ctypes.POINTER(_OniDeviceInfo)()
        n = ctypes.c_int(0)
        self.lib.oniGetDeviceList(ctypes.byref(devs), ctypes.byref(n))
        if n.value == 0:
            raise RuntimeError('未找到深度设备')
        uri = bytes(devs[0].uri)
        self.lib.oniReleaseDeviceList(devs)
        self.lib.oniDeviceOpen(uri, ctypes.byref(self._device))
        self.lib.oniDeviceCreateStream(self._device, 3, ctypes.byref(self._stream))  # 3=深度
        self.lib.oniStreamStart(self._stream)

    def read(self, timeout_ms=100):
        frame = ctypes.POINTER(_OniFrame)()
        if self.lib.oniStreamReadFrame(self._stream, ctypes.byref(frame)) != 0:
            return None
        try:
            f = frame.contents
            raw = ctypes.string_at(f.data, f.dataSize)
            arr = np.frombuffer(raw, dtype=np.uint16)
            arr = arr[:f.stride // 2 * f.height].reshape(f.height, f.stride // 2)
            return arr[:, :f.width].copy()
        finally:
            self.lib.oniFrameRelease(frame)

    def depth_to_world(self, x, y, z):
        """深度像素 (x,y) + 深度值 z(mm) -> 世界坐标 (X右,Y上,Z前) mm（工厂标定）。"""
        wx = ctypes.c_float()
        wy = ctypes.c_float()
        wz = ctypes.c_float()
        rc = self.lib.oniCoordinateConverterDepthToWorld(
            self._stream, float(x), float(y), float(z),
            ctypes.byref(wx), ctypes.byref(wy), ctypes.byref(wz))
        if rc != 0:
            return None
        return (wx.value, wy.value, wz.value)

    def close(self):
        if self._stream:
            self.lib.oniStreamDestroy(self._stream)
        if self._device:
            self.lib.oniDeviceClose(self._device)
        self.lib.oniShutdown()


def _depth_median(depth, cx, cy, r=3):
    """取 (cx,cy) 附近深度中位数（中心黑洞=0 时往邻域扩），返回 (z_mm, xi, yi) 或 (None, xi, yi)。"""
    xi, yi = int(round(cx)), int(round(cy))
    h, w = depth.shape
    for radius in range(0, r + 1):
        vals = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                yy, xx = yi + dy, xi + dx
                if 0 <= xx < w and 0 <= yy < h:
                    v = int(depth[yy, xx])
                    if v > 0:
                        vals.append(v)
        if vals:
            vals.sort()
            return vals[len(vals) // 2], xi, yi
    return None, xi, yi


def measure_bug_base(depth_cam, depth_frame, det, R_c2e, t_c2e, pose, z_offset_cm=0.0):
    """检测框中心 + 邻域中值深度 -> 机械臂基座 (x右,y前,z上) cm。失败返回 (None, dbg)。

    链（动态 eye-in-hand）：中心(cx,cy) 中值深度 -> depth_to_world(深度相机 X右Y上Z前)
        -> 翻 Y -> T_end_to_base(当前 pose) × T_cam_to_end -> 基座 cm -> 加 z_offset。
    pose = 当前臂位姿 (p21, p22, p23, p24)。
    """
    dbg = {}
    cx, cy = det['center']
    dbg['center'] = (round(cx, 1), round(cy, 1))

    z_mm, xi, yi = _depth_median(depth_frame, cx, cy)
    dbg['z_mm'] = z_mm
    if z_mm is None or z_mm <= 0:
        return None, dbg

    w = depth_cam.depth_to_world(float(xi), float(yi), float(z_mm))
    if w is None:
        dbg['err'] = 'depth_to_world 失败'
        return None, dbg
    wx, wy, wz = w
    cam = EIH.depth_world_to_cam(wx, wy, wz)
    xyz = EIH.cam_to_base_dynamic(cam, pose[0], pose[1], pose[2], pose[3], R_c2e, t_c2e)
    dbg['cam'] = (round(wx, 0), round(wy, 0), round(wz, 0))
    xyz[2] += z_offset_cm
    dbg['xyz'] = (round(xyz[0, 0], 1), round(xyz[1, 0], 1), round(xyz[2, 0], 1))
    return (float(xyz[0, 0]), float(xyz[1, 0]), float(xyz[2, 0])), dbg


def reset_arm(board):
    """恢复官方初始位置：机械臂复位 + 夹爪张开。"""
    move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]),
                 (24, RESET[24]), (25, GRIPPER_OPEN)], 1.5)


class ModelDetector:
    """ONNX YOLO 检测器（onnxruntime 本地推理）。detect(frame) 返回 {'center':(cx,cy),'conf':..} 或 None。"""

    NAME = 'fake bug'

    def __init__(self, model_path, conf=0.5):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        self.output_name = self.sess.get_outputs()[0].name
        self.conf = conf
        shp = self.sess.get_inputs()[0].shape
        self.in_h, self.in_w = int(shp[2]), int(shp[3])

    def detect(self, frame):
        h0, w0 = frame.shape[:2]
        ih, iw = self.in_h, self.in_w
        r = min(iw / w0, ih / h0)
        new_w, new_h = int(round(w0 * r)), int(round(h0 * r))
        pad_x, pad_y = (iw - new_w) // 2, (ih - new_h) // 2
        canvas = np.full((ih, iw, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(frame, (new_w, new_h))
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run([self.output_name], {self.input_name: blob})[0][0]

        nc = out.shape[0] - 4
        best = None
        for i in range(out.shape[1]):
            scores = out[4:4 + nc, i]
            cls = int(scores.argmax())
            score = float(scores[cls])
            if score < self.conf:
                continue
            cx, cy, w, h = out[0, i], out[1, i], out[2, i], out[3, i]
            x1 = (cx - w / 2 - pad_x) / r
            y1 = (cy - h / 2 - pad_y) / r
            x2 = (cx + w / 2 - pad_x) / r
            y2 = (cy + h / 2 - pad_y) / r
            x1 = max(0, min(w0, x1)); y1 = max(0, min(h0, y1))
            x2 = max(0, min(w0, x2)); y2 = max(0, min(h0, y2))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            if best is None or score > best[4]:
                best = (x1, y1, x2, y2, score)

        if best is None:
            return None
        x1, y1, x2, y2, score = best
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        return {'center': (cx, cy), 'conf': float(score),
                'w': float(x2 - x1), 'h': float(y2 - y1)}


def main():
    parser = argparse.ArgumentParser(description='2.3 自动抓取+递物（夹取前用模型检测对准）')
    parser.add_argument('--model', default='models/v8n.onnx',
                        help='YOLO ONNX 模型路径；传空串 "" 则不检测直接固定脉宽夹')
    parser.add_argument('--conf', type=float, default=0.5, help='YOLO 置信度阈值')
    parser.add_argument('--bug-height', type=float, default=BUG_HEIGHT_CM,
                        help='虫子模型实际高度(cm)：eye-in-hand 用表面深度减它，框高估距用它算距离')
    parser.add_argument('--eye-in-hand', action='store_true', default=True,
                        help='用 eye-in-hand 算虫子基座 (X,Y,Z) 再喂 IK（默认开；深度相机失败自动退回框高）')
    parser.add_argument('--no-eye-in-hand', action='store_true',
                        help='强制关闭 eye-in-hand，用框高估距')
    parser.add_argument('--cam2arm', default='config/cam2arm.yaml',
                        help='手眼标定 cam2arm.yaml（相对 _PKG_ROOT）')
    parser.add_argument('--calib-pose', default='500,705,90,260',
                        help='标定 cam2arm 时机械臂位姿 21,22,23,24（逗号分隔；拆固定手眼用，必须填正确值）')
    parser.add_argument('--z-offset', type=float, default=0.0,
                        help='夹取高度偏移 cm（加到 eye-in-hand 算出的 z 上，默认 0）')
    args = parser.parse_args()

    board = Board()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()

    # YOLO 模型（夹取前检测对准用；加载失败则不检测）
    model_det = None
    if args.model:
        model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
        try:
            model_det = ModelDetector(model_path, args.conf)
            print('YOLO 模型已加载：%s（conf=%.2f）' % (model_path, args.conf), flush=True)
        except Exception as e:
            print('YOLO 模型加载失败：%s，退回固定脉宽夹取' % e, flush=True)
            model_det = None

    # 深度相机 + 手眼标定（eye-in-hand 用；任一失败则退回框高估距）
    depth, R_c2a, t_c2a = None, None, None
    use_eih = args.eye_in_hand and not args.no_eye_in_hand and EIH is not None
    if use_eih:
        try:
            depth = DepthCam()
            depth.open()
            print('深度相机已打开（eye-in-hand 模式）', flush=True)
        except Exception as e:
            print('深度相机打开失败：%s，退回框高估距' % e, flush=True)
            depth = None
            use_eih = False
    if use_eih:
        try:
            R_c2a, t_c2a = EIH.load_cam2arm(os.path.join(_PKG_ROOT, args.cam2arm))
            calib = [int(v) for v in args.calib_pose.split(',')]
            if len(calib) != 4:
                raise ValueError('--calib-pose 需 4 个数 21,22,23,24')
            R_c2e, t_c2e = EIH.extract_cam2end(R_c2a, t_c2a, calib[0], calib[1], calib[2], calib[3])
            print('cam2arm 已加载，固定手眼已拆解（标定位姿 %s）' % calib, flush=True)
        except Exception as e:
            print('cam2arm/标定位姿解析失败：%s，退回框高估距' % e, flush=True)
            R_c2e, t_c2e = None, None
            use_eih = False

    cam = Camera(cap)

    start_server()
    set_status(state='Grab', message='自动抓取（夹取前检测对准）')

    try:
        # 1) 恢复官方初始位置（夹爪张开）
        reset_arm(board)
        time.sleep(0.5)
        # 相机朝下看（能看到目标的角度）
        board.bus_servo_set_position(0.3, [[24, 260]])
        time.sleep(0.3)

        # 2) 检测目标，算夹取舵机（优先 eye-in-hand，退回框高估距）
        grab_21, grab_22, grab_23, grab_24 = GRAB[21], GRAB[22], GRAB[23], GRAB[24]
        if model_det is not None:
            for _ in range(5):
                f = cam.read()
                if f is None:
                    time.sleep(0.05)
                    continue
                r = model_det.detect(f)
                if r is None:
                    time.sleep(0.05)
                    continue
                h = r.get('h', 0.0)
                if h <= 10:
                    continue
                xyz = None
                if use_eih and depth is not None:
                    df = depth.read(100)
                    if df is None:
                        print('eye-in-hand: 深度帧读不到', flush=True)
                    else:
                        # 测量位姿 = 复位位(21=500,22=705,23=90) + 相机 24=260 朝下看
                        xyz, dbg = measure_bug_base(depth, df, r, R_c2e, t_c2e,
                                                    (500, RESET[22], RESET[23], 260),
                                                    args.z_offset)
                        if xyz is not None:
                            print('eye-in-hand: 中心%s 深度=%smm 相机=%s -> 基座(X=%.1f,Y=%.1f,Z=%.1f)cm'
                                  % (dbg.get('center'), dbg.get('z_mm'), dbg.get('cam'),
                                     xyz[0], xyz[1], xyz[2]), flush=True)
                        else:
                            print('eye-in-hand: 深度无效 center=%s z_mm=%s'
                                  % (dbg.get('center'), dbg.get('z_mm')), flush=True)
                if xyz is not None:
                    g = compute_grab_servos(xyz)
                    if g is not None:
                        grab_21, grab_22, grab_23, grab_24 = g['21'], g['22'], g['23'], g['24']
                        print('  → IK 21=%d 22=%d 23=%d 24=%d' % (grab_21, grab_22, grab_23, grab_24),
                              flush=True)
                    else:
                        print('  → 目标 (X=%.1f,Y=%.1f,Z=%.1f) IK 无解' % xyz, flush=True)
                else:
                    # 退回框高估距：距离(cm) = F_PX * 虫子高度 / 框高，喂 (0, Y, 0)
                    D = F_PX_FULL * args.bug_height / h
                    g = compute_grab_servos((0.0, D, 0.0))
                    if g is not None:
                        grab_21, grab_22, grab_23, grab_24 = g['21'], g['22'], g['23'], g['24']
                        print('框高=%.0f → 距离=%.1fcm → IK 22=%d 23=%d'
                              % (h, D, grab_22, grab_23), flush=True)
                    else:
                        print('框高=%.0f → 距离=%.1fcm，IK 无解' % (h, D), flush=True)
                break

        # 3) 夹爪逐步下降 + 保持夹爪水平 + 目标居中（24 以水平为基准做比例俯仰微调，不累加）
        x_dis = grab_21                            # 21 从 IK 解开始，只做左右居中
        w22, z23 = RESET[22], RESET[23]            # 22/23 从复位位开始
        step_22 = (RESET[22] - grab_22) / APPROACH_STEPS  # 每步 22 下降量
        step_23 = (grab_23 - RESET[23]) / APPROACH_STEPS  # 每步 23 伸展量
        cy = 240.0                                  # 未检测到时按「已居中」处理
        for step in range(APPROACH_STEPS):
            f = cam.read()
            if f is not None and model_det is not None:
                r = model_det.detect(f)
                if r is not None:
                    cx, cy = r['center']
                    print('靠近 中心=(%.0f,%.0f)' % (cx, cy), flush=True)
                    x_dis = max(0, min(1000, int(x_dis + K_PAN * (320 - cx))))   # 左右居中
            # 夹爪小步下降/伸展
            w22 = max(0, min(1000, int(w22 - step_22)))
            z23 = max(0, min(1000, int(z23 + step_23)))
            # 24 号：以「夹爪水平 alpha=0」为基准，叠一个比例俯仰微调让目标竖直居中。
            # 关键：每步从水平基准重算（不是累加），目标低于中心时只会小幅低头、不会越降越低头。
            y_dis = max(0, min(1000, int((LEVEL_SUM - w22 - z23) + K_TILT * (240 - cy))))
            board.bus_servo_set_position(0.15, [[21, x_dis], [24, y_dis], [22, w22], [23, z23]])
            time.sleep(0.2)

        # 3) 慢慢闭合夹爪
        move(board, [(25, GRIPPER_CLOSE)], 1.5)
        set_status(last_result='done', message='已夹取')
        time.sleep(HOLD_SEC)

        # 4) 恢复机械臂初始位置（21-24 复位，夹爪保持闭合、不张开）
        move(board, [(21, RESET[21]), (22, RESET[22]), (23, RESET[23]), (24, RESET[24])], 1.5)
        set_status(last_result='done', message='已夹取并恢复')
        print('夹取成功', flush=True)
    finally:
        cam.stop()
        cap.release()
        if depth is not None:
            try:
                depth.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
