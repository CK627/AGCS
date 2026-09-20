#!/usr/bin/python3
# coding=utf8
"""探测虫子深度：检测虫子，打印检测框中心/底部的深度值(mm)。

目的：确认深度相机能不能量到虫子的深度，还是"深度黑洞"=0。
跑法（先 sudo systemctl stop spiderpi）：
    python3 probe_bug_depth.py [--model models/v8n.onnx] [--conf 0.5]

每检测到一次就打印一行：中心像素、框高、中心深度、底部(虫子下方)深度。
把虫子放到相机视角里（离地任意高度），看打印里中心/底部深度是不是 0。
"""
import os
import sys
import time
import ctypes
import argparse

import cv2
import numpy as np

from common.ros_robot_controller_sdk import Board

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------- 深度相机（OpenNI2，内联最小版）----------------
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

    def close(self):
        if self._stream:
            self.lib.oniStreamDestroy(self._stream)
        if self._device:
            self.lib.oniDeviceClose(self._device)
        self.lib.oniShutdown()


# ---------------- YOLO 检测器（内联，与其它脚本一致）----------------
class ModelDetector:
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
        return {'center': (cx, cy), 'h': float(y2 - y1), 'conf': float(score)}


def main():
    parser = argparse.ArgumentParser(description='探测虫子深度')
    parser.add_argument('--model', default='models/v8n.onnx', help='YOLO 模型路径')
    parser.add_argument('--conf', type=float, default=0.5, help='YOLO 置信度')
    args = parser.parse_args()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()

    model_path = args.model if os.path.isabs(args.model) else os.path.join(_PKG_ROOT, args.model)
    model_det = ModelDetector(model_path, args.conf)
    print('模型已加载：%s' % model_path, flush=True)

    depth = DepthCam()
    depth.open()
    print('深度相机已打开', flush=True)
    print('把虫子放到相机视角里，看打印。Ctrl+C 退出。', flush=True)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            r = model_det.detect(frame)
            if r is None:
                time.sleep(0.05)
                continue
            cx, cy = r['center']
            h = r.get('h', 0.0)
            d = depth.read(100)
            if d is None:
                print('深度读不到', flush=True)
                time.sleep(0.1)
                continue
            dh, dw = d.shape
            px = int(min(max(cx, 0), dw - 1))
            py = int(min(max(cy, 0), dh - 1))
            by = int(min(max(cy + h / 2 + 10, 0), dh - 1))  # 虫子底部再往下一点
            z_center = int(d[py, px])
            z_bottom = int(d[by, px])
            print('中心=(%.0f,%.0f) 框高=%.0f | 中心深度=%dmm 底部深度=%dmm'
                  % (cx, cy, h, z_center, z_bottom), flush=True)
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        depth.close()


if __name__ == '__main__':
    main()
