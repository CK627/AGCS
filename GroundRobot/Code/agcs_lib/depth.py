#!/usr/bin/python3
# coding=utf8
"""奥比中光 Astra Pro 深度摄像头读取封装（ctypes 直调 libOpenNI2.so）。

依赖机器人上 /home/pi/orbbec_sdk/ 的自包含 OpenNI2 运行时（配套的
libOpenNI2.so + liborbbec.so，系统 apt 装的 OpenNI2 不配套，枚举不到设备）。

深度流走 OpenNI2（USB 2bc5:0403），单位毫米。彩色设备 2bc5:0501 被内核
uvcvideo 占用（就是 /dev/video0），所以 OpenNI2 的 SENSOR_COLOR 会超时
读不到帧——彩色图直接用 OpenCV 读 /dev/video0，不要走本模块 read_color()。
read_color() 仅在将来 detach uvcvideo 后才可能生效，当前保留作参考。

用法：
    from agcs_lib.depth import DepthCamera
    cam = DepthCamera()
    cam.open()
    cam.start_depth()
    depth = cam.read_depth()   # np.uint16 (H, W)，单位 mm；超时返回 None
    cam.close()
"""
import ctypes

import numpy as np

# ---- OniCEnums.h ----
ONI_STATUS_OK = 0
ONI_STATUS_ERROR = 1
ONI_STATUS_TIME_OUT = 102

ONI_SENSOR_IR = 1
ONI_SENSOR_COLOR = 2
ONI_SENSOR_DEPTH = 3

ONI_PIXEL_FORMAT_DEPTH_1_MM = 100
ONI_PIXEL_FORMAT_RGB888 = 200
ONI_PIXEL_FORMAT_YUV422 = 201
ONI_PIXEL_FORMAT_YUYV = 205

ONI_API_VERSION = 2002
ONI_TIMEOUT_NONE = 0


# ---- OniCTypes.h 结构体（AArch64 LP64 对齐，ctypes 会自动算 offset）----
class OniVideoMode(ctypes.Structure):
    _fields_ = [
        ("pixelFormat", ctypes.c_int),
        ("resolutionX", ctypes.c_int),
        ("resolutionY", ctypes.c_int),
        ("fps", ctypes.c_int),
    ]


class OniFrame(ctypes.Structure):
    _fields_ = [
        ("dataSize", ctypes.c_int),
        ("data", ctypes.c_void_p),
        ("sensorType", ctypes.c_int),
        ("timestamp", ctypes.c_uint64),
        ("frameIndex", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("videoMode", OniVideoMode),
        ("croppingEnabled", ctypes.c_int),
        ("cropOriginX", ctypes.c_int),
        ("cropOriginY", ctypes.c_int),
        ("stride", ctypes.c_int),
    ]


class OniDeviceInfo(ctypes.Structure):
    _fields_ = [
        ("uri", ctypes.c_char * 256),
        ("vendor", ctypes.c_char * 256),
        ("name", ctypes.c_char * 256),
        ("usbVendorId", ctypes.c_uint16),
        ("usbProductId", ctypes.c_uint16),
    ]


class DepthCameraError(RuntimeError):
    pass


class DepthCamera:
    """打开 Astra Pro，读深度图 / 彩色图。"""

    DEFAULT_LIB = "/home/pi/orbbec_sdk/libOpenNI2.so"

    def __init__(self, lib_path=DEFAULT_LIB):
        self.lib = ctypes.CDLL(lib_path)
        self._device = ctypes.c_void_p()
        self._depth_stream = ctypes.c_void_p()
        self._color_stream = ctypes.c_void_p()
        self._intrinsics = None
        self._bind()

    def _bind(self):
        lib = self.lib
        # general
        lib.oniInitialize.argtypes = [ctypes.c_int]
        lib.oniInitialize.restype = ctypes.c_int
        lib.oniShutdown.argtypes = []
        lib.oniShutdown.restype = None
        lib.oniGetExtendedError.restype = ctypes.c_char_p
        # device list
        lib.oniGetDeviceList.argtypes = [
            ctypes.POINTER(ctypes.POINTER(OniDeviceInfo)),
            ctypes.POINTER(ctypes.c_int),
        ]
        lib.oniGetDeviceList.restype = ctypes.c_int
        lib.oniReleaseDeviceList.argtypes = [ctypes.POINTER(OniDeviceInfo)]
        lib.oniReleaseDeviceList.restype = ctypes.c_int
        # device
        lib.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
        lib.oniDeviceOpen.restype = ctypes.c_int
        lib.oniDeviceClose.argtypes = [ctypes.c_void_p]
        lib.oniDeviceClose.restype = ctypes.c_int
        lib.oniDeviceCreateStream.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p),
        ]
        lib.oniDeviceCreateStream.restype = ctypes.c_int
        # stream
        lib.oniStreamStart.argtypes = [ctypes.c_void_p]
        lib.oniStreamStart.restype = ctypes.c_int
        lib.oniStreamStop.argtypes = [ctypes.c_void_p]
        lib.oniStreamStop.restype = None
        lib.oniStreamDestroy.argtypes = [ctypes.c_void_p]
        lib.oniStreamDestroy.restype = None
        lib.oniStreamReadFrame.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(OniFrame)),
        ]
        lib.oniStreamReadFrame.restype = ctypes.c_int
        lib.oniFrameRelease.argtypes = [ctypes.POINTER(OniFrame)]
        lib.oniFrameRelease.restype = None
        lib.oniWaitForAnyStream.argtypes = [
            ctypes.POINTER(ctypes.c_void_p), ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.c_int,
        ]
        lib.oniWaitForAnyStream.restype = ctypes.c_int
        # 坐标转换（用驱动内置的工厂标定）
        lib.oniCoordinateConverterDepthToWorld.argtypes = [
            ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        lib.oniCoordinateConverterDepthToWorld.restype = ctypes.c_int

    # ---- 生命周期 ----
    def open(self):
        rc = self.lib.oniInitialize(ONI_API_VERSION)
        if rc != ONI_STATUS_OK:
            raise DepthCameraError("oniInitialize 失败: %d" % rc)

        devices = ctypes.POINTER(OniDeviceInfo)()
        num = ctypes.c_int(0)
        rc = self.lib.oniGetDeviceList(ctypes.byref(devices), ctypes.byref(num))
        if rc != ONI_STATUS_OK or num.value == 0:
            raise DepthCameraError("未找到 Orbbec 设备 (num=%d)" % num.value)
        uri = bytes(devices[0].uri)
        self.lib.oniReleaseDeviceList(devices)

        rc = self.lib.oniDeviceOpen(uri, ctypes.byref(self._device))
        if rc != ONI_STATUS_OK:
            raise DepthCameraError("打开设备失败: %d" % rc)

    def start_depth(self):
        rc = self.lib.oniDeviceCreateStream(
            self._device, ONI_SENSOR_DEPTH, ctypes.byref(self._depth_stream))
        if rc != ONI_STATUS_OK:
            raise DepthCameraError("创建深度流失败: %d" % rc)
        rc = self.lib.oniStreamStart(self._depth_stream)
        if rc != ONI_STATUS_OK:
            raise DepthCameraError("启动深度流失败: %d" % rc)

    def start_color(self):
        rc = self.lib.oniDeviceCreateStream(
            self._device, ONI_SENSOR_COLOR, ctypes.byref(self._color_stream))
        if rc != ONI_STATUS_OK:
            raise DepthCameraError("创建彩色流失败: %d" % rc)
        rc = self.lib.oniStreamStart(self._color_stream)
        if rc != ONI_STATUS_OK:
            raise DepthCameraError("启动彩色流失败: %d" % rc)

    def close(self):
        if self._depth_stream:
            self.lib.oniStreamDestroy(self._depth_stream)
            self._depth_stream = ctypes.c_void_p()
        if self._color_stream:
            self.lib.oniStreamDestroy(self._color_stream)
            self._color_stream = ctypes.c_void_p()
        if self._device:
            self.lib.oniDeviceClose(self._device)
            self._device = ctypes.c_void_p()
        self.lib.oniShutdown()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- 读帧 ----
    def _read_frame(self, stream, timeout_ms):
        """带超时读一帧，返回 OniFrame 指针；超时/失败返回 None。"""
        if timeout_ms:
            idx = ctypes.c_int(0)
            streams = (ctypes.c_void_p * 1)(stream)
            rc = self.lib.oniWaitForAnyStream(
                streams, 1, ctypes.byref(idx), timeout_ms)
            if rc != ONI_STATUS_OK:
                return None
        frame = ctypes.POINTER(OniFrame)()
        rc = self.lib.oniStreamReadFrame(stream, ctypes.byref(frame))
        if rc != ONI_STATUS_OK:
            return None
        return frame

    def read_depth(self, timeout_ms=2000):
        """读一帧深度图，返回 np.uint16 (H, W)，单位 mm；超时返回 None。"""
        frame = self._read_frame(self._depth_stream, timeout_ms)
        if frame is None:
            return None
        try:
            f = frame.contents
            w, h = f.width, f.height
            stride_px = f.stride // 2  # uint16 每行像素数
            raw = ctypes.string_at(f.data, f.dataSize)
            arr = np.frombuffer(raw, dtype=np.uint16)
            arr = arr[:stride_px * h].reshape(h, stride_px)
            return arr[:, :w].copy()
        finally:
            self.lib.oniFrameRelease(frame)

    def read_color(self, timeout_ms=2000):
        """读一帧彩色图，返回 np.uint8 (H, W, 3) RGB；超时/格式不支持返回 None。"""
        frame = self._read_frame(self._color_stream, timeout_ms)
        if frame is None:
            return None
        try:
            f = frame.contents
            w, h = f.width, f.height
            fmt = f.videoMode.pixelFormat
            if fmt == ONI_PIXEL_FORMAT_RGB888:
                stride_px = f.stride // 3
                raw = ctypes.string_at(f.data, f.dataSize)
                arr = np.frombuffer(raw, dtype=np.uint8)
                arr = arr[:stride_px * h * 3].reshape(h, stride_px, 3)
                return arr[:, :w, :].copy()
            # 其它格式（YUV422/YUYV 等）暂不转换，交由调用方决定
            return None
        finally:
            self.lib.oniFrameRelease(frame)

    # ---- 内参 / 坐标转换 / 点云 ----
    def depth_to_world(self, x, y, z):
        """深度像素 (x, y) + 深度值 z(mm) → 世界坐标 (X, Y, Z) mm（工厂标定）。"""
        wx = ctypes.c_float()
        wy = ctypes.c_float()
        wz = ctypes.c_float()
        rc = self.lib.oniCoordinateConverterDepthToWorld(
            self._depth_stream, float(x), float(y), float(z),
            ctypes.byref(wx), ctypes.byref(wy), ctypes.byref(wz))
        if rc != ONI_STATUS_OK:
            return None
        return (wx.value, wy.value, wz.value)

    def get_depth_intrinsics(self, probe_z=2000.0):
        """用工厂标定坐标转换器反推深度内参，返回 (fx, fy, cx, cy)。

        不假设主点在图像中心，直接从边缘探点反推。
        """
        if self._intrinsics is not None:
            return self._intrinsics
        d = self.read_depth(timeout_ms=2000)
        if d is None:
            raise DepthCameraError("读深度失败，无法标定")
        h, w = d.shape
        # 水平两边缘：X = (u-cx)*z/fx → fx = (w-1)*z/(x1-x0)，cx = -x0*fx/z
        x0, _, _ = self.depth_to_world(0.0, h / 2.0, probe_z)
        x1, _, _ = self.depth_to_world(float(w - 1), h / 2.0, probe_z)
        fx = (w - 1) * probe_z / (x1 - x0)
        cx = -x0 * fx / probe_z
        # 垂直两边缘：Y = (cy-v)*z/fy → fy = (h-1)*z/(y0-y1)，cy = y0*fy/z
        _, y0, _ = self.depth_to_world(w / 2.0, 0.0, probe_z)
        _, y1, _ = self.depth_to_world(w / 2.0, float(h - 1), probe_z)
        fy = (h - 1) * probe_z / (y0 - y1)
        cy = y0 * fy / probe_z
        self._intrinsics = (fx, fy, cx, cy)
        return self._intrinsics

    def depth_to_pointcloud(self, depth):
        """深度图 (H,W) uint16(mm) → 点云 (H,W,3) float32 世界坐标 mm。

        无效像素(0)置 NaN。坐标约定：X 向右、Y 向上、Z 向前（深度）。
        """
        fx, fy, cx, cy = self.get_depth_intrinsics()
        h, w = depth.shape
        u = np.arange(w, dtype=np.float32)[None, :]
        v = np.arange(h, dtype=np.float32)[:, None]
        z = depth.astype(np.float32)
        x = (u - cx) * z / fx
        y = (cy - v) * z / fy
        pcl = np.stack([x, y, z], axis=-1)
        pcl[z <= 0] = np.nan
        return pcl
