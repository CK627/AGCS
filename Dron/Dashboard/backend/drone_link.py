# -*- coding: utf-8 -*-
"""无人机串口链路检测（识别规则移植自 外接图传录制程序/lib/mySerial.py）。

这里只做“枚举 + 识别”，不打开串口、不占用设备，避免和 OpenFly 编队软件抢串口：
- OpenFly 遥控器虚拟串口有两个硬件编号（VID:PID），与 mySerial.vcp 完全一致；
- 仪表盘启动时调用它，判断“遥控器/无人机链路是否插在电脑上”，
  和摄像头信号检测一起组成启动自检。
"""

try:
    import serial.tools.list_ports
except ImportError:  # 没装 pyserial 时优雅降级
    serial = None


# 与 lib/mySerial.py 中 vcp 识别无人机的两个 VID:PID 保持一致
DRONE_VID_PIDS = ("VID:PID=0483:5740", "VID:PID=1209:ABD1")


def find_drone_serial():
    """返回识别到的无人机串口设备名列表（如 ['COM3']）；未连接时返回 []。"""
    if serial is None:
        return []
    found = []
    try:
        for port in serial.tools.list_ports.comports():
            hwid = port.hwid or ""
            if any(vid in hwid for vid in DRONE_VID_PIDS):
                found.append(port.device)
    except Exception:
        return []
    return found


def serial_available():
    """是否装有 pyserial（用于状态提示）。"""
    return serial is not None
