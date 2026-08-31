# ============================================================
# 文件名：mySerial.py
# 作用：在电脑端找到并打开无人机用的“虚拟串口”，
#       负责通过串口收发数据。
# 说明：
#   1. 以下每一行代码都加了中文注释。
#   2. read() 和 any() 这两个方法在原文件里是编译后的字节码，
#      这里已根据字节码手动还原成可读的 Python 代码。
# ============================================================

# 引入串口库，以及用于列出串口设备的工具
import serial, serial.tools.list_ports
# 引入 time，用于给错误提示做节流（避免设备中途拔掉时日志刷屏）
import time


# 每个错误标签 1 秒最多打印一次
_PRINT_TICKS = {}


def _log_once(tag, message):
    now = time.time()
    if now - _PRINT_TICKS.get(tag, 0) >= 1.0:
        _PRINT_TICKS[tag] = now
        print(message)


class vcp:
    # 虚拟串口类：打开串口、读写数据。

    def __init__(self):
        # 初始化：自动寻找并打开无人机对应的串口。
        self.device = False                        # 先用 False 表示“还没找到串口”
        ports = serial.tools.list_ports.comports() # 列出电脑上所有串口设备
        for p in ports:                            # 逐个串口检查
            # 如果这个串口是我们需要的无人机（两种型号的硬件编号之一）
            if "VID:PID=0483:5740" in p.hwid or "VID:PID=1209:ABD1" in p.hwid:
                self.device = p.device             # 保存串口号（比如 COM3）
                break                              # 找到就停止查找
        if self.device:                            # 如果找到了串口
            try:                                   # 尝试打开
                # 用 500000 波特率、无超时打开这个串口
                self.usart = serial.Serial(self.device, 500000, timeout=0)
            except Exception as e:                 # 打开失败
                try:
                    # 提示用户重启或关闭编队软件
                    print("串口打开异常，请尝试重启编队软件或者直接关闭编队软件！")
                    self.device = False            # 标记为“没有可用串口”
                finally:
                    e = None                       # 清理异常变量
                    del e
        else:                                      # 没找到串口
            # 提示检查遥控器是否连接成功
            print("找不到串口设备,请检查遥控器是否成功连接到电脑！")

    def write(self, pack):
        # 通过串口发送数据。
        try:                           # 尝试发送
            self.usart.write(pack)     # 把数据写进串口
        except Exception as e:         # 发送失败
            try:
                print("串口数据发送异常：", e)  # 打印错误
            finally:
                e = None               # 清理异常变量
                del e

    def read(self, size):
        # 从串口读取 size 个字节。
        try:                                  # 尝试读取
            return self.usart.read(size=size) # 返回读到的数据
        except Exception as e:                # 读取失败
            try:
                _log_once("read", "串口数据接收异常：%s" % e)  # 节流打印错误
            finally:
                e = None                      # 清理异常变量
                del e
            return bytearray()                # 失败时返回空数据

    def any(self):
        # 返回串口缓冲区里还有多少个字节没读。
        try:                                 # 尝试查询
            return self.usart.inWaiting()    # 返回待读字节数
        except Exception as e:               # 查询失败
            try:
                _log_once("any", "串口数据接收异常：%s" % e)  # 节流打印错误
            finally:
                e = None                     # 清理异常变量
                del e
            return 0                         # 失败时返回 0

    def writeStr(self, str):
        # 把一段文字按 UTF-8 编码后通过串口发出去。
        self.usart.write(str.encode("utf-8"))
