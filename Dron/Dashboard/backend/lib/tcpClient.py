# ============================================================
# 文件名：tcpClient.py
# 作用：一个简单的 TCP 客户端，用来和电脑上的
#       OpenFly 编队软件、AI 服务进行网络通信。
# 说明：以下每一行代码都加了中文注释。
# ============================================================

# 引入 socket 模块，用来做网络通信
import socket


class init:
    # TCP 客户端类。

    def __init__(self, port=8003, timeOut=0):
        # 初始化：连接本机的指定端口。
        try:
            # 新建一个 TCP 套接字
            self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_client.settimeout(0.1)                 # 连接时最多等 0.1 秒
            self.tcp_client.connect(("localhost", port))    # 连接本机的指定端口
            self.tcp_client.settimeout(timeOut)             # 连接成功后，把超时改成传入值
        except Exception as e:                             # 连接失败
            try:
                pass                                        # 先什么都不做
            finally:
                e = None                                    # 清理异常变量
                del e

    def request(self, date):
        # 发送数据，并接收服务器回复。
        try:
            self.tcp_client.send(date)        # 发送数据
            recv = self.tcp_client.recv(65535)  # 接收最多 65535 字节的回复
        except Exception as e:                # 收发失败
            try:
                recv = bytearray()            # 返回空数据
            finally:
                e = None                      # 清理异常变量
                del e
        else:
            return recv                       # 正常情况返回收到的数据
