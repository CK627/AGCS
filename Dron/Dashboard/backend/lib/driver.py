# ============================================================
# 文件名：driver.py
# 作用：负责和无人机“建立连接、收发数据”。
#       它根据运行环境选择用串口还是 TCP 连接，
#       并把飞机发回来的数据交给 flyData.py 去解析。
# 说明：以下每一行代码都加了中文注释。
# ============================================================

# 引入系统模块、线程模块和 flyData（用来解析飞机数据）
import sys, _thread, flyData
# 从 time 里引入 sleep，用来延时
from time import sleep

# 下面 4 个是“运行环境”编号，用来区分不同的开发板
MindPlus = 0   # Mind+ 电脑端软件环境
ESP32 = 1      # ESP32 开发板
mPython = 2    # 掌控板（mPython）
Pico = 3       # Raspberry Pi Pico 开发板

try:
    # 尝试导入单片机专用的模块，如果成功说明是在单片机里运行
    import uos
    from utime import ticks_ms
    sysname = uos.uname().sysname       # 读取系统名称
    if sysname == "esp32":              # 如果是 ESP32
        driverType = ESP32              # 环境标记为 ESP32
    elif sysname == "rp2":              # 如果是 RP2（Pico）
        driverType = Pico               # 环境标记为 Pico
except Exception as e:
    # 上面导入失败，说明不是单片机，而是电脑端（Mind+）
    try:
        from time import time           # 改用电脑上的 time 函数
        driverType = MindPlus           # 环境标记为 Mind+
    finally:
        # 清理临时异常变量
        e = None
        del e

if True:
    # 只有“成功识别为单片机”时才会执行下面的代码

    class Rx(object):
        # 这个类保存“接收数据”的中间状态，相当于一个接收缓冲区。
        head = 0       # 帧头
        len = 0        # 数据长度
        date = []      # 收到的数据内容
        cnt = 0        # 已经收到的字节数
        state = 0      # 当前解析状态（0~3 四种状态）
        buff = []      # 备用缓冲区
        fps = 0        # 每秒帧数
        fpsCnt = 0     # 帧数计数器


    class init:
        # 真正的驱动类：创建它就能连接飞机。

        def __init__(self, flyNum=1):
            # 初始化：准备通信端口和数据。
            self.rx = Rx()                              # 新建一个接收缓冲区
            self.flyData = flyData.init(flyNum)         # 新建传感器数据对象
            self.loop1Hz = self.getSec()                # 记录当前时间，用于每秒统计帧率
            if driverType == ESP32:                     # 如果是 ESP32
                from machine import UART, Pin           # 导入串口模块
                self.uart = UART(2, 500000)             # 打开 2 号串口，波特率 500000
                self.uartEnable = True                  # 标记串口可用
                self.type = "ESP32"                     # 记录环境类型
            elif driverType == mPython:                 # 如果是掌控板
                from machine import UART, Pin           # 导入串口模块
                self.uart = UART(1, baudrate=500000, tx=(Pin.P16), rx=(Pin.P15))  # 打开 1 号串口，指定收发引脚
                self.uartEnable = True                  # 标记串口可用
                self.type = "mPython"                   # 记录环境类型
            elif driverType == Pico:                    # 如果是 Pico
                from machine import UART, Pin           # 导入串口模块
                self.uart = UART(1, baudrate=500000, tx=(Pin(4)), rx=(Pin(5)))    # 打开 1 号串口，指定引脚
                self.uartEnable = True                  # 标记串口可用
                self.type = "Pico"                      # 记录环境类型
            else:                                       # 否则就是电脑端 Mind+
                import tcpClient                        # 导入 TCP 客户端
                from mySerial import vcp                # 导入虚拟串口
                self.tcp = tcpClient.init(port=8003)    # 连接 8003 端口
                sleep(0.5)                              # 等 0.5 秒，让连接稳定
                self.uart = vcp()                       # 打开虚拟串口
                if self.uart.device:                    # 如果找到了串口设备
                    self.uartEnable = True              # 标记串口可用
                else:                                   # 没找到串口设备
                    self.uartEnable = False             # 标记串口不可用
                self.type = "MindPlus"                  # 记录环境类型
            if self.uartEnable:                          # 如果串口可用
                _thread.start_new_thread(self.Receive_Thread, ())   # 启动后台接收线程
                print("准备就绪，开始起飞\n")            # 提示准备完成
            else:                                        # 串口不可用
                sys.exit(1)                              # 直接退出程序

        def getSec(self):
            # 获取当前时间（秒）。
            if driverType == MindPlus:   # 电脑端
                return time()            # 用 time() 获取秒数
            return ticks_ms() * 0.001    # 单片机端：毫秒转成秒

        def write(self, buff):
            # 把数据通过串口发出去。
            self.uart.write(buff)

        def Receive_Prepare(self, data):
            # 接收一个字节，按“状态机”逐步拼出一帧完整数据。
            if self.rx.state == 0:            # 状态 0：等待帧头
                if data == 170:               # 如果这个字节是帧头 170
                    self.rx.state = 1         # 进入状态 1
                    self.rx.head = data       # 保存帧头
            elif self.rx.state == 1:          # 状态 1：等待长度
                if data > 0 and data < 30:    # 长度必须是 1~29 之间的合理值
                    self.rx.state = 2         # 进入状态 2
                    self.rx.len = data        # 保存长度
                    self.rx.cnt = 0           # 清零字节计数器
                else:                         # 长度不合法
                    self.rx.state = 0         # 回到状态 0，重新等帧头
            elif self.rx.state == 2:          # 状态 2：接收数据内容
                self.rx.date.append(data)     # 把这个字节存进数据列表
                self.rx.cnt = self.rx.cnt + 1 # 计数器加 1
                if self.rx.cnt >= self.rx.len:  # 如果收够了长度
                    self.rx.state = 3         # 进入状态 3
            elif self.rx.state == 3:          # 状态 3：收最后一个校验字节
                self.rx.state = 0             # 状态归零，准备接收下一帧
                self.rx.date.append(data)     # 把校验字节也存进去
                self.Receive_Anl()            # 开始解析这一帧数据
                self.rx.date = []             # 清空数据，准备下一帧
            else:                             # 其它异常状态
                self.rx.state = 0             # 一律回到状态 0

        def Receive_Anl(self):
            # 校验收到的数据是否正确，正确就交给 flyData 解析。
            sum = 0                         # 求和变量
            sum = self.rx.head + self.rx.len  # 先把帧头和长度加起来
            for temp in self.rx.date:       # 再把数据内容逐个加上
                sum = sum + temp
            # 用总和去掉校验字节本身，再取低 8 位，得到理论上的校验值
            sum = (sum - self.rx.date[self.rx.len]) % 256
            if sum != self.rx.date[self.rx.len]:   # 如果算出来的校验值和收到的校验值不一致
                return                        # 数据有误，直接丢弃
            self.rx.fpsCnt = self.rx.fpsCnt + 1   # 帧数加 1
            if self.rx.head == 170:               # 如果帧头是 170（普通数据帧）
                self.flyData.Receive_Anl(self.rx) # 交给 flyData 去解析

        def Receive_Thread(self):
            # 后台线程：不停地读取串口数据。
            while True:                              # 无限循环
                packUart = self.uart.read(self.uart.any())  # 读出串口里当前所有的数据
                size = len(packUart)                 # 看有多少字节
                for i in range(size):                # 逐个字节处理
                    self.Receive_Prepare(packUart[i])
                self.Tcp_WrietAndRead(packUart)      # 如果是 Mind+，顺便把数据转给电脑软件
                nowTime = self.getSec()              # 获取当前时间
                if nowTime - self.loop1Hz >= 0.98:   # 大约每 1 秒统计一次帧率
                    self.loop1Hz = nowTime           # 更新统计时间
                    self.rx.fps = self.rx.fpsCnt     # 把本秒收到的帧数保存起来
                    self.rx.fpsCnt = 0               # 帧计数器清零
                sleep(0.02)                          # 每次循环休息 0.02 秒

        def Tcp_WrietAndRead(self, pack):
            # 只在 Mind+ 环境使用：把数据发给电脑上的编队软件。
            if driverType == MindPlus:               # 判断是否电脑端
                packVcp = self.tcp.request(pack)     # 通过 TCP 发送并读取回复
                if packVcp != None:                  # 如果电脑有回复数据
                    self.uart.write(packVcp)         # 把回复写回串口

        def run(self, time=-1):
            # 让程序“跑一段时间”或“跑完提示”。
            if time >= 0:               # 如果传入了正数时间
                sleep(time)             # 就等待相应时间
            else:                       # 否则表示程序运行结束
                print("程序运行完毕！")  # 提示运行完毕
                sleep(1)                # 等 1 秒
