# ============================================================
# 文件名：helloFly.py
# 作用：这是整个无人机 SDK 里最核心的文件，
#       里面定义了 class fly（无人机控制类），
#       起飞、降落、移动、旋转、灯光、机械臂等
#       所有控制无人机的函数都在这个类里。
# 说明：以下每一行代码后面都加了中文注释，用大白话解释。
# ============================================================

# 引入系统、驱动、数学相关的模块
import os, driver, math
# 从 math 里引入开平方函数 sqrt，用来算两点之间的距离
from math import sqrt
# 从 struct 里引入 pack，用来把数字打包成二进制字节，方便发给飞机
from struct import pack
# 从 random 里引入 randint，用来生成随机整数
from random import randint


class order(object):
    # 这个类是一张“命令编号表”，每个编号代表一种无人机动作。
    # 发送指令时，实际上就是告诉飞机一个数字编号。
    takeOff = 0            # 0 号命令：起飞
    flyMode = 1            # 1 号命令：飞行模式
    xySpeed = 2            # 2 号命令：水平速度
    zSpeed = 3             # 3 号命令：垂直（上下）速度
    followLine = 4         # 4 号命令：循线飞行
    moveCtrl = 5           # 5 号命令：手动方向移动
    moveSearchDot = 6      # 6 号命令：找点移动
    moveSearchTag = 7      # 7 号命令：找标签移动
    moveSearchBlob = 8     # 8 号命令：找色块移动
    goTo = 9               # 9 号命令：飞到指定坐标
    rotation = 10          # 10 号命令：旋转
    flyHigh = 11           # 11 号命令：飞到指定高度
    flipCtrl = 12          # 12 号命令：翻滚
    ledCtrl = 13           # 13 号命令：控制灯光
    mvMode = 14            # 14 号命令：视觉识别模式
    magnetCtrl = 15        # 15 号命令：控制磁吸
    servoCtrl = 16         # 16 号命令：控制舵机
    roleCtrl = 17          # 17 号命令：控制角色（机器人任务）
    lockDir = 18           # 18 号命令：锁定方向
    shootCtrl = 19         # 19 号命令：控制射击
    switchCtrl = 20        # 20 号命令：控制开关
    moveFollowTag = 21     # 21 号命令：跟随标签移动
    photographMode = 22    # 22 号命令：拍照模式
    goToTag = 23           # 23 号命令：飞到标签位置
    move = 24              # 24 号命令：坐标移动
    serServoCtrl = 25      # 25 号命令：串口舵机控制
    robotArmCtrl = 26      # 26 号命令：机械臂控制
    robotArmRecord = 27    # 27 号命令：机械臂动作录制
    robotArmFree = 28      # 28 号命令：机械臂自由转动
    robotArmCover = 29     # 29 号命令：机械臂抓取/覆盖
    setLocation = 30       # 30 号命令：设置当前位置坐标
    setTagDistance = 31    # 31 号命令：设置标签距离
    setCenterOffset = 32   # 32 号命令：设置中心偏移
    cameraDeg = 33         # 33 号命令：摄像头角度
    showStr = 34           # 34 号命令：显示字符串
    showCtrl = 35          # 35 号命令：显示控制
    irSend = 36            # 36 号命令：红外发送
    tofSwitch = 37         # 37 号命令：红外测距开关
    horAround = 38         # 38 号命令：水平绕圈飞行
    verAround = 39         # 39 号命令：垂直绕圈飞行
    sinAround = 40         # 40 号命令：正弦轨迹绕飞
    flyDir = 41            # 41 号命令：设定飞行方向
    subFlowSwitch = 42     # 42 号命令：子流程开关
    blobArea = 43          # 43 号命令：色块面积设置
    fmaInit = 253          # 253 号命令：飞机初始化
    flyCtrl = 254          # 254 号命令：飞行总控制
    nullCmd = 255          # 255 号命令：空命令（用来占位/初始化）


class fly:
    # 这是核心类：创建一台无人机后，调用它下面的方法就能控制飞机。











    def __init__(self, maxNum=1):
        # 初始化函数：创建无人机对象时自动执行。
        print("HelloFly:2025-06-06")   # 打印版本信息，表示库加载成功
        self.maxNum = maxNum           # 保存飞机数量，默认 1 架
        self.repeatCountMax = 0        # 重发次数上限，默认不重发
        self.port = driver.init(self.maxNum)   # 打开通信端口，连接飞机
        self.timerStart = self.getTicks_sec()  # 记录程序开始时间，用作计时起点
        # 为每架飞机建立一个“预览位置”列表，初始坐标都是 [0,0,0]
        self.previewLoc = [[0, 0, 0] for _ in range(self.maxNum)]
        # 为每架飞机建立一个“预览颜色”列表，初始 RGB 都是 [0,0,0]
        self.previewRgb = [[0, 0, 0] for _ in range(self.maxNum)]
        # 为每架飞机生成一个随机序号，用来判断指令有没有被飞机回复
        self.orderCount = [randint(1, 255) for _ in range(self.maxNum)]
        # 每架无人机的指令数据包序列号，范围1~255，发送指令时自增，用于识别重复数据包
        self.timeSec = [self.getTicks_sec() for _ in range(self.maxNum)]
        # 记录每架飞机最近一次发送文字的时间
        self.isDelay = [True for _ in range(self.maxNum)]
       # 是否启用自动等待，默认全部启用
        self.packData = [bytearray() for _ in range(self.maxNum)]
        # 每架飞机各准备一个空的数据包缓存
        self.horSpeed = [100 for _ in range(self.maxNum)]
         # 水平速度默认 100
        self.verSpeed = [100 for _ in range(self.maxNum)]
        # 垂直速度默认 100
        for id in range(self.maxNum):            # 逐个初始化每一架飞机
            self.sendOrderPack(id, order.nullCmd, bytearray())   # 发送一条空命令，让飞机进入待命状态
            if self.port.type == "MindPlus":                      # 如果是在 Mind+ 环境里运行
                self.sendOrderPack(id, order.fmaInit, bytearray())  # 再发一条初始化命令
            self.sleep(0.1)                                      # 等 0.1 秒，让飞机有时间处理

    def getTicks_sec(self):
        # 获取从开机到现在经过的秒数。
        return self.port.getSec()

    def getTimer(self):
        # 获取从程序开始计时到现在的秒数。
        return self.getTicks_sec() - self.timerStart

    def clearTimer(self):
        # 把计时起点重新设成“现在”，相当于秒表归零。
        self.timerStart = self.getTicks_sec()

    def showText(self, id, string):
        # 在控制台打印一段文字，并顺便显示距离上次打印过了多少秒。
        nowTime = self.getTicks_sec()                 # 拿到当前时间
        dT = nowTime - self.timeSec[id]               # 计算和上次相差多少秒
        self.timeSec[id] = nowTime                    # 更新“上次时间”为现在
        if self.port.type == "OpenMV":                # 如果是 OpenMV 环境
            print("---" + "%.2f" % dT + "s")          # 打印间隔时间（保留两位小数）
            print(string)                              # 打印真正要显示的文字
        else:                                          # 其它环境（如 Mind+）
            print("\x1b[1;34m---" + "%.2f" % dT + "s" + "\x1b[0m")  # 用蓝色打印间隔时间
            print("\x1b[1;30m" + string + "\x1b[0m")                  # 用深色打印文字

    def pyLink_pack(self, head, fun, buff):
        # 把要发送的数据打包成飞机能识别的格式。
        packData = bytearray([head, 0, fun])   # 开头先放：帧头、长度（先占位填 0）、命令
        packData.extend(buff)                  # 把真正要发送的数据追加进去
        packData[1] = len(packData) - 2        # 计算并填写“数据长度”
        sum = 0                                # 求和变量，用来算校验值
        for temp in packData:                  # 把前面所有字节逐个相加
            sum = sum + temp
        packData.extend(pack("<B", sum % 256))  # 取总和的低 8 位作为校验字节，追加到末尾
        return packData                        # 返回打包好的数据

    def sendOrderPack(self, id, cmd, pack):
        # 真正把一条命令发送给指定的飞机。
        self.orderCount[id] = self.orderCount[id] + 1        # 序号加 1
        self.orderCount[id] = self.orderCount[id] % 256      # 序号限制在 0~255 之间循环
        buff = bytearray([id, cmd, self.orderCount[id] % 256])  # 组装：飞机编号、命令、序号
        buff = buff + pack                                   # 拼上要发送的数据
        dLen = 13 - len(buff)                                # 计算还差多少字节才能凑够固定长度
        if dLen > 0:                                         # 如果还不够长
            buff.extend(bytearray(dLen))                     # 用 0 补齐到固定长度
        # 把数据打包：帧头 187、功能码 243，数据重复两遍再补两个结束字节
        self.packData[id] = self.pyLink_pack(187, 243, buff + buff + bytearray([100, 0]))
        self.port.write(self.packData[id])                   # 把打包好的数据写进串口发出去

    def sendOrder(self, id, cmd, fmt, *args):
        # 一个更方便的发送函数：你传普通数字，它自动按格式打包并发送。
        self.sendOrderPack(id, cmd, pack(fmt, *args))

    def sleep(self, sec):
        # 让程序暂停 sec 秒（实际是让通信端口继续接收数据的等待）。
        self.port.run(sec)

    def setRepeatCountMax(self, max):
        # 设置指令没回应时最多重发几次。
        self.repeatCountMax = max

    def waitOrderAck(self, id):
        # 等待飞机回复“我收到指令了”。
        if self.repeatCountMax == 0:      # 如果设置成 0，表示不等待回复，直接返回
            return
        repeatCount = 0                   # 已经重发的次数
        timerStart = self.getTicks_sec()  # 记录开始等待的时间
        # 只要飞机回复的序号和我们发送的序号不一致，就一直等
        while self.orderCount[id] != self.port.flyData.flySensor[id].orderCount:
            if self.getTicks_sec() - timerStart >= 1:   # 如果已经等了超过 1 秒还没回复
                repeatCount = repeatCount + 1           # 重发次数加 1
                if repeatCount <= self.repeatCountMax:  # 还没超过允许的重发上限
                    self.port.write(self.packData[id])  # 把上一条指令再发一遍
                    timerStart = self.getTicks_sec()    # 重新开始计时
                    print(str(id) + " 号重发 " + str(repeatCount) + " 次...")  # 提示正在重发
                else:                                    # 超过重发上限，放弃
                    print(str(id) + " 号指令发送可能失败，请检查信号干扰情况！")  # 提示发送失败
                    break                                # 跳出等待
            self.sleep(0.1)                              # 每次等 0.1 秒再检查一次

    def moveDelay(self, id):
        # 移动类指令的等待：等飞机真的移动到目标位置附近才算完成。
        if self.isDelay[id]:            # 如果开启了自动等待
            self.sleep(1)               # 先等 1 秒让飞机开始动作
            dis = 100                   # 先假设误差很大
            while dis > 10:             # 只要定位误差还大于 10，就继续等
                self.sleep(0.1)         # 每次等 0.1 秒
                dx = self.port.flyData.flySensor[id].locErr[0]  # 取 x 方向误差
                dy = self.port.flyData.flySensor[id].locErr[1]  # 取 y 方向误差
                dz = self.port.flyData.flySensor[id].locErr[2]  # 取 z 方向误差
                dis = sqrt(dx * dx + dy * dy + dz * dz)          # 用勾股定理算总误差
            self.sleep(1)               # 误差达标后再等 1 秒稳定
        self.waitOrderAck(id)           # 最后再确认飞机已经回复指令

    def autoDelay(self, id, sec=0.1):
        # 一般指令的等待：等固定时间，再确认飞机回复。
        if self.isDelay[id]:    # 如果开启了自动等待
            self.sleep(sec)     # 等 sec 秒
        self.waitOrderAck(id)   # 确认飞机回复

    def flyPreviewUpdata(self, id):
        # 只在 Mind+ 环境下，把当前预览位置/颜色/速度刷新到电脑软件界面上。
        if self.port.type == "MindPlus":   # 判断是不是 Mind+ 环境
            # 把位置、颜色、速度等打包成一串字节
            previewPack = pack("<B3B3h3h2h", id, self.previewRgb[id][0], self.previewRgb[id][1], self.previewRgb[id][2], self.previewLoc[id][0], self.previewLoc[id][1], self.previewLoc[id][2], 0, 0, 0, self.horSpeed[id], self.verSpeed[id])
            # 通过 TCP 把预览数据发给电脑软件
            self.port.Tcp_WrietAndRead(self.pyLink_pack(170, 15, previewPack))

    def flyPreviewMove(self, id, dir, distance):
        # 根据方向编号，更新电脑界面上的“预览位置”。
        if dir == 1:       # 方向 1：向 y 正方向移动
            self.previewLoc[id][1] = self.previewLoc[id][1] + distance
        elif dir == 2:     # 方向 2：向 y 负方向移动
            self.previewLoc[id][1] = self.previewLoc[id][1] - distance
        elif dir == 3:     # 方向 3：向 x 负方向移动
            self.previewLoc[id][0] = self.previewLoc[id][0] - distance
        elif dir == 4:     # 方向 4：向 x 正方向移动
            self.previewLoc[id][0] = self.previewLoc[id][0] + distance
        elif dir == 5:     # 方向 5：向上移动
            self.previewLoc[id][2] = self.previewLoc[id][2] + distance
        elif dir == 6:     # 方向 6：向下移动
            self.previewLoc[id][2] = self.previewLoc[id][2] - distance
        self.flyPreviewUpdata(id)   # 刷新预览到电脑界面

    def flyCtrl(self, id, mode):
        # 飞行总控制：mode 不同，飞机进入不同状态（比如 0 表示降落/停止）。
        if mode == 0:                   # 如果模式是 0（降落/停止）
            self.previewLoc[id][2] = 0  # 把预览高度设成 0
            self.flyPreviewUpdata(id)   # 刷新预览
        self.sendOrder(id, order.flyCtrl, "<B", mode)       # 发送飞行控制命令
        self.showText(id, "flyCtrl(" + str(id) + "," + str(mode) + ")")  # 打印正在执行的命令
        self.autoDelay(id, 3)                               # 等待 3 秒并确认回复

    def takeOff(self, id, high):
        # 起飞：让飞机飞到指定高度。
        high = int(high + 0.5)            # 把高度四舍五入成整数
        self.previewLoc[id][2] = high     # 更新预览高度
        self.flyPreviewUpdata(id)         # 刷新预览
        # 发送起飞命令，附带高度等参数
        self.sendOrder(id, order.takeOff, "<h2B2h", high, 50, 0, 0, 0)
        self.showText(id, "takeOff(" + str(id) + "," + str(high) + ")")  # 打印起飞命令
        self.autoDelay(id, 3 + high / 100)   # 根据高度估算需要等待的时间

    def setAutoDelay(self, id, auto):
        # 设置是否自动等待飞机完成动作。
        self.isDelay[id] = auto                                       # 保存自动等待开关
        self.showText(id, "setAutoDelay(" + str(id) + "," + str(auto) + ")")  # 打印设置信息

    def subFlowSwitch(self, id, mode):
        # 子流程开关。
        self.sendOrder(id, order.subFlowSwitch, "<B", mode)   # 发送子流程开关命令
        self.showText(id, "subFlowSwitch(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def flyMode(self, id, mode):
        # 设置飞行模式。
        self.sendOrder(id, order.flyMode, "<B", mode)   # 发送飞行模式命令
        self.showText(id, "flyMode(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def setCenterOffset(self, id, offset):
        # 设置图像识别的中心偏移（修正识别中心点）。
        self.sendOrder(id, order.setCenterOffset, "<2h", offset[0], offset[1])  # 发送中心偏移
        self.showText(id, "setCenterOffset(" + str(id) + ",[" + str(offset[0]) + "," + str(offset[1]) + "])")
        self.autoDelay(id)

    def setLocation(self, id, loc):
        # 设置飞机当前坐标（定位）。
        self.previewLoc[id][0] = loc[0]     # 更新预览 x
        self.previewLoc[id][1] = loc[1]     # 更新预览 y
        self.flyPreviewUpdata(id)           # 刷新预览
        self.sendOrder(id, order.setLocation, "<2h", loc[0], loc[1])  # 发送坐标命令
        self.showText(id, "setLocation(" + str(id) + ",[" + str(loc[0]) + "," + str(loc[1]) + "])")
        self.autoDelay(id)

    def setTagDistance(self, id, distance):
        # 设置识别标签的距离参数。
        self.sendOrder(id, order.setTagDistance, "<h", distance)  # 发送标签距离命令
        self.showText(id, "setTagDistance(" + str(id) + "," + str(distance) + ")")
        self.autoDelay(id)

    def xySpeed(self, id, speed):
        # 设置水平移动速度。
        speed = int(speed + 0.5)          # 四舍五入成整数
        self.horSpeed[id] = speed         # 保存水平速度
        self.sendOrder(id, order.xySpeed, "<h", speed)  # 发送速度命令
        self.showText(id, "xySpeed(" + str(id) + "," + str(speed) + ")")
        self.autoDelay(id)

    def zSpeed(self, id, speed):
        # 设置垂直（上下）移动速度。
        speed = int(speed + 0.5)          # 四舍五入成整数
        self.verSpeed[id] = speed         # 保存垂直速度
        self.sendOrder(id, order.zSpeed, "<h", speed)  # 发送速度命令
        self.showText(id, "zSpeed(" + str(id) + "," + str(speed) + ")")
        self.autoDelay(id)

    def move(self, id, mode, loc):
        # 按坐标移动：mode 表示移动方式，loc 是 [x,y,z] 目标坐标。
        loc[0] = int(loc[0] + 0.5)   # x 四舍五入
        loc[1] = int(loc[1] + 0.5)   # y 四舍五入
        loc[2] = int(loc[2] + 0.5)   # z 四舍五入
        # 发送坐标移动命令
        self.sendOrder(id, order.move, "<B3h", mode, int(loc[0]), int(loc[1]), int(loc[2]))
        self.showText(id, "move(" + str(id) + "," + str(mode) + ",[" + str(loc[0]) + "," + str(loc[1]) + "," + str(loc[2]) + "])")
        self.moveDelay(id)           # 等待移动到位

    def moveCtrl(self, id, dir, distance):
        # 手动方向移动：dir 是方向，distance 是距离。
        if dir > 6:                                # 方向编号大于 6 的是斜向移动
            distance = int(distance * 0.7071 + 0.5)  # 斜向移动时，距离要乘 0.7071 换算
        else:                                      # 普通上下左右前方向
            distance = int(distance + 0.5)         # 直接四舍五入
        self.flyPreviewMove(id, dir, distance)     # 更新预览位置
        self.sendOrder(id, order.moveCtrl, "<Bh", dir, distance)  # 发送移动命令
        self.showText(id, "moveCtrl(" + str(id) + "," + str(dir) + "," + str(distance) + ")")
        self.moveDelay(id)

    def moveSearchDot(self, id, dir, distance):
        # 朝某个方向移动，同时寻找“点”。
        if dir > 6:                                # 斜向
            distance = int(distance * 0.7071 + 0.5)
        else:                                      # 正方向
            distance = int(distance + 0.5)
        self.flyPreviewMove(id, dir, distance)     # 更新预览
        self.sendOrder(id, order.moveSearchDot, "<Bh", dir, distance)  # 发送找点移动命令
        self.showText(id, "moveSearchDot(" + str(id) + "," + str(dir) + "," + str(distance) + ")")
        self.moveDelay(id)

    def moveSearchBlob(self, id, dir, distance, blob):
        # 朝某个方向移动，同时寻找指定“色块”。blob 是色块参数。
        if dir > 6:                                # 斜向
            distance = int(distance * 0.7071 + 0.5)
        else:                                      # 正方向
            distance = int(distance + 0.5)
        self.flyPreviewMove(id, dir, distance)     # 更新预览
        # 发送找色块移动命令，附带 6 个色块参数
        self.sendOrder(id, order.moveSearchBlob, "<Bh6b", dir, distance, blob[0], blob[1], blob[2], blob[3], blob[4], blob[5])
        self.showText(id, "moveSearchDot(" + str(id) + "," + str(dir) + "," + str(distance) + ",[" + str(blob[0]) + "," + str(blob[1]) + "," + str(blob[2]) + "," + str(blob[3]) + "," + str(blob[4]) + "," + str(blob[5]) + "])")
        self.moveDelay(id)

    def moveSearchTag(self, id, dir, distance, tagID):
        # 朝某个方向移动，同时寻找指定编号的“标签”。
        if dir > 6:                                # 斜向
            distance = int(distance * 0.7071 + 0.5)
        else:                                      # 正方向
            distance = int(distance + 0.5)
        self.flyPreviewMove(id, dir, distance)     # 更新预览
        self.sendOrder(id, order.moveSearchTag, "<Bhh", dir, distance, tagID)  # 发送找标签移动命令
        self.showText(id, "moveSearchTag(" + str(id) + "," + str(dir) + "," + str(distance) + "," + str(tagID) + ")")
        self.moveDelay(id)

    def moveFollowTag(self, id, dir, distance, tagID):
        # 朝某个方向移动，同时“跟随”指定标签。
        if dir > 6:                                # 斜向
            distance = int(distance * 0.7071 + 0.5)
        else:                                      # 正方向
            distance = int(distance + 0.5)
        self.flyPreviewMove(id, dir, distance)     # 更新预览
        self.sendOrder(id, order.moveFollowTag, "<Bhh", dir, distance, tagID)  # 发送跟随标签命令
        self.showText(id, "moveFollowTag(" + str(id) + "," + str(dir) + "," + str(distance) + "," + str(tagID) + ")")
        self.moveDelay(id)

    def goTo(self, id, loc):
        # 直接飞到指定坐标 [x,y,z]。
        loc[0] = int(loc[0] + 0.5)   # x 四舍五入
        loc[1] = int(loc[1] + 0.5)   # y 四舍五入
        loc[2] = int(loc[2] + 0.5)   # z 四舍五入
        self.previewLoc[id][0] = loc[0]  # 更新预览 x
        self.previewLoc[id][1] = loc[1]  # 更新预览 y
        self.previewLoc[id][2] = loc[2]  # 更新预览 z
        self.flyPreviewUpdata(id)        # 刷新预览
        self.sendOrder(id, order.goTo, "<3h", loc[0], loc[1], loc[2])  # 发送飞往坐标命令
        self.showText(id, "goTo(" + str(id) + ",[" + str(loc[0]) + "," + str(loc[1]) + "," + str(loc[2]) + "])")
        self.moveDelay(id)

    def goToTag(self, id, tagID, high):
        # 飞到指定标签的上方。
        high = int(high + 0.5)               # 高度四舍五入
        self.sendOrder(id, order.goToTag, "<2h", tagID, high)  # 发送飞到标签命令
        self.showText(id, "goToTag(" + str(id) + "," + str(tagID) + "," + str(high) + ")")
        self.moveDelay(id)

    def rotation(self, id, angle):
        # 让飞机原地旋转指定角度。
        angle = int(angle + 0.5)             # 角度四舍五入
        self.sendOrder(id, order.rotation, "<h", angle)  # 发送旋转命令
        self.showText(id, "rotation(" + str(id) + "," + str(angle) + ")")
        self.autoDelay(id, 1 + abs(angle) / 30)   # 根据角度大小估算等待时间

    def flyDir(self, id, angle):
        # 设定飞行方向角。
        self.sendOrder(id, order.flyDir, "<h", angle)  # 发送方向角命令
        self.showText(id, "flyDir(" + str(id) + "," + str(angle) + ")")
        self.autoDelay(id, 3)

    def flyHigh(self, id, high):
        # 飞到指定高度。
        high = int(high + 0.5)               # 高度四舍五入
        self.sendOrder(id, order.flyHigh, "<h", high)  # 发送高度命令
        self.showText(id, "flyHigh(" + str(id) + "," + str(high) + ")")
        self.moveDelay(id)

    def tofSwitch(self, id, mode):
        # 开关红外测距传感器。
        self.sendOrder(id, order.tofSwitch, "<B", mode)  # 发送测距开关命令
        self.showText(id, "tofSwitch(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def flipCtrl(self, id, dir, cir):
        # 控制翻滚动作：dir 是方向，cir 是圈数。
        self.sendOrder(id, order.flipCtrl, "<2B", dir, cir)  # 发送翻滚命令
        self.showText(id, "flipCtrl(" + str(id) + "," + str(dir) + "," + str(cir) + ")")
        self.autoDelay(id, 2)

    def ledCtrl(self, id, mode, color):
        # 控制灯光：mode 是模式，color 是 [R,G,B] 颜色。
        color[0] = int(color[0] + 0.5)   # R 四舍五入
        color[1] = int(color[1] + 0.5)   # G 四舍五入
        color[2] = int(color[2] + 0.5)   # B 四舍五入
        self.previewRgb[id][0] = color[0]  # 更新预览颜色 R
        self.previewRgb[id][1] = color[1]  # 更新预览颜色 G
        self.previewRgb[id][2] = color[2]  # 更新预览颜色 B
        self.flyPreviewUpdata(id)          # 刷新预览
        self.sendOrder(id, order.ledCtrl, "<4B", mode, color[0], color[1], color[2])  # 发送灯光命令
        self.showText(id, "ledCtrl(" + str(id) + "," + str(mode) + ",[" + str(color[0]) + "," + str(color[1]) + "," + str(color[2]) + "])")
        self.autoDelay(id)

    def closeLed(self, id):
        # 关闭灯光。
        self.previewRgb[id][0] = 0   # 预览 R 归零
        self.previewRgb[id][1] = 0   # 预览 G 归零
        self.previewRgb[id][2] = 0   # 预览 B 归零
        self.flyPreviewUpdata(id)    # 刷新预览
        self.sendOrder(id, order.ledCtrl, "<4B", 0, 0, 0, 0)  # 发送关闭灯光命令
        self.showText(id, str(id) + "号关闭灯光")
        self.autoDelay(id)

    def mvCheckMode(self, id, mode):
        # 设置视觉识别检查模式。
        self.sendOrder(id, order.mvMode, "<B6bh", mode, 0, 0, 0, 0, 0, 0, 0)  # 发送识别模式命令
        self.showText(id, "mvCheckMode(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def mvCheckTag(self, id, tagID):
        # 检查指定编号的标签。
        self.sendOrder(id, order.mvMode, "<B6bh", 6, 0, 0, 0, 0, 0, 0, tagID)  # 发送检查标签命令
        self.showText(id, "mvCheckTag(" + str(id) + "," + str(tagID) + ")")
        self.autoDelay(id)

    def mvCheckBlob(self, id, type, blob):
        # 检查指定色块。
        self.sendOrder(id, order.mvMode, "<B6bh", type, blob[0], blob[1], blob[2], blob[3], blob[4], blob[5], 0)  # 发送检查色块命令
        self.showText(id, "mvCheckBlob(" + str(id) + ",[" + str(blob[0]) + "," + str(blob[1]) + "," + str(blob[2]) + "," + str(blob[3]) + "," + str(blob[4]) + "," + str(blob[5]) + "])")
        self.autoDelay(id)

    def mvBlobArea(self, id, minArea, maxArea):
        # 设置色块识别时允许的最小和最大面积。
        self.sendOrder(id, order.blobArea, "<2L", minArea, maxArea)  # 发送色块面积命令
        self.showText(id, "mvBlobArea(" + str(id) + "," + str(minArea) + "," + str(maxArea) + ")")
        self.autoDelay(id)

    def shootCtrl(self, id, mode):
        # 控制射击装置。
        self.sendOrder(id, order.shootCtrl, "<B", mode)  # 发送射击命令
        self.showText(id, "shootCtrl(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def magnetCtrl(self, id, mode):
        # 控制磁吸装置。
        self.sendOrder(id, order.magnetCtrl, "<B", mode)  # 发送磁吸命令
        self.showText(id, "magnetCtrl(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def servoCtrl(self, id, angle):
        # 控制舵机转到指定角度。
        angle = int(angle + 0.5)               # 角度四舍五入
        self.sendOrder(id, order.servoCtrl, "<B", angle)  # 发送舵机命令
        self.showText(id, "servoCtrl(" + str(id) + "," + str(angle) + ")")
        self.autoDelay(id, 0.5)

    def lockDir(self, id, mode):
        # 锁定或解锁飞行方向。
        self.sendOrder(id, order.lockDir, "<B", mode)  # 发送锁定方向命令
        self.showText(id, "lockDir(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def roleCtrl(self, id, string):
        # 发送一段“角色任务”文字（比如让飞机执行某个角色动作）。
        strBuf = string.encode("utf-8")   # 把文字转成 UTF-8 字节
        strLen = len(strBuf)              # 计算字节长度
        if strLen < 11:                   # 文字长度小于 11 字节才允许发送
            self.sendOrderPack(id, order.roleCtrl, strBuf)      # 发送角色命令
            self.showText(id, "roleCtrl(" + str(id) + ',"' + string + '")')
            self.autoDelay(id)
        else:                             # 太长就提示发送失败
            self.showText(id, "发送失败：字符超过10字节")

    def cameraDeg(self, id, deg):
        # 调整摄像头角度。
        self.sendOrder(id, order.cameraDeg, "<h", deg)  # 发送摄像头角度命令
        self.showText(id, "cameraDeg(" + str(id) + "," + str(deg) + ")")
        self.autoDelay(id, 0.5)

    def photographMode(self, id, mode):
        # 设置拍照模式。
        self.port.flyData.photo.id = id                   # 记录是哪架飞机在拍照
        self.sendOrder(id, order.photographMode, "<B", mode)  # 发送拍照模式命令
        self.showText(id, "photographMode(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)
        self.port.flyData.photo.isOk = False              # 先把“拍照完成”标志清空

    def irSend(self, id, mode, data):
        # 发送红外信号。
        self.sendOrder(id, order.irSend, "<5B", mode, data[0], data[1], data[2], data[3])  # 发送红外命令
        self.showText(id, "irSend(" + str(id) + "," + str(mode) + ",[" + hex(data[0]) + "," + hex(data[1]) + "," + hex(data[2]) + "," + hex(data[3]) + "])")
        self.autoDelay(id)

    def showStr(self, id, x, y, string, scal):
        # 在飞机屏幕上显示一段文字。
        buf = pack("<3B", x, y, scal)      # 把坐标和字号打包
        strBuf = string.encode("utf-8")    # 把文字转成 UTF-8 字节
        strLen = len(strBuf)               # 计算字节长度
        if strLen <= 7:                    # 长度不超过 7 字节才显示
            self.sendOrderPack(id, order.showStr, buf + strBuf)  # 发送显示命令
            self.showText(id, "showStr(" + str(id) + "," + str(x) + "," + str(y) + ',"' + string + '",' + str(scal) + ")")
            self.autoDelay(id)
        else:                              # 太长就提示失败
            self.showText(id, "显示失败：字符超过7字节")

    def showCtrl(self, id, mode):
        # 控制显示状态。
        self.sendOrder(id, order.showCtrl, "<B", mode)  # 发送显示控制命令
        self.showText(id, "showCtrl(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def horAround(self, id, type, cx, cy, angle, speed):
        # 水平绕圈飞行。
        self.sendOrder(id, order.horAround, "<B3hB", type, cx, cy, angle, speed)  # 发送水平绕圈命令
        self.showText(id, "horAround(" + str(id) + "," + str(type) + "," + str(cx) + "," + str(cy) + "," + str(angle) + "," + str(speed) + ")")
        self.moveDelay(id)

    def verAround(self, id, type, dist, high, angle, speed):
        # 垂直绕圈飞行。
        self.sendOrder(id, order.verAround, "<B3hB", type, dist, high, angle, speed)  # 发送垂直绕圈命令
        self.showText(id, "verAround(" + str(id) + "," + str(type) + "," + str(dist) + "," + str(high) + "," + str(angle) + "," + str(speed) + ")")
        self.moveDelay(id)

    def sinAround(self, id, dir0, dir1, start, stop, dLen, dHigh, speed):
        # 按正弦轨迹绕飞。
        self.sendOrder(id, order.sinAround, "<2B3h2B", dir0, dir1, start, stop, dLen, dHigh, speed)  # 发送正弦绕飞命令
        self.showText(id, "sinAround(" + str(id) + "," + str(dir0) + "," + str(dir1) + "," + str(start) + "," + str(stop) + "," + str(dLen) + "," + str(dHigh) + "," + str(speed) + ")")
        self.moveDelay(id)

    def photoOk(self):
        # 返回“拍照是否完成”。
        return self.port.flyData.photo.isOk

    def isPhotoOk(self):
        # 同 photoOk：判断拍照是否完成。
        return self.photoOk()

    def getKeyPress(self, id):
        # 判断指定编号的按键是否被按下。
        if self.port.flyData.keyPressId == id:  # 如果按下的就是这台飞机
            return True                          # 返回“按下了”
        return False                             # 否则返回“没按下”

    def isMvCheck(self, id, mode):
        # 判断某个视觉识别模式是否已经识别成功。
        # flag 里的每一位代表一种识别状态，用位运算取出对应位。
        return (self.port.flyData.flySensor[id].mv.flag & (1 << mode)) != 0

    def isMvCheckLine(self, id, dir):
        # 判断某个方向的循线是否识别成功。
        return (self.port.flyData.flySensor[id].mv.flag & (1 << dir)) != 0

    def getObsDistance(self, id, dir):
        # 获取某个方向上的障碍物距离，保留 1 位小数。
        return round(self.port.flyData.flySensor[id].obs_dist[dir], 1)

    def getFlySensor(self, id, type):
        # 通用的传感器数据读取函数，type 决定读取哪一种数据。
        if type == "tagID":   # 读取识别到的标签编号
            return self.port.flyData.flySensor[id].mv.tagId
        if type == "qrCode":  # 读取二维码内容
            return self.port.flyData.flySensor[id].qrCode
        if type == "brCode":  # 读取条形码内容
            return self.port.flyData.flySensor[id].brCode
        if type == "rol":     # 读取横滚角
            return round(self.port.flyData.flySensor[id].imu[0], 1)
        if type == "pit":     # 读取俯仰角
            return round(self.port.flyData.flySensor[id].imu[1], 1)
        if type == "yaw":     # 读取偏航角
            return round(self.port.flyData.flySensor[id].imu[2], 1)
        if type == "loc_x":   # 读取当前位置 x
            return round(self.port.flyData.flySensor[id].loc[0], 1)
        if type == "loc_y":   # 读取当前位置 y
            return round(self.port.flyData.flySensor[id].loc[1], 1)
        if type == "loc_z":   # 读取当前位置 z（高度）
            return round(self.port.flyData.flySensor[id].loc[2], 1)
        if type == "err_x":   # 读取 x 方向误差
            return round(self.port.flyData.flySensor[id].locErr[0], 1)
        if type == "err_y":   # 读取 y 方向误差
            return round(self.port.flyData.flySensor[id].locErr[1], 1)
        if type == "err_z":   # 读取 z 方向误差
            return round(self.port.flyData.flySensor[id].locErr[2], 1)
        if type == "vol":     # 读取电池电压
            return round(self.port.flyData.flySensor[id].vol, 2)

    def getBlobResult(self, id, type):
        # 读取色块识别的结果，type 决定读取哪种信息。
        if type == "s":   # 色块面积
            return self.port.flyData.flySensor[id].mv.blob_s
        if type == "w":   # 色块宽度
            return self.port.flyData.flySensor[id].mv.blob_w
        if type == "h":   # 色块高度
            return self.port.flyData.flySensor[id].mv.blob_h
        if type == "n":   # 色块数量
            return self.port.flyData.flySensor[id].mv.blob_n

    def getRoleNews(self, id, type):
        # 读取“角色任务”相关消息。
        if type == "details":  # 读取消息内容
            return self.port.flyData.flySensor[id].news
        if type == "id":       # 读取消息编号
            return self.port.flyData.flySensor[id].newsCount

    def clearRoleNews(self, id):
        # 清空角色任务消息。
        self.port.flyData.flySensor[id].news = ""

    def clearConsole(self):
        # 清空控制台屏幕。
        try:                     # 尝试执行
            os.system("cls")     # 调用系统命令清屏
            self.sleep(0.1)      # 稍等 0.1 秒
        except:                  # 如果失败就忽略，不报错
            pass

    def switchCtrl(self, id, mode):
        # 控制某个开关。
        self.sendOrder(id, order.switchCtrl, "<B", mode)  # 发送开关命令
        self.showText(id, "switchCtrl(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)

    def getScaleWeight(self, id):
        # 读取称重传感器的重量。
        return self.port.flyData.flySensor[id].scale_weight

    def getShootResult(self, id, type):
        # 读取射击结果，type 决定读取哪种信息。
        if type == "number":  # 命中数量
            return self.port.flyData.flySensor[id].laserTarget_count
        if type == "result":  # 射击结果
            return self.port.flyData.flySensor[id].laserTarget_result
        if type == "x":       # 命中点 x 坐标
            return self.port.flyData.flySensor[id].laserTarget_x
        if type == "y":       # 命中点 y 坐标
            return self.port.flyData.flySensor[id].laserTarget_y

    def serServoCtrl(self, id, index, value, time):
        # 控制串口舵机。
        self.sendOrder(id, order.serServoCtrl, "<B2h", index, value, time)  # 发送串口舵机命令
        self.showText(id, "serServoCtrl(" + str(id) + "," + str(index) + "," + str(value) + "," + str(time) + ")")
        self.autoDelay(id, time * 0.001)   # 根据动作时间估算等待时间

    def robotArmCtrl(self, id, index, time):
        # 控制机械臂某个关节动作。
        self.sendOrder(id, order.robotArmCtrl, "<Bh", index, time)  # 发送机械臂命令
        self.showText(id, "robotArmCtrl(" + str(id) + "," + str(index) + "," + str(time) + ")")
        self.autoDelay(id, time * 0.001)   # 根据动作时间估算等待时间

    def robotArmCover(self, id, index):
        # 控制机械臂抓取/覆盖动作。
        self.sendOrder(id, order.robotArmCover, "<B", index)  # 发送机械臂抓取命令
        self.showText(id, "robotArmCover(" + str(id) + "," + str(index) + ")")
        self.autoDelay(id)

    def robotArmRecord(self, id, mode):
        # 录制或播放机械臂动作。
        self.sendOrder(id, order.robotArmRecord, "<B", mode)  # 发送机械臂录制命令
        self.showText(id, "robotArmRecord(" + str(id) + "," + str(mode) + ")")
        self.autoDelay(id)
