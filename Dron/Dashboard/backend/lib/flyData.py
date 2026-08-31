# ============================================================
# 文件名：flyData.py
# 作用：把飞机发回来的二进制数据“翻译”成我们能看懂的数字，
#       比如电压、位置、姿态角、二维码、标签、障碍物距离等。
# 说明：以下每一行代码都加了中文注释。
# ============================================================

# 引入 unpack，用来把二进制字节解包成数字
from struct import unpack


class mv_t(object):
    # 视觉识别相关数据，用一个小类打包起来。
    flag = 0       # 识别状态标志（每一位表示一种识别是否成功）
    tagId = 0      # 识别到的标签编号
    blob_n = 0     # 识别到的色块数量
    blob_s = 0     # 色块面积
    blob_w = 0     # 色块宽度
    blob_h = 0     # 色块高度


class sensor(object):
    # 一架飞机的所有传感器数据，都在这个类里。
    id = 0                            # 飞机编号
    vol = 0                           # 电池电压
    ssi = 0                           # 信号强度
    state = 0                         # 飞机状态
    obs_dist = [255, 255, 255, 255]   # 四个方向的障碍物距离，初始很大表示“无障碍”
    imu = [0, 0, 0]                   # 姿态角（横滚、俯仰、偏航）
    loc = [0, 0, 0]                   # 当前位置 x,y,z
    locErr = [0, 0, 0]                # 定位误差 x,y,z
    sys_flag = 0                      # 系统状态标志
    laserTarget_count = 0             # 射击命中数量
    laserTarget_result = 0            # 射击结果
    laserTarget_x = 0                 # 命中点 x
    laserTarget_y = 0                 # 命中点 y
    scale_weight = 0                  # 称重传感器读数
    newsCount = 0                     # 消息编号
    newsLen = 0                       # 消息长度
    news = ""                         # 消息内容
    qrCode = ""                       # 二维码内容
    brCode = ""                       # 条形码内容
    mv = mv_t()                       # 视觉识别数据
    orderCount = 65535                # 飞机最后回复的指令序号


class photo_t(object):
    # 拍照相关数据。
    id = 0        # 哪架飞机在拍照
    isOk = False  # 拍照是否完成


class init:
    # 数据解析的主类。

    def __init__(self, flyNum):
        # 初始化：为每架飞机准备好传感器数据对象。
        self.maxNum = flyNum                                      # 保存飞机数量
        self.flySensor = [sensor() for _ in range(self.maxNum)]   # 给每架飞机各建一个 sensor
        self.keyPressId = 255                                     # 按键编号初始为 255（表示没按键）
        self.photo = photo_t()                                    # 新建拍照对象

    def getKey(self, aux4, aux5):
        # 从两组按键位里算出“按下的是哪个键”。
        for i in range(16):           # 先检查前 16 个按键位
            if aux4 & 1 << i:         # 用位运算判断第 i 位是不是 1
                return i              # 是的话，返回这个按键编号
        if aux5 & 255:                # 再检查第 16 个按键位
            return 16
        if aux5 & 65280:              # 再检查第 17 个按键位
            return 17
        return 255                    # 都没按下，返回 255

    def Receive_Anl(self, rx):
        # 根据收到的数据帧类型，解析成对应的传感器数据。
        if self.maxNum == 0:          # 如果一架飞机都没有
            return                    # 直接返回，不做处理
        if rx.date[0] == 1:           # 类型 1：飞机基础状态数据
            # 把二进制数据解包成一组数字
            pack = unpack("<3BHB4B6h3bB", bytearray(rx.date)[1:rx.len])
            id = pack[0]              # 取出飞机编号
            if id < self.maxNum:      # 编号合法才处理
                self.flySensor[id].id = pack[0]               # 保存编号
                self.flySensor[id].vol = pack[1] * 0.1        # 电压换算成实际值（除以 10）
                self.flySensor[id].ssi = pack[2]              # 保存信号强度
                self.flySensor[id].state = pack[3]            # 保存飞机状态
                self.flySensor[id].sys_flag = pack[4]         # 保存系统标志
                self.flySensor[id].obs_dist = [pack[5], pack[6], pack[7], pack[8]]  # 四个方向障碍物距离
                self.flySensor[id].imu = [pack[9] * 0.1, pack[10] * 0.1, pack[11] * 0.1]  # 姿态角除以 10
                self.flySensor[id].loc = [pack[12], pack[13], pack[14]]   # 当前位置
                self.flySensor[id].locErr = [pack[15], pack[16], pack[17]] # 定位误差
                self.flySensor[id].orderCount = pack[18]      # 保存飞机回复的指令序号
                if self.photo.id == id:                       # 如果这台飞机正在拍照
                    # 根据系统标志的第 1 位，判断拍照是否完成
                    self.photo.isOk = True if self.flySensor[id].sys_flag & 1 else False
        elif rx.date[0] == 14:        # 类型 14：视觉识别结果
            pack = unpack("<2BHBL2H", bytearray(rx.date)[1:rx.len])  # 解包
            id = pack[0]              # 飞机编号
            if id < self.maxNum:      # 编号合法才处理
                self.flySensor[id].mv.flag = pack[1]   # 保存识别标志
                self.flySensor[id].mv.tagId = pack[2]  # 保存标签编号
                self.flySensor[id].mv.blob_n = pack[3] # 保存色块数量
                self.flySensor[id].mv.blob_s = pack[4] # 保存色块面积
                self.flySensor[id].mv.blob_w = pack[5] # 保存色块宽度
                self.flySensor[id].mv.blob_h = pack[6] # 保存色块高度
        elif rx.date[0] == 2:         # 类型 2：射击和称重相关数据
            pack = unpack("<4B2h4Bf", bytearray(rx.date)[1:rx.len])  # 解包
            id = pack[0]              # 飞机编号
            if id < self.maxNum:      # 编号合法才处理
                self.flySensor[id].id = pack[0]                  # 保存编号
                self.flySensor[id].ssi = pack[1]                 # 保存信号强度
                self.flySensor[id].laserTarget_count = pack[2]   # 保存命中数量
                self.flySensor[id].laserTarget_result = pack[3]  # 保存射击结果
                self.flySensor[id].laserTarget_x = pack[4]       # 保存命中点 x
                self.flySensor[id].laserTarget_y = pack[5]       # 保存命中点 y
                self.flySensor[id].obs_dist = [pack[6], pack[7], pack[8], pack[9]]  # 障碍物距离
                self.flySensor[id].scale_weight = pack[10]       # 保存称重读数
        else:
            # 其它类型：可能是二维码、条形码、消息、提示等
            if rx.date[0] == 244 or rx.date[0] == 245 or rx.date[0] == 245 or rx.date[0] == 255:
                pack = unpack("<3B", bytearray(rx.date)[1:4])    # 先解出前三个字节
                id = pack[0]                                     # 飞机编号
                if self.flySensor[id].newsCount == pack[1]:      # 如果消息编号和上次一样
                    return                                       # 说明是重复消息，直接忽略
                if id < self.maxNum:                             # 编号合法才处理
                    self.flySensor[id].id = pack[0]              # 保存编号
                    self.flySensor[id].newsCount = pack[1]       # 保存消息编号
                    self.flySensor[id].newsLen = pack[2]         # 保存消息长度
                    try:                                         # 尝试把消息字节解码成文字
                        news = bytearray(rx.date)[4:4 + pack[2]].decode("utf-8")
                    except:                                      # 解码失败
                        news = "UnicodeError"                     # 用错误提示代替
                    if rx.date[0] == 244:                        # 类型 244 是二维码
                        self.flySensor[id].qrCode = news          # 保存二维码内容
                        print(str(id) + "号(二维码)：" + news)     # 打印二维码
                    elif rx.date[0] == 245:                      # 类型 245 是条形码
                        self.flySensor[id].brCode = news          # 保存条形码内容
                        print(str(id) + "号(条形码)：" + news)     # 打印条形码
                    elif rx.date[0] == 246:                      # 类型 246 是消息
                        self.flySensor[id].news = news            # 保存消息内容
                        print(str(id) + "号(消息)：" + news)       # 打印消息
                    else:                                        # 其它类型当作提示
                        print(str(id) + "号(提示)：" + news)       # 打印提示
            if rx.date[0] == 3:       # 类型 3：按键数据
                pack = unpack("<10H2BI", bytearray(rx.date)[1:rx.len])  # 解包
                self.keyPressId = self.getKey(pack[7], pack[8])  # 算出按下的按键编号并保存
