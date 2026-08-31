# ============================================================
# 文件名：helloAi.py
# 作用：和电脑上的 OpenFly 软件通信，调用 AI 功能，
#       比如语音识别、文字识别、语音播报、物体识别等。
# 说明：以下每一行代码都加了中文注释。
# ============================================================

# 引入 TCP 客户端和线程模块
import tcpClient, _thread
# 引入 sleep（延时）和 time（取当前时间）
from time import sleep, time


class init:
    # AI 功能的主类。

    def __init__(self):
        # 初始化：连接 OpenFly 软件，并启动后台线程。
        self.cmdString = ""                         # 要发给 AI 的命令，初始为空
        self.reqDict = {}                           # 保存 AI 返回结果的字典
        self.ssi = 1                               # 信号强度状态，初始为 1
        self.ssiCnt = 0                            # 信号计数
        self.timer = time()                        # 记录当前时间
        self.ai = tcpClient.init(port=8004, timeOut=0.02)  # 连接 8004 端口的 AI 服务
        _thread.start_new_thread(self.run, ())     # 启动后台线程，持续收发 AI 数据
        self.runFunction("初始化")                  # 让 AI 先执行“初始化”
        print("HelloAi:2025-04-15")                # 打印 AI 库版本
        print("准备就绪，开始运行\n")                # 提示准备完成

    def run(self):
        # 后台线程：不断地轮询 AI 是否返回结果。
        while True:                              # 无限循环
            string = self.cmdString              # 取出当前要执行的命令
            self.cmdString = "GET /poll HTTP/1.1"  # 把下次命令设为“轮询”（询问结果）
            self.request(string)                 # 把当前命令发给 AI
            if time() - self.timer >= 1:         # 每 1 秒检查一次连接状态
                self.timer = time()              # 更新检查时间
                self.ssi = self.ssiCnt           # 保存这一秒收到的数据次数
                self.ssiCnt = 0                  # 计数清零
                if self.ssi == 0:                # 如果一次数据都没收到
                    print("请打开OpenFly软件！")   # 提示用户打开软件
            sleep(0.02)                          # 每次循环休息 0.02 秒

    def request(self, cmdString):
        # 把命令发给 AI，并把返回结果解析进字典里。
        request = self.ai.request(cmdString.encode("utf-8"))  # 发送命令并拿到原始回复
        lines = request.decode("utf-8").splitlines()          # 把回复按行拆开
        lens = len(lines)                                     # 统计有多少行
        if lens:                                              # 如果有回复内容
            self.ssiCnt = self.ssiCnt + 1                     # 收到数据的次数加 1
            for i in range(lens):                             # 逐行处理
                words = lines[i].split()                      # 把一行按空格拆成词
                self.reqDict[words[0]] = ""                   # 以第一个词作为键，先清空值
                for j in range(1, len(words)):                # 把后面所有词重新拼成值
                    self.reqDict[words[0]] = self.reqDict[words[0]] + ("" if j == 1 else " ") + words[j]

    def getKeyValue(self, key):
        # 根据键名从字典里取一个值，取不到就返回空字符串。
        sleep(0.1)                       # 先等 0.1 秒，等结果返回
        try:
            value = self.reqDict[key]    # 尝试取值
        except:                          # 如果键不存在
            value = ""                   # 返回空字符串
        else:
            return value                 # 正常情况返回取到的值

    def runTTS(self, text):
        # 让 AI 用语音把文字念出来（语音播报）。
        self.cmdString = "GET /TTS/" + text   # 组装语音播报命令
        print("语音播报：" + text)             # 打印要播报的内容
        sleep(0.1)                            # 等 0.1 秒

    def runFunction(self, function):
        # 让 AI 执行一个指定功能（如“文字识别”）。
        self.cmdString = "GET /aiCtrl/" + function  # 组装 AI 功能命令
        sleep(0.5)                                  # 等 0.5 秒
        self.reqDict["aiFinish"] = "false"          # 把“功能完成”标志清成 false

    def isPhotoOk(self):
        # 判断 AI 拍照是否成功。
        if self.getKeyValue("aiPhotoOk") == "true":  # 如果返回 true
            return True                              # 拍照成功
        return False                                 # 否则没成功

    def isComplete(self):
        # 判断 AI 功能是否执行完成。
        if self.getKeyValue("aiFinish") == "true":   # 如果返回 true
            return True                              # 已完成
        return False                                 # 否则未完成

    def result(self, details):
        # 获取 AI 识别结果里的某个字段。
        result = self.getKeyValue("aiResult/" + details)  # 取对应字段的值
        if details == "语音识别":                          # 如果是语音识别结果
            self.reqDict["aiResult/语音识别"] = ""          # 用完就清空，避免下次读旧值
        elif details != "文字内容":                        # 如果不是纯文字内容
            result = self.strToInt(result)                 # 就尝试转成数字
        return result                                      # 返回结果

    def resultObject(self, num, details):
        # 获取“物体识别”结果里的某个字段。
        result = self.getKeyValue("aiResultObject/" + str(num) + "/" + details)  # 取对应字段
        if details != "名称":       # 如果不是物体名称
            result = self.strToInt(result)  # 尝试转成数字
        return result               # 返回结果

    def strToInt(self, string):
        # 把字符串转成整数，失败就返回 0。
        try:                              # 尝试转换
            result = int(float(string))   # 先转小数再取整
        except:                           # 转换失败
            result = 0                    # 返回 0
        else:
            return result                 # 正常情况返回转换结果


if __name__ == "__main__":
    # 直接运行这个文件时，做个小测试。
    test = init()               # 创建一个 AI 对象
    test.runFunction("文字识别")  # 让它执行“文字识别”
