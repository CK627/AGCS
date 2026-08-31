# lib（无人机 SDK）

本目录是从 `外接图传录制程序/lib` 移植过来的 OpenFly 无人机 Python SDK，
文件内容保持一致：

| 文件 | 作用 |
|------|------|
| `helloFly.py` | 无人机控制类（起飞、移动、拍照等全部指令） |
| `driver.py` | 通信驱动：按运行环境选择串口 / TCP，后台接收线程 |
| `flyData.py` | 把飞机回传的二进制数据解析成传感器数值 |
| `mySerial.py` | 在电脑端找到并打开无人机虚拟串口（识别 VID:PID） |
| `tcpClient.py` | 和 OpenFly 编队软件 / AI 服务的 TCP 客户端 |
| `helloAi.py` | 调用 OpenFly 电脑端 AI 功能 |

仪表盘当前只在“启动自检”中使用串口识别规则（见
`../drone_link.py`，只枚举不占用串口）；需要完整控制无人机时，
可把本目录加入 `sys.path` 后 `import helloFly`，用法和
`外接图传录制程序/capture_external_camera.py` 一致。

说明：`mySerial.py` 相比原版只加了一处小改动——错误提示每秒最多打印一次
（原版在串口中途拔掉时会高频刷屏），收发逻辑不变。
