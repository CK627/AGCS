# 无人机电脑端网页（视频 + 无人机数据监控）

这是无人机在地面站电脑上的网页，把监控放在一个页面：

1. **图传画面**：EWRF 5.8G 图传接收机插本机 USB（表现为一个 USB 摄像头）
   → 后台线程独占采集 → 网页 MJPEG（接收方式同
   [外接图传录制程序](../../外接图传录制程序)，地面站中枢也从这里拉流）
2. **无人机数据**：优先用 SDK（串口 helloFly）读位置/姿态/电压；
   MAVLink 仅作备用数据源
3. **自动任务**：由 OpenFly 图形化编程（编队软件）处理，本页不提供任务上传

## 技术栈（已确定）

| 层 | 选型 | 为什么 |
|----|------|--------|
| 后端 | Python（Flask + OpenFly SDK） | 串口 SDK 读位置/姿态/电压，和无人机 Python 工具链一致 |
| 前端 | 纯 HTML + CSS + 原生 JS | **不引入 React/Vue 等框架**，页面是原网页，加载快、无构建步骤 |
| 数据格式 | JSON | 后端接口全部用 JSON，前端 `fetch` 轮询即可，简单可靠 |

前端只做展示和发请求，不做复杂逻辑；无人机信息获取由 Python 后端负责。

## 启动自检（摄像头 / 链路）

服务启动时会先做一次自检，把结果打印在控制台，并持续在后台维护：

1. **无人机串口链路**：识别 OpenFly 遥控器的虚拟串口（规则移植自
   `外接图传录制程序/lib/mySerial.py`，只枚举不占用串口）。
2. **图传摄像头设备**：能不能打开 USB 摄像头（EWRF 接收机）。
3. **图传信号**：连读几帧画面，全黑、纯蓝屏/纯色屏（EWRF 无信号的典型画面）、
   静帧都视为“飞机摄像头没开/无信号”。

状态随时可查：`GET /api/camera/status`，网页“图传画面”区域也会显示
对应徽标（摄像头已开启 / 无信号 / 接收机未检测到）。

接收机后插、飞机摄像头后开机都不用重启服务：后台每 2 秒复检一次信号，
打不开设备则每 3 秒自动重试。

## 无人机数据（SDK 优先）

无人机数据优先用随 backend 自带的 OpenFly SDK（`lib/helloFly.py`）通过串口读取
（位置 X/Y/Z 厘米、姿态、电压），用法同
`外接图传录制程序/capture_external_camera.py`；pymavlink/MAVLink 作为备用数据源，
没装 pymavlink 或收不到心跳时不影响页面显示。

`/api/telemetry` 返回 `source: sdk | mavlink`，SDK 模式下额外带
`loc_x/loc_y/loc_z`（厘米）和 `sdk.serial`。

## 运行

```bash
cd Dron/Dashboard
python -m pip install -r requirements.txt
cd backend && python app.py
```

浏览器打开 http://127.0.0.1:20000（让队友访问改成 `--host 0.0.0.0`）。

服务用 `waitress`（生产级 WSGI 服务器）启动，不会出现
“This is a development server” 的开发服务器警告；未安装 `waitress` 时会
自动退回 Flask 自带开发服务器。

## 先看配置

**临时配置（图传摄像头序号 / 视频开关 / 端口）统一改本仪表盘的
[data/config.yaml](../data/config.yaml)，保存即热重载生效，无需重启**
（端口除外，改端口需重启）。

```yaml
drone:
  camera: 0     # EWRF 接收机的 USB 摄像头序号
  video: true   # 画面开关
```

摄像头序号不确定时，在本机跑
`python 外接图传录制程序/capture_external_camera.py --list` 查看（USB Video 是几号就填几号）。
接收机没插时页面会推一张"receiver not found"状态卡，插上后自动出图，不用重启。

`config.py` 只放默认值和硬件绑定项（`MAVLINK_CONN` 等）。

## 三个页面能力对应哪些接口

| 页面功能 | 后端接口 | 比喻 |
|----------|----------|------|
| 视频画面 | `/video_feed` | 把摄像头的录像转成网页能看的格式 |
| 摄像头状态 | `/api/camera/status` | 启动自检结果：设备/信号/分辨率/串口链路 |
| 实时监控 | `/api/telemetry`、`/api/history` | 从公告栏取最新纸条，再画曲线 |

## 新手建议

先读 [Code/黑话对照表.md](../Code/黑话对照表.md)，再用
[Code/闯关任务.md](../Code/闯关任务.md) 的离线数据把监控跑通。

## lib（无人机 SDK）

`lib/`（在 backend 目录内）是从 `外接图传录制程序/lib` 移植过来的
OpenFly 无人机 Python SDK
（helloFly / driver / flyData / mySerial / tcpClient / helloAi），
用途和说明见 [lib/README.md](lib/README.md)。

backend 目录自包含（代码 + lib + requirements.txt），单独拷走、装好
`pip install -r requirements.txt` 后 `python app.py` 即可运行。

## 局域网广播（让地面站自动找到本机）

服务启动后每 2 秒向局域网广播一次自己的地址（UDP 20003），
地面站中枢监听同一端口即可自动发现并连接，不用手动填 IP；
也可以直接在本仪表盘 `data/config.yaml` 的 `drone.url` 写死地址兜底。
