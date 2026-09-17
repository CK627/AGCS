# 地面站仪表盘

显示服务器、无人机、地面机器人、YOLO 模型四类信息。
全部走 HTTP 服务，不依赖图形界面。

## 数据流

```text
服务器：两种方式（data/config.yaml 的 server.mode）
  - local：中枢就部署在服务器上 → 后台线程直接读本机 CPU/内存/磁盘（无需网络）
  - http ：服务器上跑 GroundStation/Server（服务器监控服务）→ 本后端拉它的 /api/system
  → /api/server/status → 浏览器「服务器信息」面板

机器人：树莓派 autonomous_pick.py / CS-video.py（每帧 publish_frame 压缩 JPEG）
  → task_server.py /video.mjpeg（5000 端口）
  → 本后端 /robot_video_feed（代理转发）+ /api/robot/status（状态代理）
  → 浏览器实时显示机器人画面与参数

无人机：优先拉无人机电脑端仪表盘 /api/telemetry（支持编队多机 drones 数组）
  → /api/drone/status（逐台完整遥测）+ /drone_video_feed?cam=0/1（两路图传）
  → /api/drone/history（代理上游曲线：姿态/高度）
  → 兜底：数传 MAVLink → pymavlink 后台线程

YOLO 模型：
  - local（默认）：本后端 backend/yolo/ 的模型本地检测（拉无人机图传画框）
  - remote：拉独立 YOLOModel/Dashboard（20003）的 /api/detections + /video_feed（已标注）
  → /api/model/status + /video.mjpeg
```

## 目录

- `backend/`：Flask 后端（本目录）
- `frontend/`：网页前端
- `scripts/`：三平台快速启动脚本

## 启动

先在 `GroundStation/Dashboard` 目录安装依赖：

```bash
python -m pip install -r backend/requirements.txt
```

再启动后端：

```bash
cd backend
python app.py
```

浏览器打开 `http://localhost:20000`（或局域网内 `http://地面站IP:20000`）。

也可以直接用快速启动脚本：

- macOS：`bash GroundStation/Dashboard/scripts/macOS/start.sh`
- Linux：`bash GroundStation/Dashboard/scripts/Linux/start.sh`
- Windows：`GroundStation\Dashboard\scripts\Windows\start.bat`

脚本支持 `start / stop / restart / status / install / update / uninstall / help`
等命令，例如 `bash GroundStation/Dashboard/scripts/macOS/start.sh start`。

服务优先用 waitress 启动；未安装 waitress 时自动退回 Flask 开发服务器。

依赖：

```bat
python -m pip install flask waitress requests pymavlink
```

（视频预览需要 opencv-python；YOLO 检测已移到 YOLO 模型仪表盘 YOLOModel/Dashboard，
本后端只拉取检测结果，不再装 ultralytics；无人机数据由 pymavlink 直接读 MAVLink，
**不需要 ROS/MAVROS**。）

## 配置

**临时配置（机器人 IP / 无人机地址 / 视频开关 / 端口）统一改本仪表盘的
[data/config.yaml](../data/config.yaml)，保存即热重载生效，无需重启**
（端口除外，改端口需重启对应仪表盘）。

`config.py` 只放默认值和硬件绑定项（无人机数传/图传地址、YOLO 仪表盘地址）：

| 配置项 | 说明 |
|--------|------|
| `DRONE_MAVLINK` | pymavlink 数传连接串（默认 `udpin:0.0.0.0:14550`） |
| `SERVER_MODE` | 服务器信息采集方式：`local` 读本机 / `http` 拉 `server.url` / `auto` 自动判断（走 data/config.yaml `server.mode`） |
| `SERVER_URL` | 服务器监控服务地址（走 data/config.yaml `server.url`，http 模式用） |
| `DRONE_URL` | 无人机电脑端仪表盘地址（走本仪表盘 data/config.yaml `drone.url`；中枢从这里拉图传画面，不在同一台电脑时填那台电脑的 IP） |
| `YOLO_URL` | YOLO 模型仪表盘地址（走本仪表盘 data/config.yaml `yolo.url`；中枢从这里拉检测数据与标注画面） |
| `DASHBOARD_HOST` | 监听地址（默认 `0.0.0.0`；端口走本仪表盘 data/config.yaml `dashboard.hub_port`） |

## 接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 仪表盘页面 |
| `/api/server/status` | GET | 服务器信息（在线状态/延迟/CPU/内存/磁盘/运行时间） |
| `/api/drone/status` | GET | 无人机状态（`drones` 数组，逐台完整遥测：数据源/姿态/位置/定位误差/电压/避障/按键/计时器/消息） |
| `/api/drone/history` | GET | 无人机曲线（代理上游 `/api/history`，每台机的姿态与高度序列，供面板画曲线） |
| `/api/robot/status` | GET | 机器人状态（代理树莓派 `/status`） |
| `/api/model/status` | GET | YOLO 模型信息与检测统计 |
| `/drone_video_feed?cam=N` | GET | 无人机图传 MJPEG（N=0/1 两路，代理 Dron Dashboard） |
| `/robot_video_feed` | GET | 机器人摄像头画面 MJPEG（代理树莓派 `/video.mjpeg`） |
| `/video.mjpeg` | GET | 无人机图传画面 MJPEG（拉 YOLO 仪表盘 /video_feed 的已标注流转发） |
| `/api/progress/flow` | GET | 研发进度流程图数据（每模块每步的 done/active/pending + 模块进度） |
| `/api/checklist` | GET | 任务清单：各模块已勾选（active）的步骤编号 |
| `/api/checklist/toggle` | POST | 勾选/取消某一步（`{module, order, checked}`），立即推进/回退进度 |
| `/api/checklist/reset` | POST | 一键初始化：清空所有步骤、进度归零 |

## 任务清单（右上角「✅ 任务清单」）

页面右上角按钮组从左到右：**✅ 任务清单 → 🧭 进度流程 → ⚙ 配置**。

点「✅ 任务清单」弹出清单：4 个模块分组（各 6 步），每步一个勾选框 + 状态文字。
勾选**直接写回中枢进度**（等价于手动推进该步骤），弹窗里也会显示每模块百分比、
已完成步数、以及总进度条。

- **勾选第 k 步** = 推进到第 k 步（其后步骤自动清空；前面步骤由进度推导为已完成）
- **取消第 k 步** = 退回第 k 步之前
- **↺ 一键初始化** = 所有步骤清空、进度归零（等价于 `POST /api/checklist/reset`）

弹窗打开期间每 2 秒同步一次，所以**自动触发/计时完成**的步骤也会自动打勾。

## 无人机面板

页面右下「📡 无人机」是**两台左右并列**，每台一个子框，内容与无人机电脑端仪表盘（`Dron/Dashboard`）一致：

```
子框 = 标题（无人机 N + 连接状态 + 该台画面全屏）
     → 画面（/drone_video_feed?cam=N，即该台的图传）
     → 9 张数据卡片（数据源 / 姿态 r·p·y / 位置 x·y·z / 定位误差 / 电池 /
                    避障 前·后·左·右 / 遥控器按键 / 计时器 / 消息）
     → 2 条曲线（姿态曲线、高度曲线）
```

- **与其它面板的区别**：服务器 / YOLO / 机器人面板用顶部 chip 小标签展示信息，
  无人机面板改用**卡片**（与无人机仪表盘同款版式），一眼能对照到仪表盘上的同一张卡。
- **单位与仪表盘保持一致**：位置/避障/定位误差是厘米、姿态是度、电压是伏，
  中枢不做单位换算（只在 `pose` 字段里另附一份米制换算值给其它调用方）。
- **两台数据分开**：靠上游编队 `id`（`drones[0]` / `drones[1]`），两台各读各的；
  「计时器」是 SDK 的全局秒表，两台相同属正常。
- 面板全屏（⛶）会放大画面与曲线；每台子框自己的⛶只放大该台画面。
- 无人机电脑端不可达或没数据时，面板显示「等待无人机数据…」并附上具体原因
  （取自 `/api/drone/status` 的 `message`），不会一片空白。

## 服务器信息面板

页面左上「🖧 服务器信息」显示服务器在线状态、延迟、CPU / 内存 / 磁盘占用与运行时间，
采集方式由 `data/config.yaml` 的 `server.mode` 决定：

| mode | 适用场景 | 做法 |
|------|----------|------|
| `local` | **中枢就部署在服务器这台机器上**（最省事） | 后台线程直接读本机，不需要任何额外服务 |
| `http` | 中枢在别的电脑，服务器上可以装东西 | 服务器上跑 `GroundStation/Server/server_monitor.py`，中枢拉它的 `/api/system` |
| `auto`（默认） | 不确定时 | `server.url` 指向本机（127.0.0.1 / localhost / 本机 IP）就用 `local`，否则 `http` |

- `local` 不需要 `server.url`；装了 `psutil` 时跨平台，没装则退回 shell 命令（Linux / macOS）。
- `http` 用的服务在 [GroundStation/Server/](../../Server/)（服务器上跑，把信息发出来给中枢拉）：
  服务器上 `pip install -r requirements.txt && python3 server_monitor.py` 即可。

面板上采集并展示的字段（三种采集方式字段一致，取不到的显示 `-`）：

| 分组 | 内容 |
|------|------|
| 占用率卡片 | CPU 使用率（+ 核数/主频）、内存（+ 已用/总量）、磁盘（+ 已用/总量）、交换分区（+ 已用/总量） |
| 数值卡片 | 进程数、系统负载（1/5/15 分钟）、网络速率（↓/↑ 实时）、累计流量（↓/↑） |
| 详情行 | 主机名、服务器时间、处理器型号、系统内核、机器运行时长、中枢已监控时长、内存可用、磁盘可用、上次采集时间、采集方式、采集延迟、在线状态 |

- **网络速率**由中枢用相邻两次采样的累计流量差值算出（约 2 秒窗口），
  因此三种采集方式都支持，不需要服务器端额外配合；刚启动的前 2 秒显示 `-`。
- 面板基准高度由 `frontend/index.html` 里 `#serverPanel { min-height: 540px }` 控制：
  这一行越高，同行右侧的 YOLO 画面框（`flex` 自适应）就越大，想调画面大小改这个值即可。

## 无人机自动发现

地面站启动后会监听 UDP 20004，自动接收无人机电脑端（`agcs_drone`）与
YOLO 模型仪表盘（`agcs_yolo`）广播的地址，各自每 2 秒广播一次；
`drone.url` / `yolo.url` 没写或还是 127.0.0.1 时，自动使用广播发现到的地址。
注意：广播只在同一局域网内有效，跨网段需在 `data/config.yaml` 手动填地址。

## 前置条件

1. 机器人：已切成 STA 局域网模式，`autonomous_pick.py` 运行中（自动开启
   HTTP 服务，监听 5000）。
2. 无人机：数传在线（QGC 能连上），pymavlink 监听的 14550 端口能收到 MAVLink，
   图传 RTSP 可达。
3. 网络：地面站能 `ping` 通机器人；仪表盘端口如需跨网段访问，按
   [YOLO教学与空地协同.md](../../../YOLO教学与空地协同.md) 7.8 做端口转发。

## 常见问题

| 现象 | 排查 |
|------|------|
| 机器人卡片显示离线 | `ping 10.194.228.89`；确认 `autonomous_pick.py` / `CS-video.py` 已启动、Flask 已装 |
| 机器人画面不显示 | 确认机器人端在跑 `autonomous_pick.py` 或 `CS-video.py`（推流源）；浏览器直开 `http://机器人IP:5000/video.mjpeg` 可单独验证机器人端 |
| 无人机卡片显示未接入 | 确认数传在线、QGC 能连上；确认防火墙放行 UDP 14550；数传端把地面站 IP+14550 加为目标 |
| 无人机画面显示 no video / 状态卡 | 确认无人机电脑端 Dorn/Dashboard 已运行（20002）；EWRF 接收机已插 USB 且 `drone.camera` 序号正确（`capture_external_camera.py --list` 查）；中枢和它不在同一台电脑时改 data/config.yaml `drone.url` |
| 模型未加载 / YOLO 画面无标注 | 确认 YOLO 模型仪表盘（YOLOModel/Dashboard，20003）已运行且能拉到无人机画面；中枢 data/config.yaml `yolo.url` 指向它（或留 127.0.0.1 走广播自动发现） |
