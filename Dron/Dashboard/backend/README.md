# 无人机电脑端网页（视频 + UDP 监控 + 下达任务）

这是无人机在地面站电脑上的网页，把三件事放在一个页面：

1. **视频画面**：显示摄像头实时画面（RTSP → 网页 MJPEG）
2. **UDP 监控**：读无人机通过 MAVLink/UDP 发来的"小纸条"，显示姿态/GPS/电量
3. **下达任务**：填航点，预览/上传任务清单给飞控

## 技术栈（已确定）

| 层 | 选型 | 为什么 |
|----|------|--------|
| 后端 | Python（Flask + pymavlink） | 读 UDP MAVLink、解析 JSON 任务、输出 JSON 数据，和无人机 Python 工具链一致 |
| 前端 | 纯 HTML + CSS + 原生 JS | **不引入 React/Vue 等框架**，页面是原网页，加载快、无构建步骤 |
| 数据格式 | JSON | 后端接口全部用 JSON，前端 `fetch` 轮询即可，简单可靠 |

前端只做展示和发请求，不做复杂逻辑；无人机信息获取、UDP 监控、任务转 MAVLink
都由 Python 后端负责。

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

改 [config.py](config.py)：

- `MAVLINK_CONN`：UDP 数据来源，默认 `udpin:0.0.0.0:14550`
- `RTSP_URL`：摄像头地址（官方手册默认值）
- `VIDEO_ENABLED`：没有摄像头时先设为 `False`，页面视频区域显示"视频未连接"

## 三个页面能力对应哪些接口

| 页面功能 | 后端接口 | 比喻 |
|----------|----------|------|
| 视频画面 | `/video_feed` | 把摄像头的录像转成网页能看的格式 |
| 实时监控 | `/api/telemetry`、`/api/history` | 从公告栏取最新纸条，再画曲线 |
| 预览任务 | `/api/task/preview` | 先把路线清单打印出来给你核对 |
| 下达任务 | `/api/task/upload` | 把路线清单发给飞控（只上传，不启动） |

## 安全红线

1. 网页**只上传任务，不自动解锁、不自动起飞**
2. 解锁仍然用遥控器 5 通道；紧急情况遥控器 6 急停 / 7 返航
3. 航点必须是实际场地坐标，默认示例坐标只是演示

## 新手建议

先读 [Code/黑话对照表.md](../Code/黑话对照表.md)，再用
[Code/闯关任务.md](../Code/闯关任务.md) 的离线数据把监控和任务预览跑通，
最后接模拟器/真机测试"下达任务"。
