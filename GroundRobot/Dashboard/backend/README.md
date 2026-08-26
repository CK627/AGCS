# 机器人仪表盘后端

纯 HTTP 服务，跑在地面站电脑上，把机器人端的数据代理成网页能直接用的接口。

## 数据流

```text
树莓派 autonomous_pick.py（每帧 publish_frame 压缩 JPEG）
  → task_server.py /video.mjpeg（MJPEG 流，端口 5000）
  → 本后端 /video_feed（代理转发）
  → 浏览器 <img> 实时显示
```

状态与任务同样走代理：`/api/status` ↔ 机器人 `/status`，`/api/task` ↔ 机器人 `/task`。

## 启动

```bash
cd GroundRobot/Dashboard
python -m pip install -r requirements.txt
cd backend && python app.py
```

浏览器打开 http://127.0.0.1:20002。

服务优先用 waitress 启动（支持并发处理视频流与状态轮询）；未安装 waitress 时
自动退回 Flask 开发服务器（同样开了 `threaded=True`）。

## 配置

改 [config.py](config.py)：

- `ROBOT_URL`：机器人地址（当前 `http://10.194.228.87:5000`）
- `VIDEO_ENABLED`：视频开关
- `DASHBOARD_HOST` / `DASHBOARD_PORT`：监听地址/端口（默认 `20002`）

> 提示：仪表盘直接跑在树莓派本机时，把 `ROBOT_URL` 改成
> `http://127.0.0.1:5000` 即可。
