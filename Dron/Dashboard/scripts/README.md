# scripts 快速启动（只服务 Dron/Dashboard）

无人机电脑端网页：视频 + UDP 监控 + 下达任务。

## macOS / Linux

```bash
bash Dron/Dashboard/scripts/macOS/start.sh          # 启动（默认端口 20000）
bash Dron/Dashboard/scripts/macOS/start.sh status   # 查看状态
bash Dron/Dashboard/scripts/macOS/start.sh stop     # 停止
bash Dron/Dashboard/scripts/macOS/start.sh restart  # 重启
```

## Windows

```bat
Dron\Dashboard\scripts\Windows\start.bat
```

## 常用命令

| 命令 | 说明 |
|------|------|
| `start [端口]` | 启动服务（默认 20000） |
| `stop` | 停止服务 |
| `restart [端口]` | 重启服务 |
| `status` | 查看运行状态 |
| `setup` | 安装 Python 依赖（首次运行） |
| `install` | 环境检查（Python / 依赖） |
| `update` | 更新到最新版本 |
| `uninstall` | 卸载（清理 data/logs） |
| `help` | 帮助 |

启动后浏览器打开 http://127.0.0.1:20000。
