# 服务器监控（GroundStation/Server）

给地面站中枢提供系统信息：**CPU / 内存 / 磁盘 / 网络 / 运行时间 / 主机名**。

> **先看清楚用哪种方式**（中枢 `data/config.yaml` 的 `server.mode`）：
>
> | 情况 | 中枢配什么 | 要不要本服务 |
> |------|-----------|--------------|
> | **中枢就装在这台服务器上** | `mode: local` | ❌ 不用，中枢直接读本机（最省事） |
> | 中枢在另一台电脑，本机想提供系统信息 | `mode: http` | ✅ 跑本服务，中枢来拉 `/api/system` |
>
> `mode: auto`（默认）会自动判断：`server.url` 指向本机就用 `local`，否则用 `http`。

以下是 **http 模式**的用法：把本目录（`GroundStation/Server/`）的代码放到"服务器"那台机器上跑起来，
它就把系统信息发出来；中枢的 `server.url`（默认 `192.168.124.7:20006`）指向它即可收到。

## 运行

```bash
cd GroundStation/Server
python -m pip install -r requirements.txt
python3 server_monitor.py                 # 默认 0.0.0.0:20006
python3 server_monitor.py --port 20006    # 改端口
```

启动后浏览器打开 `http://<本机IP>:20006/api/system` 能看到 JSON 就对了。

## 接口

| 接口 | 用途 | 返回 |
|------|------|------|
| `GET /api/system` | 中枢显示服务器信息 | 见下 |
| `GET /status` | 轻量在线探测（中枢算延迟） | `{ok, host, time}` |

`/api/system` 返回示例：

```json
{
  "ok": true,
  "host": "server-01",
  "platform": "Linux-6.1.0-x86_64",
  "python": "3.11.2",
  "uptime_sec": 123456.0,
  "agent_uptime_sec": 3600.0,
  "cpu": {"percent": 12.5, "count": 8, "count_physical": 4, "load_avg": [0.5, 0.4, 0.3]},
  "mem": {"percent": 43.2, "total_mb": 16384.0, "used_mb": 7078.0, "available_mb": 9306.0},
  "disk": {"percent": 55.1, "total_gb": 500.0, "used_gb": 275.5, "free_gb": 224.5},
  "net": {"sent_mb": 120.5, "recv_mb": 980.2}
}
```

## 与中枢的对应关系

中枢 `GroundStation/Dashboard/data/config.yaml`：

```yaml
server:
  url: http://192.168.124.7:20006   # 本服务地址
  name: 服务器                       # 面板显示名称
  enabled: true                      # 是否显示服务器信息面板
```

如果那台机器上已经有自己的监控服务，只要它提供同样格式的 `GET /api/system`，
把 `server.url` 指过去即可，不必用本脚本。

## 说明

- 没装 `psutil` 也能启动：`/status` 在线探测照常，`/api/system` 里 CPU/内存/磁盘为空。
- 防火墙需放行 `20006`（TCP），否则中枢拉不到。
