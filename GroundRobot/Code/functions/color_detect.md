# color_detect.py 教学文档

## 1. 这个文件是做什么的

颜色识别演示程序。启动后打开摄像头，在画面上圈出目标颜色并显示中心点。
它同时具备两种形态：

1. **独立运行**：`python3 color_detect.py --color red`，适合课堂演示和调试
2. **RPC 模块**：提供 `init/start/stop/exit/run` 五个函数，可以被
   `SpiderPi.py` 主程序（上位机 App 框架）加载，玩法由手机/上位机启动

## 2. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `argparse` | Python 标准库 | 解析命令行参数（`--color`） |
| `cv2` | OpenCV | 图像显示 `imshow`、窗口事件 `waitKey` |
| `calibration.camera.Camera` | 官方 SDK | 摄像头封装（后台线程取帧） |
| `functions.robot_config` | 本项目 | 读取行为参数 |
| `functions.vision_utils` | 本项目 | `load_lab_data` / `detect_color` / `load_undistort_maps` |

## 3. 逐段讲解

### 3.1 全局状态与 RPC 接口

```python
__isRunning = False
_lab_data = None
_target_color = 'red'
_min_area = 500
```

模块级变量相当于"整个模块共享的内存"。官方 App 框架通过 RPC 调用
`init/start/stop/exit/run`，这些变量就是各函数之间传递状态的通道。

### 3.2 五个 RPC 函数的职责

| 函数 | 何时被调用 | 做什么 |
|------|-----------|--------|
| `init()` | App 初始化 | 读参数、读颜色阈值 |
| `start()` | 用户点"开始" | `__isRunning = True` |
| `stop()` | 用户点"停止" | `__isRunning = False` |
| `exit()` | 退出玩法 | 调 `stop()` |
| `run(img)` | 每帧回调 | 检测颜色并绘制，返回新图像 |

> 新手提示：`init()` 里的 `params['vision']['target_color']` 是"嵌套字典"取值，
> 看不懂先读 [robot_config.md 第 2 节「新手必读」](robot_config.md)。

`run` 里的保护判断很重要：

```python
if not __isRunning or _lab_data is None:
    return img
```

没开始或参数没加载时直接返回原图，避免空指针。

### 3.3 独立运行主循环

```python
camera = Camera()
camera.camera_open()
```

官方 `Camera` 类内部启动了一个**后台线程**不断读帧，`camera.frame` 就是最新一帧。
主循环只做三件事：取帧 → 处理 → 显示：

```python
while True:
    img = camera.frame
    if img is None:
        time.sleep(0.01)
        continue
    frame = cv2.remap(img.copy(), mapx, mapy, cv2.INTER_LINEAR)
    run(frame)
    cv2.imshow('ColorDetect', frame)
    key = cv2.waitKey(1)
    if key == 27:   # ESC
        break
```

注意 `img.copy()`：`Camera` 后台线程会不断覆盖 `frame`，如果不复制就处理，
可能在处理到一半时画面被换掉，出现撕裂/崩溃。

`cv2.waitKey(1)` 让 OpenCV 窗口刷新并接收键盘事件；返回 27 是 ESC 键的
ASCII 码。**没有 `waitKey`，`imshow` 不会显示画面**，这是最常见的"黑屏"原因之一。

## 4. 与官方原版的区别

官方 `functions/color_detect.py` 是纯 RPC 模块（无 `__main__`），并且会初始化
机械臂（`init_move`）。本文件的改进：

- 加了 `if __name__ == '__main__':`，可以独立运行，方便教学
- 不主动控制机械臂，专注"视觉"本身，职责更单一
- 保留五个 RPC 函数，接入 `SpiderPi.py` 时行为一致

> 同步脚本会覆盖树莓派上的官方同名文件；官方原版在资料包
> `3 源码资料/SpiderPi_Pro.zip`，随时可以还原。

## 5. 常见问题排查

| 现象 | 可能原因 | 排查/解决 |
|------|----------|-----------|
| 窗口黑屏 | 摄像头没开、或没调 `waitKey` | 检查 `camera.camera_open()` 是否执行；确认循环里有 `cv2.waitKey(1)` |
| `cv2.error: ... camera` | 摄像头被占用 | 先 `sudo systemctl stop spiderpi` 关闭自启主程序 |
| 识别不到颜色 | 阈值没圈住目标颜色 | 用上位机"颜色模型参数调节工具"重新取 `lab_config.yaml` |
| 命令行传了 `--color green` 但没反应 | 参数解析后没赋值 | 检查 `_target_color` 是否被 `args.color` 覆盖 |
| 画面很卡 | 每帧都做 `remap` 或检测耗时 | `remap` 只做一次映射表加载；检测小图 640x480 应 <50ms |

## 6. 动手练习

1. 把 `--color` 的默认值改成从 `robot_params.yaml` 读取，跑通"参数驱动"
2. 在画面上叠加显示 `centerX`、`centerY` 数字
3. 试着用 `cv2.rectangle` 画外接矩形替代圆形，观察哪个更贴近目标
