# vision_utils.py 教学文档

## 1. 这个文件是做什么的

这是视觉模块的"工具箱"，包含四类能力：

1. **读取配置**：颜色阈值（`lab_config.yaml`）、相机标定参数（`camera_cal.yaml`）
2. **颜色检测**：LAB 阈值分割 → 形态学去噪 → 找最大轮廓 → 输出目标中心和面积
3. **坐标变换**：把图像上的像素点换算成机械臂坐标系下的厘米坐标
4. **畸变校正**：加载相机标定结果，生成 `cv2.remap` 用的映射表

把它单独抽出来，是因为 `color_detect.py`、`autonomous_pick.py` 都要用同一套
逻辑。**好处**：改一处，处处生效；坏处是出问题时需要知道"这个函数是从哪调来的"，
所以本文档会讲清每个函数的职责和依赖。

## 2. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `cv2` | OpenCV（树莓派镜像预装） | 图像处理全部函数 |
| `numpy` | 第三方 | 矩阵运算（相机参数、坐标变换） |
| `common.misc` | 官方 SDK（`spiderpi_sdk/common_sdk`） | `map()` 线性映射 |
| `common.yaml_handle` | 官方 SDK | 读取 YAML；内含 `lab_file_path` / `camera_file_path` 常量 |
| `calibration.CalibrationConfig` | 官方 SDK（`spiderpi_sdk/camera_calibration_sdk`） | 提供标定参数路径常量 `calibration_param_path` |

> 官方 SDK 的包（`common`、`calibration`、`sensor`、`arm_ik`）在树莓派镜像里是
> 以 pip 安装包形式存在的，所以可以直接 `from common import ...`。

## 3. 关键概念：颜色识别管线

`detect_color` 里的处理顺序，每一环都有明确目的：

```
原图 → 缩放(640x480) → 高斯模糊 → 转LAB → inRange阈值掩膜
    → 腐蚀 → 膨胀 → findContours → 最大轮廓 → minEnclosingCircle → 映射回原图坐标
```

### 3.1 为什么用 LAB 颜色空间？

- **RGB** 受光照影响极大（同一个红色，亮处/暗处数值差很多）
- **LAB** 把"亮度 L"和"颜色 a、b"分开。做颜色识别时只看 a、b，对光照变化更鲁棒
- `lab_config.yaml` 里的 `min/max` 就是 LAB 三个通道的阈值范围，官方上位机
  "颜色模型参数调节工具"生成的就是这个文件

### 3.2 掩膜 + 形态学

```python
frame_mask = cv2.inRange(frame_lab, (minL,minA,minB), (maxL,maxA,maxB))
eroded  = cv2.erode(frame_mask, kernel)   # 去掉细小噪点
dilated = cv2.dilate(eroded, kernel)      # 把目标区域还原并连通
```

`inRange` 把"颜色落在阈值内"的像素标白，其余标黑；`erode` 腐蚀（白色区域变小，
去除噪点）、`dilate` 膨胀（目标区域变大，把断裂的部分连起来）。阈值调不好时，
先用一个独立脚本显示这三步的中间结果，就能快速定位是"阈值没圈住目标"还是
"噪声太多"。

### 3.3 最大轮廓与坐标映射

```python
area_max_contour, area_max = get_area_max_contour(contours, min_area)
((centerX, centerY), radius) = cv2.minEnclosingCircle(area_max_contour)
```

- `findContours` 找所有白色区域的外边界
- `get_area_max_contour` 只保留面积最大的那个，并过滤掉小于 `min_area` 的
  小目标（远处的干扰）
- `minEnclosingCircle` 求最小外接圆，得到中心点（在**缩放后**的小图上）

### 3.4 `misc.map`：小图坐标 → 原图坐标

```python
centerX = int(misc.map(centerX, 0, size[0], 0, img_w))
```

检测在 640x480 的小图上做（省 CPU），但显示和抓取要用原图坐标，所以用
`map(x, in_min, in_max, out_min, out_max)` 做线性映射：

```
out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
```

## 4. 关键概念：像素坐标 → 机械臂坐标

### 4.1 相机标定是什么

相机拍到的是一张 2D 图，但我们想知道目标在机器人前方 3D 空间里的位置。这一步
叫**单目测距/平面测距**，前提是目标落在地面上（或已知平面上）。需要的参数：

| 参数 | 含义 | 存在哪 |
|------|------|--------|
| `K`（内参矩阵） | 焦距、光心，描述镜头本身 | `camera_cal.yaml` 的 `block_params.K` |
| `R`、`T`（外参） | 相机相对地面的旋转和平移 | `block_params.R` / `T` |

这些参数是官方 `camera_cal_main.py` + AprilTag 标定板测出来的，**每个机器人
安装位置不同，标定结果也不同**，不能通用。

### 4.2 `camera_to_world` 的数学步骤（理解即可）

```python
worldPtCam   = inv(K) * [u, v, 1]      # 像素 → 相机坐标（射线方向）
worldPtPlane = inv(R) * worldPtCam     # 旋转到地面坐标系
scale        = T_z / worldPtPlane_z    # 沿射线缩放，让 z=0（落到地面）
world        = scale * worldPtPlane - inv(R)*T
```

代码里全是 `np.asmatrix` 矩阵运算，**不需要手推**，但要知道：

- 结果是毫米（`mm`）
- `pixel_to_arm_coord` 里 `int(-w[0]) / 10.0` 转成厘米并做符号转换
  （相机坐标与机械臂坐标方向相反），再叠加机械臂检测姿态的偏移

### 4.3 畸变校正

镜头边缘会有桶形畸变，`load_undistort_maps` 读取标定结果生成 `mapx/mapy`，
每帧用 `cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)` 校正。**只做一次**，
比每帧调 `undistort` 快很多。

## 5. 常见问题排查

| 现象 | 问题出在哪个环节 | 排查方法 |
|------|------------------|----------|
| 目标明明在画面里却检测不到 | 阈值 `lab_config.yaml` 没圈住颜色 | 用上位机颜色调节工具重新取阈值 |
| 检测到一堆杂物 | `min_area` 太小，或形态学没去噪 | 调大 `vision.min_area`；检查 erode/dilate 中间图 |
| 目标中心位置跳动很大 | 轮廓不稳定，或标定参数不对 | 加稳定帧数（多次取平均）；重新标定相机 |
| 抓取点偏左/偏右固定偏移 | 外参 R/T 与当前安装位置不一致 | 重新做 `camera_cal_main.py` 位置校准 |
| `camera_to_world` 数值巨大/为 0 | `T[2]` 为 0 或 K 解析错 | 打印 K/R/T 检查 `reshape(3,3)` 是否成功 |
| `np.load` 报错 | 标定 npz 文件不存在 | 先跑官方 `camera_cal_main.py` 生成标定参数 |

## 6. 动手练习

1. 单独写一个测试脚本：读摄像头 → 只显示 `frame_lab` 的 L 通道，观察亮度变化
2. 把 `detect_color` 的 `size` 改成 `(320, 240)`，比较速度和精度
3. 在画面上画一条十字线标出画面中心，用尺子量"目标中心偏离画面中心"对应的
   实际距离，验证 `camera_to_world` 是否准确
