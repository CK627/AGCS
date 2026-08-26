# yolo_detect.py 教学文档

## 1. 这个文件是做什么的

用 YOLO 模型做目标检测（可选模块）。对应 README 第三步"目标识别模型部署"：
先用颜色检测把流程跑通，等豆荚螟危害特征（蛀孔、变色豆荚）的数据集和模型训练好
之后，把检测器换成 YOLO，**主程序不用大改**。

当前豆荚螟目标还识别不了，第一步先拿通用模型（`yolov8n.pt`）验证流程：

```bash
python3 ~/spiderpi/functions/yolo_detect.py
```

## 2. 用到了哪些模块

| 模块 | 来源 | 作用 |
|------|------|------|
| `cv2` | OpenCV | 画框、画文字 |
| `ultralytics` | 第三方（需 `pip3 install ultralytics`） | YOLO 推理：加载模型、`predict` |
| `calibration.camera.Camera` | 官方 SDK | 摄像头（仅演示用） |
| `functions.vision_utils` | 本项目 | 畸变校正（仅演示用） |

`ultralytics` 是在 `__init__` 里**延迟导入**的：

```python
try:
    from ultralytics import YOLO
except ImportError:
    raise SystemExit('未安装 ultralytics，请先在树莓派执行: pip3 install ultralytics')
```

这样做的目的：没装 ultralytics 时，程序启动就给出明确提示，而不是在别处
神秘报错。**"错误要在最早的地方、用最清楚的文字暴露出来"**，这是排查问题
的第一原则。

## 3. 关键概念：YOLO 输出是什么

```python
results = self.model.predict(img, verbose=False, imgsz=self.size[0])[0]
for box in results.boxes:
    cls_id = int(box.cls[0])       # 类别编号，如 0
    conf   = float(box.conf[0])    # 置信度 0~1
    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]  # 框的左上/右下像素
```

YOLO 一次可能输出多个框，所以必须循环筛选：

1. **类别筛选**：`results.names[cls_id]` 是不是我们要的目标（默认 `pod`）
2. **置信度筛选**：`conf >= self.conf`
3. **面积筛选**：框太小（远处/误检）直接跳过

命中后返回和 `detect_color` **相同结构**的 dict：

```python
return {'center': center, 'area': area, 'color': names.get(cls_id), 'conf': conf}
```

这就是整合程序能无缝切换两种检测器的原因：上层只认 `center` 和 `area` 这两个
字段，不关心底层是颜色还是深度学习。

## 4. 模型从哪来（与项目进度的关系）

1. **现在**：用官方预训练 `yolov8n.pt` 跑通代码流程
2. **中期**：收集毛豆豆荚、蛀孔照片 → 用地面站 GPU 标注并训练
3. **后期**：导出轻量化模型（`onnx`/`tflite`）部署到树莓派，或通过 Wi-Fi
   调用地面站推理服务

## 5. 常见问题排查

| 现象 | 可能原因 | 排查/解决 |
|------|----------|-----------|
| 启动即退出，提示未安装 | 没装 ultralytics | STA 局域网模式下 `pip3 install ultralytics` |
| 不画框 | 目标类别名不匹配 | 打印 `results.names` 看模型类别，检查 `--classes` 参数 |
| 画了一堆小框 | 置信度太低 | 调高 `conf`（默认 0.45） |
| 推理很慢（<5 FPS） | 树莓派 CPU 推理 | 用 `yolov8n` 最小模型；降低 `imgsz`；或改调地面站 |
| 报 `CUDA` 相关错误 | 树莓派无 GPU | 正常现象，ultralytics 自动用 CPU |

## 6. 动手练习

1. 用 `yolov8n.pt` 检测画面里的杯子/人等，观察 `--classes` 怎么筛
2. 把 `conf` 从 0.45 改到 0.9，体会误检与漏检的权衡
3. 把返回结构改成 `label` 字段，并同步修改 `autonomous_pick.py` 的调用处，
   理解"接口约定"为什么重要
