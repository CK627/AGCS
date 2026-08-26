#!/usr/bin/env python3
# coding=utf8
"""macOS 推理/确认脚本：MPS 加速，带类别名/置信度/面积筛选。

用法：
    python predict_macos.py --model runs/pod_pest_v1/weights/best.pt --source 0
    python predict_macos.py --model best.pt --source 视频.mp4
    python predict_macos.py --model best.pt --source rtsp://...
"""
import argparse

import cv2
import torch
from ultralytics import YOLO

CONF = 0.45                 # 置信度阈值：>= 它才算"确认"
TARGET_CLASSES = {'worm'}   # 类别名要和 data.yaml 一致！
MIN_AREA = 100              # 框面积小于它的忽略


def pick_device():
    if torch.backends.mps.is_available():
        return 'mps'
    return 'cpu'


def confirm_targets(results):
    """三重筛选：类别名 + 置信度 + 面积，返回我们关心的目标。"""
    found = []
    names = results.names                    # {0: 'worm'}（单类：只识别虫）
    for box in results.boxes:
        cls_id = int(box.cls[0])
        name = names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        area = (x2 - x1) * (y2 - y1)
        if name not in TARGET_CLASSES:
            continue
        if conf < CONF:
            continue
        if area < MIN_AREA:
            continue
        found.append({
            'class': name,
            'conf': conf,
            'center': ((x1 + x2) // 2, (y1 + y2) // 2),
            'area': area,
        })
    return found


def main():
    parser = argparse.ArgumentParser(description='macOS YOLO 推理')
    parser.add_argument('--model', default='yolov8n.pt', help='模型路径')
    parser.add_argument('--source', default='0', help='图片/视频/摄像头(0)/RTSP')
    parser.add_argument('--device', default=None, help='mps/cpu（默认自动）')
    args = parser.parse_args()

    device = args.device or pick_device()
    print('使用加速后端:', device)

    model = YOLO(args.model)
    cap = cv2.VideoCapture(0) if args.source == '0' else cv2.VideoCapture(args.source)
    streak = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        results = model.predict(frame, conf=CONF, verbose=False, device=device)[0]
        found = confirm_targets(results)
        frame = results.plot()
        if found:
            streak += 1
            if streak >= 3:   # 连续 3 帧确认才报警，过滤偶发误检
                print('[确认] %s conf=%.2f center=%s'
                      % (found[0]['class'], found[0]['conf'], found[0]['center']))
        else:
            streak = 0
        cv2.imshow('predict', frame)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
