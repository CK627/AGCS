#!/usr/bin/python3
# coding=utf8
"""颜色标定辅助：抓一帧，复刻 detect_color 的管线，算出目标色块实际 LAB 范围。

复刻 detect_color：Y 通道直方图均衡 → 缩放 320x240 → 高斯模糊 → LAB。
把目标色块放在相机正前方、画面中央，运行后按输出的分位数更新
config/lab_config.yaml 的 min/max。

用法：
    python3 tasks/CS/CS-color-cal.py --color red
"""
import argparse
import sys

import cv2
import numpy as np


def detect_pipeline(bgr):
    """复刻 detect_color 的预处理：均衡 + 缩放 + 模糊 + LAB。"""
    img = bgr.copy()
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    ch = cv2.split(ycrcb)
    cv2.equalizeHist(ch[0], ch[0])
    cv2.merge(ch, ycrcb)
    img = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_NEAREST)
    img = cv2.GaussianBlur(img, (5, 5), 5)
    return cv2.cvtColor(img, cv2.COLOR_BGR2LAB)


def main():
    parser = argparse.ArgumentParser(description='颜色标定辅助')
    parser.add_argument('--color', default='red', help='目标颜色')
    args = parser.parse_args()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(5):
        cap.read()
    ok, bgr = cap.read()
    cap.release()
    if not ok:
        print('FAIL: 读帧失败')
        sys.exit(1)

    lab = detect_pipeline(bgr)
    h, w = lab.shape[:2]
    # 采样中央小区域（色块放这里）
    center = lab[h // 3:h * 2 // 3, w // 3:w * 2 // 3].reshape(-1, 3)

    print('=== 目标色块 LAB 范围（中央区域，L,A,B）===')
    for name, q in [('建议min(p10)', 10), ('p50', 50), ('建议max(p90)', 90)]:
        vals = np.percentile(center, q, axis=0).round(0).astype(int).tolist()
        print('  %s: %s' % (name, vals))

    # 打印当前 lab_config.yaml 对应颜色的阈值做对比
    print('\n把 min 设成 p10 附近、max 设成 p90 附近（留点余量）写进 lab_config.yaml')


if __name__ == '__main__':
    main()
