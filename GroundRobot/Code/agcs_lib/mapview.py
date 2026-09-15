#!/usr/bin/python3
# coding=utf8
"""2D 地图可视化：终端 ASCII 图 + PNG 预览。

机器人在现场，SSH 终端里直接看 ASCII 图就能确认扫得对不对，不用把图拉回来；
PNG 是给本地看细节用的（可选，需要 cv2）。
"""
import numpy as np


def ascii_map(points, cols=76, char_aspect=2.0, max_rows=38, margin_mm=300.0,
              marks=None, title=''):
    """把 2D 点集画成终端 ASCII 图，返回字符串（外面自己 print）。

    points: (N,2) mm，x 右 / y 前（y 轴画成朝上）。
    char_aspect: 终端字符高/宽比（一般 2:1），用来校正纵横比，不然图会被拉长。
    marks: [(x, y, '字符')] 额外标注（例如机器人所在的角）。
    """
    pts = np.asarray(points, dtype=np.float32)
    if pts.size == 0:
        return '(没有点)'
    lo = pts.min(axis=0) - margin_mm
    hi = pts.max(axis=0) + margin_mm

    span_x = max(float(hi[0] - lo[0]), 1.0)
    span_y = max(float(hi[1] - lo[1]), 1.0)
    cell = span_x / cols                                   # 每列多少 mm
    if span_y / (cell * char_aspect) > max_rows:           # 太高了，按行数反推
        cell = span_y / (max_rows * char_aspect)
    ncols = max(2, int(span_x / cell) + 1)
    nrows = max(2, int(span_y / (cell * char_aspect)) + 1)

    grid = [[' '] * ncols for _ in range(nrows)]

    def put(x, y, ch):
        c = int((x - lo[0]) / cell)
        r = int((y - lo[1]) / (cell * char_aspect))
        if 0 <= c < ncols and 0 <= r < nrows:
            grid[nrows - 1 - r][c] = ch          # y 朝上画
            return True
        return False

    for x, y in pts:
        put(x, y, '#')
    for m in (marks or []):
        put(m[0], m[1], str(m[2])[0] if len(m) > 2 else 'R')  # 一格只能放一个字符

    head = title or ('%d 点  x %.0f..%.0f  y %.0f..%.0f mm  (1 格≈%.0fmm)'
                     % (len(pts), pts[:, 0].min(), pts[:, 0].max(),
                        pts[:, 1].min(), pts[:, 1].max(), cell))
    lines = [head, '+' + '-' * ncols + '+']
    lines += ['|' + ''.join(row) + '|' for row in grid]
    lines.append('+' + '-' * ncols + '+')
    return '\n'.join(lines)


def draw_map_png(points, path, size_px=900, margin_mm=400.0,
                 marks=None, title='', background=255):
    """把 2D 点集画成 PNG 预览（需要在本地看细节时用）。返回是否成功。

    marks: [(x, y, 标签, BGR颜色)]。
    """
    import cv2  # 只有这个函数需要 cv2

    points = np.asarray(points, dtype=np.float32)
    if len(points) == 0:
        return False
    lo = points.min(axis=0) - margin_mm
    hi = points.max(axis=0) + margin_mm
    for m in (marks or []):
        lo = np.minimum(lo, np.array([m[0], m[1]], dtype=np.float32) - margin_mm)
        hi = np.maximum(hi, np.array([m[0], m[1]], dtype=np.float32) + margin_mm)
    span = float(max(hi[0] - lo[0], hi[1] - lo[1], 1.0))
    scale = (size_px - 1) / span

    def to_px(x, y):
        return (x - lo[0]) * scale, size_px - 1 - (y - lo[1]) * scale

    img = np.full((size_px, size_px, 3), background, dtype=np.uint8)
    for g in np.arange(np.ceil(lo[0] / 500.0) * 500.0, hi[0], 500.0):
        cv2.line(img, (int(to_px(g, 0)[0]), 0), (int(to_px(g, 0)[0]), size_px - 1),
                 (225, 225, 225), 1)
    for g in np.arange(np.ceil(lo[1] / 500.0) * 500.0, hi[1], 500.0):
        cv2.line(img, (0, int(to_px(0, g)[1])), (size_px - 1, int(to_px(0, g)[1])),
                 (225, 225, 225), 1)
    c0, r0 = to_px(0.0, 0.0)
    cv2.line(img, (int(c0), 0), (int(c0), size_px - 1), (200, 200, 200), 1)
    cv2.line(img, (0, int(r0)), (size_px - 1, int(r0)), (200, 200, 200), 1)

    cols = ((points[:, 0] - lo[0]) * scale).astype(np.int32)
    rows = (size_px - 1 - (points[:, 1] - lo[1]) * scale).astype(np.int32)
    ok = (cols >= 0) & (cols < size_px) & (rows >= 0) & (rows < size_px)
    img[rows[ok], cols[ok]] = (60, 60, 60)

    for m in (marks or []):
        c, r = to_px(m[0], m[1])
        color = m[3] if len(m) > 3 else (0, 0, 255)
        cv2.drawMarker(img, (int(c), int(r)), color, cv2.MARKER_CROSS, 16, 2)
        cv2.putText(img, str(m[2]), (int(c) + 8, int(r) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    txt = title or ('%d pts  x %.0f..%.0f  y %.0f..%.0f'
                    % (len(points), points[:, 0].min(), points[:, 0].max(),
                       points[:, 1].min(), points[:, 1].max()))
    cv2.putText(img, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return bool(cv2.imwrite(path, img))
