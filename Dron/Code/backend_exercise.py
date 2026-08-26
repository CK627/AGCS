#!/usr/bin/python3
# coding=utf8
"""后端训练：读取测试数据，整理成约定的 JSON 输出。

分工：网页（前端）由队友编写，你们只负责"后端"——
把无人机数据读进来、整理好、输出成约定格式（见 backend_exercise.md）。

本文件是训练骨架：读 test_data/telemetry_sample.csv，
按 TODO 补全代码后，会输出：
    1) telemetry.json   （最新一帧）
    2) history.jsonl    （历史数据，每行一条 JSON）

运行: python3 backend_exercise.py
       python3 backend_exercise.py --csv test_data/telemetry_flight.csv
"""
import argparse
import csv
import json
import os

TEST_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'test_data', 'telemetry_sample.csv')
OUT_JSON = 'telemetry.json'
OUT_JSONL = 'history.jsonl'

# 每个字段对应的类型：读 CSV 时把字符串转换成真正的数字/布尔值，
# 这样写进 JSON 时才不会变成 "0.0" 这种带引号的字符串。
FIELD_PARSERS = {
    'time': float,
    'mode': str,
    'armed': lambda value: value.strip().lower() == 'true',
    'roll_deg': float,
    'pitch_deg': float,
    'yaw_deg': float,
    'lat': float,
    'lon': float,
    'alt_m': float,
    'sats': int,
    'volt': float,
    'battery_pct': int,
    'heading_deg': int,
    'groundspeed': float,
    'climb': float,
}


def read_csv(path):
    """TODO 1: 读取 CSV，返回字典列表（每行一个 dict）。

    提示：
        with open(path, encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    注意 DictReader 读出来的值都是字符串，数字要先转 float/int。
    """
    with open(path, encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f)
        rows = []
        for raw in reader:
            row = {}
            for key, value in raw.items():
                key = key.strip()
                parser = FIELD_PARSERS.get(key)
                if parser is None:
                    # 未来 CSV 里多出的字段先原样保留，避免丢数据。
                    row[key] = value
                    continue

                value = (value or '').strip()
                if value == '':
                    row[key] = None
                    continue
                row[key] = parser(value)
            rows.append(row)
    return rows


def latest_snapshot(rows):
    """TODO 2: 取"最新一帧"作为 telemetry.json 的内容。

    要求：rows 的最后一行（最新），字段保持 csv 的字段名即可。
    提示：
        return rows[-1]
    """
    return rows[-1]


def to_history(rows):
    """TODO 3: 把历史数据转成 JSON Lines 文本。

    JSON Lines（jsonl）：每行一条 JSON，行尾加 \\n。
    提示：
        return '\\n'.join(json.dumps(r, ensure_ascii=False) for r in rows)
    """
    return ''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in rows)


def main():
    parser = argparse.ArgumentParser(description='后端训练：测试数据 → JSON')
    parser.add_argument('--csv', default=TEST_CSV, help='测试数据文件（默认 telemetry_sample.csv）')
    args = parser.parse_args()

    rows = read_csv(args.csv)
    print('读取到 %d 行' % len(rows))
    if not rows:
        print('还没实现 read_csv，请先完成 TODO 1')
        return

    with open(OUT_JSON, 'w', encoding='utf-8') as f:
        json.dump(latest_snapshot(rows), f, ensure_ascii=False, indent=2)
    with open(OUT_JSONL, 'w', encoding='utf-8') as f:
        f.write(to_history(rows))

    print('已生成 %s 和 %s' % (OUT_JSON, OUT_JSONL))
    print('检查：打开 test_data/history_sample.jsonl，与你的输出对比')


if __name__ == '__main__':
    main()