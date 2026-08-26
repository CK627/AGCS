#!/usr/bin/python3
# coding=utf8
"""统一日志：基于 Python logging，固定根目录 /home/pi/spiderpi/logs，按日期分目录。

日志文件结构：
    /home/pi/spiderpi/logs/
      └── 2026-8-27/
            └── 03-49.log      # 运行时刻（时-分）

用法：
    from agcs_lib.logs import setup_logger, get_logger
    logger = setup_logger()          # 入口处调用一次
    logger.info('开始')
    logger.debug('详细原因')          # debug 只进文件，不打印终端

结构化消息（模块 + 进度 + 原因 + 动作）：
    logger.info('[search] %s', action_msg('未检测到目标', action='舵机24调整 200->300'))
"""
import logging
import os
from datetime import datetime

_DEFAULT_LOG_ROOT = '/home/pi/spiderpi/logs'

_FMT = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s',
                         datefmt='%H:%M:%S')


def action_msg(progress, reason=None, action=None):
    """拼装结构化日志消息：进度 由于[原因] 执行：[动作]。

    reason / action 可省略，省略的部分不输出。
    """
    parts = [str(progress)]
    if reason:
        parts.append('由于%s' % reason)
    if action:
        parts.append('执行：%s' % action)
    return ' '.join(parts)


def setup_logger(name='agcs', log_root=None):
    """配置并返回 logger；文件写 日志根/日期/时间.log，控制台输出 info 及以上。

    重复调用幂等（已有 handler 时直接返回）。
    """
    if log_root is None:
        log_root = _DEFAULT_LOG_ROOT

    now = datetime.now()
    date_dir = '%d-%d-%d' % (now.year, now.month, now.day)   # 2026-8-27
    time_name = now.strftime('%H-%M')                        # 03-49
    sub_dir = os.path.join(log_root, date_dir)
    os.makedirs(sub_dir, exist_ok=True)
    log_file = os.path.join(sub_dir, '%s.log' % time_name)

    logger = logging.getLogger(name)
    if logger.handlers:          # 已配置过，避免重复加 handler
        return logger
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    fh = logging.FileHandler(log_file, encoding='utf-8')
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(_FMT)
    logger.addHandler(fh)

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(_FMT)
    logger.addHandler(ch)

    return logger


def get_logger(name='agcs'):
    """获取 logger；若尚未 setup，返回带 NullHandler 的 logger（不报错）。"""
    logger = logging.getLogger(name)
    if not logger.handlers:
        logger.addHandler(logging.NullHandler())
    return logger
