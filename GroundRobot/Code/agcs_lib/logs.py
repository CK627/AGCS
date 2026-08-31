#!/usr/bin/python3
# coding=utf8
"""统一日志：基于 Python logging，按 模块名 分目录、按运行时刻分文件。

日志文件结构（机器人端）：
    /home/pi/spiderpi/logs/
      └── 2026-8-31/            # 日期目录（随当天而定）
            ├── auto_fetch/     # 入口程序名
            │     └── 13-56.log # 运行时刻（时-分）
            ├── search/         # 库模块各自独立目录
            │     └── 13-56.log
            ├── grab/
            ├── tracker/
            └── task_server/    # 推流/任务服务：只写文件，不进终端

用法：
    from agcs_lib.logs import setup_logger, get_logger
    setup_logger('auto_fetch')      # 入口调用一次：文件 + 终端
    log = get_logger('search')      # 库模块：自动建 日期/模块名/时刻.log
    log.info('...')

结构化消息（模块 + 进度 + 原因 + 动作）：
    log.info('[search] %s', action_msg('未检测到目标', action='舵机24调整 200->300'))
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


def _module_file(log_root, name):
    """返回 (模块日志目录, 日志文件路径)：日期/模块名/时-分.log。"""
    now = datetime.now()
    date_dir = '%d-%d-%d' % (now.year, now.month, now.day)   # 2026-8-31
    time_name = now.strftime('%H-%M')                        # 13-56
    sub_dir = os.path.join(log_root, date_dir, name)
    return sub_dir, os.path.join(sub_dir, '%s.log' % time_name)


def _ensure_handlers(logger, log_root, console):
    """给 logger 装配文件 handler（可选终端 handler）；幂等。"""
    if logger.handlers:
        return logger
    file_ok = False
    try:
        sub_dir, log_file = _module_file(log_root, logger.name)
        os.makedirs(sub_dir, exist_ok=True)
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        fh = logging.FileHandler(log_file, encoding='utf-8')
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(_FMT)
        logger.addHandler(fh)
        file_ok = True
    except OSError:
        pass  # 日志目录不可写（如本机测试）：降级处理，不影响主程序
    if console:
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(_FMT)
        logger.addHandler(ch)
    if not file_ok and not console:
        logger.addHandler(logging.NullHandler())
    return logger


def setup_logger(name='agcs', log_root=None, console=True):
    """入口程序调用一次：配置 文件（日期/模块名/时刻.log）+ 终端 双输出。"""
    return _ensure_handlers(logging.getLogger(name),
                            log_root or _DEFAULT_LOG_ROOT, console=console)


def get_logger(name='agcs', log_root=None, console=False):
    """库模块获取 logger；首次调用自动创建 日期/模块名/时刻.log。

    console=False 表示只写文件不进终端（如 task_server 推流日志）。
    """
    return _ensure_handlers(logging.getLogger(name),
                            log_root or _DEFAULT_LOG_ROOT, console=console)
