#!/usr/bin/env python3
"""pytest 共享 fixture：每条用例自动录制 rosbag 证据包。

录制话题: /odom /amcl_pose /cmd_vel /tf /scan /plan
产物路径: ~/tb3_test_ws/bags/run_<时间戳>/<用例名>/
说明:
- /scan 发布端是 BEST_EFFORT，ros2 bag record 默认 RELIABLE 订阅不兼容，
  故在 run 目录写 qos_overrides.yaml 并通过 --qos-profile-overrides-path 注入
- 用例失败/超时也保证停录（yield 后统一收尾）
- 复判入口: python3 ~/tb3_test_ws/src/nav_test/nav_test/bag_analyzer.py <run目录或bag目录>
"""
import os
import shutil
import signal
import subprocess
import time

import pytest

BAG_ROOT = os.path.join(os.path.expanduser('~'), 'tb3_test_ws', 'bags')
TOPICS = ['/odom', '/amcl_pose', '/cmd_vel', '/tf', '/scan', '/plan']

# rosbag2 QoS 覆盖: /scan 传感器数据流必须 BEST_EFFORT 才能收到
_QOS_YAML = """/scan:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
"""

_run_dir = None
_bag_ok = None  # ros2 bag 可用性缓存，整个会话只探测一次


def _ensure_run_dir():
    global _run_dir
    if _run_dir is None:
        _run_dir = os.path.join(
            BAG_ROOT, 'run_' + time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(_run_dir, exist_ok=True)
        with open(os.path.join(_run_dir, 'qos_overrides.yaml'), 'w') as f:
            f.write(_QOS_YAML)
    return _run_dir


def _rosbag_available():
    global _bag_ok
    if _bag_ok is None:
        _bag_ok = shutil.which('ros2') is not None and subprocess.call(
            ['ros2', 'bag', '--help'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL) == 0
    return _bag_ok


@pytest.fixture(autouse=True)
def record_bag(request):
    """每条用例自动录制 rosbag；无论用例结果如何都优雅停录"""
    run_dir = _ensure_run_dir()
    bag_dir = os.path.join(run_dir, request.node.name)
    proc = None
    if _rosbag_available():
        proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_dir,
             '--qos-profile-overrides-path',
             os.path.join(run_dir, 'qos_overrides.yaml')] + TOPICS,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        print('[bag] ros2 bag 不可用, 跳过录制: ' + request.node.name)
    yield
    if proc is not None:
        # SIGINT 让 rosbag2 优雅关闭并落盘 metadata.yaml
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        if os.path.isdir(bag_dir) and os.listdir(bag_dir):
            print('[bag] 已保存: ' + bag_dir)
