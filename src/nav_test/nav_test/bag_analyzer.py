#!/usr/bin/env python3
"""bag_analyzer.py — rosbag 离线分析（回归/复判用）

用法:
  python3 bag_analyzer.py <bag目录> [更多目录...]
  python3 bag_analyzer.py <run目录>     # 自动分析其下所有子 bag
  示例:
    python3 bag_analyzer.py ~/tb3_test_ws/bags/run_20261022_103000

输出（每个 bag）:
  时长、各话题消息数、/odom 位移幅值、路径长、弯曲度(路径长/位移)、最大速度
  并在 bag 目录内写 analysis.json

判定约定（与项目一致）:
  - odom 与 map 存在固定旋转差，只输出旋转平移不变量（幅值/弯曲度），
    不做方向断言
  - 弯曲度: 直线导航≈1.0，绕行>1.15，位移≈0 视为未移动
"""
import json
import math
import os
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def _open_reader(bag_dir):
    """兼容 sqlite3 / mcap / 自动探测三种存储后端"""
    for sid in ('sqlite3', '', 'mcap'):
        try:
            r = SequentialReader()
            r.open(StorageOptions(uri=bag_dir, storage_id=sid),
                   ConverterOptions('cdr', 'cdr'))
            return r
        except Exception:
            continue
    return None


def analyze_bag(bag_dir):
    reader = _open_reader(bag_dir)
    if reader is None:
        return {'bag': os.path.basename(bag_dir), 'error': '无法打开 bag'}

    path = []
    counts = {}
    max_speed = 0.0
    t0 = t1 = None
    while reader.has_next():
        topic, data, ts = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if t0 is None:
            t0 = ts
        t1 = ts
        if topic == '/odom':
            try:
                odom = deserialize_message(data, Odometry)
            except Exception:
                continue
            p = odom.pose.pose.position
            path.append((p.x, p.y))
            v = abs(odom.twist.twist.linear.x)
            if v > max_speed:
                max_speed = v

    duration = (t1 - t0) / 1e9 if t0 is not None else 0.0
    displacement = path_length = 0.0
    curvature = None
    if len(path) >= 2:
        displacement = math.hypot(path[-1][0] - path[0][0],
                                  path[-1][1] - path[0][1])
        path_length = sum(
            math.hypot(path[i + 1][0] - path[i][0],
                       path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1))
        if displacement > 1e-6:
            curvature = round(path_length / displacement, 3)

    result = {
        'bag': os.path.basename(bag_dir),
        'duration_s': round(duration, 1),
        'odom_samples': len(path),
        'messages': counts,
        'displacement_m': round(displacement, 3),
        'path_length_m': round(path_length, 3),
        'curvature': curvature,
        'max_speed_ms': round(max_speed, 3),
    }
    with open(os.path.join(bag_dir, 'analysis.json'), 'w') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    return result


def _iter_bags(roots):
    """入参是 bag(含 metadata.yaml) 就直接用，否则扫一层子目录"""
    for root in roots:
        if os.path.isfile(os.path.join(root, 'metadata.yaml')):
            yield root
            continue
        if not os.path.isdir(root):
            print('跳过(不存在): ' + root)
            continue
        for name in sorted(os.listdir(root)):
            sub = os.path.join(root, name)
            if os.path.isfile(os.path.join(sub, 'metadata.yaml')):
                yield sub


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    rows = [analyze_bag(b) for b in _iter_bags(argv)]
    print()
    print('{:<38} {:>8} {:>10} {:>10} {:>9} {:>8}'.format(
        'bag', '时长s', '位移m', '路径m', '弯曲度', '峰值v'))
    for r in rows:
        if 'error' in r:
            print('{:<38} ERROR {}'.format(r['bag'], r['error']))
            continue
        print('{:<38} {:>8} {:>10} {:>10} {:>9} {:>8}'.format(
            r['bag'][:38], r['duration_s'], r['displacement_m'],
            r['path_length_m'],
            str(r['curvature']) if r['curvature'] is not None else '-',
            r['max_speed_ms']))
    print()
    print('判定参考: 位移<0.3 未移动 | 弯曲度≈1.0 直线 | >1.15 绕行')
    print('详细数据已写入各 bag 目录的 analysis.json')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
