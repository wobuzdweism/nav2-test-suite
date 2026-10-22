#!/usr/bin/env python3
import math
import time
import rclpy
import pytest
from nav_test.nav_test_node import NavTestNode
from geometry_msgs.msg import Twist


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = NavTestNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_tc001_scan_published(node):
    """TC-001：/scan 正常发布（10s 内 ≥5 帧，角度范围 ≥180°）"""
    base = node.scan_count
    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan_count - base >= 5:
            break
    got = node.scan_count - base
    assert got >= 5, '/scan 帧数不足: {}'.format(got)
    s = node.latest_scan
    total_angle = math.degrees(s.angle_max - s.angle_min)
    assert total_angle >= 180.0, 'scan 角度范围不足: {:.1f}°'.format(total_angle)
    node.get_logger().info(
        'TC-001 通过: /scan 正常, 角度范围 {:.1f}°, {} 帧'.format(total_angle, got))


def test_tc002_odom_published(node):
    """TC-002：/odom 正常发布（10s 内 ≥10 帧，位置数据合理）"""
    base = node.odom_count
    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_count - base >= 10:
            break
    got = node.odom_count - base
    assert got >= 10, '/odom 帧数不足: {}'.format(got)
    p = node.latest_odom.pose.pose.position
    assert math.isfinite(p.x) and math.isfinite(p.y), '/odom 位置数据异常'
    node.get_logger().info(
        'TC-002 通过: /odom 正常, 位置 ({:.2f}, {:.2f})'.format(p.x, p.y))


def test_tc003_tf_tree(node):
    """TC-003：/tf 树完整（map→odom→base_link→base_scan 可查）"""
    result = node.check_tf_chain(timeout=15.0)
    missing = [k for k, v in result.items() if not v]
    assert not missing, 'TF 缺失: {}'.format(missing)
    node.get_logger().info('TC-003 通过: /tf 树完整 (map→odom→base_link→base_scan)')


def test_tc004_cmd_vel_roundtrip(node):
    """TC-004：/cmd_vel 双向通信（发布零速度，3s 内订阅端收到）"""
    node.last_cmd_vel = None
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.last_cmd_vel is not None:
            break
    pub.destroy()
    assert node.last_cmd_vel is not None, '/cmd_vel 3s 内未收到消息'
    node.get_logger().info('TC-004 通过: /cmd_vel 双向通信正常')

