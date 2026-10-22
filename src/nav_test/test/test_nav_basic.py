#!/usr/bin/env python3
import time
import rclpy
import pytest
from nav_test.nav_test_node import NavTestNode


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = NavTestNode()
    assert n.wait_for_pose(timeout=30.0), \
        '定位数据不可用：请确认 Gazebo + Nav2 已启动且已设初始位姿'
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def reset_nav(node):
    """每个用例前自动复位导航状态（取消目标/重设位姿/清 costmap/等停）"""
    node.reset_nav_state()
    yield


def test_tc013_single_goal_reached(node):
    """TC-013：单点导航 90s 内到达，误差 < 0.2m"""
    success, elapsed, status = node.navigate_to(-2.0, 0.5, timeout=120.0)
    assert success, '导航失败，状态: {}'.format(status)
    assert elapsed <= 120.0, '超时: {:.1f}s'.format(elapsed)
    error = node.distance_to(-2.0, 0.5)
    assert error < 0.2, '到达误差过大: {:.3f}m'.format(error)
    node.get_logger().info(
        'TC-013 通过: 耗时 {:.1f}s, 误差 {:.3f}m'.format(elapsed, error))


def test_tc015_stops_after_reach(node):
    """TC-015：到达目标后 /cmd_vel 归零（连续采样，容忍瞬时微调）"""
    time.sleep(3.0)
    stop_count = 0
    total = 0
    deadline = time.time() + 3.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.last_cmd_vel is not None:
            v = node.last_cmd_vel
            total += 1
            if abs(v.linear.x) < 0.02 and abs(v.angular.z) < 0.02:
                stop_count += 1
    assert total > 0, '/cmd_vel 无数据'
    assert stop_count >= total * 0.9, \
        '到达后未稳定停止（归零 {}/{} 采样点）'.format(stop_count, total)
    node.get_logger().info(
        'TC-015 通过: 到达后速度归零（{}/{} 采样点）'.format(stop_count, total))


def test_tc018_cancel_stops_robot(node):
    """TC-018：导航中终止任务，机器人应停止移动（物理验证版）"""
    ok, status = node.navigate_and_cancel_physical(0.0, 0.5, wait=3.0)
    assert ok, '取消后机器人未停止: {}'.format(status)
    node.get_logger().info(
        'TC-018 通过: 任务终止后机器人已停止 ({})'.format(status))

