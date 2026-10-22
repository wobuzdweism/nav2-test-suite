#!/usr/bin/env python3
import time
import rclpy
import pytest
from nav_test.nav_test_node import NavTestNode


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = NavTestNode()
    assert n.wait_for_pose(timeout=30.0), '定位不可用, 请确认环境已启动且位姿正确'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_tc024_tf_tree_complete(node):
    """TC-024: TF 树完整性验证 (map->odom->base_footprint->base_link->base_scan)

    基于话题内容验证(绕开时间源匹配问题): 收集 /tf + /tf_static 的所有帧和
    父子关系, 断言关键帧存在且 map 可达 base_scan。
    """
    import rclpy
    from collections import deque
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    frames = set()
    edges = set()  # (parent, child)

    def _tf_cb(msg):
        for t in msg.transforms:
            p = t.header.frame_id.lstrip('/')
            c = t.child_frame_id.lstrip('/')
            if p and c:
                frames.add(p)
                frames.add(c)
                edges.add((p, c))

    # /tf 为 volatile, /tf_static 为 transient_local, 分别匹配
    qos_static = QoSProfile(
        depth=10, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub_tf = node.create_subscription(TFMessage, '/tf', _tf_cb, 50)
    sub_tf_static = node.create_subscription(TFMessage, '/tf_static', _tf_cb, qos_static)

    # 固定采集窗口, 不能提前 break: base_footprint/base_scan 来自
    # transient_local 的静态变换, 秒级即可收到; 一旦 break 就停止采集, 而
    # odom->base_footprint 由 Gazebo 差速插件高频发在易失 /tf 上, depth=10
    # 缓冲加上 spin 间隙(sleep 0.5s)丢帧会把这一环漏掉, 造成 "TF 链断裂"
    # 的假失败 (本轮实测边集合恰好只缺 odom->base_footprint)
    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(sub_tf)
    node.destroy_subscription(sub_tf_static)

    # 断言关键帧存在
    for f in ['map', 'odom', 'base_footprint', 'base_link', 'base_scan']:
        assert f in frames, 'TF 缺失帧: {} (已有: {})'.format(f, sorted(frames))

    # BFS 验证 map -> base_scan 可达 (变换链完整)
    q = deque(['map'])
    visited = {'map'}
    reachable = False
    while q:
        cur = q.popleft()
        if cur == 'base_scan':
            reachable = True
            break
        for p, c in edges:
            if p == cur and c not in visited:
                visited.add(c)
                q.append(c)
    assert reachable, 'TF 树中 map 无法到达 base_scan (边: {})'.format(sorted(edges))

    node.get_logger().info(
        'TC-024 通过: TF 树完整 (map->odom->base_footprint->base_link->base_scan), 共 {} 帧, {} 条边'.format(
            len(frames), len(edges)))
def test_tc025_custom_params_loaded(node):
    """TC-025: 验证 custom_nav2.yaml 关键参数已加载生效"""
    from rcl_interfaces.srv import ListParameters, GetParameters
    from rcl_interfaces.msg import ParameterType

    def _list_names(srv_base, keyword):
        cli = node.create_client(ListParameters, '/{}/list_parameters'.format(srv_base))
        assert cli.wait_for_service(timeout_sec=5.0), '{}/list_parameters 不可用'.format(srv_base)
        req = ListParameters.Request()  # Humble 无 deep 字段, 默认返回全量参数
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        assert fut.result() is not None, '读取 {} 参数列表失败'.format(srv_base)
        return [n for n in fut.result().result.names if keyword in n]

    def _get_values(srv_base, names):
        cli = node.create_client(GetParameters, '/{}/get_parameters'.format(srv_base))
        assert cli.wait_for_service(timeout_sec=5.0), '{}/get_parameters 不可用'.format(srv_base)
        req = GetParameters.Request()
        req.names = names
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        assert fut.result() is not None, '读取 {} 参数失败'.format(srv_base)
        return fut.result().values

    def _val(v):
        if v.type == ParameterType.PARAMETER_BOOL:
            return v.bool_value
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return v.double_value
        if v.type == ParameterType.PARAMETER_INTEGER:
            return v.integer_value
        if v.type == ParameterType.PARAMETER_STRING:
            return v.string_value
        return None

    # 1. goal_checker 容差: 应是我们配置的 0.15m
    tols = _list_names('controller_server', 'xy_goal_tolerance')
    assert tols, 'controller_server 未加载 xy_goal_tolerance 参数'
    tvals = _get_values('controller_server', tols)
    # 多个 goal_checker (默认 0.05 + 我们配置 0.15), 验证 0.15 已加载即配置生效
    assert any(abs(_val(v) - 0.15) < 1e-6 for v in tvals), \
        'xy_goal_tolerance 未按配置生效: {}'.format([_val(v) for v in tvals])

    # 2. goal_checker 状态判定: stateful 应为 False (我们配置的非状态锁定)
    states = _list_names('controller_server', 'stateful')
    assert states, 'controller_server 未加载 stateful 参数'
    svals = _get_values('controller_server', states)
    assert all(_val(v) is False for v in svals), \
        'stateful 未按配置生效: {}'.format([_val(v) for v in svals])

    node.get_logger().info(
        'TC-025 通过: xy_goal_tolerance=0.15, stateful=False 已生效 '
        '(参数名: {})'.format(tols + states))


def test_tc014_unreachable_goal(node):
    """TC-014：目标不可达（墙内），任务失败并上报，不崩溃

    注意：本条用例原先在 test_nav_advanced.py 的最前面，是该文件第一条执行的
    用例。实测(2026-10-16)：干净环境里单独跑 test_nav_basic.py(不含本条) 时
    TC-013/015/018 全过；而单独跑 test_nav_advanced.py(本条在最前) 时，本条
    之后需要导航的 TC-016/TC-017 全部失败(未开始移动 / TIMEOUT)。
    根因：向墙内 (-6,-6) 发不可达目标会触发恢复行为(spin/backup)，机器人在
    墙边原地打转、激光视野被墙占满，AMCL 失去有效观测后位姿发散，此后整批
    导航全部失败；本条自身却因为"期望失败"而判定通过。
    因此把它移到整个套件的最后执行(本文件按文件名字母序是最后一个)，让它
    的污染不再影响其它用例。它本身不受机器人起始位置影响，顺序调整安全。
    """
    success, elapsed, status = node.navigate_to(-6.0, -6.0, timeout=60.0)
    assert not success, '不可达目标不应成功'
    assert status in ('ABORTED', 'TIMEOUT', 'CANCELED'), \
        '状态异常: {}'.format(status)
    node.get_logger().info(
        'TC-014 通过: 不可达目标返回失败状态 {} ({}s)'.format(status, elapsed))
