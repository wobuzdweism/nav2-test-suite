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


def _wait_moving(node, timeout_s=75.0):
    """轮询等机器人开始移动; VM 规划阶段实测可达 60s+, 窗口须放宽"""
    end = time.time() + timeout_s
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_linear_speed() > 0.05:
            return True
        time.sleep(0.8)
    return False


def test_tc021_emergency_stop_during_navigation(node):
    """TC-021: 导航中急停, 机器人停止且 Nav2 不崩溃"""
    # 先就位到 (-2,0.5): 上一条用例结束在 (-1,0.5) 附近, AMCL 偏置可达
    # 0.37, 会使 0.5m 外的目标 (-0.5,0.5) 被 Nav2 秒判到达(0.15容差内)
    # 导致机器人永不移动; 就位后距目标 1.5m, 彻底避开秒到达陷阱
    ok, t, s = node.navigate_to(-2.0, 0.5, timeout=120.0)
    assert ok, '就位导航失败: {}'.format(s)
    time.sleep(2.0)

    gh = node.send_goal_raw(-0.5, 0.5)
    assert gh is not None, '导航目标未被接受'

    moving = _wait_moving(node)
    if not moving:
        # 前一条用例刚 clear costmap 就发目标会规划卡死: 取消后清 costmap 重试
        node.get_logger().warn('第一次未开始移动, 取消后清 costmap 重试')
        node.cancel_goal(gh)
        node.clear_costmaps()
        time.sleep(5.0)
        gh = node.send_goal_raw(-0.5, 0.5)
        assert gh is not None, '重试目标未被接受'
        moving = _wait_moving(node)
    assert moving, '机器人未开始移动, 急停测试前提不成立'

    gh.cancel_goal_async()

    deadline = time.time() + 20.0
    speed = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        speed = node.get_linear_speed()
        if speed < 0.01:
            break
        time.sleep(0.5)
    node.stop_robot()
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.2)
    assert node.get_linear_speed() < 0.01, '急停后速度未归零: {}'.format(node.get_linear_speed())

    # 急停后 Nav2 健康检查: 新目标须被接受 (取 1.5m 外目标, 避免秒判到达)
    gh2 = node.send_goal_raw(-2.0, 0.5)
    assert gh2 is not None, '急停后 Nav2 不接受新目标'
    node.cancel_goal(gh2)
    node.stop_robot()
    node.get_logger().info(
        'TC-021 通过: 急停后速度 {:.3f} m/s, 取消成功, Nav2 可接受新目标'.format(
            node.get_linear_speed()))
    time.sleep(2.0)  # 让 cancel 终态落定, 避免下一条用例发目标被抢占干扰


def test_tc022_sensor_failure_during_navigation(node):
    """TC-022: 导航中感知失效(禁用 obstacle layer 模拟激光中断) -> 系统不崩溃, 恢复后正常"""
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

    # 说明: 本环境激光由 Gazebo 插件发布(无独立节点可杀), 用禁用 costmap
    #       obstacle layer 模拟"激光感知失效"的系统级等价效果

    # 1. 发送导航目标(靠墙定位可靠点; TC-021 结束于 (-0.5,0.5) 附近,
    #    距目标 (-2,0.5) 约 1.5m, AMCL 偏置下仍有 >1.1m 移动过程)
    gh = node.send_goal_raw(-2.0, 0.5)
    assert gh is not None, '导航目标未被接受'

    moving = _wait_moving(node)
    if not moving:
        # VM 慢: 上一用例刚 cancel 过目标立即发新目标可能卡住, 取消后重试一次
        node.get_logger().warn('第一次未开始移动, 取消后清 costmap 重试')
        node.cancel_goal(gh)
        node.clear_costmaps()
        time.sleep(5.0)
        gh = node.send_goal_raw(-2.0, 0.5)
        assert gh is not None, '重试目标未被接受'
        moving = _wait_moving(node)
    assert moving, '机器人未开始移动, 感知失效测试前提不成立'

    # 3. 禁用 global+local costmap 的 obstacle layer(模拟激光数据中断)
    cli = node.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
    cli2 = node.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
    assert cli.wait_for_service(timeout_sec=5.0), 'local set_parameters 不可用'
    assert cli2.wait_for_service(timeout_sec=5.0), 'global set_parameters 不可用'
    p = Parameter()
    p.name = 'obstacle_layer.enabled'
    p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
    req = SetParameters.Request()
    req.parameters = [p]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    fut2 = cli2.call_async(req)
    rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
    assert fut.result() is not None and fut2.result() is not None, '禁用 obstacle layer 失败'
    node.get_logger().info('已禁用 global+local obstacle layer (模拟激光中断)')
    time.sleep(2.0)

    # 4. 感知失效期间: 验证 Nav2 不崩溃(action 可取消 + 可接受新目标)
    cancel_ok = node.cancel_goal(gh)
    assert cancel_ok, '感知失效期间 Nav2 取消失败, 可能已崩溃'
    gh2 = node.send_goal_raw(-1.0, 0.5)
    assert gh2 is not None, '感知失效期间 Nav2 无法接受新目标, 可能已崩溃'
    node.cancel_goal(gh2)
    node.get_logger().info('感知失效期间 Nav2 响应正常(可取消/可接受新目标)')

    # 5. 恢复 obstacle layer(模拟激光恢复)
    p.value.bool_value = True
    req3 = SetParameters.Request()
    req3.parameters = [p]
    fut3 = cli.call_async(req3)
    rclpy.spin_until_future_complete(node, fut3, timeout_sec=5.0)
    fut4 = cli2.call_async(req3)
    rclpy.spin_until_future_complete(node, fut4, timeout_sec=5.0)
    assert fut3.result() is not None and fut4.result() is not None, '恢复 obstacle layer 失败'
    node.get_logger().info('已恢复 obstacle layer')
    time.sleep(3.0)

    # 6. 恢复后验证导航功能正常: 导航到左上角靠墙点
    ok, elapsed, status = node.navigate_to(-2.0, 0.5, timeout=120.0)
    assert ok, '恢复感知后导航失败: {}'.format(status)
    node.get_logger().info('TC-022 通过: 恢复后导航正常 ({}s)'.format(elapsed))
