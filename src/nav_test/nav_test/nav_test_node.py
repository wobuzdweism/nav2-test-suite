#!/usr/bin/env python3
"""nav_test_node.py — TurtleBot3 Nav2 自动化测试节点
提供导航、定位、速度、障碍物生成/删除等测试基础能力。
QoS: /amcl_pose 订阅使用 TRANSIENT_LOCAL，兼容 AMCL 保留历史帧，
解决机器人静止时测试进程晚加入收不到位姿的问题。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient

# action_msgs/msg/GoalStatus 状态码 -> 名称
# 1=ACCEPTED 2=EXECUTING 3=CANCELING 4=SUCCEEDED 5=CANCELED 6=ABORTED
_STATUS_NAMES = {
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}


class NavTestNode(Node):
    def __init__(self):
        super().__init__('nav_test_node')

        # ---- 定位 ----
        # AMCL 发布 /amcl_pose 使用 transient_local，订阅必须匹配才能收到
        # 保留的历史帧（机器人静止时 AMCL 不发布新帧，晚加入的订阅者
        # 靠 QoS 拿到最后一帧）
        self._amcl_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest_pose = None
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, self._amcl_qos)

        # ---- 速度 (TC-015 到达停止 用) ----
        self.last_cmd_vel = None
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- 激光 (TC-001 用) ----
        # BEST_EFFORT 订阅可同时兼容 RELIABLE / BEST_EFFORT 的发布者
        self.latest_scan = None
        self.scan_count = 0
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))

        # ---- 里程计 (TC-002/TC-021 急停测速 用) ----
        self.latest_odom = None
        self.odom_count = 0
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ---- 导航 action client ----
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ---- 轨迹采样 (TC-019/020 绕行判定 用) ----
        self.path_samples = []
        self._sample_sub = None

    # ============ 回调 ============
    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _cmd_vel_cb(self, msg):
        self.last_cmd_vel = msg

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self.scan_count += 1

    def _odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_count += 1

    def _sample_cb(self, msg):
        p = msg.pose.pose.position
        self.path_samples.append((p.x, p.y))

    # ============ 定位 ============
    def get_pose(self):
        """返回 (x, y, yaw)，未收到位姿时返回 None"""
        if self.latest_pose is None:
            return None
        p = self.latest_pose.pose.pose.position
        q = self.latest_pose.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, yaw)

    def wait_for_pose(self, timeout=30.0):
        """等待 /amcl_pose 可用，返回是否就绪"""
        deadline = time.time() + timeout
        while self.latest_pose is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.latest_pose is not None

    def distance_to(self, x, y):
        """当前位置到 (x,y) 的距离（米）"""
        p = self.get_pose()
        if p is None:
            return float('inf')
        return math.hypot(p[0] - x, p[1] - y)

    def check_tf_chain(self, timeout=15.0):
        """检查 TF 链 map→odom→base_footprint→base_link→base_scan 是否可查,
        返回 {链段: 是否存在}（TC-003 用）"""
        from tf2_msgs.msg import TFMessage
        edges = set()

        def _cb(msg):
            for t in msg.transforms:
                p = t.header.frame_id.lstrip('/')
                c = t.child_frame_id.lstrip('/')
                if p and c:
                    edges.add((p, c))

        # /tf 为 volatile, /tf_static 为 transient_local, 分别匹配
        qos_static = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        s1 = self.create_subscription(TFMessage, '/tf', _cb, 10)
        s2 = self.create_subscription(TFMessage, '/tf_static', _cb, qos_static)
        need = [('map', 'odom'), ('odom', 'base_footprint'),
                ('base_footprint', 'base_link'), ('base_link', 'base_scan')]
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if all(e in edges for e in need):
                break
        self.destroy_subscription(s1)
        self.destroy_subscription(s2)
        result = {}
        for p, c in need:
            key = '{}->{}'.format(p, c)
            if (p, c) in edges:
                result[key] = True
            elif p == 'odom' and ('odom', 'base_link') in edges:
                result[key] = True  # 部分 URDF 直连 odom->base_link, 视为等价
            else:
                result[key] = False
        return result

    def reset_nav_state(self, x=-2.0, y=-0.5):
        """发布 initialpose 重置 AMCL 定位（只改声明，不移动物理位置）"""
        pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info('已发布 initialpose ({}, {}) x5'.format(x, y))

    # ============ 导航 ============
    def send_goal_raw(self, x, y, frame_id='map'):
        """发送导航目标但不等待完成，返回 goal handle（失败返回 None）"""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose 服务不可用')
            return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().accepted:
            self.get_logger().warn('目标未接受 ({}, {})'.format(x, y))
            return None
        return future.result()

    def navigate_to(self, x, y, timeout=120.0):
        """导航到 (x,y)，返回 (success, elapsed, status)"""
        gh = self.send_goal_raw(x, y)
        if gh is None:
            return (False, 0.0, 'REJECTED')
        start = time.time()
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        elapsed = time.time() - start
        if not result_future.done():
            self.get_logger().warn('导航超时, 尝试取消')
            cf = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cf, timeout_sec=5.0)
            return (False, elapsed, 'TIMEOUT')
        status_code = result_future.result().status if result_future.result() is not None else 0
        status = _STATUS_NAMES.get(status_code, str(status_code))
        self.get_logger().info(
            'navigate_to({}, {}) -> {} ({}s)'.format(x, y, status, elapsed))
        return (status == 'SUCCEEDED', elapsed, status)

    def navigate_sampled(self, x, y, timeout=120.0):
        """导航并记录 /amcl_pose 轨迹到 self.path_samples（TC-019/020 用）"""
        self.path_samples = []
        self._sample_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._sample_cb, self._amcl_qos)
        try:
            return self.navigate_to(x, y, timeout)
        finally:
            self.destroy_subscription(self._sample_sub)
            self._sample_sub = None

    def cancel_goal(self, gh):
        """取消指定 goal，返回是否成功"""
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cf, timeout_sec=5.0)
        return cf.result() is not None

    def navigate_and_cancel_physical(self, x, y, wait=3.0):
        """发送导航目标, wait 秒后终止任务, 并物理验证机器人停止（TC-018 用）。
        返回 (ok, status): ok=True 表示取消成功且机器人已物理停止"""
        gh = self.send_goal_raw(x, y)
        if gh is None:
            return (False, 'REJECTED')
        # 等 wait 秒让机器人开始移动
        end = time.time() + wait
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        # 取消任务
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cf, timeout_sec=5.0)
        if cf.result() is None:
            self.stop_robot()
            return (False, 'CANCEL_FAILED')
        # 操作员接管语义: 取消后补零速度确保停止
        self.stop_robot()
        # 物理验证: /odom 速度降到 < 0.05 m/s (最多 8s, 容忍惯性+取消异步)
        speed = 9.9
        deadline = time.time() + 8.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            speed = self.get_linear_speed()
            if abs(speed) < 0.05:
                break
            time.sleep(0.3)
        if abs(speed) >= 0.05:
            self.stop_robot()
            return (False, 'STILL_MOVING {:.2f}m/s'.format(speed))
        # 确认 action 终态
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=5.0)
        if rf.done() and rf.result() is not None:
            status = _STATUS_NAMES.get(rf.result().status, str(rf.result().status))
        else:
            status = 'CANCELED'
        return (status in ('CANCELED', 'SUCCEEDED'), status)

    def stop_robot(self):
        """向 /cmd_vel 发送零速度（紧急停止）"""
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info('已发送零速度指令')

    def get_linear_speed(self):
        """返回当前线速度 (m/s), 来自 /odom twist (实际运动速度, 更可靠)"""
        if self.latest_odom is None:
            return 0.0
        return self.latest_odom.twist.twist.linear.x

    # ============ 障碍物 (TC-019/020) ============
    def spawn_cylinder(self, name, x, y, radius=0.2, height=0.6):
        """在 (x,y) 生成静态圆柱障碍，返回是否成功"""
        from gazebo_msgs.srv import SpawnEntity
        cli = self.create_client(SpawnEntity, '/spawn_entity')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('spawn_entity 服务不可用')
            return False
        req = SpawnEntity.Request()
        req.name = name
        req.xml = """<sdf version='1.6'>
  <model name='{0}'>
    <static>true</static>
    <pose>{1:.2f} {2:.2f} {3:.2f} 0 0 0</pose>
    <link name='link'>
      <collision name='collision'>
        <geometry><cylinder><radius>{4:.2f}</radius><length>{5:.2f}</length></cylinder></geometry>
      </collision>
      <visual name='visual'>
        <geometry><cylinder><radius>{4:.2f}</radius><length>{5:.2f}</length></cylinder></geometry>
        <material><ambient>0.8 0.1 0.1 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>""".format(name, x, y, height / 2.0, radius, height)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = fut.result() is not None and bool(fut.result().success)
        self.get_logger().info('spawn {} -> {}'.format(name, ok))
        return ok

    def delete_obstacle(self, name):
        """删除障碍模型，返回是否成功"""
        from gazebo_msgs.srv import DeleteEntity
        cli = self.create_client(DeleteEntity, '/delete_entity')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('delete_entity 服务不可用')
            return False
        req = DeleteEntity.Request()
        req.name = name
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = fut.result() is not None and bool(fut.result().success)
        self.get_logger().info('delete {} -> {}'.format(name, ok))
        return ok

    def clear_costmaps(self):
        """清空 global/local costmap（删除障碍后清除残留 lethal 栅格）"""
        from nav2_msgs.srv import ClearEntireCostmap
        cleared = True
        for srv_name in ['/global_costmap/clear_entirely_global_costmap',
                         '/local_costmap/clear_entirely_local_costmap']:
            cli = self.create_client(ClearEntireCostmap, srv_name)
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('{} 不可用'.format(srv_name))
                cleared = False
                continue
            fut = cli.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            ok = fut.result() is not None  # ClearEntireCostmap 响应无 success 字段, 有响应即成功
            self.get_logger().info('clear {} -> {}'.format(srv_name, ok))
            cleared = cleared and ok
        return cleared
