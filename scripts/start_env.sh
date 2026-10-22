#!/usr/bin/env bash
# TurtleBot3 Nav2 测试环境一键启动
set -e

# 1. 清理残留进程
pkill -9 -f gzserver 2>/dev/null  || true; pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null  || true; pkill -9 -f component_container 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null  || true; pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f pytest 2>/dev/null  || true
sleep 3

source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

# 2. 双终端并行启动 Gazebo + Nav2 (日志落盘)
echo "启动 Gazebo + Nav2 ..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > /tmp/gazebo.log 2>&1 &
sleep 5
stdbuf -oL ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/turtlebot3_ws/maps/turtlebot3_world_map.yaml \
  use_sim_time:=true \
  params_file:=$HOME/tb3_test_ws/custom_nav2.yaml > /tmp/nav2.log 2>&1 &

# 3. 关 RViz + 等 Nav2 就绪
pkill -f rviz2 2>/dev/null  || true
sleep 20

# 4. 设置初始位姿
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}" > /dev/null
sleep 8

# 5. 等 Nav2 action 就绪 (替代固定 sleep 猜测)
echo "等待 navigate_to_pose 就绪 ..."
nav2_ready=""
for i in $(seq 1 30); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    nav2_ready=1
    break
  fi
  sleep 2
done
if [ -z "$nav2_ready" ]; then
  echo "Nav2 60s 未就绪, 请检查 /tmp/nav2.log"; exit 1
fi
echo "Nav2 已就绪"

# 6. 确认定位收敛 (transient_local QoS, 静止时也能拿到保留帧)
echo "等待 AMCL 收敛 ..."
converged=""
for i in $(seq 1 20); do
  pose=$(ros2 topic echo /amcl_pose --once \
    --qos-durability transient_local --qos-reliability reliable \
    2>/dev/null | grep -E "^\s+x:" | head -1 | awk '{print $2}')
  if [ -n "$pose" ] && awk -v p="$pose" 'BEGIN { exit !(p > -2.2 && p < -1.8) }'; then
    echo "AMCL 收敛: x=$pose (目标 -2.0)"
    converged=1
    break
  fi
  sleep 2
done
if [ -z "$converged" ]; then
  echo "AMCL 20 次未收敛, 请检查初始位姿与 /tmp/nav2.log"; exit 1
fi
echo "环境就绪, 可以运行测试"
echo "导航日志: /tmp/nav2.log  仿真日志: /tmp/gazebo.log"
