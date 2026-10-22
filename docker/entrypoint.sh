#!/usr/bin/env bash
# 容器内一键拉起 Gazebo(headless) + Nav2 + AMCL, 就绪后执行传入命令
# 逻辑与宿主机 scripts/start_env.sh 一致, 路径改为容器内固化路径
set -e

# 1. 清理残留进程
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f component_container 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 3

# 2. 环境
source /opt/ros/humble/setup.bash
source /opt/nav_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

# 3. 启动 Gazebo(headless) + Nav2 (日志落盘)
echo "[sut] 启动 Gazebo + Nav2 ..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > /tmp/gazebo.log 2>&1 &
sleep 5
stdbuf -oL ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=/opt/nav_env/map/turtlebot3_world_map.yaml \
  use_sim_time:=true \
  params_file:=/opt/nav_env/custom_nav2.yaml > /tmp/nav2.log 2>&1 &
sleep 20
pkill -f rviz2 2>/dev/null || true

# 4. 设置初始位姿
echo "[sut] 设置初始位姿 (-2.0, -0.5) ..."
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}" > /dev/null
sleep 8

# 5. 等 Nav2 action 就绪
echo "[sut] 等待 navigate_to_pose 就绪 ..."
ready=""
for i in $(seq 1 30); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then ready=1; break; fi
  sleep 2
done
if [ -z "$ready" ]; then
  echo "[sut] Nav2 60s 未就绪, /tmp/nav2.log 尾部:"
  tail -30 /tmp/nav2.log
  exit 1
fi

# 6. 确认 AMCL 收敛 (transient_local QoS, 静止时也能拿到保留帧)
echo "[sut] 等待 AMCL 收敛 ..."
conv=""
for i in $(seq 1 20); do
  pose=$(ros2 topic echo /amcl_pose --once \
    --qos-durability transient_local --qos-reliability reliable \
    2>/dev/null | grep -E "^\s+x:" | head -1 | awk '{print $2}')
  if [ -n "$pose" ] && awk -v p="$pose" 'BEGIN { exit !(p > -2.2 && p < -1.8) }'; then
    conv=1; break
  fi
  sleep 2
done
if [ -z "$conv" ]; then
  echo "[sut] AMCL 未收敛, /tmp/nav2.log 尾部:"
  tail -30 /tmp/nav2.log
  exit 1
fi

echo "[sut] 环境就绪, 执行: $*"
exec "$@"
