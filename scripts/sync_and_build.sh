#!/usr/bin/env bash
# 一键同步+编译: 把共享文件夹里修复好的测试代码拷进 ~/tb3_test_ws 并编译
# 用法: bash "/mnt/hgfs/VMware_share/sync_and_build.sh"
set -e

SRC="/mnt/hgfs/VMware_share/tb3_test_ws/src/nav_test"
DST="$HOME/tb3_test_ws/src/nav_test"

if [ ! -f "$HOME/tb3_test_ws/src/nav_test/setup.py" ]; then
  echo "错误: $HOME/tb3_test_ws/src/nav_test 包不存在 (setup.py 缺失)"
  exit 1
fi

mkdir -p "$DST/nav_test" "$DST/test"
cp -v "$SRC/nav_test/nav_test_node.py" "$DST/nav_test/"
cp -v "$SRC/nav_test/bag_analyzer.py" "$DST/nav_test/"
cp -v "$SRC/test/"*.py "$DST/test/"

cd "$HOME/tb3_test_ws"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
export TURTLEBOT3_MODEL=burger
colcon build --packages-select nav_test
source install/setup.bash

echo ""
echo "=========================================="
echo " 同步+编译完成! 下一步跑测试:"
echo " cd ~/tb3_test_ws && python3 -m pytest src/nav_test/test/ -v -s"
echo "=========================================="
