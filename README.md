# nav2_test_suite — ROS2 机器人导航系统测试套件

针对 TurtleBot3 (burger) + Gazebo Classic + Nav2 + AMCL 仿真环境的系统性导航测试项目：
45 条功能/安全/系统级测试用例设计执行，16 条 pytest 自动化用例全绿，定位 2 个真实缺陷，
并集成 rosbag2 证据录制与离线指标复判。

> 本仓库为纯测试侧工程：不修改被测系统（Nav2/TB3），只观测、判定、取证。

## 被测系统（SUT）

| 组件 | 版本/说明 |
|---|---|
| OS / 中间件 | Ubuntu 22.04 / ROS2 Humble (DDS) |
| 仿真 | Gazebo Classic 11 + turtlebot3_simulations |
| 导航栈 | Nav2（planner/controller/behavior/amcl/map_server，自定义参数） |
| 机器人 | TurtleBot3 burger（差速底盘，单线激光） |
| 测试框架 | pytest + rclpy（action client / topic / TF） |

## 测试设计

45 条用例按五个维度覆盖（详见 [docs/test_cases.xlsx](docs/test_cases.xlsx)）：

1. **通信层**：话题/action 数据完整性、QoS、频率（TC-001~）
2. **功能层**：单点导航、重复目标、取消、抢占、目标切换（TC-013~）
3. **避障层**：静态/动态障碍绕行、障碍增删后恢复（TC-019~）
4. **安全层**：急停、限速、不可达目标、异常输入（TC-021~）
5. **系统层**：Nav2 进程重启韧性、并发导航抢占、TF 树完整性、长时间运行（TC-029~）

结果：**通过 40 / 失败 2（真实缺陷）/ 未有效验证 3（VM 算力受限，附日志证据）**。

## 框架结构（两层）

```
nav_test_node.py   封装层：导航 action 客户端、位姿/速度/激光订阅、
                   轨迹采样、障碍物生成/删除、Nav2 进程级重启、
                   状态复位（取消目标/重发 initialpose/清 costmap）
test/test_*.py     用例层：6 个文件 16 条用例，每条只写"步骤 + 断言"
test/conftest.py   autouse fixture：每条用例自动录制 rosbag 证据包
nav_test/bag_analyzer.py  离线分析：从 bag 的 /odom 计算位移幅值/路径长/
                   弯曲度/峰值速度，写 analysis.json
```

## 关键工程决策（踩坑实录）

**1. 到达判定不用 AMCL 直读，用 odom 相对位移 + 非对称门限**
AMCL 稳态读数普遍偏离真实位置 0.2-0.37m 且方向随机。位移断言两端各带
0.15m 容差 + 随机偏置，差分直接叠加进读数——±0.3m 对称窗必然偶发误杀。
最终方案：odom 轨迹首末差值对比预期幅值，非对称门限 0.3-1.8m
（下限拦截"没走过去"类假成功，上限拦截失控）。

**2. 位移方向断言全部作废，改用旋转平移不变量**
本环境 odom 与 map 存在约 55° 固定旋转差（TB3 默认 spawn 朝向与
initialpose yaw 的差值经 AMCL map→odom 修正补偿）。任何"odom 方向 vs
map 预期方向"的判定全部无效。所有指标只用不变量：位移幅值与弯曲度
（路径长/位移幅值，直线 ≈1.0，绕行 >1.15）。

**3. 失败注入用例必须排在套件最后（A/B 实验定位）**
TC-014 向墙内发不可达目标，触发恢复行为使 AMCL 失去有效观测、位姿发散，
其后所有导航用例连锁失败——而它自身因"期望失败"反而判通过。
用 A/B 实验坐实污染链后，将其移至套件末尾隔离。

**4. rosbag2 证据链：录制与断言解耦**
每条用例 autouse fixture 自动录制 /odom /amcl_pose /cmd_vel /tf /scan /plan
（/scan 发布端为 BEST_EFFORT，需注入 QoS 覆盖）。指标从 bag 离线计算，
不依赖 live 时序窗口，失败可回放复判。实际输出：

```
bag                              时长s   位移m   路径m   弯曲度  峰值v
test_tc013_single_goal_reached    12.9   0.76   0.765   1.007   0.22
test_tc018_cancel_stops_robot      4.7   0.441  0.476   1.081   0.225
```

## 定位的两个真实缺陷

| 编号 | 现象 | 根因 |
|---|---|---|
| TC-028 | 对物理不可达的围困目标，Nav2 误报 SUCCEEDED | 未建图障碍导致 AMCL 定位漂移，goal_checker 未复核物理位置即判达标 |
| TC-033 | 障碍紧贴目标点时导航无超时终止，机器人贴障死等 | 进度检查器与目标容差耦合，缺乏不可达场景的终止机制 |

## 环境可复现性

仿真环境曾因虚拟机快照回滚发生五处丢失（hgfs 挂载/源码包/编译产物/
Gazebo 在线模型缓存/地图文件），逐项定位并修复后全量重编译恢复 16 条全绿。
修复过程沉淀为一键脚本（scripts/）与诊断命令清单，详见 commit 历史。

## 运行方式

```bash
# 终端1：启动 Gazebo + Nav2，等待"环境就绪"
bash scripts/start_env.sh

# 终端2：同步编译 + 跑全量自动化
bash scripts/sync_and_build.sh
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/tb3_test_ws/install/setup.bash
cd ~/tb3_test_ws && python3 -m pytest src/nav_test/test/ -v -s

# 离线分析本轮 bag 证据
python3 src/nav_test/nav_test/bag_analyzer.py ~/tb3_test_ws/bags/run_<时间戳>
```

## 目录结构

```
src/nav_test/          ROS2 Python 测试包（封装层 + 用例层 + 录包 fixture）
config/custom_nav2.yaml  被测 Nav2 参数（goal_checker/costmap/速度限制等）
scripts/               环境启动与一键同步编译脚本
docs/test_cases.xlsx   45 条用例设计表（步骤/判定/结果/根因备注）
.github/workflows/     CI：编译检查 + 语法级 lint + 用例收集
```
