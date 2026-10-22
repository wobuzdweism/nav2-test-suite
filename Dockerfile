# SUT 镜像: Gazebo(headless) + Nav2 + TurtleBot3 + 测试套件 一体化
# 构建: docker build -t nav2-test-sut .
# 运行: docker run --rm nav2-test-sut            (默认跑全量 pytest)
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-turtlebot3 \
      ros-humble-turtlebot3-msgs \
      python3-colcon-common-extensions \
      python3-pytest \
      git \
      procps \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3 仿真包 (构建时从官方 humble 分支拉取)
RUN mkdir -p /opt/tb3_ws/src \
 && git clone --depth 1 -b humble \
      https://github.com/ROBOTIS-Git/turtlebot3_simulations.git \
      /opt/tb3_ws/src/turtlebot3_simulations \
 && cd /opt/tb3_ws && . /opt/ros/humble/setup.bash && colcon build --symlink-install

# 测试包 (随仓库携带)
COPY src/nav_test /opt/nav_ws/src/nav_test
RUN cd /opt/nav_ws && . /opt/ros/humble/setup.bash && colcon build --symlink-install

# Gazebo 在线模型离线内置 (ground_plane/sun), 避免运行时联网下载失败
COPY docker/gazebo_models /root/.gazebo/models

# 地图直接取自 turtlebot3_navigation2 官方包 (turtlebot3_world 预建图), 改名并同步 yaml
RUN mkdir -p /opt/nav_env/map \
 && cp /opt/ros/humble/share/turtlebot3_navigation2/map/map.pgm /opt/nav_env/map/turtlebot3_world_map.pgm \
 && cp /opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml /opt/nav_env/map/turtlebot3_world_map.yaml \
 && sed -i 's|^image:.*|image: turtlebot3_world_map.pgm|' /opt/nav_env/map/turtlebot3_world_map.yaml

# Nav2 参数 (被测配置)
COPY config/custom_nav2.yaml /opt/nav_env/custom_nav2.yaml

COPY docker/entrypoint.sh /opt/nav_env/entrypoint.sh
RUN chmod +x /opt/nav_env/entrypoint.sh

WORKDIR /opt/nav_ws
ENTRYPOINT ["/opt/nav_env/entrypoint.sh"]
CMD ["python3", "-m", "pytest", "src/nav_test/test/", "-v", "-s"]
