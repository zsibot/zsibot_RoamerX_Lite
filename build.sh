#! /bin/bash

set -e

# ====== 1. ROS 环境 ======
source /opt/ros/humble/setup.bash

# ====== 2. 检查参数 ======
if [ $# -eq 0 ]; then
  echo "Usage: $0 {all deploy}"
  echo "./build.sh all             [build all packages including forward and nav2]"
  exit 1
fi

# ====== 3. Debug 模式开关 ======
DEBUG_MODE=false
if [[ "$2" == "debug" ]]; then
  DEBUG_MODE=true
  echo "[zsibot shell log] => DEBUG mode enabled."
  CMAKE_DEBUG_ARGS="-DBUILD_DEBUG_PACKAGES=ON"
else
  CMAKE_DEBUG_ARGS="-DBUILD_DEBUG_PACKAGES=OFF"
fi

case $1 in

all)
  echo "[zsibot shell log] => Start compiling forward packages and nav2..."
  echo "[zsibot shell log] => "

  # 自动扫描 forward 下的所有 ROS2 包
  FORWARD_DIR="src/forward"
  FORWARD_PACKAGES=$(find "$FORWARD_DIR" -mindepth 1 -maxdepth 1 -type d \
                      -exec test -f "{}/package.xml" \; -print | xargs -n1 basename)

  echo "[zsibot shell log] => Detected forward packages: $FORWARD_PACKAGES"

  # 编译 forward 包 + nav2 相关包
colcon build \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $CMAKE_DEBUG_ARGS \
  --packages-select \
    ecal2ros \
    pub_tf \
    common \
    robots_dog_msgs \
    nav2_amcl \
    nav2_behavior_tree \
    nav2_bt_navigator \
    nav2_controller \
    nav2_core \
    nav2_costmap_2d \
    nav2_lifecycle_manager \
    nav2_map_server \
    nav2_msgs \
    nav2_navfn_planner \
    nav2_planner \
    nav2_recoveries \
    nav2_regulated_pure_pursuit_controller \
    nav2_rviz_plugins \
    nav2_util \
    nav2_voxel_grid \
    nav2_waypoint_follower \
  --parallel-workers 8

  echo "[zsibot shell log] => "
  echo "[zsibot shell log] => Build Finished: OK"
  ;;

*)
  echo "无效的参数: $1"
  echo "Usage: $0 {all deploy nx}"
  exit 1
  ;;
esac
