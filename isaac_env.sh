#!/bin/bash
# isaac_env.sh — Isaac Sim 仿真一键启动
# 用法:
#   ./isaac_env.sh            启动（含 RViz，用于 Nav2 发目标点）
#   ./isaac_env.sh --no-rviz  启动（不含 RViz）
#   ./isaac_env.sh stop       关闭所有进程

# 自动检测 workspace 路径（脚本所在目录即 workspace）
export ROBOT_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Isaac Python：优先环境变量，其次自动在 conda 环境中查找
if [[ -z "$ISAAC_PYTHON" ]]; then
  ISAAC_PYTHON=$(conda run -n isaac which python3 2>/dev/null || echo "")
  [[ -z "$ISAAC_PYTHON" ]] && ISAAC_PYTHON="$HOME/miniconda3/envs/isaac/bin/python3.10"
fi
export ISAAC_PYTHON
NAV_PARAMS="$ROBOT_WS/install/robot_navigation/share/robot_navigation/params/nav2_params_isaac.yaml"

if [[ "$1" == "stop" ]]; then
  pkill -9 -f "isaac_scene.py|robot_state_publisher|joint_state_publisher|rviz2|nav2|slam_toolbox|pointcloud_to_laserscan" 2>/dev/null
  echo "✅ 已关闭所有进程"
  exit 0
fi

NO_RVIZ=false
[[ "$1" == "--no-rviz" ]] && NO_RVIZ=true

[[ ! -f "$ISAAC_PYTHON" ]] && { echo "❌ 找不到 $ISAAC_PYTHON"; exit 1; }

# 清理可能残留的旧进程（防止重复启动导致多个 pc2scan/slam_toolbox 实例）
echo "🧹 清理旧进程..."
pkill -9 -f "pc2scan.py" 2>/dev/null
pkill -9 -f "async_slam_toolbox" 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/setup.bash"

# 强制 Vulkan 使用 NVIDIA GPU（避免 Intel 核显被选中导致黑屏）
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# [1] URDF + RViz（RViz 用于 Nav2 发目标点和可视化）
echo "[1/3] URDF + RViz..."
if [[ "$NO_RVIZ" == "true" ]]; then
  ros2 launch robot_description isaac_sim_robot.launch.py rviz:=false &
else
  ros2 launch robot_description isaac_sim_robot.launch.py rviz:=true &
fi
URDF_PID=$!
sleep 2

# [2] Isaac Sim 场景
echo "[2/3] Isaac Sim..."
ros2 launch isaac_bridge isaac_sim.launch.py &
ISAAC_PID=$!
sleep 5

# [3] slam_toolbox 在线建图（发布 /map，同时提供 map→odom TF）
# 等待更长时间确保 Isaac 已发布 /scan
echo "[3/4] slam_toolbox 在线建图..."
echo "  等待 Isaac Sim /scan 就绪..."
for i in $(seq 1 30); do
  if ros2 topic info /scan 2>/dev/null | grep -q "Publisher count"; then
    break
  fi
  sleep 1
done
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$ROBOT_WS/src/robot_navigation/params/slam_toolbox_params.yaml" \
  use_sim_time:=true &
SLAM_PID=$!
sleep 5

# [4] Nav2 导航（等 slam 发出第一张 /map 再启动）
echo "[4/4] Nav2 导航..."
echo "  等待 /map 就绪..."
for i in $(seq 1 30); do
  if ros2 topic info /map 2>/dev/null | grep -q "Publisher count"; then
    break
  fi
  sleep 1
done
ros2 launch robot_navigation navigation.launch.py \
  params_file:="$NAV_PARAMS" &
NAV_PID=$!

echo ""
echo "============================================================"
echo " Isaac Sim 仿真已启动"
echo " 渲染: RayTracedLighting (offline kit 下实际禁用)"
echo " PhysX LiDAR: /lidar/points (3D PointCloud2, 16线±15°) → pointcloud_to_laserscan → /scan
 odom: /odom (~60Hz)"
echo " SLAM: slam_toolbox 在线建图 → /map (实时来自 Isaac)"
echo ""
echo " 话题: /lidar/points  /scan  /odom  /cmd_vel  /map"
echo " RViz: 使用 Nav2 Goal 工具发送目标点"
echo " 停止: ./isaac_env.sh stop"
echo "============================================================"

cleanup() {
  kill $URDF_PID $ISAAC_PID $SLAM_PID $NAV_PID 2>/dev/null
  pkill -9 -f "isaac_scene.py" 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM
wait
