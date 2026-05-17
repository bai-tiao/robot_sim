#!/bin/bash
# isaac_env.sh — Isaac Sim 仿真一键启动（dualmap 架构）
#
# 用法:
#   ./isaac_env.sh               启动（ground_truth 定位，含 RViz）
#   ./isaac_env.sh --no-rviz     启动（不含 RViz）
#   ./isaac_env.sh slam          启动（slam_toolbox 定位模式）
#   ./isaac_env.sh map           建图模式（slam_toolbox 建图，键盘控制，无导航规划）
#   ./isaac_env.sh map save      保存当前地图到 maps/my_map.yaml
#   ./isaac_env.sh stop          关闭所有进程
#
# cmd_vel 优先级：键盘（/cmd_vel）> 导航（/nav_cmd_vel）
#   键盘 1 秒内有消息时，导航指令被忽略；超过 1 秒无键盘输入，导航接管
#   因此可同时运行键盘和导航，键盘随时接管
#
# 完整数据流：
#   Isaac Sim → /lidar/points → pc2scan.py → /scan + /patchwork/non_ground
#   slam_toolbox → /map
#   /state_estimation (ground_truth: isaac_scene.py | slam: slam_pose_bridge)
#   /goal_pose → goal_to_plan → planner_server → /plan
#   local_planner_mine (DWA) → /cmd_vel → Isaac Sim
#
# RViz 发目标：
#   Tool Properties → 2D Goal Pose → Topic: /goal_pose

export ROBOT_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Isaac Python：优先环境变量，其次自动查找 conda 环境
if [[ -z "$ISAAC_PYTHON" ]]; then
  ISAAC_PYTHON=$(conda run -n isaac which python3 2>/dev/null || echo "")
  [[ -z "$ISAAC_PYTHON" ]] && ISAAC_PYTHON="$HOME/miniconda3/envs/isaac/bin/python3.10"
fi
export ISAAC_PYTHON

# ── 停止命令 ──────────────────────────────────────────────────────
if [[ "$1" == "stop" ]]; then
  echo "🛑 关闭所有仿真进程..."
  pkill -9 -f "isaac_scene.py|robot_state_publisher|joint_state_publisher|rviz2" 2>/dev/null
  pkill -9 -f "pc2scan.py|slam_toolbox|async_slam_toolbox|goal_to_plan.py|slam_pose_bridge.py" 2>/dev/null
  pkill -9 -f "my_planner_node|planner_server|lifecycle_manager|map_server" 2>/dev/null
  echo "✅ 已关闭所有进程"
  exit 0
fi

# ── 地图保存命令 ──────────────────────────────────────────────────
if [[ "$1" == "map" && "$2" == "save" ]]; then
  source /opt/ros/humble/setup.bash
  source "$ROBOT_WS/install/setup.bash"
  MAP_DIR="$ROBOT_WS/src/robot_navigation/maps"
  MAP_NAME="${3:-my_map}"
  echo "💾 保存地图到 $MAP_DIR/$MAP_NAME ..."
  ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" \
    --ros-args -p use_sim_time:=true
  echo "✅ 地图已保存：$MAP_DIR/${MAP_NAME}.pgm / .yaml"
  exit 0
fi

# ── 参数解析 ──────────────────────────────────────────────────────
LOC_MODE="ground_truth"
MAP_MODE=false
NO_RVIZ=false
for arg in "$@"; do
  [[ "$arg" == "slam" ]]      && LOC_MODE="slam"
  [[ "$arg" == "map" ]]       && MAP_MODE=true && LOC_MODE="slam"
  [[ "$arg" == "--no-rviz" ]] && NO_RVIZ=true
done

[[ ! -f "$ISAAC_PYTHON" ]] && { echo "❌ 找不到 Isaac Python: $ISAAC_PYTHON"; exit 1; }

# ── 清理残留进程 ──────────────────────────────────────────────────
echo "🧹 清理旧进程..."
pkill -9 -f "pc2scan.py|async_slam_toolbox|goal_to_plan.py|slam_pose_bridge.py|my_planner_node" 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/setup.bash"

# ── GPU 显示设置 ──────────────────────────────────────────────────
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 定位模式传给 isaac_scene.py
export ISAAC_LOCALIZATION_MODE="$LOC_MODE"

echo "======================================================"
echo " Isaac Sim 仿真启动 | 定位模式: $LOC_MODE"
echo "======================================================"

# ── [1] URDF + RViz ───────────────────────────────────────────────
echo "[1/4] URDF + RViz..."
if [[ "$NO_RVIZ" == "true" ]]; then
  ros2 launch robot_description isaac_sim_robot.launch.py rviz:=false &
else
  ros2 launch robot_description isaac_sim_robot.launch.py rviz:=true &
fi
URDF_PID=$!
sleep 2

# ── [2] Isaac Sim 场景（含 /lidar/points /odom /state_estimation TF）─
echo "[2/4] Isaac Sim..."
ros2 launch isaac_bridge isaac_sim.launch.py mode:="$LOC_MODE" &
ISAAC_PID=$!

# 等待 Isaac Sim 发布 /clock（仿真已启动）
echo "  等待 Isaac Sim /clock 就绪..."
for i in $(seq 1 60); do
  if ros2 topic info /clock 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "  /clock 已就绪 (${i}s)"
    break
  fi
  sleep 1
done
sleep 3

# ── [3] 导航栈（含 pc2scan slam planner goal_bridge local_planner）─
echo "[3/4] 导航栈 (pc2scan + slam_toolbox + planner_server + local_planner)..."

if [[ "$MAP_MODE" == "true" ]]; then
  # 建图模式：只启动 pc2scan + slam_toolbox（建图），不启动规划器
  echo "  [建图模式] 只启动 pc2scan + slam_toolbox，用键盘控制机器人探索"
  echo "  保存地图：另开终端执行 ./isaac_env.sh map save [地图名]"
  ros2 launch robot_navigation navigation.launch.py \
    use_sim_time:=true \
    mode:=slam_map_only &
else
  ros2 launch robot_navigation navigation.launch.py \
    use_sim_time:=true \
    mode:="$LOC_MODE" &
fi
NAV_PID=$!

echo ""
echo "======================================================"
echo " 仿真已启动"
echo " 定位: $LOC_MODE$([ "$MAP_MODE" == "true" ] && echo ' [建图模式]')"
echo " 话题: /lidar/points  /scan  /patchwork/non_ground"
echo "       /odom  /state_estimation  /map  /plan"
echo "       /cmd_vel(键盘) > /nav_cmd_vel(导航) → Isaac"
echo " 目标: RViz → 2D Goal Pose → Topic: /goal_pose"
echo " 建图: ./isaac_env.sh map save [名字]  保存地图"
echo " 停止: ./isaac_env.sh stop"
echo "======================================================"

cleanup() {
  echo "🛑 正在关闭..."
  kill $URDF_PID $ISAAC_PID $NAV_PID 2>/dev/null
  pkill -9 -f "isaac_scene.py|pc2scan.py|async_slam_toolbox|goal_to_plan.py|my_planner_node|planner_server|lifecycle_manager|slam_pose_bridge.py" 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM
wait
