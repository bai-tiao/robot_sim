#!/bin/bash
# ============================================================
# unity_env.sh — Unity 仿真环境一键启动
# ============================================================
# 用法:
#   ./unity_env.sh              启动 (含 RViz)
#   ./unity_env.sh --no-rviz    启动 (不含 RViz)
#   ./unity_env.sh stop         关闭所有相关进程
# ============================================================

ROBOT_WS="/home/zsh/VLN/robot_sim"

# Unity 可执行文件路径：优先使用环境变量 UNITY_EXEC，否则用默认路径
UNITY_EXEC="${UNITY_EXEC:-/home/zsh/autonomy_stack_diablo_setup/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64}"

# ── stop 子命令 ──────────────────────────────────────────────
if [[ "$1" == "stop" ]]; then
  echo "正在清理 Unity 仿真相关进程..."
  pkill -9 -f "Model.x86_64|vehicleSimulator|sim_image_repub|ros_tcp_endpoint|sensorScanGeneration|robot_state_publisher|joint_state_publisher|static_transform_publisher|rviz2" 2>/dev/null
  sleep 1
  echo "✅ 清理完成"
  exit 0
fi

NO_RVIZ=false
for arg in "$@"; do [[ "$arg" == "--no-rviz" ]] && NO_RVIZ=true; done

# ── 1. 加载环境 ──────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source "$ROBOT_WS/install/setup.bash"

# ── 2. 启动 URDF + 静态 TF + RViz (robot_description 包) ────
echo "[1/3] 启动 URDF/TF..."
RVIZ_FLAG="true"
[[ "$NO_RVIZ" == "true" ]] && RVIZ_FLAG="false"
ros2 launch robot_description unity_sim.launch.py rviz:=$RVIZ_FLAG &
LAUNCH_PID=$!
sleep 2

# ── 3. 启动 Unity 可执行文件 ─────────────────────────────────
echo "[2/3] 启动 Unity..."
"$UNITY_EXEC" &
UNITY_PID=$!
sleep 3   # 等 Unity 加载完再连 TCP

# ── 4. 启动 ROS2 ↔ Unity 桥接节点 (unity_bridge 包) ─────────
echo "[3/3] 启动 Unity 桥接节点..."
ros2 launch unity_bridge unity_sim.launch.py &
BRIDGE_PID=$!

echo ""
echo "============================================"
echo "Unity 仿真已就绪"
echo "  /registered_scan   世界系点云 (frame=map)"
echo "  /sensor_scan       传感器系点云 (给 nav2 用)"
echo "  /state_estimation  里程计 (map→sensor)"
echo "  /cmd_vel           速度指令输入 (TwistStamped)"
echo "============================================"
echo "Ctrl+C 退出..."

cleanup() {
  kill $LAUNCH_PID $UNITY_PID $BRIDGE_PID 2>/dev/null
  pkill -9 -f "Model.x86_64|vehicleSimulator|sim_image_repub|ros_tcp_endpoint|sensorScanGeneration|robot_state_publisher|joint_state_publisher|static_transform_publisher|rviz2" 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM
wait
