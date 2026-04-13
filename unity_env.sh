#!/bin/bash
# ============================================================
# unity_env.sh — Unity 仿真环境启动脚本
# ============================================================
#
# TF 树 (Unity 模式):
#   vehicleSimulator  → map → sensor         (机器人位姿, 200Hz)
#   static TF         → sensor → base_footprint (z=-0.75, launch 文件发布)
#   robot_state_pub   → base_footprint → base_link → ...  (URDF 内部链)
#
# 话题流:
#   Unity (TCP:10000)
#     → /registered_scan  (PointCloud2, frame_id=map, 已配准点云)
#     → /unity_sim/set_model_state (接收vehicleSimulator发来的机器人位置)
#
#   vehicleSimulator
#     ← /cmd_vel  (TwistStamped, 你的导航算法输出)
#     → /state_estimation  (Odometry, frame_id=map, child=sensor)
#     → TF: map→sensor
#
#   sensorScanGeneration
#     ← /registered_scan + /state_estimation
#     → /sensor_scan  (传感器坐标系下的局部点云, frame_id=sensor_at_scan)
#     → TF: map→sensor_at_scan
#
# 用法:
#   ./unity_env.sh              # 默认启动 (含RViz)
#   ./unity_env.sh --no-rviz    # 不启动 RViz
#   ./unity_env.sh stop         # 关闭所有相关进程
# ============================================================

# 不用 set -e，避免 source 时某些警告导致退出

# ── stop 子命令：一键杀死所有相关进程 ───────────────────────
if [[ "$1" == "stop" ]]; then
  echo "正在清理所有 Unity 仿真相关进程..."
  pkill -9 -f "Model.x86_64|vehicleSimulator|sim_image_repub|ros_tcp_endpoint|robot_state_publisher|joint_state_publisher|sensorScanGeneration|static_transform_publisher|rviz2" 2>/dev/null
  sleep 1
  REMAINING=$(ps aux | grep -E "Model\.x86_64|vehicleSimulator|sim_image_repub|ros_tcp_endpoint|robot_state_publisher|sensorScanGeneration|rviz2" | grep -v grep)
  if [[ -z "$REMAINING" ]]; then
    echo "✅ 所有进程已清理完毕"
  else
    echo "⚠️  以下进程仍在运行:"
    echo "$REMAINING"
  fi
  exit 0
fi

CMU_WS="/home/zsh/autonomy_stack_diablo_setup"
ROBOT_WS="/home/zsh/VLN/robot_sim"
UNITY_DIR="$CMU_WS/src/base_autonomy/vehicle_simulator/mesh/unity/environment"
RVIZ_CFG="$ROBOT_WS/install/robot_description/share/robot_description/config/gazebo_sim.rviz"

NO_RVIZ=false
for arg in "$@"; do
  [[ "$arg" == "--no-rviz" ]] && NO_RVIZ=true
done

# ── 1. 加载两个工作空间 ──────────────────────────────────────
echo "[1/5] 加载 ROS2 环境 + CMU工作空间 + 你的robot_sim工作空间..."
source /opt/ros/humble/setup.bash
source "$CMU_WS/install/setup.bash"         # CMU的节点: vehicle_simulator, ros_tcp_endpoint
source "$ROBOT_WS/install/setup.bash"       # 你的节点: robot_description

# ── 2. 启动你的 URDF / TF / RViz (用 launch 文件) ──────────
echo "[2/5] 启动 robot_state_publisher + joint_state_publisher + RViz..."
# unity_sim.launch.py 用 xacro Command() 直接展开 URDF，
# 同时启动 joint_state_publisher(发布关节零位) 保证 TF 完整
if [ "$NO_RVIZ" = false ]; then
  ros2 launch robot_description unity_sim.launch.py &
else
  ros2 launch robot_description unity_sim.launch.py rviz:=false &
fi
LAUNCH_PID=$!
echo "      launch PID: $LAUNCH_PID"
sleep 3

# ── 3. 启动 Unity 环境 ───────────────────────────────────────
echo "[3/5] 启动 Unity 环境 (后台, 约3秒加载)..."
"$UNITY_DIR/Model.x86_64" &
UNITY_PID=$!
echo "      Unity PID: $UNITY_PID"
sleep 3

# ── 4. 启动 CMU ROS2 节点 ────────────────────────────────────
echo "[4/5] 启动 ros_tcp_endpoint / sim_image_repub / vehicleSimulator..."

# ros_tcp_endpoint: Unity ↔ ROS2 TCP桥接
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000 &
sleep 1

# sim_image_repub: Unity压缩图 → raw sensor_msgs/Image
ros2 run vehicle_simulator sim_image_repub \
  --ros-args \
  -p camera_in_topic:=/camera/image/compressed \
  -p camera_raw_out_topic:=/camera/image \
  -p depth_in_topic:=/camera/depth/image_raw \
  -p depth_raw_out_topic:=/camera/depth/image &
sleep 1

# vehicleSimulator: 订阅 /cmd_vel(TwistStamped) → 积分位姿
#   → 发布 /state_estimation(Odometry)
#   → 发布 /unity_sim/set_model_state(PoseStamped) 让Unity同步机器人位置
ros2 run vehicle_simulator vehicleSimulator \
  --ros-args \
  -p vehicleHeight:=0.75 \
  -p adjustZ:=true \
  -p adjustIncl:=false &
VSIM_PID=$!
echo "      vehicleSimulator PID: $VSIM_PID"
sleep 1

# ── 5. 启动 sensorScanGeneration ─────────────────────────────
echo "[5/5] 启动 sensorScanGeneration (点云坐标系转换: map→sensor_at_scan)..."
# 订阅: /registered_scan (map系点云) + /state_estimation (里程计)
# 发布: /sensor_scan (传感器坐标系下的局部点云, frame_id=sensor_at_scan)
# 同时发布 TF: map→sensor_at_scan
ros2 run sensor_scan_generation sensorScanGeneration &
SSG_PID=$!
echo "      sensorScanGeneration PID: $SSG_PID"

# RViz2 已由 unity_sim.launch.py 启动

echo ""
echo "=========================================="
echo "Unity 仿真环境已就绪"
echo ""
echo "TF 树:"
echo "  map → sensor              (vehicleSimulator, 200Hz)"
echo "  sensor → base_footprint   (静态, z=-0.75)"
echo "  base_footprint → base_link → ...  (robot_state_publisher)"
echo ""
echo "传感器数据 (Unity 此场景只有 LiDAR):"
echo "  /registered_scan     世界坐标系点云 (frame_id=map)"
echo "  /sensor_scan         传感器坐标系点云 (frame_id=sensor_at_scan)"
echo ""
echo "里程计 (vehicleSimulator积分):"
echo "  输入: /cmd_vel  (geometry_msgs/TwistStamped)"
echo "  输出: /state_estimation  (nav_msgs/Odometry)"
echo ""
echo "注意: vehicleSimulator 订阅 TwistStamped (有 header),"
echo "      如你的导航发布 Twist (无 header), 需加转换节点"
echo "=========================================="
echo ""
echo "按 Ctrl+C 关闭所有节点..."

# Ctrl+C 后清理所有子进程
cleanup() {
  echo "正在关闭所有节点..."
  kill $LAUNCH_PID $UNITY_PID $VSIM_PID $SSG_PID 2>/dev/null
  pkill -9 -f "sim_image_repub|ros_tcp_endpoint|rviz2|Model.x86_64|robot_state_publisher|joint_state_publisher|static_transform_publisher|sensorScanGeneration" 2>/dev/null
  exit 0
}
trap cleanup SIGINT SIGTERM
wait
