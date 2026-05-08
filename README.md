# robot_sim（Isaac Sim + ROS2 Humble）

本仓库主流程：Isaac Sim 提供仿真真值位姿与点云 → `pc2scan.py` 转换为 `/scan` → `slam_toolbox` 在线建图 → Nav2 导航。

## 环境前提

| 项目 | 要求 |
|------|------|
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| 仿真引擎 | Isaac Sim 4.5.x（pip 安装到 conda 环境 `isaac`）|
| 工作区 | 已执行 `colcon build` |

## 快速开始

```bash
export ROBOT_WS=/path/to/robot_sim   # ← 修改为实际路径
cd $ROBOT_WS
source /opt/ros/humble/setup.bash
colcon build
```

```bash
# 一键启动（含 RViz）
cd $ROBOT_WS
./isaac_env.sh

# 无 RViz 启动
cd $ROBOT_WS
./isaac_env.sh --no-rviz

# 停止所有进程
cd $ROBOT_WS
./isaac_env.sh stop
```

## 数据流

```
isaac_scene.py
  ├── /clock
  ├── /lidar/points  (PointCloud2, PhysX LiDAR)
  ├── /odom
  └── TF: map→odom (identity, 60Hz)  /  odom→base_footprint (ground truth, 60Hz)
        ↓
     pc2scan.py  (navigation.launch.py 内启动)
        └── /scan  (前180°, 高度过滤)
              ↓
           slam_toolbox → /map
           Nav2 (local/global costmap)
```

## 渲染模式

Isaac Sim 有三种渲染器，代码字符串与 UI 对应如下：

| 代码字符串 | UI 显示名称 | 光追 | 说明 |
|----------|-----------|------|------|
| `Storm` | （不在 UI 渲染菜单内） | ❌ 无光追 | 默认值。Hydra/OpenGL 后端，与 RTX 完全独立。无 PBR 材质，小车显示线框/单色。任何显卡可用。 |
| `RayTracedLighting` | **RTX - Real-Time** | ✅ 实时 | RTX 实时光次。PBR 材质正常显示。需 RTX 显卡；40 系验证可用；50 系在 Isaac 4.5 不稳定。 |
| `PathTracing` | **RTX - Interactive (Path Tracing)** | ✅ 离线 | 最高质量光追，每帧多次采样。适合渲染截图，不适合实时仿真。 |

> **Storm 不出现在 UI 渲染菜单里**，是因为它是完全不同的渲染后端（Hydra，非 RTX）。UI 菜单里只列 RTX 关联的模式。

```bash
# 切换渲染器（在启动前设置）
export ISAAC_RENDERER=RayTracedLighting   # → UI 显示 RTX - Real-Time
export ISAAC_RENDERER=PathTracing         # → UI 显示 RTX - Interactive (Path Tracing)
export ISAAC_RENDERER=Storm               # 默认，不出现在 RTX 菜单
cd $ROBOT_WS
./isaac_env.sh
```

> **关于我们代码里的 `ISAAC_RENDERER` 是否真的生效？**  
> 生效。`isaac_scene.py` 第 101 行 `"renderer": os.environ.get("ISAAC_RENDERER", "Storm")` 会把字符串传给 `SimulationApp`，这与在 Isaac UI 里手动切换渲染器等价。  
> 50 系显卡切到 `RayTracedLighting` 后 **Isaac 确实进入了 RTX Real-Time 模式**，但 Blackwell GPU 在 Isaac 4.5 的 RTX 驱动适配不完整，表现为材质仍然异常、可能黑屏等。这是 GPU 驱动/Isaac 版本兼容问题，不是代码未生效。

> **50 系用户**：保持 `Storm` 默认值即可。小车材质无法显示是 Storm 的已知限制，预计 Isaac Sim 4.6+ 修复 Blackwell 支持后可正常使用 RTX 模式。

## 小车材质不显示的原因

Storm 渲染器使用 Hydra（OpenGL 光栅化），不支持 USD PBR 材质的完整渲染，因此 URDF 导入后的小车模型只显示几何线框或简单颜色。切换到 `RayTracedLighting` 后材质会正常显示。

## 常用检查命令

```bash
source /opt/ros/humble/setup.bash
source $ROBOT_WS/install/setup.bash

ros2 topic hz /scan                     # 验证 LaserScan 频率
ros2 topic hz /odom                     # 验证里程计频率
ros2 topic echo /lidar/points --once    # 查看点云格式
ros2 run tf2_tools view_frames          # 输出 TF 树到 frames.gv
```

## 单独查看机器人模型（仅可视化，不启动仿真）

```bash
cd $ROBOT_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_description display_robot.launch.py
```