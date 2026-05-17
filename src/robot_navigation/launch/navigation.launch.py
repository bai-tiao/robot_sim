"""
navigation.launch.py
====================
Isaac Sim 导航启动（完整 dualmap 架构）

数据流（与 dualmap_nav 完全一致）：

  /lidar/points → pc2scan.py → /scan ──────────────────────────────┐
                             → /patchwork/non_ground ─→ GridMap     │
  /state_estimation ─────────────────────→ GridMap + LocalPlanner   │
                                           └─/grid_map/grid_map     │
  /goal_pose → goal_to_plan → planner_server (/compute_path_to_pose)│
                                   ↓ /plan                          │
                              LocalPlanner (DWA) → /cmd_vel         │
                                                                     │
  /scan → slam_toolbox → /map ───→ global_costmap (static_layer) ←─┘

定位模式：
  ground_truth（默认）: isaac_scene.py 发布 /state_estimation + map→odom identity
  slam:                 slam_toolbox scan_matching → TF，slam_pose_bridge → /state_estimation
                        需同时设置 ISAAC_LOCALIZATION_MODE=slam 启动 isaac_sim

用法（通过 isaac_env.sh 自动调用，无需手动执行）：
  ros2 launch robot_navigation navigation.launch.py            # ground_truth
  ros2 launch robot_navigation navigation.launch.py mode:=slam # slam 定位

RViz 发目标：
  Tool Properties → 2D Goal Pose → Topic: /goal_pose
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_SYSTEM_PYTHON = '/usr/bin/python3.10'
_ROS_PYTHONPATH = (
    '/opt/ros/humble/local/lib/python3.10/dist-packages:'
    '/opt/ros/humble/lib/python3.10/site-packages:'
    + os.environ.get('PYTHONPATH', '')
)


def generate_launch_description():

    pkg_nav = get_package_share_directory('robot_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Isaac Sim 使用仿真时钟'
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='ground_truth',
        description='ground_truth / slam / slam_map_only'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    # 与 dualmap_nav 一致：用 map_server 加载默认地图，不再依赖 slam_toolbox 动态地图尺寸
    # blank_map.yaml: 100x100m 空白地图，机器人永远在地图范围内，彻底消除 out-of-bounds
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': os.path.join(pkg_nav, 'maps', 'blank_map.yaml'),
        }],
    )
    # map_server 也是 lifecycle 节点，加入 lifecycle_manager 管理

    # ── [1] pc2scan.py — /lidar/points → /scan + /patchwork/non_ground ──
    _pc2scan = os.path.join(
        get_package_share_directory('isaac_bridge'), 'scripts', 'pc2scan.py')
    pc2scan_node = ExecuteProcess(
        cmd=[
            _SYSTEM_PYTHON, _pc2scan,
            '--ros-args',
            '-r', 'cloud_in:=/lidar/points',
            '-r', 'scan:=/scan',
            '-r', 'nonground:=/patchwork/non_ground',
            '-p', 'min_height:=-0.1',
            '-p', 'max_height:=1.5',
            '-p', 'lidar_pitch:=-0.4947',
            '-p', 'angle_min:=-1.5708',
            '-p', 'angle_max:=1.5708',
            '-p', 'angle_increment:=0.00349',
            '-p', 'range_min:=0.1',
            '-p', 'range_max:=30.0',
            '-p', 'use_inf:=true',
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        additional_env={'PYTHONPATH': _ROS_PYTHONPATH},
    )

    # ── [2] slam_toolbox — /scan → /map + TF ────────────────────────────
    # ground_truth 模式: 不启动！map_server 已提供 blank_map，两者同时发 /map 会冲突
    # slam 模式:         启动 slam_toolbox 做定位+建图
    # slam_map_only 模式: 建图模式（不启动规划器，只启动 slam_toolbox + pc2scan）
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_nav, 'params', 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(PythonExpression(
            ["'", mode, "' == 'slam' or '", mode, "' == 'slam_map_only'"]))
    )

    # ── [3] planner_server (NavFn A*) — 与 dualmap lio_map_align 一致 ──
    # slam_map_only 建图模式不启动规划器
    _nav_active = PythonExpression(["'", mode, "' != 'slam_map_only'"])
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            os.path.join(pkg_nav, 'params', 'global_planner_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(_nav_active),
    )

    # planner_server 是 lifecycle 节点，需要 lifecycle_manager 管理
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'planner_server'],
            'bond_timeout': 0.0,
        }],
        condition=IfCondition(_nav_active),
    )
    # 建图模式：只需 lifecycle 管理 map_server（给 RViz 显示地图用）
    lifecycle_mgr_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam_map_only'"])),
    )

    # ── [4] goal_to_plan — /goal_pose → planner action → /plan ──────────
    _goal_to_plan = os.path.join(pkg_nav, 'scripts', 'goal_to_plan.py')
    goal_to_plan = ExecuteProcess(
        cmd=[
            _SYSTEM_PYTHON, _goal_to_plan,
            '--ros-args',
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        additional_env={'PYTHONPATH': _ROS_PYTHONPATH},
        condition=IfCondition(_nav_active),
    )

    # ── [5] local_planner_mine — GridMap + DWA ──────────────────────────
    local_planner = Node(
        package='local_planner_mine',
        executable='my_planner_node',
        name='my_planner_node',
        output='screen',
        parameters=[
            os.path.join(pkg_nav, 'params', 'local_planner_params.yaml'),
        ],
        # 导航指令发到 /nav_cmd_vel，isaac_scene.py 实现「键盘优先」逻辑
        # 键盘 1 秒内有消息 → 用键盘；否则 → 用导航指令
        remappings=[('/cmd_vel', '/nav_cmd_vel')],
        condition=IfCondition(_nav_active),
    )

    # ── [6] slam_pose_bridge（slam 模式专用）────────────────────────────
    _bridge = os.path.join(pkg_nav, 'scripts', 'slam_pose_bridge.py')
    slam_pose_bridge = ExecuteProcess(
        cmd=[
            _SYSTEM_PYTHON, _bridge,
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'publish_rate:=20.0',
        ],
        output='screen',
        additional_env={'PYTHONPATH': _ROS_PYTHONPATH},
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        map_server_node,
        pc2scan_node,
        slam_toolbox,
        planner_server,
        lifecycle_mgr,
        lifecycle_mgr_map,
        goal_to_plan,
        local_planner,
        slam_pose_bridge,
    ])
