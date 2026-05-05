"""
navigation.launch.py
====================
启动 Nav2 导航栈（Isaac Sim 模式）

支持模式 (mode 参数):
  isaac — Isaac Sim 仿真模式 (默认): slam_toolbox 在线建图 + Nav2 导航
  nav   — 加载已有地图 + AMCL 定位 + 导航（真实机器人）

用法:
  # Isaac 仿真导航（默认）
  ros2 launch robot_navigation navigation.launch.py use_sim_time:=true

  # 已有地图导航（真实机器人）
  ros2 launch robot_navigation navigation.launch.py \\
      mode:=nav map:=/path/to/map.yaml use_sim_time:=false

全局规划: NavFn (A*)  — nav2_params_isaac.yaml
局部规划: DWB         — nav2_params_isaac.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_nav = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── 参数声明 ──────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='isaac',
        description='运行模式: isaac=Isaac Sim仿真(slam_toolbox建图), nav=已有地图(真实机器人)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='true=Isaac Sim, false=真实机器人'
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='地图 yaml 文件路径 (mode=nav 时使用)'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'params', 'nav2_params.yaml'),
        description='Nav2 参数文件'
    )
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── PointCloud2 → LaserScan ───────────────────────────────
    # Isaac PhysX LiDAR (16线 ±15°) 发布 /lidar/points (PointCloud2)
    # 这里将其投影为 /scan (LaserScan) 供 slam_toolbox 和 Nav2 使用
    # min_height/max_height 选取接近地面水平面的点（高度相对 lidar_link 坐标系）
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/lidar/points'),
            ('scan',     '/scan'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            # base_footprint 坐标系下做高度切片，与雷达安装角度无关
            # 取机器人高度 0.1m~0.7m 范围（覆盖从底盘上沿到雷达）
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.5,
            'min_height': 0.1,     # 离地 0.1m 以上（过滤地面）
            'max_height': 0.7,     # 离地 0.7m 以下（过滤天花板）
            'angle_min':  -1.5708, # -90°（前半圆）
            'angle_max':   1.5708, # +90°
            'angle_increment': 0.00349,  # 0.2°
            'scan_time':  0.1,
            'range_min':  0.1,
            'range_max':  50.0,
            'use_inf':    True,
        }],
    )

    # ── Nav2 核心导航节点 ─────────────────────────────────────
    nav2_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'false',
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # ── map_server + amcl (nav 模式) ──────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': LaunchConfiguration('map'),
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )

    # ── Lifecycle Manager ─────────────────────────────────────
    lifecycle_nodes_base = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
        'velocity_smoother',
    ]
    lifecycle_nodes_nav = ['map_server', 'amcl'] + lifecycle_nodes_base

    lifecycle_mgr_isaac = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes_base,
        }],
        condition=IfCondition(PythonExpression(["'" , mode, "' == 'isaac'"]))
    )

    lifecycle_mgr_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes_nav,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        map_arg,
        params_arg,
        pointcloud_to_laserscan,
        nav2_core,
        map_server,
        amcl,
        lifecycle_mgr_isaac,
        lifecycle_mgr_nav,
    ])
