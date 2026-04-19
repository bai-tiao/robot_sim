"""
navigation.launch.py
====================
启动 Nav2 导航栈

支持模式 (mode 参数):
  unity — Unity 仿真模式 (默认): vehicleSimulator 提供定位, 无需 SLAM
  nav   — 加载已有地图 + AMCL 定位 + 导航

用法:
  # Unity 仿真导航
  ros2 launch robot_navigation navigation.launch.py

  # 已有地图导航
  ros2 launch robot_navigation navigation.launch.py \\
      mode:=nav map:=/path/to/map.yaml use_sim_time:=false

全局规划: NavFn (A*)  — nav2_params.yaml planner_server 节
局部规划: DWB         — nav2_params.yaml FollowPath 节, 换算法只改这里
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
        'mode', default_value='unity',
        description='运行模式: unity=Unity仿真(无需SLAM), nav=已有地图'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true=Gazebo, false=Unity/真实机器人'
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
    # 将 /sensor_scan 转为 /scan 供代价地图障碍物层使用
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/sensor_scan'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'sensor',
            'transform_tolerance': 0.1,
            'min_height': -0.5,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.00349,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 100.0,
            'use_inf': True,
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

    lifecycle_mgr_unity = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes_base,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'unity'"]))
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
        lifecycle_mgr_unity,
        lifecycle_mgr_nav,
    ])
