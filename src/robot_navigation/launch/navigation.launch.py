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
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
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

    # ── PointCloud2 → LaserScan (Python 实现) ───────────────────
    # 用自定义 Python 节点替代 C++ pointcloud_to_laserscan_node
    # 原因：C++ 节点与 Isaac OmniGraph (RELIABLE QoS) 存在兼容性问题，
    #       Python 节点可显式指定 RELIABLE 订阅，无 TF 依赖，无时间戳问题
    # 注意：必须用系统 python3（/opt/ros/humble 环境），
    #       Isaac Python 环境的 rclpy/sensor_msgs 版本与 Humble 不一定兼容
    _pc2scan_script = os.path.join(
        get_package_share_directory('isaac_bridge'), 'scripts', 'pc2scan.py')
    pointcloud_to_laserscan = ExecuteProcess(
        cmd=[
            '/usr/bin/python3.10', _pc2scan_script,
            '--ros-args',
            '-r', 'cloud_in:=/lidar/points',
            '-r', 'scan:=/scan',
            '-p', 'min_height:=-0.3',
            '-p', 'max_height:=0.3',
            '-p', 'angle_min:=-3.1416',
            '-p', 'angle_max:=3.1416',
            '-p', 'angle_increment:=0.00349',
            '-p', 'range_min:=0.1',
            '-p', 'range_max:=50.0',
            '-p', 'use_inf:=true',
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        additional_env={
            # 追加 ROS2 Humble 的包路径（不覆盖已有 PYTHONPATH，避免丢失 workspace 包）
            'PYTHONPATH': '/opt/ros/humble/local/lib/python3.10/dist-packages:'
                          '/opt/ros/humble/lib/python3.10/site-packages:'
                          + os.environ.get('PYTHONPATH', ''),
        }
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
