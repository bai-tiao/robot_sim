"""
navigation.launch.py
====================
功能: 启动 Nav2 导航栈

定位来源:
  FAST-LIO2 发布 /Odometry 和 TF: map -> odom -> base_link
  Nav2 直接订阅该 TF 树, 无需另外启动 AMCL 或 map_server。

规划器说明:
  - 全局规划器: NavFn (A* 模式)  【TODO】换 Smac/Theta* 见 nav2_params.yaml
  - 局部规划器: DWB              【TODO】换 MPPI 见 nav2_params.yaml

使用方式:
  # FAST-LIO2 建图后保存地图, 将路径填入 map 参数
  ros2 launch robot_navigation navigation.launch.py map:=<map.yaml路径>

依赖安装:
  sudo apt install ros-humble-nav2-bringup
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_navigation  = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── 参数声明 ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )

    # 地图文件 (YAML格式)
    # 【TODO】使用FAST-LIO2建图后生成的地图文件路径
    map_arg = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(pkg_navigation, 'config', 'maps', 'map.yaml'),
        description='预建地图YAML文件路径'
    )

    # Nav2参数文件
    nav2_params_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(pkg_navigation, 'params', 'nav2_params.yaml'),
        description='Nav2参数文件路径'
    )

    # ── Nav2 完整导航栈 ───────────────────────────────────────────────
    # 使用 nav2_bringup 包的标准launch文件
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map'),
            # 是否自动激活生命周期节点
            # 【TODO】若需手动管理生命周期则改为false
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        nav2_params_arg,
        nav2_bringup,
    ])
