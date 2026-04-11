"""
navigation.launch.py
====================
功能: 导航栈启动入口 (占位)

定位来源:
  FAST-LIO2 发布 TF: map -> odom -> base_link
  导航算法直接订阅该 TF 树。

【TODO】确认导航算法后补充:
  - 全局规划器 (Left-turn / A* / 其他)
  - 局部规划器 (ego_planner / 其他 ROS 算法)
  - 对应的 IncludeLaunchDescription 和参数传递

当前此文件仅作占位, 尚不可运行。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_navigation = get_package_share_directory('robot_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )

    # 【TODO】在此处 IncludeLaunchDescription 启动确认后的导航算法

    return LaunchDescription([
        use_sim_time_arg,
    ])
