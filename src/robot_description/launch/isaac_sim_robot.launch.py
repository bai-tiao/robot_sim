"""
isaac_sim_robot.launch.py  (robot_description 包)
=================================================
启动 robot_state_publisher + joint_state_publisher + RViz2。
配合 isaac_bridge/isaac_sim.launch.py 使用。

用法:
  ros2 launch robot_description isaac_sim_robot.launch.py
  ros2 launch robot_description isaac_sim_robot.launch.py rviz:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')

    rviz_arg = DeclareLaunchArgument(
        name='rviz', default_value='true',
        description='是否启动 RViz2'
    )

    robot_desc = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 延迟 2s 启动，等 RSP 先发布 /robot_description 话题
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_gui': False, 'use_sim_time': True}]
    )

    rviz_config = os.path.join(pkg, 'config', 'gazebo_sim.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([rviz_arg, rsp, TimerAction(period=2.0, actions=[jsp]), rviz])
