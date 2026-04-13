"""
unity_sim.launch.py
===================
为 Unity 仿真环境启动 robot_state_publisher + joint_state_publisher + RViz2
不启动 Gazebo，配合 unity_env.sh 使用

TF 树说明:
  vehicleSimulator 发布:  map → sensor  (200Hz, 机器人质心位姿)
  本 launch 补充静态 TF:  sensor → base_footprint  (z = -0.75, 即地面)
  robot_state_publisher:  base_footprint → base_link → ... (URDF 内部链)

  这样 RViz Fixed Frame = map 时，URDF 模型跟随 Unity 仿真机器人移动。

用法:
  ros2 launch robot_description unity_sim.launch.py
  ros2 launch robot_description unity_sim.launch.py rviz:=false
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

    # robot_state_publisher: 展开 URDF, 发布 /robot_description 话题 + TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }]
    )

    # joint_state_publisher: 持续发布所有关节零位
    # 必须等 robot_state_publisher 先发布 /robot_description 话题，延迟 2 秒启动
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_gui': False,
        }]
    )
    delayed_jsp = TimerAction(period=2.0, actions=[jsp])

    # 静态 TF: sensor → base_footprint
    # vehicleSimulator 发布 map→sensor (机器人质心, z = terrain + 0.75)
    # base_footprint 是地面投影, 在 sensor 下方 0.75m
    # 有了这条链, RViz 里 URDF 模型就能跟随 Unity 机器人正确显示
    static_tf_sensor_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_to_base_footprint',
        output='screen',
        arguments=['0', '0', '-0.75', '0', '0', '0', 'sensor', 'base_footprint']
    )

    rviz_config = os.path.join(pkg, 'config', 'gazebo_sim.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([rviz_arg, rsp, delayed_jsp, static_tf_sensor_to_base, rviz])
