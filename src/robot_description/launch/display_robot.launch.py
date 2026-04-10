"""
display_robot.launch.py
======================
功能: 在RViz2中可视化机器人模型 (不启动Gazebo)
用途: 调试URDF, 检查坐标系和传感器安装位置

启动后可在RViz2中看到:
  - 机器人TF坐标系树
  - 机器人模型 (RobotModel显示)
  - 用joint_state_publisher_gui拖动轮子关节
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_description')

    # ── 参数声明 ──────────────────────────────────────────────────────
    urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value=os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
        description='URDF/Xacro文件的绝对路径'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='是否使用仿真时钟(Gazebo时用true)'
    )

    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=os.path.join(pkg_share, 'config', 'robot_description.rviz'),
        description='RViz2配置文件路径'
    )

    # ── xacro解析为URDF字符串 ─────────────────────────────────────────
    robot_description = Command([
        'xacro ', LaunchConfiguration('urdf_model')
    ])

    # ── robot_state_publisher ────────────────────────────────────────
    # 发布 robot_description 参数 和 TF (fixed joints)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ── joint_state_publisher_gui ────────────────────────────────────
    # GUI滑动条控制关节角度, 仅用于调试
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ── RViz2 ────────────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        urdf_model_arg,
        use_sim_time_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ])
