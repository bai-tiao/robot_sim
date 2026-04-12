"""
gazebo_sim.launch.py
====================
功能: 启动 Gazebo Classic + 加载机器人 + RViz2

依赖安装:
  sudo apt install ros-humble-gazebo-ros-pkgs

使用:
  cd ~/VLN/robot_sim
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch robot_description gazebo_sim.launch.py

参数:
  world   : world 文件路径 (默认 indoor_simple.world)
  gui     : 是否显示 Gazebo GUI (默认 true)
  rviz    : 是否启动 RViz2 (默认 true)
  x,y,yaw : 机器人初始位置
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── 参数 ──────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg, 'worlds', 'indoor_simple.world'),
        description='Gazebo world 文件路径, 默认使用本地 indoor_simple.world'
    )
    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true',
        description='是否显示 Gazebo GUI'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rviz', default_value='true',
        description='是否启动 RViz2'
    )
    x_arg   = DeclareLaunchArgument(name='x',   default_value='0.0')
    y_arg   = DeclareLaunchArgument(name='y',   default_value='0.0')
    yaw_arg = DeclareLaunchArgument(name='yaw', default_value='0.0')

    # ── URDF ──────────────────────────────────────────────────────────
    robot_desc = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')]),
        value_type=str
    )

    # ── robot_state_publisher ─────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ── Gazebo ────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui':   LaunchConfiguration('gui'),
        }.items()
    )

    # ── Spawn 机器人 (延迟等 Gazebo 启动) ────────────────────────────
    # 使用系统 python3 (/usr/bin/python3) 显式调用 spawn_entity.py,
    # 避免 conda 环境的 python3 抢占 PATH 导致 lxml 找不到
    spawn = TimerAction(period=3.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            prefix='/usr/bin/python3',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'diff_drive_robot',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', '0.15',   # 略高于地面防止初始穿模
                '-Y', LaunchConfiguration('yaw'),
            ]
        )
    ])

    # ── RViz2 ─────────────────────────────────────────────────────────
    # 加载预设配置: RobotModel + LiDAR点云(/lidar/points) + Odometry + TF
    rviz_config = os.path.join(pkg, 'config', 'gazebo_sim.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg, gui_arg, rviz_arg,
        x_arg, y_arg, yaw_arg,
        rsp,
        gazebo,
        spawn,
        rviz,
    ])
