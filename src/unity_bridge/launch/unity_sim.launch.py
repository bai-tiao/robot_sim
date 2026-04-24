"""
unity_sim.launch.py  (unity_bridge 包)
=======================================
启动 Unity 仿真所需的全部 ROS2 节点。
配合 unity_env.sh 或单独使用。

话题架构:
  Unity (TCP:10000)
    → /registered_scan       PointCloud2, frame_id=map  (已配准世界系点云)
    ← /unity_sim/set_model_state  PoseStamped            (机器人位置同步)

  vehicle_simulator
    ← /cmd_vel               TwistStamped               (你的导航速度指令)
    → /state_estimation      Odometry, map→sensor        (虚拟里程计)
    → TF: map → sensor       (200Hz)

  sensor_scan_generation
    ← /registered_scan + /state_estimation
    → /sensor_scan           PointCloud2, sensor_at_scan (局部点云, 给 nav2 用)
    → TF: map → sensor_at_scan

  static_transform_publisher
    → TF: sensor → base_footprint  (z=-vehicleHeight, 让 URDF 落在地面)

用法:
  ros2 launch unity_bridge unity_sim.launch.py
  ros2 launch unity_bridge unity_sim.launch.py vehicle_height:=0.75
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 参数 ──────────────────────────────────────────────────
    vh_arg = DeclareLaunchArgument(
        'vehicle_height', default_value='0.75',
        description='机器人质心离地高度 (m), 用于 sensor→base_footprint 静态TF'
    )
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port', default_value='10000',
        description='Unity ros_tcp_endpoint TCP 端口'
    )

    vehicle_height = LaunchConfiguration('vehicle_height')
    tcp_port = LaunchConfiguration('tcp_port')

    # ── ros_tcp_endpoint: Unity ↔ ROS2 TCP 桥接 ──────────────
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': '0.0.0.0',
            'ROS_TCP_PORT': tcp_port,
        }]
    )

    # ── sim_image_repub: Unity 压缩图 → raw Image ─────────────
    # 当前 Unity 场景无相机，此节点不消耗资源（无数据则静默）
    sim_image_repub = Node(
        package='vehicle_simulator',
        executable='sim_image_repub',
        name='sim_image_repub',
        output='log',
        parameters=[{
            'camera_in_topic':       '/camera/image/compressed',
            'camera_raw_out_topic':  '/camera/image',
            'depth_in_topic':        '/camera/depth/image_raw',
            'depth_raw_out_topic':   '/camera/depth/image',
        }]
    )

    # ── vehicleSimulator: 虚拟里程计 ─────────────────────────
    vehicle_simulator = Node(
        package='vehicle_simulator',
        executable='vehicleSimulator',
        name='vehicle_simulator',
        output='screen',
        parameters=[{
            'vehicleHeight': vehicle_height,
            'adjustZ':       True,   # 根据地形点云调整 z 高度
            'adjustIncl':    False,  # 不调整倾斜角
        }]
    )

    # ── sensorScanGeneration: map系点云 → sensor_at_scan系 ───
    sensor_scan_gen = Node(
        package='sensor_scan_generation',
        executable='sensorScanGeneration',
        name='sensor_scan_generation',
        output='screen',
    )

    # ── 静态 TF: sensor → base_footprint ─────────────────────
    # vehicleSimulator 发布 map→sensor, z = terrain + vehicleHeight
    # base_footprint 在地面 (z=0), 所以偏移 = -vehicleHeight
    # 有了此 TF, URDF 模型才能正确落在地面跟随机器人移动
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_to_base_footprint',
        output='log',
        # xyz rpy parent child
        # z = -vehicleHeight; 用字符串拼接因为 LaunchConfiguration 不支持算术
        arguments=['0', '0', '-0.75', '0', '0', '0', 'sensor', 'base_footprint']
    )

    # ── visualization_tools: 读 map.ply → 发布 /overall_map ──
    viz_tools = Node(
        package='visualization_tools',
        executable='visualizationTools',
        name='visualization_tools',
        output='log',
        parameters=[{
            'mapFile': '/home/zsh/autonomy_stack_diablo_setup/src/base_autonomy/vehicle_simulator/mesh/unity/map.ply',
        }]
    )

    # ── octomap_server: /overall_map (PointCloud2) → /map (OccupancyGrid) ──
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='log',
        parameters=[{
            'resolution': 0.15,
            'frame_id': 'map',
            'sensor_model/max_range': 50.0,
            'occupancy_min_z': 0.1,
            'occupancy_max_z': 2.0,
            'filter_ground': True,
            'ground_filter/distance': 0.04,
            'ground_filter/angle': 0.15,
            'ground_filter/plane_distance': 0.07,
        }],
        remappings=[
            ('cloud_in', '/overall_map'),
            ('projected_map', '/map'),
        ]
    )

    return LaunchDescription([
        vh_arg,
        tcp_port_arg,
        ros_tcp_endpoint,
        sim_image_repub,
        vehicle_simulator,
        sensor_scan_gen,
        static_tf,
        viz_tools,
        octomap,
    ])
