"""
isaac_sim.launch.py — Isaac Sim 仿真启动文件
=============================================
替代 unity_bridge/launch/unity_sim.launch.py

话题对照（unity → isaac）:
  /state_estimation  →  /odom
    /sensor_scan       →  /scan  (由 pc2scan.py 从 /lidar/points 转换)
  /registered_scan   →  不再需要
    TF: map→sensor     →  TF: odom→base_footprint

Nav2 使用时需修改 nav2_params.yaml:
  odom_topic: /odom
  robot_base_frame: base_footprint
    global_frame: odom（local costmap）/ map（global costmap）
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Isaac Sim 场景脚本路径
_SCRIPT = os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'isaac_scene.py'
)

# Isaac conda 环境的 Python 路径
_ISAAC_PYTHON = os.environ.get(
    'ISAAC_PYTHON',
    os.path.expanduser('~/miniconda3/envs/isaac/bin/python3.10')
)


def generate_launch_description():

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true=无窗口模式（SSH/服务器）'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Isaac Sim 使用仿真时钟'
    )

    # ── Isaac Sim 主进程 ──────────────────────────────────────────
    # 渲染模式说明：
    #   默认由 isaac_scene.py 的 SimulationApp 读取 ISAAC_RENDERER
    #   未设置时默认 Storm；设置 ISAAC_RENDERER=RayTracedLighting 可切换 RTX
    # 启动参数说明：
    #   --/app/renderer/resolution/*  设置窗口分辨率
    #   --/persistent/isaac/asset_root/default=  跳过 Nucleus 资产下载检查
    isaac_sim = ExecuteProcess(
        cmd=[
            _ISAAC_PYTHON, _SCRIPT,
            # 渲染器由 isaac_scene.py 内部设置，此处不传 renderer 参数
            # '--/renderer/enabled=0',                      # ← 关闭光追，50系稳定
            '--/app/renderer/resolution/width=1280',
            '--/app/renderer/resolution/height=720',
            '--/persistent/isaac/asset_root/default=',   # 跳过 Nucleus 检查
        ],
        # 注：最终渲染器以 isaac_scene.py 中 SimulationApp 配置为准
        output='screen',
        additional_env={
            'DISPLAY': os.environ.get('DISPLAY', ':0'),
            'ROBOT_WS': os.environ.get('ROBOT_WS', os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')),
        }
    )

    # ── PointCloud2 → LaserScan 转换在 robot_navigation 启动 ──────
    # isaac_scene.py 发布 /lidar/points (PointCloud2)
    # navigation.launch.py 中运行 pc2scan.py 生成 /scan

    # ── visualization_tools / octomap 已移除 ─────────────────────
    # 原 Unity 流程：map.ply → visualization_tools → octomap → /map
    # 现 Isaac 流程：/scan → slam_toolbox → /map（动态实时建图）
    # viz_tools 和 octomap 不再需要，且 map.ply 不存在会导致 launch 崩溃

    # ── TF 说明 ───────────────────────────────────────────────────────────────
    # map→odom：由 isaac_scene.py 在主循环动态发布 identity（抑制 slam 覆盖）
    # odom→base_footprint：由 isaac_scene.py 基于 Isaac 物理真值动态发布

    return LaunchDescription([
        headless_arg,
        use_sim_time_arg,
        isaac_sim,
        # viz_tools 已移除（map.ply 不存在）
        # octomap 已移除（改用 slam_toolbox）
        # map_to_odom_tf 节点已移除（由 isaac_scene.py 发布 map→odom）
    ])
