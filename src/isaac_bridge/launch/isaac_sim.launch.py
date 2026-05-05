"""
isaac_sim.launch.py — Isaac Sim 仿真启动文件
=============================================
替代 unity_bridge/launch/unity_sim.launch.py

话题对照（unity → isaac）:
  /state_estimation  →  /odom
  /sensor_scan       →  /scan  (真实激光，有遮挡)
  /registered_scan   →  不再需要
  TF: map→sensor     →  TF: odom→base_footprint

Nav2 使用时需修改 nav2_params.yaml:
  odom_topic: /odom
  robot_base_frame: base_footprint
  global_frame: odom   (或 map，搭配 slam_toolbox)
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
    # 关键参数说明：
    #   --/renderer/enabled=0         关闭 RTX 光追，改用 Storm (OpenGL)
    #   --/app/window/hideUi=0        显示 UI（headless=false 时）
    #   --/persistent/isaac/asset_root/default  跳过 Nucleus 资产下载检查
    isaac_sim = ExecuteProcess(
        cmd=[
            _ISAAC_PYTHON, _SCRIPT,
            '--/renderer/enabled=0',                      # ← 关闭光追，50系稳定
            '--/app/renderer/resolution/width=1280',
            '--/app/renderer/resolution/height=720',
            '--/persistent/isaac/asset_root/default=',   # 跳过 Nucleus 检查
        ],
        output='screen',
        additional_env={
            'DISPLAY': os.environ.get('DISPLAY', ':0'),
            'ROBOT_WS': os.environ.get('ROBOT_WS', os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')),
        }
    )

    # ── pointcloud_to_laserscan 不再需要 ─────────────────────────
    # Isaac LiDAR 直接发布 /scan (LaserScan)，不需要转换节点

    # ── visualization_tools / octomap 已移除 ─────────────────────
    # 原 Unity 流程：map.ply → visualization_tools → octomap → /map
    # 现 Isaac 流程：/scan → slam_toolbox → /map（动态实时建图）
    # viz_tools 和 octomap 不再需要，且 map.ply 不存在会导致 launch 崩溃

    # ── 静态 TF 占位 ──────────────────────────────────────────────────────────
    # map→odom：slam_toolbox 会动态发布此 TF，这里不再添加静态占位
    # （静态占位会被 slam_toolbox 的动态 TF 覆盖，但可能引起 TF 冲突警告）
    # odom→base_footprint：由 isaac_scene.py 的 rclpy TF broadcaster 动态发布

    return LaunchDescription([
        headless_arg,
        use_sim_time_arg,
        isaac_sim,
        # viz_tools 已移除（map.ply 不存在）
        # octomap 已移除（改用 slam_toolbox）
        # map_to_odom_tf 已移除（slam_toolbox 发布动态 map→odom TF）
    ])
