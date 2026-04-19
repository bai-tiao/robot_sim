#!/usr/bin/env python3
"""
pointcloud_to_map.py
将 /registered_scan (map系 PointCloud2) 投影为 2D OccupancyGrid 发布到 /map
Unity 模式专用：Unity 提供整个房间的全局点云，一次性建立全局静态地图
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from builtin_interfaces.msg import Time


class PointCloudToMap(Node):
    def __init__(self):
        super().__init__('pointcloud_to_map')

        # 地图参数
        self.declare_parameter('resolution', 0.1)       # 每格 0.1m
        self.declare_parameter('z_min', -0.3)           # 只投影这个高度范围内的点
        self.declare_parameter('z_max',  2.0)           # 过滤地面和天花板
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('update_once', True)     # True=只建一次地图后停止更新

        self.resolution = self.get_parameter('resolution').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.map_frame = self.get_parameter('map_frame').value
        self.update_once = self.get_parameter('update_once').value

        self.map_built = False

        # 订阅全局点云
        self.sub = self.create_subscription(
            PointCloud2,
            '/registered_scan',
            self.cloud_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
        )

        # 发布地图 (transient_local 保证后启动的节点也能收到)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.get_logger().info('等待 /registered_scan 数据建立全局地图...')

    def cloud_callback(self, msg: PointCloud2):
        if self.update_once and self.map_built:
            return

        # 读取点云 xyz
        points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ])

        if len(points) == 0:
            return

        # 高度过滤：只保留 z_min ~ z_max 范围内的点（墙壁/家具，去掉地面和天花）
        mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        points = points[mask]

        if len(points) == 0:
            return

        # 计算地图边界（加 1m 边距）
        margin = 1.0
        x_min = float(np.min(points[:, 0])) - margin
        y_min = float(np.min(points[:, 1])) - margin
        x_max = float(np.max(points[:, 0])) + margin
        y_max = float(np.max(points[:, 1])) + margin

        width  = int((x_max - x_min) / self.resolution) + 1
        height = int((y_max - y_min) / self.resolution) + 1

        # 初始化为 free (0)
        grid = np.zeros(width * height, dtype=np.int8)

        # 把有点的格子标为 occupied (100)
        ix = ((points[:, 0] - x_min) / self.resolution).astype(int)
        iy = ((points[:, 1] - y_min) / self.resolution).astype(int)

        # 裁剪越界
        valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        ix = ix[valid]
        iy = iy[valid]
        idx = iy * width + ix
        grid[idx] = 100

        # 膨胀一格（避免障碍物格子太稀疏）
        occupied = np.where(grid == 100)[0]
        for di in [-1, 0, 1]:
            for dj in [-width, 0, width]:
                neighbors = occupied + di + dj
                valid_n = (neighbors >= 0) & (neighbors < len(grid))
                grid[neighbors[valid_n]] = np.maximum(grid[neighbors[valid_n]], 100)

        # 构建 OccupancyGrid
        occ = OccupancyGrid()
        occ.header.stamp = msg.header.stamp
        occ.header.frame_id = self.map_frame
        occ.info = MapMetaData()
        occ.info.resolution = self.resolution
        occ.info.width = width
        occ.info.height = height
        occ.info.origin.position.x = x_min
        occ.info.origin.position.y = y_min
        occ.info.origin.position.z = 0.0
        occ.info.origin.orientation.w = 1.0
        occ.data = grid.tolist()

        self.pub.publish(occ)
        self.map_built = True
        self.get_logger().info(
            f'全局地图已发布: {width}x{height} 格, 分辨率={self.resolution}m, '
            f'原点=({x_min:.1f}, {y_min:.1f}), 障碍物格数={int(np.sum(grid==100))}'
        )


def main():
    rclpy.init()
    node = PointCloudToMap()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
