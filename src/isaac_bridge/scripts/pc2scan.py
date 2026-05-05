#!/usr/bin/env python3
"""
pc2scan.py — PointCloud2 → LaserScan 转换节点
=============================================
替代 pointcloud_to_laserscan_node（C++ 节点存在 QoS 兼容性问题）。
订阅 /lidar/points (PointCloud2)，输出 /scan (LaserScan)。

特性:
  - 用 RELIABLE QoS 订阅（与 Isaac OmniGraph 兼容）
  - 无 TF 依赖（直接在点云原始帧处理）
  - use_sim_time 安全（时间戳直接从消息头复制）
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
import struct


class Pc2ScanNode(Node):
    def __init__(self):
        super().__init__('pc2scan')
        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter('min_height',      -2.0)
        self.declare_parameter('max_height',       2.0)
        self.declare_parameter('angle_min',       -1.5708)
        self.declare_parameter('angle_max',        1.5708)
        self.declare_parameter('angle_increment',  0.00349)
        self.declare_parameter('scan_time',        0.1)
        self.declare_parameter('range_min',        0.1)
        self.declare_parameter('range_max',       50.0)
        self.declare_parameter('use_inf',          True)

        self.min_height      = self.get_parameter('min_height').value
        self.max_height      = self.get_parameter('max_height').value
        self.angle_min       = self.get_parameter('angle_min').value
        self.angle_max       = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.scan_time       = self.get_parameter('scan_time').value
        self.range_min       = self.get_parameter('range_min').value
        self.range_max       = self.get_parameter('range_max').value
        self.use_inf         = self.get_parameter('use_inf').value

        # RELIABLE QoS —— 与 Isaac OmniGraph RELIABLE 发布者匹配
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PointCloud2, 'cloud_in', self.cloud_cb, reliable_qos)
        self.pub = self.create_publisher(LaserScan, 'scan', best_effort_qos)
        self.get_logger().info(
            f'pc2scan 已启动: height=[{self.min_height},{self.max_height}] '
            f'angle=[{math.degrees(self.angle_min):.1f}°,{math.degrees(self.angle_max):.1f}°] '
            f'range=[{self.range_min},{self.range_max}]')

    def cloud_cb(self, msg: PointCloud2):
        n_bins = int(math.ceil(
            (self.angle_max - self.angle_min) / self.angle_increment))

        fill = float('inf') if self.use_inf else self.range_max - 1e-2
        ranges = [fill] * n_bins

        # 解析点云字段偏移量
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x': x_off = field.offset
            elif field.name == 'y': y_off = field.offset
            elif field.name == 'z': z_off = field.offset

        if x_off is None or y_off is None or z_off is None:
            self.get_logger().warn('点云缺少 x/y/z 字段', throttle_duration_sec=5)
            return

        data = msg.data
        ps   = msg.point_step
        fmt  = '<f'  # little-endian float32

        for i in range(msg.width * msg.height):
            off = i * ps
            x = struct.unpack_from(fmt, data, off + x_off)[0]
            y = struct.unpack_from(fmt, data, off + y_off)[0]
            z = struct.unpack_from(fmt, data, off + z_off)[0]

            # NaN/Inf 跳过
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            # 高度过滤
            if z < self.min_height or z > self.max_height:
                continue
            # 距离
            r = math.hypot(x, y)
            if r < self.range_min or r > self.range_max:
                continue
            # 方位角
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            # 映射到 bin
            idx = int((angle - self.angle_min) / self.angle_increment)
            idx = max(0, min(n_bins - 1, idx))
            if r < ranges[idx]:
                ranges[idx] = r

        scan = LaserScan()
        # 使用节点当前时间（与 TF 同步）而不是直接复制点云时间戳
        # 原因：点云时间戳来自 Isaac OmniGraph，可能略早于 RSP 发布的 TF，
        #       导致 slam_toolbox / RViz 的 TF 查找失败（"queue is full" 警告）
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min       = self.angle_min
        scan.angle_max       = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment  = 0.0
        scan.scan_time       = self.scan_time
        scan.range_min       = self.range_min
        scan.range_max       = self.range_max
        scan.ranges          = ranges
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = Pc2ScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
