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
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
import struct


class Pc2ScanNode(Node):
    def __init__(self):
        super().__init__('pc2scan')
        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter('min_height',      -0.1)   # 世界坐标系下，地面以上 0.1m
        self.declare_parameter('max_height',       1.5)   # 世界坐标系下，障碍物上限
        self.declare_parameter('angle_min',       -3.1416)
        self.declare_parameter('angle_max',        3.1416)
        self.declare_parameter('angle_increment',  0.00349)
        self.declare_parameter('scan_time',        0.1)
        self.declare_parameter('range_min',        0.1)
        self.declare_parameter('range_max',       50.0)
        self.declare_parameter('use_inf',          True)
        # lidar_pitch: lidar_link 相对于机器人本体的俧仰角(rad)
        # URDF sensors.xacro: rpy="0 -0.4947 0" → pitch = -0.4947 rad (-28.3°)斜向上
        # pc2scan 用此角补偿，将 z_lidar 还原为近似世界坐标系中的高度
        self.declare_parameter('lidar_pitch',     -0.4947)

        self.min_height      = self.get_parameter('min_height').value
        self.max_height      = self.get_parameter('max_height').value
        self.angle_min       = self.get_parameter('angle_min').value
        self.angle_max       = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.scan_time       = self.get_parameter('scan_time').value
        self.range_min       = self.get_parameter('range_min').value
        self.range_max       = self.get_parameter('range_max').value
        self.use_inf         = self.get_parameter('use_inf').value
        pitch                = self.get_parameter('lidar_pitch').value
        # 静态旋转矩阵（绕 Y 轴旋转 -pitch，将 lidar 坐标系 z 换算到世界坐标系 z）
        # z_world ≈ -x_lidar * sin(pitch) + z_lidar * cos(pitch)
        self._cp = math.cos(pitch)   # cos(-0.4947) ≈  0.8788
        self._sp = math.sin(pitch)   # sin(-0.4947) ≈ -0.4772

        # ── lidar_link → base_footprint 静态变换 ───────────────────────
        # URDF 链：base_footprint→base_link(z=0.14275) →top_plate_link(z+0.23445) →lidar_link(xyz=0.10,0,0.042, R_y(-0.4947))
        # 合计平移（base_footprint系下lidar原点）= (0.10, 0.0, 0.4192)
        # R_y(theta): [[c,0,s],[0,1,0],[-s,0,c]]   theta=-0.4947
        # 用于将 /patchwork/non_ground 从 lidar_link 变换到 base_footprint，
        # 使 GridMap 的 T_world_sensor * p_sensor 计算正确（GridMap 假设点在 base_footprint 系）
        _p = pitch   # -0.4947 rad
        self._R_y = [
            [ math.cos(_p), 0.0, math.sin(_p)],   # row 0
            [ 0.0,          1.0, 0.0          ],   # row 1
            [-math.sin(_p), 0.0, math.cos(_p) ],   # row 2
        ]
        self._lidar_tx = 0.10    # lidar origin x in base_footprint
        self._lidar_ty = 0.0
        self._lidar_tz = 0.4192  # 0.14275 + 0.23445 + 0.042

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
        # 非地面点云发布（供 local_planner_mine GridMap 使用，替代 patchwork）
        # 点云坐标系已变换到 base_footprint（与 /state_estimation child_frame 一致）
        # 使用 RELIABLE QoS 保证 GridMap（C++ message_filters::Synchronizer）能匹配到消息
        self.nonground_pub = self.create_publisher(PointCloud2, 'nonground', reliable_qos)
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

        # 非地面点云收集（xyz float32，与输入格式相同）
        nonground_points = []

        for i in range(msg.width * msg.height):
            off = i * ps
            x = struct.unpack_from(fmt, data, off + x_off)[0]
            y = struct.unpack_from(fmt, data, off + y_off)[0]
            z = struct.unpack_from(fmt, data, off + z_off)[0]

            # NaN/Inf 跳过
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            # 将 lidar 坐标系的 z 补偿到近似世界坐标系
            # z_world ≈ -x*sin(pitch) + z*cos(pitch)（绕Y轴逆旋转，消除俯仰角）
            # 注意：此公式等效于完整变换的 z 分量减去 lidar 高度 0.4192m
            # 所以地面点（bz_base≈0）对应 z_world≈-0.4192，被 min_height=-0.1 正确过滤
            z_world = -x * self._sp + z * self._cp
            if z_world < self.min_height or z_world > self.max_height:
                continue

            # 收集通过高度过滤的点到非地面点云
            nonground_points.append((x, y, z))

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
        # ★ 必须使用点云自带时间戳，不能用 get_clock().now()
        # 原因：slam_toolbox 在查找 scan 对应的 TF 时，用的是 scan.header.stamp
        #       Isaac OmniGraph 发布 /lidar/points 与发布 /clock 是同一帧，时间戳一致
        #       如果改用节点时钟，scan 时间戳 > TF 时间戳 → slam_toolbox 找不到对应 TF
        #       → 建图时每帧位姿错误 → 地图重叠严重
        scan.header.stamp    = msg.header.stamp
        # 点云已从 lidar_link 变换到 base_footprint 系，frame_id 必须对应
        # 这样 global_costmap obstacle_layer 能用 TF(base_footprint→map) 正确标注障碍
        scan.header.frame_id = 'base_footprint'
        scan.angle_min       = self.angle_min
        scan.angle_max       = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment  = 0.0
        scan.scan_time       = self.scan_time
        scan.range_min       = self.range_min
        scan.range_max       = self.range_max
        scan.ranges          = ranges
        self.pub.publish(scan)

        # ── 发布非地面点云 /patchwork/non_ground ──────────────────────
        # 供 local_planner_mine GridMap 节点使用，替代真实机器人上的 patchwork 地面分割
        # ★ 坐标系变换：lidar_link → base_footprint
        # ★ 始终发布（即使空帧）：GridMap 的 ApproximateTime 同步器需要持续收到消息才能触发回调
        #   若场景无障碍物导致 nonground_points 为空而不发消息，同步器永远不触发
        #   → has_map_ 永远是 false → PlanningLoop 第一行 return → 机器人不动
        R = self._R_y
        tx, ty, tz = self._lidar_tx, self._lidar_ty, self._lidar_tz
        # 机器人自身过滤半径：底盘半径约 0.203m，加 0.1m 余量
        # 防止激光雷达扫到自身底盘/轮子，导致 GridMap 把机器人原点标记为障碍
        _self_filter_r2 = 0.303 ** 2  # (0.203 + 0.1)^2
        transformed = []
        for (lx, ly, lz) in nonground_points:
            bx = R[0][0]*lx + R[0][1]*ly + R[0][2]*lz + tx
            by = R[1][0]*lx + R[1][1]*ly + R[1][2]*lz + ty
            bz = R[2][0]*lx + R[2][1]*ly + R[2][2]*lz + tz
            if bx*bx + by*by < _self_filter_r2:
                continue  # 过滤机器人自身
            transformed.append((bx, by, bz))
        ng = PointCloud2()
        ng.header.stamp    = msg.header.stamp
        ng.header.frame_id = 'base_footprint'
        ng.height = 1
        ng.width  = len(transformed)      # 0 = 空帧，同步器仍然触发
        ng.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        ng.is_bigendian = False
        ng.point_step   = 12
        ng.row_step     = 12 * len(transformed)
        ng.is_dense     = True
        if transformed:
            raw = bytearray(ng.row_step)
            for j, (bx, by, bz) in enumerate(transformed):
                struct.pack_into('<fff', raw, j * 12, bx, by, bz)
            ng.data = bytes(raw)
        self.nonground_pub.publish(ng)


def main():
    rclpy.init()
    node = Pc2ScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
