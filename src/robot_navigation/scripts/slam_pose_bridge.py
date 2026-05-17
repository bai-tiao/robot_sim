#!/usr/bin/env python3
"""
slam_pose_bridge.py — TF → /state_estimation 桥接节点
======================================================
将 slam_toolbox 发布的 TF (map → base_footprint) 转为
Odometry 话题 /state_estimation，供 local_planner_mine 使用。

适用于阶段2（slam 定位模式）：
  slam_toolbox（scan matching 开启）→ map→odom TF
  isaac_scene.py → odom→base_footprint TF
  本节点 → /state_estimation (map frame Odometry)

切换到 FASTLIO 后，停用本节点，改用 /fastlio_odom → /state_estimation remap。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math


class SlamPoseBridge(Node):
    def __init__(self):
        super().__init__('slam_pose_bridge')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self.map_frame   = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Odometry, '/state_estimation', 10)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)
        self.get_logger().info(
            f'slam_pose_bridge 已启动: TF {self.map_frame}→{self.robot_frame} → /state_estimation'
        )

    def timer_cb(self):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rclpy.time.Time(),          # 最新可用 TF
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception:
            return

        tr = t.transform.translation
        rot = t.transform.rotation

        # 四元数 → yaw（仅用于速度估算，位置直接用 TF）
        # ROS geometry_msgs quaternion: x y z w
        qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        odom = Odometry()
        odom.header.stamp    = t.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id  = self.robot_frame

        odom.pose.pose.position.x    = tr.x
        odom.pose.pose.position.y    = tr.y
        odom.pose.pose.position.z    = tr.z
        odom.pose.pose.orientation.x = rot.x
        odom.pose.pose.orientation.y = rot.y
        odom.pose.pose.orientation.z = rot.z
        odom.pose.pose.orientation.w = rot.w

        # twist 留零（slam 不提供速度，local_planner 不强依赖）
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = SlamPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
