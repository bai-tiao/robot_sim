# scripts/goal_to_plan.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

class GoalToPlan(Node):
    def __init__(self):
        super().__init__('goal_to_plan')
        self._client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self._path_pub = self.create_publisher(Path, '/plan', 10)
        self._sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.get_logger().info('Ready. Use RViz2 "2D Goal Pose" to set target.')

    def goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'Got goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('planner_server not available!')
            return

        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.planner_id = 'GridBased'
        goal.use_start = False

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        path = future.result().result.path
        n = len(path.poses)
        if n == 0:
            self.get_logger().warn('Empty path returned!')
            return
        self._path_pub.publish(path)
        self.get_logger().info(f'Path published: {n} waypoints')

def main():
    rclpy.init()
    rclpy.spin(GoalToPlan())
    rclpy.shutdown()

if __name__ == '__main__':
    main()