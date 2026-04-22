"""Navigate through all waypoints in sequence using Nav2."""

import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')

        # Load waypoints
        pkg_dir = get_package_share_directory('rob_project')
        waypoints_file = os.path.join(pkg_dir, 'config', 'waypoints.yaml')
        with open(waypoints_file, 'r') as f:
            config = yaml.safe_load(f)

        self.waypoints = config['mission']['route']
        self.current_index = 0
        self.navigating = False

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info(
            f'Waypoint runner loaded {len(self.waypoints)} waypoints'
        )
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f'  [{i}] {wp["name"]}: ({wp["x"]:.3f}, {wp["y"]:.3f})'
            )

        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting route.')
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('=== ROUTE COMPLETE! All waypoints reached. ===')
            return

        wp = self.waypoints[self.current_index]
        self.get_logger().info(
            f'--- Navigating to [{self.current_index}/{len(self.waypoints)-1}] '
            f'{wp["name"]} ({wp["x"]:.3f}, {wp["y"]:.3f}) ---'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            wp = self.waypoints[self.current_index]
            self.get_logger().error(f'Goal rejected for {wp["name"]}! Skipping.')
            self.current_index += 1
            self.send_next_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass  # Could log distance remaining

    def result_callback(self, future):
        result = future.result()
        wp = self.waypoints[self.current_index]

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'=== Reached {wp["name"]}! ===')
        else:
            self.get_logger().warn(
                f'Failed to reach {wp["name"]} (status={result.status}). '
                f'Continuing to next waypoint.'
            )

        self.current_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Route cancelled by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
