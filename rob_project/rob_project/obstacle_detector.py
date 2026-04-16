"""
Obstacle Detector node - Detects dynamic obstacles using LiDAR.
FIB UPC - Robotics Project

Processes LaserScan data to detect obstacles that may not be in the
static map. Publishes obstacle information and triggers emergency
stop if obstacles are dangerously close.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from rclpy.clock import Clock

from rob_project.utils import get_min_range_in_sector


class ObstacleDetector(Node):
    """Node that monitors LiDAR for dynamic obstacles and triggers safety stops."""

    # Sector definitions (radians, robot frame: 0=front, positive=left)
    FRONT_MIN = -0.40
    FRONT_MAX = 0.40
    FRONT_LEFT_MIN = 0.40
    FRONT_LEFT_MAX = 1.05
    FRONT_RIGHT_MIN = -1.05
    FRONT_RIGHT_MAX = -0.40
    LEFT_MIN = 1.05
    LEFT_MAX = 1.92
    RIGHT_MIN = -1.92
    RIGHT_MAX = -1.05

    def __init__(self):
        super().__init__('obstacle_detector')

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10
        )

        # Parameters
        self.declare_parameter('emergency_stop_distance', 0.15)
        self.declare_parameter('warning_distance', 0.40)
        self.declare_parameter('enabled', True)

        self.emergency_dist = self.get_parameter(
            'emergency_stop_distance'
        ).value
        self.warning_dist = self.get_parameter('warning_distance').value
        self.enabled = self.get_parameter('enabled').value

        # State
        self.scan_data = None
        self.emergency_active = False

        # Subscribers
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_best_effort
        )

        # Publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', qos_reliable
        )
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', qos_reliable
        )

        # Check obstacles at 10 Hz
        self.create_timer(0.1, self.check_obstacles)

        self.get_logger().info(
            f'Obstacle detector active. '
            f'Emergency: {self.emergency_dist}m, Warning: {self.warning_dist}m'
        )

    def scan_callback(self, msg):
        """Store latest scan data."""
        self.scan_data = msg

    def check_obstacles(self):
        """Periodically check for dangerous obstacles."""
        if not self.enabled or self.scan_data is None:
            return

        # Check all sectors
        front = get_min_range_in_sector(
            self.scan_data, self.FRONT_MIN, self.FRONT_MAX
        )
        front_left = get_min_range_in_sector(
            self.scan_data, self.FRONT_LEFT_MIN, self.FRONT_LEFT_MAX
        )
        front_right = get_min_range_in_sector(
            self.scan_data, self.FRONT_RIGHT_MIN, self.FRONT_RIGHT_MAX
        )

        min_front = min(front, front_left, front_right)

        # Emergency stop
        if min_front < self.emergency_dist:
            if not self.emergency_active:
                self.get_logger().warn(
                    f'EMERGENCY STOP! Obstacle at {min_front:.2f}m'
                )
                self.emergency_active = True

            # Publish emergency flag
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            # Send zero velocity
            stop_msg = TwistStamped()
            stop_msg.header.stamp = Clock().now().to_msg()
            stop_msg.header.frame_id = ''
            self.cmd_vel_pub.publish(stop_msg)

        elif self.emergency_active and min_front > self.warning_dist:
            # Clear emergency when obstacle is far enough
            self.get_logger().info('Emergency cleared. Obstacle moved away.')
            self.emergency_active = False
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_stop_pub.publish(emergency_msg)

    def get_sector_ranges(self):
        """Get minimum ranges for all sectors. Useful for external queries.

        Returns:
            dict: Sector name -> min range.
        """
        if self.scan_data is None:
            inf = float('inf')
            return {
                'front': inf, 'front_left': inf, 'front_right': inf,
                'left': inf, 'right': inf,
            }

        return {
            'front': get_min_range_in_sector(
                self.scan_data, self.FRONT_MIN, self.FRONT_MAX
            ),
            'front_left': get_min_range_in_sector(
                self.scan_data, self.FRONT_LEFT_MIN, self.FRONT_LEFT_MAX
            ),
            'front_right': get_min_range_in_sector(
                self.scan_data, self.FRONT_RIGHT_MIN, self.FRONT_RIGHT_MAX
            ),
            'left': get_min_range_in_sector(
                self.scan_data, self.LEFT_MIN, self.LEFT_MAX
            ),
            'right': get_min_range_in_sector(
                self.scan_data, self.RIGHT_MIN, self.RIGHT_MAX
            ),
        }


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Obstacle detector interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
