"""
Precision Parking node - Phase III docking maneuver.
FIB UPC - Robotics Project

Executes a precision approach to park the robot exactly at the center
of the charging station. Uses direct velocity control (not Nav2) for
fine-grained positioning with continuous LiDAR safety monitoring.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from rclpy.clock import Clock

from rob_project.utils import (
    euler_from_quaternion,
    angle_difference,
    angle_to_target,
    distance,
    get_min_range_in_sector,
)


class PrecisionParking(Node):
    """Node that performs precision parking into the charging station."""

    # Parking state machine
    STATE_IDLE = 'IDLE'
    STATE_ALIGN = 'ALIGN'           # Rotate to face station center
    STATE_APPROACH = 'APPROACH'     # Drive toward station center
    STATE_FINAL_ALIGN = 'FINAL_ALIGN'  # Final orientation adjustment
    STATE_DONE = 'DONE'
    STATE_ABORTED = 'ABORTED'

    def __init__(self):
        super().__init__('precision_parking')

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10
        )

        # Parameters
        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('angle_tolerance', 0.08)
        self.declare_parameter('safety_distance', 0.10)
        self.declare_parameter('approach_slowdown', 0.20)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.safety_dist = self.get_parameter('safety_distance').value
        self.approach_slowdown = self.get_parameter('approach_slowdown').value

        # State
        self.state = self.STATE_IDLE
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.scan_data = None

        # Subscribers
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_best_effort
        )
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_best_effort
        )
        self.create_subscription(
            PoseStamped, '/parking_goal', self.goal_callback, qos_reliable
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', qos_reliable
        )
        self.parking_status_pub = self.create_publisher(
            String, '/parking_status', qos_reliable
        )

        # Control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Precision parking node ready (waiting for goal).')

    def odom_callback(self, msg):
        """Update robot pose."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        """Store latest scan."""
        self.scan_data = msg

    def goal_callback(self, msg):
        """Receive parking goal and start the maneuver."""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_yaw = euler_from_quaternion(msg.pose.orientation)
        self.state = self.STATE_ALIGN
        self.get_logger().info(
            f'Parking goal received: ({self.target_x:.2f}, {self.target_y:.2f}), '
            f'yaw={math.degrees(self.target_yaw):.1f} deg. Starting ALIGN.'
        )

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """Publish a TwistStamped command."""
        msg = TwistStamped()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_status(self, status):
        """Publish parking status."""
        msg = String()
        msg.data = status
        self.parking_status_pub.publish(msg)

    def check_safety(self):
        """Check if any obstacle is dangerously close in front.

        Returns:
            bool: True if safe to continue, False if must stop.
        """
        if self.scan_data is None:
            return True
        front_min = get_min_range_in_sector(self.scan_data, -0.5, 0.5)
        if front_min < self.safety_dist:
            self.get_logger().warn(
                f'Parking safety stop! Obstacle at {front_min:.2f}m'
            )
            return False
        return True

    def control_loop(self):
        """Main parking control loop."""
        if self.state == self.STATE_IDLE or self.state == self.STATE_DONE:
            return

        if self.state == self.STATE_ABORTED:
            self.publish_cmd()
            return

        # Safety check in all active states
        if not self.check_safety() and self.state == self.STATE_APPROACH:
            self.publish_cmd()
            self.state = self.STATE_ABORTED
            self.publish_status('ABORTED')
            self.get_logger().error('Parking aborted due to safety constraint.')
            return

        if self.state == self.STATE_ALIGN:
            self._do_align()
        elif self.state == self.STATE_APPROACH:
            self._do_approach()
        elif self.state == self.STATE_FINAL_ALIGN:
            self._do_final_align()

        self.publish_status(self.state)

    def _do_align(self):
        """Rotate to face the station center."""
        target_angle = angle_to_target(
            self.robot_x, self.robot_y, self.target_x, self.target_y
        )
        error = angle_difference(target_angle, self.robot_yaw)

        if abs(error) < self.angle_tol:
            self.publish_cmd()
            self.state = self.STATE_APPROACH
            self.get_logger().info('Aligned. Starting APPROACH.')
            return

        # Proportional angular control
        kp = 1.5
        angular = max(-self.angular_speed, min(self.angular_speed, kp * error))
        self.publish_cmd(angular_z=angular)

    def _do_approach(self):
        """Drive toward station center."""
        dist = distance(
            self.robot_x, self.robot_y, self.target_x, self.target_y
        )

        if dist < self.pos_tol:
            self.publish_cmd()
            self.state = self.STATE_FINAL_ALIGN
            self.get_logger().info('At station center. Starting FINAL_ALIGN.')
            return

        # Correct heading while approaching
        target_angle = angle_to_target(
            self.robot_x, self.robot_y, self.target_x, self.target_y
        )
        angle_err = angle_difference(target_angle, self.robot_yaw)

        # Speed proportional to distance (slow down near target)
        kp_lin = 0.5
        linear = min(self.linear_speed, kp_lin * dist)

        # If heading error is large, reduce forward speed
        if abs(angle_err) > 0.3:
            linear *= 0.3

        kp_ang = 1.5
        angular = max(-self.angular_speed, min(self.angular_speed, kp_ang * angle_err))

        self.publish_cmd(linear_x=linear, angular_z=angular)

    def _do_final_align(self):
        """Final orientation adjustment at station center."""
        error = angle_difference(self.target_yaw, self.robot_yaw)

        if abs(error) < self.angle_tol:
            self.publish_cmd()
            self.state = self.STATE_DONE
            self.publish_status('DONE')
            self.get_logger().info('PARKING COMPLETE!')
            return

        kp = 1.2
        angular = max(-self.angular_speed, min(self.angular_speed, kp * error))
        self.publish_cmd(angular_z=angular)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionParking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Precision parking interrupted.')
        node.publish_cmd()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
