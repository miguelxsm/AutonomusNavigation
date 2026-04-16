"""
CSV Logger node - Records mission telemetry data.
FIB UPC - Robotics Project

Subscribes to robot pose, mission state, and detection topics to generate
a CSV log file with: timestamp, current_phase, robot_pose (x, y, yaw),
and detected obstacle/station positions.
"""

import os
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from rob_project.utils import euler_from_quaternion


class CsvLogger(Node):
    """Node that logs mission telemetry to a CSV file."""

    def __init__(self):
        super().__init__('csv_logger')

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10
        )

        # State
        self.current_phase = 'INIT'
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.station_x = None
        self.station_y = None
        self.obstacle_count = 0

        # CSV setup
        log_dir = os.path.expanduser('~/rob_project_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'mission_log_{timestamp_str}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'phase', 'robot_x', 'robot_y', 'robot_yaw',
            'station_x', 'station_y', 'obstacle_count'
        ])

        self.get_logger().info(f'Logging to: {self.csv_path}')

        # Subscribers
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_best_effort
        )
        self.create_subscription(
            String, '/mission_state', self.phase_callback, qos_reliable
        )
        self.create_subscription(
            PoseStamped, '/station_pose', self.station_callback, qos_reliable
        )

        # Log timer - write a row every 0.5 seconds
        self.create_timer(0.5, self.log_timer_callback)

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def phase_callback(self, msg):
        """Update current mission phase."""
        self.current_phase = msg.data

    def station_callback(self, msg):
        """Update detected station pose."""
        self.station_x = msg.pose.position.x
        self.station_y = msg.pose.position.y

    def log_timer_callback(self):
        """Write a row to the CSV file."""
        self.csv_writer.writerow([
            time.time(),
            self.current_phase,
            f'{self.robot_x:.4f}',
            f'{self.robot_y:.4f}',
            f'{self.robot_yaw:.4f}',
            f'{self.station_x:.4f}' if self.station_x is not None else '',
            f'{self.station_y:.4f}' if self.station_y is not None else '',
            self.obstacle_count,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        """Clean up CSV file on shutdown."""
        self.csv_file.close()
        self.get_logger().info(f'CSV log saved: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('CSV Logger interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
