"""Publish a short startup rotation to help SLAM initialize in simulation."""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class SimWarmupMotion(Node):
    """Small scripted motion to create an initial map in simulation."""

    def __init__(self):
        super().__init__('sim_warmup_motion')

        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('motion_duration', 6.0)
        self.declare_parameter('angular_speed', 0.35)

        self.start_delay = float(self.get_parameter('start_delay').value)
        self.motion_duration = float(self.get_parameter('motion_duration').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.start_time = time.time()
        self.finished = False

        self.create_timer(0.1, self.control_loop, clock=self.clock)
        self.get_logger().info(
            'Warmup motion ready. '
            f'Delay={self.start_delay:.1f}s duration={self.motion_duration:.1f}s'
        )

    def publish_cmd(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.finished:
            return

        elapsed = time.time() - self.start_time
        if elapsed < self.start_delay:
            return

        if elapsed < self.start_delay + self.motion_duration:
            self.publish_cmd(self.angular_speed)
            return

        self.publish_cmd(0.0)
        self.finished = True
        self.get_logger().info('Warmup motion finished.')


def main(args=None):
    rclpy.init(args=args)
    node = SimWarmupMotion()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
