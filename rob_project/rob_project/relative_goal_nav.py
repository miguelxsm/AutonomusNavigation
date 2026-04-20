"""Send a Nav2 goal relative to the robot start pose."""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from rob_project.utils import create_pose_stamped, euler_from_quaternion


class RelativeGoalNavigator(Node):
    """Capture the initial pose and send a relative Nav2 goal."""

    def __init__(self):
        super().__init__('relative_goal_navigator')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('use_start_orientation', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('fallback_robot_frame', 'base_footprint')
        self.declare_parameter('action_name', '/navigate_to_pose')

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.use_start_orientation = bool(
            self.get_parameter('use_start_orientation').value
        )
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.fallback_robot_frame = self.get_parameter(
            'fallback_robot_frame'
        ).value
        self.action_name = self.get_parameter('action_name').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)

        self.start_pose = None
        self.goal_sent = False
        self.finished = False
        self.active_robot_frame = self.robot_frame
        self.last_wait_reason = None

        self.timer = self.create_timer(
            0.2,
            self.control_loop,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

        self.get_logger().info(
            'Relative navigator ready. Waiting for TF and Nav2 action server...'
        )

    def control_loop(self):
        """Wait for the initial transform and then send one goal."""
        if self.finished or self.goal_sent:
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self.log_wait_reason(
                f'Waiting for Nav2 action server: {self.action_name}'
            )
            return

        if self.start_pose is None:
            self.start_pose = self.lookup_current_pose()
            if self.start_pose is None:
                self.log_wait_reason(
                    'Action server ready. Waiting for TF '
                    f'{self.global_frame}->{self.robot_frame}'
                    + (
                        '' if self.fallback_robot_frame == self.robot_frame else
                        f' or {self.global_frame}->{self.fallback_robot_frame}'
                    )
                )
                return

            self.last_wait_reason = None
            start_x, start_y, start_yaw = self.start_pose
            self.get_logger().info(
                'Captured start pose in %s: x=%.3f y=%.3f yaw=%.3f rad'
                % (self.global_frame, start_x, start_y, start_yaw)
            )

        self.send_relative_goal()

    def lookup_current_pose(self):
        """Read the current robot pose from TF."""
        candidate_frames = [self.robot_frame]
        if self.fallback_robot_frame and \
           self.fallback_robot_frame != self.robot_frame:
            candidate_frames.append(self.fallback_robot_frame)

        transform = None
        for frame in candidate_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    frame,
                    rclpy.time.Time(),
                )
                if frame != self.active_robot_frame:
                    self.active_robot_frame = frame
                    self.get_logger().info(
                        f'Using robot frame: {self.active_robot_frame}'
                    )
                break
            except TransformException as exc:
                self.get_logger().debug(
                    f'TF not ready yet for {frame}: {exc}'
                )

        if transform is None:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = euler_from_quaternion(rotation)
        return (translation.x, translation.y, yaw)

    def send_relative_goal(self):
        """Convert relative coordinates into a map-frame Nav2 goal."""
        start_x, start_y, start_yaw = self.start_pose

        if self.use_start_orientation:
            goal_abs_x = (
                start_x
                + self.goal_x * math.cos(start_yaw)
                - self.goal_y * math.sin(start_yaw)
            )
            goal_abs_y = (
                start_y
                + self.goal_x * math.sin(start_yaw)
                + self.goal_y * math.cos(start_yaw)
            )
            goal_abs_yaw = start_yaw + self.goal_yaw
        else:
            goal_abs_x = start_x + self.goal_x
            goal_abs_y = start_y + self.goal_y
            goal_abs_yaw = self.goal_yaw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = create_pose_stamped(
            goal_abs_x,
            goal_abs_y,
            goal_abs_yaw,
            frame_id=self.global_frame,
            stamp=self.get_clock().now().to_msg(),
        )

        self.goal_sent = True
        self.get_logger().info(
            'Sending relative goal dx=%.3f dy=%.3f dyaw=%.3f -> '
            'absolute x=%.3f y=%.3f yaw=%.3f'
            % (
                self.goal_x,
                self.goal_y,
                self.goal_yaw,
                goal_abs_x,
                goal_abs_y,
                goal_abs_yaw,
            )
        )

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def log_wait_reason(self, reason):
        """Log waiting state only when it changes."""
        if reason != self.last_wait_reason:
            self.last_wait_reason = reason
            self.get_logger().info(reason)

    def goal_response_callback(self, future):
        """Handle action goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal.')
            self.finished = True
            return

        self.get_logger().info('Nav2 accepted the goal.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle final action result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully.')
        else:
            self.get_logger().error(f'Navigation finished with status {status}.')

        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = RelativeGoalNavigator()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
