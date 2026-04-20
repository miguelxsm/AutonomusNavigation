"""
Mission Planner node - Central state machine for the 3-phase mission.
FIB UPC - Robotics Project

Orchestrates the full autonomous navigation mission:
  Phase I:   Navigate from Zona 1 -> Punt B -> Punt O -> Zona 2
  Phase II:  Explore Passadis, detect charging station, save SLAM map
  Phase III: Return to Punt Base, then precision park at station center
"""

import math
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from rob_project.utils import (
    euler_from_quaternion,
    create_pose_stamped,
)


class MissionPlanner(Node):
    """Central mission orchestrator with 3-phase state machine."""

    # Mission states
    STATE_INIT = 'INIT'
    STATE_PHASE_I = 'PHASE_I'
    STATE_PHASE_I_NAV = 'PHASE_I_NAV'
    STATE_PHASE_II = 'PHASE_II'
    STATE_PHASE_II_EXPLORE = 'PHASE_II_EXPLORE'
    STATE_PHASE_II_RETURN = 'PHASE_II_RETURN'
    STATE_SAVE_MAP = 'SAVE_MAP'
    STATE_PHASE_III = 'PHASE_III'
    STATE_PHASE_III_GOTO_BASE = 'PHASE_III_GOTO_BASE'
    STATE_PHASE_III_PARK = 'PHASE_III_PARK'
    STATE_DONE = 'MISSION_COMPLETE'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('mission_planner')

        self.cb_group = ReentrantCallbackGroup()

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10
        )

        # ---- Parameters (loaded from waypoints.yaml via launch) ----
        self.declare_parameter('punt_b.x', 3.72)
        self.declare_parameter('punt_b.y', 2.55)
        self.declare_parameter('punt_o.x', 5.10)
        self.declare_parameter('punt_o.y', 12.61)
        self.declare_parameter('punt_base.x', 5.00)
        self.declare_parameter('punt_base.y', 11.69)
        self.declare_parameter('passadis.x_min', 0.30)
        self.declare_parameter('passadis.x_max', 7.12)
        self.declare_parameter('passadis.y_min', 11.00)
        self.declare_parameter('passadis.y_max', 13.00)
        self.declare_parameter('passadis.exploration_step', 1.0)
        self.declare_parameter('map_save_path', '/tmp/slam_map')

        # Load waypoints
        self.punt_b = (
            self.get_parameter('punt_b.x').value,
            self.get_parameter('punt_b.y').value,
        )
        self.punt_o = (
            self.get_parameter('punt_o.x').value,
            self.get_parameter('punt_o.y').value,
        )
        self.punt_base = (
            self.get_parameter('punt_base.x').value,
            self.get_parameter('punt_base.y').value,
        )
        self.passadis_bounds = {
            'x_min': self.get_parameter('passadis.x_min').value,
            'x_max': self.get_parameter('passadis.x_max').value,
            'y_min': self.get_parameter('passadis.y_min').value,
            'y_max': self.get_parameter('passadis.y_max').value,
        }
        self.exploration_step = self.get_parameter(
            'passadis.exploration_step'
        ).value
        self.map_save_path = self.get_parameter('map_save_path').value

        # ---- State ----
        self.state = self.STATE_INIT
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.station_pose = None  # PoseStamped when detected
        self.station_detected = False
        self.emergency_stop = False
        self.nav_goal_active = False
        self.nav_goal_succeeded = False
        self.nav_goal_failed = False
        self.odom_received = False
        self.waiting_for_odom_logged = False
        self.current_waypoint_idx = 0
        self.phase_i_waypoints = []
        self.exploration_waypoints = []

        # ---- Nav2 Action Client ----
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )

        # ---- Subscribers ----
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_best_effort
        )
        self.create_subscription(
            PoseStamped, '/station_pose', self.station_pose_callback, qos_reliable
        )
        self.create_subscription(
            Bool, '/station_found', self.station_found_callback, qos_reliable
        )
        self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, qos_reliable
        )
        self.create_subscription(
            String, '/parking_status', self.parking_status_callback, qos_reliable
        )

        # ---- Publishers ----
        self.state_pub = self.create_publisher(
            String, '/mission_state', qos_reliable
        )
        self.station_enable_pub = self.create_publisher(
            Bool, '/station_detector/enable', qos_reliable
        )
        self.parking_goal_pub = self.create_publisher(
            PoseStamped, '/parking_goal', qos_reliable
        )

        # ---- Main loop at 2 Hz ----
        self.create_timer(0.5, self.mission_loop)

        self.get_logger().info('Mission Planner initialized. Waiting for Nav2...')

        # Wait for Nav2 action server
        self._wait_for_nav2()

    def _wait_for_nav2(self):
        """Wait for Nav2 navigate_to_pose action server."""
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if self.nav_to_pose_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().info('Nav2 action server available.')
            self.state = self.STATE_PHASE_I
        else:
            self.get_logger().error('Nav2 action server not available after 30s!')
            self.state = self.STATE_ERROR

    # ---- Callbacks ----

    def odom_callback(self, msg):
        """Update robot pose."""
        self.odom_received = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = euler_from_quaternion(msg.pose.pose.orientation)

    def station_pose_callback(self, msg):
        """Store detected station pose."""
        self.station_pose = msg

    def station_found_callback(self, msg):
        """Mark station as detected."""
        if msg.data:
            self.station_detected = True
            self.get_logger().info('Station detection confirmed by detector.')

    def emergency_callback(self, msg):
        """Handle emergency stop signals."""
        self.emergency_stop = msg.data

    def parking_status_callback(self, msg):
        """Handle parking status updates."""
        if msg.data == 'DONE':
            self.state = self.STATE_DONE
            self.get_logger().info('Parking complete! Mission accomplished.')
        elif msg.data == 'ABORTED':
            self.get_logger().error('Parking aborted! Attempting retry...')
            # Could retry or enter error state
            self.state = self.STATE_ERROR

    # ---- Navigation helpers ----

    def send_nav_goal(self, x, y, yaw=0.0):
        """Send a navigation goal to Nav2.

        Args:
            x, y: Target coordinates in map frame.
            yaw: Target orientation.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = create_pose_stamped(x, y, yaw, frame_id='map')
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Sending nav goal: ({x:.2f}, {y:.2f})')
        self.nav_goal_active = True
        self.nav_goal_succeeded = False
        self.nav_goal_failed = False

        future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav goal rejected!')
            self.nav_goal_active = False
            self.nav_goal_failed = True
            return

        self.get_logger().info('Nav goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        self.nav_goal_active = False
        self.nav_goal_succeeded = result.status == 4
        self.nav_goal_failed = not self.nav_goal_succeeded
        if self.nav_goal_succeeded:
            self.get_logger().info('Nav goal completed successfully.')
        else:
            self.get_logger().warn(
                f'Nav goal finished with status {result.status}.'
            )

    def can_start_navigation(self):
        """Return True only when odom is available for Nav2 usage."""
        if self.odom_received:
            return True

        if not self.waiting_for_odom_logged:
            self.get_logger().info('Waiting for /odom before starting mission...')
            self.waiting_for_odom_logged = True
        return False

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional logging)."""
        pass

    # ---- Exploration waypoint generation ----

    def generate_exploration_waypoints(self):
        """Generate lawnmower pattern waypoints for Passadis exploration.

        Creates a zigzag pattern covering the Passadis area.

        Returns:
            list[tuple]: List of (x, y) waypoints.
        """
        waypoints = []
        bounds = self.passadis_bounds
        step = self.exploration_step

        x = bounds['x_min'] + 0.5  # Offset from walls
        x_max = bounds['x_max'] - 0.5
        y_min = bounds['y_min'] + 0.3
        y_max = bounds['y_max'] - 0.3

        going_up = True
        while x <= x_max:
            if going_up:
                waypoints.append((x, y_min))
                waypoints.append((x, y_max))
            else:
                waypoints.append((x, y_max))
                waypoints.append((x, y_min))
            going_up = not going_up
            x += step

        self.get_logger().info(
            f'Generated {len(waypoints)} exploration waypoints.'
        )
        return waypoints

    # ---- Map saving ----

    def save_slam_map(self):
        """Save the current SLAM map using map_saver_cli."""
        self.get_logger().info(f'Saving SLAM map to {self.map_save_path}...')
        try:
            subprocess.run(
                [
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', self.map_save_path,
                    '--ros-args', '-p', 'save_map_timeout:=10.0',
                ],
                timeout=20,
                capture_output=True,
                text=True,
            )
            self.get_logger().info('Map saved successfully.')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Map save timed out.')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')

    # ---- Enable/disable station detector ----

    def set_station_detector(self, enabled):
        """Enable or disable the station detector node."""
        msg = Bool()
        msg.data = enabled
        self.station_enable_pub.publish(msg)
        self.get_logger().info(
            f'Station detector {"enabled" if enabled else "disabled"}.'
        )

    # ---- Main mission loop ----

    def mission_loop(self):
        """Central state machine executed at 2 Hz."""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # Don't proceed during emergency
        if self.emergency_stop and self.state not in (
            self.STATE_DONE, self.STATE_ERROR, self.STATE_INIT
        ):
            return

        if self.state not in (self.STATE_INIT, self.STATE_DONE, self.STATE_ERROR):
            if not self.can_start_navigation():
                return

        # ---- PHASE I: Navigate Zona1 -> B -> O -> Zona2 ----
        if self.state == self.STATE_PHASE_I:
            self.get_logger().info('=== PHASE I: Global Navigation ===')
            # Build waypoint list
            self.phase_i_waypoints = [
                self.punt_b,
                self.punt_o,
                self.punt_base,  # End of Phase I = arrive at Punt Base (Zona 2)
            ]
            self.current_waypoint_idx = 0
            self.state = self.STATE_PHASE_I_NAV
            # Send first waypoint
            wp = self.phase_i_waypoints[0]
            self.send_nav_goal(wp[0], wp[1])

        elif self.state == self.STATE_PHASE_I_NAV:
            if self.nav_goal_failed:
                self.get_logger().warn('Retrying current Phase I waypoint.')
                wp = self.phase_i_waypoints[self.current_waypoint_idx]
                self.send_nav_goal(wp[0], wp[1])
            elif self.nav_goal_succeeded and not self.nav_goal_active:
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx < len(self.phase_i_waypoints):
                    wp = self.phase_i_waypoints[self.current_waypoint_idx]
                    self.get_logger().info(
                        f'Phase I: waypoint {self.current_waypoint_idx + 1}/'
                        f'{len(self.phase_i_waypoints)}'
                    )
                    self.send_nav_goal(wp[0], wp[1])
                else:
                    self.get_logger().info('=== PHASE I COMPLETE ===')
                    self.state = self.STATE_PHASE_II

        # ---- PHASE II: Explore Passadis + Detect Station ----
        elif self.state == self.STATE_PHASE_II:
            self.get_logger().info('=== PHASE II: Exploration & Detection ===')
            self.set_station_detector(True)
            self.exploration_waypoints = self.generate_exploration_waypoints()
            self.current_waypoint_idx = 0
            self.state = self.STATE_PHASE_II_EXPLORE
            if self.exploration_waypoints:
                wp = self.exploration_waypoints[0]
                self.send_nav_goal(wp[0], wp[1])

        elif self.state == self.STATE_PHASE_II_EXPLORE:
            # Check if station was found - can stop exploring early
            if self.station_detected:
                self.get_logger().info(
                    'Station found during exploration! Returning to base.'
                )
                self.state = self.STATE_PHASE_II_RETURN
                self.send_nav_goal(self.punt_base[0], self.punt_base[1])
                return

            if self.nav_goal_failed:
                self.get_logger().warn('Retrying current exploration waypoint.')
                wp = self.exploration_waypoints[self.current_waypoint_idx]
                self.send_nav_goal(wp[0], wp[1])
            elif self.nav_goal_succeeded and not self.nav_goal_active:
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx < len(self.exploration_waypoints):
                    wp = self.exploration_waypoints[self.current_waypoint_idx]
                    self.get_logger().info(
                        f'Exploration: waypoint {self.current_waypoint_idx + 1}/'
                        f'{len(self.exploration_waypoints)}'
                    )
                    self.send_nav_goal(wp[0], wp[1])
                else:
                    self.get_logger().info('Exploration complete.')
                    if not self.station_detected:
                        self.get_logger().warn(
                            'Station NOT found during exploration! '
                            'Returning to base anyway.'
                        )
                    self.state = self.STATE_PHASE_II_RETURN
                    self.send_nav_goal(self.punt_base[0], self.punt_base[1])

        elif self.state == self.STATE_PHASE_II_RETURN:
            if self.nav_goal_failed:
                self.get_logger().warn('Retrying return to Punt Base.')
                self.send_nav_goal(self.punt_base[0], self.punt_base[1])
            elif self.nav_goal_succeeded and not self.nav_goal_active:
                self.get_logger().info('Back at Punt Base.')
                self.state = self.STATE_SAVE_MAP

        elif self.state == self.STATE_SAVE_MAP:
            self.save_slam_map()
            self.get_logger().info('=== PHASE II COMPLETE ===')
            if self.station_detected:
                self.state = self.STATE_PHASE_III
            else:
                self.get_logger().error(
                    'Cannot proceed to Phase III: station not detected.'
                )
                self.state = self.STATE_ERROR

        # ---- PHASE III: Precision Parking ----
        elif self.state == self.STATE_PHASE_III:
            self.get_logger().info('=== PHASE III: Precision Parking ===')
            self.set_station_detector(False)

            if self.station_pose is not None:
                # Navigate close to the station first using Nav2
                station_x = self.station_pose.pose.position.x
                station_y = self.station_pose.pose.position.y

                # Approach point: 0.5m in front of station
                station_yaw = euler_from_quaternion(
                    self.station_pose.pose.orientation
                )
                approach_x = station_x - 0.5 * math.cos(station_yaw)
                approach_y = station_y - 0.5 * math.sin(station_yaw)

                self.state = self.STATE_PHASE_III_GOTO_BASE
                self.send_nav_goal(approach_x, approach_y, station_yaw)

        elif self.state == self.STATE_PHASE_III_GOTO_BASE:
            if self.nav_goal_failed:
                self.get_logger().warn('Retrying approach to station.')
                station_x = self.station_pose.pose.position.x
                station_y = self.station_pose.pose.position.y
                station_yaw = euler_from_quaternion(
                    self.station_pose.pose.orientation
                )
                approach_x = station_x - 0.5 * math.cos(station_yaw)
                approach_y = station_y - 0.5 * math.sin(station_yaw)
                self.send_nav_goal(approach_x, approach_y, station_yaw)
            elif self.nav_goal_succeeded and not self.nav_goal_active:
                self.get_logger().info(
                    'Near station. Handing off to precision parking.'
                )
                self.state = self.STATE_PHASE_III_PARK
                # Publish station pose as parking goal
                self.parking_goal_pub.publish(self.station_pose)

        elif self.state == self.STATE_PHASE_III_PARK:
            # Waiting for parking_status_callback to set DONE or ERROR
            pass

        elif self.state == self.STATE_DONE:
            self.get_logger().info(
                '========================================\n'
                '  MISSION COMPLETE - Robot parked!\n'
                '========================================'
            )
            # Publish final state and stop the loop
            self.state = 'FINISHED'

        elif self.state == self.STATE_ERROR:
            self.get_logger().error('Mission in ERROR state.')

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Mission planner shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission planner interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
