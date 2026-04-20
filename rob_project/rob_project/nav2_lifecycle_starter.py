"""Sequential lifecycle starter for local Nav2 validation."""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2LifecycleStarter(Node):
    """Configure and activate Nav2 nodes sequentially with generous waits."""

    def __init__(self):
        super().__init__('nav2_lifecycle_starter')

        self.declare_parameter(
            'node_names',
            [
                'slam_toolbox',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'velocity_smoother',
                'bt_navigator',
                'waypoint_follower',
            ],
        )
        self.declare_parameter('start_delay', 8.0)
        self.declare_parameter('service_wait_timeout', 30.0)
        self.declare_parameter('transition_timeout', 30.0)

        self.node_names = list(self.get_parameter('node_names').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.service_wait_timeout = float(
            self.get_parameter('service_wait_timeout').value
        )
        self.transition_timeout = float(
            self.get_parameter('transition_timeout').value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def run(self):
        """Run the startup sequence."""
        self.get_logger().info(
            f'Waiting {self.start_delay:.1f}s before lifecycle transitions...'
        )
        time.sleep(self.start_delay)

        for node_name in self.node_names:
            if node_name == 'planner_server':
                if not self.wait_for_map_tf():
                    self.get_logger().error(
                        'map frame did not become available in time.'
                    )
                    return False
            if not self.configure_and_activate(node_name):
                self.get_logger().error(f'Failed to bring up {node_name}.')
                return False

        self.get_logger().info('All requested Nav2 nodes are active.')
        return True

    def wait_for_map_tf(self):
        """Wait until SLAM publishes a usable map transform."""
        deadline = time.time() + self.transition_timeout
        self.get_logger().info(
            'Waiting for SLAM to publish map -> base_footprint/base_link TF...'
        )

        while time.time() < deadline:
            for frame in ('base_footprint', 'base_link'):
                try:
                    if self.tf_buffer.can_transform(
                        'map', frame, rclpy.time.Time()
                    ):
                        self.get_logger().info(
                            f'Map TF available using frame {frame}.'
                        )
                        return True
                except TransformException:
                    pass

            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)

        return False

    def configure_and_activate(self, node_name):
        """Bring a lifecycle node to ACTIVE state."""
        change_client = self.create_client(ChangeState, f'/{node_name}/change_state')
        state_client = self.create_client(GetState, f'/{node_name}/get_state')

        if not change_client.wait_for_service(timeout_sec=self.service_wait_timeout):
            self.get_logger().error(
                f'ChangeState service not available for {node_name}.'
            )
            return False

        if not state_client.wait_for_service(timeout_sec=self.service_wait_timeout):
            self.get_logger().error(
                f'GetState service not available for {node_name}.'
            )
            return False

        state_id = self.get_state(state_client, node_name)
        if state_id is None:
            return False

        if state_id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f'{node_name} already active.')
            return True

        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info(f'Configuring {node_name}...')
            if not self.change_state(
                change_client,
                node_name,
                Transition.TRANSITION_CONFIGURE,
                'configure',
            ):
                return False
            state_id = self.wait_for_states(
                state_client,
                node_name,
                [
                    State.PRIMARY_STATE_INACTIVE,
                    State.PRIMARY_STATE_ACTIVE,
                ],
            )
            if state_id is None:
                return False

            if state_id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f'{node_name} became active directly.')
                return True

        state_id = self.get_state(state_client, node_name)
        if state_id is None:
            return False

        if state_id == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info(f'Activating {node_name}...')
            if not self.change_state(
                change_client,
                node_name,
                Transition.TRANSITION_ACTIVATE,
                'activate',
            ):
                return False
            return self.wait_for_states(
                state_client,
                node_name,
                [State.PRIMARY_STATE_ACTIVE],
            ) is not None

        if state_id == State.PRIMARY_STATE_ACTIVE:
            return True

        self.get_logger().error(
            f'{node_name} ended in unexpected lifecycle state id {state_id}.'
        )
        return False

    def get_state(self, state_client, node_name):
        """Query the current lifecycle state."""
        request = GetState.Request()
        future = state_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.transition_timeout
        )

        if not future.done() or future.result() is None:
            self.get_logger().error(f'Could not query lifecycle state of {node_name}.')
            return None

        state = future.result().current_state
        self.get_logger().info(f'{node_name} state: {state.label} ({state.id})')
        return state.id

    def change_state(self, change_client, node_name, transition_id, label):
        """Request a lifecycle transition."""
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = change_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.transition_timeout
        )

        if not future.done() or future.result() is None:
            self.get_logger().error(f'{label} request timed out for {node_name}.')
            return False

        if not future.result().success:
            self.get_logger().warn(
                f'{label} request reported failure for {node_name}; '
                'will verify actual lifecycle state before aborting.'
            )

        return True

    def wait_for_states(self, state_client, node_name, expected_states):
        """Poll until the node reaches one of the expected states."""
        deadline = time.time() + self.transition_timeout
        while time.time() < deadline:
            state_id = self.get_state(state_client, node_name)
            if state_id in expected_states:
                return state_id
            time.sleep(0.5)

        self.get_logger().error(
            f'{node_name} did not reach expected states {expected_states}.'
        )
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LifecycleStarter()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
