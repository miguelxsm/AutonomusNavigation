"""Launch the relative Nav2 goal node with sim time enabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    robot_frame = LaunchConfiguration('robot_frame')

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='0.6'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        Node(
            package='rob_project',
            executable='relative_goal_nav',
            name='relative_goal_nav',
            output='both',
            emulate_tty=True,
            parameters=[{
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_yaw': goal_yaw,
                'robot_frame': robot_frame,
                'fallback_robot_frame': 'base_footprint',
                'use_sim_time': True,
            }],
        ),
    ])
