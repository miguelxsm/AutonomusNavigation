"""
Full mission launch file.
Launches SLAM, Nav2, and all custom mission nodes.

Usage:
  ros2 launch rob_project mission_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rob_project')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    mission_params_file = os.path.join(pkg_dir, 'config', 'mission_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Launch arguments ----
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock',
    )

    # ---- SLAM Toolbox (online async) ----
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    # ---- Nav2 Bringup ----
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )

    # ---- Custom mission nodes ----
    mission_planner_node = Node(
        package='rob_project',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[
            mission_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    obstacle_detector_node = Node(
        package='rob_project',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[{
            'emergency_stop_distance': 0.15,
            'warning_distance': 0.40,
            'use_sim_time': use_sim_time,
        }],
    )

    station_detector_node = Node(
        package='rob_project',
        executable='station_detector',
        name='station_detector',
        output='screen',
        parameters=[{
            'station_side': 0.40,
            'pillar_diameter': 0.05,
            'enabled': False,
            'use_sim_time': use_sim_time,
        }],
    )

    precision_parking_node = Node(
        package='rob_project',
        executable='precision_parking',
        name='precision_parking',
        output='screen',
        parameters=[{
            'linear_speed': 0.05,
            'angular_speed': 0.2,
            'position_tolerance': 0.03,
            'angle_tolerance': 0.08,
            'use_sim_time': use_sim_time,
        }],
    )

    csv_logger_node = Node(
        package='rob_project',
        executable='csv_logger',
        name='csv_logger',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_sim_time,
        slam_toolbox_node,
        nav2_launch,
        mission_planner_node,
        obstacle_detector_node,
        station_detector_node,
        precision_parking_node,
        csv_logger_node,
    ])
