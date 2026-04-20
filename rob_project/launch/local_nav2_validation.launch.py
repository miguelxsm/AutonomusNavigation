"""Local validation stack: TurtleBot3 sim + SLAM + minimal Nav2 + optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    pkg_dir = get_package_share_directory('rob_project')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    use_warmup_motion = LaunchConfiguration('use_warmup_motion')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    params_file = LaunchConfiguration('params_file')

    rviz_config = os.path.join(tb3_nav2_dir, 'rviz', 'tb3_navigation2.rviz')
    default_params = os.path.join(tb3_nav2_dir, 'param', 'burger.yaml')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz with the TurtleBot3 Nav2 config',
    )
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo headless for CLI testing',
    )
    declare_use_warmup_motion = DeclareLaunchArgument(
        'use_warmup_motion', default_value='true',
        description='Rotate the robot briefly at startup to help SLAM initialize',
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value='turtlebot3_world.world',
        description='World file for headless sim or reference name',
    )
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='-2.0',
        description='Initial robot x position',
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='-0.5',
        description='Initial robot y position',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 parameter file to use',
    )

    headless_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'tb3_sim_headless.launch.py')
        ),
        condition=IfCondition(headless),
        launch_arguments={
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': 'true',
        }.items(),
    )

    # --- GUI sim: custom project map world ---
    project_world = os.path.join(pkg_dir, 'worlds', 'project_map.world')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_gazebo_launch_dir = os.path.join(tb3_gazebo_dir, 'launch')

    gui_gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(headless),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', project_world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gui_gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(headless),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
    )

    gui_robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_launch_dir, 'robot_state_publisher.launch.py')
        ),
        condition=UnlessCondition(headless),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    gui_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        condition=UnlessCondition(headless),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    gui_set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models'),
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    warmup_motion_node = Node(
        package='rob_project',
        executable='sim_warmup_motion',
        name='sim_warmup_motion',
        output='screen',
        parameters=[{
            'start_delay': 4.0,
            'motion_duration': 8.0,
            'angular_speed': 0.35,
            'use_sim_time': False,
        }],
        condition=IfCondition(use_warmup_motion),
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'minimal_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'use_respawn': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_headless,
        declare_use_warmup_motion,
        declare_world,
        declare_x_pose,
        declare_y_pose,
        declare_params_file,
        headless_sim,
        gui_set_env,
        gui_gz_server,
        gui_gz_client,
        gui_robot_state_pub,
        gui_spawn_robot,
        warmup_motion_node,
        slam_toolbox_node,
        nav2_stack,
        rviz_node,
    ])
