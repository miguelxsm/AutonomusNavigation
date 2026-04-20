#!/usr/bin/env bash

set +e

patterns=(
  "ros2 launch rob_project local_nav2_validation.launch.py"
  "ros2 launch rob_project relative_goal_test.launch.py"
  "ros2 run turtlebot3_teleop teleop_keyboard"
  "ruby /usr/bin/gz sim"
  "ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim"
  "gz sim"
  "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"
  "/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher"
  "/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node"
  "/opt/ros/jazzy/lib/nav2_controller/controller_server"
  "/opt/ros/jazzy/lib/nav2_smoother/smoother_server"
  "/opt/ros/jazzy/lib/nav2_planner/planner_server"
  "/opt/ros/jazzy/lib/nav2_behaviors/behavior_server"
  "/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator"
  "/opt/ros/jazzy/lib/nav2_waypoint_follower/waypoint_follower"
  "/opt/ros/jazzy/lib/nav2_velocity_smoother/velocity_smoother"
  "/home/miguel/Escritorio/ROB/project/install/rob_project/lib/rob_project/nav2_lifecycle_starter"
  "/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager"
)

for pattern in "${patterns[@]}"; do
  pkill -f "$pattern"
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -9 -f "$pattern"
done

sleep 1

echo "Local ROS/Gazebo processes cleaned."
