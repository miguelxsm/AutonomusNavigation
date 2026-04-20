#!/usr/bin/env bash

set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

goal_x="${1:-0.6}"
goal_y="${2:-0.0}"
goal_yaw="${3:-0.0}"

cd "$script_dir"
source "$script_dir/source_local_env.sh"
source "$script_dir/install/setup.bash"
exec "$script_dir/run_ros_logged.sh" ros2 launch rob_project relative_goal_test.launch.py \
  goal_x:="$goal_x" goal_y:="$goal_y" goal_yaw:="$goal_yaw"
