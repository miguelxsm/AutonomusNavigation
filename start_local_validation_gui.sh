#!/usr/bin/env bash

set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$script_dir"
./cleanup_ros_local.sh
source "$script_dir/source_local_env.sh"
colcon build --packages-select rob_project
source "$script_dir/install/setup.bash"
exec "$script_dir/run_ros_logged.sh" ros2 launch rob_project local_nav2_validation.launch.py headless:=false use_rviz:=true
