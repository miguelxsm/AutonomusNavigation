#!/usr/bin/env bash

set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$script_dir"
source "$script_dir/source_local_env.sh"
exec ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/cmd_vel_nav
