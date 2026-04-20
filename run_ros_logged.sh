#!/usr/bin/env bash

set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "Usage: ./run_ros_logged.sh <command> [args...]" >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
timestamp="$(date +%Y%m%d_%H%M%S)"
session_dir="$script_dir/runtime_logs/$timestamp"
console_log="$session_dir/console.log"

mkdir -p "$session_dir/ros"
ln -sfn "$session_dir" "$script_dir/runtime_logs/latest"

export ROS_LOG_DIR="$session_dir/ros"

cd "$script_dir"

set +u
source "$script_dir/source_local_env.sh"

if [ -f "$script_dir/install/setup.bash" ]; then
  source "$script_dir/install/setup.bash"
fi
set -u

{
  echo "Timestamp: $timestamp"
  echo "Command: $*"
  echo "ROS_LOG_DIR: $ROS_LOG_DIR"
  echo "Working directory: $script_dir"
  echo
} | tee "$console_log"

set +e
"$@" 2>&1 | tee -a "$console_log"
status=${PIPESTATUS[0]}
set -e

{
  echo
  echo "Exit status: $status"
  echo "Console log: $console_log"
  echo "ROS logs dir: $ROS_LOG_DIR"
} | tee -a "$console_log"

exit "$status"
