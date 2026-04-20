#!/usr/bin/env bash

set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
map_name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
output_dir="$script_dir/generated_maps"
output_path="$output_dir/$map_name"

mkdir -p "$output_dir"

cd "$script_dir"
source "$script_dir/source_local_env.sh"
source "$script_dir/install/setup.bash"

echo "Saving map to ${output_path}.yaml/.pgm"
exec ros2 run nav2_map_server map_saver_cli -f "$output_path"
