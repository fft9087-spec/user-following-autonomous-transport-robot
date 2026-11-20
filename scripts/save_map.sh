#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/env.sh"
mkdir -p "$root/outputs/maps"
name="${1:-corridor_map}"
exec ros2 run nav2_map_server map_saver_cli -f "$root/outputs/maps/$name" --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0 -p map_subscribe_transient_local:=true
