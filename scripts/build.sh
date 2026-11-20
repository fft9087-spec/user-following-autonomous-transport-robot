#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sim_ws="${CAPSTONE_WS:-$HOME/capstone_robot_ws}"
test -f /opt/ros/humble/setup.bash || { echo 'ROS 2 Humble is required. See README.md.'; exit 1; }
if [ "$sim_ws" = "$root" ]; then echo 'Choose a separate Linux build workspace.'; exit 2; fi
mkdir -p "$sim_ws/src/physical_ai_robot_description"
cp -a "$root/src/physical_ai_robot_description/." "$sim_ws/src/physical_ai_robot_description/"
cp "$root/config/fastdds_udp_sim.xml" "$sim_ws/fastdds_udp_sim.xml"
source /opt/ros/humble/setup.bash
cd "$sim_ws"
colcon build --packages-select physical_ai_robot_description --symlink-install
printf '\nBuilt workspace: %s\n' "$sim_ws"
