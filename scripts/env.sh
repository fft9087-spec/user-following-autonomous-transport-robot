#!/usr/bin/env bash
# Source this file in every additional terminal: source scripts/env.sh
sim_ws="${CAPSTONE_WS:-$HOME/capstone_robot_ws}"
if [ ! -f "$sim_ws/install/setup.bash" ]; then echo 'Run bash scripts/build.sh first.'; return 1 2>/dev/null || exit 1; fi
source /opt/ros/humble/setup.bash
source "$sim_ws/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="$sim_ws/fastdds_udp_sim.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export GAZEBO_MODEL_DATABASE_URI=
