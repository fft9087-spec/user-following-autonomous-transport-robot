#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-nav}"
case "$mode" in view|slam|nav) ;; *) echo 'Usage: bash scripts/run.sh [view|slam|nav] [launch arguments]'; exit 2;; esac
if [ "$#" -gt 0 ]; then shift; fi
source "$root/scripts/env.sh"
if pgrep -x gzserver >/dev/null; then echo 'Gazebo is already running. Close its launch before starting another.'; exit 1; fi
exec ros2 launch physical_ai_robot_description cad_v11.launch.py "mode:=$mode" "$@"
