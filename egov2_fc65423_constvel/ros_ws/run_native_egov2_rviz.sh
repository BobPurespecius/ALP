#!/usr/bin/env bash
set -e

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export EGO_PLANNER_VARIANT=egov2
export ROS_MASTER_URI="${NATIVE_EGOV2_ROS_MASTER_URI:-http://127.0.0.1:11321}"
export NATIVE_EGOV2_VISIBILITY_CSV="${NATIVE_EGOV2_VISIBILITY_CSV:-$workspace_dir/native_egov2_visibility.csv}"
source "$workspace_dir/setup_runtime_env.sh" >/dev/null 2>&1

ego_v2_home="$workspace_dir/src/EGO-Planner-v2/swarm-playground/tracking_ws"
export CMAKE_PREFIX_PATH="$ego_v2_home/devel:$workspace_dir/devel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$ego_v2_home/devel/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

launch_file="$workspace_dir/src/multi_uav_formation/launch/native_egov2_rviz.launch"
log_file="$workspace_dir/native_egov2_rviz.log"

printf 'Starting native EGOv2 RViz simulation (logs: %s)\n' "$log_file"
exec roslaunch "$launch_file" "$@" >"$log_file" 2>&1
