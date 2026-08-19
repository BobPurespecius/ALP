#!/usr/bin/env bash
set -euo pipefail

experiment_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_ws="$experiment_dir/ros_ws"
tracking_ws="$ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws"
base_ros_ws="/home/bob/ALP/guidance/ros_ws"

source /opt/ros/noetic/setup.bash
source "$base_ros_ws/devel/setup.bash"
source "$ros_ws/devel/setup.bash"

export EGO_PLANNER_VARIANT=egov2
export ROS_MASTER_URI="${NATIVE_EGOV2_ROS_MASTER_URI:-http://127.0.0.1:11361}"
export ROS_PACKAGE_PATH="$ros_ws/src:$tracking_ws/src:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$ros_ws/devel:$base_ros_ws/devel:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$ros_ws/devel/lib:$base_ros_ws/devel/lib:${LD_LIBRARY_PATH:-}"
export NATIVE_EGOV2_EXPERIMENT_DIR="$experiment_dir"
export NATIVE_EGOV2_VISIBILITY_CSV="${NATIVE_EGOV2_VISIBILITY_CSV:-$experiment_dir/constvel_visibility.csv}"
visibility_stem="${NATIVE_EGOV2_VISIBILITY_CSV%.*}"
export NATIVE_EGOV2_TRAJECTORY_CSV="${NATIVE_EGOV2_TRAJECTORY_CSV:-${visibility_stem}_trajectory.csv}"

early_avoidance_enabled="${NATIVE_EGOV2_EARLY_AVOIDANCE:-false}"
moving_obstacle_time_aware_cost_enabled="${NATIVE_EGOV2_MOVING_COST:-true}"

launch_file="$ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch"
log_file="${NATIVE_EGOV2_LOG_FILE:-$experiment_dir/constvel_rviz.log}"

printf 'Starting isolated EGOv2 constant-velocity gradient simulation (logs: %s)\n' "$log_file"
exec roslaunch "$launch_file" \
  early_avoidance_enabled:="$early_avoidance_enabled" \
  moving_obstacle_time_aware_cost_enabled:="$moving_obstacle_time_aware_cost_enabled" \
  "$@" >"$log_file" 2>&1
