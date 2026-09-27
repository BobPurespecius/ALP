#!/usr/bin/env bash
# ROS 与 runner 日志写入本次 runs/ 目录，避免 tmpfs 累积占用内存。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
cd "$DIR"
RUN_DIR="$DIR/runs/final3_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN_DIR/ros_home" "$RUN_DIR/ros_log"
bash "$DIR/cleanup_own_nodes.sh" final3 >/dev/null 2>&1
sleep 5
export DISPLAY=:0
export ROS_HOME="$RUN_DIR/ros_home"
export ROS_LOG_DIR="$RUN_DIR/ros_log"
export NATIVE_EGOV2_LOG_FILE=/dev/null
export NATIVE_EGOV2_VISIBILITY_CSV="$DIR/constvel_visibility.csv"
export NATIVE_EGOV2_SCENE_FILE="$DIR/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json"
export NATIVE_EGOV2_ROS_MASTER_URI="http://127.0.0.1:11361"
export NATIVE_EGOV2_ENABLE_RVIZ=false NATIVE_EGOV2_WAIT_FOR_RVIZ=false
export ROSCONSOLE_CONFIG_FILE="$DIR/rosconsole_quiet.cfg"
timeout 900 bash "$DIR/run_constvel_gradient_rviz.sh" rviz:=false > "$RUN_DIR/final3_run.log" 2>&1
echo "exit=$?"
