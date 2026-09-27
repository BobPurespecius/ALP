#!/usr/bin/env bash
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
cd "$DIR"
bash "$DIR/cleanup_own_nodes.sh" final2 >/dev/null 2>&1
sleep 5
rm -rf "$DIR/.ros_home/log"/* 2>/dev/null
export DISPLAY=:0
export NATIVE_EGOV2_LOG_FILE=/dev/null
# 关键：不让 rosout 落盘（前几次失败都是 rosout 写满磁盘导致任务停住）
roslaunch --disable-rosout-logs "$DIR/ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch" \
  rviz:=false launch_scene:=true launch_experiment:=true \
  sync_dynamic_motion_to_target_start:=true \
  scene_file:="$DIR/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json" \
  > /tmp/final2_run.log 2>&1
