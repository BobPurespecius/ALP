#!/usr/bin/env bash
# ROS 与 probe 日志全部写入本次 runs/ 目录，避免 tmpfs 累积占用内存。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
cd "$DIR"
RUN_DIR="$DIR/runs/probe_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN_DIR/ros_home" "$RUN_DIR/ros_log"
bash "$DIR/cleanup_own_nodes.sh" probe >/dev/null 2>&1
sleep 5
export DISPLAY=:0
export ROS_HOME="$RUN_DIR/ros_home"
export ROS_LOG_DIR="$RUN_DIR/ros_log"
export NATIVE_EGOV2_LOG_FILE=/dev/null
export NATIVE_EGOV2_EXPERIMENT_DIR="$DIR"
export NATIVE_EGOV2_VISIBILITY_CSV="$DIR/probe_visibility.csv"
export NATIVE_EGOV2_TRAJECTORY_CSV="$DIR/probe_trajectory.csv"
export CMAKE_PREFIX_PATH="$DIR/ros_ws/devel:/home/bob/ALP/guidance/ros_ws/devel:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$DIR/ros_ws/devel/lib:/home/bob/ALP/guidance/ros_ws/devel/lib:${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="$DIR/ros_ws/src:$DIR/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src:${ROS_PACKAGE_PATH:-}"
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source "$DIR/ros_ws/devel/setup.bash" >/dev/null 2>&1
export ROS_MASTER_URI=http://127.0.0.1:11361
roslaunch "$DIR/ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch" \
  rviz:=false launch_scene:=true launch_experiment:=true \
  sync_dynamic_motion_to_target_start:=true \
  scene_file:="$DIR/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json" \
  > "$RUN_DIR/probe_run.log" 2>&1 &
LP=$!
LP=$!
sleep 75
echo "===== 话题存在性 ====="
rostopic list 2>/dev/null | grep -E "broadcast|tracking_ready|motion_start" | head -10
echo "===== 广播发布率(5s) ====="
timeout 8 rostopic hz /broadcast_traj_from_planner 2>&1 | head -3
echo "===== 接收侧发布率(5s) ====="
timeout 8 rostopic hz /broadcast_traj_to_planner 2>&1 | head -3
echo "===== tracking_ready 各机 ====="
for u in uav0 uav1 uav2; do printf "%s: " $u; timeout 4 rostopic echo -n1 /target_tracking/tracking_ready/$u 2>&1 | head -1; done
echo "===== 运行进程 ====="
pgrep -c -f ego_planner_node
kill -INT $LP 2>/dev/null
exit 0
