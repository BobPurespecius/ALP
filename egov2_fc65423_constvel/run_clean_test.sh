#!/usr/bin/env bash
# 标准测试运行：ROS 与 runner 日志写入本次 runs/ 目录，不写 tmpfs。
# 启动前强制校验 UDP 8081 无残留（SO_REUSEPORT 残留会让轨迹广播被轮询分发）。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
cd "$DIR"
RUN_DIR="$DIR/runs/clean_test_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN_DIR/ros_home" "$RUN_DIR/ros_log"
bash "$DIR/cleanup_own_nodes.sh" cleantest
export DISPLAY=:0
export ROS_LOG_DIR="$RUN_DIR/ros_log"
export ROS_HOME="$RUN_DIR/ros_home"
export NATIVE_EGOV2_LOG_FILE=/dev/null
export NATIVE_EGOV2_VISIBILITY_CSV="$DIR/constvel_visibility.csv"
timeout 900 bash "$DIR/run_round2_full_on.sh" rviz:=true > "$RUN_DIR/cleantest_run.log" 2>&1
echo "exit=$?"
