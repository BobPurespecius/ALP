#!/usr/bin/env bash
# 测试运行：显式补齐 rviz:=false 分支缺失的 sync_dynamic_motion_to_target_start:=true
# 并在运行中周期清理已轮转的 rosout 段，避免磁盘被吃满。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
TAG="${1:-test}"
cd "$DIR"
bash "$DIR/cleanup_own_nodes.sh" "$TAG" >/dev/null 2>&1
sleep 5
rm -rf "$DIR/.ros_home/log"/* 2>/dev/null
export DISPLAY=:0
export ROSCONSOLE_CONFIG_FILE="$DIR/rosconsole_quiet.cfg"

# 后台清理器：只清本目录 .ros_home 下已轮转的 rosout 段
( while true; do
    find "$DIR/.ros_home/log" -name 'rosout.log.[0-9]*' -delete 2>/dev/null
    sleep 20
  done ) &
CLEANER=$!

timeout 900 bash "$DIR/run_round2_full_on.sh" rviz:=false \
  sync_dynamic_motion_to_target_start:=true launch_scene:=true launch_experiment:=true \
  > "/tmp/${TAG}_run.log" 2>&1
RC=$?
kill $CLEANER 2>/dev/null
echo "exit=$RC"
