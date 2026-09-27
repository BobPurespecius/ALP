#!/usr/bin/env bash
# 已验证成功的运行路径（rviz:=true）+ 后台 rosout 清理，防磁盘被吃满。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
TAG="${1:-final}"
cd "$DIR"
bash "$DIR/cleanup_own_nodes.sh" "$TAG" >/dev/null 2>&1
sleep 5
rm -rf "$DIR/.ros_home/log"/* 2>/dev/null
export DISPLAY=:0
( while true; do
    find "$DIR/.ros_home/log" -name 'rosout.log.[0-9]*' -delete 2>/dev/null
    sleep 20
  done ) &
CLEANER=$!
timeout 900 bash "$DIR/run_round2_full_on.sh" rviz:=true > "/tmp/${TAG}_run.log" 2>&1
RC=$?
kill $CLEANER 2>/dev/null
echo "exit=$RC"
