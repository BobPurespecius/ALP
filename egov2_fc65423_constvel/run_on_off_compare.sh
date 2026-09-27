#!/usr/bin/env bash
# 连续跑 合围 ON / OFF 两次，产出各自的日志与可见性 CSV
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
cd "$DIR"

cleanup() {
  for p in $(ps -ef | grep -E "[e]go_planner|[r]oslaunch|[r]osmaster|[t]raj_server|[n]ative_egov2|[t]arget_state|[q]uadrotor|[p]cl_render|[n]odelet|[t]eam_target|[r]viz|[b]ridge|[t]raj2odom|[s]o3_control|[o]dom_visual" | awk '{print $2}'); do
    kill -9 "$p" 2>/dev/null
  done
  sleep 4
}

run_one() {
  local tag="$1" enc="$2"
  echo "=========== [$tag] encirclement=$enc  $(date +%T) ==========="
  cleanup
  rm -f "$DIR/sim_${tag}.log"
  ROS_HOME=$DIR/.ros_home ROS_LOG_DIR=$DIR/.ros_home/log rosmaster --core -p 11361 > /tmp/rm_${tag}.log 2>&1 &
  for k in $(seq 1 25); do sleep 1; ss -ltn 2>/dev/null | grep -q 11361 && break; done
  ss -ltn 2>/dev/null | grep -q 11361 || { echo "master 起不来"; return 1; }
  NATIVE_EGOV2_LOG_FILE="$DIR/sim_${tag}.log" \
  NATIVE_EGOV2_VISIBILITY_CSV="$DIR/vis_${tag}.csv" \
  NATIVE_EGOV2_ENABLE_RVIZ=false NATIVE_EGOV2_WAIT_FOR_RVIZ=false \
  NATIVE_EGOV2_ENCIRCLEMENT="$enc" \
    timeout 420 ./run_round2_full_on.sh rviz:=false > /dev/null 2>&1
  echo "[$tag] 结束 $(date +%T)  log=$(stat -c%s $DIR/sim_${tag}.log 2>/dev/null) 字节"
  echo "  commits=$(grep -ac 'planner-traj-commit' $DIR/sim_${tag}.log)  mission_start=$(grep -ac 'startup-mission-start-observed' $DIR/sim_${tag}.log)"
}

run_one encON  true
run_one encOFF false
cleanup
echo "=========== 两次完成 $(date +%T) ==========="
ls -la "$DIR"/vis_enc*.csv 2>/dev/null
