#!/usr/bin/env bash
# 不带 RViz 连续跑仿真，抓规划器的 SIGSEGV backtrace
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
cd "$DIR"
for i in 1 2 3; do
  echo "=================== 第 $i 次尝试 $(date +%T) ==================="
  for p in $(pgrep -x rosmaster); do kill $p 2>/dev/null; done
  sleep 2
  rm -f /tmp/gdbrun_*.log sim_run_catch_$i.log
  ROS_HOME=$DIR/.ros_home ROS_LOG_DIR=$DIR/.ros_home/log rosmaster --core -p 11361 > /tmp/rm_catch_$i.log 2>&1 &
  for k in $(seq 1 20); do sleep 1; ss -ltn 2>/dev/null | grep -q 11361 && break; done
  ss -ltn 2>/dev/null | grep -q 11361 || { echo "master 起不来，跳过"; continue; }
  NATIVE_EGOV2_LOG_FILE=$DIR/sim_run_catch_$i.log \
  NATIVE_EGOV2_ENABLE_RVIZ=false NATIVE_EGOV2_WAIT_FOR_RVIZ=false \
    timeout 300 ./run_round2_full_on.sh rviz:=false > $DIR/sim_run_catch_${i}_runner.log 2>&1
  echo "run $i 结束 $(date +%T)"
  # 检查是否抓到栈
  if grep -ql "SIGSEGV CAUGHT\|SIGABRT CAUGHT" /tmp/gdbrun_*.log 2>/dev/null; then
     echo ">>> 第 $i 次命中崩溃，backtrace 已保存"
     break
  fi
  if grep -q "process has died" $DIR/sim_run_catch_$i.log 2>/dev/null; then
     echo ">>> 第 $i 次有进程死亡但 gdb 未捕获（可能是 exit code -6 或其它）"
  fi
done
echo "=================== 全部尝试结束 $(date +%T) ==================="
ls -la /tmp/gdbrun_*.log 2>/dev/null
