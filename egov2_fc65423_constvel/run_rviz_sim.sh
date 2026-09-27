#!/usr/bin/env bash
# !!! 清理只针对本仓库/native_egov2 的节点；若 RRCT 等其他仿真在跑，
# !!! 严禁使用宽泛 pkill/pgrep，必须按 PID 前缀或 ROS_MASTER_URI 隔离。
# 带 RViz 的一次正常仿真：long_cylinder_forest.json，FULL ON。
# 注意：清理逻辑不得匹配本脚本自身的命令行（TAG 里可能含 "rviz"），
# 因此用明确的节点可执行名做 pgrep，并排除自身与父进程。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
TAG="${1:-rviz1}"
KEEP_RE='los-window-semantics|los-plane-audit|los-occlusion-interval|clearance-gain-decision|clearance-gain-audit|los-occlusion-audit|planner-traj-commit|continuous-motion-activation|execution-handoff|prefix-progress|fresh-time-allocation|fresh-init-snapshot|soft-visibility|visibility-topology|side-both-failed|timebase-audit|los-plane-audit|swarm-violation|terminal-hold|TERMINAL_HOLD|encirclement-hypothesis-planning|replan-retry-cap|MOVING_ROLLING_RETRY_CAP|dynamic-prediction|DYNAMIC_PREDICTION|execution-safety|INVALIDATED|collision|COLLISION|ERROR|WARN|FATAL|SIGSEGV|process has died|rviz-startup|mission'

cd "$DIR"
SELF=$$
PARENT=$PPID
bash "$DIR/cleanup_own_nodes.sh" "$TAG"
sleep 5

rm -rf "$DIR/.ros_home/log"/* 2>/dev/null
rm -f "$DIR/vis_${TAG}.csv" "$DIR/vis_${TAG}_trajectory.csv"

export DISPLAY="${DISPLAY:-:0}"
export NATIVE_EGOV2_ENABLE_RVIZ=true
export NATIVE_EGOV2_WAIT_FOR_RVIZ=true

echo "=== [$TAG] start $(date +%T) DISPLAY=$DISPLAY ==="
NATIVE_EGOV2_LOG_FILE=/dev/null \
NATIVE_EGOV2_VISIBILITY_CSV="$DIR/vis_${TAG}.csv" \
timeout 900 "$DIR/run_round2_full_on.sh" rviz:=true 2>&1 \
  | grep -aE "$KEEP_RE" > "$DIR/sim_${TAG}.log"
echo "=== [$TAG] end $(date +%T) ==="
echo "commits=$(grep -ac 'planner-traj-commit' "$DIR/sim_${TAG}.log" 2>/dev/null)"
echo "traj_rows=$(wc -l < "$DIR/vis_${TAG}_trajectory.csv" 2>/dev/null)"
df -h /home/bob | tail -1
