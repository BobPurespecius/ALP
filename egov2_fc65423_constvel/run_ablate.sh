#!/usr/bin/env bash
# !!! 清理只针对本仓库/native_egov2 的节点；若 RRCT 等其他仿真在跑，
# !!! 严禁使用宽泛 pkill/pgrep，必须按 PID 前缀或 ROS_MASTER_URI 隔离。
# 本轮（fix4）真实仿真：long_cylinder_forest.json，FULL ON。
# 磁盘控制：runner 的 log_file 指向 /dev/null，stdout 经白名单过滤后写入
# sim_<TAG>.log，避免 minco-gradient-audit 之类的刷屏把日志推到 GB 级。
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
TAG="${1:-ablate}"
EXTRA="${2:-}"
KEEP_RE='planner-traj-commit|continuous-motion-activation|execution-handoff|prefix-progress|fresh-time-allocation|fresh-init-snapshot|soft-visibility|visibility-topology|side-both-failed|timebase-audit|los-plane-audit|collision|COLLISION|SWARM|swarm-violation|terminal-hold|TERMINAL_HOLD|encirclement-hypothesis-planning|joint-|JOINT|replan-retry-cap|MOVING_ROLLING_RETRY_CAP|dynamic-prediction|DYNAMIC_PREDICTION|execution-safety|INVALIDATED|low-speed|motion-continuity|first-stop-and-go|ERROR|WARN|FATAL|SIGSEGV|process has died'

cd "$DIR"
for p in $(ps -ef | grep -E "[e]go_planner|[r]oslaunch|[r]osmaster|[t]raj_server|[n]ative_egov2|[q]uadrotor|[p]cl_render|[n]odelet|[t]eam_target|[b]ridge|[t]raj2odom|[s]o3_control|[o]dom_visual|[r]viz" | awk '{print $2}'); do
  kill -9 "$p" 2>/dev/null
done
sleep 5

rm -rf "$DIR/.ros_home/log"/* 2>/dev/null
rm -f "$DIR/vis_${TAG}.csv" "$DIR/vis_${TAG}_trajectory.csv"

echo "=== [$TAG] start $(date +%T) ==="
# log_file=/dev/null：runner 的所有 tee/重定向都被丢弃，只保留 stdout。
NATIVE_EGOV2_LOG_FILE=/dev/null \
NATIVE_EGOV2_VISIBILITY_CSV="$DIR/vis_${TAG}.csv" \
timeout 600 "$DIR/run_round2_full_on.sh" rviz:=false $EXTRA 2>&1 \
  | grep -aE "$KEEP_RE" > "$DIR/sim_${TAG}.log"
echo "=== [$TAG] end $(date +%T) ==="
echo "log_bytes=$(stat -c%s "$DIR/sim_${TAG}.log" 2>/dev/null)"
echo "traj_rows=$(wc -l < "$DIR/vis_${TAG}_trajectory.csv" 2>/dev/null)"
echo "commits=$(grep -ac 'planner-traj-commit' "$DIR/sim_${TAG}.log" 2>/dev/null)"
grep -a "visibility-topology-audit" "$DIR/sim_${TAG}.log" | tail -1
df -h /home/bob | tail -1
