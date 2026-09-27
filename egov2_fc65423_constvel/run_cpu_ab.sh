#!/usr/bin/env bash
# Controlled A/B on the SAME map (occlusion_forest_v3.json):
#   encOFF_v3 : encirclement + risk candidates + candidate acceptance + vis ranking + K-of-N ALL OFF
#   encON_v3  : same as v3 run (everything ON)
# With 1 Hz per-process CPU sampling to test the "node starvation" hypothesis.
set -u
DIR=/home/bob/ALP/egov2_fc65423_constvel
SCENE="$DIR/ros_ws/src/multi_uav_formation/scenes/occlusion_forest_v3.json"
cd "$DIR"

cleanup() {
  for p in $(ps -ef | grep -E "[e]go_planner|[r]oslaunch|[r]osmaster|[t]raj_server|[n]ative_egov2|[q]uadrotor|[p]cl_render|[n]odelet|[t]eam_target|[b]ridge|[t]raj2odom|[s]o3_control|[o]dom_visual|[r]viz" | awk '{print $2}'); do
    kill -9 "$p" 2>/dev/null
  done
  sleep 5
}

sample_cpu() {
  local out="$1"
  echo "wall_s,pid,comm,pcpu,rss_kb" > "$out"
  local t0; t0=$(date +%s)
  while true; do
    local now; now=$(date +%s)
    ps -eo pid,comm,pcpu,rss --sort=-pcpu --no-headers 2>/dev/null | head -25 | \
      awk -v t=$((now-t0)) -F' ' '{printf "%s,%s,%s,%s,%s\n", t, $1, $2, $3, $4}' >> "$out"
    sleep 1
    [[ -f /tmp/stop_cpu_sampler ]] && break
  done
}

run_one() {
  local tag="$1" enc="$2" extra="$3"
  echo "=========== [$tag] encirclement=$enc extra=[$extra] $(date +%T) ==========="
  cleanup
  rm -f /tmp/stop_cpu_sampler
  sample_cpu "$DIR/cpu_${tag}.csv" &
  local sampler=$!

  # shellcheck disable=SC2086
  env NATIVE_EGOV2_LOG_FILE="$DIR/sim_${tag}.log" \
      NATIVE_EGOV2_VISIBILITY_CSV="$DIR/vis_${tag}.csv" \
      NATIVE_EGOV2_ENABLE_RVIZ=false NATIVE_EGOV2_WAIT_FOR_RVIZ=false \
      NATIVE_EGOV2_ENCIRCLEMENT="$enc" \
      $extra \
      timeout 420 ./run_round2_full_on.sh scene_file:="$SCENE" rviz:=false \
      > /dev/null 2>&1

  touch /tmp/stop_cpu_sampler; wait $sampler 2>/dev/null
  echo "[$tag] done $(date +%T)"
  echo "  log_bytes=$(stat -c%s "$DIR/sim_${tag}.log" 2>/dev/null)"
  echo "  traj_rows=$(wc -l < "$DIR/vis_${tag}_trajectory.csv" 2>/dev/null)"
  echo "  commits=$(grep -ac 'planner-traj-commit' "$DIR/sim_${tag}.log" 2>/dev/null)"
}

OFF_EXTRA="NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_CANDIDATE_ACCEPTANCE=false NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_K_OF_N_VISIBILITY=false NATIVE_EGOV2_JOINT_TOPOLOGY=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_MOVING_COST=false"

run_one encOFF_v3 false "$OFF_EXTRA"
run_one encON_v3  true  ""
cleanup
echo "=========== A/B finished $(date +%T) ==========="
