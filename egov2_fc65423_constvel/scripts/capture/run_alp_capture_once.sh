#!/usr/bin/env bash
# =============================================================================
# ALP 抓帧工作流 S0:一次改动只跑一次 —— 合围 ON + RViz 抓帧 + 产物落盘
#
#   bash scripts/capture/run_alp_capture_once.sh [--timeout 300]
#
# 做的事:
#   1) 拒绝并发(同类进程已存在 / master 端口被占 → 退出)
#   2) S1: 从生产 rviz 配置派生只读 capture 配置(跟随目标红球,纯可视化)
#   3) 起仿真核心(canonical runner --headless,参数与带 RViz 时完全一致)
#   4) master 就绪后起我们自己独立的 rviz(派生配置)+ annotator + 抓帧
#   5) runner 退出后停抓帧,写 manifest.json 绑定 capture_dir 与 run_dir
#
# 红线:抓帧链路只读运行;派生配置只写 capture 目录;生产配置/场景/阈值不动。
# =============================================================================
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROD_RVIZ_CONFIG="$PROJECT_ROOT/ros_ws/src/multi_uav_formation/config/native_egov2.rviz"

CORE_TIMEOUT=300
# 默认 0.5 s;现象检索(绕行/打转/合围)建议 0.2 s,否则 1 s 级的动作只有 2 帧。
CAPTURE_PERIOD=0.5
MAX_FRAMES=1600
# spheres: 三机覆盖球(像素法按机辨识用)
# none   : 不发覆盖球,保留生产自带正常无人机模型(三机同色 mesh)
OVERLAYS_MODE=spheres
while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) CORE_TIMEOUT="${2:?}"; shift 2 ;;
    --period)  CAPTURE_PERIOD="${2:?}"; shift 2 ;;
    --overlays) OVERLAYS_MODE="${2:?}"; shift 2 ;;   # spheres|none
    --max-frames) MAX_FRAMES="${2:?}"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done

# shellcheck source=../lib/alp_params.sh
source "$SCRIPT_DIR/../lib/alp_params.sh" 2>/dev/null || true
MASTER_URI="${ALP_ROS_MASTER_URI:-http://127.0.0.1:11361}"
MASTER_PORT="${MASTER_URI##*:}"

# ---------------------------------------------------------------- 1) 拒绝并发
if pgrep -f "run_alp_full_on.sh" >/dev/null 2>&1; then
  echo "[capture] REFUSE: run_alp_full_on.sh already running" >&2; exit 1
fi
if timeout 1 bash -c "</dev/tcp/127.0.0.1/${MASTER_PORT}" 2>/dev/null; then
  echo "[capture] REFUSE: master port ${MASTER_PORT} already in use" >&2; exit 1
fi
if pgrep -f "native_egov2_rviz_scene|ego_replan_fsm|traj_server|multi_uav_topology" >/dev/null 2>&1; then
  echo "[capture] REFUSE: ALP planner nodes already running" >&2; exit 1
fi

# ---------------------------------------------------------------- 2) 目录与 S1
STAMP="$(date +%Y%m%d_%H%M%S)_$$"
CAPTURE_DIR="$PROJECT_ROOT/captures/$STAMP"
mkdir -p "$CAPTURE_DIR"
STOP_FILE="$CAPTURE_DIR/.capture_stop"

python3 "$SCRIPT_DIR/rviz_make_capture_config.py" \
  --source "$PROD_RVIZ_CONFIG" \
  --out "$CAPTURE_DIR/rviz_capture.rviz" \
  --report "$CAPTURE_DIR/rviz_config_changes.json" \
  --target-frame alp_capture/target \
  --visible-height 24 \
  --fov-deg 45 \
  --window 1848x1016

# ---------------------------------------------------------------- 3) ROS 环境
# ROS 的 setup.sh 在 set -u 下会因未绑定的 ROS_DISTRO 报错,这里临时放宽
set +u
source /opt/ros/noetic/setup.bash
if [[ -f "$PROJECT_ROOT/ros_ws/devel/setup.bash" ]]; then
  source "$PROJECT_ROOT/ros_ws/devel/setup.bash"
fi
set -u
export ROS_MASTER_URI="$MASTER_URI"
export DISPLAY="${DISPLAY:-:0}"
# rviz 启动会弹 "ROS Noetic goes end-of-life" 公告框,恰好盖住画面中心的
# 目标区(相机跟随目标,合围队形也都在那个矩形里),必须关掉
export DISABLE_ROS1_EOL_WARNINGS=1

# ---------------------------------------------------------------- 4) 起仿真核心
bash "$SCRIPT_DIR/../run_alp_full_on.sh" --headless --timeout "$CORE_TIMEOUT" \
  > "$CAPTURE_DIR/runner_console.log" 2>&1 &
RUNNER_PID=$!
echo "[capture] runner pid=$RUNNER_PID capture_dir=$CAPTURE_DIR"

# master 就绪(最多 150 s)。
# 注意:裸 TCP(/dev/tcp)只说明端口 bind,不代表 rosmaster 的 XML-RPC 已就绪;
# rviz 在这个窗口期启动会弹模态 "Could not contact ROS master" 对话框且不再重试
# (RRCT 工作流踩过的同一个坑)。因此先 TCP,再用 rosnode list 做真实 XML-RPC 往返。
master_up=0
for _ in $(seq 1 150); do
  if timeout 1 bash -c "</dev/tcp/127.0.0.1/${MASTER_PORT}" 2>/dev/null; then
    master_up=1; break
  fi
  sleep 1
done
if [[ $master_up -ne 1 ]]; then
  echo "[capture] FAIL: master not up in 150 s" >&2
  touch "$STOP_FILE"; kill "$RUNNER_PID" 2>/dev/null || true
  exit 3
fi
xmlrpc_ready=0
for _ in $(seq 1 60); do
  if timeout 5 rosnode list >/dev/null 2>&1; then
    xmlrpc_ready=1; break
  fi
  sleep 1
done
if [[ $xmlrpc_ready -ne 1 ]]; then
  echo "[capture] FAIL: master XML-RPC not answering (rosnode list) in 60 s" >&2
  touch "$STOP_FILE"; kill "$RUNNER_PID" 2>/dev/null || true
  exit 3
fi
echo "[capture] master XML-RPC ready"

# ---------------------------------------------------------------- 5) 我们的 rviz + annotator + 抓帧
rviz -d "$CAPTURE_DIR/rviz_capture.rviz" > "$CAPTURE_DIR/rviz_console.log" 2>&1 &
RVIZ_PID=$!
echo "[capture] our rviz pid=$RVIZ_PID (窗口按 PID 锁定,其他 rviz 不受影响)"

python3 "$SCRIPT_DIR/alp_capture_annotator.py" \
  --viz-clock "$CAPTURE_DIR/viz_clock.jsonl" \
  --overlays "$OVERLAYS_MODE" \
  > "$CAPTURE_DIR/annotator.log" 2>&1 &
ANN_PID=$!

python3 -u "$SCRIPT_DIR/rviz_capture_frames.py" \
  --out "$CAPTURE_DIR" \
  --display "$DISPLAY" \
  --require-pid "$RVIZ_PID" \
  --window-name-regex "rviz" \
  --period "$CAPTURE_PERIOD" \
  --quality 85 \
  --max-frames "$MAX_FRAMES" \
  --window-timeout 360 \
  --stop-file "$STOP_FILE" \
  --ros-clock "$CAPTURE_DIR/viz_clock.jsonl" \
  > "$CAPTURE_DIR/capture_console.log" 2>&1 &
CAPTURE_PID=$!

# ---------------------------------------------------------------- 6) 等 runner 收敛
RUNNER_RC=0
wait "$RUNNER_PID" || RUNNER_RC=$?

# ---------------------------------------------------------------- 7) 停抓帧,收产物
touch "$STOP_FILE"
for _ in $(seq 1 10); do
  kill -0 "$CAPTURE_PID" 2>/dev/null || break
  sleep 1
done
kill "$ANN_PID" 2>/dev/null || true
kill "$RVIZ_PID" 2>/dev/null || true

RUN_DIR="$(ls -td "$PROJECT_ROOT/runs/"*/ 2>/dev/null | head -1)"
RUN_DIR="${RUN_DIR%/}"

cat > "$CAPTURE_DIR/manifest.json" <<EOF
{
  "capture_dir": "$CAPTURE_DIR",
  "run_dir": "$RUN_DIR",
  "runner_exit_code": $RUNNER_RC,
  "rviz_pid": $RVIZ_PID,
  "capture_pid": $CAPTURE_PID,
  "annotator_pid": $ANN_PID,
  "core_timeout_sec": $CORE_TIMEOUT,
  "master_uri": "$MASTER_URI",
  "started_at": "$STAMP",
  "notes": "派生 rviz 配置只写本目录;annotator 只发布 /alp_capture/* 与 alp_capture/target TF;抓帧按 PID 锁定我们自己的 rviz 窗口"
}
EOF

echo "capture_dir=$CAPTURE_DIR"
echo "run_dir=$RUN_DIR"
echo "runner_exit_code=$RUNNER_RC"
exit "$RUNNER_RC"
