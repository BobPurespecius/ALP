#!/usr/bin/env bash
# =============================================================================
# ALP 正式仿真唯一入口（canonical runner）
#
#   用法:
#       cd /home/bob/ALP/egov2_fc65423_constvel
#       ./scripts/run_alp_full_on.sh                 # 带 RViz 的完整实验
#       ./scripts/run_alp_full_on.sh --headless      # 只跑仿真核心，不起 RViz
#       ./scripts/run_alp_full_on.sh --timeout 300   # 自定义核心运行时长上限
#
# 设计约束（本轮收敛目标）：
#   1. 只有一个入口。run_show.sh / run_round2_full_on.sh /
#      run_constvel_gradient_rviz.sh 都只是 exec 到本脚本的薄 alias。
#   2. 环境（ROS_HOME / ROS_LOG_DIR / ROS_MASTER_URI）只在本脚本里决定一次，
#      不再有外层设置、内层覆盖。
#   3. 只清理"本次 RUN_ID 自己创建的进程"。绝不按进程名全机 kill。
#   4. RViz 是观察器，其退出/崩溃不影响仿真核心。
#   5. roslaunch 的真实退出码完整传播到本脚本的退出码。
#   6. 证据全部保留在 runs/<RUN_ID>/ 下。
# =============================================================================
set -Eeuo pipefail

# ------------------------------------------------------------------ 0. 基本路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCENARIO="$PROJECT_ROOT/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json"
LAUNCH_FILE="$PROJECT_ROOT/ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch"
BASE_ROS_WS="/home/bob/ALP/guidance/ros_ws"
ROS_WS="$PROJECT_ROOT/ros_ws"
TRACKING_WS="$ROS_WS/src/EGO-Planner-v2/swarm-playground/tracking_ws"

# 唯一的参数来源：scripts/lib/alp_params.sh
# shellcheck source=lib/alp_params.sh
source "$SCRIPT_DIR/lib/alp_params.sh"

# ------------------------------------------------------------------ 1. CLI 解析
RVIZ_ENABLED="$ALP_DEFAULT_RVIZ"
CORE_TIMEOUT_SEC="$ALP_DEFAULT_CORE_TIMEOUT"
BOOT_TIMEOUT_SEC="$ALP_DEFAULT_BOOT_TIMEOUT"
EXTRA_LAUNCH_ARGS=()
ABLATION_MODE_RAW=""
ABLATION_EXPLICIT=false
LEGACY_PT_REQUESTED=false

usage() {
  cat <<'EOF'
ALP canonical simulation runner

  --headless            不启动 RViz（仿真核心参数与带 RViz 时完全一致）
  --rviz                启动 RViz（默认）
  --timeout <sec>       仿真核心最长运行时长（默认 900）
  --boot-timeout <sec>  等待 BOOT-12 的最长时间（默认 180）
  --scenario <json>     使用仓库内已有正式场景（默认 long_cylinder_forest.json）
  --k3-repair on|off    K3 relay repair 开关（默认 off）
  --k3-escalation on|off K3 Local escalation 开关（Local early authority + C3/D3 comparator，默认 off）
  --ablation <mode>     正式单变量消融模式（八选一）
  --pt                  旧兼容入口；未指定 --ablation 时启用 Team P/T
  --ros-arg k:=v        透传额外 roslaunch 参数（仅限诊断用途）
  -h | --help           显示本帮助

唯一命令：
  ./scripts/run_alp_full_on.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)      RVIZ_ENABLED=false; shift ;;
    --rviz)          RVIZ_ENABLED=true; shift ;;
    --timeout)       CORE_TIMEOUT_SEC="${2:?--timeout 需要参数}"; shift 2 ;;
    --boot-timeout)  BOOT_TIMEOUT_SEC="${2:?--boot-timeout 需要参数}"; shift 2 ;;
    --scenario)      SCENARIO="${2:?--scenario 需要参数}"; shift 2 ;;
    --k3-repair)
      case "${2:?--k3-repair 需要 on 或 off}" in
        on) export ALP_TEAM_K3_REPAIR_ENABLED=true ;;
        off) export ALP_TEAM_K3_REPAIR_ENABLED=false ;;
        *) echo "--k3-repair 只能是 on 或 off" >&2; exit 2 ;;
      esac
      shift 2 ;;
    --k3-escalation)
      case "${2:?--k3-escalation 需要 on 或 off}" in
        on) export ALP_K3_LOCAL_ESCALATION_ENABLED=true ;;
        off) export ALP_K3_LOCAL_ESCALATION_ENABLED=false ;;
        *) echo "--k3-escalation 只能是 on 或 off" >&2; exit 2 ;;
      esac
      shift 2 ;;
    --ablation)      ABLATION_MODE_RAW="${2:?--ablation 需要参数}"; ABLATION_EXPLICIT=true; shift 2 ;;
    --pt)            LEGACY_PT_REQUESTED=true; shift ;;
    --ros-arg)       EXTRA_LAUNCH_ARGS+=("${2:?--ros-arg 需要参数}"); shift 2 ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f "$SCENARIO" ]]; then
  echo "场景文件不存在: $SCENARIO" >&2
  exit 2
fi

# ------------------------------------------------------------------ 2. RUN_ID 与运行目录
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
RUN_DIR="$PROJECT_ROOT/runs/$RUN_ID"
mkdir -p "$RUN_DIR"
MANIFEST="$RUN_DIR/run_manifest.txt"
RESOLVED="$RUN_DIR/resolved_params.txt"
PIDS_FILE="$RUN_DIR/pids.txt"
STDOUT_LOG="$RUN_DIR/roslaunch_stdout.log"
EXIT_STATUS="$RUN_DIR/exit_status.txt"
PROC_STATUS="$RUN_DIR/process_status.txt"
ABLATION_MANIFEST="$RUN_DIR/ablation_manifest.txt"
ABLATION_COUNTERS="$RUN_DIR/ablation_counters.txt"

# 从这一刻开始，本脚本产生的所有屏幕输出同时落盘到 run 目录。
exec > >(tee -a "$RUN_DIR/runner_console.log") 2>&1

log()  { printf '%s %s\n' "$(date '+%H:%M:%S')" "$*"; }
boot() { printf '[BOOT-%02d] %s\n' "$1" "$2"; }

# ------------------------------------------------------------------ 3. 环境（唯一来源）
# ROS 的 setup.sh 在 set -u 下会因未绑定的 ROS_DISTRO 报错,临时放宽
# (20260925_021 前实证;恢复后需保持)。
set +u
source /opt/ros/noetic/setup.bash
source "$BASE_ROS_WS/devel/setup.bash"
source "$ROS_WS/devel/setup.bash"
set -u

export ROS_MASTER_URI="$ALP_ROS_MASTER_URI"
export ROS_PACKAGE_PATH="$ROS_WS/src:$TRACKING_WS/src:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$ROS_WS/devel:$BASE_ROS_WS/devel:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$ROS_WS/devel/lib:$BASE_ROS_WS/devel/lib:${LD_LIBRARY_PATH:-}"
export EGO_PLANNER_VARIANT=egov2
export DISPLAY="${DISPLAY:-:0}"

# ROS_HOME：每个 RUN_ID 一个独立可写目录（~/.ros 在本环境是只读的）。
export ROS_HOME="$RUN_DIR/ros_home"
mkdir -p "$ROS_HOME"

# ROS_LOG_DIR 必须写入持久磁盘上的本次 run 目录。/dev/shm 是 RAM-backed
# tmpfs；ROS 的 rosout 轮转日志可达 GB 级，多轮残留会挤满共享内存并触发 OOM。
# 只在这里决定一次，不存在任何内层覆盖。
ALP_ROS_LOG_DIR="$RUN_DIR/ros_log"
ros_log_dir_choice="persistent_run_dir(tmpfs_disabled)"
mkdir -p "$ALP_ROS_LOG_DIR"
export ROS_LOG_DIR="$ALP_ROS_LOG_DIR"

# 运行期产物全部写进本次 run 目录。
export NATIVE_EGOV2_EXPERIMENT_DIR="$PROJECT_ROOT"
export NATIVE_EGOV2_LOG_FILE="$RUN_DIR/planner_rosout.log"
export NATIVE_EGOV2_VISIBILITY_CSV="$RUN_DIR/visibility.csv"
export NATIVE_EGOV2_TRAJECTORY_CSV="$RUN_DIR/visibility_trajectory.csv"
export NATIVE_EGOV2_SCENE_FILE="$SCENARIO"
export NATIVE_EGOV2_ROS_MASTER_URI="$ALP_ROS_MASTER_URI"

# ------------------------------------------------------------------ 4. 参数（唯一解析点）
ABLATION_CONFIG_CLI="$ROS_WS/devel/lib/traj_utils/ablation_config_cli"
export ALP_FORMAL_ABLATION=false
if [[ "$ABLATION_EXPLICIT" == "true" ]]; then
  if [[ ! -x "$ABLATION_CONFIG_CLI" ]]; then
    echo "ABORT: ablation_config_cli 尚未构建: $ABLATION_CONFIG_CLI" >&2
    exit 2
  fi
  if [[ "$ABLATION_MODE_RAW" == *","* ]]; then
    echo "ABORT: FORMAL_ABLATION_MODE_MUST_BE_SINGLE_CHOICE" >&2
    exit 2
  fi
  # 解析与 derived switches 只来自共享 C++ AblationConfig；runner 不维护
  # 第二张模式矩阵。
  eval "$("$ABLATION_CONFIG_CLI" --shell "$ABLATION_MODE_RAW")"
  if [[ "$LEGACY_PT_REQUESTED" == "true" && "$ABLATION_TEAM_PT" != "true" ]]; then
    echo "CONFLICTING_ARGUMENTS:" >&2
    echo "NO_TEAM_PT cannot be combined with --pt" >&2
    exit 2
  fi
  for extra in "${EXTRA_LAUNCH_ARGS[@]}"; do
    key="${extra%%:=*}"
    case "$key" in
      ablation_mode|enable_joint_pt_optimization|enable_team_visibility_optimizer|enable_joint_topology_coordination|enable_encirclement_tracking|enable_cooperative_viewpoint_reference)
        echo "ABORT: FORMAL_ABLATION_AUTHORITY_CANNOT_BE_OVERRIDDEN_BY_--ros-arg: $extra" >&2
        exit 2
        ;;
    esac
  done
  export ALP_FORMAL_ABLATION=true
  export ALP_ABLATION_MODE="$ABLATION_CLI_MODE"
  export NATIVE_EGOV2_JOINT_PT="$ABLATION_TEAM_PT"
  "$ABLATION_CONFIG_CLI" --manifest "$ABLATION_CLI_MODE" | tee "$ABLATION_MANIFEST"
else
  # Exact legacy compatibility: without --ablation, --pt keeps its historical
  # meaning and no formal mode is asserted.
  export NATIVE_EGOV2_JOINT_PT="$LEGACY_PT_REQUESTED"
  echo "ABLATION_MODE=UNSPECIFIED_LEGACY_COMPATIBILITY"
fi
apply_alp_env

# ------------------------------------------------------------------ 4.5 resolved truth 打印
# 阶段 A（配置 authority）：runner 把最终解析结果原样打印到控制台。
# 与 runs/<RUN_ID>/resolved_params.txt、roslaunch_argv.txt、节点侧
# [ablation-config] 自证共同构成可对账的四重证据。
echo "==================== RESOLVED CONFIG TRUTH ===================="
if [[ "$ALP_FORMAL_ABLATION" == "true" ]]; then
  echo "ABLATION_MODE=$ALP_ABLATION_MODE"
  echo "LOCAL_VISIBILITY=$ABLATION_LOCAL_VIS"
  echo "TEAM_PT=$ABLATION_TEAM_PT"
  echo "LOS_TOPOLOGY=$ABLATION_LOS_TOPOLOGY"
  echo "DYNAMIC_BODY_TOPOLOGY=$ABLATION_DYNAMIC_BODY_TOPOLOGY"
  echo "TARGET_FACING_ASTAR=$ABLATION_TARGET_FACING_ASTAR"
  echo "SAFE_SEED_RETENTION=$ABLATION_SAFE_SEED_RETENTION"
  echo "LOCAL_GAP=$ABLATION_LOCAL_GAP"
  echo "ENCIRCLEMENT=$ABLATION_ENCIRCLEMENT"
  echo "COOPERATIVE_REFERENCE=$ABLATION_COOPERATIVE_REFERENCE"
  echo "PHYSICAL_SAFETY=$ABLATION_PHYSICAL_SAFETY"
  echo "JOINT_YAW=$ABLATION_JOINT_YAW"
else
  echo "ABLATION_MODE=UNSPECIFIED_LEGACY_COMPATIBILITY"
  echo "TEAM_PT=$NATIVE_EGOV2_JOINT_PT"
fi
echo "---- launch args（唯一参数来源 alp_params.sh）----"
for entry in "${ALP_RESOLVED_PARAMS[@]}"; do
  echo "  $entry"
done
echo "==============================================================="

# ------------------------------------------------------------------ 5. 参数与清单落盘
{
  echo "==================== RUN MANIFEST ===================="
  echo "RUN_ID                 = $RUN_ID"
  echo "PROJECT_ROOT           = $PROJECT_ROOT"
  echo "SCENARIO               = $SCENARIO"
  echo "LAUNCH_FILE            = $LAUNCH_FILE"
  echo "ROS_HOME               = $ROS_HOME"
  echo "ROS_LOG_DIR            = $ALP_ROS_LOG_DIR   ($ros_log_dir_choice)"
  echo "ROS_MASTER_URI         = $ROS_MASTER_URI"
  echo "RVIZ_ENABLED           = $RVIZ_ENABLED"
  echo "CORE_TIMEOUT_SEC       = $CORE_TIMEOUT_SEC"
  echo "BOOT_TIMEOUT_SEC       = $BOOT_TIMEOUT_SEC"
  echo "RUN_DIR                = $RUN_DIR"
  echo "FORMAL_ABLATION        = $ALP_FORMAL_ABLATION"
  echo "ABLATION_MODE          = ${ALP_ABLATION_MODE:-UNSPECIFIED_LEGACY}"
  echo "HOSTNAME               = $(hostname)"
  echo "DATE                   = $(date --iso-8601=ns)"
  echo "======================================================"
} | tee "$MANIFEST"

{
  echo "==================== RESOLVED PLANNER PARAMS ===================="
  printf '%-52s = %s\n' "SCENARIO"                 "$SCENARIO"
  printf '%-52s = %s\n' "scene nMovingObstacle"    "$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['nMovingObstacle'])" "$SCENARIO")"
  printf '%-52s = %s\n' "scene target speed"       "$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['targetTracking']['speed'])" "$SCENARIO")"
  printf '%-52s = %s\n' "scene static obstacles"   "$(python3 -c "import json,sys;print(len(json.load(open(sys.argv[1]))['obstacleData']))" "$SCENARIO")"
  for kv in "${ALP_RESOLVED_PARAMS[@]}"; do
    printf '%-52s = %s\n' "${kv%%=*}" "${kv#*=}"
  done
  echo "================================================================"
} | tee "$RESOLVED"

log "RUN_ID=$RUN_ID"
log "RUN_DIR=$RUN_DIR"
log "ROS_HOME=$ROS_HOME"
log "ROS_LOG_DIR=$ROS_LOG_DIR ($ros_log_dir_choice)"
log "ROS_MASTER_URI=$ROS_MASTER_URI"
log "RVIZ_ENABLED=$RVIZ_ENABLED"
log "FORMAL_ABLATION=$ALP_FORMAL_ABLATION mode=${ALP_ABLATION_MODE:-UNSPECIFIED_LEGACY}"

# ------------------------------------------------------------------ 6. 端口 ownership 检查
# 只允许清理"本项目上一轮遗留、且有 PID ownership 记录"的进程。
# 未知占用者一律只报告、不 kill。
port_owner_pids() {
  { ss -lunp 2>/dev/null; ss -ltnp 2>/dev/null; } \
    | awk -v port=":$1" '$0 ~ port {print}' \
    | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u || true
}

pid_is_ours() {
  local pid="$1" cmd
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  [[ -z "$cmd" ]] && return 1
  case "$cmd" in
    *"$PROJECT_ROOT"*) return 0 ;;
  esac
  # 也接受历史上由本项目 run 目录记录过的 PID。
  local f
  for f in "$PROJECT_ROOT"/runs/*/pids.txt; do
    [[ -f "$f" ]] || continue
    grep -qx "$pid" "$f" 2>/dev/null && return 0
  done
  return 1
}

check_and_reclaim_port() {
  local port="$1" label="$2" pids pid cmd
  pids="$(port_owner_pids "$port")"
  [[ -z "$pids" ]] && { log "[PORT-OK] $label :$port 空闲"; return 0; }
  local reclaimable=() foreign=()
  for pid in $pids; do
    if pid_is_ours "$pid"; then reclaimable+=("$pid"); else foreign+=("$pid"); fi
  done
  if [[ ${#foreign[@]} -gt 0 ]]; then
    echo "PORT_CONFLICT"
    echo "PORT=$port"
    for pid in "${foreign[@]}"; do
      echo "OWNER_PID=$pid"
      echo "OWNER_CMD=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || echo '<gone>')"
    done
    echo "ACTION=REFUSE_TO_KILL_UNKNOWN_PROCESS"
    return 1
  fi
  log "[PORT-RECLAIM] $label :$port 被本项目上一轮遗留进程占用，清理: ${reclaimable[*]}"
  local p
  for p in "${reclaimable[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${reclaimable[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  sleep 1
  pids="$(port_owner_pids "$port")"
  [[ -z "$pids" ]] && { log "[PORT-OK] $label :$port 已释放"; return 0; }
  echo "PORT_CONFLICT"
  echo "PORT=$port"
  echo "ACTION=PORT_STILL_BUSY_AFTER_RECLAIM"
  return 1
}

if ! check_and_reclaim_port 8081 "UDP 轨迹桥"; then
  echo "FIRST_BOOT_FAILURE_STAGE=BOOT-01"
  echo "FIRST_BOOT_FAILURE_REASON=PORT_CONFLICT_UDP_8081"
  exit 3
fi
master_port="${ROS_MASTER_URI##*:}"
if ! check_and_reclaim_port "$master_port" "ROS master"; then
  echo "FIRST_BOOT_FAILURE_STAGE=BOOT-02"
  echo "FIRST_BOOT_FAILURE_REASON=PORT_CONFLICT_ROS_MASTER"
  exit 3
fi

boot 1 "environment ready"

# ------------------------------------------------------------------ 7. 启动 roslaunch（独立 process group）
CORE_PGID=""
CORE_EXIT_CODE=""
CLEANUP_STATUS="NOT_RUN"
RVIZ_EXIT_CODE="NOT_STARTED"
CORE_FINISHED=0

# 关键：ALP_LAUNCH_ARGS 已经是 bash 数组，必须原样展开传递。
# 之前这里用 `read -r -a` 重新切分，而 read 只消费第一行，导致 25 个参数里
# 只有第 1 个真正到达 roslaunch——所有 ON 开关都退回 launch 默认值（等价 OFF）。
launch_args=("${ALP_LAUNCH_ARGS[@]}")
launch_args+=("${EXTRA_LAUNCH_ARGS[@]}")

# 启动参数自证：把真正传给 roslaunch 的 argv 原样打印并落盘，
# 以后任何"看起来是 ON 实际是 OFF"都能一眼看出来。
{
  echo "==================== ROSLAUNCH ARGV ($((${#launch_args[@]} + 1)) 项) ===================="
  printf 'roslaunch %s\n' "$LAUNCH_FILE"
  for a in "${launch_args[@]}"; do printf '  %s\n' "$a"; done
  printf '  rviz:=%s\n' "$RVIZ_ENABLED"
  echo "================================================================"
} | tee "$RUN_DIR/roslaunch_argv.txt"

setsid bash -c '
  set -o pipefail
  exec roslaunch "$0" "$@"
' "$LAUNCH_FILE" "${launch_args[@]}" rviz:="$RVIZ_ENABLED" \
  > >(tee -a "$STDOUT_LOG") 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PIDS_FILE"
sleep 1
CORE_PGID="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ' || true)"
log "[LAUNCH] roslaunch pid=$LAUNCH_PID pgid=${CORE_PGID:-unknown}"

if [[ -z "$CORE_PGID" ]]; then
  echo "FIRST_BOOT_FAILURE_STAGE=BOOT-02"
  echo "FIRST_BOOT_FAILURE_REASON=ROSLAUNCH_FAILED_TO_START"
  exit 4
fi

# 收集"本次 RUN_ID 拥有"的全部进程：
#   1) roslaunch 自己所在的 process group（setsid 保证它独立于用户 shell）
#   2) 以 roslaunch 为根的整棵后代进程树（roslaunch 启动的节点常常另起 session，
#      不能只靠 process group 判断归属）
# 只写进本 RUN_ID 的 pids.txt，绝不触碰其它进程。
ancestors_of() {  # 输出 pid 的所有祖先，用于确认归属
  local pid="$1" ppid
  while [[ -n "$pid" && "$pid" != "0" && "$pid" != "1" ]]; do
    ppid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    [[ -z "$ppid" ]] && break
    echo "$ppid"
    pid="$ppid"
  done
}

descendants_of() {
  local root="$1" frontier="$1" next pid child
  local -a all=()
  while [[ -n "$frontier" ]]; do
    next=""
    for pid in $frontier; do
      for child in $(ps -eo pid=,ppid= 2>/dev/null | awk -v p="$pid" '$2==p {print $1}'); do
        all+=("$child")
        next="$next $child"
      done
    done
    frontier="$next"
  done
  [[ ${#all[@]} -gt 0 ]] && printf '%s\n' "${all[@]}"
  return 0
}

record_own_pids() {
  {
    echo "$LAUNCH_PID"
    [[ -n "${CORE_PGID:-}" ]] && ps -eo pid=,pgid= 2>/dev/null | awk -v g="$CORE_PGID" '$2==g {print $1}'
    descendants_of "$LAUNCH_PID"
  } | grep -E '^[0-9]+$' | sort -u -n > "$PIDS_FILE" || true
}

# ------------------------------------------------------------------ 8. 退出/清理语义
cleanup_run() {
  local reason="$1"
  log "[CLEANUP] reason=$reason；只清理本次 RUN_ID (roslaunch pid=$LAUNCH_PID) 拥有的进程"
  record_own_pids
  local owned_count
  owned_count=$(wc -l < "$PIDS_FILE" 2>/dev/null || echo 0)
  local pid
  # 第一轮：SIGTERM 本 RUN_ID 拥有的全部进程（先后代、后根，避免 roslaunch 先死
  # 导致孤儿进程失去归属线索）。
  local ordered
  ordered=$(tac "$PIDS_FILE" 2>/dev/null || cat "$PIDS_FILE")
  for pid in $ordered; do
    [[ "$pid" == "$$" ]] && continue
    kill -TERM "$pid" 2>/dev/null || true
  done
  if [[ -n "$CORE_PGID" ]]; then kill -TERM -- "-$CORE_PGID" 2>/dev/null || true; fi
  sleep 3
  # 第二轮：仍未退出的本次进程才 SIGKILL。
  local survivors=0
  for pid in $ordered; do
    [[ "$pid" == "$$" ]] && continue
    if [[ -d "/proc/$pid" ]]; then
      kill -KILL "$pid" 2>/dev/null && survivors=$((survivors+1)) || true
    fi
  done
  sleep 1
  local left=0
  for pid in $(cat "$PIDS_FILE" 2>/dev/null); do
    [[ -d "/proc/$pid" ]] && left=$((left+1))
  done
  {
    echo "CLEANUP_REASON=$reason"
    echo "CLEANUP_TARGET_PGID=$CORE_PGID"
    echo "CLEANUP_OWNED_PROCESS_COUNT=$owned_count"
    echo "CLEANUP_FORCE_KILLED=$survivors"
    echo "CLEANUP_REMAINING_OWNED=$left"
    echo "CLEANUP_KILLED_FOREIGN_PROCESSES=0"
  } >> "$PROC_STATUS"
  if [[ "$left" -eq 0 ]]; then CLEANUP_STATUS="OK"; else CLEANUP_STATUS="INCOMPLETE"; fi
  log "[CLEANUP] status=$CLEANUP_STATUS owned=$owned_count force_killed=$survivors remaining=$left"
}

FINALIZED=0
write_ablation_counter_summary() {
  [[ "$ALP_FORMAL_ABLATION" == "true" ]] || return 0
  local names=(
    LOCAL_VIS_COST_EVAL_COUNT
    TEAM_PT_ATTEMPT_COUNT
    LOS_TOPOLOGY_DISPATCH_COUNT
    DYNAMIC_BODY_TOPOLOGY_DISPATCH_COUNT
    TARGET_FACING_ASTAR_ATTEMPT_COUNT
    SAFE_SEED_RETENTION_USED_COUNT
    LOCAL_GAP_COST_EVAL_COUNT
  )
  {
    echo "ABLATION_MODE=${ALP_ABLATION_MODE}"
    local name value
    for name in "${names[@]}"; do
      value="$(grep -oE "${name}=[0-9]+" "$STDOUT_LOG" 2>/dev/null \
        | cut -d= -f2 | sort -n | tail -1 || true)"
      echo "${name}=${value:-0}"
    done
    echo "RUNTIME_COMPONENT_MODES=$(grep -oE '\[ablation-config\] component=[^ ]+ mode=[^ ]+' "$STDOUT_LOG" 2>/dev/null | sort -u | tr '\n' ';' || true)"
  } > "$ABLATION_COUNTERS"
  cat "$ABLATION_COUNTERS"
}

finalize() {
  local final="$1"
  [[ $FINALIZED -eq 1 ]] && return 0
  FINALIZED=1
  {
    echo "RUN_ID=$RUN_ID"
    echo "CORE_EXIT_CODE=${CORE_EXIT_CODE:-NOT_AVAILABLE}"
    echo "RVIZ_EXIT_CODE=${RVIZ_EXIT_CODE}"
    echo "CLEANUP_STATUS=${CLEANUP_STATUS}"
    echo "FINAL_EXIT_CODE=${final}"
    echo "LAST_BOOT_STAGE=${LAST_BOOT_STAGE:-BOOT-00}"
    echo "SIMULATION_REACHED_BOOT_12=${SIMULATION_REACHED_BOOT_12:-NO}"
  } > "$EXIT_STATUS"
  write_ablation_counter_summary
  cat "$EXIT_STATUS"
  log "[EVIDENCE] 本次运行全部证据保留在: $RUN_DIR"
  trap - EXIT INT TERM
  exit "$final"
}

on_signal() {
  log "[SIGNAL] 收到终止信号，开始收敛"
  core_stop_requested=1
}
core_stop_requested=0
trap on_signal INT TERM

LAST_BOOT_STAGE="BOOT-01"
SIMULATION_REACHED_BOOT_12="NO"
FIRST_BOOT_FAILURE_STAGE=""
FIRST_BOOT_FAILURE_REASON=""

trap 'rc=$?; if [[ $CORE_FINISHED -eq 0 ]]; then cleanup_run "EXIT_TRAP"; fi; finalize "$rc"' EXIT

# ------------------------------------------------------------------ 9. BOOT 阶段诊断
ros_ok() { timeout 5 rosnode list >/dev/null 2>&1; }
node_alive() { rosnode list 2>/dev/null | grep -qx "$1"; }
topic_has_publisher() { rostopic list 2>/dev/null | grep -qx "$1"; }
topic_hz_ok() { timeout 6 rostopic hz -w 5 "$1" 2>&1 | grep -qE 'average rate'; }

boot_fail() {
  FIRST_BOOT_FAILURE_STAGE="$1"
  FIRST_BOOT_FAILURE_REASON="$2"
  LAST_BOOT_STAGE="$1"
  echo "BOOT_FAILURE_STAGE=$FIRST_BOOT_FAILURE_STAGE"
  echo "FIRST_FAILED_CONDITION=$FIRST_BOOT_FAILURE_REASON"
  log "[BOOT-FAIL] stage=$1 reason=$2"
  # 把当下 ROS graph 快照留证
  { echo "--- rosnode list ---"; rosnode list 2>&1 || true;
    echo "--- rostopic list ---"; rostopic list 2>&1 || true; } > "$RUN_DIR/ros_graph_at_failure.txt" 2>&1 || true
}

wait_for() {
  # wait_for <stage_num> <描述> <超时秒> <判定命令...>
  # Feedback126 §12: the timeout is TRUE WALL-CLOCK seconds.  The previous
  # implementation counted loop iterations while a single BOOT-12 probe
  # itself sleeps 12-45 s (displacement_over_3s + rostopic timeouts), so
  # "--boot-timeout 180" meant hours and a stuck boot produced GB-scale
  # logs.  The deadline is now measured with date +%s before each probe.
  local stage="$1" desc="$2" timeout_s="$3"; shift 3
  local deadline now
  deadline=$(( $(date +%s) + timeout_s ))
  while :; do
    if [[ $core_stop_requested -eq 1 ]]; then return 2; fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then return 3; fi
    now=$(date +%s)
    if (( now >= deadline )); then
      log "[BOOT-TIMEOUT] stage=BOOT-$(printf '%02d' "$stage") wallclock_timeout_s=$timeout_s"
      return 1
    fi
    if "$@" >/dev/null 2>&1; then
      boot "$stage" "$desc"
      LAST_BOOT_STAGE="BOOT-$(printf '%02d' "$stage")"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic_message() {
  local topic="$1"
  timeout 5 rostopic echo -n 1 "$topic" >/dev/null 2>&1
}

# BOOT-02 ROS master
if ! wait_for 2 "ROS master ready" 45 ros_ok; then
  boot_fail "BOOT-02" "ROS_MASTER_NOT_REACHABLE_OR_ROSLAUNCH_DIED"
  CORE_FINISHED=1; cleanup_run "BOOT02_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 5
fi

# BOOT-03 必需节点已生成
required_nodes=(
  "/native_target_state_coordinator"
  "/cooperative_viewpoint_manager"
  "/drone_0_ego_planner_node"
  "/drone_1_ego_planner_node"
  "/drone_2_ego_planner_node"
  "/drone_0_traj_server"
  "/drone_1_traj_server"
  "/drone_2_traj_server"
)
all_required_nodes() {
  local n
  for n in "${required_nodes[@]}"; do node_alive "$n" || return 1; done
  return 0
}
if ! wait_for 3 "required nodes spawned" "$BOOT_TIMEOUT_SEC" all_required_nodes; then
  missing=""
  for n in "${required_nodes[@]}"; do node_alive "$n" || missing="$missing $n"; done
  boot_fail "BOOT-03" "MISSING_NODES:$missing"
  CORE_FINISHED=1; cleanup_run "BOOT03_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 6
fi

# BOOT-04/05/06 分别确认 planner / traj_server / target coordinator 存活
wait_for 4 "planner nodes alive" 20 \
  bash -c 'rosnode list 2>/dev/null | grep -q "/drone_0_ego_planner_node"' \
  || { boot_fail "BOOT-04" "PLANNER_NODE_NOT_ALIVE"; CORE_FINISHED=1; cleanup_run "BOOT04_FAILED"; wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?; finalize 6; }
wait_for 5 "traj_server alive" 20 \
  bash -c 'rosnode list 2>/dev/null | grep -q "/drone_0_traj_server"' \
  || { boot_fail "BOOT-05" "TRAJ_SERVER_NOT_ALIVE"; CORE_FINISHED=1; cleanup_run "BOOT05_FAILED"; wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?; finalize 6; }
wait_for 6 "target coordinator alive" 20 \
  bash -c 'rosnode list 2>/dev/null | grep -q "/native_target_state_coordinator"' \
  || { boot_fail "BOOT-06" "TARGET_COORDINATOR_NOT_ALIVE"; CORE_FINISHED=1; cleanup_run "BOOT06_FAILED"; wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?; finalize 6; }

# BOOT-06B 配置对账（阶段 A）：runner expected vs 节点 actual。
# 节点启动时各自打印 [ablation-config] component=... mode=... team_pt=...；
# 这里从 stdout 日志解析实际值并与 runner 解析的权威值比对，不一致直接 FAIL，
# 不允许"打印 ON 实际 OFF"悄悄继续。
reconcile_ablation_config() {
  # 节点侧 AblationMode 统一打印规范化大写名；CLI 接受小写别名。
  # 对账前必须使用同一规范，否则 `--ablation full` 会把真实 FULL
  # 错判为启动失败并在 BOOT-06B 等待 30 秒。
  local expected_mode="${ALP_ABLATION_MODE^^}" expected_team_pt="$NATIVE_EGOV2_JOINT_PT"
  local comp mode team_pt line
  for comp in planner_manager topology_coordinator poly_traj_optimizer; do
    line="$(grep -o "\[ablation-config\] component=${comp} .*" "$STDOUT_LOG" 2>/dev/null | tail -1)"
    [[ -n "$line" ]] || return 1
    mode="$(printf '%s' "$line" | grep -o 'mode=[A-Z_0-9]*' | head -1 | cut -d= -f2)"
    team_pt="$(printf '%s' "$line" | grep -o 'team_pt=[01]' | head -1 | cut -d= -f2)"
    [[ -n "$mode" && -n "$team_pt" ]] || return 1
    if [[ "$ALP_FORMAL_ABLATION" == "true" ]]; then
      [[ "$mode" == "$expected_mode" ]] || { echo "RECONCILE_MISMATCH component=$comp mode_actual=$mode mode_expected=$expected_mode"; return 1; }
    fi
    local expected_pt_bit="0"; [[ "$expected_team_pt" == "true" ]] && expected_pt_bit="1"
    [[ "$team_pt" == "$expected_pt_bit" ]] || { echo "RECONCILE_MISMATCH component=$comp team_pt_actual=$team_pt team_pt_expected=$expected_pt_bit"; return 1; }
  done
  echo "RECONCILE_OK mode=${expected_mode:-UNSPECIFIED_LEGACY_COMPATIBILITY} team_pt=$expected_team_pt"
  return 0
}
reconciled=false
for _ in $(seq 1 30); do
  if reconcile_ablation_config > "$RUN_DIR/ablation_reconciliation.txt" 2>&1; then
    reconciled=true; break
  fi
  sleep 1
done
if [[ "$reconciled" != "true" ]]; then
  cat "$RUN_DIR/ablation_reconciliation.txt" >&2
  boot_fail "BOOT-06B" "ABLATION_CONFIG_RECONCILIATION_MISMATCH"
  CORE_FINISHED=1; cleanup_run "BOOT06B_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 6
fi
boot 6 "ablation config reconciled: $(cat "$RUN_DIR/ablation_reconciliation.txt")"

# BOOT-07 首帧 odom
odom_all_live() {
  local d
  for d in 0 1 2; do
    wait_for_topic_message "/drone_${d}_visual_slam/odom" || return 1
  done
  return 0
}
if ! wait_for 7 "first odometry received" "$BOOT_TIMEOUT_SEC" odom_all_live; then
  boot_fail "BOOT-07" "NO_ODOMETRY_ON_/drone_N_visual_slam/odom"
  CORE_FINISHED=1; cleanup_run "BOOT07_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 7
fi

# BOOT-08 首条本地轨迹发布
first_traj_published() {
  local d
  for d in 0 1 2; do
    topic_has_publisher "/drone_${d}_planning/trajectory" || return 1
  done
  wait_for_topic_message "/drone_0_planning/trajectory"
}
if ! wait_for 8 "first local trajectory published" "$BOOT_TIMEOUT_SEC" first_traj_published; then
  boot_fail "BOOT-08" "NO_TRAJECTORY_ON_/drone_N_planning/trajectory"
  CORE_FINISHED=1; cleanup_run "BOOT08_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 8
fi

# BOOT-09 首条轨迹被激活（pos_cmd 开始输出）
first_traj_activated() {
  local d
  for d in 0 1 2; do
    wait_for_topic_message "/drone_${d}_planning/pos_cmd" || return 1
  done
  return 0
}
if ! wait_for 9 "first trajectory activated" "$BOOT_TIMEOUT_SEC" first_traj_activated; then
  boot_fail "BOOT-09" "TRAJECTORY_NOT_ACTIVATED_NO_POS_CMD"
  CORE_FINISHED=1; cleanup_run "BOOT09_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 9
fi

# BOOT-10 tracking_ready 状态
tracking_ready_all() {
  local d
  for d in 0 1 2; do
    topic_has_publisher "/target_tracking/tracking_ready/uav${d}" || return 1
  done
  return 0
}
if ! wait_for 10 "tracking_ready status" "$BOOT_TIMEOUT_SEC" tracking_ready_all; then
  boot_fail "BOOT-10" "TRACKING_READY_TOPIC_MISSING"
  CORE_FINISHED=1; cleanup_run "BOOT10_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 10
fi

# BOOT-11 target_start 已发布
if ! wait_for 11 "target_start observed" "$BOOT_TIMEOUT_SEC" wait_for_topic_message /target_tracking/target_start_time; then
  boot_fail "BOOT-11" "TARGET_START_TIME_NOT_PUBLISHED"
  CORE_FINISHED=1; cleanup_run "BOOT11_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 11
fi

# BOOT-12 仿真真的在跑：目标在动 + 三机 odom 持续更新 + 轨迹 id 推进
# 分项累积判定：一旦观测到该项在动就置 YES 并保持（成功路径也会落盘，
# 否则 set -u 会在通过后因未绑定变量把 runner 杀掉——20260919_023458 实证）。
# 判据（阶段 A 修订）：3 s 窗口内 **3D 累积位移** > 0.05 m，或该载具执行链
# 仍然在线（pos_cmd / ground_truth 持续有新消息）。旧判据只看 x 方向瞬时
# 位移，载机短暂减速或目标到达后停止都会造成假失败（2/5 次运行受害）。
BOOT12_TARGET=NO
BOOT12_UAV0=NO
BOOT12_UAV1=NO
BOOT12_UAV2=NO
odom_xyz() {
  timeout 6 rostopic echo -n 1 "$1" 2>/dev/null \
    | awk '/position:/{f=1;next} f&&/x:/{print $2;next} f&&/y:/{print $2;next} f&&/z:/{print $2;exit}'
}
displacement_over_3s() {  # $1=topic → 3 s 位移是否 > 0.05 m
  local a b
  a="$(odom_xyz "$1")" ; sleep 3 ; b="$(odom_xyz "$1")"
  [[ -n "$a" && -n "$b" ]] || return 1
  awk -v a="$a" -v b="$b" 'BEGIN{
    split(a,A," "); split(b,B," ");
    d=sqrt((B[1]-A[1])^2+(B[2]-A[2])^2+(B[3]-A[3])^2);
    exit !(d>0.05)}'
}
topic_alive() {  # $1=topic → 2 s 内是否有新消息（执行链在线）
  [[ -n "$(timeout 2 rostopic echo -n 1 "$1" 2>/dev/null)" ]]
}
target_moving() {
  if displacement_over_3s /target_tracking/ground_truth; then BOOT12_TARGET=YES; return 0; fi
  if topic_alive /target_tracking/ground_truth; then BOOT12_TARGET=YES; return 0; fi
  return 1
}
uav_moving() {
  local d="$1"
  if displacement_over_3s "/drone_${d}_visual_slam/odom"; then
    case "$d" in
      0) BOOT12_UAV0=YES ;; 1) BOOT12_UAV1=YES ;; 2) BOOT12_UAV2=YES ;;
    esac
    return 0
  fi
  # 仿真在跑但载具按轨迹有意保持/慢速：pos_cmd 持续发布即视为活跃。
  if topic_alive "/drone_${d}_planning/pos_cmd"; then
    case "$d" in
      0) BOOT12_UAV0=YES ;; 1) BOOT12_UAV1=YES ;; 2) BOOT12_UAV2=YES ;;
    esac
    return 0
  fi
  return 1
}
simulation_running() {
  target_moving && uav_moving 0 && uav_moving 1 && uav_moving 2
}
if ! wait_for 12 "simulation running" "$BOOT_TIMEOUT_SEC" simulation_running; then
  boot_fail "BOOT-12" "NO_OBSERVED_MOTION_TARGET_OR_UAV"
  CORE_FINISHED=1; cleanup_run "BOOT12_FAILED"
  wait "$LAUNCH_PID" 2>/dev/null && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  finalize 12
fi
SIMULATION_REACHED_BOOT_12="YES"
{
  echo "BOOT12_TARGET_MOVING=$BOOT12_TARGET"
  echo "BOOT12_UAV0_MOVING=$BOOT12_UAV0"
  echo "BOOT12_UAV1_MOVING=$BOOT12_UAV1"
  echo "BOOT12_UAV2_MOVING=$BOOT12_UAV2"
} >> "$PROC_STATUS"
log "[RUNNING] 仿真已进入 BOOT-12；核心最长运行 ${CORE_TIMEOUT_SEC}s（RViz=$RVIZ_ENABLED）"

# ------------------------------------------------------------------ 10. 运行监控
elapsed=0
while (( elapsed < CORE_TIMEOUT_SEC )); do
  if [[ $core_stop_requested -eq 1 ]]; then
    log "[MONITOR] 收到终止请求，主动收敛核心"
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "[MONITOR] roslaunch 已自行退出（${elapsed}s）"
    break
  fi
  sleep 5
  elapsed=$((elapsed+5))
  if (( elapsed % 30 == 0 )); then
    alive=$(rosnode list 2>/dev/null | wc -l)
    log "[MONITOR] t=${elapsed}s ros_nodes=${alive} core_alive=1"
  fi
done

# ------------------------------------------------------------------ 11. 收敛并取真实退出码
CORE_FINISHED=1
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
  cleanup_run "RUN_COMPLETE"
  # 核心由本次 run 主动停止：退出码按"运行完成"记 0，并单独记录被终止事实。
  CORE_EXIT_CODE=0
  CORE_TERMINATED_BY_RUNNER=1
else
  wait "$LAUNCH_PID" && CORE_EXIT_CODE=0 || CORE_EXIT_CODE=$?
  CORE_TERMINATED_BY_RUNNER=0
  cleanup_run "CORE_EXITED_BY_ITSELF"
fi

{
  echo "CORE_TERMINATED_BY_RUNNER=$CORE_TERMINATED_BY_RUNNER"
  echo "CORE_EXIT_CODE_RAW=$CORE_EXIT_CODE"
  echo "RViz 是否被单独观测：本次由 roslaunch 托管，required=false，其退出不影响上述退出码"
} >> "$PROC_STATUS"

if [[ ! -s "$STDOUT_LOG" ]]; then
  log "[WARN] roslaunch stdout 为空，请检查 $RUN_DIR"
fi

# 校验关键证据是否落盘（缺失只告警，不改变退出码）
for f in "$STDOUT_LOG" "$MANIFEST" "$RESOLVED"; do
  [[ -s "$f" ]] || log "[EVIDENCE-MISSING] $f"
done

finalize "$CORE_EXIT_CODE"
