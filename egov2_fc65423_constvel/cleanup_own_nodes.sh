#!/usr/bin/env bash
# =============================================================================
# ALP 运行清理工具（按 RUN_ID ownership，绝不按进程名全机 kill）
#
# 用法：
#   ./cleanup_own_nodes.sh                 # 只报告：列出本项目相关进程与端口占用
#   ./cleanup_own_nodes.sh --run <RUN_ID>  # 只清理 runs/<RUN_ID>/pids.txt 里记录过的进程
#   ./cleanup_own_nodes.sh --stale         # 清理"命令行包含本项目根目录"的遗留进程
#                                          # （仍会逐个打印 PID/CMD，绝不匹配无主进程）
#
# 明确禁止：扫描 /proc 看到 ego_planner_node / roslaunch / rviz / traj_server
# 就直接 kill -9。那会误杀其它工程（包括别人的仿真）与用户的 shell/编辑器。
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
MODE="report"
RUN_ID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run)   MODE="run"; RUN_ID="${2:?--run 需要 RUN_ID}"; shift 2 ;;
    --stale) MODE="stale"; shift ;;
    -h|--help)
      sed -n '2,16p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

self_pid=$$
# 本脚本与其调用者同属一个 process group。绝不能把该组里的任何进程当作清理
# 目标——那会把调用它的 shell / runner 一起杀掉（历史上真的发生过）。
self_pgid="$(ps -o pgid= -p "$self_pid" 2>/dev/null | tr -d ' ' || true)"
killed=0
skipped_foreign=0

is_own_group() {
  local pid="$1" pgid
  [[ -n "$self_pgid" ]] || return 1
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  [[ "$pgid" == "$self_pgid" ]]
}

cmdline_of() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null || true; }

kill_one() {
  local pid="$1" label="$2"
  [[ "$pid" == "$self_pid" ]] && return 0
  is_own_group "$pid" && return 0
  local cmd; cmd="$(cmdline_of "$pid" 2>/dev/null)"
  [[ -z "$cmd" ]] && return 0
  case "$cmd" in *cleanup_own_nodes.sh*) return 0 ;; esac
  if kill -TERM "$pid" 2>/dev/null; then
    echo "  [TERM] pid=$pid ($label) cmd=${cmd:0:140}"
    killed=$((killed+1))
  fi
}

wait_and_force() {
  local pids=("$@")
  [[ ${#pids[@]} -eq 0 ]] && return 0
  sleep 2
  local pid
  for pid in "${pids[@]}"; do
    is_own_group "$pid" && continue
    if [[ -d "/proc/$pid" ]]; then
      kill -KILL "$pid" 2>/dev/null && echo "  [KILL] pid=$pid 未在 SIGTERM 后退出"
    fi
  done
}

report() {
  echo "[cleanup_own_nodes] mode=$MODE project_root=$PROJECT_ROOT"
  echo "--- 本项目相关的存活进程（只报告，不清理）---"
  local d pid cmd found=0
  for d in /proc/[0-9]*; do
    pid="${d#/proc/}"
    [[ "$pid" == "$self_pid" ]] && continue
    cmd="$(cmdline_of "$pid" 2>/dev/null)"
    [[ -z "$cmd" ]] && continue
    case "$cmd" in *"$PROJECT_ROOT"*) ;; *) continue ;; esac
    echo "  pid=$pid cmd=${cmd:0:160}"
    found=$((found+1))
  done
  [[ $found -eq 0 ]] && echo "  (无)"
  echo "--- 关键端口 ---"
  local port
  for port in 8081 11361; do
    local owners
    owners="$({ ss -lunp 2>/dev/null; ss -ltnp 2>/dev/null; } | awk -v p=":$port" '$0 ~ p' | head -5)"
    if [[ -n "$owners" ]]; then
      echo "  :$port 被占用："
      echo "$owners" | sed 's/^/    /'
    else
      echo "  :$port 空闲"
    fi
  done
}

case "$MODE" in
  report)
    report
    ;;
  run)
    pids_file="$PROJECT_ROOT/runs/$RUN_ID/pids.txt"
    if [[ ! -f "$pids_file" ]]; then
      echo "[cleanup_own_nodes] 找不到 $pids_file，拒绝清理（无 ownership 记录）" >&2
      exit 1
    fi
    echo "[cleanup_own_nodes] 只清理 runs/$RUN_ID 记录过的进程"
    mapfile -t target_pids < <(sort -u "$pids_file" | grep -E '^[0-9]+$' || true)
    for pid in "${target_pids[@]}"; do kill_one "$pid" "$RUN_ID"; done
    wait_and_force "${target_pids[@]}"
    ;;
  stale)
    echo "[cleanup_own_nodes] 清理命令行包含本项目根目录的遗留进程"
    mapfile -t target_pids < <(
      for d in /proc/[0-9]*; do
        pid="${d#/proc/}"
        [[ "$pid" == "$self_pid" ]] && continue
        is_own_group "$pid" && continue
        cmd="$(cmdline_of "$pid" 2>/dev/null)"
        [[ -z "$cmd" ]] && continue
        case "$cmd" in *cleanup_own_nodes.sh*) continue ;; esac
        case "$cmd" in *"$PROJECT_ROOT"*) echo "$pid" ;; esac
      done
    )
    if [[ ${#target_pids[@]} -eq 0 ]]; then
      echo "  没有需要清理的遗留进程"
    else
      for pid in "${target_pids[@]}"; do kill_one "$pid" "stale"; done
      wait_and_force "${target_pids[@]}"
    fi
    ;;
esac

# 端口状态复核（只报告）
for port in 8081 11361; do
  n=$({ ss -lun 2>/dev/null; ss -ltn 2>/dev/null; } | grep -c ":$port" || true)
  echo "[cleanup_own_nodes] port_${port}_listeners=$n"
done
echo "[cleanup_own_nodes] terminated=$killed foreign_processes_touched=0"
