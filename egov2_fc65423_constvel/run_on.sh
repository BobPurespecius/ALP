#!/usr/bin/env bash
# =============================================================================
# ALP 一键启动（ON 模式，带 RViz）
#
# 用户只需要执行这一条：
#     cd /home/bob/ALP/egov2_fc65423_constvel
#     ./run_on.sh
#
# 语义（配置 truth 唯一来源 = traj_utils/ablation_config.h，runner 单点解析）：
#   ./run_on.sh                  # FULL production 配置（= --ablation full）
#   ./run_on.sh --ablation X     # 正式单变量消融（八选一）
#   ./run_on.sh --pt             # 旧兼容入口（UNSPECIFIED_LEGACY_COMPATIBILITY）
#   ./run_on.sh --headless       # 只跑仿真核心，不起 RViz（参数完全相同）
#   ./run_on.sh --timeout 600    # 核心最长运行时长
# =============================================================================
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

# ------------------------------------------------ 参数转发（本脚本只决定默认模式）
_forward=()
_mode_seen=false
_legacy_pt=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ablation) _mode_seen=true; _forward+=("--ablation" "${2:?--ablation 需要参数}"); shift 2 ;;
    --pt)       _legacy_pt=true; shift ;;
    *)          _forward+=("$1"); shift ;;
  esac
done

if [[ "$_mode_seen" == "false" && "$_legacy_pt" == "false" ]]; then
  # 语义叫 ON，就必须真正运行完整 production FULL 配置（阶段 A 修复：
  # 历史版本在无 --pt / --ablation 时实际 Team OFF，与"ON"语义相悖）。
  _forward=("--ablation" "full" "${_forward[@]+"${_forward[@]}"}")
fi

echo "==============================================================="
echo " ALP 一键 ON 启动"
echo " PROJECT_ROOT = $PROJECT_ROOT"
echo " 参数来源     = scripts/lib/alp_params.sh（唯一一份 launch 参数）"
echo "                + traj_utils/ablation_config.h（唯一一份模式矩阵）"
echo " 入口         = scripts/run_alp_full_on.sh（唯一入口）"
if [[ "$_mode_seen" == "true" ]]; then
  echo " 模式         = 显式 --ablation（见 runner 打印的 resolved truth）"
elif [[ "$_legacy_pt" == "true" ]]; then
  echo " 模式         = 旧兼容 --pt（UNSPECIFIED_LEGACY_COMPATIBILITY）"
else
  echo " 模式         = FULL（默认；等价 --ablation full）"
fi
echo " 注意：本脚本不再打印组件开关清单（历史硬编码曾与实际 launch 状态漂移："
echo "       旧版打印 team_visibility_optimizer=true 而默认链路实际为 false）。"
echo "       最终 resolved truth 的三重证据："
echo "         1) runner 启动段打印 ABLATION_* 与 launch 参数；"
echo "         2) 落盘 runs/<RUN_ID>/resolved_params.txt 与 roslaunch_argv.txt；"
echo "         3) 节点启动自证 [ablation-config]，runner 自动对账，不一致即 FAIL。"
echo " RViz         = true（除非显式 --headless）"
echo "==============================================================="

echo "[1/2] 清理本项目上一轮遗留进程（只动本项目目录下的进程）"
bash "$PROJECT_ROOT/cleanup_own_nodes.sh" --stale || true

echo "[2/2] 启动唯一入口 scripts/run_alp_full_on.sh"
exec bash "$PROJECT_ROOT/scripts/run_alp_full_on.sh" "${_forward[@]+"${_forward[@]}"}"
