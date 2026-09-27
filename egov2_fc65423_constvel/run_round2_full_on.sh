#!/usr/bin/env bash
# =============================================================================
# 已废弃（deprecated）：薄 alias。
#
# 本脚本不再设置 ROS_HOME / ROS_LOG_DIR / FULL-ON 参数，也不再转发到
# run_constvel_gradient_rviz.sh（那个脚本同样已废弃）。
# 它只做一件事：exec 到唯一正式入口 scripts/run_alp_full_on.sh。
#
# 唯一命令：
#     cd /home/bob/ALP/egov2_fc65423_constvel
#     ./scripts/run_alp_full_on.sh
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[deprecated] run_round2_full_on.sh 已废弃，转发到唯一入口 scripts/run_alp_full_on.sh" >&2
exec "$SCRIPT_DIR/scripts/run_alp_full_on.sh" "$@"
