#!/usr/bin/env bash
# =============================================================================
# 已废弃（deprecated）：薄 alias。
#
# 本脚本不再设置任何环境变量、不再解析任何参数、不再调用其它 wrapper。
# 它只做一件事：exec 到唯一正式入口 scripts/run_alp_full_on.sh。
#
# 唯一命令：
#     cd /home/bob/ALP/egov2_fc65423_constvel
#     ./scripts/run_alp_full_on.sh
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[deprecated] run_show.sh 已废弃，转发到唯一入口 scripts/run_alp_full_on.sh" >&2
exec "$SCRIPT_DIR/scripts/run_alp_full_on.sh" "$@"
