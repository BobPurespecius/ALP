#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cli="$project_root/ros_ws/devel/lib/traj_utils/ablation_config_cli"
if [[ ! -x "$cli" ]]; then
  echo "ablation_config_cli 尚未构建；请先构建 traj_utils" >&2
  exit 1
fi
exec "$cli" --list
