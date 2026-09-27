#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
while IFS= read -r mode; do
  printf './run_on.sh --headless --timeout 200 --ablation %s\n' "$mode"
done < <("$project_root/scripts/list_ablation_modes.sh")
