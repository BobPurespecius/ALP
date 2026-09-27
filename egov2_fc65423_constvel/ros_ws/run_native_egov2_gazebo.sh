#!/usr/bin/env bash

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$workspace_dir/run_native_egov2_rviz.sh" "$@"
