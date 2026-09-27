#!/usr/bin/env bash
set -euo pipefail

# Independent Rao 2025 harness. Existing door/stair and target-tracking
# launchers are intentionally untouched.
CONTAINER_NAME="px4_noetic_dev"
ROS_WS="/app/guidance/ros_ws"
CSV_PATH="${RAO_CSV_PATH:-/tmp/rao_2025_reproduction.csv}"
RVIZ_FLAG="${RAO_RVIZ:-true}"
DURATION="${RAO_DURATION:-0.0}"
DOCKER_TTY=()

if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY=(-it)
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container ${CONTAINER_NAME} is not running." >&2
  exit 1
fi

docker exec "${DOCKER_TTY[@]}" "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/noetic/setup.bash && source ${ROS_WS}/devel/setup.bash 2>/dev/null || true; \
   roslaunch target_tracking rao_2025_full.launch rviz:=${RVIZ_FLAG} csv_path:=${CSV_PATH} duration:=${DURATION}"
