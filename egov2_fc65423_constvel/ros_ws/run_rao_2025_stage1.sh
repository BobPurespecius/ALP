#!/usr/bin/env bash
set -euo pipefail

# Stage 1 only validates and publishes the paper-aligned experiment contract.
# It deliberately starts no planner, simulator, PX4, Gazebo, EGO, or RViz.
CONTAINER_NAME="px4_noetic_dev"
ROS_WS="/app/guidance/ros_ws"
PROFILE="${RAO_PROFILE:-simulation}"
MANIFEST_PATH="${RAO_MANIFEST_PATH:-/tmp/rao_2025_manifest.json}"
CONFIG_PATH="${ROS_WS}/src/target_tracking/config/rao_2025_protocol.yaml"
DOCKER_TTY=()

case "${PROFILE}" in
  simulation|comparison) ;;
  *)
    echo "RAO_PROFILE must be simulation or comparison, got: ${PROFILE}" >&2
    exit 2
    ;;
esac

if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY=(-it)
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container ${CONTAINER_NAME} is not running." >&2
  exit 1
fi

docker exec "${DOCKER_TTY[@]}" "${CONTAINER_NAME}" bash -lc \
  "python3 '${ROS_WS}/src/target_tracking/scripts/rao_2025_protocol.py' \
     --check --config '${CONFIG_PATH}' --profile '${PROFILE}' --manifest '${MANIFEST_PATH}' && \
   source /opt/ros/noetic/setup.bash && source '${ROS_WS}/devel/setup.bash' && \
   roslaunch target_tracking rao_2025_stage1.launch \
     profile:='${PROFILE}' manifest_path:='${MANIFEST_PATH}'"
