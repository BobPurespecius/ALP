#!/usr/bin/env bash
# 编译并运行"方位项 cost/gradient 一致性"有限差分核对。
# 只做这一项数学验证，不跑任何大规模测试矩阵。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/ros_ws"
BASE_ROS_WS="/home/bob/ALP/guidance/ros_ws"
OUT_DIR="${1:-$PROJECT_ROOT/runs/_math_check}"

source /opt/ros/noetic/setup.bash
source "$BASE_ROS_WS/devel/setup.bash"
source "$ROS_WS/devel/setup.bash"

mkdir -p "$OUT_DIR"
BIN="$OUT_DIR/bearing_gradient_finite_difference"

g++ -std=c++17 -O2 -o "$BIN" \
  "$SCRIPT_DIR/bearing_gradient_finite_difference.cpp" \
  -I"$ROS_WS/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/include" \
  -I"$ROS_WS/src/multi_uav_formation/include" \
  -I"$ROS_WS/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/include" \
  -I"$ROS_WS/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_env/include" \
  -I"$ROS_WS/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/path_searching/include" \
  $(pkg-config --cflags-only-I eigen3 2>/dev/null || echo "-I/usr/include/eigen3") \
  -I/opt/ros/noetic/include \
  -L"$ROS_WS/devel/lib" -L"$BASE_ROS_WS/devel/lib" -L/opt/ros/noetic/lib \
  -ltraj_opt -lplan_env -lpath_searching -ltraj_utils \
  -lroscpp -lrosconsole -lrostime -lrosconsole_log4cxx -lrosconsole_backend_interface \
  -lcv_bridge -lxmlrpcpp -lcpp_common -lroslib -lboost_system -lboost_filesystem \
  -lpthread -ldl -lm \
  $(pkg-config --cflags --libs pcl_common-1.10 2>/dev/null || pkg-config --cflags --libs pcl_common 2>/dev/null || echo "-I/usr/include/pcl-1.10")

fd_args=()
[[ "${ALP_FD_PREFIX:-0}" == "1" ]] && fd_args+=(--with-prefix)
"$BIN" "${fd_args[@]}" | tee "$OUT_DIR/bearing_gradient_finite_difference.txt"
