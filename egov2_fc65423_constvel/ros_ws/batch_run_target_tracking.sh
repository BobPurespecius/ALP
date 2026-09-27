#!/bin/bash

set -euo pipefail

RUNS=5
DURATION=110
COOLDOWN=2
ANALYSIS_DIR=""
LAUNCH_FILE="target_tracking_platform_single.launch"
VERBOSE="false"
SUCCESS_ERROR="0.8"
PLANNER_BACKEND="egov2"
SCENE_NAME="platform_track"
VEHICLE_MODEL="neverlost_livox_pitch30"
CURRENT_PID=""

usage() {
    cat <<EOF
Usage:
  ./batch_run_target_tracking.sh SCENE RUNS DURATION PLANNER
  ./batch_run_target_tracking.sh [options]

Examples:
  ./batch_run_target_tracking.sh stair_climb 1 110 egov2
  ./batch_run_target_tracking.sh narrow_door 5 110 fov

Options:
  --runs N             Number of runs. Default: 5
  --duration SEC       Wall-time timeout for each run. Default: 110
  --cooldown SEC       Seconds to wait between runs. Default: 2
  --analysis-dir DIR   Output directory. Default: analysis/target_tracking_<timestamp>
  --success-error M    Success threshold for mean/final tracking error. Default: 0.8
  --planner egov2|fov  Planner backend. Default: egov2
  --scene NAME         Scene/world basename. Default: platform_track
  --vehicle NAME       PX4 Gazebo vehicle model. Default: neverlost_livox_pitch30
  -h, --help           Show this help
EOF
}

if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
    if [ "$#" -ne 4 ]; then
        echo "ERROR: positional form requires exactly 4 arguments: SCENE RUNS DURATION PLANNER"
        usage
        exit 1
    fi
    SCENE_NAME="$1"
    RUNS="$2"
    DURATION="$3"
    PLANNER_BACKEND="$4"
    shift 4
fi

while [ "$#" -gt 0 ]; do
    case "$1" in
        --runs)
            RUNS="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --cooldown)
            COOLDOWN="$2"
            shift 2
            ;;
        --analysis-dir)
            ANALYSIS_DIR="$2"
            shift 2
            ;;
        --success-error)
            SUCCESS_ERROR="$2"
            shift 2
            ;;
        --planner)
            PLANNER_BACKEND="$2"
            shift 2
            ;;
        --scene)
            SCENE_NAME="$2"
            shift 2
            ;;
        --vehicle)
            VEHICLE_MODEL="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "$SCRIPT_DIR/setup_runtime_env.sh" ]; then
    export EGO_PLANNER_VARIANT="egov2"
    set +u
    source "$SCRIPT_DIR/setup_runtime_env.sh" >/tmp/target_tracking_batch_runtime_env.log
    set -u
fi

EGO_V2_TRACKING_WS="$SCRIPT_DIR/src/EGO-Planner-v2/swarm-playground/tracking_ws"
EGO_V2_TRACKING_SRC="$EGO_V2_TRACKING_WS/src"
EGO_V2_TRACKING_DEVEL="$EGO_V2_TRACKING_WS/devel"
EGO_V2_PACKAGE_DIR="$EGO_V2_TRACKING_SRC/planner/plan_manage"
EGO_V2_BINARY_DIR="$EGO_V2_TRACKING_DEVEL/lib/ego_planner"
EGO_V2_PARAM_FILE="$EGO_V2_PACKAGE_DIR/launch/advanced_param.xml"
RUNTIME_ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}"
RUNTIME_CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"
RUNTIME_PYTHONPATH="${PYTHONPATH:-}"
RUNTIME_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"

if [ ! -f "$EGO_V2_PACKAGE_DIR/package.xml" ]; then
    echo "ERROR: EGO-Planner-v2 tracking package does not exist: $EGO_V2_PACKAGE_DIR"
    exit 1
fi
if [ ! -f "$EGO_V2_PARAM_FILE" ]; then
    echo "ERROR: EGO-Planner-v2 tracking launch file does not exist: $EGO_V2_PARAM_FILE"
    exit 1
fi

if [ -f "$EGO_V2_TRACKING_DEVEL/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$EGO_V2_TRACKING_DEVEL/setup.bash" --extend
    set -u
fi

# Both the legacy planner and EGO-Planner-v2 export a package named ego_planner.
# Keep the tracking workspace first so roslaunch includes the matching parameters
# and rosrun selects the corresponding binaries. Restore the general runtime
# paths because copied catkin setup files may omit PX4 and Gazebo packages.
export ROS_PACKAGE_PATH="$EGO_V2_TRACKING_SRC:$RUNTIME_ROS_PACKAGE_PATH:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$EGO_V2_TRACKING_DEVEL:$SCRIPT_DIR/devel:$RUNTIME_CMAKE_PREFIX_PATH:${CMAKE_PREFIX_PATH:-}"
export PYTHONPATH="$EGO_V2_TRACKING_DEVEL/lib/python3/dist-packages:$RUNTIME_PYTHONPATH:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$EGO_V2_TRACKING_DEVEL/lib:$SCRIPT_DIR/devel/lib:$RUNTIME_LD_LIBRARY_PATH:${LD_LIBRARY_PATH:-}"

if [ -z "$ANALYSIS_DIR" ]; then
    ANALYSIS_DIR="$SCRIPT_DIR/analysis/target_tracking_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$ANALYSIS_DIR"
ANALYSIS_DIR="$(cd "$ANALYSIS_DIR" && pwd)"
mkdir -p "$ANALYSIS_DIR/logs"
: > "$ANALYSIS_DIR/run_logs.txt"

PACKAGE_DIR="$SCRIPT_DIR/src/multi_uav_formation"
SCENE_FILE="$PACKAGE_DIR/scenes/$SCENE_NAME.json"
WORLD_FILE="$PACKAGE_DIR/worlds/$SCENE_NAME.world"
WORLD_GENERATOR="$PACKAGE_DIR/scripts/generate_world_from_scene.py"
TARGET_MODE="waypoints"
TARGET_SPEED="0.45"
TAKEOFF_HEIGHT="1.4"
TARGET_HEIGHT="1.4"
TARGET_WAYPOINTS_USE_Z="false"
LOCK_EGO_TRACKING_HEIGHT="true"
TRACK_MAX_CONTROL_Z="1.4"
TARGET_WAYPOINTS="-8.2,0.0;-6.0,0.3;-3.7,-0.2;-1.4,0.4;1.2,-0.2;3.6,0.4;6.3,-0.1;9.2,0.3;12.0,0.0;14.2,0.6;16.0,0.0;14.6,-0.5;12.2,0.4;10.0,-0.3;12.4,0.5;14.8,-0.4;16.0,0.0"
TARGET_SEGMENT_SPEEDS=""
UAV1_START_X="-8.8"
UAV1_START_Y="-2.6"
UAV2_START_X="-8.8"
UAV2_START_Y="0.0"
UAV3_START_X="-8.8"
UAV3_START_Y="2.6"
EGO_MAP_SIZE_X="44.0"
EGO_MAP_SIZE_Y="18.0"
EGO_MAP_SIZE_Z="4.5"
EGO_MAX_VEL="1.0"
EGO_MAX_ACC="1.5"
EGO_PLANNING_HORIZON="5.0"

if [ -f "$SCENE_FILE" ]; then
    eval "$(
        python3 - "$SCENE_FILE" <<'PY'
import json
import shlex
import sys

with open(sys.argv[1]) as f:
    scene = json.load(f)

tracking = scene.get('targetTracking') or {}
ego = scene.get('egoPlanner') or {}

def emit(name, value):
    print(f'{name}={shlex.quote(str(value))}')

if 'mode' in tracking:
    emit('TARGET_MODE', tracking['mode'])
if 'speed' in tracking:
    emit('TARGET_SPEED', tracking['speed'])
if 'takeoffHeight' in tracking:
    emit('TAKEOFF_HEIGHT', tracking['takeoffHeight'])
if 'targetHeight' in tracking:
    emit('TARGET_HEIGHT', tracking['targetHeight'])
if 'useWaypointZ' in tracking:
    emit('TARGET_WAYPOINTS_USE_Z', str(bool(tracking['useWaypointZ'])).lower())
if 'lockEgoTrackingHeight' in tracking:
    emit('LOCK_EGO_TRACKING_HEIGHT', str(bool(tracking['lockEgoTrackingHeight'])).lower())
if 'trackMaxControlZ' in tracking:
    emit('TRACK_MAX_CONTROL_Z', tracking['trackMaxControlZ'])
if 'waypointsENU' in tracking:
    waypoints = []
    for point in tracking['waypointsENU']:
        if len(point) >= 3:
            waypoints.append('{:.3f},{:.3f},{:.3f}'.format(float(point[0]), float(point[1]), float(point[2])))
        elif len(point) >= 2:
            waypoints.append('{:.3f},{:.3f}'.format(float(point[0]), float(point[1])))
    if waypoints:
        emit('TARGET_WAYPOINTS', ';'.join(waypoints))
if 'segmentSpeeds' in tracking:
    speeds = [str(float(speed)) for speed in tracking['segmentSpeeds']]
    if speeds:
        emit('TARGET_SEGMENT_SPEEDS', ';'.join(speeds))
takeoff_points = scene.get('takeoffPointENU') or []
for idx, point in enumerate(takeoff_points[:3], start=1):
    if len(point) >= 2:
        emit(f'UAV{idx}_START_X', point[0])
        emit(f'UAV{idx}_START_Y', point[1])
if 'mapSizeENU' in ego and len(ego['mapSizeENU']) >= 3:
    emit('EGO_MAP_SIZE_X', ego['mapSizeENU'][0])
    emit('EGO_MAP_SIZE_Y', ego['mapSizeENU'][1])
    emit('EGO_MAP_SIZE_Z', ego['mapSizeENU'][2])
if 'maxVel' in ego:
    emit('EGO_MAX_VEL', ego['maxVel'])
if 'maxAcc' in ego:
    emit('EGO_MAX_ACC', ego['maxAcc'])
if 'planningHorizon' in ego:
    emit('EGO_PLANNING_HORIZON', ego['planningHorizon'])
PY
    )"
fi

case "$PLANNER_BACKEND" in
    ego)
        echo "WARNING: planner backend 'ego' is a compatibility alias for 'egov2'."
        echo "Use 'egov2' to make the tracking planner version explicit."
        PLANNER_BACKEND="egov2"
        ;;
    egov2|fov)
        ;;
    *)
        echo "ERROR: unsupported planner backend: $PLANNER_BACKEND"
        echo "Expected one of: egov2, fov"
        exit 1
        ;;
esac

FOV_PLANNING_ENABLED="false"
if [ "$PLANNER_BACKEND" = "fov" ]; then
    FOV_PLANNING_ENABLED="true"
fi

force_cleanup() {
    set +e
    timeout 3s rosnode kill -a >/dev/null 2>&1
    pkill -f '[r]oslaunch.*target_tracking_platform' >/dev/null 2>&1
    pkill -f '[p]x4' >/dev/null 2>&1
    pkill -f '[g]zserver' >/dev/null 2>&1
    pkill -f '[g]zclient' >/dev/null 2>&1
    pkill -f '[m]avros_node' >/dev/null 2>&1
    pkill -f '[r]osmaster' >/dev/null 2>&1
    pkill -f '[r]oscore' >/dev/null 2>&1
    sleep 1
    set -e
}

stop_current_run() {
    set +e
    if [ -n "$CURRENT_PID" ] && kill -0 "$CURRENT_PID" >/dev/null 2>&1; then
        kill -INT "$CURRENT_PID" >/dev/null 2>&1
        sleep 1
    fi
    if [ -n "$CURRENT_PID" ] && kill -0 "$CURRENT_PID" >/dev/null 2>&1; then
        kill -TERM "$CURRENT_PID" >/dev/null 2>&1
        sleep 1
    fi
    if [ -n "$CURRENT_PID" ] && kill -0 "$CURRENT_PID" >/dev/null 2>&1; then
        kill -KILL "$CURRENT_PID" >/dev/null 2>&1
    fi
    CURRENT_PID=""
    set -e
}

handle_interrupt() {
    trap - INT TERM
    echo
    echo "Interrupted. Stopping target tracking batch..."
    stop_current_run
    force_cleanup
    exit 130
}

trap handle_interrupt INT TERM

echo "Target tracking batch"
if [ "$PLANNER_BACKEND" = "egov2" ]; then
    echo "Tracking algorithm: EGO-Planner-v2 tracking node + P230/MAVROS command bridge"
else
    echo "Tracking algorithm: EGO-Planner-v2 baseline with FOV visibility metrics + P230/MAVROS command bridge"
fi
echo "Planner backend: $PLANNER_BACKEND"
echo "Vehicle model: $VEHICLE_MODEL"
echo "Scene: $SCENE_FILE"
echo "World: $WORLD_FILE"
echo "Target waypoints: $TARGET_WAYPOINTS"
if [ -n "$TARGET_SEGMENT_SPEEDS" ]; then
    echo "Target segment speeds: $TARGET_SEGMENT_SPEEDS"
fi
echo "UAV starts: uav1=($UAV1_START_X,$UAV1_START_Y) uav2=($UAV2_START_X,$UAV2_START_Y) uav3=($UAV3_START_X,$UAV3_START_Y)"
echo "Target waypoint z enabled: $TARGET_WAYPOINTS_USE_Z"
echo "EGO map size: $EGO_MAP_SIZE_X x $EGO_MAP_SIZE_Y x $EGO_MAP_SIZE_Z"
echo "EGO limits: max_vel=$EGO_MAX_VEL max_acc=$EGO_MAX_ACC planning_horizon=$EGO_PLANNING_HORIZON"
echo "Runs: $RUNS"
echo "Duration timeout per run: $DURATION s"
echo "Analysis dir: $ANALYSIS_DIR"

if [ ! -f "$SCENE_FILE" ]; then
    echo "ERROR: scene file does not exist: $SCENE_FILE"
    exit 1
fi
if [ ! -f "$WORLD_GENERATOR" ]; then
    echo "ERROR: world generator does not exist: $WORLD_GENERATOR"
    exit 1
fi

if ! rospack find quadrotor_msgs >/dev/null 2>&1; then
    echo "ERROR: ROS cannot find package quadrotor_msgs."
    echo "Compile/source the workspace with EGO-Planner-v2 tracking_ws packages first."
    exit 1
fi

if [ "$PLANNER_BACKEND" = "egov2" ] || [ "$PLANNER_BACKEND" = "fov" ]; then
    RESOLVED_EGO_PLANNER="$(rospack find ego_planner 2>/dev/null || true)"
    if [ -z "$RESOLVED_EGO_PLANNER" ]; then
        echo "ERROR: ROS cannot find package ego_planner."
        echo "Expected EGO-Planner-v2 tracking package under $SCRIPT_DIR/src/EGO-Planner-v2"
        exit 1
    fi

    RESOLVED_EGO_PLANNER="$(readlink -f "$RESOLVED_EGO_PLANNER")"
    EXPECTED_EGO_PLANNER="$(readlink -f "$EGO_V2_PACKAGE_DIR")"
    if [ "$RESOLVED_EGO_PLANNER" != "$EXPECTED_EGO_PLANNER" ]; then
        echo "ERROR: ROS resolved the wrong ego_planner package."
        echo "Resolved: $RESOLVED_EGO_PLANNER"
        echo "Expected: $EXPECTED_EGO_PLANNER"
        echo "Check ROS_PACKAGE_PATH and stale catkin setup files."
        exit 1
    fi
    echo "EGO-Planner-v2 package: $RESOLVED_EGO_PLANNER"

    set +e
    timeout 1s rosrun ego_planner traj_server \
        __name:=target_tracking_batch_compile_check \
        >/tmp/target_tracking_ego_planner_check.log 2>&1
    CHECK_STATUS=$?
    set -e
    if grep -q "Couldn't find executable named traj_server" /tmp/target_tracking_ego_planner_check.log; then
        echo "ERROR: ego_planner/traj_server is not runnable. Compile the workspace first:"
        echo "  cd $SCRIPT_DIR"
        echo "  catkin_make"
        echo "Then rerun this batch script."
        cat /tmp/target_tracking_ego_planner_check.log
        exit 1
    fi
    if [ "$CHECK_STATUS" -ne 0 ] && [ "$CHECK_STATUS" -ne 124 ] && [ "$CHECK_STATUS" -ne 143 ]; then
        echo "WARNING: ego_planner/traj_server preflight returned status $CHECK_STATUS."
        echo "Continuing because the executable exists; see /tmp/target_tracking_ego_planner_check.log if launch fails."
    fi
fi

for ((run_idx=1; run_idx<=RUNS; run_idx++)); do
    LOG_FILE="$ANALYSIS_DIR/logs/target_tracking_run_$(printf '%03d' "$run_idx").csv"
    ROS_LOG="$ANALYSIS_DIR/logs/roslaunch_run_$(printf '%03d' "$run_idx").log"

    echo
    echo "[$run_idx/$RUNS] Starting target tracking run"
    python3 "$WORLD_GENERATOR" \
        --scene-file "$SCENE_FILE" \
        --world-out "$WORLD_FILE" \
        --world-name "$SCENE_NAME"
    force_cleanup
    sleep "$COOLDOWN"

    set +e
    timeout "$DURATION" roslaunch multi_uav_formation "$LAUNCH_FILE" \
        verbose:="$VERBOSE" \
        world:="$WORLD_FILE" \
        scene_file:="$SCENE_FILE" \
        vehicle:="$VEHICLE_MODEL" \
        planner_backend:="$PLANNER_BACKEND" \
        ego_package_dir:="$EGO_V2_PACKAGE_DIR" \
        ego_binary_dir:="$EGO_V2_BINARY_DIR" \
        ego_advanced_param_file:="$EGO_V2_PARAM_FILE" \
        fov_planning_enabled:="$FOV_PLANNING_ENABLED" \
        ego_map_size_x:="$EGO_MAP_SIZE_X" \
        ego_map_size_y:="$EGO_MAP_SIZE_Y" \
        ego_map_size_z:="$EGO_MAP_SIZE_Z" \
        ego_max_vel:="$EGO_MAX_VEL" \
        ego_max_acc:="$EGO_MAX_ACC" \
        ego_planning_horizon:="$EGO_PLANNING_HORIZON" \
        uav1_start_x:="$UAV1_START_X" \
        uav1_start_y:="$UAV1_START_Y" \
        uav2_start_x:="$UAV2_START_X" \
        uav2_start_y:="$UAV2_START_Y" \
        uav3_start_x:="$UAV3_START_X" \
        uav3_start_y:="$UAV3_START_Y" \
        target_mode:="$TARGET_MODE" \
        target_speed:="$TARGET_SPEED" \
        takeoff_height:="$TAKEOFF_HEIGHT" \
        target_height:="$TARGET_HEIGHT" \
        target_waypoints_use_z:="$TARGET_WAYPOINTS_USE_Z" \
        target_segment_speeds:="$TARGET_SEGMENT_SPEEDS" \
        lock_ego_tracking_height:="$LOCK_EGO_TRACKING_HEIGHT" \
        track_max_control_z:="$TRACK_MAX_CONTROL_Z" \
        target_waypoints:="$TARGET_WAYPOINTS" \
        track_log_file:="$LOG_FILE" >"$ROS_LOG" 2>&1 &
    CURRENT_PID=$!
    wait "$CURRENT_PID"
    status=$?
    CURRENT_PID=""
    set -e

    if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
        echo "[$run_idx/$RUNS] WARNING: roslaunch exited with status $status. See $ROS_LOG"
        tail -80 "$ROS_LOG" || true
    fi

    force_cleanup

    if [ -f "$LOG_FILE" ]; then
        echo "$LOG_FILE" >> "$ANALYSIS_DIR/run_logs.txt"
        echo "[$run_idx/$RUNS] Saved: $LOG_FILE"
    else
        echo "[$run_idx/$RUNS] WARNING: no tracking CSV produced: $LOG_FILE"
        tail -80 "$ROS_LOG" || true
    fi
done

python3 "$SCRIPT_DIR/src/multi_uav_formation/scripts/analyze_target_tracking_runs.py" \
    --log-dir "$ANALYSIS_DIR/logs" \
    --out-dir "$ANALYSIS_DIR" \
    --scene-file "$SCENE_FILE" \
    --success-error "$SUCCESS_ERROR"

echo
echo "Batch complete."
echo "Run logs: $ANALYSIS_DIR/run_logs.txt"
echo "Summary: $ANALYSIS_DIR/target_tracking_summary.csv"
echo "Team summary: $ANALYSIS_DIR/target_tracking_team_summary.csv"
echo "Overall: $ANALYSIS_DIR/target_tracking_overall.json"
