#!/bin/bash

set -euo pipefail

SCENE="more_obstacles"
PLANNER="ego"
RVIZ_ENABLED="false"
RUNS=5
DURATION=60
COOLDOWN=2
STARTUP_TIMEOUT=180
SAVE_TIMEOUT=8
SAVE_SERVICE_TIMEOUT=15
STATUS_SERVICE_TIMEOUT=3
ANALYSIS_DIR=""
SESSION_NAME="px4-noetic-gazebo-multi-uav-formation"
SIM_TIME_POLL_PERIOD=0.05
GOAL_CHECK_PERIOD=0.5

usage() {
    cat <<EOF
Usage: ./batch_run_experiments.sh [options]

Options:
  --scene NAME          Scene JSON/world name. Default: more_obstacles
  --planner NAME        Planner backend: ego or egov2. Default: ego
  --rviz                Enable RViz and SingleRun visualization markers. Default: disabled
  --runs N             Number of runs. Default: 5
  --duration SEC       Gazebo sim-time seconds to let each run execute after startup. Default: 60
  --startup-timeout SEC Seconds to wait for Gazebo/PX4/SingleRun startup. Default: 180
  --cooldown SEC       Seconds to wait between runs after cleanup. Default: 2
  --save-timeout SEC   Seconds to wait for saved data files after duration. Default: 8
  --analysis-dir DIR   Analysis output directory. Default: analysis/batch_<timestamp>
  -h, --help           Show this help

Example:
  ./batch_run_experiments.sh --scene more_obstacles --runs 20 --duration 60
  ./batch_run_experiments.sh --scene platform --planner egov2 --runs 10 --duration 80
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --scene)
            SCENE="$2"
            shift 2
            ;;
        --planner)
            PLANNER="$2"
            shift 2
            ;;
        --rviz)
            RVIZ_ENABLED="true"
            shift
            ;;
        --runs)
            RUNS="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --startup-timeout)
            STARTUP_TIMEOUT="$2"
            shift 2
            ;;
        --cooldown)
            COOLDOWN="$2"
            shift 2
            ;;
        --save-timeout)
            SAVE_TIMEOUT="$2"
            shift 2
            ;;
        --analysis-dir)
            ANALYSIS_DIR="$2"
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

case "$PLANNER" in
    ego|egov2)
        ;;
    *)
        echo "Invalid planner: $PLANNER"
        usage
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "$SCRIPT_DIR/setup_runtime_env.sh" ]; then
    # Keep the batch supervisor in the same ROS/Gazebo environment as the tmux panes.
    # Without this, rosservice/rosnode startup checks can miss running SingleRun nodes.
    # shellcheck disable=SC1091
    set +u
    source "$SCRIPT_DIR/setup_runtime_env.sh" >/tmp/multi_uav_batch_runtime_env.log
    set -u
fi

if [ -z "$ANALYSIS_DIR" ]; then
    ANALYSIS_DIR="$SCRIPT_DIR/analysis/batch_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$ANALYSIS_DIR"

RUN_ROOT="$SCRIPT_DIR/data/multi_uav_formation/SingleRun"
mkdir -p "$RUN_ROOT"
RUN_LIST="$ANALYSIS_DIR/run_dirs.txt"
: > "$RUN_LIST"

expected_uavs_for_scene() {
    local scene_file="$SCRIPT_DIR/src/multi_uav_formation/scenes/$SCENE.json"
    if [ ! -f "$scene_file" ]; then
        echo 5
        return
    fi
    python3 - "$scene_file" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], 'r') as f:
        config = json.load(f)
    print(int(config.get('number', 5)))
except Exception:
    print(5)
PY
}

generate_world_for_scene() {
    local scene_file="$SCRIPT_DIR/src/multi_uav_formation/scenes/$SCENE.json"
    local world_file="$SCRIPT_DIR/src/multi_uav_formation/worlds/$SCENE.world"
    local generator="$SCRIPT_DIR/src/multi_uav_formation/scripts/generate_world_from_scene.py"

    if [ ! -f "$scene_file" ]; then
        echo "WARNING: scene file not found; cannot generate world: $scene_file" | tee -a "$ANALYSIS_DIR/warnings.log"
        return 1
    fi
    if [ ! -f "$generator" ]; then
        echo "WARNING: world generator not found: $generator" | tee -a "$ANALYSIS_DIR/warnings.log"
        return 1
    fi

    python3 "$generator" --scene-file "$scene_file" --world-out "$world_file"
}

stop_debug_recorder() {
    set +e
    if command -v rosnode >/dev/null 2>&1; then
        rosnode list 2>/dev/null | grep '/multi_uav_debug_recorder' | xargs -r -n1 rosnode kill >/dev/null 2>&1
    fi
    pkill -f 'record_debug.py' >/dev/null 2>&1
    pkill -f 'rosbag record' >/dev/null 2>&1
    set -e
}

stop_single_run_nodes() {
    set +e
    if command -v rosnode >/dev/null 2>&1; then
        rosnode list 2>/dev/null | grep '/single_run_' | xargs -r -n1 rosnode kill >/dev/null 2>&1
    fi
    set -e
}

count_data_files() {
    local run_dir="$1"
    if [ ! -d "$run_dir" ]; then
        echo 0
        return
    fi
    find "$run_dir" -maxdepth 1 -name 'data_*.pkl' -type f | wc -l
}

wait_for_data_files() {
    local run_dir="$1"
    local expected="$2"
    local timeout="$3"
    local elapsed=0
    local count=0

    while [ "$elapsed" -le "$timeout" ]; do
        count="$(count_data_files "$run_dir")"
        if [ "$count" -ge "$expected" ]; then
            echo "All $count/$expected data files saved."
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    count="$(count_data_files "$run_dir")"
    echo "WARNING: saved only $count/$expected data files after ${timeout}s." | tee -a "$ANALYSIS_DIR/warnings.log"
    return 1
}

wait_for_save_services() {
    local expected="$1"
    local timeout="$2"
    local elapsed=0
    local ready=0
    local service=""
    local services=""

    set +e
    while [ "$elapsed" -le "$timeout" ]; do
        ready=0
        services="$(timeout "${STATUS_SERVICE_TIMEOUT}s" rosservice list 2>/dev/null || true)"
        for ((uav=1; uav<=expected; uav++)); do
            service="/single_run_${uav}/save_log"
            if printf '%s\n' "$services" | grep -qx "$service"; then
                ready=$((ready + 1))
            fi
        done

        if [ "$ready" -eq "$expected" ]; then
            echo "All $ready/$expected SingleRun save services are ready."
            set -e
            return 0
        fi

        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "WARNING: only $ready/$expected SingleRun save services ready after ${timeout}s." | tee -a "$ANALYSIS_DIR/warnings.log"
    set -e
    return 1
}

wait_for_mavros_states() {
    local expected="$1"
    local timeout="$2"
    local rc=0

    set +e
    python3 - "$expected" "$timeout" <<'PY'
import sys
import time

expected = int(sys.argv[1])
timeout = float(sys.argv[2])

try:
    import rospy
    from mavros_msgs.msg import State
except Exception as exc:
    print(f"WARNING: cannot import ROS/MAVROS Python dependencies: {exc}", flush=True)
    sys.exit(1)

states = {}

def callback_for(uav_number):
    def callback(msg):
        states[uav_number] = msg
    return callback

try:
    rospy.init_node('wait_for_mavros_states', anonymous=True, disable_signals=True)
    subscribers = [
        rospy.Subscriber(f'/uav{uav}/mavros/state', State, callback_for(uav), queue_size=1)
        for uav in range(1, expected + 1)
    ]
except Exception as exc:
    print(f"WARNING: failed to subscribe MAVROS states: {exc}", flush=True)
    sys.exit(1)

deadline = time.time() + timeout
last_report = 0.0
last_summary = ''

while time.time() <= deadline and not rospy.is_shutdown():
    ready = 0
    summary_parts = []
    for uav in range(1, expected + 1):
        state = states.get(uav)
        if state is None:
            summary_parts.append(f'uav{uav}:no_msg')
            continue

        mode = state.mode or '<empty>'
        summary_parts.append(
            f'uav{uav}:connected={int(bool(state.connected))},mode={mode},armed={int(bool(state.armed))}'
        )
        if state.connected and state.mode:
            ready += 1

    last_summary = '; '.join(summary_parts)
    if ready == expected:
        print(f"All {ready}/{expected} MAVROS FCU states are ready.")
        print(f"MAVROS states: {last_summary}")
        sys.exit(0)

    now = time.time()
    if now - last_report >= 5.0:
        print(f"Waiting for MAVROS FCU states: {ready}/{expected} ready. {last_summary}", flush=True)
        last_report = now

    time.sleep(0.1)

print(f"WARNING: MAVROS FCU states not ready after {timeout:.1f}s. {last_summary}", flush=True)
sys.exit(1)
PY
    rc=$?
    set -e
    return "$rc"
}

gazebo_sim_time() {
    local response=""
    local direct_time=""
    local secs=""
    local nsecs=""

    if ! command -v rosservice >/dev/null 2>&1; then
        return 1
    fi

    response="$(timeout 2s rosservice call /gazebo/get_world_properties "{}" 2>/dev/null || true)"
    direct_time="$(printf '%s\n' "$response" | awk '/sim_time:/ && NF >= 2 {print $2; exit}')"
    if printf '%s\n' "$direct_time" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
        printf '%s\n' "$direct_time"
        return 0
    fi

    secs="$(printf '%s\n' "$response" | awk '/secs:/ {print $2; exit}')"
    nsecs="$(printf '%s\n' "$response" | awk '/nsecs:/ {print $2; exit}')"
    if printf '%s\n' "$secs" | grep -Eq '^[0-9]+$' && printf '%s\n' "${nsecs:-0}" | grep -Eq '^[0-9]+$'; then
        awk -v secs="$secs" -v nsecs="${nsecs:-0}" 'BEGIN { printf "%.9f\n", secs + nsecs / 1000000000.0 }'
        return 0
    fi

    return 1
}

wait_for_sim_time() {
    local timeout="$1"
    local elapsed=0
    local sim_time=""

    while [ "$elapsed" -le "$timeout" ]; do
        sim_time="$(gazebo_sim_time || true)"
        if [ -n "$sim_time" ]; then
            printf '%s\n' "$sim_time"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 1
}

all_uavs_reached_goal() {
    local expected="$1"
    local service=""
    local response=""

    if ! command -v rosservice >/dev/null 2>&1; then
        return 1
    fi

    for ((uav=1; uav<=expected; uav++)); do
        service="/single_run_${uav}/status"
        response="$(timeout "${STATUS_SERVICE_TIMEOUT}s" rosservice call "$service" "{}" 2>/dev/null || true)"
        if printf '%s\n' "$response" | grep -q 'egoGoalReached=1'; then
            continue
        fi
        if printf '%s\n' "$response" | grep -Eq 'egoGoalReached[^[:alnum:]_]+true'; then
            continue
        fi
        return 1
    done

    return 0
}

wait_for_sim_duration_or_goal() {
    local start_sim_time="$1"
    local duration="$2"
    local expected="$3"
    local checks=0
    local last_goal_check_epoch=0
    local now_epoch=0
    local current_sim_time=""
    local sim_elapsed="0"
    local sim_remaining="$duration"
    local duration_reached=0

    while true; do
        current_sim_time="$(gazebo_sim_time || true)"
        if [ -z "$current_sim_time" ]; then
            echo "WARNING: cannot read Gazebo sim time; stopping this run to avoid uncontrolled overtime." | tee -a "$ANALYSIS_DIR/warnings.log"
            return 0
        fi

        duration_reached="$(awk -v now="$current_sim_time" -v start="$start_sim_time" -v dur="$duration" 'BEGIN { print ((now - start) >= dur) ? 1 : 0 }')"
        if [ "$duration_reached" -eq 1 ]; then
            sim_elapsed="$(awk -v now="$current_sim_time" -v start="$start_sim_time" 'BEGIN { printf "%.2f", now - start }')"
            echo "Sim-time duration reached (${sim_elapsed}s >= ${duration}s); stopping this run."
            return 0
        fi

        if [ "$checks" -eq 0 ] || [ $((checks % 50)) -eq 0 ]; then
            sim_elapsed="$(awk -v now="$current_sim_time" -v start="$start_sim_time" 'BEGIN { printf "%.2f", now - start }')"
            sim_remaining="$(awk -v dur="$duration" -v elapsed_sim="$sim_elapsed" 'BEGIN { rem = dur - elapsed_sim; if (rem < 0) rem = 0; printf "%.2f", rem }')"
            echo "Running simulation; sim ${sim_elapsed}/${duration}s, ${sim_remaining}s until sim-time limit..."
        fi

        now_epoch="$(date +%s.%N)"
        if awk -v now="$now_epoch" -v last="$last_goal_check_epoch" -v period="$GOAL_CHECK_PERIOD" 'BEGIN { exit !((now - last) >= period) }'; then
            last_goal_check_epoch="$now_epoch"
            if all_uavs_reached_goal "$expected"; then
                echo "All $expected/$expected UAVs reached final goals; ending this run early."
                return 0
            fi
        fi

        sleep "$SIM_TIME_POLL_PERIOD"
        checks=$((checks + 1))
    done
}

request_save_logs() {
    local expected="$1"
    local ok=0
    local service=""
    local status_dir=""
    local pid=""
    local pids=()

    set +e
    if ! command -v rosservice >/dev/null 2>&1; then
        echo "WARNING: rosservice not found; cannot request SingleRun save services." | tee -a "$ANALYSIS_DIR/warnings.log"
        set -e
        return 1
    fi

    status_dir="$(mktemp -d)"
    for ((uav=1; uav<=expected; uav++)); do
        service="/single_run_${uav}/save_log"
        (
            if timeout "${SAVE_SERVICE_TIMEOUT}s" rosservice call "$service" "{}" >/dev/null 2>&1; then
                echo "ok" > "$status_dir/$uav"
            else
                echo "fail" > "$status_dir/$uav"
            fi
        ) &
        pids+=("$!")
    done

    for pid in "${pids[@]}"; do
        wait "$pid"
    done

    for ((uav=1; uav<=expected; uav++)); do
        service="/single_run_${uav}/save_log"
        if [ "$(cat "$status_dir/$uav" 2>/dev/null)" = "ok" ]; then
            echo "Requested save: $service"
            ok=$((ok + 1))
        else
            echo "WARNING: failed to request save service: $service" | tee -a "$ANALYSIS_DIR/warnings.log"
        fi
    done
    rm -rf "$status_dir"

    set -e
    [ "$ok" -eq "$expected" ]
}

force_cleanup() {
    set +e
    stop_debug_recorder
    stop_single_run_nodes
    set +e
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1
    pkill -f '[r]oslaunch.*multi_uav_mavros_sitl' >/dev/null 2>&1
    pkill -f '[r]oslaunch.*ego_more_obstacles' >/dev/null 2>&1
    pkill -f '[r]oslaunch.*ego_v2_more_obstacles' >/dev/null 2>&1
    pkill -x px4 >/dev/null 2>&1
    pkill -x gazebo >/dev/null 2>&1
    pkill -x gzserver >/dev/null 2>&1
    pkill -x gzclient >/dev/null 2>&1
    set -e
}

reset_px4_iris_param_cache() {
    local ros_home="${ROS_HOME:-}"
    local cleared=0

    if [ -z "$ros_home" ]; then
        if [ -z "${HOME:-}" ]; then
            return 0
        fi
        ros_home="$HOME/.ros"
    fi

    if [ ! -d "$ros_home" ]; then
        return 0
    fi

    set +e
    cleared="$(
        find "$ros_home" -maxdepth 2 -type f \
            \( -path "$ros_home/sitl_iris_*/parameters.bson" -o \
               -path "$ros_home/sitl_iris_*/parameters_backup.bson" -o \
               -path "$ros_home/sitl_iris_*/param_import_fail.bson" \) \
            -print -delete 2>/dev/null | wc -l
    )"
    cleared=$((cleared + $(
        find "$ros_home" -maxdepth 4 -type f \
            \( -path "$ros_home/sitl_*/eeprom/parameters_*" -o \
               -path "$ros_home/sitl_*/eeprom/parameters_backup_*" \) \
            -print -delete 2>/dev/null | wc -l
    )))
    set -e

    if [ "${cleared:-0}" -gt 0 ] 2>/dev/null; then
        echo "Cleared $cleared stale PX4 SITL parameter cache file(s)."
    fi
}

pause_gazebo_physics() {
    set +e
    if command -v rosservice >/dev/null 2>&1; then
        timeout 3s rosservice call /gazebo/pause_physics "{}" >/dev/null 2>&1 || true
    fi
    set -e
}

finish_run() {
    local run_dir="$1"
    local expected="$2"

    pause_gazebo_physics
    request_save_logs "$expected" || true
    wait_for_data_files "$run_dir" "$expected" "$SAVE_TIMEOUT" || true
    stop_debug_recorder
    force_cleanup
}

trap force_cleanup EXIT

echo "Batch scene: $SCENE"
echo "Planner: $PLANNER"
echo "Runs: $RUNS"
echo "Duration per run: $DURATION s sim time"
echo "Cooldown between runs: $COOLDOWN s"
echo "Startup timeout: $STARTUP_TIMEOUT s"
echo "Save timeout: $SAVE_TIMEOUT s"
echo "Analysis dir: $ANALYSIS_DIR"

EXPECTED_UAVS="$(expected_uavs_for_scene)"
echo "Expected UAV data files per run: $EXPECTED_UAVS"

for ((run_idx=1; run_idx<=RUNS; run_idx++)); do
    PREFIX="batch_${SCENE}_${PLANNER}_$(date +%Y%m%d_%H%M%S)_$(printf '%03d' "$run_idx")"
    RUN_DIR="$RUN_ROOT/$PREFIX"

    echo
    echo "[$run_idx/$RUNS] Starting run prefix: $PREFIX"
    force_cleanup
    reset_px4_iris_param_cache
    sleep "$COOLDOWN"

    if ! generate_world_for_scene; then
        echo "[$run_idx/$RUNS] ERROR: failed to generate Gazebo world from scene; cleaning up." | tee -a "$ANALYSIS_DIR/warnings.log"
        force_cleanup
        sleep "$COOLDOWN"
        continue
    fi

    if ! tmuxinator start --no-attach -p tmuxinator.yml scene="$SCENE" planner="$PLANNER" run_prefix="$PREFIX" enable_rviz="$RVIZ_ENABLED"; then
        echo "[$run_idx/$RUNS] ERROR: tmuxinator failed to start; cleaning up." | tee -a "$ANALYSIS_DIR/warnings.log"
        force_cleanup
        sleep "$COOLDOWN"
        continue
    fi

    echo "[$run_idx/$RUNS] Simulation launched."
    echo "[$run_idx/$RUNS] Waiting for Gazebo/PX4/SingleRun startup for up to ${STARTUP_TIMEOUT}s..."
    if ! wait_for_save_services "$EXPECTED_UAVS" "$STARTUP_TIMEOUT"; then
        echo "[$run_idx/$RUNS] ERROR: startup did not complete; cleaning up." | tee -a "$ANALYSIS_DIR/warnings.log"
        force_cleanup
        sleep "$COOLDOWN"
        continue
    fi
    if ! wait_for_mavros_states "$EXPECTED_UAVS" "$STARTUP_TIMEOUT"; then
        echo "[$run_idx/$RUNS] ERROR: MAVROS/PX4 FCU state did not become ready; cleaning up." | tee -a "$ANALYSIS_DIR/warnings.log"
        force_cleanup
        sleep "$COOLDOWN"
        continue
    fi

    START_SIM_TIME="$(wait_for_sim_time "$STARTUP_TIMEOUT" || true)"
    if [ -z "$START_SIM_TIME" ]; then
        echo "[$run_idx/$RUNS] ERROR: could not read Gazebo sim time; cleaning up." | tee -a "$ANALYSIS_DIR/warnings.log"
        force_cleanup
        sleep "$COOLDOWN"
        continue
    fi

    echo "[$run_idx/$RUNS] Startup ready at sim_time=${START_SIM_TIME}s. Stopping when all UAVs reach goals or after ${DURATION}s sim time."

    STOP_REASON="$(
        python3 "$SCRIPT_DIR/src/multi_uav_formation/scripts/wait_for_stop_condition.py" \
            --duration "$DURATION" \
            --expected-uavs "$EXPECTED_UAVS" \
            --scene-file "$SCRIPT_DIR/src/multi_uav_formation/scenes/$SCENE.json" \
            --goal-check-period "$GOAL_CHECK_PERIOD" | tail -1
    )"
    echo "[$run_idx/$RUNS] Stop condition: ${STOP_REASON:-unknown}"

    echo "[$run_idx/$RUNS] Run stop condition reached; pausing Gazebo, saving data, then cleaning up simulation..."
    finish_run "$RUN_DIR" "$EXPECTED_UAVS"
    sleep "$COOLDOWN"

    if [ -d "$RUN_DIR" ]; then
        echo "$RUN_DIR" >> "$RUN_LIST"
        echo "[$run_idx/$RUNS] Saved: $RUN_DIR"
    else
        echo "[$run_idx/$RUNS] WARNING: expected run directory not found: $RUN_DIR" | tee -a "$ANALYSIS_DIR/warnings.log"
    fi
done

echo
echo "Analyzing collected runs..."
ANALYZE_ARGS=()
while IFS= read -r run_dir; do
    if [ -n "$run_dir" ]; then
        ANALYZE_ARGS+=(--run-dir "$run_dir")
    fi
done < "$RUN_LIST"

if [ "${#ANALYZE_ARGS[@]}" -eq 0 ]; then
    echo "No run directories collected; skip analysis."
    exit 1
fi

python3 "$SCRIPT_DIR/src/multi_uav_formation/scripts/analyze_runs.py" \
    "${ANALYZE_ARGS[@]}" \
    --out-dir "$ANALYSIS_DIR"

echo
echo "Batch complete."
echo "Run list: $RUN_LIST"
echo "Summary: $ANALYSIS_DIR/run_summary.csv"
echo "Vehicle details: $ANALYSIS_DIR/vehicle_summary.csv"
echo "Overall: $ANALYSIS_DIR/overall_summary.json"
