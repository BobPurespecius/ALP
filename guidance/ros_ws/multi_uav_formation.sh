#!/bin/bash

scene=$1
numbers=$2
leaders=$3
run_prefix_arg=$4
planner_arg=${5:-ego}
enable_rviz_arg=${6:-false}
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
planner_setup=""
planner_src_dir=""
planner_devel_dir=""
planner_launch=""

set_planner_paths() {
    local setup_file="$1"
    local launch_file="$2"
    planner_setup="$setup_file"
    planner_devel_dir="$(cd "$(dirname "$setup_file")" && pwd)"
    planner_src_dir="$(cd "$planner_devel_dir/.." && pwd)/src"
    planner_launch="$launch_file"
}

find_ego_setup() {
    local candidate
    for candidate in \
        "$script_dir/src/ego-planner/devel/setup.bash" \
        "$script_dir/../../ego-planner/devel/setup.bash" \
        "/app/guidance/ros_ws/src/ego-planner/devel/setup.bash" \
        "/app/ego-planner/devel/setup.bash"
    do
        if [ -f "$candidate" ]; then
            set_planner_paths "$candidate" "ego_more_obstacles.launch"
            return 0
        fi
    done
    return 1
}

find_egov2_setup() {
    local candidate
    for candidate in \
        "$script_dir/src/EGO-Planner-v2/swarm-playground/tracking_ws/devel/setup.bash" \
        "/app/guidance/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/devel/setup.bash"
    do
        if [ -f "$candidate" ]; then
            set_planner_paths "$candidate" "ego_v2_more_obstacles.launch"
            return 0
        fi
    done
    return 1
}

find_planner_setup() {
    case "$planner_arg" in
        ego)
            find_ego_setup
            ;;
        egov2)
            find_egov2_setup
            ;;
        *)
            return 1
            ;;
    esac
}

scene_has_moving_obstacles() {
    local scene_file="$script_dir/src/multi_uav_formation/scenes/$scene.json"
    python3 - "$scene_file" <<'PY'
import json
import sys

scene_file = sys.argv[1]
try:
    with open(scene_file, 'r') as f:
        data = json.load(f)
except Exception:
    sys.exit(1)

sys.exit(0 if data.get('movingObstacleData') else 1)
PY
}

scene_uses_ego() {
    local scene_file="$script_dir/src/multi_uav_formation/scenes/$scene.json"
    python3 - "$scene_file" <<'PY'
import json
import sys

scene_file = sys.argv[1]
try:
    with open(scene_file, 'r') as f:
        data = json.load(f)
except Exception:
    sys.exit(1)

ego = data.get('egoPlanner') or {}
sys.exit(0 if ego.get('enabled', False) else 1)
PY
}

ego_launch_args_for_scene() {
    local scene_file="$script_dir/src/multi_uav_formation/scenes/$scene.json"
    python3 - "$scene_file" "$planner_arg" "$numbers" <<'PY'
import json
import sys

scene_file = sys.argv[1]
planner = sys.argv[2] if len(sys.argv) > 2 else 'ego'
requested_drone_count = int(sys.argv[3]) if len(sys.argv) > 3 else 5
try:
    with open(scene_file, 'r') as f:
        scene = json.load(f)
except Exception:
    sys.exit(0)

ego = scene.get('egoPlanner') or {}
map_size = ego.get('mapSizeENU') or []
args = []
goals = ego.get('goalsENU') or []
route_waypoints = ego.get('routeWaypointsENU') or []
platform = scene.get('platformData') or {}
PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM = 0.2

if requested_drone_count < 1 or requested_drone_count > 5:
    raise SystemExit('ordinary EGO launch supports between 1 and 5 UAVs')
if len(route_waypoints) > 4:
    raise SystemExit('egoPlanner.routeWaypointsENU supports at most 4 intermediate points')
for idx, point in enumerate(route_waypoints):
    if not isinstance(point, list) or len(point) < 3:
        raise SystemExit(f'egoPlanner.routeWaypointsENU[{idx}] must be [x, y, z]')

args.extend([
    f"drone_count:={requested_drone_count}",
    f"point_num:={len(route_waypoints) + 1}",
])
for idx, point in enumerate(route_waypoints):
    args.extend([
        f"route_point{idx}_x:={float(point[0])}",
        f"route_point{idx}_y:={float(point[1])}",
        f"route_point{idx}_z:={float(point[2])}",
    ])

def platform_top_z():
    if platform.get('topZ') is not None:
        return float(platform['topZ'])
    size = platform.get('sizeENU')
    if size and len(size) >= 3:
        return float(size[2])
    return None

def hover_reference_z():
    top_z = platform_top_z()
    if top_z is None:
        return None
    hover_above_platform = float(platform.get(
        'hoverAbovePlatform',
        PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM
    ))
    return top_z + hover_above_platform

def platform_safety_max_height():
    if platform.get('safetyMaxHeight') is not None:
        return float(platform['safetyMaxHeight'])
    hover_z = hover_reference_z()
    if hover_z is None:
        return None
    safety_margin = float(platform.get('safetyHeightMargin', 0.8))
    safety_min_height = float(platform.get('safetyMinHeight', 0.5))
    first_takeoff_z = None
    takeoff_points = scene.get('takeoffPointENU') or []
    if takeoff_points and len(takeoff_points[0]) >= 3:
        first_takeoff_z = float(takeoff_points[0][2])
    candidates = [
        hover_z + safety_margin,
        safety_min_height + 0.8,
    ]
    if first_takeoff_z is not None:
        candidates.append(first_takeoff_z + 0.5)
    return max(candidates)

platform_default_goal_z = hover_reference_z()
configured_goal_zs = [
    float(goal[2])
    for goal in goals
    if len(goal) >= 3
]

if len(map_size) >= 3:
    map_size_z = float(map_size[2])
    reference_zs = list(configured_goal_zs)
    if platform_default_goal_z is not None:
        reference_zs.append(float(platform_default_goal_z))
    if reference_zs:
        map_size_z = max(map_size_z, max(reference_zs) + float(platform.get('mapSizeZMargin', 1.0)))
    args.extend([
        f"map_size_x:={float(map_size[0])}",
        f"map_size_y:={float(map_size[1])}",
        f"map_size_z:={map_size_z}",
    ])
if 'maxVel' in ego:
    max_vel = ego.get('egov2MaxVel', ego['maxVel']) if planner == 'egov2' else ego['maxVel']
    args.append(f"max_vel:={float(max_vel)}")
if 'maxAcc' in ego:
    max_acc = ego.get('egov2MaxAcc', ego['maxAcc']) if planner == 'egov2' else ego['maxAcc']
    args.append(f"max_acc:={float(max_acc)}")
if 'planningHorizon' in ego:
    args.append(f"planning_horizon:={float(ego['planningHorizon'])}")
if 'presetStartDelayStep' in ego:
    args.append(f"preset_start_delay_step:={max(0.0, float(ego['presetStartDelayStep']))}")
if 'trackingErrorReplanThreshold' in ego:
    args.append(f"tracking_error_replan_threshold:={max(0.0, float(ego['trackingErrorReplanThreshold']))}")
if 'trackingVelocityErrorReplanThreshold' in ego:
    args.append(f"tracking_velocity_error_replan_threshold:={max(0.0, float(ego['trackingVelocityErrorReplanThreshold']))}")
args.append(f"obstacles_inflation:={max(0.0, float(ego.get('obstacleInflation', 0.35)))}")
args.append(f"obstacle_clearance:={max(0.0, float(ego.get('plannerObstacleClearance', 0.5)))}")
args.append(f"swarm_clearance:={max(0.0, float(ego.get('swarmClearance', 0.5)))}")
virtual_ceil_height = ego.get('virtualCeilHeight')
if virtual_ceil_height is None and platform:
    virtual_ceil_height = platform_safety_max_height()
if virtual_ceil_height is not None:
    args.append(f"virtual_ceil_height:={float(virtual_ceil_height)}")
if 'movingObstacleTimeAwareCostEnabled' in ego:
    args.append(
        f"use_time_aware_moving_obj_cost:={str(bool(ego['movingObstacleTimeAwareCostEnabled'])).lower()}"
    )
if 'movingObstacleTimeAwareClearance' in ego:
    args.append(f"moving_obj_clearance:={float(ego['movingObstacleTimeAwareClearance'])}")
if 'movingObstacleTimeAwareLambda' in ego:
    args.append(f"moving_obj_lambda:={float(ego['movingObstacleTimeAwareLambda'])}")
if 'movingObstacleTimeAwarePredictionHorizon' in ego:
    args.append(f"moving_obj_prediction_horizon:={float(ego['movingObstacleTimeAwarePredictionHorizon'])}")
if 'movingObstacleTimeAwareDecayTau' in ego:
    args.append(f"moving_obj_time_decay_tau:={float(ego['movingObstacleTimeAwareDecayTau'])}")
if 'movingObstacleTimeAwareMaxGrad' in ego:
    args.append(f"moving_obj_max_grad:={float(ego['movingObstacleTimeAwareMaxGrad'])}")
moving_obstacles = scene.get('movingObstacleData') or {}
if moving_obstacles:
    args.append(f"moving_obj_num:={len(moving_obstacles)}")
for idx, goal in enumerate(goals[:5]):
    if len(goal) >= 3:
        goal_z = goal[2]
    elif len(goal) >= 2 and platform_default_goal_z is not None:
        goal_z = platform_default_goal_z
    else:
        continue
    if len(goal) >= 2:
        args.extend([
            f"goal{idx}_x:={float(goal[0])}",
            f"goal{idx}_y:={float(goal[1])}",
            f"goal{idx}_z:={float(goal_z)}",
        ])

print(' '.join(args))
PY
}

if [ -z "$2" ]; then
    echo "Usage: ./multi_uav_formation.sh <scene_name> <number_of_uavs> [number_of_leaders] [run_prefix] [planner: ego|egov2] [enable_rviz: true|false]"
    exit 1
fi

case "$planner_arg" in
    ego|egov2)
        ;;
    *)
        echo "Invalid planner: $planner_arg"
        echo "Usage: ./multi_uav_formation.sh <scene_name> <number_of_uavs> [number_of_leaders] [run_prefix] [planner: ego|egov2]"
        exit 1
        ;;
esac

case "$enable_rviz_arg" in
    true|false)
        ;;
    *)
        echo "Invalid enable_rviz value: $enable_rviz_arg"
        echo "Expected true or false."
        exit 1
        ;;
esac

run_prefix="source \"$script_dir/setup_runtime_env.sh\";"
planner_run_prefix="$run_prefix"
single_run_prefix="$run_prefix"

if scene_uses_ego; then
    if find_planner_setup; then
        echo "Using planner backend: $planner_arg"
        echo "Using planner setup: $planner_setup"
        echo "Using planner launch: $planner_launch"
        # Some copied devel spaces keep stale absolute paths in .catkin. Repair the
        # package path after sourcing so rospack resolves the selected planner first.
        planner_run_prefix+=" source \"$planner_setup\";"
        planner_run_prefix+=" export ROS_PACKAGE_PATH=\"$planner_src_dir:$script_dir/src:\${ROS_PACKAGE_PATH:-}\";"
        planner_run_prefix+=" export CMAKE_PREFIX_PATH=\"$planner_devel_dir:$script_dir/devel:\${CMAKE_PREFIX_PATH:-}\";"
        planner_run_prefix+=" export PYTHONPATH=\"$planner_devel_dir/lib/python3/dist-packages:\${PYTHONPATH:-}\";"
        planner_run_prefix+=" export LD_LIBRARY_PATH=\"$planner_devel_dir/lib:\${LD_LIBRARY_PATH:-}\";"
        single_run_prefix="$planner_run_prefix"
    else
        echo "WARNING: $planner_arg planner setup file not found."
        echo "Searched:"
        if [ "$planner_arg" = "egov2" ]; then
            echo "  $script_dir/src/EGO-Planner-v2/swarm-playground/tracking_ws/devel/setup.bash"
            echo "  /app/guidance/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/devel/setup.bash"
        else
            echo "  $script_dir/src/ego-planner/devel/setup.bash"
            echo "  $script_dir/../../ego-planner/devel/setup.bash"
            echo "  /app/guidance/ros_ws/src/ego-planner/devel/setup.bash"
            echo "  /app/ego-planner/devel/setup.bash"
        fi
        echo "Continuing with normal MUSK windows; planner bridge will be disabled."
    fi
fi

if scene_uses_ego && [ -n "$planner_setup" ]; then
    ego_launch_args="$(ego_launch_args_for_scene)"
    tmux new-window -n "${planner_arg}_planner" "bash -c '$planner_run_prefix roslaunch multi_uav_formation $planner_launch $ego_launch_args; exec bash'"
fi

if scene_has_moving_obstacles; then
    moving_scene_file="$script_dir/src/multi_uav_formation/scenes/$scene.json"
    moving_world_file="$script_dir/src/multi_uav_formation/worlds/$scene.world"
    tmux new-window -n moving_obs "bash -c '$run_prefix rosrun multi_uav_formation move_obstacles.py --scene-file \"$moving_scene_file\" --world-file \"$moving_world_file\" --debug; exec bash'"
fi

tmux new-window -n uavs

for ((i=1; i<=numbers; i++))
do
    prefix_option=""
    if [ -n "$run_prefix_arg" ]; then
        prefix_option="--prefix $run_prefix_arg"
    fi
    planner_option="--planner-backend $planner_arg"
    rviz_option=""
    if [ "$enable_rviz_arg" = "true" ]; then
        rviz_option="--enable-rviz"
    fi
    if [ "$i" -eq 1 ]; then
        tmux send-keys -t uavs "bash -c '$single_run_prefix rosrun multi_uav_formation SingleRun.py --number $i --scene $scene $prefix_option $planner_option $rviz_option; exec bash'" C-m
    else
        tmux split-window -t uavs -h "bash -c '$single_run_prefix rosrun multi_uav_formation SingleRun.py --number $i --scene $scene $prefix_option $planner_option $rviz_option; exec bash'"
    fi
    tmux select-layout -t uavs tiled >/dev/null
done

tmux select-window -t uavs
