#!/bin/bash

scene=$1
numbers=$2
leaders=$3
run_prefix_arg=$4
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ego_setup=""

find_ego_setup() {
    local candidate
    for candidate in \
        "$script_dir/src/ego-planner/devel/setup.bash" \
        "$script_dir/../../ego-planner/devel/setup.bash" \
        "/app/guidance/ros_ws/src/ego-planner/devel/setup.bash" \
        "/app/ego-planner/devel/setup.bash"
    do
        if [ -f "$candidate" ]; then
            ego_setup="$candidate"
            return 0
        fi
    done
    return 1
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
    python3 - "$scene_file" <<'PY'
import json
import sys

scene_file = sys.argv[1]
try:
    with open(scene_file, 'r') as f:
        scene = json.load(f)
except Exception:
    sys.exit(0)

ego = scene.get('egoPlanner') or {}
map_size = ego.get('mapSizeENU') or []
args = []
goals = ego.get('goalsENU') or []
platform = scene.get('platformData') or {}
PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM = 0.2

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
    args.append(f"max_vel:={float(ego['maxVel'])}")
if 'maxAcc' in ego:
    args.append(f"max_acc:={float(ego['maxAcc'])}")
if 'planningHorizon' in ego:
    args.append(f"planning_horizon:={float(ego['planningHorizon'])}")
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
    echo "Usage: ./multi_uav_formation.sh <scene_name> <number_of_uavs> [number_of_leaders] [run_prefix]"
    exit 1
fi

run_prefix="source \"$script_dir/setup_runtime_env.sh\";"

if scene_uses_ego; then
    if find_ego_setup; then
        echo "Using EGO planner setup: $ego_setup"
    else
        echo "WARNING: EGO planner setup file not found."
        echo "Searched:"
        echo "  $script_dir/src/ego-planner/devel/setup.bash"
        echo "  $script_dir/../../ego-planner/devel/setup.bash"
        echo "  /app/guidance/ros_ws/src/ego-planner/devel/setup.bash"
        echo "  /app/ego-planner/devel/setup.bash"
        echo "Continuing with normal MUSK windows; EGO planner bridge will be disabled."
    fi
fi

if scene_uses_ego && [ -n "$ego_setup" ]; then
    ego_launch_args="$(ego_launch_args_for_scene)"
    tmux new-window -n ego_planner "bash -c '$run_prefix roslaunch multi_uav_formation ego_more_obstacles.launch $ego_launch_args; exec bash'"
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
    if [ "$i" -eq 1 ]; then
        tmux send-keys -t uavs "bash -c '$run_prefix rosrun multi_uav_formation SingleRun.py --number $i --scene $scene $prefix_option; exec bash'" C-m
    else
        tmux split-window -t uavs -h "bash -c '$run_prefix rosrun multi_uav_formation SingleRun.py --number $i --scene $scene $prefix_option; exec bash'"
    fi
    tmux select-layout -t uavs tiled >/dev/null
done

tmux select-window -t uavs
