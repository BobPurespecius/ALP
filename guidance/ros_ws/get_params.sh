#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./get_params.sh <scene_name>"
    exit 1
fi

scene=$1
echo "Getting params for $scene"
sceneconfig_path="$PWD/src/multi_uav_formation/scenes/$1.json"

eval "$(python3 - "$sceneconfig_path" <<'PY'
import json
import shlex
import sys

with open(sys.argv[1]) as scene_file:
    data = json.load(scene_file)

defaults = [
    (-4.8, -4.8),
    (-4.8, -2.4),
    (-4.8, 0.0),
    (-4.8, 2.4),
    (-4.8, 4.8),
]
points = data.get('takeoffPointENU') or data.get('preparePointENU') or []

def emit(name, value):
    print(f'export {name}={shlex.quote(str(value))}')

emit('UAV_NUM', int(data['number']))
emit('LEADERS_NUM', int(data['leaders']))
for index, (default_x, default_y) in enumerate(defaults, start=1):
    point = points[index - 1] if index <= len(points) else ()
    emit(f'UAV{index}_START_X', float(point[0]) if len(point) >= 1 else default_x)
    emit(f'UAV{index}_START_Y', float(point[1]) if len(point) >= 2 else default_y)
PY
)"

echo "UAV_NUM: $UAV_NUM"
echo "LEADERS_NUM: $LEADERS_NUM"
for ((uav_index=1; uav_index<=UAV_NUM; uav_index++)); do
    eval "uav_x=\${UAV${uav_index}_START_X}"
    eval "uav_y=\${UAV${uav_index}_START_Y}"
    echo "UAV${uav_index}_START: ($uav_x, $uav_y)"
done
