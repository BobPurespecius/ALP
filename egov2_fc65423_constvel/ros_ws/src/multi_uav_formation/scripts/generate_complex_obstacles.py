#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os


DEFAULT_OBSTACLE_COLOR = '0 0.8 0 0.85'


ZIGZAG_OBSTACLES = [
    (-3.4, -3.8, 0.55),
    (-2.7, -1.2, 0.60),
    (-2.0, 2.4, 0.50),
    (-1.2, -3.0, 0.45),
    (-0.6, 0.4, 0.70),
    (0.2, 3.5, 0.50),
    (1.0, -4.2, 0.50),
    (1.6, -1.5, 0.65),
    (2.3, 1.7, 0.55),
    (3.0, 4.0, 0.45),
    (3.8, -3.2, 0.65),
    (4.5, -0.2, 0.50),
    (5.1, 2.8, 0.70),
    (5.9, -4.1, 0.50),
    (6.5, -1.1, 0.55),
    (7.2, 1.9, 0.60),
    (8.0, -2.8, 0.45),
    (8.5, 3.8, 0.50),
]


GATE_OBSTACLES = [
    (-2.8, -2.0, 0.65),
    (-2.8, 2.0, 0.65),
    (-1.2, -3.8, 0.55),
    (-1.2, -0.8, 0.55),
    (-1.2, 2.4, 0.55),
    (0.4, -2.4, 0.70),
    (0.4, 1.2, 0.70),
    (1.8, -4.2, 0.50),
    (1.8, -1.2, 0.60),
    (1.8, 2.4, 0.60),
    (3.2, -2.6, 0.65),
    (3.2, 0.7, 0.65),
    (4.6, -4.0, 0.55),
    (4.6, -0.8, 0.55),
    (4.6, 2.2, 0.55),
    (6.0, -2.0, 0.70),
    (6.0, 1.5, 0.70),
    (7.4, -3.5, 0.50),
    (7.4, -0.2, 0.50),
    (7.4, 3.0, 0.50),
]


MOVING_OBSTACLES = [
    {
        'centerENU': [0.0, -2.4],
        'radius': 0.38,
        'axisENU': [0.0, 1.0],
        'amplitude': 1.1,
        'period': 8.0,
        'phase': 0.0,
    },
    {
        'centerENU': [3.2, 0.2],
        'radius': 0.4,
        'axisENU': [0.0, 1.0],
        'amplitude': 1.3,
        'period': 10.0,
        'phase': 1.57,
    },
    {
        'centerENU': [6.2, 3.5],
        'radius': 0.36,
        'axisENU': [0.0, -1.0],
        'amplitude': 1.0,
        'period': 9.0,
        'phase': 3.14,
    },
]


def obstacle_map(obstacles):
    return {
        str(idx): {
            'centerENU': [round(x, 3), round(y, 3)],
            'radius': round(radius, 3),
        }
        for idx, (x, y, radius) in enumerate(obstacles)
    }


def moving_obstacle_map(obstacles):
    return {
        str(idx): obstacle
        for idx, obstacle in enumerate(copy.deepcopy(obstacles))
    }


def obstacle_pose(obstacle, t):
    center = obstacle.get('centerENU', [0.0, 0.0])
    axis = obstacle.get('axisENU', [1.0, 0.0])
    amplitude = float(obstacle.get('amplitude', 0.0))
    period = max(float(obstacle.get('period', 1.0)), 1e-6)
    phase = float(obstacle.get('phase', 0.0))
    axis_norm = math.hypot(float(axis[0]), float(axis[1]))
    if axis_norm < 1e-6:
        unit_axis = [1.0, 0.0]
    else:
        unit_axis = [float(axis[0]) / axis_norm, float(axis[1]) / axis_norm]
    offset = amplitude * math.sin(2.0 * math.pi * t / period + phase)
    return [
        float(center[0]) + unit_axis[0] * offset,
        float(center[1]) + unit_axis[1] * offset,
    ]


def write_world(path, world_name, obstacles, height, moving_obstacles=None):
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.6">',
        f'  <world name="{world_name}">',
        '    <include><uri>model://sun</uri></include>',
        '    <include><uri>model://ground_plane</uri></include>',
        '    <physics name="default_physics" default="0" type="ode">',
        '      <gravity>0 0 -9.8066</gravity>',
        '      <ode>',
        '        <solver><type>quick</type><iters>10</iters><sor>1.3</sor></solver>',
        '        <constraints>',
        '          <cfm>0</cfm>',
        '          <erp>0.2</erp>',
        '          <contact_max_correcting_vel>100</contact_max_correcting_vel>',
        '          <contact_surface_layer>0.001</contact_surface_layer>',
        '        </constraints>',
        '      </ode>',
        '      <max_step_size>0.004</max_step_size>',
        '      <real_time_factor>1</real_time_factor>',
        '      <real_time_update_rate>250</real_time_update_rate>',
        '    </physics>',
    ]
    z = height / 2.0
    for idx, (x, y, radius) in enumerate(obstacles):
        lines.extend([
            f'    <model name="complex_obstacle_{idx}">',
            '      <static>true</static>',
            f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            f'          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>',
            f'          <material><ambient>{DEFAULT_OBSTACLE_COLOR}</ambient><diffuse>{DEFAULT_OBSTACLE_COLOR}</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
        ])
    for idx, obstacle in enumerate(moving_obstacles or []):
        x, y = obstacle_pose(obstacle, 0.0)
        radius = float(obstacle.get('radius', 0.4))
        obstacle_height = float(obstacle.get('height', height))
        z = float(obstacle.get('zMin', 0.0)) + obstacle_height / 2.0
        lines.extend([
            f'    <model name="complex_moving_obstacle_{idx}">',
            '      <static>false</static>',
            f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>',
            '      <link name="link">',
            '        <gravity>false</gravity>',
            '        <inertial>',
            '          <mass>1.0</mass>',
            '          <inertia>',
            '            <ixx>0.1</ixx><ixy>0.0</ixy><ixz>0.0</ixz>',
            '            <iyy>0.1</iyy><iyz>0.0</iyz>',
            '            <izz>0.1</izz>',
            '          </inertia>',
            '        </inertial>',
            '        <collision name="collision">',
            f'          <geometry><cylinder><radius>{radius:.3f}</radius><length>{obstacle_height:.3f}</length></cylinder></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            f'          <geometry><cylinder><radius>{radius:.3f}</radius><length>{obstacle_height:.3f}</length></cylinder></geometry>',
            f'          <material><ambient>{DEFAULT_OBSTACLE_COLOR}</ambient><diffuse>{DEFAULT_OBSTACLE_COLOR}</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
        ])
    lines.extend(['  </world>', '</sdf>', ''])
    with open(path, 'w') as f:
        f.write('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description='Generate harder obstacle JSON/world variants for more_obstacles.')
    parser.add_argument('--template', default='/app/guidance/ros_ws/src/multi_uav_formation/scenes/more_obstacles.json')
    parser.add_argument('--scene-out', default='/app/guidance/ros_ws/src/multi_uav_formation/scenes/more_obstacles_complex.json')
    parser.add_argument('--world-out', default='/app/guidance/ros_ws/src/multi_uav_formation/worlds/more_obstacles_complex.world')
    parser.add_argument('--pattern', choices=['zigzag', 'gates'], default='zigzag')
    parser.add_argument('--height', type=float, default=3.0)
    parser.add_argument('--cloud-resolution', type=float, default=0.18)
    parser.add_argument('--planning-horizon', type=float, default=6.0)
    parser.add_argument('--max-vel', type=float, default=0.9)
    parser.add_argument('--max-acc', type=float, default=1.3)
    parser.add_argument('--obstacle-clearance', type=float, default=0.35)
    args = parser.parse_args()

    with open(args.template, 'r') as f:
        scene = json.load(f)

    obstacles = ZIGZAG_OBSTACLES if args.pattern == 'zigzag' else GATE_OBSTACLES
    scene = copy.deepcopy(scene)
    scene['obstacleData'] = obstacle_map(obstacles)
    scene['movingObstacleData'] = moving_obstacle_map(MOVING_OBSTACLES)
    scene['nMovingObstacle'] = len(MOVING_OBSTACLES)
    scene['formationTime'] = max(float(scene.get('formationTime', 40)), 55)
    scene.setdefault('egoPlanner', {})
    scene['egoPlanner']['cloudResolution'] = args.cloud_resolution
    scene['egoPlanner']['planningHorizon'] = args.planning_horizon
    scene['egoPlanner']['maxVel'] = args.max_vel
    scene['egoPlanner']['maxAcc'] = args.max_acc
    scene['egoPlanner']['obstacleClearance'] = args.obstacle_clearance
    scene['egoPlanner']['obstacleHeight'] = args.height
    scene['egoPlanner']['goalTolerance'] = min(float(scene['egoPlanner'].get('goalTolerance', 0.45)), 0.4)

    os.makedirs(os.path.dirname(args.scene_out), exist_ok=True)
    os.makedirs(os.path.dirname(args.world_out), exist_ok=True)
    with open(args.scene_out, 'w') as f:
        json.dump(scene, f, indent=4)
        f.write('\n')

    world_name = os.path.splitext(os.path.basename(args.world_out))[0]
    write_world(args.world_out, world_name, obstacles, args.height, MOVING_OBSTACLES)

    print(f'Wrote scene: {args.scene_out}')
    print(f'Wrote world: {args.world_out}')
    print(f'Obstacle count: {len(obstacles)}')
    print(f'Moving obstacle count: {len(MOVING_OBSTACLES)}')


if __name__ == '__main__':
    main()
