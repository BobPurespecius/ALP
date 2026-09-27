#!/usr/bin/env python3

import argparse
import json
import math
import os


DEFAULT_OBSTACLE_COLOR = [0.0, 0.8, 0.0, 0.85]
DEFAULT_PLATFORM_COLOR = [0.48, 0.48, 0.48, 1.0]


def sorted_items(data):
    def key(item):
        name, _value = item
        try:
            return int(name)
        except ValueError:
            return name

    return sorted((data or {}).items(), key=key)


def rgba(value, default):
    if isinstance(default, str):
        default_parts = [float(component) for component in default.split()]
    else:
        default_parts = [float(component) for component in default]

    if value is None:
        parts = default_parts
    elif isinstance(value, str):
        parts = [float(component) for component in value.split()]
    else:
        parts = [float(component) for component in value]
    if len(parts) < 4:
        parts = parts + default_parts[len(parts):]
    return ' '.join(f'{component:.3f}'.rstrip('0').rstrip('.') for component in parts[:4])


def unit_axis(axis):
    if not axis or len(axis) < 2:
        return [1.0, 0.0]
    norm = math.hypot(float(axis[0]), float(axis[1]))
    if norm < 1e-6:
        return [1.0, 0.0]
    return [float(axis[0]) / norm, float(axis[1]) / norm]


def moving_obstacle_pose(obstacle, t=0.0):
    center = obstacle.get('centerENU', [0.0, 0.0])
    axis = unit_axis(obstacle.get('axisENU', [1.0, 0.0]))
    amplitude = float(obstacle.get('amplitude', 0.0))
    period = max(float(obstacle.get('period', 1.0)), 1e-6)
    phase = float(obstacle.get('phase', 0.0))
    offset = amplitude * math.sin(2.0 * math.pi * t / period + phase)
    return [float(center[0]) + axis[0] * offset, float(center[1]) + axis[1] * offset]


def goals_center(scene):
    goals = (scene.get('egoPlanner') or {}).get('goalsENU') or []
    xy_goals = [
        [float(goal[0]), float(goal[1])]
        for goal in goals
        if len(goal) >= 2
    ]
    if not xy_goals:
        return None
    return [
        sum(goal[0] for goal in xy_goals) / len(xy_goals),
        sum(goal[1] for goal in xy_goals) / len(xy_goals),
    ]


def add_cylinder_model(lines, name, x, y, radius, height, z_min, static, color):
    z = float(z_min) + float(height) / 2.0
    lines.extend([
        f'    <model name="{name}">',
        f'      <static>{str(bool(static)).lower()}</static>',
        f'      <pose>{float(x):.3f} {float(y):.3f} {z:.3f} 0 0 0</pose>',
        '      <link name="link">',
    ])

    if not static:
        lines.extend([
            '        <gravity>false</gravity>',
            '        <inertial>',
            '          <mass>1.0</mass>',
            '          <inertia>',
            '            <ixx>0.1</ixx><ixy>0.0</ixy><ixz>0.0</ixz>',
            '            <iyy>0.1</iyy><iyz>0.0</iyz>',
            '            <izz>0.1</izz>',
            '          </inertia>',
            '        </inertial>',
        ])

    lines.extend([
        '        <collision name="collision">',
        f'          <geometry><cylinder><radius>{float(radius):.3f}</radius><length>{float(height):.3f}</length></cylinder></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        f'          <geometry><cylinder><radius>{float(radius):.3f}</radius><length>{float(height):.3f}</length></cylinder></geometry>',
        f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>',
        '        </visual>',
        '      </link>',
        '    </model>',
    ])


def add_wall_model(lines, name, center, size, yaw, z_min, color):
    if len(size) < 2:
        raise ValueError('wall sizeENU needs at least length and thickness')
    height = float(size[2] if len(size) >= 3 else 3.0)
    z = float(z_min) + height / 2.0
    lines.extend([
        f'    <model name="{name}">',
        '      <static>true</static>',
        f'      <pose>{float(center[0]):.3f} {float(center[1]):.3f} {z:.3f} 0 0 {float(yaw):.6f}</pose>',
        '      <link name="link">',
        '        <collision name="collision">',
        f'          <geometry><box><size>{float(size[0]):.3f} {float(size[1]):.3f} {height:.3f}</size></box></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        f'          <geometry><box><size>{float(size[0]):.3f} {float(size[1]):.3f} {height:.3f}</size></box></geometry>',
        f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>',
        '        </visual>',
        '      </link>',
        '    </model>',
    ])


def add_platform_model(lines, platform, scene):
    center = platform.get('centerENU', [0.0, 0.0])
    if platform.get('centerENU') is None:
        center = goals_center(scene) or center
    size = platform.get('sizeENU', [1.0, 1.0, 0.5])
    height = float(size[2])
    top_z = float(platform.get('topZ', height))
    z = top_z - height / 2.0
    color = rgba(platform.get('colorRGBA'), DEFAULT_PLATFORM_COLOR)
    name = platform.get('modelName', 'goal_platform')

    lines.extend([
        f'    <model name="{name}">',
        '      <static>true</static>',
        f'      <pose>{float(center[0]):.3f} {float(center[1]):.3f} {z:.3f} 0 0 0</pose>',
        '      <link name="link">',
        '        <collision name="collision">',
        f'          <geometry><box><size>{float(size[0]):.3f} {float(size[1]):.3f} {height:.3f}</size></box></geometry>',
        '        </collision>',
        '        <visual name="visual">',
        f'          <geometry><box><size>{float(size[0]):.3f} {float(size[1]):.3f} {height:.3f}</size></box></geometry>',
        f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>',
        '        </visual>',
        '      </link>',
        '    </model>',
    ])


def build_world(scene, world_name):
    ego = scene.get('egoPlanner') or {}
    default_height = float(ego.get('obstacleHeight', 3.0))
    static_color = rgba(scene.get('obstacleColorRGBA'), DEFAULT_OBSTACLE_COLOR)
    moving_color = rgba(scene.get('movingObstacleColorRGBA'), DEFAULT_OBSTACLE_COLOR)

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

    for idx, (_key, obstacle) in enumerate(sorted_items(scene.get('obstacleData'))):
        center = obstacle.get('centerENU', [0.0, 0.0])
        add_cylinder_model(
            lines,
            obstacle.get('modelName', f'{world_name}_obstacle_{idx}'),
            center[0],
            center[1],
            obstacle.get('radius', 0.5),
            obstacle.get('height', default_height),
            obstacle.get('zMin', 0.0),
            True,
            rgba(obstacle.get('colorRGBA'), static_color),
        )

    for idx, (_key, wall) in enumerate(sorted_items(scene.get('wallData'))):
        add_wall_model(
            lines,
            wall.get('modelName', f'{world_name}_wall_{idx}'),
            wall.get('centerENU', [0.0, 0.0]),
            wall.get('sizeENU', [1.0, 0.2, default_height]),
            wall.get('yawRad', 0.0),
            wall.get('zMin', 0.0),
            rgba(wall.get('colorRGBA'), static_color),
        )

    platform = scene.get('platformData') or {}
    if platform:
        add_platform_model(lines, platform, scene)

    for idx, (_key, obstacle) in enumerate(sorted_items(scene.get('movingObstacleData'))):
        center = moving_obstacle_pose(obstacle, 0.0)
        add_cylinder_model(
            lines,
            obstacle.get('modelName', f'complex_moving_obstacle_{idx}'),
            center[0],
            center[1],
            obstacle.get('radius', 0.4),
            obstacle.get('height', default_height),
            obstacle.get('zMin', 0.0),
            False,
            rgba(obstacle.get('colorRGBA'), moving_color),
        )

    lines.extend(['  </world>', '</sdf>', ''])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='Generate a Gazebo world from a multi_uav_formation scene JSON.')
    parser.add_argument('--scene-file', required=True)
    parser.add_argument('--world-out', required=True)
    parser.add_argument('--world-name', default='')
    args = parser.parse_args()

    with open(args.scene_file, 'r') as f:
        scene = json.load(f)

    world_name = args.world_name or os.path.splitext(os.path.basename(args.world_out))[0]
    os.makedirs(os.path.dirname(os.path.abspath(args.world_out)), exist_ok=True)
    with open(args.world_out, 'w') as f:
        f.write(build_world(scene, world_name))

    print(f'Generated world: {args.world_out}')


if __name__ == '__main__':
    main()
