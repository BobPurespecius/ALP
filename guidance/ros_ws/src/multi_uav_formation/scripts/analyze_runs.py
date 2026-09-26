#!/usr/bin/env python3

import argparse
import csv
import glob
import json
import math
import os
import pickle
from pathlib import Path

import numpy as np


# Maximum horizontal extent of the Iris collision geometry: rotor arm offset
# plus rotor collision radius in px4_models/neverlost_livox_pitch30/iris.sdf.
DEFAULT_VEHICLE_COLLISION_RADIUS_M = 0.384

PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM = 0.2


def load_pickle(path):
    with open(path, 'rb') as f:
        return pickle.load(f)


def as_array(entries, key):
    values = []
    for entry in entries:
        value = entry.get(key)
        if value is None:
            values.append([np.nan, np.nan, np.nan])
        else:
            values.append(np.asarray(value, dtype=float))
    return np.asarray(values, dtype=float)


def choose_position_array(entries):
    feedback_positions = as_array(entries, 'mePositionENU')
    mavros_positions = as_array(entries, 'mavrosPositionENU')
    if not (
        len(mavros_positions) == len(feedback_positions) and
        len(mavros_positions) > 0 and
        np.any(np.isfinite(mavros_positions))
    ):
        mavros_positions = feedback_positions
    gazebo_positions = as_array(entries, 'gazeboPositionENU')
    if (
        len(gazebo_positions) == len(feedback_positions) and
        len(gazebo_positions) > 0 and
        np.all(np.isfinite(gazebo_positions[-1]))
    ):
        return gazebo_positions, 'gazeboPositionENU', mavros_positions, gazebo_positions
    return feedback_positions, 'mePositionENU', mavros_positions, gazebo_positions


def time_array(entries):
    values = []
    for idx, entry in enumerate(entries):
        value = entry.get('t')
        try:
            values.append(float(value))
        except (TypeError, ValueError):
            values.append(float(idx))
    return np.asarray(values, dtype=float)


def finite_difference(values, time):
    if len(values) < 2:
        return np.zeros_like(values)
    dt = np.gradient(time)
    dt[np.abs(dt) < 1e-6] = np.nan
    grad = np.gradient(values, axis=0)
    return grad / dt[:, None]


def norm_rows(values):
    if len(values) == 0:
        return np.asarray([], dtype=float)
    return np.linalg.norm(values, axis=1)


def path_length(positions):
    if len(positions) < 2:
        return 0.0
    return float(np.nansum(norm_rows(np.diff(positions, axis=0))))


def turn_angle_stats(positions):
    if len(positions) < 3:
        return 0.0, 0.0
    steps = np.diff(positions[:, :2], axis=0)
    lengths = norm_rows(steps)
    valid = lengths > 1e-6
    steps = steps[valid]
    lengths = lengths[valid]
    if len(steps) < 2:
        return 0.0, 0.0

    a = steps[:-1]
    b = steps[1:]
    denom = norm_rows(a) * norm_rows(b)
    denom[denom < 1e-9] = np.nan
    cosang = np.einsum('ij,ij->i', a, b) / denom
    cosang = np.clip(cosang, -1.0, 1.0)
    angles = np.arccos(cosang)
    return float(np.nanmean(np.abs(angles))), float(np.nanmax(np.abs(angles)))


def sorted_items(data):
    def key(item):
        name, _value = item
        try:
            return int(name)
        except ValueError:
            return name

    return sorted(data.items(), key=key)


def moving_obstacle_center(obstacle, t):
    center = obstacle.get('centerENU', [0.0, 0.0])
    axis = obstacle.get('axisENU', [1.0, 0.0])
    amplitude = float(obstacle.get('amplitude', 0.0))
    period = max(float(obstacle.get('period', 1.0)), 1e-6)
    phase = float(obstacle.get('phase', 0.0))

    if len(axis) < 2:
        unit_axis = np.asarray([1.0, 0.0], dtype=float)
    else:
        unit_axis = np.asarray([axis[0], axis[1]], dtype=float)
        axis_norm = np.linalg.norm(unit_axis)
        if axis_norm < 1e-6:
            unit_axis = np.asarray([1.0, 0.0], dtype=float)
        else:
            unit_axis = unit_axis / axis_norm

    offset = amplitude * math.sin(2.0 * math.pi * t / period + phase)
    return [float(center[0]) + float(unit_axis[0]) * offset,
            float(center[1]) + float(unit_axis[1]) * offset]


def configured_moving_obstacles(config, t):
    states = []
    for name, obstacle in sorted_items(config.get('movingObstacleData', {})):
        state = dict(obstacle)
        state['name'] = name
        state['type'] = 'moving'
        state['centerENU'] = moving_obstacle_center(obstacle, t)
        states.append(state)
    return states


def vehicle_collision_radius(config):
    value = config.get('vehicleCollisionRadius')
    if value is None:
        value = (config.get('egoPlanner') or {}).get(
            'vehicleCollisionRadius', DEFAULT_VEHICLE_COLLISION_RADIUS_M)
    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        return DEFAULT_VEHICLE_COLLISION_RADIUS_M


def obstacle_clearance(positions, obstacle_data, vehicle_radius=0.0,
                       min_height=None, default_height=3.0):
    if not obstacle_data:
        return math.inf
    if min_height is not None and len(positions):
        positions = positions[positions[:, 2] >= min_height]
    if len(positions) == 0:
        return math.inf
    min_clearance = math.inf
    for obstacle in obstacle_data.values():
        center = np.asarray(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)
        radius = float(obstacle.get('radius', 0.0))
        z_min = float(obstacle.get('zMin', 0.0))
        z_max = z_min + float(obstacle.get('height', default_height))
        height_mask = (positions[:, 2] >= z_min) & (positions[:, 2] <= z_max)
        obstacle_positions = positions[height_mask]
        distances = norm_rows(obstacle_positions[:, :2] - center)
        if len(distances):
            surface_clearance = distances - radius - vehicle_radius
            min_clearance = min(min_clearance, float(np.nanmin(surface_clearance)))
    return min_clearance


def moving_obstacle_clearance(entries, positions, config, time, vehicle_radius=0.0,
                              min_height=None):
    if not entries or len(positions) == 0:
        return math.inf

    min_clearance = math.inf
    default_height = float((config.get('egoPlanner') or {}).get('obstacleHeight', 3.0))
    for idx, (entry, position) in enumerate(zip(entries, positions)):
        if min_height is not None and position[2] < min_height:
            continue

        obstacles = entry.get('movingObstacleStates')
        if not obstacles:
            t = float(time[idx]) if idx < len(time) and math.isfinite(float(time[idx])) else float(idx)
            obstacles = configured_moving_obstacles(config, t)

        for obstacle in obstacles:
            z_min = float(obstacle.get('zMin', 0.0))
            z_max = z_min + float(obstacle.get('height', default_height))
            if position[2] < z_min or position[2] > z_max:
                continue
            center = np.asarray(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)
            radius = float(obstacle.get('radius', 0.0))
            clearance = float(
                np.linalg.norm(position[:2] - center) - radius - vehicle_radius)
            min_clearance = min(min_clearance, clearance)

    return min_clearance


def combined_obstacle_clearance(entries, positions, config, time, min_height=None):
    default_height = float((config.get('egoPlanner') or {}).get('obstacleHeight', 3.0))
    vehicle_radius = vehicle_collision_radius(config)
    static_clearance = obstacle_clearance(
        positions,
        config.get('obstacleData', {}),
        vehicle_radius=vehicle_radius,
        min_height=min_height,
        default_height=default_height
    )
    dynamic_clearance = moving_obstacle_clearance(
        entries,
        positions,
        config,
        time,
        vehicle_radius=vehicle_radius,
        min_height=min_height
    )
    return min(static_clearance, dynamic_clearance)


def platform_top_z(platform):
    if platform.get('topZ') is not None:
        return float(platform['topZ'])
    size = platform.get('sizeENU')
    if size is not None and len(size) >= 3:
        return float(size[2])
    return None


def platform_landing_reference_z(config, platform, uav_number):
    for key in ('landingTargetZ', 'targetZ'):
        if platform.get(key) is not None:
            return float(platform[key])

    top_z = platform_top_z(platform)
    if top_z is None:
        return None

    reference_height = platform.get('vehicleReferenceHeight')
    if reference_height is None:
        takeoff = config.get('takeoffPointENU', [])
        if len(takeoff) >= uav_number and len(takeoff[uav_number - 1]) >= 3:
            reference_height = takeoff[uav_number - 1][2]
    return top_z + float(reference_height or 0.0)


def platform_hover_reference_z(config, platform, uav_number):
    top_z = platform_top_z(platform)
    if top_z is None:
        return None

    hover_above_platform = float(platform.get(
        'hoverAbovePlatform',
        PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM
    ))
    return top_z + hover_above_platform


def nearest_goal(config, uav_number):
    ego = config.get('egoPlanner', {})
    goals = ego.get('goalsENU', [])
    if len(goals) >= uav_number:
        goal = list(goals[uav_number - 1])
        platform = config.get('platformData', {})
        landing_z = platform_landing_reference_z(config, platform, uav_number)
        hover_z = platform_hover_reference_z(config, platform, uav_number)

        if len(goal) >= 2 and platform.get('landOnTop', False) and landing_z is not None:
            goal = [goal[0], goal[1], landing_z]
        elif len(goal) == 2 and hover_z is not None:
            goal = [goal[0], goal[1], hover_z]
        elif len(goal) == 2:
            if hover_z is not None:
                goal.append(hover_z)
            elif landing_z is not None:
                goal.append(landing_z)
        return entry_array(goal)
    prepare = config.get('preparePointENU', [])
    if len(prepare) >= uav_number:
        return np.asarray(prepare[uav_number - 1], dtype=float)
    return None


def last_entry_value(entries, key):
    for entry in reversed(entries):
        value = entry.get(key)
        if value is None:
            continue
        if isinstance(value, str) and value == '':
            continue
        else:
            return value
    return ''


def numeric_entry_values(entries, key):
    values = []
    for entry in entries:
        try:
            value = float(entry.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def entry_array(value):
    if value is None:
        return None
    if isinstance(value, str) and value == '':
        return None
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.ndim == 0 or array.shape[0] < 3 or not np.all(np.isfinite(array[:3])):
        return None
    return array[:3]


def summarize_vehicle(run_data, goal_tolerance, clearance_margin,
                      goal_xy_tolerance=None, goal_z_tolerance=None, goal_speed_tolerance=None):
    entries = run_data.get('data', [])
    config = run_data.get('config') or run_data.get('sceneConfig') or {}
    params = run_data.get('params', {})
    uav_number = int(run_data.get('selfnumber', 0))
    clearance_min_height = params.get('safety', {}).get('min_height')
    if clearance_min_height is not None:
        clearance_min_height = float(clearance_min_height)

    if not entries:
        return {
            'uav': uav_number,
            'samples': 0,
            'success': False,
            'failure_reason': 'no_data',
        }

    time = time_array(entries)
    positions, position_source, mavros_positions, gazebo_positions = choose_position_array(entries)
    velocities = as_array(entries, 'meVelocity')
    logged_acc = as_array(entries, 'meAccelerationENU')
    controls = as_array(entries, 'u')

    goal = nearest_goal(config, uav_number)
    final_position = positions[-1]
    final_distance = math.inf
    final_xy_error = math.inf
    final_z_error = math.inf
    min_goal_distance = math.inf
    reached_goal = False
    reached_goal_split = False
    if goal is not None:
        position_errors = positions - goal
        goal_distances = norm_rows(position_errors)
        xy_distances = norm_rows(position_errors[:, :2])
        z_distances = np.abs(position_errors[:, 2])
        final_distance = float(goal_distances[-1])
        final_xy_error = float(xy_distances[-1])
        final_z_error = float(z_distances[-1])
        min_goal_distance = float(np.nanmin(goal_distances))
        reached_goal = min_goal_distance <= goal_tolerance
        if goal_xy_tolerance is not None and goal_z_tolerance is not None:
            reached_goal_split = bool(np.nanmin((xy_distances <= goal_xy_tolerance) & (z_distances <= goal_z_tolerance)))

    velocity_acc = finite_difference(velocities, time)
    jerk_from_vel = finite_difference(velocity_acc, time)
    jerk_from_logged_acc = finite_difference(logged_acc, time)
    control_delta = np.diff(controls, axis=0) if len(controls) > 1 else np.zeros((0, 3))
    mean_turn_angle, max_turn_angle = turn_angle_stats(positions)
    min_clearance = combined_obstacle_clearance(
        entries,
        positions,
        config,
        time,
        min_height=clearance_min_height
    )

    finite_time = time[np.isfinite(time)]
    duration = float(finite_time[-1] - finite_time[0]) if len(finite_time) >= 2 else 0.0
    speed = norm_rows(velocities)
    final_speed = float(speed[-1]) if len(speed) else math.inf
    acceleration = norm_rows(velocity_acc)
    jerk = norm_rows(jerk_from_vel)
    logged_jerk = norm_rows(jerk_from_logged_acc)
    final_entry = entries[-1]
    platform_data = config.get('platformData', {})
    platform_requires_landing = bool(platform_data.get('landOnTop', False))
    platform_final_hover = bool(platform_data) and not platform_requires_landing
    final_armed = final_entry.get('armed')
    if isinstance(final_armed, str):
        final_armed = final_armed.lower() in ('true', '1', 'yes')
    landing_state_success = True
    if platform_requires_landing:
        landing_state_success = (
            final_entry.get('state', '') == 'END' and
            final_armed is False
        )

    if goal_xy_tolerance is not None and goal_z_tolerance is not None:
        final_success = (
            goal is not None and
            final_xy_error <= goal_xy_tolerance and
            final_z_error <= goal_z_tolerance and
            (
                platform_final_hover or
                goal_speed_tolerance is None or
                final_speed <= goal_speed_tolerance
            ) and
            landing_state_success
        )
        reached_goal = reached_goal_split
    else:
        final_success = (
            goal is not None and
            final_distance <= goal_tolerance and
            landing_state_success
        )
    # Clearance remains a diagnostic metric, but run success is currently
    # defined only by whether the vehicle finishes at its target.
    success = final_success

    failure_reasons = []
    if goal is None:
        failure_reasons.append('missing_goal')
    elif not final_success:
        if goal_xy_tolerance is not None and final_xy_error > goal_xy_tolerance:
            failure_reasons.append('final_xy_error')
        if goal_z_tolerance is not None and final_z_error > goal_z_tolerance:
            failure_reasons.append('final_z_error')
        if (
            not platform_final_hover and
            goal_speed_tolerance is not None and
            final_speed > goal_speed_tolerance
        ):
            failure_reasons.append('final_speed_error')
        if goal_xy_tolerance is None and goal_z_tolerance is None:
            failure_reasons.append('final_goal_error')
    if platform_requires_landing and not landing_state_success:
        if final_entry.get('state', '') != 'END':
            failure_reasons.append('final_state_not_end')
        if final_armed is not False:
            failure_reasons.append('final_armed')
    if not failure_reasons:
        failure_reasons.append('ok')

    last_ego_cmd_pos = entry_array(last_entry_value(entries, 'egoLastCommandPositionENU'))
    last_ego_cmd_vel = entry_array(last_entry_value(entries, 'egoLastCommandVelocityENU'))
    last_ego_cmd_goal_error = ''
    if goal is not None and last_ego_cmd_pos is not None:
        last_ego_cmd_goal_error = float(np.linalg.norm(last_ego_cmd_pos - goal))

    command_ages = numeric_entry_values(entries, 'egoCommandAge')
    final_ego_command_age = float(command_ages[-1]) if command_ages else ''
    max_ego_command_age = float(max(command_ages)) if command_ages else ''

    safety_violation_samples = sum(1 for entry in entries if entry.get('safetyViolation'))
    safety_warning_samples = sum(1 for entry in entries if entry.get('safetyWarning'))
    final_gazebo_mavros_error = ''
    final_gazebo_mavros_dx = ''
    final_gazebo_mavros_dy = ''
    final_gazebo_mavros_dz = ''
    if (
        len(gazebo_positions) > 0 and
        len(mavros_positions) > 0 and
        np.all(np.isfinite(gazebo_positions[-1])) and
        np.all(np.isfinite(mavros_positions[-1]))
    ):
        final_delta = gazebo_positions[-1] - mavros_positions[-1]
        final_gazebo_mavros_error = float(np.linalg.norm(final_delta))
        final_gazebo_mavros_dx = float(final_delta[0])
        final_gazebo_mavros_dy = float(final_delta[1])
        final_gazebo_mavros_dz = float(final_delta[2])

    return {
        'uav': uav_number,
        'samples': len(entries),
        'position_source': position_source,
        'duration_s': duration,
        'final_state': final_entry.get('state', ''),
        'final_state_time_s': float(final_entry.get('stateTime', 0.0) or 0.0),
        'final_task_time_s': float(final_entry.get('taskTime', 0.0) or 0.0),
        'final_armed': final_armed if final_armed is not None else '',
        'success': success,
        'reached_goal_once': reached_goal,
        'ego_goal_reached_flag': bool(final_entry.get('egoGoalReached', False)),
        'failure_reason': '+'.join(failure_reasons),
        'final_x': float(final_position[0]),
        'final_y': float(final_position[1]),
        'final_z': float(final_position[2]),
        'goal_x': float(goal[0]) if goal is not None else '',
        'goal_y': float(goal[1]) if goal is not None else '',
        'goal_z': float(goal[2]) if goal is not None else '',
        'final_goal_error_m': final_distance,
        'final_xy_error_m': final_xy_error,
        'final_z_error_m': final_z_error,
        'final_speed_mps': final_speed,
        'min_goal_error_m': min_goal_distance,
        'last_ego_cmd_x': float(last_ego_cmd_pos[0]) if last_ego_cmd_pos is not None else '',
        'last_ego_cmd_y': float(last_ego_cmd_pos[1]) if last_ego_cmd_pos is not None else '',
        'last_ego_cmd_z': float(last_ego_cmd_pos[2]) if last_ego_cmd_pos is not None else '',
        'last_ego_cmd_goal_error_m': last_ego_cmd_goal_error,
        'last_ego_cmd_speed_mps': float(np.linalg.norm(last_ego_cmd_vel)) if last_ego_cmd_vel is not None else '',
        'final_ego_command_age_s': final_ego_command_age,
        'max_ego_command_age_s': max_ego_command_age,
        'final_safety_violation': final_entry.get('safetyViolation', ''),
        'final_safety_warning': final_entry.get('safetyWarning', ''),
        'safety_violation_samples': safety_violation_samples,
        'safety_warning_samples': safety_warning_samples,
        'final_gazebo_mavros_error_m': final_gazebo_mavros_error,
        'final_gazebo_mavros_dx_m': final_gazebo_mavros_dx,
        'final_gazebo_mavros_dy_m': final_gazebo_mavros_dy,
        'final_gazebo_mavros_dz_m': final_gazebo_mavros_dz,
        'path_length_m': path_length(positions),
        'min_obstacle_clearance_m': min_clearance,
        'mean_speed_mps': float(np.nanmean(speed)) if len(speed) else 0.0,
        'max_speed_mps': float(np.nanmax(speed)) if len(speed) else 0.0,
        'rms_accel_mps2': float(np.sqrt(np.nanmean(acceleration ** 2))) if len(acceleration) else 0.0,
        'max_accel_mps2': float(np.nanmax(acceleration)) if len(acceleration) else 0.0,
        'rms_jerk_mps3': float(np.sqrt(np.nanmean(jerk ** 2))) if len(jerk) else 0.0,
        'max_jerk_mps3': float(np.nanmax(jerk)) if len(jerk) else 0.0,
        'rms_logged_jerk_mps3': float(np.sqrt(np.nanmean(logged_jerk ** 2))) if len(logged_jerk) else 0.0,
        'mean_turn_angle_rad': mean_turn_angle,
        'max_turn_angle_rad': max_turn_angle,
        'mean_control_delta': float(np.nanmean(norm_rows(control_delta))) if len(control_delta) else 0.0,
        'max_control_delta': float(np.nanmax(norm_rows(control_delta))) if len(control_delta) else 0.0,
    }


def summarize_run(run_dir, goal_tolerance, clearance_margin, require_complete=True,
                  goal_xy_tolerance=None, goal_z_tolerance=None, goal_speed_tolerance=None):
    files = sorted(glob.glob(os.path.join(run_dir, 'data_*.pkl')))
    vehicles = []
    config = None
    for path in files:
        try:
            data = load_pickle(path)
            config = data.get('config') or data.get('sceneConfig') or config
            vehicles.append(summarize_vehicle(
                data,
                goal_tolerance,
                clearance_margin,
                goal_xy_tolerance=goal_xy_tolerance,
                goal_z_tolerance=goal_z_tolerance,
                goal_speed_tolerance=goal_speed_tolerance,
            ))
        except Exception as exc:
            vehicles.append({
                'uav': os.path.basename(path),
                'samples': 0,
                'success': False,
                'failure_reason': f'load_error:{exc}',
            })

    expected = int((config or {}).get('number', len(vehicles)))
    complete = len(files) == expected
    vehicle_successes = [bool(v.get('success')) for v in vehicles]
    run_success = bool(vehicle_successes) and all(vehicle_successes)
    if require_complete:
        run_success = run_success and complete

    numeric_keys = [
        'duration_s',
        'final_goal_error_m',
        'final_xy_error_m',
        'final_z_error_m',
        'final_speed_mps',
        'final_gazebo_mavros_error_m',
        'final_gazebo_mavros_dx_m',
        'final_gazebo_mavros_dy_m',
        'final_gazebo_mavros_dz_m',
        'path_length_m',
        'min_obstacle_clearance_m',
        'mean_speed_mps',
        'max_speed_mps',
        'rms_accel_mps2',
        'max_accel_mps2',
        'rms_jerk_mps3',
        'max_jerk_mps3',
        'rms_logged_jerk_mps3',
        'mean_turn_angle_rad',
        'max_turn_angle_rad',
        'mean_control_delta',
        'max_control_delta',
    ]
    aggregate = {}
    for key in numeric_keys:
        values = [float(v[key]) for v in vehicles if key in v and v[key] != '' and math.isfinite(float(v[key]))]
        if values:
            aggregate[f'mean_{key}'] = float(np.mean(values))
            aggregate[f'max_{key}'] = float(np.max(values))
            aggregate[f'min_{key}'] = float(np.min(values))

    reasons = sorted(set(str(v.get('failure_reason', 'unknown')) for v in vehicles))
    return {
        'run_dir': run_dir,
        'expected_uavs': expected,
        'data_files': len(files),
        'complete': complete,
        'success': run_success,
        'vehicle_success_rate': float(np.mean(vehicle_successes)) if vehicle_successes else 0.0,
        'failure_reasons': ';'.join(reasons),
        **aggregate,
    }, vehicles


def find_run_dirs(root):
    dirs = []
    for path in sorted(glob.glob(os.path.join(root, '*'))):
        if os.path.isdir(path) and glob.glob(os.path.join(path, 'data_*.pkl')):
            dirs.append(path)
    return dirs


def write_csv(path, rows):
    if not rows:
        return
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description='Analyze MUSK SingleRun batches for success rate and trajectory smoothness.')
    parser.add_argument('--root', default='/app/guidance/ros_ws/data/multi_uav_formation/SingleRun',
                        help='Directory containing timestamped SingleRun folders.')
    parser.add_argument('--run-dir', action='append', default=[],
                        help='Analyze one specific run directory. Can be passed multiple times.')
    parser.add_argument('--out-dir', default='/app/guidance/ros_ws/analysis',
                        help='Directory for CSV/JSON analysis outputs.')
    parser.add_argument('--goal-tolerance', type=float, default=None,
                        help='Goal success tolerance. Defaults to egoPlanner.goalTolerance or 0.45.')
    parser.add_argument('--goal-xy-tolerance', type=float, default=None,
                        help='Final XY success tolerance. Defaults to platform landingXYTolerance, then egoPlanner.goalXYTolerance.')
    parser.add_argument('--goal-z-tolerance', type=float, default=None,
                        help='Final height success tolerance. Defaults to platform landingHeightTolerance, then egoPlanner.goalZTolerance.')
    parser.add_argument('--goal-speed-tolerance', type=float, default=None,
                        help='Final speed success tolerance. Defaults to platform landingSpeedTolerance, then egoPlanner.goalSpeedTolerance.')
    parser.add_argument('--clearance-margin', type=float, default=0.0,
                        help='Deprecated compatibility option; clearance does not affect success.')
    parser.add_argument('--allow-incomplete', action='store_true',
                        help='Do not mark a run failed only because some data_*.pkl files are missing.')
    args = parser.parse_args()

    run_dirs = args.run_dir or find_run_dirs(args.root)
    os.makedirs(args.out_dir, exist_ok=True)

    run_rows = []
    vehicle_rows = []
    for run_dir in run_dirs:
        files = sorted(glob.glob(os.path.join(run_dir, 'data_*.pkl')))
        goal_tolerance = args.goal_tolerance
        if goal_tolerance is None and files:
            try:
                config = load_pickle(files[0]).get('config', {})
                goal_tolerance = float(config.get('egoPlanner', {}).get('goalTolerance', 0.45))
            except Exception:
                goal_tolerance = 0.45
        if goal_tolerance is None:
            goal_tolerance = 0.45
        goal_xy_tolerance = args.goal_xy_tolerance
        goal_z_tolerance = args.goal_z_tolerance
        goal_speed_tolerance = args.goal_speed_tolerance
        if (goal_xy_tolerance is None or goal_z_tolerance is None or goal_speed_tolerance is None) and files:
            try:
                config = load_pickle(files[0]).get('config', {})
                ego = config.get('egoPlanner', {})
                platform = config.get('platformData', {})
                if platform.get('landOnTop', False):
                    if goal_xy_tolerance is None and 'landingXYTolerance' in platform:
                        goal_xy_tolerance = float(platform['landingXYTolerance'])
                    if goal_z_tolerance is None and 'landingHeightTolerance' in platform:
                        goal_z_tolerance = float(platform['landingHeightTolerance'])
                    if goal_speed_tolerance is None and 'landingSpeedTolerance' in platform:
                        goal_speed_tolerance = float(platform['landingSpeedTolerance'])
                if goal_xy_tolerance is None and 'goalXYTolerance' in ego:
                    goal_xy_tolerance = float(ego['goalXYTolerance'])
                if goal_z_tolerance is None and 'goalZTolerance' in ego:
                    goal_z_tolerance = float(ego['goalZTolerance'])
                if goal_speed_tolerance is None and 'goalSpeedTolerance' in ego:
                    goal_speed_tolerance = float(ego['goalSpeedTolerance'])
            except Exception:
                pass

        run_summary, vehicle_summaries = summarize_run(
            run_dir,
            goal_tolerance,
            args.clearance_margin,
            require_complete=not args.allow_incomplete,
            goal_xy_tolerance=goal_xy_tolerance,
            goal_z_tolerance=goal_z_tolerance,
            goal_speed_tolerance=goal_speed_tolerance,
        )
        run_rows.append(run_summary)
        for vehicle in vehicle_summaries:
            vehicle_rows.append({'run_dir': run_dir, **vehicle})

    total_runs = len(run_rows)
    successful_runs = sum(1 for row in run_rows if row.get('success'))
    overall = {
        'total_runs': total_runs,
        'successful_runs': successful_runs,
        'run_success_rate': successful_runs / total_runs if total_runs else 0.0,
        'mean_vehicle_success_rate': float(np.mean([row['vehicle_success_rate'] for row in run_rows])) if run_rows else 0.0,
    }

    write_csv(os.path.join(args.out_dir, 'run_summary.csv'), run_rows)
    write_csv(os.path.join(args.out_dir, 'vehicle_summary.csv'), vehicle_rows)
    with open(os.path.join(args.out_dir, 'overall_summary.json'), 'w') as f:
        json.dump(overall, f, indent=2)

    print(json.dumps(overall, indent=2))
    print(f'Run summary: {os.path.join(args.out_dir, "run_summary.csv")}')
    print(f'Vehicle summary: {os.path.join(args.out_dir, "vehicle_summary.csv")}')


if __name__ == '__main__':
    main()
