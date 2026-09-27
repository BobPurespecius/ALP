#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import re


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def load_scene_obstacles(scene_file):
    if not scene_file or not os.path.exists(scene_file):
        return []
    with open(scene_file) as f:
        scene = json.load(f)
    obstacles = []
    for obstacle in (scene.get('obstacleData') or {}).values():
        center = obstacle.get('centerENU', [0.0, 0.0])
        obstacles.append({
            'shape': 'cylinder',
            'center': [float(center[0]), float(center[1])],
            'radius': float(obstacle.get('radius', 0.5)),
            'height': float(obstacle.get('height', (scene.get('egoPlanner') or {}).get('obstacleHeight', 3.0))),
            'z_min': float(obstacle.get('zMin', 0.0)),
        })
    for obstacle in (scene.get('boxObstacleData') or {}).values():
        center = obstacle.get('centerENU', [0.0, 0.0])
        size = obstacle.get('sizeENU', [1.0, 1.0, 1.0])
        obstacles.append({
            'shape': 'box',
            'center': [float(center[0]), float(center[1])],
            'size': [float(size[0]), float(size[1]), float(size[2]) if len(size) >= 3 else 1.0],
            'z_min': float(obstacle.get('zMin', 0.0)),
            'yaw': float(obstacle.get('yaw', 0.0)),
        })
    for obstacle in (scene.get('movingObstacleData') or {}).values():
        center = obstacle.get('centerENU', [0.0, 0.0])
        axis = obstacle.get('axisENU', [1.0, 0.0])
        norm = math.hypot(float(axis[0]), float(axis[1])) if len(axis) >= 2 else 0.0
        axis_xy = [1.0, 0.0] if norm < 1e-6 else [float(axis[0]) / norm, float(axis[1]) / norm]
        obstacles.append({
            'shape': 'cylinder',
            'center': [float(center[0]), float(center[1])],
            'radius': float(obstacle.get('radius', 0.5)),
            'height': float(obstacle.get('height', (scene.get('egoPlanner') or {}).get('obstacleHeight', 3.0))),
            'z_min': float(obstacle.get('zMin', 0.0)),
            'axis': axis_xy,
            'amplitude': float(obstacle.get('amplitude', 0.0)),
            'period': max(float(obstacle.get('period', 1.0)), 1e-6),
            'phase': float(obstacle.get('phase', 0.0)),
        })
    return obstacles


def obstacle_center(obstacle, stamp):
    center = obstacle['center']
    if 'axis' not in obstacle:
        return center
    offset = obstacle['amplitude'] * math.sin(2.0 * math.pi * stamp / obstacle['period'] + obstacle['phase'])
    return [center[0] + obstacle['axis'][0] * offset, center[1] + obstacle['axis'][1] * offset]


def segment_point_distance(ax, ay, bx, by, px, py):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    denom = vx * vx + vy * vy
    if denom < 1e-9:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    cx = ax + ratio * vx
    cy = ay + ratio * vy
    return math.hypot(px - cx, py - cy)


def segment_cylinder_intersects(start, end, center, radius, z_min, z_max):
    sx = start[0] - center[0]
    sy = start[1] - center[1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    radial_a = dx * dx + dy * dy
    radial_b = 2.0 * (sx * dx + sy * dy)
    radial_c = sx * sx + sy * sy - radius * radius

    if radial_a < 1e-9:
        if radial_c > 0.0:
            return False
        radial_t0, radial_t1 = 0.0, 1.0
    else:
        disc = radial_b * radial_b - 4.0 * radial_a * radial_c
        if disc < 0.0:
            return False
        root = math.sqrt(max(disc, 0.0))
        radial_t0 = (-radial_b - root) / (2.0 * radial_a)
        radial_t1 = (-radial_b + root) / (2.0 * radial_a)
        radial_t0, radial_t1 = max(0.0, min(radial_t0, radial_t1)), min(1.0, max(radial_t0, radial_t1))
        if radial_t0 > radial_t1:
            return False

    if abs(dz) < 1e-9:
        if start[2] < z_min or start[2] > z_max:
            return False
        z_t0, z_t1 = 0.0, 1.0
    else:
        tz0 = (z_min - start[2]) / dz
        tz1 = (z_max - start[2]) / dz
        z_t0 = max(0.0, min(tz0, tz1))
        z_t1 = min(1.0, max(tz0, tz1))
        if z_t0 > z_t1:
            return False

    return max(radial_t0, z_t0) <= min(radial_t1, z_t1)


def segment_box_intersects(start, end, center, size, z_min, yaw, margin):
    sz = size[2] if len(size) >= 3 else 1.0
    box_center = [center[0], center[1], z_min + sz * 0.5]
    c = math.cos(-yaw)
    s = math.sin(-yaw)

    def to_local(point):
        x = point[0] - box_center[0]
        y = point[1] - box_center[1]
        z = point[2] - box_center[2]
        return [c * x - s * y, s * x + c * y, z]

    p0 = to_local(start)
    p1 = to_local(end)
    direction = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    half = [size[0] * 0.5 + margin, size[1] * 0.5 + margin, sz * 0.5 + margin]
    t_min, t_max = 0.0, 1.0
    for axis in range(3):
        if abs(direction[axis]) < 1e-9:
            if p0[axis] < -half[axis] or p0[axis] > half[axis]:
                return False
            continue
        t0 = (-half[axis] - p0[axis]) / direction[axis]
        t1 = (half[axis] - p0[axis]) / direction[axis]
        t_min = max(t_min, min(t0, t1))
        t_max = min(t_max, max(t0, t1))
        if t_min > t_max:
            return False
    return True


def row_visible(row, obstacles, fov_half_angle_rad, max_distance, occlusion_margin):
    ux = finite_float(row.get('uav_x'))
    uy = finite_float(row.get('uav_y'))
    uz = finite_float(row.get('uav_z'))
    tx = finite_float(row.get('true_target_x'))
    ty = finite_float(row.get('true_target_y'))
    tz = finite_float(row.get('true_target_z'))
    yaw = finite_float(row.get('feedback_yaw_rad'))
    if yaw is None:
        yaw = finite_float(row.get('yaw_rad'))
    stamp = finite_float(row.get('ros_time')) or 0.0
    if None in (ux, uy, uz, tx, ty, tz, yaw):
        logged_visible = row.get('target_visible')
        if logged_visible not in (None, ''):
            return str(logged_visible).strip().lower() in ('1', 'true', 'yes')
        return False
    distance = math.hypot(tx - ux, ty - uy)
    if distance < 0.2 or distance > max_distance:
        return False
    bearing = math.atan2(ty - uy, tx - ux)
    angle_error = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
    if abs(angle_error) > fov_half_angle_rad:
        return False
    start = [ux, uy, uz]
    end = [tx, ty, tz]
    for obstacle in obstacles:
        ox, oy = obstacle_center(obstacle, stamp)
        if obstacle.get('shape') == 'box':
            if segment_box_intersects(
                start,
                end,
                [ox, oy],
                obstacle.get('size', [1.0, 1.0, 1.0]),
                obstacle.get('z_min', 0.0),
                obstacle.get('yaw', 0.0),
                occlusion_margin,
            ):
                return False
        elif segment_cylinder_intersects(
            start,
            end,
            [ox, oy],
            obstacle['radius'] + occlusion_margin,
            obstacle.get('z_min', 0.0),
            obstacle.get('z_min', 0.0) + obstacle.get('height', 3.0),
        ):
            return False
    return True


def mean(values):
    values = list(values)
    return sum(values) / len(values) if values else float('nan')


def rms(values):
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def derivatives(samples, value_key, time_key='ros_time'):
    result = []
    last_t = None
    last_v = None
    for row in samples:
        t = finite_float(row.get(time_key))
        v = finite_float(row.get(value_key))
        if t is None or v is None:
            continue
        if last_t is not None and t > last_t:
            result.append((v - last_v) / (t - last_t))
        last_t = t
        last_v = v
    return result


def norm_series(xs, ys, zs):
    return [
        math.sqrt(x * x + y * y + z * z)
        for x, y, z in zip(xs, ys, zs)
    ]


def analyze_file(path, success_error):
    rows = load_rows(path)
    track_rows = [row for row in rows if row.get('stage') == 'TRACK']
    if not track_rows:
        return {
            'file': path,
            'samples': len(rows),
            'track_samples': 0,
            'success': 0,
        }

    errors = [finite_float(row.get('tracking_error_m')) for row in track_rows]
    errors = [value for value in errors if value is not None]
    estimate_errors = []
    ego_planner_samples = 0
    for row in track_rows:
        if row.get('target_source') == 'ego_planner':
            ego_planner_samples += 1
        tx = finite_float(row.get('target_x'))
        ty = finite_float(row.get('target_y'))
        tz = finite_float(row.get('target_z'))
        rx = finite_float(row.get('true_target_x'))
        ry = finite_float(row.get('true_target_y'))
        rz = finite_float(row.get('true_target_z'))
        if None not in (tx, ty, tz, rx, ry, rz):
            estimate_errors.append(math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2 + (tz - rz) ** 2))

    track_times = [finite_float(row.get('track_time')) for row in track_rows]
    track_times = [value for value in track_times if value is not None]
    duration = max(track_times) - min(track_times) if len(track_times) >= 2 else 0.0

    vx = derivatives(track_rows, 'uav_x')
    vy = derivatives(track_rows, 'uav_y')
    vz = derivatives(track_rows, 'uav_z')
    speed = norm_series(vx, vy, vz)

    ax = derivatives([{'ros_time': r.get('ros_time'), 'v': v} for r, v in zip(track_rows[1:], vx)], 'v')
    ay = derivatives([{'ros_time': r.get('ros_time'), 'v': v} for r, v in zip(track_rows[1:], vy)], 'v')
    az = derivatives([{'ros_time': r.get('ros_time'), 'v': v} for r, v in zip(track_rows[1:], vz)], 'v')
    accel = norm_series(ax, ay, az)

    jx = derivatives([{'ros_time': r.get('ros_time'), 'a': a} for r, a in zip(track_rows[2:], ax)], 'a')
    jy = derivatives([{'ros_time': r.get('ros_time'), 'a': a} for r, a in zip(track_rows[2:], ay)], 'a')
    jz = derivatives([{'ros_time': r.get('ros_time'), 'a': a} for r, a in zip(track_rows[2:], az)], 'a')
    jerk = norm_series(jx, jy, jz)

    final_error = errors[-1] if errors else float('nan')
    max_error = max(errors) if errors else float('nan')
    mean_error = mean(errors)
    rms_error = rms(errors)
    success = int(bool(errors) and mean_error <= success_error and final_error <= success_error)

    return {
        'file': path,
        'samples': len(rows),
        'track_samples': len(track_rows),
        'ego_planner_sample_ratio': ego_planner_samples / len(track_rows) if track_rows else 0.0,
        'success': success,
        'duration_s': duration,
        'mean_target_estimation_error_m': mean(estimate_errors),
        'max_target_estimation_error_m': max(estimate_errors) if estimate_errors else float('nan'),
        'mean_tracking_error_m': mean_error,
        'rms_tracking_error_m': rms_error,
        'max_tracking_error_m': max_error,
        'final_tracking_error_m': final_error,
        'mean_speed_mps': mean(speed),
        'max_speed_mps': max(speed) if speed else float('nan'),
        'rms_accel_mps2': rms(accel),
        'max_accel_mps2': max(accel) if accel else float('nan'),
        'rms_jerk_mps3': rms(jerk),
        'max_jerk_mps3': max(jerk) if jerk else float('nan'),
    }


def write_csv(path, rows):
    if not rows:
        return
    keys = list(rows[0].keys())
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def team_key(path):
    name = os.path.basename(path)
    match = re.match(r'(target_tracking_run_\d+)(?:_uav\d+)?\.csv$', name)
    return match.group(1) if match else os.path.splitext(name)[0]


def nearest_row(rows, target_time, max_dt):
    best = None
    best_dt = None
    for row in rows:
        t = finite_float(row.get('ros_time'))
        if t is None:
            continue
        dt = abs(t - target_time)
        if best_dt is None or dt < best_dt:
            best = row
            best_dt = dt
    if best is None or best_dt is None or best_dt > max_dt:
        return None
    return best


def analyze_team_group(key, paths, obstacles, args):
    paths = sorted(paths)
    series = []
    for path in paths:
        rows = [row for row in load_rows(path) if row.get('stage') == 'TRACK']
        series.append(rows)
    if len(series) < 2 or any(not rows for rows in series):
        return {
            'team_run': key,
            'uav_logs': len(paths),
            'team_samples': 0,
            'all_times_have_visible_uav': 0,
            'visibility_requirement_met': 0,
            'zero_visible_samples': 0,
            'first_zero_visible_time_s': '',
            'one_visible_ratio': 0.0,
            'two_visible_ratio': 0.0,
            'three_visible_ratio': 0.0,
        }

    fov_half_angle_rad = math.radians(args.fov_half_angle_deg)
    samples = 0
    visible_counts = []
    first_zero_visible_time = None
    min_pair_distance = float('inf')
    max_tracking_error = 0.0
    mean_tracking_values = []

    for reference in series[0]:
        t = finite_float(reference.get('ros_time'))
        if t is None:
            continue
        aligned = [reference]
        for rows in series[1:]:
            row = nearest_row(rows, t, args.team_sync_tolerance)
            if row is None:
                aligned = []
                break
            aligned.append(row)
        if not aligned:
            continue

        samples += 1
        visible_count = sum(
            1 for row in aligned
            if row_visible(row, obstacles, fov_half_angle_rad, args.fov_max_distance, args.occlusion_margin)
        )
        visible_counts.append(visible_count)
        if visible_count == 0 and first_zero_visible_time is None:
            first_zero_visible_time = finite_float(reference.get('track_time'))
        for idx, a in enumerate(aligned):
            ax = finite_float(a.get('uav_x'))
            ay = finite_float(a.get('uav_y'))
            az = finite_float(a.get('uav_z'))
            err = finite_float(a.get('tracking_error_m'))
            if err is not None:
                mean_tracking_values.append(err)
                max_tracking_error = max(max_tracking_error, err)
            for b in aligned[idx + 1:]:
                bx = finite_float(b.get('uav_x'))
                by = finite_float(b.get('uav_y'))
                bz = finite_float(b.get('uav_z'))
                if None not in (ax, ay, az, bx, by, bz):
                    min_pair_distance = min(
                        min_pair_distance,
                        math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2),
                    )

    if samples == 0:
        return {
            'team_run': key,
            'uav_logs': len(paths),
            'team_samples': 0,
            'all_times_have_visible_uav': 0,
            'visibility_requirement_met': 0,
            'zero_visible_samples': 0,
            'first_zero_visible_time_s': '',
            'one_visible_ratio': 0.0,
            'two_visible_ratio': 0.0,
            'three_visible_ratio': 0.0,
        }

    zero_visible = sum(1 for count in visible_counts if count == 0)
    any_visible = sum(1 for count in visible_counts if count > 0)
    all_visible = sum(1 for count in visible_counts if count == len(series))
    one_visible = sum(1 for count in visible_counts if count == 1)
    two_visible = sum(1 for count in visible_counts if count == 2)
    three_visible = sum(1 for count in visible_counts if count == 3)
    return {
        'team_run': key,
        'uav_logs': len(paths),
        'team_samples': samples,
        'all_times_have_visible_uav': int(zero_visible == 0),
        'visibility_requirement_met': int(zero_visible == 0),
        'zero_visible_samples': zero_visible,
        'first_zero_visible_time_s': first_zero_visible_time if first_zero_visible_time is not None else '',
        'any_visible_ratio': any_visible / samples,
        'zero_visible_ratio': zero_visible / samples,
        'one_visible_ratio': one_visible / samples,
        'two_visible_ratio': two_visible / samples,
        'three_visible_ratio': three_visible / samples,
        'all_visible_ratio': all_visible / samples,
        'mean_visible_uav_count': mean(visible_counts),
        'min_visible_uav_count': min(visible_counts),
        'min_pair_distance_m': min_pair_distance,
        'mean_tracking_error_m': mean(mean_tracking_values),
        'max_tracking_error_m': max_tracking_error,
    }


def analyze_team(log_files, args):
    obstacles = load_scene_obstacles(args.scene_file)
    groups = {}
    for path in log_files:
        groups.setdefault(team_key(path), []).append(path)
    return [analyze_team_group(key, paths, obstacles, args) for key, paths in sorted(groups.items())]


def main():
    parser = argparse.ArgumentParser(description='Analyze target tracking CSV logs.')
    parser.add_argument('--log-file', action='append', default=[])
    parser.add_argument('--log-dir', default='')
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--success-error', type=float, default=0.8)
    parser.add_argument('--scene-file', default='')
    parser.add_argument('--fov-half-angle-deg', type=float, default=42.5)
    parser.add_argument('--fov-max-distance', type=float, default=8.0)
    parser.add_argument('--occlusion-margin', type=float, default=0.08)
    parser.add_argument('--team-sync-tolerance', type=float, default=0.07)
    args = parser.parse_args()

    log_files = list(args.log_file)
    if args.log_dir:
        for name in sorted(os.listdir(args.log_dir)):
            if name.endswith('.csv') and name.startswith('target_tracking_run_'):
                log_files.append(os.path.join(args.log_dir, name))

    os.makedirs(args.out_dir, exist_ok=True)
    rows = [analyze_file(path, args.success_error) for path in log_files if os.path.exists(path)]
    write_csv(os.path.join(args.out_dir, 'target_tracking_summary.csv'), rows)
    team_rows = analyze_team([path for path in log_files if os.path.exists(path)], args)
    write_csv(os.path.join(args.out_dir, 'target_tracking_team_summary.csv'), team_rows)

    valid = [row for row in rows if row.get('track_samples', 0)]
    overall = {
        'runs': len(rows),
        'valid_runs': len(valid),
        'success_rate': mean(row['success'] for row in valid) if valid else 0.0,
        'mean_tracking_error_m': mean(row['mean_tracking_error_m'] for row in valid),
        'mean_ego_planner_sample_ratio': mean(row['ego_planner_sample_ratio'] for row in valid),
        'mean_target_estimation_error_m': mean(row['mean_target_estimation_error_m'] for row in valid),
        'mean_rms_tracking_error_m': mean(row['rms_tracking_error_m'] for row in valid),
        'mean_max_tracking_error_m': mean(row['max_tracking_error_m'] for row in valid),
        'mean_duration_s': mean(row['duration_s'] for row in valid),
        'mean_rms_accel_mps2': mean(row['rms_accel_mps2'] for row in valid),
        'mean_rms_jerk_mps3': mean(row['rms_jerk_mps3'] for row in valid),
    }
    valid_team = [row for row in team_rows if row.get('team_samples', 0)]
    if valid_team:
        overall.update({
            'team_runs': len(team_rows),
            'valid_team_runs': len(valid_team),
            'team_all_times_have_visible_uav_rate': mean(row['all_times_have_visible_uav'] for row in valid_team),
            'team_visibility_requirement_met_rate': mean(row['visibility_requirement_met'] for row in valid_team),
            'team_total_zero_visible_samples': sum(row['zero_visible_samples'] for row in valid_team),
            'team_mean_any_visible_ratio': mean(row['any_visible_ratio'] for row in valid_team),
            'team_mean_zero_visible_ratio': mean(row['zero_visible_ratio'] for row in valid_team),
            'team_mean_one_visible_ratio': mean(row['one_visible_ratio'] for row in valid_team),
            'team_mean_two_visible_ratio': mean(row['two_visible_ratio'] for row in valid_team),
            'team_mean_three_visible_ratio': mean(row['three_visible_ratio'] for row in valid_team),
            'team_mean_all_visible_ratio': mean(row['all_visible_ratio'] for row in valid_team),
            'team_mean_visible_uav_count': mean(row['mean_visible_uav_count'] for row in valid_team),
            'team_min_pair_distance_m': min(row['min_pair_distance_m'] for row in valid_team),
        })
    with open(os.path.join(args.out_dir, 'target_tracking_overall.json'), 'w') as f:
        json.dump(overall, f, indent=2, sort_keys=True)

    print('Target tracking summary:', os.path.join(args.out_dir, 'target_tracking_summary.csv'))
    print('Target tracking overall:', os.path.join(args.out_dir, 'target_tracking_overall.json'))


if __name__ == '__main__':
    main()
