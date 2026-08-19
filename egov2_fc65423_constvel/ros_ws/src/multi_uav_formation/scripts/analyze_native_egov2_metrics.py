#!/usr/bin/env python3
"""Analyze external EGOv2 visibility and odometry metrics.

This script intentionally has no ROS dependency and never feeds metrics back
into the planner. It accepts one visibility CSV and one trajectory CSV.
"""

import argparse
import csv
import json
import math
import os


def floats(values):
    return [float(value) for value in values]


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def percentile(values, fraction):
    if not values:
        return 0.0
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def visibility_metrics(path, window):
    with open(path, newline='') as stream:
        rows = list(csv.DictReader(stream))
    rows = [row for row in rows if float(row['time_s']) >= 0.0 and
            (window is None or float(row['time_s']) <= window)]
    if not rows:
        return {'samples': 0}
    times = [float(row['time_s']) for row in rows]
    counts = [int(row['visible_count']) for row in rows]
    duration = max(0.0, times[-1] - times[0])
    result = {
        'samples': len(rows),
        'window_s': duration,
        'mean_visible_uavs': sum(counts) / len(counts),
        'all_visible_samples': sum(count == 3 for count in counts),
        'all_visible_ratio': sum(count == 3 for count in counts) / len(counts),
        'zero_visible_samples': sum(count == 0 for count in counts),
        'zero_visible_ratio': sum(count == 0 for count in counts) / len(counts),
        'visibility_count_histogram': {
            str(count): counts.count(count) for count in range(4)},
    }

    def outages(predicate):
        spans = []
        start = None
        for index, row in enumerate(rows):
            time = float(row['time_s'])
            active = predicate(row)
            if active and start is None:
                start = time
            if (not active or index == len(rows) - 1) and start is not None:
                end = time
                if not active and index > 0:
                    end = float(rows[index - 1]['time_s'])
                spans.append(max(0.0, end - start))
                start = None
        return spans

    zero_spans = outages(lambda row: int(row['visible_count']) == 0)
    result.update({
        'zero_visible_outage_count': len(zero_spans),
        'zero_visible_total_duration_s': sum(zero_spans),
        'zero_visible_longest_outage_s': max(zero_spans, default=0.0),
        'zero_visible_mean_outage_s': (sum(zero_spans) / len(zero_spans)
                                       if zero_spans else 0.0),
        'zero_visible_outage_rate_per_min': (60.0 * len(zero_spans) / duration
                                             if duration > 1.0e-6 else 0.0),
    })
    for uav in range(1, 4):
        key = 'visible_uav{}'.format(uav)
        visible = [int(row[key]) for row in rows]
        spans = outages(lambda row, key=key: int(row[key]) == 0)
        result['uav{}_visibility_ratio'.format(uav)] = sum(visible) / len(visible)
        result['uav{}_outage_count'.format(uav)] = len(spans)
        result['uav{}_longest_outage_s'.format(uav)] = max(spans, default=0.0)
        result['uav{}_total_outage_s'.format(uav)] = sum(spans)
    return result


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def trajectory_metrics(path, window, clearance_threshold, start_time):
    with open(path, newline='') as stream:
        raw = list(csv.DictReader(stream))
    grouped = {}
    for row in raw:
        time = float(row['time_s'])
        if time < start_time or (window is not None and time > window):
            continue
        grouped.setdefault(int(row['uav_id']), []).append(row)
    result = {}
    all_static = []
    all_moving = []
    team_errors = []
    formation_spreads = []
    time_groups = {}
    for row in raw:
        time = float(row['time_s'])
        if time >= start_time and (window is None or time <= window):
            time_groups.setdefault(row['time_s'], []).append(row)
    target_samples = []
    for time_key, rows_at_time in sorted(time_groups.items(), key=lambda item: float(item[0])):
        row = rows_at_time[0]
        target_samples.append((float(time_key), floats([
            row['target_x'], row['target_y'], row['target_z']])))
    for uav, rows in sorted(grouped.items()):
        rows.sort(key=lambda row: float(row['time_s']))
        positions = [floats([row['x'], row['y'], row['z']]) for row in rows]
        velocities = [floats([row['vx'], row['vy'], row['vz']]) for row in rows]
        times = [float(row['time_s']) for row in rows]
        speeds = [vector_norm(value) for value in velocities]
        accelerations = []
        jerks = []
        path_length = 0.0
        for index, (previous, current) in enumerate(zip(positions, positions[1:])):
            dt = times[index + 1] - times[index]
            if dt <= 1.0e-5 or dt > 0.5:
                continue
            path_length += vector_norm([b - a for a, b in zip(previous, current)])
        acceleration_times = []
        for index, (previous, current) in enumerate(zip(velocities, velocities[1:])):
            dt = times[index + 1] - times[index]
            if dt > 1.0e-5 and dt <= 0.5:
                accelerations.append([(b - a) / dt for a, b in zip(previous, current)])
                acceleration_times.append(times[index + 1])
        for index, (previous, current) in enumerate(zip(accelerations, accelerations[1:])):
            dt = acceleration_times[index + 1] - acceleration_times[index]
            if dt > 1.0e-5 and dt <= 0.5:
                jerks.append(vector_norm((current[i] - previous[i]) / dt
                                         for i in range(3)))
        start = positions[0]
        end = positions[-1]
        displacement = vector_norm([b - a for a, b in zip(start, end)])
        target_errors = [float(row['target_distance_m']) for row in rows]
        centroid_errors = [float(row['centroid_target_distance_m']) for row in rows]
        static = [float(row['static_clearance_m']) for row in rows]
        moving = [float(row['moving_clearance_m']) for row in rows]
        all_static.extend(static)
        all_moving.extend(moving)
        result['uav{}'.format(uav)] = {
            'samples': len(rows),
            'path_length_m': path_length,
            'straight_line_displacement_m': displacement,
            'path_inefficiency': path_length / displacement if displacement > 1.0e-6 else 0.0,
            'mean_speed_mps': sum(speeds) / len(speeds) if speeds else 0.0,
            'max_speed_mps': max(speeds, default=0.0),
            'rms_speed_mps': rms(speeds),
            'rms_acceleration_mps2': rms([vector_norm(value) for value in accelerations]),
            'max_acceleration_mps2': max([vector_norm(value) for value in accelerations], default=0.0),
            'p95_acceleration_mps2': percentile(
                [vector_norm(value) for value in accelerations], 0.95),
            'rms_jerk_mps3': rms(jerks),
            'max_jerk_mps3': max(jerks, default=0.0),
            'p95_jerk_mps3': percentile(jerks, 0.95),
            'integrated_squared_jerk': sum(value * value for value in jerks),
            'mean_target_distance_m': sum(target_errors) / len(target_errors),
            'rms_target_distance_m': rms(target_errors),
            'max_target_distance_m': max(target_errors, default=0.0),
            'mean_centroid_target_distance_m': sum(centroid_errors) / len(centroid_errors),
            'min_static_clearance_m': min(static, default=0.0),
            'min_moving_clearance_m': min(moving, default=0.0),
            'static_clearance_violation_samples': sum(value <= 0.0 for value in static),
            'moving_clearance_violation_samples': sum(value <= 0.0 for value in moving),
            'time_below_clearance_threshold_s': sum(
                max(0.0, float(current['time_s']) - float(previous['time_s']))
                for previous, current in zip(rows, rows[1:])
                if float(current['static_clearance_m']) < clearance_threshold or
                float(current['moving_clearance_m']) < clearance_threshold),
        }
    for rows_at_time in time_groups.values():
        points = [floats([row['x'], row['y'], row['z']]) for row in rows_at_time]
        if not points:
            continue
        centroid = [sum(point[index] for point in points) / len(points)
                    for index in range(3)]
        target = floats([rows_at_time[0]['target_x'], rows_at_time[0]['target_y'],
                         rows_at_time[0]['target_z']])
        team_errors.append(vector_norm([a - b for a, b in zip(centroid, target)]))
        pair_distances = [vector_norm([a - b for a, b in zip(first, second)])
                          for index, first in enumerate(points)
                          for second in points[index + 1:]]
        if pair_distances:
            formation_spreads.append(sum(pair_distances) / len(pair_distances))
    mean_spacing = (sum(formation_spreads) / len(formation_spreads)
                    if formation_spreads else 0.0)
    target_path_length = sum(
        vector_norm([b - a for a, b in zip(previous, current)])
        for (_, previous), (_, current) in zip(target_samples, target_samples[1:]))
    target_displacement = (vector_norm([b - a for a, b in zip(
        target_samples[0][1], target_samples[-1][1])]) if target_samples else 0.0)
    result['team'] = {
        'min_static_clearance_m': min(all_static, default=0.0),
        'min_moving_clearance_m': min(all_moving, default=0.0),
        'mean_centroid_target_distance_m': (sum(team_errors) / len(team_errors)
                                            if team_errors else 0.0),
        'rms_centroid_target_distance_m': rms(team_errors),
        'max_centroid_target_distance_m': max(team_errors, default=0.0),
        'mean_pairwise_spacing_m': mean_spacing,
        'std_pairwise_spacing_m': (math.sqrt(
            sum((value - mean_spacing) ** 2 for value in formation_spreads)
            / len(formation_spreads)) if formation_spreads else 0.0),
        'target_path_length_m': target_path_length,
        'target_straight_line_displacement_m': target_displacement,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--visibility-csv', required=True)
    parser.add_argument('--trajectory-csv', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--window-s', type=float, default=30.0)
    parser.add_argument('--trajectory-start-s', type=float, default=2.0)
    parser.add_argument('--clearance-threshold-m', type=float, default=0.5)
    args = parser.parse_args()
    metrics = {
        'visibility': visibility_metrics(args.visibility_csv, args.window_s),
        'trajectory': trajectory_metrics(args.trajectory_csv, args.window_s,
                                          args.clearance_threshold_m,
                                          args.trajectory_start_s),
        'inputs': {
            'visibility_csv': os.path.abspath(args.visibility_csv),
            'trajectory_csv': os.path.abspath(args.trajectory_csv),
            'window_s': args.window_s,
            'trajectory_start_s': args.trajectory_start_s,
            'clearance_threshold_m': args.clearance_threshold_m,
        },
    }
    with open(args.output, 'w') as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
