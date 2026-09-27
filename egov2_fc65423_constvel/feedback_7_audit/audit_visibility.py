#!/usr/bin/env python3
"""Read-only reconstruction for feedback_7.

This script only consumes the frozen scene, existing benchmark CSVs, and the
existing ALP launcher log.  It does not modify planner/runtime state.
"""

import bisect
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress_v2.json"
DATA = ROOT / "scene_geometry_visibility_v2_20260902/visibility_benchmark"
OUT = Path(__file__).resolve().parent / "audit_metrics.json"


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def lerp(a, b, u):
    return a + (b - a) * u


def interp_rows(rows, t):
    times = [float(row["time_s"]) for row in rows]
    index = bisect.bisect_left(times, t)
    if index <= 0:
        return rows[0]
    if index >= len(rows):
        return rows[-1]
    lo, hi = rows[index - 1], rows[index]
    t0, t1 = float(lo["time_s"]), float(hi["time_s"])
    if t1 <= t0:
        return lo
    u = (t - t0) / (t1 - t0)
    result = dict(lo)
    for key in ("x", "y", "z", "vx", "vy", "vz", "target_x", "target_y", "target_z",
                "target_vx", "target_vy", "target_vz", "actual_yaw_rad"):
        result[key] = lerp(float(lo[key]), float(hi[key]), u)
    result["time_s"] = t
    return result


def obstacle_items(scene):
    return [(str(key), value) for key, value in scene["obstacleData"].items()]


def static_surface_clearance(point, obstacle):
    cx, cy = obstacle["centerENU"][:2]
    radial = math.hypot(point[0] - cx, point[1] - cy) - float(obstacle["radius"])
    zmin = float(obstacle.get("zMin", 0.0))
    zmax = zmin + float(obstacle.get("height", 3.6))
    if zmin <= point[2] <= zmax:
        return radial
    vertical = zmin - point[2] if point[2] < zmin else point[2] - zmax
    return vertical if radial <= 0.0 else math.hypot(radial, vertical)


def segment_cylinder_intersects(start, end, cylinder):
    """Exact implementation of native_egov2_rviz_scene.py's finite cylinder test."""
    cx, cy, radius, zmin, zmax = cylinder
    dx, dy, dz = (end[i] - start[i] for i in range(3))
    fx, fy = start[0] - cx, start[1] - cy
    a = dx * dx + dy * dy
    intervals = []
    if a <= 1.0e-12:
        if fx * fx + fy * fy <= radius * radius:
            intervals.append((0.0, 1.0))
    else:
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            root = math.sqrt(max(0.0, disc))
            lo = max(0.0, (-b - root) / (2.0 * a))
            hi = min(1.0, (-b + root) / (2.0 * a))
            if lo <= hi:
                intervals.append((lo, hi))
    for lo, hi in intervals:
        zlo = start[2] + lo * dz
        zhi = start[2] + hi * dz
        if max(min(zlo, zhi), zmin) <= min(max(zlo, zhi), zmax):
            return True
    return False


def moving_center(obstacle, elapsed):
    axis = obstacle.get("axisENU", [1.0, 0.0])
    norm = math.hypot(float(axis[0]), float(axis[1])) or 1.0
    ux, uy = float(axis[0]) / norm, float(axis[1]) / norm
    omega = 2.0 * math.pi / max(float(obstacle.get("period", 1.0)), 1.0e-9)
    offset = float(obstacle.get("amplitude", 0.0)) * math.sin(
        omega * elapsed + float(obstacle.get("phase", 0.0)))
    center = obstacle["centerENU"]
    return float(center[0]) + ux * offset, float(center[1]) + uy * offset


def cylinders(scene, elapsed):
    static = []
    for key, obstacle in obstacle_items(scene):
        center = obstacle["centerENU"]
        zmin = float(obstacle.get("zMin", 0.0))
        static.append((key, obstacle.get("modelName", key), (
            float(center[0]), float(center[1]), float(obstacle["radius"]) + 0.08,
            zmin, zmin + float(obstacle.get("height", 3.6)))))
    dynamic = []
    for key, obstacle in scene["movingObstacleData"].items():
        center = moving_center(obstacle, elapsed)
        zmin = float(obstacle.get("zMin", 0.0))
        dynamic.append((str(key), obstacle.get("modelName", str(key)), (
            center[0], center[1], float(obstacle["radius"]) + 0.08,
            zmin, zmin + float(obstacle.get("height", 3.6)))))
    return static, dynamic


def episodes(tagged, max_gap=0.08):
    result = []
    start = previous = None
    value = None
    for t, current in tagged:
        if value is None or current != value or (previous is not None and t - previous > max_gap):
            if value is not None:
                result.append({"start": start, "end": previous, "cause": value,
                               "duration": max(0.0, previous - start)})
            start, value = t, current
        previous = t
    if value is not None:
        result.append({"start": start, "end": previous, "cause": value,
                       "duration": max(0.0, previous - start)})
    return result


def runtime_visibility(scene, rows, vis_rows, motion_offset):
    by_uav = defaultdict(list)
    for row in rows:
        by_uav[int(row["uav_id"])].append(row)
    for values in by_uav.values():
        values.sort(key=lambda row: float(row["time_s"]))
    result = {}
    for uav in (1, 2, 3):
        exclusive = Counter()
        inclusive = Counter()
        tagged = []
        detailed = []
        for visibility in vis_rows:
            t = float(visibility["time_s"])
            row = interp_rows(by_uav[uav], t)
            p = tuple(float(row[key]) for key in ("x", "y", "z"))
            target = tuple(float(row[key]) for key in ("target_x", "target_y", "target_z"))
            horizontal_range = math.hypot(target[0] - p[0], target[1] - p[1])
            rel = abs(float(visibility[f"uav{uav}_relative_bearing_rad"]))
            statics, dynamics = cylinders(scene, t + motion_offset)
            static_hits = [name for _, name, cyl in statics if segment_cylinder_intersects(p, target, cyl)]
            dynamic_hits = [name for _, name, cyl in dynamics if segment_cylinder_intersects(p, target, cyl)]
            flags = {
                "out_of_range": horizontal_range < 0.2 or horizontal_range > 8.0,
                "fov": rel > math.radians(42.5),
                "static_los": bool(static_hits),
                "dynamic_los": bool(dynamic_hits),
            }
            if int(visibility[f"visible_uav{uav}"]):
                cause = "visible"
            elif flags["out_of_range"]:
                cause = "out_of_range"
            elif flags["fov"]:
                cause = "fov"
            elif flags["static_los"]:
                cause = "static_los"
            elif flags["dynamic_los"]:
                cause = "dynamic_los"
            else:
                cause = "unexplained_or_alignment"
            exclusive[cause] += 1
            for key, active in flags.items():
                if active and cause != "visible":
                    inclusive[key] += 1
            tagged.append((t, cause))
            if cause != "visible":
                detailed.append({"time": t, "cause": cause, "range": horizontal_range,
                                 "position": p, "target": target,
                                 "static_hits": static_hits, "dynamic_hits": dynamic_hits,
                                 "relative_bearing_deg": math.degrees(rel)})
        total = len(vis_rows)
        result[str(uav)] = {
            "total": total,
            "exclusive_counts": dict(exclusive),
            "exclusive_fraction": {key: value / total for key, value in exclusive.items()},
            "inclusive_counts": dict(inclusive),
            "invisible_episodes": [entry for entry in episodes(tagged)
                                   if entry["cause"] != "visible"],
            "longest_invisible_episodes": sorted(
                [entry for entry in episodes(tagged) if entry["cause"] != "visible"],
                key=lambda entry: entry["duration"], reverse=True)[:20],
            "detailed_invisible": detailed,
        }
    return result


COMMAND_RE = re.compile(
    r"\[traj-server-command\] node=/drone_(\d+)_traj_server trajectory_id=(\d+) "
    r"command_time=([0-9.]+) position=([-0-9.eE]+),([-0-9.eE]+),([-0-9.eE]+)")
PUBLISH_RE = re.compile(
    r"\[planner-traj-publish\] drone_id=(\d+) trajectory_id=(\d+) publish_time=([0-9.]+) "
    r"start_time=([0-9.]+) duration=([0-9.]+)")
SELECT_RE = re.compile(
    r"\[planner-traj-selected\] drone_id=(\d+) candidate_type=([A-Z_]+) obstacle_id=(-?\d+) "
    r"next_trajectory_id=(\d+) selected_time=([0-9.]+) duration=([0-9.]+) "
    r"predicted_dynamic_clearance=([-0-9.infnaINFNA]+)")


def parse_log(path):
    commands, publishes, selected = [], [], []
    for line_no, line in enumerate(path.read_text(errors="ignore").splitlines(), 1):
        match = COMMAND_RE.search(line)
        if match:
            commands.append({"line": line_no, "drone": int(match.group(1)), "traj_id": int(match.group(2)),
                             "time": float(match.group(3)),
                             "position": tuple(float(match.group(i)) for i in (4, 5, 6))})
        match = PUBLISH_RE.search(line)
        if match:
            publishes.append({"line": line_no, "drone": int(match.group(1)), "traj_id": int(match.group(2)),
                              "publish": float(match.group(3)), "start": float(match.group(4)),
                              "duration": float(match.group(5))})
        match = SELECT_RE.search(line)
        if match:
            try:
                clearance = float(match.group(7))
            except ValueError:
                clearance = None
            selected.append({"line": line_no, "drone": int(match.group(1)), "kind": match.group(2),
                             "obstacle_id": int(match.group(3)), "traj_id": int(match.group(4)),
                             "time": float(match.group(5)), "duration": float(match.group(6)),
                             "predicted_dynamic_clearance": clearance})
    return commands, publishes, selected


def fit_command_offset(commands, odom_rows):
    sample = commands[::max(1, len(commands) // 500)]
    best = None
    # Both streams begin at the same target-tracking epoch.  Constrain the fit
    # around that directly observed epoch; an unconstrained fit can lock onto a
    # one-second-late local minimum while the formation is moving steadily.
    initial = commands[0]["time"] - float(odom_rows[0]["time_s"])
    for stage, step, span in ((0, 0.005, 0.20), (1, 0.0002, 0.01)):
        center = (initial if best is None else best[0])
        candidates = [center - span + index * step for index in range(int(2 * span / step) + 1)]
        for offset in candidates:
            errors = []
            for command in sample:
                relative = command["time"] - offset
                if relative < float(odom_rows[0]["time_s"]) or relative > float(odom_rows[-1]["time_s"]):
                    continue
                row = interp_rows(odom_rows, relative)
                p = tuple(float(row[key]) for key in ("x", "y", "z"))
                errors.append(math.dist(command["position"], p))
            if not errors:
                continue
            errors.sort()
            score = errors[len(errors) // 2]
            if best is None or score < best[1]:
                best = (offset, score)
    epoch = commands[0]["time"] - float(odom_rows[0]["time_s"])
    # `best` measures the controller's tracking lag, not the clock epoch.  The
    # shared epoch is directly observable from the first simultaneous command
    # and trajectory sample and is what collision-time alignment requires.
    return epoch, best[1]


def static_collision_episodes(scene, rows, uav=3):
    entries = []
    for row in rows:
        if int(row["uav_id"]) != uav:
            continue
        t = float(row["time_s"])
        point = tuple(float(row[key]) for key in ("x", "y", "z"))
        best = min(((static_surface_clearance(point, obstacle), key,
                     obstacle.get("modelName", key)) for key, obstacle in obstacle_items(scene)),
                   key=lambda entry: entry[0])
        entries.append((t, best[0], best[1], best[2], point, row))
    active = [(t, clearance < 0.0) for t, clearance, *_ in entries]
    raw_eps = episodes([(t, "collision" if state else "clear") for t, state in active])
    result = []
    for ep in raw_eps:
        if ep["cause"] != "collision":
            continue
        samples = [entry for entry in entries if ep["start"] - 1e-9 <= entry[0] <= ep["end"] + 1e-9]
        deepest = min(samples, key=lambda entry: entry[1])
        result.append({"start": ep["start"], "end": ep["end"], "duration": ep["duration"],
                       "samples": len(samples), "deepest_time": deepest[0],
                       "deepest_clearance": deepest[1], "obstacle_id": deepest[2],
                       "obstacle_name": deepest[3], "odom_position": deepest[4],
                       "target_position": tuple(float(deepest[5][key]) for key in
                                                ("target_x", "target_y", "target_z")),
                       "target_distance": float(deepest[5]["target_distance_m"])})
    return result


def attach_command_and_planner(scene, collision_eps, commands, publishes, selected, odom_rows, offset):
    obstacles = {key: obstacle for key, obstacle in obstacle_items(scene)}
    selected_map = {(entry["drone"], entry["traj_id"]): entry for entry in selected}
    publish_map = {(entry["drone"], entry["traj_id"]): entry for entry in publishes}
    for ep in collision_eps:
        abs_start, abs_end = ep["start"] + offset, ep["end"] + offset
        relevant = [entry for entry in commands if entry["drone"] == 2 and
                    abs_start - 0.08 <= entry["time"] <= abs_end + 0.08]
        obstacle = obstacles[ep["obstacle_id"]]
        command_clearances = [(static_surface_clearance(entry["position"], obstacle), entry)
                              for entry in relevant]
        if command_clearances:
            command_min, command_entry = min(command_clearances, key=lambda pair: pair[0])
            relative = command_entry["time"] - offset
            odom = interp_rows(odom_rows, relative)
            odom_p = tuple(float(odom[key]) for key in ("x", "y", "z"))
            ep["command_min_clearance_same_obstacle"] = command_min
            ep["command_at_min"] = command_entry
            ep["command_odom_error_at_min"] = math.dist(command_entry["position"], odom_p)
        before = [entry for entry in commands if entry["drone"] == 2 and entry["time"] <= abs_end]
        if before:
            active = before[-1]
            ep["active_command"] = active
            ep["active_publish"] = publish_map.get((2, active["traj_id"]))
            ep["active_selection"] = selected_map.get((2, active["traj_id"]))


def target_comparison(method_rows):
    result = {"speed_sequences": {}, "pairwise": {}}
    target_series = {}
    for method, rows in method_rows.items():
        uav1 = [row for row in rows if int(row["uav_id"]) == 1]
        speeds = []
        for row in uav1:
            speed = math.sqrt(sum(float(row[key]) ** 2 for key in ("target_vx", "target_vy", "target_vz")))
            if speed > 0.05 and not any(abs(speed - value) < 1e-4 for value in speeds):
                speeds.append(speed)
        result["speed_sequences"][method] = speeds
        target_series[method] = uav1
    methods = sorted(method_rows)
    for i, first in enumerate(methods):
        for second in methods[i + 1:]:
            end = min(float(target_series[first][-1]["time_s"]), float(target_series[second][-1]["time_s"]))
            errors = []
            t = 0.0
            while t <= end:
                a = interp_rows(target_series[first], t)
                b = interp_rows(target_series[second], t)
                pa = tuple(float(a[key]) for key in ("target_x", "target_y", "target_z"))
                pb = tuple(float(b[key]) for key in ("target_x", "target_y", "target_z"))
                errors.append(math.dist(pa, pb))
                t += 0.1
            result["pairwise"][f"{first}_vs_{second}"] = {
                "duration": end, "rms_position_difference": math.sqrt(sum(x*x for x in errors) / len(errors)),
                "max_position_difference": max(errors), "mean_position_difference": sum(errors) / len(errors)}
    return result


def main():
    scene = json.loads(SCENE_PATH.read_text())
    final_metrics = json.loads((ROOT / "scene_geometry_visibility_v2_20260902/final_metrics.json").read_text())
    method_rows = {}
    for method in ("native", "gradient", "alp"):
        method_rows[method] = read_csv(DATA / method / f"{method}_trajectory.csv")
    alp_rows = method_rows["alp"]
    alp_vis = read_csv(DATA / "alp/alp_visibility.csv")
    motion_offset = float(final_metrics["visibility"]["alp"]["motion_phase_offset_s"])
    visibility = runtime_visibility(scene, alp_rows, alp_vis, motion_offset)
    commands, publishes, selected = parse_log(DATA / "alp/alp.launcher.log")
    odom3 = sorted([row for row in alp_rows if int(row["uav_id"]) == 3],
                   key=lambda row: float(row["time_s"]))
    command3 = [entry for entry in commands if entry["drone"] == 2]
    command_offset, command_fit_median = fit_command_offset(command3, odom3)
    collisions = static_collision_episodes(scene, alp_rows, 3)
    attach_command_and_planner(scene, collisions, commands, publishes, selected, odom3, command_offset)
    for ep in collisions:
        t = ep["deepest_time"]
        closest_vis = min(alp_vis, key=lambda row: abs(float(row["time_s"]) - t))
        detail = min(visibility["3"]["detailed_invisible"],
                     key=lambda row: abs(row["time"] - t), default=None)
        ep["visibility_row"] = {key: closest_vis[key] for key in closest_vis}
        ep["visibility_detail_nearest"] = detail

    task_duration = float(alp_vis[-1]["time_s"]) - float(alp_vis[0]["time_s"])
    gates = scene["dynamicGateDesign"]["gates"]
    local_theory = json.loads((ROOT / "scene_geometry_visibility_v2_20260902/visibility_geometry_audit.json").read_text())
    output = {
        "scene_sha256_expected": "c7c28831b4306899b3305f4b4aace136a164191429749c8fc9417d5f7b27261a",
        "task_visibility_duration_s": task_duration,
        "required_block_seconds": {str(level): (1.0 - level) * task_duration for level in (0.90, 0.85, 0.80)},
        "three_gate_1p7s_budget": {"seconds": 5.1, "max_global_drop_fraction": 5.1 / task_duration},
        "gate_encounter_design": gates,
        "theoretical_audit": local_theory,
        "alp_visibility_reconstruction": visibility,
        "target_comparison": target_comparison(method_rows),
        "command_clock_offset": command_offset,
        "command_clock_fit_median_error": command_fit_median,
        "alp_uav3_static_collision_episodes": collisions,
        "log_counts": {"commands": len(commands), "publishes": len(publishes), "selected": len(selected)},
    }
    OUT.write_text(json.dumps(output, indent=2, sort_keys=True))
    print(OUT)
    print(json.dumps({
        "duration": task_duration,
        "required": output["required_block_seconds"],
        "uav3_exclusive": visibility["3"]["exclusive_fraction"],
        "uav3_longest": visibility["3"]["longest_invisible_episodes"][:8],
        "command_offset": command_offset,
        "command_fit": command_fit_median,
        "collisions": collisions,
        "target": output["target_comparison"],
    }, indent=2))


if __name__ == "__main__":
    main()
