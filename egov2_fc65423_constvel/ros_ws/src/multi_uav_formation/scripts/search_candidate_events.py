#!/usr/bin/env python3
"""Offline inverse design for candidate-aware moving-obstacle events.

This is deliberately a precheck, not a second MINCO implementation.  It uses
the current EGOv2 side initialization (plus/minus 0.8*sin(pi*u)) and the
current temporal scan p(t/s), then ranks physically admissible sinusoidal
obstacles.  Runtime logs remain the authority for final candidate outcomes.
"""

import argparse
import csv
import json
import math
import os
from collections import defaultdict


SCALES = [1.05 + 0.05 * i for i in range(7)]
SIDE_OFFSET = 0.8
RISK_HORIZON = 2.0
LOCAL_INIT_DURATION = 5.5
SAMPLE_DT = 0.05


def load_tracks(path):
    tracks = defaultdict(list)
    targets = {}
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            t = float(row["time_s"])
            uid = int(row["uav_id"])
            tracks[uid].append({
                "t": t,
                "p": (float(row["x"]), float(row["y"])),
                "v": (float(row["vx"]), float(row["vy"])),
            })
            targets.setdefault(t, (float(row["target_x"]), float(row["target_y"])))
    for values in tracks.values():
        values.sort(key=lambda row: row["t"])
    return tracks, sorted((t, p) for t, p in targets.items())


def interp(values, t):
    if t <= values[0]["t"]:
        return values[0]
    if t >= values[-1]["t"]:
        return values[-1]
    lo, hi = 0, len(values) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if values[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = values[lo], values[hi]
    u = (t - a["t"]) / max(1.0e-9, b["t"] - a["t"])
    return {
        "t": t,
        "p": tuple(a["p"][i] + u * (b["p"][i] - a["p"][i]) for i in range(2)),
        "v": tuple(a["v"][i] + u * (b["v"][i] - a["v"][i]) for i in range(2)),
    }


def static_clearance(scene, x, y):
    values = []
    for obstacle in (scene.get("obstacleData") or {}).values():
        ox, oy = obstacle.get("centerENU", [0.0, 0.0])
        values.append(math.hypot(x - ox, y - oy) - float(obstacle.get("radius", 0.5)))
    return min(values) if values else float("inf")


def obstacle_position(candidate, t):
    omega = 2.0 * math.pi / candidate["period"]
    axis_x, axis_y = candidate["axisENU"]
    offset = candidate["amplitude"] * math.sin(omega * t + candidate["phase"])
    return (
        candidate["x0"] + axis_x * offset,
        candidate["y0"] + axis_y * offset,
    )


def path_samples(track, t_conflict, scale=1.0, side=0):
    values = []
    at_conflict = interp(track, t_conflict)
    vx, vy = at_conflict["v"]
    speed = math.hypot(vx, vy)
    if speed < 0.25:
        return values
    side_dir = (vy / speed, -vx / speed)
    for i in range(int(RISK_HORIZON / SAMPLE_DT) + 1):
        local_t = min(RISK_HORIZON, i * SAMPLE_DT)
        nominal_t = t_conflict + local_t / scale
        point = interp(track, nominal_t)["p"]
        if side:
            u = min(1.0, max(0.0, local_t / LOCAL_INIT_DURATION))
            offset = side * SIDE_OFFSET * math.sin(math.pi * u)
            point = (point[0] + offset * side_dir[0], point[1] + offset * side_dir[1])
        values.append((local_t, point))
    return values


def min_clearance(path, candidate, t_conflict):
    if not path:
        return float("nan")
    return min(
        math.hypot(point[0] - obstacle_position(candidate, t_conflict + local_t)[0],
                   point[1] - obstacle_position(candidate, t_conflict + local_t)[1])
        for local_t, point in path
    )


def static_path_ok(scene, paths):
    return min(
        static_clearance(scene, point[0], point[1])
        for path in paths
        for _local_t, point in path
    ) >= 0.65


def make_candidate(track, uid, t_conflict, radial_offset, amplitude, period,
                   branch, axis_name, axis):
    current = interp(track, t_conflict)
    vx, vy = current["v"]
    speed = math.hypot(vx, vy)
    side_dir = (vy / speed, -vx / speed)
    desired_x = current["p"][0] + radial_offset * side_dir[0]
    desired_y = current["p"][1] + radial_offset * side_dir[1]
    # Put the obstacle at the intended conflict point at a zero crossing.
    # Branch selects which way it traverses that point, not a random phase.
    angle = math.pi if branch else 0.0
    phase = angle - 2.0 * math.pi * t_conflict / period
    return {
        "uav_id": uid,
        "conflict_time": t_conflict,
        "x0": desired_x - axis[0] * amplitude * math.sin(angle),
        "y0": desired_y - axis[1] * amplitude * math.sin(angle),
        "axisENU": axis,
        "axis_name": axis_name,
        "amplitude": amplitude,
        "period": period,
        "phase": phase,
        "radial_offset": radial_offset,
    }


def obstacle_motion_ok(scene, candidate):
    """Keep the entire oscillation inside map bounds and off static cylinders."""
    safety = scene.get("safety", {})
    radius = 0.28
    margin = radius + 0.15
    x_min = float(safety.get("xMin", -39.0)) + margin
    x_max = float(safety.get("xMax", 39.0)) - margin
    y_min = float(safety.get("yMin", -13.5)) + margin
    y_max = float(safety.get("yMax", 13.5)) - margin
    for index in range(81):
        t = candidate["conflict_time"] + candidate["period"] * index / 80.0
        x, y = obstacle_position(candidate, t)
        if x < x_min or x > x_max or y < y_min or y > y_max:
            return False
        if static_clearance(scene, x, y) < margin:
            return False
    return True


def score_event(scene, track, candidate, paths=None, static_margin=None):
    t_conflict = candidate["conflict_time"]
    if paths is None:
        nominal = path_samples(track, t_conflict, 1.0, 0)
        plus = path_samples(track, t_conflict, 1.0, 1)
        minus = path_samples(track, t_conflict, 1.0, -1)
    else:
        nominal, plus, minus = paths
    if not nominal:
        return None
    if static_margin is None:
        static_margin = min(static_clearance(scene, point[0], point[1])
                            for path in (nominal, plus, minus)
                            for _local_t, point in path)
    if static_margin < 0.65:
        return None
    d_nominal = min_clearance(nominal, candidate, t_conflict)
    d_plus = min_clearance(plus, candidate, t_conflict)
    d_minus = min_clearance(minus, candidate, t_conflict)
    temporal_values = []
    for scale in SCALES:
        temporal_path = path_samples(track, t_conflict, scale, 0)
        temporal_values.append((min_clearance(temporal_path, candidate, t_conflict), scale))
    d_temporal, best_scale = max(temporal_values)
    result = dict(candidate)
    result.update({
        "d_nominal": d_nominal,
        "d_plus": d_plus,
        "d_minus": d_minus,
        "d_temporal_best": d_temporal,
        "delta_spatial": max(d_plus, d_minus) - d_nominal,
        "delta_temporal": d_temporal - d_nominal,
        "best_side": "PLUS" if d_plus >= d_minus else "MINUS",
        "best_scale": best_scale,
        "static_clearance": static_margin,
    })
    result["predicted_type"] = classify(result)
    return result


def classify(result):
    ds, dt = result["delta_spatial"], result["delta_temporal"]
    if ds >= 0.10 and dt >= 0.10:
        return "Mixed"
    if ds >= 0.10 and dt < 0.05:
        return "Spatial"
    if ds < 0.05 and dt >= 0.10:
        return "Temporal"
    return "Low-information"


def robust(scene, track, result):
    checks = []
    for dt in (-0.2, 0.0, 0.2):
        for phase_delta in (-0.08, 0.0, 0.08):
            probe = dict(result)
            probe["conflict_time"] += dt
            probe["phase"] += phase_delta
            scored = score_event(scene, track, probe)
            checks.append(scored is not None and classify(scored) == result["predicted_type"])
    result["robustness"] = {
        "checks": len(checks),
        "passed": sum(checks),
        "ratio": sum(checks) / len(checks),
        "pass": sum(checks) >= 7,
    }
    return result


def search(scene, tracks):
    ranked = []
    # A deterministic, physically spaced grid.  Values span slow/fast and
    # shallow/deep crossings without treating phase or amplitude as random
    # trial knobs.  Phase is analytically derived from the conflict point.
    amplitudes = (0.35, 0.70, 1.10)
    periods = (7.0, 9.5, 13.0, 16.0)
    offsets = (-0.30, 0.0, 0.30)
    for uid, track in sorted(tracks.items()):
        for t_conflict in [8.0 + 0.5 * i for i in range(160)]:
            if t_conflict > track[-1]["t"] - RISK_HORIZON:
                break
            current = interp(track, t_conflict)
            if math.hypot(*current["v"]) < 0.25:
                continue
            paths = (path_samples(track, t_conflict, 1.0, 0),
                     path_samples(track, t_conflict, 1.0, 1),
                     path_samples(track, t_conflict, 1.0, -1))
            static_margin = min(static_clearance(scene, point[0], point[1])
                                for path in paths
                                for _local_t, point in path)
            if static_margin < 0.65:
                continue
            tangent = (current["v"][0] / math.hypot(*current["v"]),
                       current["v"][1] / math.hypot(*current["v"]))
            normal = (tangent[1], -tangent[0])
            diagonal_plus = ((tangent[0] + normal[0]) / math.sqrt(2.0),
                             (tangent[1] + normal[1]) / math.sqrt(2.0))
            diagonal_minus = ((tangent[0] - normal[0]) / math.sqrt(2.0),
                              (tangent[1] - normal[1]) / math.sqrt(2.0))
            axes = (("tangent", tangent), ("normal", normal),
                    ("diag_plus", diagonal_plus), ("diag_minus", diagonal_minus))
            for amplitude in amplitudes:
                for period in periods:
                    for offset in offsets:
                        for axis_name, axis in axes:
                            for branch in (0, 1):
                                candidate = make_candidate(track, uid, t_conflict,
                                                           offset, amplitude, period,
                                                           branch, axis_name, axis)
                                scored = score_event(scene, track, candidate,
                                                     paths=paths,
                                                     static_margin=static_margin)
                                if scored is None:
                                    continue
                                if scored["predicted_type"] not in ("Spatial", "Temporal", "Mixed"):
                                    continue
                                if not obstacle_motion_ok(scene, candidate):
                                    continue
                                scored = robust(scene, track, scored)
                                if not scored["robustness"]["pass"]:
                                    continue
                                # Prefer the type-specific threshold margin, then map clearance.
                                if scored["predicted_type"] == "Mixed":
                                    margin = min(scored["delta_spatial"] - 0.10,
                                                 scored["delta_temporal"] - 0.10)
                                elif scored["predicted_type"] == "Temporal":
                                    margin = min(0.05 - scored["delta_spatial"],
                                                 scored["delta_temporal"] - 0.10)
                                else:
                                    margin = min(scored["delta_spatial"] - 0.10,
                                                 0.05 - scored["delta_temporal"])
                                scored["ranking_margin"] = margin
                                ranked.append(scored)
    ranked.sort(key=lambda x: (
        0 if x["predicted_type"] == "Mixed" else 1 if x["predicted_type"] == "Temporal" else 2,
        -x["ranking_margin"], -x["static_clearance"]))
    return ranked


def scene_from_event(base, event, kind, output_path):
    scene = json.loads(json.dumps(base))
    obstacle = {
        "modelName": "candidate_{}_uav{}".format(kind.lower(), event["uav_id"]),
        "centerENU": [event["x0"], event["y0"]],
        "radius": 0.28,
        "height": 3.6,
        "axisENU": event["axisENU"],
        "amplitude": event["amplitude"],
        "period": event["period"],
        "phase": event["phase"],
        "eventDerivation": event,
    }
    scene["movingObstacleData"] = {"0": obstacle}
    scene["nMovingObstacle"] = 1
    scene["eventGenerated"] = {"generator": "search_candidate_events.py",
                                "predicted_type": kind}
    with open(output_path, "w") as stream:
        json.dump(scene, stream, indent=4)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--trajectory-csv", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    with open(args.base_scene) as stream:
        scene = json.load(stream)
    tracks, _targets = load_tracks(args.trajectory_csv)
    ranked = search(scene, tracks)
    os.makedirs(args.output_dir, exist_ok=True)
    type_counts = {kind: sum(x["predicted_type"] == kind for x in ranked)
                   for kind in ("Mixed", "Temporal", "Spatial")}
    report = {
        "source": {
            "base_scene": os.path.abspath(args.base_scene),
            "trajectory_csv": os.path.abspath(args.trajectory_csv),
            "uav_ids": sorted(tracks),
        },
        "thresholds": {"spatial": 0.10, "temporal": 0.10,
                       "spatial_only_temporal_max": 0.05,
                       "temporal_only_spatial_max": 0.05},
        "search_grid": {
            "conflict_time_s": "8.0 through 87.5, 0.5 s spacing",
            "radial_offset_m": [-0.30, 0.0, 0.30],
            "amplitude_m": [0.35, 0.70, 1.10],
            "period_s": [7.0, 9.5, 13.0, 16.0],
            "axis": ["tangent", "normal", "diag_plus", "diag_minus"],
            "phase": "analytically set to a zero crossing at conflict time; both crossing directions tested",
        },
        "physical_filters": {
            "minimum_uav_speed_mps": 0.25,
            "minimum_static_path_clearance_m": 0.65,
            "moving_obstacle_radius_m": 0.28,
            "moving_obstacle_static_margin_m": 0.43,
            "map_boundary_margin_m": 0.43,
            "robustness": "type retained for >= 7 of 9 checks over conflict time +/-0.2 s and phase +/-0.08 rad",
        },
        "candidate_count": len(ranked),
        "candidate_count_by_type": type_counts,
        "candidates": ranked,
    }
    with open(os.path.join(args.output_dir, "candidate_event_search.json"), "w") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    by_type = {}
    for kind in ("Mixed", "Temporal", "Spatial"):
        values = [x for x in ranked if x["predicted_type"] == kind]
        if values:
            by_type[kind] = values[0]
            scene_from_event(scene, values[0], kind,
                             os.path.join(args.output_dir,
                                          "candidate_event_{}.json".format(kind.lower())))
    with open(os.path.join(args.output_dir, "candidate_event_search.md"), "w") as stream:
        stream.write("# Candidate-Aware Event Search\n\n")
        stream.write("Candidates are derived deterministically from the C2 trajectory CSV, not from random phase/amplitude trials. The precheck mirrors the current MINCO-before-candidate geometry: SIDE uses `+/-0.8*sin(pi*u)` and TEMPORAL uses `p_nominal(t/scale)` for scales 1.05 through 1.35.\n\n")
        stream.write("Source trajectory: `{}`.\n\n".format(os.path.abspath(args.trajectory_csv)))
        stream.write("Grid: conflict times 8.0--87.5 s (0.5 s); radial offsets -0.30/0.00/0.30 m; amplitudes 0.35/0.70/1.10 m; periods 7.0/9.5/13.0/16.0 s; tangent/normal/two diagonal axes; analytic zero-crossing phase in both travel directions.\n\n")
        stream.write("Filters: UAV speed >= 0.25 m/s, nominal/SIDE static clearance >= 0.65 m, moving cylinder keeps >= 0.43 m static/map margin over a full period, and event type survives at least 7/9 perturbations (`t_conflict +/- 0.2 s`, `phase +/- 0.08 rad`).\n\n")
        stream.write("Robust candidates: **{}** (Mixed {}, Temporal {}, Spatial {}).\n\n".format(
            len(ranked), type_counts["Mixed"], type_counts["Temporal"], type_counts["Spatial"]))
        stream.write("| Type | UAV | t_conflict | d_nominal | d_plus | d_minus | d_temporal | delta_spatial | delta_temporal | side | scale | robustness |\n|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|\n")
        for kind in ("Mixed", "Temporal", "Spatial"):
            if kind not in by_type:
                stream.write("| {} | - | - | no robust candidate | | | | | | | | |\n".format(kind))
                continue
            x = by_type[kind]
            stream.write("| {} | {} | {:.2f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {} | {:.2f} | {}/{} |\n".format(
                kind, x["uav_id"], x["conflict_time"], x["d_nominal"], x["d_plus"],
                x["d_minus"], x["d_temporal_best"], x["delta_spatial"],
                x["delta_temporal"], x["best_side"], x["best_scale"],
                x["robustness"]["passed"], x["robustness"]["checks"]))
        stream.write("\nThe search models only pre-MINCO geometry. Runtime validation remains required because the replan trajectory can diverge from the C2 source trajectory and MINCO can change candidate clearance.\n")
    print(json.dumps({"candidate_count": len(ranked),
                      "top_by_type": by_type}, indent=2))


if __name__ == "__main__":
    main()
