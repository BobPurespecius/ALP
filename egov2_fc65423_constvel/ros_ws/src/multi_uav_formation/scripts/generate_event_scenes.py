#!/usr/bin/env python3
"""Generate deterministic event-focused scenes from an observed trajectory.

The generator does not touch planner code.  It uses the target/UAV trajectory
CSV to choose conflict times and derives sinusoid phase from the requested
position at that time.  The generated JSON stores the derivation metadata so
that a later run can compare measured candidate clearances with the geometric
prediction.
"""

import argparse
import csv
import json
import math
import os
from copy import deepcopy


def load_target_samples(path):
    samples = {}
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            key = row["time_s"]
            if key not in samples:
                samples[key] = {
                    "time": float(row["time_s"]),
                    "target": [float(row["target_x"]), float(row["target_y"])],
                    "uav": {},
                }
            samples[key]["uav"][int(row["uav_id"])] = [
                float(row["x"]), float(row["y"])
            ]
    return sorted(samples.values(), key=lambda item: item["time"])


def nearest_target_sample(samples, x):
    return min(samples, key=lambda item: abs(item["target"][0] - x))


def static_clearance(scene, x, y):
    obstacles = (scene.get("obstacleData") or {}).values()
    values = []
    for obstacle in obstacles:
        ox, oy = obstacle.get("centerENU", [0.0, 0.0])
        values.append(math.hypot(x - ox, y - oy) - float(obstacle.get("radius", 0.5)))
    return min(values) if values else float("inf")


def side_profile(scene, x, y, offset=0.8):
    return {
        "minus": static_clearance(scene, x, y - offset),
        "center": static_clearance(scene, x, y),
        "plus": static_clearance(scene, x, y + offset),
    }


def sinusoid_phase(conflict_time, desired_offset, amplitude, period, positive_velocity=True):
    """Return phase so amplitude*sin(omega*t+phase)=desired_offset.

    The branch is selected to produce a crossing velocity at the conflict,
    making temporal scenes leave the path after the planned encounter.
    """
    ratio = max(-1.0, min(1.0, desired_offset / amplitude))
    angle = math.asin(ratio)
    if positive_velocity and math.cos(angle) < 0.0:
        angle = math.pi - angle
    return angle - 2.0 * math.pi * conflict_time / period


def obstacle(name, x, y, conflict_time, amplitude, period, desired_offset, metadata):
    phase = sinusoid_phase(conflict_time, desired_offset, amplitude, period)
    result = {
        "modelName": name,
        "centerENU": [round(x, 4), round(y, 4)],
        "radius": 0.28,
        "height": 3.6,
        "axisENU": [0.0, 1.0],
        "amplitude": amplitude,
        "period": period,
        "phase": phase,
    }
    result["eventDerivation"] = dict(metadata)
    result["eventDerivation"].update({
        "conflict_time_s": round(conflict_time, 4),
        "desired_offset_at_conflict_m": desired_offset,
        "derived_phase_rad": round(phase, 6),
    })
    return result


def base_scene(path):
    with open(path) as stream:
        scene = json.load(stream)
    return scene


def make_scene(base, kind, events):
    scene = deepcopy(base)
    scene["movingObstacleData"] = {str(i): event for i, event in enumerate(events)}
    scene["nMovingObstacle"] = len(events)
    scene["eventGenerated"] = {
        "generator": "generate_event_scenes.py",
        "kind": kind,
        "source_scene": "long_cylinder_forest.json",
        "source_trajectory": "v2_C2_visibility_run1_trajectory.csv",
        "classification": {
            "spatial": "best spatial improvement >= 0.10m and temporal < 0.05m",
            "temporal": "both spatial improvements < 0.05m and temporal >= 0.10m",
            "mixed": "best spatial >= 0.10m and temporal >= 0.10m",
        },
    }
    return scene


def build_scenes(base, samples):
    # Conflict x values are selected from observed route sections with at least
    # two UAV samples.  They are not random; each time is taken from the CSV.
    picks = [
        ("spatial", -16.0, 0.18, 20.0, 0.0, "upper route with a free upper side"),
        ("spatial", 10.0, 0.18, 20.0, 0.10, "middle route with symmetric static clearance"),
        ("spatial", -24.0, 0.16, 20.0, 0.08, "early route, near-symmetric static clearance"),
        ("temporal", -8.0, 1.45, 8.0, 0.0, "lateral crossing with short occupancy"),
        ("temporal", 4.0, 1.40, 7.5, 0.0, "middle crossing with independent phase"),
        ("temporal", 10.0, 1.45, 8.5, 0.0, "late-middle crossing with independent phase"),
        ("mixed", -20.0, 0.85, 9.5, 0.32, "offset crossing with a usable upper side"),
        ("mixed", 10.0, 0.80, 10.0, 0.28, "offset crossing with a usable lower side"),
        ("mixed", -24.0, 0.75, 9.0, 0.25, "early offset crossing and timing window"),
    ]
    generated = {"spatial": [], "temporal": [], "mixed": []}
    derivation = []
    for index, (kind, x, amplitude, period, desired_offset, intent) in enumerate(picks):
        sample = nearest_target_sample(samples, x)
        tx, ty = sample["target"]
        profile = side_profile(base, tx, ty)
        metadata = {
            "expected_type": kind,
            "intent": intent,
            "target_position_at_conflict": [round(tx, 4), round(ty, 4)],
            "observed_sample_time_s": round(sample["time"], 4),
            "static_side_clearance_m": {
                key: round(value, 4) for key, value in profile.items()
            },
            "side_offset_m": 0.8,
            "source_rule": "nearest observed target trajectory sample by x",
        }
        event = obstacle(
            "event_{}_{}".format(kind, index),
            tx,
            ty,
            sample["time"],
            amplitude,
            period,
            desired_offset,
            metadata,
        )
        generated[kind].append(event)
        derivation.append(event)
    return generated, derivation


def write_json(path, value):
    with open(path, "w") as stream:
        json.dump(value, stream, indent=4)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--trajectory-csv", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    base = base_scene(args.base_scene)
    samples = load_target_samples(args.trajectory_csv)
    if not samples:
        raise SystemExit("trajectory CSV contains no samples")
    generated, derivation = build_scenes(base, samples)
    names = {
        "spatial": "event_generated_spatial.json",
        "temporal": "event_generated_temporal.json",
        "mixed": "event_generated_mixed.json",
    }
    for kind, filename in names.items():
        write_json(
            os.path.join(args.output_dir, filename),
            make_scene(base, kind, generated[kind]),
        )
    report = {
        "source_scene": os.path.abspath(args.base_scene),
        "source_trajectory": os.path.abspath(args.trajectory_csv),
        "sample_count": len(samples),
        "events_per_scene": {kind: len(events) for kind, events in generated.items()},
        "events": derivation,
        "note": "Parameters are geometrically derived recommendations; candidate improvements require runtime measurement.",
    }
    write_json(os.path.join(args.output_dir, "event_generated_recommendations.json"), report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
