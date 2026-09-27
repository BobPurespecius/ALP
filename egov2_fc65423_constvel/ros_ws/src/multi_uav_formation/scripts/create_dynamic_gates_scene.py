#!/usr/bin/env python3
"""Create the dynamic-topology pressure-map scene from the production scene.

The gate parameters are intentionally kept in scene JSON (the existing runtime
motion model is reused).  This script only copies/rearranges scene data and
adds design metadata; it does not alter planner parameters.
"""

import argparse
import copy
import json
import math
import os


def phase_zero_crossing(period, encounter_time, descending=False):
    omega = 2.0 * math.pi / period
    if descending:
        return math.pi - omega * encounter_time
    return -omega * encounter_time


def phase_center_dwell(period, encounter_time):
    omega = 2.0 * math.pi / period
    return -math.pi / 2.0 - omega * encounter_time


def make_scene(source):
    with open(source, "r", encoding="utf-8") as stream:
        scene = json.load(stream)

    # Route-local frames use the nominal target route.  t/n are unit tangent
    # and left horizontal normal at each gate.  Gate centers lie on the route.
    gates = [
        {
            "id": "A1", "type": "CROSSING", "encounter_time": 16.0,
            "center": [-18.5, 4.0], "t": [0.981, -0.196], "n": [0.196, 0.981],
            "amplitude": 1.10, "period": 2.0 * math.pi * 1.10 / 0.55,
            "direction": "UP", "intended_topology": "EITHER",
        },
        {
            "id": "B1", "type": "DWELL", "encounter_time": 31.0,
            "center": [-6.0, -1.0], "t": [0.983, 0.184], "n": [-0.184, 0.983],
            "amplitude": 1.00, "period": 12.0,
            "direction": "CENTER_TURNING_POINT", "intended_topology": "EITHER",
        },
        {
            "id": "C1", "type": "LEFT_OPEN", "encounter_time": 46.0,
            "center": [7.0, 1.75], "t": [0.992, 0.124], "n": [-0.124, 0.992],
            "amplitude": 0.90, "period": 11.5,
            "direction": "CENTER_TURNING_POINT", "intended_topology": "LEFT",
        },
        {
            "id": "C2", "type": "RIGHT_OPEN", "encounter_time": 58.0,
            "center": [18.5, 2.0], "t": [0.8, -0.6], "n": [0.6, 0.8],
            "amplitude": 0.90, "period": 11.5,
            "direction": "CENTER_TURNING_POINT", "intended_topology": "RIGHT",
        },
        {
            "id": "A2", "type": "CROSSING", "encounter_time": 72.0,
            "center": [29.5, -1.5], "t": [0.936, 0.351], "n": [-0.351, 0.936],
            "amplitude": 1.10, "period": 2.0 * math.pi * 1.10 / 0.55,
            "direction": "DOWN", "intended_topology": "EITHER",
        },
    ]

    moving = {}
    for idx, gate in enumerate(gates):
        center = list(gate["center"])
        axis = list(gate["n"])
        amp = gate["amplitude"]
        period = gate["period"]
        if gate["type"] == "CROSSING":
            phase = phase_zero_crossing(
                period, gate["encounter_time"], descending=(gate["direction"] == "DOWN")
            )
            if gate["direction"] == "DOWN":
                axis = [-axis[0], -axis[1]]
        else:
            # Baseline is shifted by +A along the normal, so the turning point
            # reaches the nominal route center at encounter_time.
            center = [center[0] + axis[0] * amp, center[1] + axis[1] * amp]
            phase = phase_center_dwell(period, gate["encounter_time"])
        moving[str(idx)] = {
            "modelName": "dynamic_gate_" + gate["id"],
            "centerENU": [round(center[0], 4), round(center[1], 4)],
            "radius": 0.28,
            "height": 3.6,
            "axisENU": [round(axis[0], 6), round(axis[1], 6)],
            "amplitude": round(amp, 6),
            "period": round(period, 6),
            "phase": round(phase, 6),
        }

    # Preserve the ten-object runtime interface.  These five background
    # movers are outside the route corridor and are deliberately slow.
    background = [
        ([-30.0, 8.0], 0.25, 16.0, 0.0),
        ([-10.0, -8.0], 0.25, 17.0, 1.2),
        ([5.0, 8.0], 0.25, 18.0, 2.4),
        ([20.0, -8.0], 0.25, 16.5, 3.6),
        ([35.0, 8.0], 0.25, 17.5, 4.8),
    ]
    for j, (center, amp, period, phase) in enumerate(background, start=5):
        moving[str(j)] = {
            "modelName": "dynamic_gate_background_%d" % (j - 5),
            "centerENU": center,
            "radius": 0.28,
            "height": 3.6,
            "axisENU": [0.0, 1.0],
            "amplitude": amp,
            "period": period,
            "phase": phase,
        }

    # Remove/relocate only the few forest cylinders that sit on the intended
    # open sides.  The rest of the original forest is unchanged.
    relocations = {
        "8": [10.0, 6.8],   # C1 left/open side clearance
        "11": [16.5, -5.8], # C2 right/open side clearance
        "15": [31.5, -4.8], # keep A2 crossing alternatives clear
        "22": [28.0, -5.2],
    }
    for key, center in relocations.items():
        if key in scene.get("obstacleData", {}):
            scene["obstacleData"][key]["centerENU"] = center

    # Add two static cylinders per C gate on the blocked side.  At 1.05 m
    # normal offset they do not overlap the physical dynamic cylinder, while
    # their inflated footprints remove the blocked bypass.
    next_id = max(int(k) for k in scene.get("obstacleData", {}).keys()) + 1
    for gate, blocked_sign in ((gates[2], -1.0), (gates[3], +1.0)):
        n = gate["n"]
        t = gate["t"]
        g = gate["center"]
        for blocker_idx, along in enumerate((-1.15, 1.15)):
            center = [
                g[0] + blocked_sign * n[0] * 1.05 + t[0] * along,
                g[1] + blocked_sign * n[1] * 1.05 + t[1] * along,
            ]
            scene.setdefault("obstacleData", {})[str(next_id)] = {
                "modelName": "dynamic_gate_%s_blocker_%d" % (gate["id"], blocker_idx),
                "centerENU": [round(center[0], 4), round(center[1], 4)],
                "radius": 0.55,
                "height": 3.6,
            }
            next_id += 1

    scene["movingObstacleData"] = moving
    scene["nMovingObstacle"] = len(moving)
    scene["dynamicGateDesign"] = {
        "source_scene": os.path.basename(source),
        "route_coordinate_definition": "t=route tangent, n=left horizontal normal",
        "uav_radius_m": 0.30,
        "dynamic_radius_m": 0.28,
        "static_gate_blocker_radius_m": 0.55,
        "dynamic_center_distance_contract_m": 1.10,
        "gate_spacing_policy_s": ">= 8 s",
        "gates": gates,
        "background_dynamic_obstacles": 5,
    }
    return scene


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    scene = make_scene(args.source)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(scene, stream, indent=2)
        stream.write("\n")
    print("Wrote", args.output)


if __name__ == "__main__":
    main()
