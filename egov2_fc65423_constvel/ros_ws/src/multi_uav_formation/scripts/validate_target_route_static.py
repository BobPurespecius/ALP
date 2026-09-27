#!/usr/bin/env python3
"""Exact 2-D target-route/static-geometry contract checker.

Every target polyline segment is tested against every cylinder expanded by the
target body radius plus scene margin, and every oriented wall expanded by that
same amount.  The command exits non-zero when any intersection is found and
prints one machine-readable JSON record per scene.
"""

import argparse
import json
import math
from pathlib import Path
import sys


EPS = 1.0e-9


def segment_point_distance(first, second, point):
    dx, dy = second[0] - first[0], second[1] - first[1]
    denominator = dx * dx + dy * dy
    if denominator <= EPS:
        return math.hypot(first[0] - point[0], first[1] - point[1])
    alpha = ((point[0] - first[0]) * dx +
             (point[1] - first[1]) * dy) / denominator
    alpha = max(0.0, min(1.0, alpha))
    return math.hypot(first[0] + alpha * dx - point[0],
                      first[1] + alpha * dy - point[1])


def segment_intersects_aabb(first, second, half):
    lower, upper = 0.0, 1.0
    for origin, delta, extent in zip(
            first, (second[0] - first[0], second[1] - first[1]), half):
        if abs(delta) <= EPS:
            if origin < -extent or origin > extent:
                return False
            continue
        enter = (-extent - origin) / delta
        leave = (extent - origin) / delta
        if enter > leave:
            enter, leave = leave, enter
        lower, upper = max(lower, enter), min(upper, leave)
        if lower > upper:
            return False
    return True


def wall_local(point, wall):
    center = wall["centerENU"]
    yaw = float(wall.get("yawRad", 0.0))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0] - center[0], point[1] - center[1]
    return [cosine * dx + sine * dy, -sine * dx + cosine * dy]


def point_aabb_clearance(point, half):
    dx = max(abs(point[0]) - half[0], 0.0)
    dy = max(abs(point[1]) - half[1], 0.0)
    return math.hypot(dx, dy)


def segment_aabb_clearance(first, second, half):
    if segment_intersects_aabb(first, second, half):
        return 0.0
    corners = ((-half[0], -half[1]), (half[0], -half[1]),
               (half[0], half[1]), (-half[0], half[1]))
    return min(point_aabb_clearance(first, half),
               point_aabb_clearance(second, half),
               *(segment_point_distance(first, second, corner)
                 for corner in corners))


def validate(path, body_radius, scene_margin):
    scene = json.loads(path.read_text(encoding="utf-8"))
    route = scene["targetTracking"]["waypointsENU"]
    if len(route) < 2:
        raise ValueError("target route needs at least two waypoints")
    margin = body_radius + scene_margin
    collisions = []
    minimum_residual = float("inf")

    for key, obstacle in (scene.get("obstacleData") or {}).items():
        radius = float(obstacle["radius"])
        for segment_index, (first, second) in enumerate(zip(route, route[1:])):
            surface = segment_point_distance(first, second,
                                             obstacle["centerENU"]) - radius
            minimum_residual = min(minimum_residual, surface - margin)
            if surface <= margin + EPS:
                collisions.append({
                    "primitive": "cylinder", "key": str(key),
                    "name": obstacle.get("modelName", str(key)),
                    "segment": segment_index, "residual_m": surface - margin,
                })

    for key, wall in (scene.get("wallData") or {}).items():
        size = wall["sizeENU"]
        expanded = [0.5 * float(size[0]) + margin,
                    0.5 * float(size[1]) + margin]
        # The exact rejection contract is intersection with the margin-expanded
        # oriented rectangle.  A conservative residual based on its local AABB
        # is reported for diagnostics.
        for segment_index, (first, second) in enumerate(zip(route, route[1:])):
            local_first, local_second = wall_local(first, wall), wall_local(second, wall)
            if segment_intersects_aabb(local_first, local_second, expanded):
                collisions.append({
                    "primitive": "wall", "key": str(key),
                    "name": wall.get("modelName", str(key)),
                    "segment": segment_index, "residual_m": None,
                })
            physical_half = [0.5 * float(size[0]), 0.5 * float(size[1])]
            minimum_residual = min(
                minimum_residual,
                segment_aabb_clearance(local_first, local_second,
                                       physical_half) - margin)

    result = {
        "scene": str(path),
        "route_waypoint_count": len(route),
        "route_segment_count": len(route) - 1,
        "target_body_radius_m": body_radius,
        "target_scene_margin_m": scene_margin,
        "expanded_margin_m": margin,
        "target_static_intersections": len(collisions),
        "minimum_target_static_margin_m": round(minimum_residual, 6),
        "collisions": collisions,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenes", nargs="+", type=Path)
    parser.add_argument("--target-body-radius", type=float, default=0.25)
    parser.add_argument("--target-scene-margin", type=float, default=0.05)
    args = parser.parse_args()
    failed = False
    for path in args.scenes:
        result = validate(path.resolve(), args.target_body_radius,
                          args.target_scene_margin)
        print(json.dumps(result, sort_keys=True))
        failed = failed or result["target_static_intersections"] != 0
    return 2 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
