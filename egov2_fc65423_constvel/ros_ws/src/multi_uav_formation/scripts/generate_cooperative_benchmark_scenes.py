#!/usr/bin/env python3
"""Generate deterministic ALP cooperative benchmark scenes.

The generator reads the production scene, launch defaults, and the current
SIDE amplitude from planner_manager.cpp.  Only benchmark geometry choices
(station spacing, desired occlusion angle, and crossing speed) live here.
No planner or controller parameter is changed.
"""

import argparse
import copy
import itertools
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
PACKAGE = SCRIPT.parent.parent
ROOT = SCRIPT.parents[4]
SCENES = PACKAGE / "scenes"
DEFAULT_SOURCE = SCENES / "long_cylinder_forest_visibility_stress.json"
DEFAULT_LAUNCH = PACKAGE / "launch" / "native_egov2_rviz.launch"
DEFAULT_PLANNER = (
    ROOT / "ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/"
    "plan_manage/src/planner_manager.cpp"
)
AVOIDANCE_OUTPUT = SCENES / "alp_cooperative_avoidance_benchmark.json"
VISIBILITY_OUTPUT = SCENES / "alp_cooperative_visibility_benchmark.json"

GENERATOR_ID = "generate_cooperative_benchmark_scenes.py"
TARGET_ROUTE_VALIDATOR = SCRIPT.parent / "validate_target_route_static.py"
EPS = 1.0e-9


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def add(a, b):
    return [a[0] + b[0], a[1] + b[1]]


def subtract(a, b):
    return [a[0] - b[0], a[1] - b[1]]


def scale(a, factor):
    return [a[0] * factor, a[1] * factor]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def norm(a):
    return math.hypot(a[0], a[1])


def unit(a):
    length = norm(a)
    if length <= EPS:
        raise ValueError("zero-length vector")
    return [a[0] / length, a[1] / length]


def distance(a, b):
    return norm(subtract(a, b))


def left_normal(tangent):
    return [-tangent[1], tangent[0]]


def right_normal(tangent):
    # Matches planner_manager.cpp: tangent.cross(UnitZ()).
    return [tangent[1], -tangent[0]]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_distance(first, second):
    return abs(wrap_angle(first - second))


def round_vector(value, digits=6):
    return [round(float(component), digits) for component in value]


class Polyline:
    def __init__(self, points):
        if len(points) < 2:
            raise ValueError("target route needs at least two points")
        self.points = [[float(point[0]), float(point[1])] for point in points]
        self.segment_lengths = [
            distance(first, second)
            for first, second in zip(self.points, self.points[1:])
        ]
        if any(length <= EPS for length in self.segment_lengths):
            raise ValueError("target route contains a zero-length segment")
        self.cumulative = [0.0]
        for length in self.segment_lengths:
            self.cumulative.append(self.cumulative[-1] + length)
        self.length = self.cumulative[-1]

    def sample(self, arc_length):
        s = clamp(float(arc_length), 0.0, self.length)
        for index, end in enumerate(self.cumulative[1:]):
            if s <= end + EPS:
                begin = self.cumulative[index]
                alpha = (s - begin) / self.segment_lengths[index]
                first = self.points[index]
                second = self.points[index + 1]
                point = add(first, scale(subtract(second, first), alpha))
                tangent = unit(subtract(second, first))
                return point, tangent
        return self.points[-1], unit(subtract(self.points[-1], self.points[-2]))


def segment_point_distance(first, second, point):
    delta = subtract(second, first)
    denominator = dot(delta, delta)
    if denominator <= EPS:
        return distance(first, point)
    alpha = clamp(dot(subtract(point, first), delta) / denominator, 0.0, 1.0)
    return distance(add(first, scale(delta, alpha)), point)


def segment_intersects_circle(first, second, center, radius):
    return segment_point_distance(first, second, center) <= radius + 1.0e-9


def wall_geometry(wall):
    center = [float(value) for value in wall["centerENU"][:2]]
    size = wall.get("sizeENU", [1.0, 0.2, wall.get("height", 3.0)])
    if len(size) < 2 or float(size[0]) <= 0.0 or float(size[1]) <= 0.0:
        raise ValueError("wall sizeENU needs positive length and thickness")
    return center, [0.5 * float(size[0]), 0.5 * float(size[1])], float(
        wall.get("yawRad", 0.0))


def world_to_wall_local(point, wall):
    center, _half, yaw = wall_geometry(wall)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0] - center[0], point[1] - center[1]
    return [cosine * dx + sine * dy, -sine * dx + cosine * dy]


def wall_local_to_world(point, wall):
    center, _half, yaw = wall_geometry(wall)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [center[0] + cosine * point[0] - sine * point[1],
            center[1] + sine * point[0] + cosine * point[1]]


def wall_corners(wall):
    _center, half, _yaw = wall_geometry(wall)
    return [wall_local_to_world([sign_x * half[0], sign_y * half[1]], wall)
            for sign_x, sign_y in ((-1.0, -1.0), (1.0, -1.0),
                                   (1.0, 1.0), (-1.0, 1.0))]


def point_wall_clearance(point, wall):
    local = world_to_wall_local(point, wall)
    _center, half, _yaw = wall_geometry(wall)
    dx = max(abs(local[0]) - half[0], 0.0)
    dy = max(abs(local[1]) - half[1], 0.0)
    return math.hypot(dx, dy)


def segment_intersects_aabb(first, second, half):
    lower, upper = 0.0, 1.0
    for origin, delta, extent in zip(
            first, subtract(second, first), half):
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


def segment_wall_clearance(first, second, wall):
    local_first = world_to_wall_local(first, wall)
    local_second = world_to_wall_local(second, wall)
    _center, half, _yaw = wall_geometry(wall)
    if segment_intersects_aabb(local_first, local_second, half):
        return 0.0
    corners = [[-half[0], -half[1]], [half[0], -half[1]],
               [half[0], half[1]], [-half[0], half[1]]]
    return min(
        point_wall_clearance(first, wall),
        point_wall_clearance(second, wall),
        *(segment_point_distance(local_first, local_second, corner)
          for corner in corners),
    )


def wall_wall_clearance(first, second):
    first_corners = wall_corners(first)
    second_corners = wall_corners(second)
    for index in range(4):
        first_a, first_b = first_corners[index], first_corners[(index + 1) % 4]
        if segment_wall_clearance(first_a, first_b, second) <= EPS:
            return 0.0
    return min(
        *(point_wall_clearance(point, second) for point in first_corners),
        *(point_wall_clearance(point, first) for point in second_corners),
    )


def primitive_point_clearance(point, cylinders, walls):
    values = [distance(point, item["centerENU"]) - float(item["radius"])
              for item in cylinders]
    values.extend(point_wall_clearance(point, item) for item in walls)
    return min(values, default=float("inf"))


def primitive_segment_clearance(first, second, cylinders, walls):
    values = [segment_point_distance(first, second, item["centerENU"]) -
              float(item["radius"]) for item in cylinders]
    values.extend(segment_wall_clearance(first, second, item) for item in walls)
    return min(values, default=float("inf"))


def launch_defaults(path):
    root = ET.parse(path).getroot()
    defaults = {}
    for element in root.findall("./arg"):
        if "name" in element.attrib:
            defaults[element.attrib["name"]] = element.attrib.get(
                "default", element.attrib.get("value", "")
            )

    pattern = re.compile(r"^\$\(arg ([^)]+)\)$")

    def resolve(value):
        match = pattern.match(value)
        return defaults[match.group(1)] if match else value

    drones = []
    for include in root.findall("./include"):
        values = {
            item.attrib.get("name"): resolve(item.attrib.get("value", ""))
            for item in include.findall("./arg")
        }
        if "drone_id" not in values or "relative_tracking_x" not in values:
            continue
        drone_id = int(values["drone_id"])
        drones.append({
            "drone_id": drone_id,
            "initial": [
                float(values["init_x"]),
                float(values["init_y"]),
                float(values["init_z"]),
            ],
            "tracking_offset": [
                float(values["relative_tracking_x"]),
                float(values["relative_tracking_y"]),
                float(values["relative_tracking_z"]),
            ],
        })
    drones.sort(key=lambda item: item["drone_id"])
    if [item["drone_id"] for item in drones] != [0, 1, 2]:
        raise ValueError("failed to recover all three production UAV configurations")
    return defaults, drones


def float_default(defaults, name):
    try:
        return float(defaults[name])
    except (KeyError, ValueError) as error:
        raise ValueError("missing numeric launch default: {}".format(name)) from error


def bool_default(defaults, name):
    return defaults.get(name, "false").strip().lower() in ("1", "true", "yes", "on")


def side_offset_from_source(path):
    text = path.read_text(encoding="utf-8")
    match = re.search(
        r"constexpr\s+double\s+kCandidateSideOffset\s*=\s*([-+0-9.eE]+)", text
    )
    if not match:
        raise ValueError("kCandidateSideOffset not found in {}".format(path))
    return float(match.group(1))


def base_scene(source, route, speed, drones):
    source_scene = json.loads(source.read_text(encoding="utf-8"))
    scene = copy.deepcopy(source_scene)
    # The source scene carries its own explanatory gate layout.  It is not a
    # runtime schema field, and retaining it would falsely describe the new
    # benchmark geometry after obstacleData/movingObstacleData are replaced.
    scene.pop("dynamicGateDesign", None)
    scene.pop("benchmarkDesign", None)
    height = float((source_scene.get("targetTracking") or {}).get("targetHeight", 1.5))
    route_3d = [[point[0], point[1], height] for point in route.points]
    scene["number"] = 3
    scene["takeoffPointENU"] = [round_vector(item["initial"]) for item in drones]
    scene["preparePointENU"] = copy.deepcopy(scene["takeoffPointENU"])
    target = copy.deepcopy(source_scene.get("targetTracking") or {})
    target.update({
        "mode": "waypoints",
        "speed": speed,
        "dynamicObstacleAvoidanceEnabled": False,
        "takeoffHeight": height,
        "targetHeight": height,
        "useWaypointZ": False,
        "lockEgoTrackingHeight": True,
        "trackMaxControlZ": height,
        "waypointsENU": route_3d,
    })
    scene["targetTracking"] = target
    planner = copy.deepcopy(source_scene.get("egoPlanner") or {})
    goal = route.points[-1]
    planner["goalsENU"] = [
        [round(goal[0] + item["tracking_offset"][0], 6),
         round(goal[1] + item["tracking_offset"][1], 6), height]
        for item in drones
    ]
    scene["egoPlanner"] = planner
    return scene


def station_arc_lengths(route_length, count, end_margin):
    usable = route_length - 2.0 * end_margin
    if usable <= 0.0:
        raise ValueError("route is too short for benchmark stations")
    return [
        end_margin + usable * index / float(count - 1)
        for index in range(count)
    ]


def phase_for_crossing(encounter_time, desired_offset, amplitude, period,
                       velocity_sign):
    ratio = clamp(desired_offset / amplitude, -1.0, 1.0)
    angle = math.asin(ratio)
    if velocity_sign < 0.0:
        angle = math.pi - angle
    omega = 2.0 * math.pi / period
    phase = angle - omega * encounter_time
    speed = amplitude * omega * math.cos(angle)
    return phase, speed


def moving_position(obstacle, elapsed):
    axis = unit(obstacle["axisENU"])
    amplitude = float(obstacle["amplitude"])
    period = float(obstacle["period"])
    phase = float(obstacle["phase"])
    offset = amplitude * math.sin(2.0 * math.pi * elapsed / period + phase)
    return add(obstacle["centerENU"], scale(axis, offset))


def target_position_at_time(route, speed, elapsed):
    return route.sample(min(route.length, max(0.0, elapsed * speed)))[0]


def minimum_target_dynamic_clearance(route, speed, moving, target_radius,
                                     target_margin):
    duration = route.length / speed
    best = float("inf")
    best_record = None
    steps = max(1, int(math.ceil(duration / 0.05)))
    for index in range(steps + 1):
        elapsed = duration * index / float(steps)
        target = target_position_at_time(route, speed, elapsed)
        for obstacle in moving.values():
            center = moving_position(obstacle, elapsed)
            clearance = distance(target, center) - (
                target_radius + float(obstacle["radius"]) + target_margin
            )
            if clearance < best:
                best = clearance
                best_record = {
                    "time_s": elapsed,
                    "target": target,
                    "obstacle": obstacle["modelName"],
                    "center": center,
                }
    return best, best_record


def minimum_route_static_clearance(route, obstacles):
    best = float("inf")
    best_name = None
    for obstacle in obstacles.values():
        center = obstacle["centerENU"]
        radius = float(obstacle["radius"])
        for first, second in zip(route.points, route.points[1:]):
            clearance = segment_point_distance(first, second, center) - radius
            if clearance < best:
                best = clearance
                best_name = obstacle["modelName"]
    return best, best_name


def minimum_dynamic_static_swept_clearance(moving, static, margin=0.0):
    """Exact clearance between each sinusoid's full swept segment and statics."""
    best = float("inf")
    best_record = None
    for dynamic in moving.values():
        axis = unit(dynamic["axisENU"])
        amplitude = float(dynamic["amplitude"])
        first = add(dynamic["centerENU"], scale(axis, -amplitude))
        second = add(dynamic["centerENU"], scale(axis, amplitude))
        for fixed in static.values():
            clearance = segment_point_distance(
                first, second, fixed["centerENU"]
            ) - float(dynamic["radius"]) - float(fixed["radius"]) - margin
            if clearance < best:
                best = clearance
                best_record = {
                    "moving": dynamic["modelName"],
                    "static": fixed["modelName"],
                    "swept_segment_start": round_vector(first),
                    "swept_segment_end": round_vector(second),
                }
    return best, best_record


def minimum_dynamic_pair_clearance(moving, duration, step=0.02):
    best = float("inf")
    best_record = None
    items = list(moving.values())
    steps = max(1, int(math.ceil(duration / step)))
    for index in range(steps + 1):
        elapsed = duration * index / float(steps)
        centers = [moving_position(item, elapsed) for item in items]
        for first_index in range(len(items)):
            for second_index in range(first_index + 1, len(items)):
                clearance = distance(
                    centers[first_index], centers[second_index]
                ) - float(items[first_index]["radius"]) - float(
                    items[second_index]["radius"]
                )
                if clearance < best:
                    best = clearance
                    best_record = {
                        "time_s": round(elapsed, 6),
                        "first": items[first_index]["modelName"],
                        "second": items[second_index]["modelName"],
                    }
    return best, best_record


def minimum_initial_static_clearance(drones, static):
    best = float("inf")
    best_record = None
    for drone in drones:
        initial = drone["initial"][:2]
        for obstacle in static.values():
            clearance = distance(initial, obstacle["centerENU"]) - float(
                obstacle["radius"]
            )
            if clearance < best:
                best = clearance
                best_record = {
                    "drone": drone["drone_id"],
                    "static": obstacle["modelName"],
                }
    return best, best_record


def minimum_static_pair_clearance(static):
    best = float("inf")
    best_record = None
    items = list(static.values())
    for first, second in itertools.combinations(items, 2):
        clearance = distance(first["centerENU"], second["centerENU"]) - (
            float(first["radius"]) + float(second["radius"])
        )
        if clearance < best:
            best = clearance
            best_record = {
                "first": first["modelName"],
                "second": second["modelName"],
            }
    return best, best_record


def minimum_route_wall_clearance(route, walls):
    best = float("inf")
    best_name = None
    for wall in walls.values():
        for first, second in zip(route.points, route.points[1:]):
            clearance = segment_wall_clearance(first, second, wall)
            if clearance < best:
                best = clearance
                best_name = wall["modelName"]
    return best, best_name


def target_static_intersections(route, cylinders, walls, margin):
    """Return primitives intersecting the complete margin-expanded route."""
    intersections = []
    for key, obstacle in cylinders.items():
        expanded_radius = float(obstacle["radius"]) + float(margin)
        if any(segment_intersects_circle(first, second,
                                         obstacle["centerENU"],
                                         expanded_radius)
               for first, second in zip(route.points, route.points[1:])):
            intersections.append({
                "type": "cylinder", "key": str(key),
                "name": obstacle["modelName"],
            })
    for key, wall in walls.items():
        center, half, yaw = wall_geometry(wall)
        expanded = [half[0] + float(margin), half[1] + float(margin)]
        hit = False
        for first, second in zip(route.points, route.points[1:]):
            cosine, sine = math.cos(yaw), math.sin(yaw)
            local = []
            for point in (first, second):
                dx, dy = point[0] - center[0], point[1] - center[1]
                local.append([cosine * dx + sine * dy,
                              -sine * dx + cosine * dy])
            if segment_intersects_aabb(local[0], local[1], expanded):
                hit = True
                break
        if hit:
            intersections.append({
                "type": "wall", "key": str(key),
                "name": wall["modelName"],
            })
    return intersections


def minimum_dynamic_wall_swept_clearance(moving, walls, margin=0.0):
    best = float("inf")
    best_record = None
    for dynamic in moving.values():
        axis = unit(dynamic["axisENU"])
        amplitude = float(dynamic["amplitude"])
        first = add(dynamic["centerENU"], scale(axis, -amplitude))
        second = add(dynamic["centerENU"], scale(axis, amplitude))
        for wall in walls.values():
            clearance = segment_wall_clearance(first, second, wall) - (
                float(dynamic["radius"]) + margin)
            if clearance < best:
                best = clearance
                best_record = {
                    "moving": dynamic["modelName"],
                    "wall": wall["modelName"],
                    "swept_segment_start": round_vector(first),
                    "swept_segment_end": round_vector(second),
                }
    return best, best_record


def minimum_initial_wall_clearance(drones, walls):
    best = float("inf")
    best_record = None
    for drone in drones:
        for wall in walls.values():
            clearance = point_wall_clearance(drone["initial"][:2], wall)
            if clearance < best:
                best = clearance
                best_record = {
                    "drone": drone["drone_id"],
                    "wall": wall["modelName"],
                }
    return best, best_record


def minimum_static_primitive_pair_clearance(static, walls):
    best, best_record = minimum_static_pair_clearance(static)
    for cylinder in static.values():
        for wall in walls.values():
            clearance = point_wall_clearance(cylinder["centerENU"], wall) - float(
                cylinder["radius"])
            if clearance < best:
                best = clearance
                best_record = {
                    "first": cylinder["modelName"],
                    "second": wall["modelName"],
                }
    for first, second in itertools.combinations(walls.values(), 2):
        clearance = wall_wall_clearance(first, second)
        if clearance < best:
            best = clearance
            best_record = {
                "first": first["modelName"],
                "second": second["modelName"],
            }
    return best, best_record


def radius_distribution(static):
    radii = [float(item["radius"]) for item in static.values()]
    return {
        "minimum_m": round(min(radii), 6),
        "mean_m": round(sum(radii) / len(radii), 6),
        "maximum_m": round(max(radii), 6),
        "fine_count": sum(value < 0.40 for value in radii),
        "medium_count": sum(0.40 <= value < 0.75 for value in radii),
        "coarse_count": sum(value >= 0.75 for value in radii),
    }


def obstacle_occludes_tracking_shell(route, obstacle, offsets, margin,
                                     side_offset=0.0, route_step=0.35):
    """Sample real target-to-candidate rays, solely for pressure classification."""
    count = max(1, int(math.ceil(route.length / route_step)))
    for index in range(count + 1):
        target, tangent = route.sample(route.length * index / float(count))
        side = right_normal(tangent)
        for offset in offsets:
            nominal = add(target, offset[:2])
            for sign in (-1.0, 0.0, 1.0):
                endpoint = add(nominal, scale(side, sign * side_offset))
                if segment_intersects_circle(
                        target, endpoint, obstacle["centerENU"],
                        float(obstacle["radius"]) + margin):
                    return True
    return False


def pressure_classification(route, static, drones, planner_clearance,
                            visibility_margin, side_offset):
    offsets = [item["tracking_offset"] for item in drones]
    result = {
        "motion_affecting": [],
        "visibility_occluding": [],
        "tracking_shell_near": [],
        "background_only": [],
    }
    for obstacle in static.values():
        name = obstacle["modelName"]
        radius = float(obstacle["radius"])
        route_surface, _ = minimum_route_static_clearance(
            route, {"obstacle": obstacle}
        )
        shell_distance = float("inf")
        samples = max(1, int(math.ceil(route.length / 0.40)))
        for index in range(samples + 1):
            target, tangent = route.sample(route.length * index / float(samples))
            side = right_normal(tangent)
            for offset in offsets:
                nominal = add(target, offset[:2])
                for sign in (-1.0, 0.0, 1.0):
                    point = add(nominal, scale(side, sign * side_offset))
                    shell_distance = min(
                        shell_distance,
                        distance(point, obstacle["centerENU"]) - radius,
                    )
        if shell_distance <= planner_clearance + 0.35:
            result["motion_affecting"].append(name)
        if shell_distance <= planner_clearance + 1.50 or route_surface <= 4.5:
            result["tracking_shell_near"].append(name)
        if obstacle_occludes_tracking_shell(
                route, obstacle, offsets, visibility_margin, side_offset):
            result["visibility_occluding"].append(name)
        if (name not in result["motion_affecting"] and
                name not in result["visibility_occluding"] and
                name not in result["tracking_shell_near"]):
            result["background_only"].append(name)
    return {
        key + "_count": len(value) for key, value in result.items()
    }


def candidate_paths_at_station(center, tangent, offsets, side_offset,
                               samples=81, length=4.0):
    side_direction = right_normal(tangent)
    result = []
    for offset in offsets:
        candidates = {}
        for name, sign in (("NOMINAL", 0.0), ("PLUS", 1.0), ("MINUS", -1.0)):
            points = []
            for index in range(samples):
                u = index / float(samples - 1)
                target = add(center, scale(tangent, (u - 0.5) * length))
                lateral = scale(side_direction, sign * side_offset * math.sin(math.pi * u))
                points.append(add(add(target, offset[:2]), lateral))
            candidates[name] = points
        result.append(candidates)
    return result


def path_minimum_distance(first, second):
    return min(distance(a, b) for a, b in zip(first, second))


def path_obstacle_clearance(path, obstacles, radius):
    return min(distance(point, obstacle) - radius for point in path for obstacle in obstacles)


def find_topology_conflict(center, tangent, offsets, side_offset,
                           swarm_clearance, swarm_margin, moving_clearance):
    paths = candidate_paths_at_station(center, tangent, offsets, side_offset)
    outer_lateral = max(abs(offsets[0][1]), abs(offsets[2][1]))
    lateral_candidates = [
        outer_lateral + side_offset + delta
        for delta in (0.20, 0.30, 0.40, 0.50)
    ]
    radius_candidates = (0.24, 0.28, 0.32)
    tangent_offset = 0.5 * (offsets[0][0] + offsets[2][0])
    obstacle_x = center[0] + tangent_offset
    names = ("NOMINAL", "PLUS", "MINUS")
    for lateral, radius in itertools.product(lateral_candidates, radius_candidates):
        blockers = [[obstacle_x, center[1] - lateral],
                    [obstacle_x, center[1] + lateral]]
        clearances = []
        local = []
        executable = []
        for drone_index in range(3):
            values = {
                name: path_obstacle_clearance(paths[drone_index][name], blockers, radius)
                for name in names
            }
            clearances.append(values)
            local.append(max(names, key=lambda name: (values[name], name == "NOMINAL")))
            executable.append([
                name for name in names
                if values[name] + radius >= moving_clearance - 1.0e-6
            ])
        local_min = min(
            path_minimum_distance(paths[i][local[i]], paths[j][local[j]])
            for i in range(3) for j in range(i + 1, 3)
        )
        best_combo = None
        best_min = -1.0
        for combo in itertools.product(*executable):
            pair_min = min(
                path_minimum_distance(paths[i][combo[i]], paths[j][combo[j]])
                for i in range(3) for j in range(i + 1, 3)
            )
            if pair_min > best_min:
                best_combo = combo
                best_min = pair_min
        if (local_min < swarm_clearance and
                best_combo is not None and
                best_min > swarm_clearance + swarm_margin):
            return {
                "blocker_lateral_offset_m": lateral,
                "blocker_radius_m": radius,
                "blocker_centers_at_encounter": blockers,
                "local_combination": local,
                "local_pairwise_min_distance_m": local_min,
                "alternative_combination": list(best_combo),
                "alternative_pairwise_min_distance_m": best_min,
                "candidate_obstacle_clearance_m": clearances,
                "world_space_samples": len(paths[0]["NOMINAL"]),
            }
    raise RuntimeError("no topology-conflict station found by geometric search")


def obstacle_with_derivation(name, center, radius, height, derivation):
    return {
        "modelName": name,
        "centerENU": round_vector(center),
        "radius": round(radius, 6),
        "height": round(height, 6),
        "benchmarkDerivation": derivation,
    }


def wall_with_derivation(name, center, length, thickness, height, yaw,
                         derivation):
    return {
        "modelName": name,
        "centerENU": round_vector(center),
        "sizeENU": [round(float(length), 6), round(float(thickness), 6),
                    round(float(height), 6)],
        "yawRad": round(float(yaw), 9),
        "benchmarkDerivation": derivation,
    }


def transverse_occlusion_wall(target, tangent, sight, length,
                              required_route_clearance,
                              visibility_margin):
    """Place a wide wall across a true target-to-viewpoint LOS.

    The long wall axis is the route normal.  Its route-facing edge stays beyond
    the target body + scene margin, while its thin tangent coordinate is placed
    at the actual LOS crossing.  This avoids the previous impossible design of
    centering a wide transverse wall directly on the target route.
    """
    normal = left_normal(tangent)
    normal_projection = dot(sight, normal)
    if abs(normal_projection) <= EPS:
        raise RuntimeError("viewpoint LOS is parallel to target route")
    maximum_occluding_inner = (abs(normal_projection) +
                               visibility_margin - 0.01)
    inner_clearance = max(
        required_route_clearance + 0.02,
        min(required_route_clearance + 0.08, maximum_occluding_inner))
    if abs(normal_projection) + visibility_margin < inner_clearance:
        raise RuntimeError("viewpoint LOS never reaches route-safe wall edge")
    crossing_normal = min(abs(normal_projection), inner_clearance + 0.04)
    fraction = min(1.0, crossing_normal / abs(normal_projection))
    tangent_projection = fraction * dot(sight, tangent)
    signed_center_normal = math.copysign(
        0.5 * length + inner_clearance, normal_projection)
    center = add(target, add(scale(tangent, tangent_projection),
                             scale(normal, signed_center_normal)))
    yaw_before = math.atan2(tangent[1], tangent[0])
    yaw_after = wrap_angle(yaw_before + 0.5 * math.pi)
    return center, yaw_before, yaw_after, fraction, inner_clearance


def moving_obstacle_with_derivation(name, desired_center, outward_axis,
                                    radius, height, amplitude, desired_offset,
                                    crossing_speed, encounter_time,
                                    velocity_sign, derivation):
    axis = unit(outward_axis)
    transverse_speed = math.sqrt(max(EPS, amplitude * amplitude - desired_offset * desired_offset))
    period = 2.0 * math.pi * transverse_speed / crossing_speed
    phase, actual_speed = phase_for_crossing(
        encounter_time, desired_offset, amplitude, period, velocity_sign
    )
    baseline = subtract(desired_center, scale(axis, desired_offset))
    payload = {
        "modelName": name,
        "centerENU": round_vector(baseline),
        "radius": round(radius, 6),
        "height": round(height, 6),
        "axisENU": round_vector(axis),
        "amplitude": round(amplitude, 6),
        "period": round(period, 6),
        "phase": round(phase, 9),
        "benchmarkDerivation": dict(derivation),
    }
    payload["benchmarkDerivation"].update({
        "encounter_time_s": round(encounter_time, 6),
        "desired_center_at_encounter": round_vector(desired_center),
        "desired_axis_offset_m": desired_offset,
        "derived_crossing_speed_mps": round(actual_speed, 6),
        "phase_formula": "asin(offset/amplitude)-2*pi*t/period with velocity branch",
    })
    return payload


def map_bounds(scene):
    safety = scene.get("safety") or {}
    if all(name in safety for name in ("xMin", "xMax", "yMin", "yMax")):
        return [float(safety["xMin"]), float(safety["xMax"]),
                float(safety["yMin"]), float(safety["yMax"])]
    size = (scene.get("egoPlanner") or {}).get("mapSizeENU", [82.0, 28.0, 5.0])
    return [-0.5 * float(size[0]), 0.5 * float(size[0]),
            -0.5 * float(size[1]), 0.5 * float(size[1])]


def obstacle_inside_bounds(obstacle, bounds, moving=False):
    x_min, x_max, y_min, y_max = bounds
    center = obstacle["centerENU"]
    extent = float(obstacle["radius"])
    if moving:
        extent += abs(float(obstacle.get("amplitude", 0.0)))
    return (x_min + extent <= center[0] <= x_max - extent and
            y_min + extent <= center[1] <= y_max - extent)


def wall_inside_bounds(wall, bounds):
    x_min, x_max, y_min, y_max = bounds
    return all(x_min <= corner[0] <= x_max and y_min <= corner[1] <= y_max
               for corner in wall_corners(wall))


def make_avoidance_scene(source, route, speed, defaults, drones, side_offset):
    scene = base_scene(source, route, speed, drones)
    planner = scene["egoPlanner"]
    operational_max_z = float((scene.get("safety") or {}).get("maxHeight", 4.5))
    obstacle_height = max(float(planner.get("obstacleHeight", 3.6)),
                          operational_max_z + 0.50)
    cloud_resolution = float(planner.get("cloudResolution", 0.22))
    obstacle_inflation = float(planner.get("obstacleInflation", 0.35))
    planner_clearance = float(planner.get("plannerObstacleClearance", 0.5))
    swarm_clearance = float(planner.get(
        "swarmClearance", float_default(defaults, "swarm_clearance")
    ))
    moving_clearance = float_default(defaults, "moving_obj_clearance")
    visibility_margin = float_default(defaults, "visibility_occlusion_margin")
    target_body_radius = 0.25
    target_scene_margin = 0.05
    target_static_required_clearance = target_body_radius + target_scene_margin

    station_count = int(clamp(round(route.length / 10.0), 5, 7))
    station_distances = station_arc_lengths(route.length, station_count, 8.0)
    static_radius = max(1.4 * cloud_resolution, 0.85 * obstacle_inflation)
    los_gap = 0.04
    outward_shift = static_radius + visibility_margin + los_gap
    moving_radius = max(0.28, 1.25 * cloud_resolution)
    amplitude = 0.90
    crossing_speed = 0.55
    desired_axis_offset = -0.50
    # Keep the moving hazards close enough to trigger the production dynamic
    # predictor, while separating their full sinusoidal swept volumes from the
    # static station cylinders.  The shift is along the route, not across it,
    # so the intended SIDE/topology decisions remain world-space decisions.
    dynamic_tangent_shift = 0.82
    dynamic_static_margin = 0.08

    static = {}
    stations = []
    for index, arc_length in enumerate(station_distances):
        center, tangent = route.sample(arc_length)
        normal = left_normal(tangent)
        drone_index = 0 if index % 2 == 0 else 2
        preferred = drones[drone_index]["tracking_offset"][:2]
        side_sign = -1.0 if dot(preferred, normal) < 0.0 else 1.0
        outward = scale(normal, side_sign)
        obstacle_center = None
        selected_outward_shift = None
        for extra_shift in (0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40):
            trial_shift = outward_shift + extra_shift
            trial_center = add(add(center, preferred),
                               scale(outward, trial_shift))
            trial = {"candidate": {
                "centerENU": trial_center, "radius": static_radius,
                "modelName": "avoidance-route-clearance-probe"}}
            if minimum_route_static_clearance(route, trial)[0] > \
                    target_static_required_clearance + 0.05:
                obstacle_center = trial_center
                selected_outward_shift = trial_shift
                break
        if obstacle_center is None:
            raise RuntimeError(
                "failed to move avoidance station {} clear of target route".format(
                    index + 1))
        key = str(len(static))
        static[key] = obstacle_with_derivation(
            "avoidance_shell_station_{:02d}".format(index + 1),
            obstacle_center, static_radius, obstacle_height,
            {
                "station": index + 1,
                "route_arc_length_m": round(arc_length, 6),
                "target_center": round_vector(center),
                "targeted_drone": drone_index,
                "preferred_tracking_offset": round_vector(preferred),
                "outward_shift_formula": "radius + visibility_margin + los_gap",
                "outward_shift_m": round(selected_outward_shift, 6),
                "nominal_static_surface_clearance_m": round(outward_shift - static_radius, 6),
                "nominal_los_surface_gap_m": round(los_gap, 6),
                "pressure_role": "tracking_shell_blocker",
            },
        )
        stations.append({
            "station": index + 1,
            "route_arc_length_m": round(arc_length, 6),
            "target_center": round_vector(center),
            "tangent": round_vector(tangent),
            "normal": round_vector(normal),
            "targeted_drone": drone_index,
            "static_obstacle": static[key]["modelName"],
        })

    # Put a small hard point on a current SIDE seed at station 3.  The point is
    # not on the target centerline or nominal LOS, but a static colliding SIDE
    # seed has a tangentially open A* repair around it.
    repair_station_index = min(2, station_count - 1)
    repair = stations[repair_station_index]
    repair_center = repair["target_center"]
    repair_tangent = repair["tangent"]
    repair_normal = repair["normal"]
    repair_drone = repair["targeted_drone"]
    repair_preferred = drones[repair_drone]["tracking_offset"][:2]
    repair_side_direction = right_normal(repair_tangent)
    forced_side_sign = -1.0 if repair_drone == 0 else 1.0
    repair_profile = scale(repair_side_direction, forced_side_sign * 0.25 * side_offset)
    repair_obstacle_center = add(add(repair_center, repair_preferred), repair_profile)
    repair_radius = max(1.15 * cloud_resolution, 0.65 * static_radius)
    static[str(len(static))] = obstacle_with_derivation(
        "avoidance_local_astar_repair_point",
        repair_obstacle_center, repair_radius, obstacle_height,
        {
            "station": repair_station_index + 1,
            "targeted_drone": repair_drone,
            "side_seed_fraction": 0.25,
            "side_offset_from_planner_source_m": side_offset,
            "intent": "SIDE seed intersects locally; tangential detour remains open",
            "pressure_role": "rejoin_blocker",
        },
    )

    topology_index = station_count // 2
    topology_station = stations[topology_index]
    topology = find_topology_conflict(
        topology_station["target_center"], topology_station["tangent"],
        [item["tracking_offset"] for item in drones], side_offset,
        swarm_clearance, float_default(defaults, "temporal_swarm_trigger_margin"),
        moving_clearance,
    )
    topology.update({
        "station": topology_index + 1,
        "route_arc_length_m": topology_station["route_arc_length_m"],
        "target_center": topology_station["target_center"],
    })

    moving = {}
    single_indices = sorted(set((0, 1, repair_station_index, 4,
                                 station_count - 2)))
    for event_index, station_index in enumerate(single_indices):
        station = stations[station_index]
        drone_index = station["targeted_drone"]
        preferred = drones[drone_index]["tracking_offset"][:2]
        normal = station["normal"]
        outward_sign = -1.0 if dot(preferred, normal) < 0.0 else 1.0
        outward = scale(normal, outward_sign)
        encounter = station["route_arc_length_m"] / speed
        obstacle = None
        for tangent_shift, outward_shift in itertools.product(
                (0.82, 0.55, 1.10, 0.25, -0.25, 1.35),
                (0.40, 0.62, 0.84, 1.06)):
            desired_center = add(
                add(add(station["target_center"], preferred),
                    scale(station["tangent"], tangent_shift)),
                scale(outward, outward_shift))
            trial = moving_obstacle_with_derivation(
                "avoidance_dynamic_crossing_{:02d}".format(event_index + 1),
                desired_center, outward, moving_radius, obstacle_height,
                amplitude, desired_axis_offset, crossing_speed, encounter,
                1.0 if event_index % 2 == 0 else -1.0,
                {
                    "station": station_index + 1,
                    "targeted_drone": drone_index,
                    "tangent_shift_m": tangent_shift,
                    "outward_shift_m": outward_shift,
                    "placement": "analytic target/static-safe search",
                })
            target_gap, _ = minimum_target_dynamic_clearance(
                route, speed, {"trial": trial}, target_body_radius,
                target_scene_margin)
            static_gap, _ = minimum_dynamic_static_swept_clearance(
                {"trial": trial}, static, dynamic_static_margin)
            if target_gap > 0.02 and static_gap > 0.02:
                obstacle = trial
                break
        if obstacle is None:
            raise RuntimeError(
                "failed to place target-safe dynamic crossing at station {}".format(
                    station_index + 1))
        moving[str(len(moving))] = obstacle

    # Paired outer blockers create the searched local-inward topology event.
    topology_center = topology_station["target_center"]
    topology_tangent = topology_station["tangent"]
    topology_normal = topology_station["normal"]
    encounter = topology_station["route_arc_length_m"] / speed
    average_outer_tangent = 0.5 * (
        dot(drones[0]["tracking_offset"][:2], topology_tangent) +
        dot(drones[2]["tracking_offset"][:2], topology_tangent)
    )
    for pair_index, lateral_sign in enumerate((-1.0, 1.0)):
        desired_center = add(
            add(add(topology_center,
                    scale(topology_tangent, average_outer_tangent)),
                scale(topology_tangent, dynamic_tangent_shift)),
            scale(topology_normal,
                  lateral_sign * topology["blocker_lateral_offset_m"]),
        )
        outward = scale(topology_normal, lateral_sign)
        moving[str(len(moving))] = moving_obstacle_with_derivation(
            "avoidance_topology_blocker_{}".format("south" if lateral_sign < 0 else "north"),
            desired_center, outward, topology["blocker_radius_m"],
            obstacle_height, amplitude, desired_axis_offset, crossing_speed,
            encounter, 1.0 if pair_index == 0 else -1.0,
            {
                "station": topology_index + 1,
                "topology_conflict_member": True,
                "world_space_local_combination": topology["local_combination"],
                "world_space_alternative_combination": topology["alternative_combination"],
            },
        )

    # A deliberately irregular, mixed-scale stress forest.  Constraint trees
    # alternate sides along the tracking shell, cluster-edge trees create
    # short chicanes, and an outer band preserves bypasses without becoming
    # decorative map-edge density.
    forest_target_count = 48
    forest_added = 0
    forest_columns = 16
    for column in range(forest_columns):
        base_arc = 2.5 + (route.length - 5.0) * column / float(
            forest_columns - 1)
        base_arc += (0.31, -0.18, 0.12, -0.27)[column % 4]
        base_point, base_tangent = route.sample(base_arc)
        base_normal = left_normal(base_tangent)
        inner_sign = -1.0 if column % 2 == 0 else 1.0
        for band in ("constraint", "cluster_edge", "outer"):
            if band == "constraint":
                sign = inner_sign
                radius = (0.42, 0.50, 0.58, 0.66)[column % 4]
                base_offset = radius + 1.08 + 0.10 * (column % 3)
            elif band == "cluster_edge":
                sign = -inner_sign
                radius = (0.48, 0.62, 0.78, 0.54)[column % 4]
                base_offset = radius + 1.72 + 0.13 * ((column + 1) % 3)
            else:
                sign = inner_sign if column % 3 else -inner_sign
                radius = (0.28, 0.34, 0.44, 0.82)[column % 4]
                base_offset = radius + 3.55 + 0.18 * ((column + 2) % 3)
            placed = False
            for tangent_shift, outward_shift in itertools.product(
                    (0.0, 0.48, -0.48, 0.96, -0.96, 1.44, -1.44),
                    (0.0, 0.20, 0.40, 0.65)):
                candidate_center = add(
                    add(base_point, scale(base_tangent, tangent_shift)),
                    scale(base_normal, sign * (base_offset + outward_shift)))
                candidate = obstacle_with_derivation(
                    "avoidance_forest_{:02d}".format(forest_added + 1),
                    candidate_center, radius, obstacle_height,
                    {
                        "layout": "deterministic_staggered_mixed_scale_stress_forest",
                        "column": column + 1,
                        "route_arc_length_m": round(base_arc, 6),
                        "band": band,
                        "intent": (
                            "alternating motion constraint and local chicane"
                            if band != "outer" else
                            "near-shell outer bypass boundary"
                        ),
                        "pressure_role": (
                            "tracking_shell_blocker" if band == "constraint" else
                            "topology_shaper" if band == "cluster_edge" else
                            "background_forest"
                        ),
                    },
                )
                trial = {"candidate": candidate}
                swept_clearance, _ = minimum_dynamic_static_swept_clearance(
                    moving, trial, dynamic_static_margin
                )
                if swept_clearance <= 0.0:
                    continue
                if min(distance(candidate["centerENU"], item["centerENU"]) -
                       float(candidate["radius"]) - float(item["radius"])
                       for item in static.values()) <= 0.16:
                    continue
                if minimum_route_static_clearance(route, trial)[0] <= \
                        target_static_required_clearance:
                    continue
                static[str(len(static))] = candidate
                forest_added += 1
                placed = True
                break
            if not placed:
                raise RuntimeError(
                    "failed to place avoidance forest column={} band={}".format(
                        column + 1, band
                    )
                )
    if forest_added != forest_target_count:
        raise RuntimeError(
            "failed to place avoidance forest: added={}/{}".format(
                forest_added, forest_target_count
            )
        )

    scene["obstacleData"] = static
    scene["movingObstacleData"] = moving
    scene["nMovingObstacle"] = len(moving)

    route_static_clearance, route_static_name = minimum_route_static_clearance(route, static)
    route_static_hits = target_static_intersections(
        route, static, {}, target_static_required_clearance)
    dynamic_clearance, dynamic_record = minimum_target_dynamic_clearance(
        route, speed, moving, target_body_radius, target_scene_margin
    )
    swept_clearance, swept_record = minimum_dynamic_static_swept_clearance(
        moving, static, dynamic_static_margin
    )
    dynamic_pair_clearance, dynamic_pair_record = minimum_dynamic_pair_clearance(
        moving, route.length / speed
    )
    initial_static_clearance, initial_static_record = minimum_initial_static_clearance(
        drones, static
    )
    static_pair_clearance, static_pair_record = minimum_static_pair_clearance(static)
    pressure = pressure_classification(
        route, static, drones, planner_clearance, visibility_margin, side_offset
    )
    pressure_roles = {
        role: sum(item.get("benchmarkDerivation", {}).get("pressure_role") == role
                  for item in static.values())
        for role in ("tracking_shell_blocker", "topology_shaper",
                     "rejoin_blocker", "background_forest")
    }
    route_segment_pressure = []
    segment_count = 8
    for segment_index in range(segment_count):
        first_s = route.length * segment_index / float(segment_count)
        second_s = route.length * (segment_index + 1) / float(segment_count)
        first, _ = route.sample(first_s)
        second, _ = route.sample(second_s)
        nearby = [item["modelName"] for item in static.values()
                  if segment_point_distance(first, second, item["centerENU"]) -
                  float(item["radius"]) <= 4.5]
        route_segment_pressure.append({
            "segment": segment_index + 1,
            "arc_length_range_m": [round(first_s, 6), round(second_s, 6)],
            "tracking_shell_static_count": len(nearby),
        })
    bounds = map_bounds(scene)
    all_in_bounds = (
        all(obstacle_inside_bounds(item, bounds) for item in static.values()) and
        all(obstacle_inside_bounds(item, bounds, moving=True) for item in moving.values())
    )
    height_covers_flight = all(
        float(item.get("height", 0.0)) >= operational_max_z + 0.20
        for item in list(static.values()) + list(moving.values())
    )

    interaction_count = 0
    open_bypass_count = 0
    closed_bypass_stations = []
    for station in stations:
        target = station["target_center"]
        drone = station["targeted_drone"]
        preferred = drones[drone]["tracking_offset"][:2]
        nominal = add(target, preferred)
        nearby = [item for item in static.values()
                  if distance(item["centerENU"], nominal) < 2.0]
        if nearby and min(distance(nominal, item["centerENU"]) - item["radius"]
                          for item in nearby) < planner_clearance:
            interaction_count += 1
        tangent = station["tangent"]
        side = right_normal(tangent)
        probes = [add(nominal, scale(side, sign * 1.6 * side_offset))
                  for sign in (-1.0, 1.0)]
        if any(all(distance(probe, item["centerENU"]) - item["radius"] >= planner_clearance
                   for item in nearby) for probe in probes):
            open_bypass_count += 1
        else:
            closed_bypass_stations.append(station["station"])

    checks = {
        "target_centerline_static_free":
            route_static_clearance > target_static_required_clearance,
        "target_centerline_min_static_surface_clearance_m": round(route_static_clearance, 6),
        "target_body_radius_m": target_body_radius,
        "target_scene_margin_m": target_scene_margin,
        "target_route_residual_margin_m": round(
            route_static_clearance - target_static_required_clearance, 6),
        "target_static_intersections": len(route_static_hits),
        "target_static_intersection_details": route_static_hits,
        "closest_static_obstacle_to_target_route": route_static_name,
        "target_dynamic_trajectory_free": dynamic_clearance > 0.0,
        "target_min_dynamic_safety_clearance_m": round(dynamic_clearance, 6),
        "closest_target_dynamic_event": dynamic_record,
        "dynamic_static_swept_volume_free": swept_clearance > 0.0,
        "dynamic_static_swept_margin_m": dynamic_static_margin,
        "minimum_dynamic_static_swept_clearance_m": round(swept_clearance, 6),
        "closest_dynamic_static_sweep": swept_record,
        "dynamic_pair_no_unreasonable_overlap": dynamic_pair_clearance > 0.0,
        "minimum_dynamic_pair_clearance_m": round(dynamic_pair_clearance, 6),
        "closest_dynamic_pair_event": dynamic_pair_record,
        "uav_initial_positions_static_free": initial_static_clearance > 0.05,
        "minimum_uav_initial_static_clearance_m": round(initial_static_clearance, 6),
        "closest_uav_initial_static": initial_static_record,
        "minimum_static_pair_clearance_m": round(static_pair_clearance, 6),
        "closest_static_pair": static_pair_record,
        "radius_distribution": radius_distribution(static),
        "effective_pressure_classification": pressure,
        "pressure_role_counts": pressure_roles,
        "route_segment_static_pressure": route_segment_pressure,
        "tracking_shell_interaction_stations": interaction_count,
        "open_bypass_stations": open_bypass_count,
        "all_obstacles_inside_map_bounds": all_in_bounds,
        "obstacle_height_covers_flight": height_covers_flight,
        "operational_uav_max_z_m": operational_max_z,
        "interaction_obstacles_below_flight_ceiling": sum(
            float(item.get("height", 0.0)) <= operational_max_z + 0.20
            for item in list(static.values()) + list(moving.values())
        ),
        "topology_conflict_found": True,
    }
    if not all((checks["target_centerline_static_free"],
                checks["target_static_intersections"] == 0,
                checks["target_dynamic_trajectory_free"],
                checks["dynamic_static_swept_volume_free"],
                checks["dynamic_pair_no_unreasonable_overlap"],
                checks["uav_initial_positions_static_free"],
                checks["all_obstacles_inside_map_bounds"],
                checks["obstacle_height_covers_flight"],
                checks["topology_conflict_found"])):
        raise RuntimeError("avoidance geometry self-check failed: {}".format(checks))
    if interaction_count != station_count or open_bypass_count != station_count:
        raise RuntimeError(
            "avoidance station interaction/open-bypass contract failed: "
            "interactions={}/{} open_bypass={}/{} closed={}".format(
                interaction_count, station_count, open_bypass_count, station_count,
                closed_bypass_stations,
            )
        )

    scene["benchmarkDesign"] = {
        "generator": GENERATOR_ID,
        "benchmark": "cooperative_avoidance",
        "source_scene": source.name,
        "route_policy": "preserve every waypoint of the production forest route",
        "station_count": station_count,
        "station_spacing_m": round(station_distances[1] - station_distances[0], 6),
        "static_stations": stations,
        "dynamic_crossing_station_count": len(single_indices) + 1,
        "dynamic_obstacle_count": len(moving),
        "static_obstacle_count": len(static),
        "background_forest_count": forest_added,
        "map_area_m2": round((bounds[1] - bounds[0]) *
                             (bounds[3] - bounds[2]), 6),
        "static_density_per_m2": round(
            len(static) / ((bounds[1] - bounds[0]) * (bounds[3] - bounds[2])), 9
        ),
        "astar_repair_station": repair_station_index + 1,
        "topology_conflict_station": topology,
        "formulae": {
            "static_radius": "max(1.4*cloud_resolution, 0.85*obstacle_inflation)",
            "static_center": "target_station + preferred_tracking_offset + outward_shift*route_normal",
            "dynamic_phase": "analytic sinusoid phase at route_distance/target_speed",
            "dynamic_tangent_shift": "0.82 m along route to separate moving swept volumes from statics",
            "background_forest": "deterministic staggered inner/outer bands with clearance filtering",
            "side_seed": "planner_source_side_offset*sin(pi*u)*world_right_normal",
        },
        "productionParameters": {
            "tracking_offsets": [round_vector(item["tracking_offset"]) for item in drones],
            "tracking_radii_m": [round(norm(item["tracking_offset"][:2]), 6) for item in drones],
            "preferred_bearings_deg": [
                round(math.degrees(math.atan2(item["tracking_offset"][1],
                                              item["tracking_offset"][0])), 6)
                for item in drones
            ],
            "side_offset_m": side_offset,
            "planner_static_clearance_m": planner_clearance,
            "moving_clearance_m": moving_clearance,
            "swarm_clearance_m": swarm_clearance,
            "visibility_occlusion_margin_m": visibility_margin,
        },
        "selfCheck": checks,
    }
    return scene


def ray_visible(target, bearing, radius, obstacles, occlusion_margin, walls=()):
    endpoint = add(target, [radius * math.cos(bearing), radius * math.sin(bearing)])
    cylinder_clear = not any(
        segment_intersects_circle(
            target, endpoint, obstacle["centerENU"],
            float(obstacle["radius"]) + occlusion_margin,
        )
        for obstacle in obstacles
    )
    wall_clear = not any(
        segment_wall_clearance(target, endpoint, wall) <= occlusion_margin
        for wall in walls
    )
    return cylinder_clear and wall_clear


def free_viewing_sectors(target, tracking_radius, obstacles, occlusion_margin,
                         sample_step_deg=1.0, walls=()):
    count = int(round(360.0 / sample_step_deg))
    visible = [
        ray_visible(target, math.radians(index * sample_step_deg),
                    tracking_radius, obstacles, occlusion_margin, walls)
        for index in range(count)
    ]
    if all(visible):
        return [{"start_deg": 0.0, "end_deg": 360.0, "width_deg": 360.0}], 1.0
    if not any(visible):
        return [], 0.0
    start_block = next(index for index, value in enumerate(visible) if not value)
    ordered = [visible[(start_block + 1 + index) % count] for index in range(count)]
    sectors = []
    start = None
    for index, value in enumerate(ordered + [False]):
        if value and start is None:
            start = index
        if not value and start is not None:
            first_index = (start_block + 1 + start) % count
            length = index - start
            start_deg = first_index * sample_step_deg
            sectors.append({
                "start_deg": round(start_deg, 6),
                "end_deg": round((start_deg + length * sample_step_deg) % 360.0, 6),
                "width_deg": round(length * sample_step_deg, 6),
            })
            start = None
    return sectors, sum(visible) / float(count)


def recoverable_viewpoints(target, tracking_radius, obstacles, occlusion_margin,
                           planner_clearance, separation_deg, walls=()):
    candidates = []
    for degree in range(360):
        bearing = math.radians(degree)
        point = add(target, [tracking_radius * math.cos(bearing),
                             tracking_radius * math.sin(bearing)])
        if not ray_visible(target, bearing, tracking_radius, obstacles,
                           occlusion_margin, walls):
            continue
        if primitive_point_clearance(point, obstacles, walls) < planner_clearance:
            continue
        candidates.append(degree)
    for first, second in itertools.combinations(candidates, 2):
        separation = min(abs(first - second), 360 - abs(first - second))
        if separation >= separation_deg:
            return True, [first, second], len(candidates) / 360.0
    return False, [], len(candidates) / 360.0


def make_visibility_scene(source, route, speed, defaults, drones, side_offset):
    scene = base_scene(source, route, speed, drones)
    planner = scene["egoPlanner"]
    operational_max_z = float((scene.get("safety") or {}).get("maxHeight", 4.5))
    obstacle_height = max(float(planner.get("obstacleHeight", 3.6)),
                          operational_max_z + 0.50)
    planner_clearance = float(planner.get("plannerObstacleClearance", 0.5))
    cloud_resolution = float(planner.get("cloudResolution", 0.22))
    moving_clearance = float_default(defaults, "moving_obj_clearance")
    candidate_trigger_margin = float_default(defaults, "candidate_trigger_margin")
    visibility_margin = float_default(defaults, "visibility_occlusion_margin")
    separation_deg = float_default(defaults, "team_visibility_preferred_separation_deg")
    tracking_radii = [norm(item["tracking_offset"][:2]) for item in drones]
    tracking_radius = sum(tracking_radii) / len(tracking_radii)
    preferred = [
        math.atan2(item["tracking_offset"][1], item["tracking_offset"][0])
        for item in drones
    ]
    target_body_radius = 0.25
    target_scene_margin = 0.05
    target_static_required_clearance = target_body_radius + target_scene_margin

    station_count = 7
    station_distances = station_arc_lengths(route.length, station_count, 8.0)
    # A static obstacle capable of blocking UAV1's preferred ray would lie on
    # the straight target route itself.  Use the two outer viewpoints for
    # physically route-safe single/double occlusion and keep UAV1 as the
    # reliable K-of-N support view.
    station_blocks = [(2,), (0,), (0,), (0, 2), (0,), (0,), (0, 2)]
    # The two double-occlusion stations are candidate-decision stations.  The
    # alternative SIDE is the one moving away from the stationary risk-band
    # enabler below.  SIDE names remain diagnostics; all LOS checks use the
    # resulting world-space point.
    decision_specs = {
        3: {"decision_uav": 0, "other_occluded_uav": 2,
            "support_visible_uav": 1, "local_candidate": "NOMINAL",
            "alternative_candidate": "MINUS"},
        6: {"decision_uav": 2, "other_occluded_uav": 0,
            "support_visible_uav": 1, "local_candidate": "NOMINAL",
            "alternative_candidate": "PLUS"},
    }
    r_occ = min(0.50, 0.35 * min(tracking_radii))
    single_angle_deg = 14.0
    double_angle_deg = 14.0
    radius_single = r_occ * math.sin(math.radians(single_angle_deg) / 2.0)
    radius_double = r_occ * math.sin(math.radians(double_angle_deg) / 2.0)
    # Move the central occluder far enough for a measured physical centerline
    # gap.  The existing LOS margin still covers the 180-degree preferred ray,
    # while the nearest outer preferred bearing remains outside that margin.
    central_line_physical_gap = 0.0
    central_bias_deg = 0.0
    core_target_route_margin = target_static_required_clearance + 0.04
    central_target_route_margin = core_target_route_margin
    core_static_pair_margin = 0.12

    static = {}
    walls = {}
    stations = []
    for index, (arc_length, blocked) in enumerate(zip(station_distances, station_blocks)):
        target, tangent = route.sample(arc_length)
        local_obstacles = []
        local_walls = []
        for member_index, drone_index in enumerate(blocked):
            bearing = preferred[drone_index]
            bias = 0.0
            center_bearing = bearing + bias
            original_center = add(
                target, [r_occ * math.cos(center_bearing),
                         r_occ * math.sin(center_bearing)])
            original_radius = 0.065
            # The original 0.5 m radial placement put every core center less
            # than 0.247 m from the straight target route.  A normal physical
            # radius therefore cannot remain at that center.  Preserve the
            # exact LOS bearing and move only radially outward until the
            # cylinder has an explicit scene-geometry route margin.
            if index in decision_specs:
                wall_length = 2.40
                wall_center, yaw_before, yaw_after, los_fraction, inner_clearance = \
                    transverse_occlusion_wall(
                        target, tangent,
                        drones[drone_index]["tracking_offset"][:2],
                        wall_length, target_static_required_clearance,
                        visibility_margin)
                wall = wall_with_derivation(
                    "visibility_station_{:02d}_screen_uav{}".format(
                        index + 1, drone_index),
                    wall_center, wall_length, 0.20, obstacle_height, yaw_after,
                    {
                        "station": index + 1,
                        "blocked_drone": drone_index,
                        "target_center": round_vector(target),
                        "preferred_bearing_deg": round(math.degrees(bearing), 6),
                        "original_center": round_vector(original_center),
                        "original_radius_m": original_radius,
                        "original_height_m": round(obstacle_height, 6),
                        "core_role": "primary_double_station_wall",
                        "target_route_margin_m": round(inner_clearance, 6),
                        "sight_ray_fraction": round(los_fraction, 6),
                        "replacement": "normal wall replaces route-unsafe thin column",
                        "yaw_before_rad": round(yaw_before, 9),
                        "yaw_after_rad": round(yaw_after, 9),
                        "rotation_fix_rad": round(0.5 * math.pi, 9),
                        "orientation": "transverse_to_target_route_and_los",
                    },
                )
                walls[str(len(walls))] = wall
                local_walls.append(wall)
                continue
            radius = 0.43
            core_role = "single_station_visibility_occluder"
            lateral_factor = abs(dot(unit(drones[drone_index]["tracking_offset"][:2]),
                                     left_normal(tangent)))
            if lateral_factor < 1.0e-6:
                raise RuntimeError("core LOS ray is parallel to target route")
            route_margin = core_target_route_margin
            minimum_center_distance = (radius + route_margin) / lateral_factor
            role_distance = (1.34 if core_role == "primary_double_station_occluder"
                             else 1.55)
            center_distance = max(minimum_center_distance, role_distance)
            center = None
            # Curved-route neighbors can be closer than the local station
            # tangent predicts.  Search only outward on the same LOS ray and,
            # if necessary, minimally shrink this newly added cylinder.
            for trial_radius in (0.43, 0.40, 0.37, 0.34, 0.31):
                minimum_center_distance = (
                    trial_radius + route_margin) / lateral_factor
                for trial_distance in (
                        max(minimum_center_distance, role_distance),
                        max(minimum_center_distance, role_distance) + 0.05,
                        max(minimum_center_distance, role_distance) + 0.10,
                        max(minimum_center_distance, role_distance) + 0.15):
                    trial_center = add(
                        target, [trial_distance * math.cos(center_bearing),
                                 trial_distance * math.sin(center_bearing)])
                    trial = {"candidate": {
                        "centerENU": trial_center, "radius": trial_radius,
                        "modelName": "route-clearance-probe"}}
                    if (minimum_route_static_clearance(route, trial)[0] >
                            target_static_required_clearance + 0.05 and
                            trial_distance <= tracking_radii[drone_index] +
                            trial_radius):
                        radius, center_distance, center = (
                            trial_radius, trial_distance, trial_center)
                        break
                if center is not None:
                    break
            if center is None:
                raise RuntimeError(
                    "no route-safe core occluder at station {}".format(index + 1))
            obstacle = obstacle_with_derivation(
                "visibility_station_{:02d}_block_uav{}".format(index + 1, drone_index),
                center, radius, obstacle_height,
                {
                    "station": index + 1,
                    "blocked_drone": drone_index,
                    "target_center": round_vector(target),
                    "preferred_bearing_deg": round(math.degrees(bearing), 6),
                    "occluder_center_bearing_deg": round(math.degrees(center_bearing), 6),
                    "occluder_distance_from_target_m": round(center_distance, 6),
                    "original_center": round_vector(original_center),
                    "original_radius_m": original_radius,
                    "original_height_m": round(obstacle_height, 6),
                    "core_role": core_role,
                    "target_route_margin_m": route_margin,
                    "desired_physical_occlusion_angle_deg": round(
                        math.degrees(2.0 * math.asin(radius / r_occ)), 6
                    ),
                    "radius_formula": "role radius; center moves radially on unchanged LOS bearing",
                    "central_line_bias_deg": round(math.degrees(bias), 6),
                },
            )
            static[str(len(static))] = obstacle
            local_obstacles.append(obstacle)

        nominal_visible = [
            ray_visible(target, bearing, tracking_radii[drone_index],
                        local_obstacles, visibility_margin, local_walls)
            for drone_index, bearing in enumerate(preferred)
        ]
        sectors, free_fraction = free_viewing_sectors(
            target, tracking_radius, local_obstacles, visibility_margin,
            walls=local_walls
        )
        recoverable, recovery_pair, endpoint_free_fraction = recoverable_viewpoints(
            target, tracking_radius, local_obstacles, visibility_margin,
            planner_clearance, separation_deg, local_walls
        )
        stations.append({
            "station": index + 1,
            "route_arc_length_m": round(arc_length, 6),
            "target_center": round_vector(target),
            "tangent": round_vector(tangent),
            "blocked_preferred_drone_ids": list(blocked),
            "nominal_preferred_visibility": nominal_visible,
            "nominal_visible_count": sum(nominal_visible),
            "free_angular_fraction": round(free_fraction, 6),
            "free_endpoint_fraction": round(endpoint_free_fraction, 6),
            "free_viewing_sectors": sectors,
            "elastic_k2_recoverable": recoverable,
            "example_recovery_bearings_deg": recovery_pair,
        })

        if index in decision_specs:
            spec = dict(decision_specs[index])
            decision_uav = spec["decision_uav"]
            local_kind = spec["local_candidate"]
            nominal_point = add(target, drones[decision_uav]["tracking_offset"][:2])
            def candidate_point(kind):
                if kind == "NOMINAL":
                    return nominal_point
                sign = 1.0 if kind == "PLUS" else -1.0
                return add(nominal_point,
                           scale(right_normal(tangent), sign * side_offset))
            paths = candidate_paths_at_station(
                target, tangent, [item["tracking_offset"] for item in drones],
                side_offset,
            )
            targets = [add(target, scale(tangent, (u / 80.0 - 0.5) * 4.0))
                       for u in range(81)]
            def path_visibility_fraction(drone_id, kind):
                visible = 0
                for target_sample, point in zip(targets, paths[drone_id][kind]):
                    cylinder_blocked = any(segment_intersects_circle(
                        target_sample, point, item["centerENU"],
                        float(item["radius"]) + visibility_margin,
                    ) for item in local_obstacles)
                    wall_blocked = any(
                        segment_wall_clearance(target_sample, point, item) <=
                        visibility_margin for item in local_walls)
                    visible += int(not cylinder_blocked and not wall_blocked)
                return visible / float(len(targets))
            candidate_visibility = {
                kind: path_visibility_fraction(decision_uav, kind)
                for kind in ("NOMINAL", "PLUS", "MINUS")
            }
            candidate_station_visible = {
                kind: primitive_segment_clearance(
                    target, candidate_point(kind), local_obstacles,
                    local_walls) > visibility_margin
                for kind in ("NOMINAL", "PLUS", "MINUS")
            }
            original_alternative = spec["alternative_candidate"]
            alternative = original_alternative
            local_point = candidate_point(local_kind)
            alternative_point = candidate_point(alternative)
            local_visibility = candidate_visibility[local_kind]
            alternative_visibility = candidate_visibility[alternative]
            local_visible = candidate_station_visible[local_kind]
            alternative_visible = candidate_station_visible[alternative]
            other = spec["other_occluded_uav"]
            support = spec["support_visible_uav"]
            other_visible = nominal_visible[other]
            support_visible = nominal_visible[support]
            local_k2 = int(local_visible) + int(other_visible) + int(support_visible) >= 2
            alternative_k2 = (
                int(alternative_visible) + int(other_visible) + int(support_visible) >= 2
            )
            candidate_clearance = {
                kind: primitive_point_clearance(
                    candidate_point(kind), local_obstacles, local_walls)
                for kind in ("NOMINAL", "PLUS", "MINUS")
            }
            candidate_k2 = {
                kind: int(candidate_station_visible[kind]) +
                      int(other_visible) + int(support_visible) >= 2
                for kind in ("NOMINAL", "PLUS", "MINUS")
            }
            # These are simultaneous station clearances.  Sweeping a fixed
            # station obstacle against every point of the local seed would
            # discard time alignment and report false collisions.
            local_clearance = min(
                primitive_point_clearance(local_point, local_obstacles, local_walls),
                float("inf"),
            )
            alternative_clearance = min(
                primitive_point_clearance(
                    alternative_point, local_obstacles, local_walls),
                float("inf"),
            )
            spec.update({
                "station": index + 1,
                "station_position": round_vector(target),
                "original_alternative_candidate": original_alternative,
                "alternative_candidate": alternative,
                "local_candidate_los": "VISIBLE" if local_visible else "BLOCKED",
                "alternative_candidate_los": (
                    "VISIBLE" if alternative_visible else "BLOCKED"
                ),
                "other_uav_los": "VISIBLE" if other_visible else "BLOCKED",
                "support_uav_los": "VISIBLE" if support_visible else "BLOCKED",
                "local_predicted_k2": int(local_k2),
                "alternative_predicted_k2": int(alternative_k2),
                "candidate_los_angular_separation_deg": round(math.degrees(
                    angle_distance(
                        math.atan2(local_point[1] - target[1], local_point[0] - target[0]),
                        math.atan2(alternative_point[1] - target[1],
                                   alternative_point[0] - target[0]),
                    )
                ), 6),
                "local_candidate_visibility_fraction": round(local_visibility, 6),
                "alternative_candidate_visibility_fraction": round(
                    alternative_visibility, 6),
                "candidate_visibility_fraction": {
                    kind: round(value, 6)
                    for kind, value in candidate_visibility.items()
                },
                "candidate_station_visible": {
                    kind: bool(value)
                    for kind, value in candidate_station_visible.items()
                },
                "candidate_predicted_k2": {
                    kind: int(value) for kind, value in candidate_k2.items()
                },
                "local_static_clearance_m": round(local_clearance, 6),
                "alternative_static_clearance_m": round(alternative_clearance, 6),
                "candidate_static_clearance_m": {
                    kind: round(value, 6)
                    for kind, value in candidate_clearance.items()
                },
                "local_static_feasible": local_clearance >= planner_clearance,
                "alternative_static_feasible": alternative_clearance >= planner_clearance,
                "candidate_space_exists": any(
                    value >= planner_clearance
                    for value in candidate_clearance.values()),
                "visibility_difference": round(
                    max(candidate_visibility.values()) -
                    min(candidate_visibility.values()), 6),
                "occlusion_role_preserved": (
                    len(set(candidate_station_visible.values())) > 1 and
                    any(candidate_k2.values()) and
                    not all(candidate_k2.values())
                ),
                "occluders": [
                    {"type": "cylinder", "center": item["centerENU"],
                     "radius": item["radius"]}
                    for item in local_obstacles
                ] + [
                    {"type": "wall", "center": item["centerENU"],
                     "size": item["sizeENU"], "yaw": item["yawRad"]}
                    for item in local_walls
                ],
            })
            stations[-1]["k2Decision"] = spec

    double_stations = [item for item in stations
                       if len(item["blocked_preferred_drone_ids"]) == 2]
    core_cylinder_count = len(static)
    core_wall_count = len(walls)
    core_occluder_names = (
        [item["modelName"] for item in static.values()] +
        [item["modelName"] for item in walls.values()]
    )
    core_occluder_centers = {
        item["modelName"]: list(item["centerENU"]) for item in static.values()
    }
    core_occluder_centers.update({
        item["modelName"]: list(item["centerENU"]) for item in walls.values()
    })
    core_geometry = []
    for item in static.values():
        derivation = item["benchmarkDerivation"]
        target = derivation["target_center"]
        before = derivation["original_center"]
        after = item["centerENU"]
        bearing_before = math.degrees(math.atan2(
            before[1] - target[1], before[0] - target[0]))
        bearing_after = math.degrees(math.atan2(
            after[1] - target[1], after[0] - target[0]))
        core_geometry.append({
            "id": item["modelName"],
            "role": derivation["core_role"],
            "center_before": before,
            "center_after": after,
            "los_bearing_before_deg": round(bearing_before, 6),
            "los_bearing_after_deg": round(bearing_after, 6),
            "radius_before_m": derivation["original_radius_m"],
            "radius_after_m": item["radius"],
            "height_before_m": derivation["original_height_m"],
            "height_after_m": item["height"],
            "target_route_margin_after_m": round(
                abs(after[1]) - float(item["radius"]), 6),
        })
    for item in walls.values():
        derivation = item["benchmarkDerivation"]
        target = derivation["target_center"]
        before = derivation["original_center"]
        after = item["centerENU"]
        bearing_before = math.degrees(math.atan2(
            before[1] - target[1], before[0] - target[0]))
        bearing_after = math.degrees(math.atan2(
            after[1] - target[1], after[0] - target[0]))
        core_geometry.append({
            "id": item["modelName"],
            "role": derivation["core_role"],
            "primitive": "wall",
            "center_before": before,
            "center_after": after,
            "los_bearing_before_deg": round(bearing_before, 6),
            "los_bearing_after_deg": round(bearing_after, 6),
            "radius_before_m": derivation["original_radius_m"],
            "size_after_m": item["sizeENU"],
            "height_before_m": derivation["original_height_m"],
            "height_after_m": item["sizeENU"][2],
            "target_route_margin_after_m": round(
                min(segment_wall_clearance(first, second, item)
                    for first, second in zip(route.points, route.points[1:])), 6),
        })

    # Route-transverse screens are offset to one side of the target route.  Each
    # blocks the true world-space LOS of one outer UAV while leaving its
    # outward SIDE candidate and the other observation directions available.
    wall_candidate_checks = []
    screen_specs = ((0, 0), (2, 1), (3, 0), (5, 2))
    for screen_index, (gap_index, drone_index) in enumerate(screen_specs):
        arc_length = 0.5 * (
            station_distances[gap_index] + station_distances[gap_index + 1]
        )
        target, tangent = route.sample(arc_length)
        sight = drones[drone_index]["tracking_offset"][:2]
        wall_length = 2.40
        center, yaw_before, yaw_after, los_fraction, inner_clearance = \
            transverse_occlusion_wall(
                target, tangent, sight, wall_length,
                target_static_required_clearance, visibility_margin)
        wall = wall_with_derivation(
            "visibility_offset_screen_{:02d}".format(screen_index + 1),
            center, wall_length, 0.20, obstacle_height, yaw_after,
            {
                "between_stations": [gap_index + 1, gap_index + 2],
                "targeted_drone": drone_index,
                "role": ("short_offset_screen" if screen_index == 0 else
                         "staggered_offset_screen"),
                "sight_ray_fraction": round(los_fraction, 6),
                "target_route_inner_clearance_m": round(inner_clearance, 6),
                "orientation": "transverse_to_target_route_and_los",
                "pressure_role": "visibility_screen",
                "yaw_before_rad": round(yaw_before, 9),
                "yaw_after_rad": round(yaw_after, 9),
                "rotation_fix_rad": round(0.5 * math.pi, 9),
            },
        )
        route_gap = min(segment_wall_clearance(first, second, wall)
                        for first, second in zip(route.points, route.points[1:]))
        cylinder_gap = min(
            (point_wall_clearance(item["centerENU"], wall) - float(item["radius"])
             for item in static.values()), default=float("inf"))
        if route_gap <= target_static_required_clearance or cylinder_gap <= 0.12:
            raise RuntimeError(
                "visibility wall placement invalid: wall={} route_gap={} cylinder_gap={}".format(
                    wall["modelName"], route_gap, cylinder_gap))
        walls[str(len(walls))] = wall

        candidate_visible = {}
        candidate_clearance = {}
        nominal = add(target, drones[drone_index]["tracking_offset"][:2])
        for kind, sign in (("NOMINAL", 0.0), ("PLUS", 1.0), ("MINUS", -1.0)):
            point = add(nominal, scale(right_normal(tangent), sign * side_offset))
            candidate_visible[kind] = primitive_segment_clearance(
                target, point, static.values(), walls.values()) > visibility_margin
            candidate_clearance[kind] = primitive_point_clearance(
                point, static.values(), walls.values())
        executable_visible = [
            kind for kind in ("NOMINAL", "PLUS", "MINUS")
            if candidate_visible[kind] and
            candidate_clearance[kind] >= planner_clearance
        ]
        check = {
            "wall": wall["modelName"],
            "between_stations": [gap_index + 1, gap_index + 2],
            "targeted_drone": drone_index,
            "candidate_los_visible": candidate_visible,
            "candidate_static_clearance_m": {
                kind: round(value, 6)
                for kind, value in candidate_clearance.items()
            },
            "nominal_los_blocked": not candidate_visible["NOMINAL"],
            "executable_visible_candidates": executable_visible,
            "candidate_dependent": len(set(candidate_visible.values())) > 1,
            "target_route_clearance_m": round(route_gap, 6),
        }
        wall_candidate_checks.append(check)
        if (not check["nominal_los_blocked"] or
                not check["candidate_dependent"] or
                not executable_visible):
            raise RuntimeError("wall is not candidate-dependent: {}".format(check))

    # Feature 2 ranks a candidate set; with no dynamic risk the current
    # production chain correctly produces only a NOMINAL singleton.  Add one
    # small stationary risk-band probe at each double-occlusion station.  Each
    # probe is outside the nominal tracking shell, is farther than the hard
    # moving clearance, but lies inside clearance+trigger_margin.  It creates
    # a short NOMINAL/SIDE choice without becoming a collision obstacle.
    moving = {}
    enabler_distance = moving_clearance + 0.5 * candidate_trigger_margin
    enabler_radius = max(0.08, 0.35 * cloud_resolution)
    for enabler_index, station in enumerate(double_stations):
        drone_index = 0 if enabler_index == 0 else 2
        preferred_offset = drones[drone_index]["tracking_offset"][:2]
        tangent = station["tangent"]
        normal = left_normal(tangent)
        outward_sign = -1.0 if dot(preferred_offset, normal) < 0.0 else 1.0
        nominal_center = add(station["target_center"], preferred_offset)
        center = None
        for tangent_shift in (0.0, 0.15, -0.15, 0.30, -0.30):
            trial_center = add(
                add(nominal_center,
                    scale(normal, outward_sign * enabler_distance)),
                scale(tangent, tangent_shift))
            wall_gap = min(
                (point_wall_clearance(trial_center, wall) - enabler_radius
                 for wall in walls.values()), default=float("inf"))
            route_gap = primitive_point_clearance(
                trial_center, static.values(), ())
            if (wall_gap > 0.05 and route_gap > 0.05 and
                    math.hypot(enabler_distance, tangent_shift) <=
                    moving_clearance + candidate_trigger_margin):
                center = trial_center
                break
        if center is None:
            raise RuntimeError("failed to place wall-safe visibility enabler")
        moving[str(enabler_index)] = {
            "modelName": "visibility_candidate_enabler_{:02d}".format(enabler_index + 1),
            "centerENU": round_vector(center),
            "radius": round(enabler_radius, 6),
            "height": round(obstacle_height, 6),
            "axisENU": round_vector(normal),
            "amplitude": 0.0,
            "period": 10.0,
            "phase": 0.0,
            "benchmarkDerivation": {
                "station": station["station"],
                "targeted_drone": drone_index,
                "role": "low-interference candidate-set enabler for production K-of-N ranking",
                "nominal_center_distance_m": round(enabler_distance, 6),
                "hard_moving_clearance_m": moving_clearance,
                "candidate_trigger_distance_m": round(
                    moving_clearance + candidate_trigger_margin, 6),
                "placement_formula": (
                    "target_station + preferred_offset + "
                    "(moving_clearance + 0.5*trigger_margin)*outward_normal"
                ),
                "outside_tracking_shell": True,
                "stationary_runtime_probe": True,
            },
        }

    # Five time-aligned moving occluders cross distinct outer-UAV sight rays at
    # quarter-gap locations.  Together with the two stationary candidate
    # enablers this raises B from four to seven effective dynamic events.  The
    # quarter-gap phase keeps their full swept volumes clear of the screens at
    # the gap midpoints.
    dynamic_roles = []
    dynamic_specs = ((0, 2), (1, 0), (2, 2), (4, 0), (3, 2))
    for event_index, (gap_index, drone_index) in enumerate(dynamic_specs):
        arc_length = (0.75 * station_distances[gap_index] +
                      0.25 * station_distances[gap_index + 1])
        target, tangent = route.sample(arc_length)
        sight = drones[drone_index]["tracking_offset"][:2]
        crossing_axis = tangent
        obstacle = None
        for sight_fraction, tangent_shift in itertools.product(
                (0.72, 0.82, 0.90, 0.98, 1.06),
                (0.0, 0.20, -0.20, 0.40, -0.40,
                 0.60, -0.60, 0.80, -0.80, 1.00, -1.00)):
            desired_center = add(
                add(target, scale(sight, sight_fraction)),
                scale(tangent, tangent_shift))
            trial = moving_obstacle_with_derivation(
                "visibility_dynamic_los_crossing_{:02d}".format(event_index + 1),
                desired_center, crossing_axis, 0.16, obstacle_height,
                0.50, 0.0, 0.40, arc_length / speed,
                1.0 if event_index == 0 else -1.0,
                {
                    "between_stations": [gap_index + 1, gap_index + 2],
                    "targeted_drone": drone_index,
                    "role": "LOS crossing + candidate-specific tracking-shell pressure",
                    "sight_ray_fraction": sight_fraction,
                    "tangent_shift_m": tangent_shift,
                })
            target_gap, _ = minimum_target_dynamic_clearance(
                route, speed, {"trial": trial}, 0.25, 0.05)
            cylinder_gap, _ = minimum_dynamic_static_swept_clearance(
                {"trial": trial}, static, 0.05)
            wall_gap, _ = minimum_dynamic_wall_swept_clearance(
                {"trial": trial}, walls, 0.05)
            if target_gap > 0.02 and cylinder_gap > 0.02 and wall_gap > 0.02:
                obstacle = trial
                break
        if obstacle is None:
            raise RuntimeError(
                "failed to place target-safe dynamic LOS crossing {}".format(
                    event_index + 1))
        moving[str(len(moving))] = obstacle
        dynamic_roles.append({
            "model": obstacle["modelName"],
            "role": obstacle["benchmarkDerivation"]["role"],
            "targeted_drone": drone_index,
        })

    # Two compact three-cylinder clusters complement the nine-cylinder core.
    # Their normal radii broaden LOS obstruction, but the small count keeps B
    # visibly and behaviorally distinct from the avoidance forest.
    supplemental_target = 6
    supplemental_added = 0
    station_centers = [item["target_center"] for item in stations]
    bounds = map_bounds(scene)
    supplemental_gaps = (1, 4)
    for group_index, gap_index in enumerate(supplemental_gaps):
        arc_length = 0.5 * (
            station_distances[gap_index] + station_distances[gap_index + 1]
        )
        target, tangent = route.sample(arc_length)
        primary_sign = 1.0 if group_index % 2 == 0 else -1.0
        for band in ("los_companion", "cluster_shoulder", "cluster_outer"):
            if band == "los_companion":
                radius = 0.30
                base_center = add(
                    add(target, scale(tangent, -1.00)),
                    scale(left_normal(tangent), primary_sign *
                          (radius + target_static_required_clearance + 0.05)))
            elif band == "cluster_shoulder":
                radius = 0.38
                base_center = add(
                    add(target, scale(tangent, -1.50)),
                    scale(left_normal(tangent), primary_sign * 1.45))
            else:
                radius = 0.50
                base_center = add(
                    add(target, scale(tangent, 0.40)),
                    scale(left_normal(tangent), primary_sign * 2.30))
            placed = False
            for tangent_shift, outward_shift in itertools.product(
                    (0.0, 0.30, -0.30, 0.60, -0.60),
                    (0.0, 0.12, 0.24)):
                normal = left_normal(tangent)
                candidate = obstacle_with_derivation(
                    "visibility_cluster_{:02d}".format(supplemental_added + 1),
                    add(add(base_center, scale(tangent, tangent_shift)),
                        scale(normal, primary_sign * outward_shift)),
                    radius, obstacle_height,
                    {
                        "layout": "compact_three_cylinder_visibility_cluster",
                        "between_stations": [gap_index + 1, gap_index + 2],
                        "band": band,
                        "intent": "stable finite-width LOS and viewpoint perturbation",
                        "pressure_role": "cylinder_cluster",
                    },
                )
                trial = {"candidate": candidate}
                if not obstacle_inside_bounds(candidate, bounds):
                    continue
                if minimum_route_static_clearance(route, trial)[0] <= \
                        target_static_required_clearance:
                    continue
                if min(distance(candidate["centerENU"], item["centerENU"]) -
                       float(candidate["radius"]) - float(item["radius"])
                       for item in static.values()) <= 0.14:
                    continue
                swept, _ = minimum_dynamic_static_swept_clearance(
                    moving, trial, 0.05
                )
                if swept <= 0.0:
                    continue
                if min((point_wall_clearance(candidate["centerENU"], wall) - radius
                        for wall in walls.values()), default=float("inf")) <= 0.12:
                    continue
                # Preserve the analytically designed K2 stations; the added
                # forest supplies pressure between them rather than erasing
                # their local candidate choice.
                if any(distance(candidate["centerENU"], center) - radius < 0.90
                       for center in station_centers):
                    continue
                static[str(len(static))] = candidate
                supplemental_added += 1
                placed = True
                break
            if not placed:
                raise RuntimeError(
                    "failed to place visibility cluster group={} band={}".format(
                        group_index + 1, band
                    )
                )
    if supplemental_added != supplemental_target:
        raise RuntimeError(
            "failed to place visibility cluster cylinders: added={}/{}".format(
                supplemental_added, supplemental_target
            )
        )

    scene["obstacleData"] = static
    scene["wallData"] = walls
    scene["movingObstacleData"] = moving
    scene["nMovingObstacle"] = len(moving)

    cylinder_route_clearance, cylinder_route_name = minimum_route_static_clearance(
        route, static)
    wall_route_clearance, wall_route_name = minimum_route_wall_clearance(route, walls)
    route_static_hits = target_static_intersections(
        route, static, walls, target_static_required_clearance)
    if wall_route_clearance < cylinder_route_clearance:
        route_static_clearance, route_static_name = wall_route_clearance, wall_route_name
        route_static_type = "wall"
    else:
        route_static_clearance, route_static_name = (
            cylinder_route_clearance, cylinder_route_name)
        route_static_type = "cylinder"
    all_in_bounds = (
        all(obstacle_inside_bounds(item, bounds) for item in static.values()) and
        all(wall_inside_bounds(item, bounds) for item in walls.values())
    )
    moving_in_bounds = all(
        obstacle_inside_bounds(item, bounds, moving=True) for item in moving.values()
    )
    target_dynamic_clearance, target_dynamic_record = minimum_target_dynamic_clearance(
        route, speed, moving, 0.25, 0.05
    )
    cylinder_swept_clearance, cylinder_swept_record = minimum_dynamic_static_swept_clearance(
        moving, static, 0.05
    )
    wall_swept_clearance, wall_swept_record = minimum_dynamic_wall_swept_clearance(
        moving, walls, 0.05)
    if wall_swept_clearance < cylinder_swept_clearance:
        swept_clearance, swept_record = wall_swept_clearance, wall_swept_record
        swept_static_type = "wall"
    else:
        swept_clearance, swept_record = (
            cylinder_swept_clearance, cylinder_swept_record)
        swept_static_type = "cylinder"
    dynamic_pair_clearance, dynamic_pair_record = minimum_dynamic_pair_clearance(
        moving, route.length / speed
    )
    cylinder_initial_clearance, cylinder_initial_record = minimum_initial_static_clearance(
        drones, static
    )
    wall_initial_clearance, wall_initial_record = minimum_initial_wall_clearance(
        drones, walls)
    if wall_initial_clearance < cylinder_initial_clearance:
        initial_static_clearance, initial_static_record = (
            wall_initial_clearance, wall_initial_record)
    else:
        initial_static_clearance, initial_static_record = (
            cylinder_initial_clearance, cylinder_initial_record)
    static_pair_clearance, static_pair_record = \
        minimum_static_primitive_pair_clearance(static, walls)
    core_only = {str(index): static[str(index)]
                 for index in range(core_cylinder_count)}
    core_walls_only = {str(index): walls[str(index)]
                       for index in range(core_wall_count)}
    core_pair_clearance, core_pair_record = \
        minimum_static_primitive_pair_clearance(core_only, core_walls_only)
    pressure = pressure_classification(
        route, static, drones, planner_clearance, visibility_margin, side_offset
    )
    blocked_ids = {
        drone_id for station in stations
        for drone_id, visible in enumerate(station["nominal_preferred_visibility"])
        if not visible
    }
    threatened = [item["station"] for item in stations
                  if item["nominal_visible_count"] < 2]
    recoverable = [item["station"] for item in stations
                   if item["elastic_k2_recoverable"]]
    decision_stations = [item["k2Decision"] for item in stations
                         if "k2Decision" in item]
    min_free_fraction = min(item["free_endpoint_fraction"] for item in stations)
    station_spacing = station_distances[1] - station_distances[0]
    checks = {
        "target_centerline_physically_free":
            route_static_clearance > target_static_required_clearance,
        "target_centerline_min_physical_clearance_m": round(route_static_clearance, 6),
        "closest_occluder_to_target_route": route_static_name,
        "closest_target_static_primitive_type": route_static_type,
        "target_body_radius_m": target_body_radius,
        "target_scene_margin_m": target_scene_margin,
        "target_route_residual_margin_m": round(
            route_static_clearance - target_static_required_clearance, 6),
        "target_static_intersections": len(route_static_hits),
        "target_static_intersection_details": route_static_hits,
        "all_preferred_bearings_blocked_somewhere": blocked_ids == {0, 1, 2},
        "all_outer_preferred_bearings_blocked_somewhere":
            {0, 2}.issubset(blocked_ids),
        "central_support_view_preserved": 1 not in blocked_ids,
        "double_occlusion_station_count": len(double_stations),
        "nominal_k2_threatened_stations": threatened,
        "elastic_k2_recoverable_stations": recoverable,
        "all_double_occlusions_k2_recoverable": all(
            item["elastic_k2_recoverable"] for item in double_stations
        ),
        "minimum_free_tracking_shell_fraction": round(min_free_fraction, 6),
        "recovery_spacing_m": round(station_spacing, 6),
        "recovery_spacing_s": round(station_spacing / speed, 6),
        "all_obstacles_inside_map_bounds": all_in_bounds,
        "candidate_enablers_inside_map_bounds": moving_in_bounds,
        "target_dynamic_trajectory_free": target_dynamic_clearance > 0.0,
        "target_min_dynamic_safety_clearance_m": round(target_dynamic_clearance, 6),
        "closest_target_dynamic_event": target_dynamic_record,
        "dynamic_static_swept_volume_free": swept_clearance > 0.0,
        "dynamic_static_swept_margin_m": 0.05,
        "minimum_dynamic_static_swept_clearance_m": round(swept_clearance, 6),
        "closest_dynamic_static_sweep": swept_record,
        "closest_dynamic_static_primitive_type": swept_static_type,
        "minimum_dynamic_wall_swept_clearance_m": round(
            wall_swept_clearance, 6),
        "dynamic_pair_no_unreasonable_overlap": dynamic_pair_clearance > 0.0,
        "minimum_dynamic_pair_clearance_m": round(dynamic_pair_clearance, 6),
        "closest_dynamic_pair_event": dynamic_pair_record,
        "uav_initial_positions_static_free": initial_static_clearance > 0.05,
        "minimum_uav_initial_static_clearance_m": round(initial_static_clearance, 6),
        "closest_uav_initial_static": initial_static_record,
        "minimum_static_pair_clearance_m": round(static_pair_clearance, 6),
        "closest_static_pair": static_pair_record,
        "core_static_pair_intersection_free": core_pair_clearance >= core_static_pair_margin,
        "minimum_core_static_pair_clearance_m": round(core_pair_clearance, 6),
        "closest_core_static_pair": core_pair_record,
        "radius_distribution": radius_distribution(static),
        "effective_pressure_classification": pressure,
        "obstacle_height_covers_flight": all(
            float(item.get("height", 0.0)) >= operational_max_z + 0.20
            for item in list(static.values()) + list(moving.values())) and all(
            float(item.get("sizeENU", [0.0, 0.0, 0.0])[2]) >=
            operational_max_z + 0.20 for item in walls.values()),
        "operational_uav_max_z_m": operational_max_z,
        "interaction_obstacles_below_flight_ceiling": sum(
            float(item.get("height", 0.0)) <= operational_max_z + 0.20
            for item in list(static.values()) + list(moving.values())
        ) + sum(float(item.get("sizeENU", [0.0, 0.0, 0.0])[2]) <=
                operational_max_z + 0.20 for item in walls.values()),
        "static_cylinder_count": len(static),
        "wall_count": len(walls),
        "cylinder_cluster_count": len(supplemental_gaps),
        "core_thin_columns_remaining": sum(
            float(static[str(index)]["radius"]) < 0.28
            for index in range(core_cylinder_count)),
        "wall_candidate_checks": wall_candidate_checks,
        "all_walls_candidate_dependent": all(
            item["candidate_dependent"] for item in wall_candidate_checks),
        "walls_block_all_candidates": any(
            not item["executable_visible_candidates"]
            for item in wall_candidate_checks),
        "dynamic_obstacle_count": len(moving),
        "candidate_enabler_count": len(double_stations),
        "k2_decision_station_count": len(decision_stations),
        "all_decision_local_k2_fail": all(
            item["local_predicted_k2"] == 0 for item in decision_stations),
        "all_decision_alternative_k2_recover": all(
            item["alternative_predicted_k2"] == 1 for item in decision_stations),
        "all_decision_candidates_static_feasible": all(
            item["candidate_space_exists"]
            for item in decision_stations),
        "all_decision_occlusion_roles_preserved": all(
            item["occlusion_role_preserved"] for item in decision_stations),
        "decision_candidate_clearances": [
            {"station": item["station"],
             "local_m": item["local_static_clearance_m"],
             "alternative_m": item["alternative_static_clearance_m"]}
            for item in decision_stations
        ],
        "decision_candidate_visibility": [
            {"station": item["station"],
             "local": item["local_candidate"],
             "alternative": item["alternative_candidate"],
             "fractions": item["candidate_visibility_fraction"],
             "station_visible": item["candidate_station_visible"],
             "k2": item["candidate_predicted_k2"]}
            for item in decision_stations
        ],
    }
    required = (
        checks["target_centerline_physically_free"] and
        checks["target_static_intersections"] == 0 and
        checks["all_outer_preferred_bearings_blocked_somewhere"] and
        checks["central_support_view_preserved"] and
        checks["double_occlusion_station_count"] == 2 and
        checks["all_double_occlusions_k2_recoverable"] and
        checks["minimum_free_tracking_shell_fraction"] >= 0.40 and
        checks["all_obstacles_inside_map_bounds"] and
        checks["candidate_enablers_inside_map_bounds"] and
        checks["target_dynamic_trajectory_free"] and
        checks["dynamic_static_swept_volume_free"] and
        checks["dynamic_pair_no_unreasonable_overlap"] and
        checks["uav_initial_positions_static_free"] and
        checks["core_static_pair_intersection_free"] and
        checks["obstacle_height_covers_flight"] and
        checks["k2_decision_station_count"] == 2 and
        checks["all_decision_local_k2_fail"] and
        checks["all_decision_alternative_k2_recover"] and
        checks["all_decision_candidates_static_feasible"] and
        checks["all_decision_occlusion_roles_preserved"]
        and checks["all_walls_candidate_dependent"]
        and not checks["walls_block_all_candidates"]
    )
    if not required:
        raise RuntimeError("visibility geometry self-check failed: {}".format(checks))

    scene["benchmarkDesign"] = {
        "generator": GENERATOR_ID,
        "benchmark": "cooperative_visibility",
        "source_scene": source.name,
        "route_policy": "preserve every waypoint of the production forest route",
        "visibility_station_count": station_count,
        "single_occlusion_station_count": station_count - len(double_stations),
        "double_occlusion_station_count": len(double_stations),
        "candidate_enabler_count": len(double_stations),
        "dynamic_obstacle_count": len(moving),
        "static_obstacle_count": len(static) + len(walls),
        "static_cylinder_count": len(static),
        "wall_count": len(walls),
        "cylinder_cluster_count": len(supplemental_gaps),
        "supplemental_static_count": supplemental_added,
        "dynamic_pressure_roles": dynamic_roles,
        "core_target_route_changed": False,
        "core_station_centers_changed": False,
        "original_occluder_centers_changed": True,
        "core_occluder_names": core_occluder_names,
        "core_occluder_centers": core_occluder_centers,
        "coreObstacleGeometry": core_geometry,
        "wallCandidateChecks": wall_candidate_checks,
        "wallImplementation": {
            "schema": "wallData: centerENU + sizeENU + yawRad",
            "rviz": "Marker.CUBE plus filled wall point-cloud lattice",
            "static_map": "same filled wall point-cloud lattice",
            "runtime_los": "exact segment-vs-oriented-box test",
            "planner_los": "StaticLosGeometry oriented-box clearance",
        },
        "map_area_m2": round((bounds[1] - bounds[0]) *
                             (bounds[3] - bounds[2]), 6),
        "static_density_per_m2": round(
            (len(static) + len(walls)) / ((bounds[1] - bounds[0]) *
                                          (bounds[3] - bounds[2])), 9
        ),
        "stations": stations,
        "k2DecisionStations": decision_stations,
        "formulae": {
            "occluder_center": "target_station + r_occ*[cos(phi_block), sin(phi_block)]",
            "occluder_radius": "role radius with radial-only relocation on original LOS bearing",
            "central_support_view": "kept statically clear because its LOS is collinear with target route",
            "wall_center": "target_at_interstation_gap + 0.62*outer_tracking_offset",
            "wall_orientation": "target-route tangent + 90 deg; crosses the true LOS",
        },
        "productionParameters": {
            "tracking_offsets": [round_vector(item["tracking_offset"]) for item in drones],
            "tracking_radii_m": [round(value, 6) for value in tracking_radii],
            "preferred_bearings_deg": [round(math.degrees(value), 6) for value in preferred],
            "team_visibility_k": int(float_default(defaults, "team_visibility_k")),
            "team_visibility_preferred_separation_deg": separation_deg,
            "visibility_occlusion_margin_m": visibility_margin,
            "elastic_delta_phi_max_deg": float_default(defaults, "elastic_delta_phi_max_deg"),
            "elastic_radial_slack_max_m": float_default(defaults, "elastic_radial_slack_max"),
            "side_offset_m": side_offset,
            "moving_clearance_m": moving_clearance,
            "candidate_trigger_margin_m": candidate_trigger_margin,
        },
        "benchmarkParameters": {
            "occluder_distance_from_target_m": round(r_occ, 6),
            "single_occlusion_angle_deg": single_angle_deg,
            "double_occlusion_angle_deg": double_angle_deg,
            "original_core_radius_m": round(radius_single, 6),
            # The redesigned double-occlusion stations use wall screens.  The
            # remaining cylindrical core occluders all use the normal 0.43 m
            # radius; do not report the old thin-column lower bound here.
            "core_radius_range_after_m": [0.43, 0.43],
            "core_target_route_margin_m": core_target_route_margin,
            "central_core_target_route_margin_m": central_target_route_margin,
            "core_static_pair_margin_m": core_static_pair_margin,
            "interaction_obstacle_height_m": round(obstacle_height, 6),
            "central_occluder_bias_deg": round(central_bias_deg, 6),
            "central_line_physical_gap_m": central_line_physical_gap,
            "target_body_plus_scene_margin_m": target_static_required_clearance,
            "screen_length_m": 2.40,
            "screen_thickness_m": 0.20,
            "candidate_enabler_nominal_distance_m": round(enabler_distance, 6),
            "candidate_enabler_radius_m": round(enabler_radius, 6),
        },
        "selfCheck": checks,
    }
    return scene


def validate_shared_contract(avoidance, visibility):
    shared = {}
    for key, first, second in (
            ("target_start", avoidance["targetTracking"]["waypointsENU"][0],
             visibility["targetTracking"]["waypointsENU"][0]),
            ("target_goal", avoidance["targetTracking"]["waypointsENU"][-1],
             visibility["targetTracking"]["waypointsENU"][-1]),
            ("target_speed", avoidance["targetTracking"]["speed"],
             visibility["targetTracking"]["speed"]),
            ("uav_initial_state", avoidance["takeoffPointENU"],
             visibility["takeoffPointENU"]),
            ("safety_bounds", avoidance.get("safety"), visibility.get("safety")),
            ("ego_planner_limits", {
                name: avoidance["egoPlanner"].get(name)
                for name in ("maxVel", "maxAcc", "planningHorizon", "mapSizeENU")
             }, {
                name: visibility["egoPlanner"].get(name)
                for name in ("maxVel", "maxAcc", "planningHorizon", "mapSizeENU")
             })):
        shared[key] = first == second
    if not all(shared.values()):
        raise RuntimeError("shared benchmark contract failed: {}".format(shared))
    return shared


def write_generated(path, payload):
    if path.exists():
        existing = json.loads(path.read_text(encoding="utf-8"))
        generator = (existing.get("benchmarkDesign") or {}).get("generator")
        if generator != GENERATOR_ID:
            raise RuntimeError("refusing to overwrite non-generated scene: {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=str(path.parent),
            prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
        json.dump(payload, stream, indent=2, sort_keys=False)
        stream.write("\n")
        temporary = Path(stream.name)
    os.chmod(str(temporary), 0o644)
    os.replace(str(temporary), str(path))


def validate_generated_payload(payload, label):
    """Invoke the standalone exact route/static validator for every build."""
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix="alp_{}_".format(label),
            suffix=".json", delete=False) as stream:
        json.dump(payload, stream, indent=2, sort_keys=False)
        stream.write("\n")
        temporary = Path(stream.name)
    try:
        result = subprocess.run(
            [sys.executable, str(TARGET_ROUTE_VALIDATOR), str(temporary)],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False)
        output = result.stdout.strip()
        if output:
            print("[target-route-static-check] {} {}".format(label, output))
        if result.returncode != 0:
            raise RuntimeError(
                "{} target/static validation failed: {} {}".format(
                    label, output, result.stderr.strip()))
    finally:
        temporary.unlink(missing_ok=True)


def print_summary(avoidance, visibility, avoidance_path, visibility_path):
    a = avoidance["benchmarkDesign"]
    ap = a["productionParameters"]
    ac = a["selfCheck"]
    topology = a["topology_conflict_station"]
    v = visibility["benchmarkDesign"]
    vp = v["productionParameters"]
    vb = v["benchmarkParameters"]
    vc = v["selfCheck"]

    print("=== AVOIDANCE BENCHMARK ===")
    print("output:", avoidance_path)
    print("target route:", avoidance["targetTracking"]["waypointsENU"])
    print("tracking radius:", ap["tracking_radii_m"])
    print("preferred bearings:", ap["preferred_bearings_deg"])
    print("static obstacles:", a["static_obstacle_count"],
          "(stations:", a["station_count"],
          "+ A* repair: 1 + forest:", a["background_forest_count"], ")")
    print("map area:", a["map_area_m2"], "m^2; static density:",
          a["static_density_per_m2"], "obstacles/m^2")
    print("dynamic crossing stations:", a["dynamic_crossing_station_count"],
          "obstacles:", a["dynamic_obstacle_count"])
    print("topology conflict station:", topology["station"],
          topology["target_center"])
    print("predicted nominal dynamic conflicts:", a["dynamic_obstacle_count"])
    print("predicted local topology:", topology["local_combination"])
    print("predicted local topology pairwise min distance:",
          round(topology["local_pairwise_min_distance_m"], 6))
    print("predicted alternative topology:", topology["alternative_combination"])
    print("predicted alternative topology min distance:",
          round(topology["alternative_pairwise_min_distance_m"], 6))
    print("target centerline collision:",
          "NO" if (ac["target_centerline_static_free"] and
                   ac["target_dynamic_trajectory_free"]) else "YES")
    print("minimum dynamic-static swept clearance:",
          ac["minimum_dynamic_static_swept_clearance_m"], "m")
    print("minimum dynamic-dynamic clearance:",
          ac["minimum_dynamic_pair_clearance_m"], "m")
    print("minimum UAV-initial/static clearance:",
          ac["minimum_uav_initial_static_clearance_m"], "m")
    print("radius distribution:", ac["radius_distribution"])
    print("effective pressure:", ac["effective_pressure_classification"])
    print("pressure roles:", ac["pressure_role_counts"])
    print("route-segment static pressure:",
          ac["route_segment_static_pressure"])

    print("\n=== VISIBILITY BENCHMARK ===")
    print("output:", visibility_path)
    print("visibility stations:", v["visibility_station_count"])
    print("single occlusion stations:", v["single_occlusion_station_count"])
    print("double occlusion stations:", v["double_occlusion_station_count"])
    print("candidate-set enablers:", v["candidate_enabler_count"],
          "(outside nominal tracking shell)")
    print("dynamic obstacles:", v["dynamic_obstacle_count"])
    print("static primitives:", v["static_obstacle_count"],
          "cylinders:", v["static_cylinder_count"],
          "walls:", v["wall_count"],
          "clusters:", v["cylinder_cluster_count"])
    print("core cylinders:", sum(len(item["blocked_preferred_drone_ids"])
                                  for item in v["stations"]),
          "cluster cylinders:", v["supplemental_static_count"])
    print("map area:", v["map_area_m2"], "m^2; static density:",
          v["static_density_per_m2"], "obstacles/m^2")
    print("radius distribution:", vc["radius_distribution"])
    print("effective pressure:", vc["effective_pressure_classification"])
    print("minimum target/static physical clearance:",
          vc["target_centerline_min_physical_clearance_m"], "m")
    print("minimum dynamic-static swept clearance:",
          vc["minimum_dynamic_static_swept_clearance_m"], "m")
    print("minimum dynamic-dynamic clearance:",
          vc["minimum_dynamic_pair_clearance_m"], "m")
    print("minimum UAV-initial/static clearance:",
          vc["minimum_uav_initial_static_clearance_m"], "m")
    print("interaction obstacles below flight ceiling:",
          vc["interaction_obstacles_below_flight_ceiling"])
    print("wall candidate checks:", vc["wall_candidate_checks"])
    print("preferred bearings blocked:", vp["preferred_bearings_deg"])
    print("occlusion angle:", vb["single_occlusion_angle_deg"],
          "deg (single and per double occluder)")
    print("original core occluder radius:", vb["original_core_radius_m"])
    print("core occluder radius range after:", vb["core_radius_range_after_m"])
    print("free viewing sectors:", [
        {"station": item["station"],
         "free_fraction": item["free_endpoint_fraction"],
         "example": item["example_recovery_bearings_deg"]}
        for item in v["stations"]
    ])
    print("nominal K2 threatened stations:",
          vc["nominal_k2_threatened_stations"])
    print("elastic K2 recoverable stations:",
          vc["elastic_k2_recoverable_stations"])
    for decision in v.get("k2DecisionStations", []):
        print("\n=== K2 DECISION STATION {} ===".format(decision["station"]))
        print("decision_uav:", decision["decision_uav"])
        print("other_occluded_uav:", decision["other_occluded_uav"])
        print("support_visible_uav:", decision["support_visible_uav"])
        print("LOCAL candidate:", decision["local_candidate"])
        print("ALTERNATIVE candidate:", decision["alternative_candidate"])
        print("LOCAL candidate LOS:", decision["local_candidate_los"])
        print("ALTERNATIVE candidate LOS:", decision["alternative_candidate_los"])
        print("other UAV LOS:", decision["other_uav_los"])
        print("support UAV LOS:", decision["support_uav_los"])
        print("LOCAL predicted K2:", decision["local_predicted_k2"])
        print("ALTERNATIVE predicted K2:", decision["alternative_predicted_k2"])
        print("LOCAL static feasible:",
              "YES" if decision["local_static_feasible"] else "NO")
        print("ALTERNATIVE static feasible:",
              "YES" if decision["alternative_static_feasible"] else "NO")
        print("candidate LOS angular separation:",
              decision["candidate_los_angular_separation_deg"])
        print("occluders:", decision["occluders"])
        print("station position:", decision["station_position"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--launch", type=Path, default=DEFAULT_LAUNCH)
    parser.add_argument("--planner-source", type=Path, default=DEFAULT_PLANNER)
    parser.add_argument("--avoidance-output", type=Path, default=AVOIDANCE_OUTPUT)
    parser.add_argument("--visibility-output", type=Path, default=VISIBILITY_OUTPUT)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--visibility-only", action="store_true",
                        help="write only B; leave the frozen avoidance scene untouched")
    args = parser.parse_args()

    source = args.source.resolve()
    defaults, drones = launch_defaults(args.launch.resolve())
    side_offset = side_offset_from_source(args.planner_source.resolve())
    source_scene = json.loads(source.read_text(encoding="utf-8"))
    source_route = source_scene["targetTracking"]["waypointsENU"]
    route = Polyline(source_route)
    speed = float_default(defaults, "target_speed")

    avoidance = make_avoidance_scene(
        source, route, speed, defaults, drones, side_offset
    )
    visibility = make_visibility_scene(
        source, route, speed, defaults, drones, side_offset
    )
    shared = validate_shared_contract(avoidance, visibility)
    avoidance["benchmarkDesign"]["sharedBenchmarkContract"] = shared
    visibility["benchmarkDesign"]["sharedBenchmarkContract"] = shared

    # This external analytic contract is intentionally invoked even for
    # --check-only, so no generated route can bypass collision validation.
    validate_generated_payload(avoidance, "A")
    validate_generated_payload(visibility, "B")

    if not args.check_only:
        if not args.visibility_only:
            write_generated(args.avoidance_output.resolve(), avoidance)
        write_generated(args.visibility_output.resolve(), visibility)
    print_summary(
        avoidance, visibility,
        args.avoidance_output.resolve(), args.visibility_output.resolve()
    )


if __name__ == "__main__":
    main()
