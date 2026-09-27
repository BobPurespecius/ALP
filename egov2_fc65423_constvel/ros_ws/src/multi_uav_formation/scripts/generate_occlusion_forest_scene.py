#!/usr/bin/env python3
"""生成"遮挡有效、密度合适"的遮挡森林场景（只改障碍与目标轨迹）。

设计依据（来自 vis_encON_trajectory.csv 的实测作业几何）：
  UAV 稳定待在离目标约 1.38 m 的观测环上（P90 1.70 m），高度与目标一致；
  因此只有落在"目标航迹 ±1.5 m 走廊"内的障碍才可能遮挡 LOS。
  旧场景 36 根静态柱有 33 根距航迹 >2 m，10 个动态障碍有 6 个振幅不足，
  实测遮挡率 <1%，对追踪几乎不构成挑战。

本脚本：
  1) 生成一条对静态障碍保持 >=0.30 m 净空的目标航迹（平滑、长度与旧场景同量级）；
  2) 在航迹两侧 0.9~1.6 m 交替布静态柱（正好能把观测环上的 UAV 挡在目标之外），
     并按需插入少量贴近航迹（0.35~0.5 m）的柱子形成真实的绕行约束；
  3) 布动态障碍：振幅 1.6~2.4 m 且横向偏移使其摆动扫过航迹走廊；
  4) 用经验作业几何做遮挡代理评估，输出每个障碍的遮挡占比与连续遮挡段，
     不满足目标区间就重采样，直到达标。

用法:
  python3 generate_occlusion_forest_scene.py --base <旧场景> --out <新场景> [--report <json>]

自然 Team stress 使用独立的 --natural-team-stress 模式：复用基础森林，按 seed
生成平滑路线、可选轻微稀疏化或补充静态树，并重采样既有动态障碍相位；不使用
legacy 模式中沿路线布置障碍的逻辑。
"""

import argparse
import json
import math
import random
from pathlib import Path

TARGET_CLEARANCE = 0.30      # 目标半径 0.25 + 场景余量 0.05
STATIC_BAND = (0.90, 1.60)   # 静态柱到航迹的横向距离（走廊内，可遮挡观测环）
TIGHT_BAND = (0.36, 0.55)    # 少量贴近航迹的柱子（形成真实约束但不碰撞）
DYN_BAND = (1.10, 2.20)      # 动态障碍基准横向偏移
DYN_AMP = (1.60, 2.40)       # 动态振幅（保证摆动扫过走廊）
DYN_PERIOD = (6.0, 10.0)
STATIC_RATE = (0.04, 0.12)   # 单根静态柱遮挡占比目标区间
DYN_RATE = (0.02, 0.08)
UNION_RATE = (0.08, 0.22)    # 任一障碍遮挡的样本占比
MAX_EPISODE_S = 2.5          # 连续遮挡最长时间
TARGET_SPEED = 0.928
CAMERA_HALF_HFOV_DEG = 42.5
CAMERA_MAX_RANGE = 8.0

# Natural Team-stress pre-screen profile.  The forest remains independent of
# the target route: the route uses the same seeded sinusoid builder below,
# while the existing static trees are only thinned and moving phases resampled.
NATURAL_ROUTE_AMPLITUDE_1 = (2.0, 6.0)
NATURAL_ROUTE_AMPLITUDE_2 = (0.25, 1.5)
NATURAL_ROUTE_CYCLES_1 = (0.7, 2.2)
NATURAL_ROUTE_CYCLES_2 = (2.0, 4.5)
TEAM_SCREEN_FORMATION = (
    # Median target-frame offsets from stable FULL run 20260923_125315_270760.
    (2.083, math.radians(-40.263)),
    (1.270, math.radians(171.269)),
    (2.105, math.radians(37.333)),
)
TEAM_SCREEN_STATIC_INNER = 0.08
TEAM_SCREEN_LOS_SMOOTHING = 0.15
TEAM_SCREEN_TRIGGER = 0.30
TEAM_SCREEN_DT = 0.15


def route_points(wps, step):
    """把折线航迹按 step 重采样成点列。"""
    pts = []
    for a, b in zip(wps, wps[1:]):
        seg = math.dist(a, b)
        n = max(1, int(seg / step))
        for i in range(n):
            t = i / n
            pts.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    pts.append((wps[-1][0], wps[-1][1]))
    return pts


def seg_point_distance(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.dist(a, p)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den))
    return math.dist((a[0] + t * dx, a[1] + t * dy), p)


def route_clearance(wps, obstacle):
    c = obstacle["centerENU"]
    return min(seg_point_distance(a, b, c) for a, b in zip(wps, wps[1:])) - obstacle["radius"]


def blocked(uav, target, center, radius):
    dx, dy = target[0] - uav[0], target[1] - uav[1]
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.dist(uav, center) <= radius
    t = max(0.0, min(1.0, ((center[0] - uav[0]) * dx + (center[1] - uav[1]) * dy) / den))
    return math.dist((uav[0] + t * dx, uav[1] + t * dy), center) <= radius


def uav_offsets(rng, n):
    """经验作业几何：沿航向 -1.7~+1.3 m、横向 -1.7~+0.9 m 上采样观测环位置。"""
    out = []
    for _ in range(n):
        along = rng.uniform(-1.69, 1.32)
        lat = rng.uniform(-1.66, 0.84)
        # 保持 1.2~1.7 m 的观测半径（实测 P10~P90）
        norm = math.hypot(along, lat)
        if norm > 1e-6:
            want = rng.uniform(1.30, 1.70)
            along, lat = along / norm * want, lat / norm * want
        out.append((along, lat))
    return out


def evaluate(wps, static_obs, dyn_obs, offsets, dt=0.05):
    """对采样时刻评估：每个障碍是否遮挡了任一架 UAV 的 LOS。"""
    pts = route_points(wps, TARGET_SPEED * dt)
    total = len(pts)
    hit_static = {k: 0 for k in static_obs}
    hit_dyn = {k: 0 for k in dyn_obs}
    union = 0
    union_series = []
    for idx, tp in enumerate(pts):
        along_dir = None
        if idx + 1 < total:
            dx, dy = pts[idx + 1][0] - tp[0], pts[idx + 1][1] - tp[1]
            n = math.hypot(dx, dy)
            along_dir = (dx / n, dy / n) if n > 1e-9 else (1.0, 0.0)
        else:
            along_dir = (1.0, 0.0)
        ux, uy = along_dir
        any_hit = False
        for k, o in static_obs.items():
            c = o["centerENU"]
            for (a, l) in offsets:
                uav = (tp[0] + ux * a - uy * l, tp[1] + uy * a + ux * l)
                if blocked(uav, tp, c, o["radius"]):
                    hit_static[k] += 1
                    any_hit = True
                    break
        t_elapsed = idx * dt
        for k, o in dyn_obs.items():
            ax, ay = o.get("axisENU", [1.0, 0.0])
            n = math.hypot(ax, ay) or 1.0
            ax, ay = ax / n, ay / n
            w = 2 * math.pi / max(o.get("period", 1.0), 1e-6)
            off = o.get("amplitude", 0.0) * math.sin(w * t_elapsed + o.get("phase", 0.0))
            c = (o["centerENU"][0] + ax * off, o["centerENU"][1] + ay * off)
            for (a, l) in offsets:
                uav = (tp[0] + ux * a - uy * l, tp[1] + uy * a + ux * l)
                if blocked(uav, tp, c, o["radius"]):
                    hit_dyn[k] += 1
                    any_hit = True
                    break
        union += 1 if any_hit else 0
        union_series.append(any_hit)
    # 连续遮挡段
    eps, cur = [], 0
    for v in union_series:
        if v:
            cur += 1
        elif cur:
            eps.append(cur * dt)
            cur = 0
    if cur:
        eps.append(cur * dt)
    return {
        "samples": total,
        "static_rate": {k: v / total for k, v in hit_static.items()},
        "dyn_rate": {k: v / total for k, v in hit_dyn.items()},
        "union_rate": union / total,
        "episodes": len(eps),
        "max_episode_s": max(eps) if eps else 0.0,
        "mean_episode_s": (sum(eps) / len(eps)) if eps else 0.0,
    }


def build_wps(base_wps, rng, span=(-33.0, 36.0),
              amplitude_1=(0.5, 1.2), amplitude_2=(0.25, 0.6),
              cycles_1=(1.5, 2.5), cycles_2=(3.5, 5.0)):
    """生成平滑航迹：在旧航迹基础上做小幅横向平滑起伏，端点不变。"""
    n = 26
    xs = [span[0] + (span[1] - span[0]) * i / (n - 1) for i in range(n)]
    # Seeded low-frequency sinusoids keep the route smooth and independent of
    # obstacle positions.  The default ranges preserve the legacy generator.
    a1, a2 = rng.uniform(*amplitude_1), rng.uniform(*amplitude_2)
    p1, p2 = rng.uniform(0, 2 * math.pi), rng.uniform(0, 2 * math.pi)
    k1, k2 = rng.uniform(*cycles_1), rng.uniform(*cycles_2)
    ys = [a1 * math.sin(k1 * (x - xs[0]) / (xs[-1] - xs[0]) * 2 * math.pi + p1) +
          a2 * math.sin(k2 * (x - xs[0]) / (xs[-1] - xs[0]) * 2 * math.pi + p2) for x in xs]
    ys[0], ys[-1] = base_wps[0][1], base_wps[-1][1]
    return [[x, y, 1.5] for x, y in zip(xs, ys)]


def directional_clearance_risk(clearance, inner, outer):
    """Match tracking_visibility_geometry.h's directional LOS risk curve."""
    if clearance >= outer:
        return 0.0
    width = outer - inner
    if clearance <= inner:
        d = (inner - clearance) / width
        return 1.0 + d - (1.0 - math.exp(-2.0 * d)) / 2.0
    u = (outer - clearance) / width
    return u * u * (3.0 - 2.0 * u)


def segment_cylinder_clearance(start, end, center, radius):
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-12:
        return math.dist(start, center) - radius
    alpha = max(0.0, min(1.0, (
        (center[0] - start[0]) * dx + (center[1] - start[1]) * dy
    ) / denominator))
    nearest = (start[0] + alpha * dx, start[1] + alpha * dy)
    return math.dist(nearest, center) - radius


def route_samples_with_time(wps, speed, dt):
    samples = []
    travelled = 0.0
    for first, second in zip(wps, wps[1:]):
        length = math.dist(first[:2], second[:2])
        count = max(1, int(length / max(speed * dt, 1.0e-6)))
        yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        for index in range(count):
            alpha = index / float(count)
            distance = travelled + alpha * length
            samples.append((
                (first[0] + alpha * (second[0] - first[0]),
                 first[1] + alpha * (second[1] - first[1])),
                distance / speed, yaw,
            ))
        travelled += length
    samples.append(((wps[-1][0], wps[-1][1]), travelled / speed, 0.0))
    return samples


def natural_team_m2_screen(scene, dt=TEAM_SCREEN_DT):
    """Cheap LOS-only team-M2 proxy for seeded scene ranking.

    It uses a stable FULL-run formation template and the shared differentiable
    cylinder clearance risk.  FOV/range are held clear because the template is
    1.27--2.11 m from the target and target-facing yaw is assumed.  Runtime
    planner logs remain authoritative for exact Team margins and events.
    """
    tracking = scene.get("targetTracking") or {}
    wps = tracking.get("waypointsENU") or []
    speed = float(tracking.get("speed", TARGET_SPEED))
    samples = route_samples_with_time(wps, speed, dt)
    static_obs = list((scene.get("obstacleData") or {}).values())
    dynamic_obs = list((scene.get("movingObstacleData") or {}).values())
    m2_values = []
    m2_limiters = []
    minimum_dynamic_target_clearance = float("inf")

    for target, elapsed, yaw in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        uav_margins = []
        uav_limiters = []
        for radius, angle in TEAM_SCREEN_FORMATION:
            along, lateral = radius * math.cos(angle), radius * math.sin(angle)
            observer = (
                target[0] + c * along - s * lateral,
                target[1] + s * along + c * lateral,
            )
            static_risk = max((directional_clearance_risk(
                segment_cylinder_clearance(
                    observer, target, obstacle["centerENU"],
                    float(obstacle.get("radius", 0.5))),
                TEAM_SCREEN_STATIC_INNER,
                TEAM_SCREEN_STATIC_INNER + TEAM_SCREEN_LOS_SMOOTHING,
            ) for obstacle in static_obs), default=0.0)
            dynamic_risk = 0.0
            for obstacle in dynamic_obs:
                axis_x, axis_y = obstacle.get("axisENU", [1.0, 0.0])
                axis_norm = math.hypot(axis_x, axis_y) or 1.0
                axis_x, axis_y = axis_x / axis_norm, axis_y / axis_norm
                period = max(float(obstacle.get("period", 1.0)), 1.0e-6)
                phase = (2.0 * math.pi * elapsed / period +
                         float(obstacle.get("phase", 0.0)))
                amplitude = float(obstacle.get("amplitude", 0.0))
                center = (
                    float(obstacle["centerENU"][0]) +
                    axis_x * amplitude * math.sin(phase),
                    float(obstacle["centerENU"][1]) +
                    axis_y * amplitude * math.sin(phase),
                )
                clearance = segment_cylinder_clearance(
                    observer, target, center,
                    float(obstacle.get("radius", 0.5)))
                dynamic_risk = max(dynamic_risk, directional_clearance_risk(
                    clearance, TEAM_SCREEN_STATIC_INNER,
                    TEAM_SCREEN_STATIC_INNER + TEAM_SCREEN_LOS_SMOOTHING,
                ))
                minimum_dynamic_target_clearance = min(
                    minimum_dynamic_target_clearance,
                    math.dist(target, center) -
                    float(obstacle.get("radius", 0.5)),
                )
            uav_margins.append(1.0 - 2.0 * max(static_risk, dynamic_risk))
            uav_limiters.append(
                "STATIC" if static_risk >= dynamic_risk else "DYNAMIC_LOS")
        # M2 is the second-largest of the three channel margins.
        m2_order = sorted(range(len(uav_margins)),
                          key=lambda index: uav_margins[index], reverse=True)
        m2_index = m2_order[1]
        m2_values.append(uav_margins[m2_index])
        m2_limiters.append(uav_limiters[m2_index])

    events = []
    start = None
    for index, margin in enumerate(m2_values):
        active = margin < TEAM_SCREEN_TRIGGER
        if active and start is None:
            start = index
        if (not active or index == len(m2_values) - 1) and start is not None:
            end = index if active else index - 1
            event_margins = m2_values[start:end + 1]
            event_min_index = start + event_margins.index(min(event_margins))
            events.append({
                "begin_s": round(samples[start][1], 3),
                "end_s": round(samples[end][1], 3),
                "min_m2_proxy": round(min(event_margins), 6),
                "min_limiter_proxy": m2_limiters[event_min_index],
            })
            start = None

    route_length = sum(math.dist(a[:2], b[:2]) for a, b in zip(wps, wps[1:]))
    target_static_margin = min((route_clearance(wps, obstacle)
                                for obstacle in static_obs), default=float("inf")) - TARGET_CLEARANCE
    max_turn_deg = 0.0
    for index in range(1, len(wps) - 1):
        first = (wps[index][0] - wps[index - 1][0],
                 wps[index][1] - wps[index - 1][1])
        second = (wps[index + 1][0] - wps[index][0],
                  wps[index + 1][1] - wps[index][1])
        denominator = math.hypot(*first) * math.hypot(*second)
        if denominator > 1.0e-12:
            cosine = max(-1.0, min(1.0,
                (first[0] * second[0] + first[1] * second[1]) / denominator))
            max_turn_deg = max(max_turn_deg, math.degrees(math.acos(cosine)))
    return {
        "screen_method": "LOS-only nominal M2 proxy; stable FULL-run formation offsets",
        "sample_dt_s": dt,
        "route_length_m": round(route_length, 3),
        "route_max_turn_deg": round(max_turn_deg, 3),
        "route_max_abs_y_m": round(max(abs(point[1]) for point in wps), 3),
        "minimum_target_static_margin_m": round(target_static_margin, 6),
        "minimum_dynamic_target_surface_clearance_m": round(
            minimum_dynamic_target_clearance, 6),
        "m2_proxy_min": round(min(m2_values), 6) if m2_values else None,
        "m2_proxy_event_count": len(events),
        # The LOS-only proxy has no trajectory/SCP deficit evidence, so it
        # reports crossings without labeling them shallow or infeasible.
        "m2_proxy_risk_events": len(events),
        "events": events,
    }


def add_natural_static_trees(scene, count, seed):
    """Add seeded trees across the forest using its observed longitudinal and row layout."""
    if count <= 0:
        return
    existing = scene.get("obstacleData") or {}
    obstacles = list(existing.values())
    if not obstacles:
        raise ValueError("cannot sample additional trees without a base forest")
    points = [obstacle["centerENU"] for obstacle in obstacles]
    x_min = min(point[0] for point in points)
    x_max = max(point[0] for point in points)
    y_values = [float(point[1]) for point in points]
    radii = [float(obstacle.get("radius", 0.43)) for obstacle in obstacles]
    heights = [float(obstacle.get("height", 3.6)) for obstacle in obstacles]
    numeric_keys = [int(key) for key in existing if str(key).isdigit()]
    next_key = max(numeric_keys, default=-1) + 1

    rng = random.Random(seed)
    accepted = []
    minimum_center_spacing = 1.55
    attempts = 0
    while len(accepted) < count and attempts < count * 5000:
        attempts += 1
        x = rng.uniform(x_min + 0.5, x_max - 0.5)
        # Preserve the source forest's multi-row cross-track distribution while
        # adding small natural jitter.  Placement is independent of the route.
        y = max(min(y_values) - 0.35, min(max(y_values) + 0.35,
                                          rng.choice(y_values) + rng.gauss(0.0, 0.45)))
        point = (x, y)
        if any(math.dist(point, old) < minimum_center_spacing
               for old in points + accepted):
            continue
        accepted.append(point)

    if len(accepted) != count:
        raise ValueError("could not place requested natural trees with source spacing")

    for index, point in enumerate(accepted):
        key = str(next_key + index)
        existing[key] = {
            "modelName": f"forest_cylinder_{key}",
            "centerENU": [round(point[0], 3), round(point[1], 3)],
            "radius": rng.choice(radii),
            "height": rng.choice(heights),
        }
    scene["obstacleData"] = existing


def natural_team_candidate(base_scene, seed, static_retention=0.95,
                          dynamic_phase_seed=None, extra_static_count=0,
                          extra_static_seed=None):
    """Seed one route, optionally add natural trees, thin, and reseed phases."""
    route_rng = random.Random(seed)
    base_wps = (base_scene.get("targetTracking") or {}).get("waypointsENU") or []
    wps = build_wps(
        base_wps, route_rng,
        amplitude_1=NATURAL_ROUTE_AMPLITUDE_1,
        amplitude_2=NATURAL_ROUTE_AMPLITUDE_2,
        cycles_1=NATURAL_ROUTE_CYCLES_1,
        cycles_2=NATURAL_ROUTE_CYCLES_2,
    )
    scene = json.loads(json.dumps(base_scene))
    scene["targetTracking"]["waypointsENU"] = [
        [round(x, 3), round(y, 3), 1.5] for x, y, _ in wps
    ]
    scene["targetTracking"]["speed"] = float(
        (base_scene.get("targetTracking") or {}).get("speed", TARGET_SPEED))

    thinning_rng = random.Random(seed + 1_000_000)
    scene["obstacleData"] = {
        key: obstacle for key, obstacle in
        (base_scene.get("obstacleData") or {}).items()
        if thinning_rng.random() >= 1.0 - static_retention
    }
    if extra_static_seed is None:
        extra_static_seed = seed + 3_000_000
    add_natural_static_trees(scene, extra_static_count, extra_static_seed)
    if dynamic_phase_seed is None:
        dynamic_phase_seed = seed + 2_000_000
    phase_rng = random.Random(dynamic_phase_seed)
    for obstacle in (scene.get("movingObstacleData") or {}).values():
        obstacle["phase"] = round(phase_rng.uniform(0.0, 2.0 * math.pi), 6)
    scene["nMovingObstacle"] = len(scene.get("movingObstacleData") or {})
    scene["naturalTeamStress"] = {
        "generator": "generate_occlusion_forest_scene.py",
        "source_scene": "long_cylinder_forest.json",
        "scene_seed": seed,
        "target_route_seed": seed,
        "static_thinning_seed": seed + 1_000_000,
        "static_tree_retention_probability": static_retention,
        "extra_static_tree_count": extra_static_count,
        "extra_static_tree_seed": extra_static_seed,
        "dynamic_phase_seed": dynamic_phase_seed,
        "route_profile": {
            "amplitude_1_m": NATURAL_ROUTE_AMPLITUDE_1,
            "amplitude_2_m": NATURAL_ROUTE_AMPLITUDE_2,
            "cycles_1": NATURAL_ROUTE_CYCLES_1,
            "cycles_2": NATURAL_ROUTE_CYCLES_2,
        },
        "obstacle_policy": (
            "retain original centers/radii/heights; optionally add seeded "
            "static cylinders sampled across the forest's existing x extent "
            "and empirical y-row distribution; randomly thin existing static "
            "cylinders and resample only existing dynamic phases"
        ),
    }
    return scene


def natural_candidate_geometry(scene, dt=TEAM_SCREEN_DT):
    """Fast safety/shape prefilter before the more expensive LOS M2 proxy."""
    tracking = scene.get("targetTracking") or {}
    wps = tracking.get("waypointsENU") or []
    speed = float(tracking.get("speed", TARGET_SPEED))
    static_obs = list((scene.get("obstacleData") or {}).values())
    dynamic_obs = list((scene.get("movingObstacleData") or {}).values())
    static_margin = min((route_clearance(wps, obstacle)
                         for obstacle in static_obs), default=float("inf")) - TARGET_CLEARANCE
    samples = route_samples_with_time(wps, speed, dt)
    dynamic_target_clearance = float("inf")
    for target, elapsed, _ in samples:
        for obstacle in dynamic_obs:
            axis_x, axis_y = obstacle.get("axisENU", [1.0, 0.0])
            axis_norm = math.hypot(axis_x, axis_y) or 1.0
            phase = (2.0 * math.pi * elapsed /
                     max(float(obstacle.get("period", 1.0)), 1.0e-6) +
                     float(obstacle.get("phase", 0.0)))
            amplitude = float(obstacle.get("amplitude", 0.0))
            center = (
                float(obstacle["centerENU"][0]) +
                axis_x / axis_norm * amplitude * math.sin(phase),
                float(obstacle["centerENU"][1]) +
                axis_y / axis_norm * amplitude * math.sin(phase),
            )
            dynamic_target_clearance = min(
                dynamic_target_clearance,
                math.dist(target, center) - float(obstacle.get("radius", 0.5)),
            )
    route_length = sum(math.dist(a[:2], b[:2]) for a, b in zip(wps, wps[1:]))
    max_turn_deg = 0.0
    for index in range(1, len(wps) - 1):
        first = (wps[index][0] - wps[index - 1][0],
                 wps[index][1] - wps[index - 1][1])
        second = (wps[index + 1][0] - wps[index][0],
                  wps[index + 1][1] - wps[index][1])
        denominator = math.hypot(*first) * math.hypot(*second)
        if denominator > 1.0e-12:
            cosine = max(-1.0, min(1.0,
                (first[0] * second[0] + first[1] * second[1]) / denominator))
            max_turn_deg = max(max_turn_deg, math.degrees(math.acos(cosine)))
    return {
        "route_length_m": round(route_length, 3),
        "route_max_turn_deg": round(max_turn_deg, 3),
        "route_max_abs_y_m": round(max(abs(point[1]) for point in wps), 3),
        "minimum_target_static_margin_m": round(static_margin, 6),
        "minimum_dynamic_target_surface_clearance_m": round(
            dynamic_target_clearance, 6),
    }


def place_static(wps, rng, n_static, n_tight, radius=0.43, height=3.6):
    """沿航迹两侧交替布柱：主体落在走廊带（可遮挡），少量贴近航迹（形成约束）。"""
    pts = route_points(wps, 0.25)
    total = len(pts)
    obs = {}
    used = []
    idx = 0
    for i in range(n_static):
        frac = (i + 0.5) / n_static
        j = min(total - 2, max(1, int(frac * (total - 2))))
        px, py = pts[j]
        dx, dy = pts[j + 1][0] - px, pts[j + 1][1] - py
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        nx, ny = -uy, ux
        side = 1.0 if (i % 2 == 0) else -1.0
        band = TIGHT_BAND if i < n_tight else STATIC_BAND
        d = rng.uniform(*band)
        # 侧向抖动，避免完全规则
        d *= rng.uniform(0.95, 1.15)
        cx, cy = px + nx * side * d, py + ny * side * d
        key = f"{i:02d}"
        obs[key] = {
            "modelName": f"occlusion_forest_cylinder_{i:02d}",
            "centerENU": [round(cx, 3), round(cy, 3)],
            "radius": radius,
            "height": height,
        }
        used.append((cx, cy))
    return obs


def place_dynamic(wps, rng, n_dyn, radius=0.28, height=3.6):
    """动态障碍：基准落在走廊内，振幅保证摆动扫过航迹。"""
    pts = route_points(wps, 0.25)
    total = len(pts)
    obs = {}
    for i in range(n_dyn):
        frac = (i + 0.5) / n_dyn
        j = min(total - 2, max(1, int(frac * (total - 2))))
        px, py = pts[j]
        dx, dy = pts[j + 1][0] - px, pts[j + 1][1] - py
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        # 摆动轴垂直于航迹（扫过走廊），基准横向偏移在走廊内
        side = 1.0 if (i % 2 == 0) else -1.0
        off = rng.uniform(*DYN_BAND) * side
        axis = [-uy, ux]
        obs[f"{i}"] = {
            "modelName": f"occlusion_forest_moving_{i}",
            "centerENU": [round(px + (-uy) * off, 3), round(py + ux * off, 3)],
            "radius": radius,
            "height": height,
            "axisENU": [round(axis[0], 4), round(axis[1], 4)],
            "amplitude": round(rng.uniform(*DYN_AMP), 3),
            "period": round(rng.uniform(*DYN_PERIOD), 3),
            "phase": round(rng.uniform(0, 2 * math.pi), 3),
        }
    return obs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--report", type=Path, default=None)
    ap.add_argument("--seed", type=int, default=20260918)
    ap.add_argument("--static", type=int, default=30)
    ap.add_argument("--tight", type=int, default=6)
    ap.add_argument("--dynamic", type=int, default=10)
    ap.add_argument("--attempts", type=int, default=400)
    ap.add_argument(
        "--natural-team-stress", action="store_true",
        help="保留基础森林的自然布局，用 seed 生成路线/稀疏化/动态相位")
    ap.add_argument(
        "--static-retention", type=float, default=0.95,
        help="natural mode 中每根既有静态树独立保留概率")
    ap.add_argument(
        "--extra-static-count", type=int, default=0,
        help="natural mode 中按原森林空间分布添加的静态树数量")
    ap.add_argument(
        "--extra-static-seed", type=int, default=None,
        help="单独指定新增静态树的 seed；默认使用 scene seed + 3000000")
    ap.add_argument(
        "--dynamic-phase-seed", type=int, default=None,
        help="单独指定原有动态障碍相位的随机种子；默认使用 scene seed + 2000000")
    ap.add_argument(
        "--search-seeds", nargs=2, type=int, metavar=("START", "COUNT"),
        help="natural mode 批量筛 seed；只输出摘要，不写场景文件")
    ap.add_argument(
        "--min-route-margin", type=float, default=0.05,
        help="natural seed 筛选要求的 target 对静态树净空余量，单位 m")
    args = ap.parse_args()

    scene = json.loads(args.base.read_text())
    base_wps = scene["targetTracking"]["waypointsENU"]

    if args.natural_team_stress:
        if not 0.0 < args.static_retention <= 1.0:
            ap.error("--static-retention 必须位于 (0, 1]")
        if args.extra_static_count < 0:
            ap.error("--extra-static-count 不能为负数")

        def build_and_screen(seed):
            candidate = natural_team_candidate(
                scene, seed, static_retention=args.static_retention,
                dynamic_phase_seed=args.dynamic_phase_seed,
                extra_static_count=args.extra_static_count,
                extra_static_seed=args.extra_static_seed)
            screen = natural_candidate_geometry(candidate)
            safe = (
                screen["minimum_target_static_margin_m"] >= args.min_route_margin and
                screen["minimum_dynamic_target_surface_clearance_m"] >= TARGET_CLEARANCE and
                screen["route_length_m"] <= 120.0 and
                screen["route_max_turn_deg"] <= 65.0 and
                screen["route_max_abs_y_m"] <= 10.0
            )
            if safe:
                screen.update(natural_team_m2_screen(candidate))
            else:
                screen.update({
                    "screen_method": "LOS-only nominal M2 proxy; skipped after geometry prefilter",
                    "m2_proxy_min": None,
                    "m2_proxy_event_count": None,
                    "m2_proxy_risk_events": None,
                    "events": [],
                })
            return candidate, {"seed": seed, "safe_geometry": safe,
                               "static_trees": len(candidate.get("obstacleData", {})),
                               "dynamic_obstacles": len(candidate.get("movingObstacleData", {})),
                               **screen}

        if args.search_seeds:
            seed_start, seed_count = args.search_seeds
            if seed_count <= 0:
                ap.error("--search-seeds COUNT 必须为正数")
            screened = [build_and_screen(seed)[1]
                        for seed in range(seed_start, seed_start + seed_count)]
            screened.sort(key=lambda item: (
                item["safe_geometry"],
                item["m2_proxy_risk_events"] or 0,
                item["minimum_target_static_margin_m"],
            ), reverse=True)
            report = {
                "source_scene": str(args.base),
                "seed_start": seed_start,
                "seeds_screened": seed_count,
                "static_retention_probability": args.static_retention,
                "minimum_route_margin_m": args.min_route_margin,
                "safe_geometry_candidates": sum(
                    int(item["safe_geometry"]) for item in screened),
                "ranked_candidates": screened[:50],
            }
            if args.report:
                args.report.write_text(json.dumps(report, indent=2,
                                                  ensure_ascii=False) + "\n")
            print(json.dumps(report, indent=2, ensure_ascii=False))
            return

        if args.out is None:
            ap.error("natural mode 生成场景需要 --out，或使用 --search-seeds")
        candidate, screen = build_and_screen(args.seed)
        if not screen["safe_geometry"]:
            raise SystemExit(
                "seed {} 未通过静态/动态 target 净空或路线形状筛选: {}".format(
                    args.seed, json.dumps(screen, ensure_ascii=False)))
        candidate["naturalTeamStress"]["preScreen"] = screen
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(candidate, indent=2, ensure_ascii=False) + "\n")
        report = {"out": str(args.out), **screen}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2,
                                              ensure_ascii=False) + "\n")
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return

    if args.out is None:
        ap.error("legacy generation mode 需要 --out")

    rng = random.Random(args.seed)
    offsets = uav_offsets(rng, 60)
    best = None
    for attempt in range(args.attempts):
        wps = build_wps(base_wps, rng)
        static_obs = place_static(wps, rng, args.static, args.tight)
        dyn_obs = place_dynamic(wps, rng, args.dynamic)
        # 硬约束：目标对静态障碍净空
        worst = min(route_clearance(wps, o) for o in static_obs.values())
        if worst < TARGET_CLEARANCE + 0.02:
            continue
        rep = evaluate(wps, static_obs, dyn_obs, offsets)
        sr = [v for v in rep["static_rate"].values() if v > 0]
        dr = [v for v in rep["dyn_rate"].values() if v > 0]
        n_dyn_active = len(dr)
        ok = (UNION_RATE[0] <= rep["union_rate"] <= UNION_RATE[1] and
              rep["max_episode_s"] <= MAX_EPISODE_S and
              len(sr) >= args.static * 0.6 and
              all(STATIC_RATE[0] * 0.5 <= v <= STATIC_RATE[1] * 2 for v in sr) and
              n_dyn_active >= max(3, int(args.dynamic * 0.4)))
        score = (abs(rep["union_rate"] - 0.15) +
                 0.02 * max(0.0, rep["max_episode_s"] - 1.5) +
                 0.5 * (1.0 - len(sr) / max(1, args.static)))
        if best is None or (ok and score < best[0]) or (ok and not best[1]):
            best = (score, ok, wps, static_obs, dyn_obs, rep, worst)
            if ok and score < 0.03:
                break
    if best is None:
        raise SystemExit("未找到满足约束的布点，请放宽参数")
    score, ok, wps, static_obs, dyn_obs, rep, worst = best

    scene["targetTracking"]["waypointsENU"] = [[round(x, 3), round(y, 3), 1.5] for x, y, _ in wps]
    scene["targetTracking"]["speed"] = TARGET_SPEED
    scene["obstacleData"] = static_obs
    scene["movingObstacleData"] = dyn_obs
    scene["nMovingObstacle"] = len(dyn_obs)
    args.out.write_text(json.dumps(scene, indent=2, ensure_ascii=False) + "\n")

    report = {
        "out": str(args.out),
        "ok": ok,
        "target_route_length_m": round(sum(math.dist(a[:2], b[:2]) for a, b in zip(wps, wps[1:])), 2),
        "min_target_static_clearance_m": round(worst, 4),
        "union_blocked_rate": round(rep["union_rate"], 4),
        "episodes": rep["episodes"],
        "max_episode_s": round(rep["max_episode_s"], 3),
        "mean_episode_s": round(rep["mean_episode_s"], 3),
        "static_obstacles": len(static_obs),
        "static_with_effect": sum(1 for v in rep["static_rate"].values() if v > 0),
        "dynamic_obstacles": len(dyn_obs),
        "dynamic_with_effect": sum(1 for v in rep["dyn_rate"].values() if v > 0),
        "top_static": sorted(((k, round(v, 4)) for k, v in rep["static_rate"].items()),
                             key=lambda kv: -kv[1])[:8],
        "dynamic_rates": sorted(((k, round(v, 4)) for k, v in rep["dyn_rate"].items()),
                                key=lambda kv: -kv[1]),
    }
    if args.report:
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
