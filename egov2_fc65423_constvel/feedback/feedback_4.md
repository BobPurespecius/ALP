# C1_RELAXED_GEOMETRY_REPORT

Date: 2026-09-02

## Scope and artifacts

- Source scene: `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2.json`
- New scene: `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2_c1_relaxed.json`
- New-scene SHA256: `ca080a86cc0f53ee49b722e82c6f59deb044c8111ff659a811ccd80ddcf5b1a1`
- Geometry audit: `/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/c1_geometry_audit.json`
- ALP sanity run: `/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/c1_alp_sanity/`
- Planner/controller/algorithm source modified: **NO**

## C1 exact geometry in frozen v2

The JSON distinguishes the route conflict center from the oscillator base center.

| Field | Value |
|---|---|
| Gate ID/type | `C1` / `LEFT_OPEN` |
| Route conflict center | `(7.0000, 1.7500)` m |
| Dynamic base center | `(6.8884, 2.6428)` m |
| Radius | `0.2800 m` |
| Axis | `(-0.124, 0.992)`; unit direction is effectively identical |
| Amplitude | `0.9000 m` |
| Period | `11.5000 s` |
| Phase | `-29.764836 rad` |
| Nominal tangent `t` | `(0.992, 0.124)` |
| Physical LEFT normal `n` | `(-0.124, 0.992)` |
| Full motion endpoint 1 | `(7.000031, 1.749750)` m |
| Full motion endpoint 2 | `(6.776769, 3.535850)` m |
| Target encounter time | `46.353038 s` target-clock design time |
| UAV corridor event | `48.953038 s` target-clock design time |

The physical LEFT-open direction is `+n`: mainly increasing `y`, with a small decrease in `x`.

## Target and nominal corridor near C1

Relevant target waypoints are:

- waypoint 13: `(3.0, 0.6, 1.5)`
- waypoint 14: `(7.0, 1.75, 1.5)`
- waypoint 15: `(11.0, 2.25, 1.5)`

Thus the target passes through the route conflict center, while the moving cylinder reaches that center later for the UAV encounter. The target route and all launch parameters are unchanged in the relaxed scene.

## Static cylinders within 5 m of the dynamic base center

Coordinates `along` and `left_n` are projections relative to route conflict center `(7.0, 1.75)`.

| Name | Center | Radius | Distance to dynamic base | along | left_n | Relation to LEFT-open lane |
|---|---:|---:|---:|---:|---:|---|
| `long_forest_cylinder_09` | `(7.42,-0.40)` | 0.43 | 3.089 | +0.150 | -2.185 | Far on blocked/right side |
| `long_forest_cylinder_19` | `(2.69,5.15)` | 0.50 | 4.890 | -3.854 | +3.907 | Far-left approach boundary, outside local 3 m probe |
| `dynamic_gate_C1_blocker_0` | `(5.9894,0.5658)` | 0.55 | 2.263 | -1.149 | -1.049 | Blocked-side approach pillar; limits LEFT entry clearance |
| `dynamic_gate_C1_blocker_1` | `(8.2710,0.8510)` | 0.55 | 2.263 | +1.149 | -1.049 | Blocked-side exit pillar; limits LEFT exit clearance |

## Local planar sketch

```text
                         +n = physical LEFT / open side
                         ↖
                 LEFT-open bypass corridor
             ================================
                            ○ C1 moving obstacle
 target / nominal  -------> X route conflict center ------->
                    ● blocker_0       ● blocker_1
                     approach          rejoin/exit
                    negative n = blocked/right side
```

The two dedicated pillars are symmetric in route coordinates. Therefore the original sampled `1.5997 m` open-lane minimum is an entry/exit restriction, not a single unrelated forest pillar. The latest benchmark collision itself occurred near the conflict center (`along≈-0.045 m`, `left_n≈-0.187 m`), so the immediate contact was dynamic; the static pillars constrained how early and how cleanly a LEFT bypass could enter and rejoin.

## Minimal relaxation

Only the two C1-specific blocked-side pillars were moved. Their radii remain `0.55 m`.

| Pillar | Old center | New center | Move |
|---|---:|---:|---|
| `dynamic_gate_C1_blocker_0` | `(5.9894,0.5658)` | `(6.014207,0.367344)` | `0.20 m` along `-n` |
| `dynamic_gate_C1_blocker_1` | `(8.2710,0.8510)` | `(8.295807,0.652544)` | `0.20 m` along `-n` |

No dynamic-obstacle radius, speed, amplitude, period, phase, target waypoint, planner parameter, or controller setting was changed.

## Geometry verification

| Check | Before | After | Result |
|---|---:|---:|---|
| LEFT-open lane min static surface clearance at `+1.1 n` | 1.599706 m | 1.799706 m | Target range reached with a 0.20 m move |
| Blocked/right lane min static surface clearance at `-1.1 n` | N/A | -0.400294 m | Still physically constrained |
| Target continuous min static clearance | 0.344444 m | 0.344444 m | Contract `>=0.30 m` passes |
| Target static collision count | 0 | 0 | PASS |
| JSON parsing | PASS | PASS | PASS |
| Model names unique | YES | YES | PASS |

`C1_BLOCKED_SIDE_STILL_CONSTRAINED` is YES because the right-side lane intersects the unchanged-radius blocker geometry even after the small outward shift. C1 remains a directional gate rather than becoming a symmetric open area.

## One-shot ALP sanity result

Run configuration was the current ALP baseline: same scene except the two pillar centers, RViz OFF, target-facing yaw ON, `max_jer=22`, and target-final-stop + 3 s shutdown.

| Metric | Result |
|---|---:|
| Task complete | YES |
| C1 result | `UNSAFE` but not collision |
| C1 collision episodes/samples | `0 / 0` |
| C1 unsafe episodes/samples `<0.5 m` | `2 / 57` |
| C1 minimum dynamic surface clearance | `0.049851 m` |
| All-Gate dynamic collision episodes/samples | `0 / 0` |
| All-Gate unsafe samples | `74` |
| UAV static collision episodes/samples | `0 / 0` |
| UAV minimum static clearance | `0.219718 m` |
| Runtime target static collision samples | `0` |
| Runtime target minimum static clearance | `0.344471 m` |
| Actual nonzero target speed | `0.918720 m/s` |
| Auto shutdown / exit | YES / SIGINT code 0 |

This is exactly one ALP run. Although the former two-sample C1 collision did not recur, the `0.0499 m` minimum and 57 unsafe samples show that the Gate remains challenging. No second geometry adjustment was made.

## Final fields

`C1_LOCATION:` route conflict center `(7.0,1.75)`; dynamic base center `(6.8884,2.6428)`

`C1_NEARBY_STATIC_OBSTACLES:` `long_forest_cylinder_09`, `long_forest_cylinder_19`, `dynamic_gate_C1_blocker_0`, `dynamic_gate_C1_blocker_1`

`C1_STATIC_OBSTACLES_MOVED:` `dynamic_gate_C1_blocker_0` and `_1`, each `0.20 m` toward the blocked/right side; radii unchanged

`C1_OPEN_CLEARANCE_BEFORE:` `1.599706 m`

`C1_OPEN_CLEARANCE_AFTER:` `1.799706 m`

`C1_BLOCKED_SIDE_STILL_CONSTRAINED:` YES

`TARGET_STATIC_COLLISION:` 0

`C1_ALP_RESULT_AFTER_RELAX:` Task complete; C1 collision `0 episodes / 0 samples`; C1 unsafe `2 episodes / 57 samples`; min dynamic surface clearance `0.049851 m`; static collision 0

