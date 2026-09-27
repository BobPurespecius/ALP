# VISIBILITY_STRESS_SCENE_DESIGN_AND_BENCHMARK

Date: 2026-09-02

## Artifacts and freeze

- Scene: `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`
- Frozen SHA256: `430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`
- Geometry audit: `/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/visibility_scene_audit.json`
- Run data: `/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/visibility_benchmark/`
- Metrics: `/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/visibility_benchmark/benchmark_metrics.json`
- Planner/controller/algorithm source modified: **NO**

The scene was generated from frozen v2 before any of these three runs. The original forest cylinders `0..35` and target path are retained. The four former C1/C2-specific gate blockers are replaced by three method-independent visibility occluders. Dynamic obstacles are replaced by exactly V1/V2/V3.

## Design contract

Representative observer geometry is defined from target route coordinates, not from a method's measured trajectory:

- nominal/blocked-side observer: target `-1.8 t + 1.1 blocked_n`
- open-side observer: target `-1.8 t + 1.3 open_n`
- occluder: target `-1.10 t + 0.72 blocked_n`
- static occluder radius: `0.32 m`
- LOS design margin: `0.15 m`
- UAV inflated static geometry contract: `0.30 + 0.35 = 0.65 m`

For every Gate, the nominal observer LOS intersects its occluder while the open-side observer LOS clears it by about `0.908 m`. This construction used target route, nominal formation corridor, and LEFT/RIGHT topology only; no Native, Gradient, or ALP trajectory was used to place an obstacle.

## Visibility Gate geometry

| Gate | Type/open side | Center | Target encounter | UAV event | Max speed | Nominal LOS surface clearance | Open LOS surface clearance | Open-lane static clearance |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| V1 | crossing / LEFT | `(3.0,0.6)` | 41.823 s | 43.823 s | 0.524 m/s | -0.279 m | 0.908 m | 1.700 m |
| V2 | crossing / RIGHT | `(11.0,2.25)` | 50.741 s | 52.741 s | 0.524 m/s | -0.279 m | 0.908 m | 1.545 m |
| V3 | dwell / LEFT | `(29.5,-1.5)` | 73.337 s | 76.037 s | 0.500 m/s | -0.279 m | 0.908 m | 1.700 m |

Each blocked-side lane has only about `0.060 m` static surface clearance at its representative offset, while each open lane exceeds the `0.65 m` inflated-UAV contract. The open side is therefore usable but still bounded by the forest; it is not a large empty corridor.

Dynamic-to-static minimum surface gaps are `0.500 m` (V1), `0.500 m` (V2), and `0.715 m` (V3), so no moving cylinder is spawned inside a static cylinder.

## Static and target sanity

| Check | Result |
|---|---:|
| JSON parsing | PASS |
| Model names unique | YES |
| Target continuous static collisions | 0 |
| Target continuous minimum static clearance | 0.344444 m |
| Target `>=0.30 m` contract | PASS |
| Static collision-free bypass exists at V1/V2/V3 | YES |
| Nominal representative LOS occluded at V1/V2/V3 | YES |
| Open representative LOS clear at V1/V2/V3 | YES |
| Dynamic speeds within 0.4–0.6 m/s | YES |

The existing target coordinator's full speed-scale grid was replayed offline for motion-clock lead `2.45–2.85 s`. Every segment selected scale `0.99`; actual run logs confirmed the identical nonzero target speed `[0.918720] m/s` for all methods.

## Fair-run controls

- Same scene hash in Native, Gradient, and ALP metadata: YES.
- Scene hash rechecked before each launch: YES.
- Same target waypoints and target speed: YES.
- Same target-facing yaw, RViz OFF, `max_jer=22`, initial state, evaluator, and final-stop cutoff: YES.
- All three reached final target and held stop for about 3 s: YES.
- All three exited through SIGINT with code 0: YES.
- Residual ALP/ROS processes or occupied benchmark ports: NONE.
- Scene or planner changes between methods: NONE.

## Main benchmark results

Dynamic clearance is signed center-to-cylinder-surface clearance; `<=0` is collision and `<0.5 m` is unsafe.

| Metric | Native | Gradient | ALP |
|---|---:|---:|---:|
| Task complete | YES | YES | YES |
| Dynamic collision episodes | 3 | 4 | 1 |
| Dynamic collision samples | 40 | 58 | 7 |
| Dynamic unsafe samples | 288 | 273 | 46 |
| Minimum dynamic clearance | -0.219 m | -0.229 m | -0.229 m |
| Static collision episodes/samples | 0 / 0 | 0 / 0 | 0 / 0 |
| Minimum UAV static clearance | 0.169 m | 0.187 m | 0.187 m |
| Tracking mean | 1.525 m | 1.551 m | 1.641 m |
| Tracking P95 | 1.762 m | 1.780 m | 2.198 m |
| Mean path length | 95.298 m | 84.692 m | 79.593 m |
| Planning latency median | 0.330 ms | 0.367 ms | 0.360 ms |
| Planning latency P95 | 0.689 ms | 0.714 ms | 0.762 ms |
| Planning latency max | 30.283 ms | 21.616 ms | 23.037 ms |
| UAV1 visibility | 98.75% | 95.63% | 94.21% |
| UAV2 visibility | 96.00% | 96.44% | 96.36% |
| UAV3 visibility | 96.55% | 96.85% | 96.90% |
| All-3 visibility | 92.43% | 91.13% | 88.88% |
| At-least-2 visibility | 98.88% | 97.79% | 98.59% |
| At-least-1 visibility | 100.00% | 100.00% | 100.00% |
| None-visible | 0.00% | 0.00% | 0.00% |
| Static LOS blocked, all UAV decisions | 0.47% | 0.46% | 0.85% |
| Dynamic LOS blocked, all UAV decisions | 2.36% | 3.23% | 3.29% |
| FOV loss, all UAV decisions | 0.07% | 0.00% | 0.03% |

ALP is substantially better by collision/unsafe episode count, but its one V3 penetration is deep enough that the global minimum clearance is not better than the other methods. Native has the best measured visibility and tracking; ALP has the shortest executed path.

## Dynamic result by Gate

| Gate | Native | Gradient | ALP |
|---|---|---|---|
| V1 | COLLISION; -0.219 m; 123 unsafe; 20 collision samples | COLLISION; -0.140 m; 106; 13 | UNSAFE; +0.367 m; 27; 0 |
| V2 | COLLISION; -0.200 m; 79; 15 | COLLISION; -0.148 m; 75; 13 | SAFE; +0.701 m; 0; 0 |
| V3 | COLLISION; -0.018 m; 86; 5 | COLLISION; -0.229 m; 92; 32 | COLLISION; -0.229 m; 19; 7 |

The intended topology challenge is therefore real dynamically: ALP avoided collision at V1 and V2 and reduced total unsafe samples, while V3 remains a failure for all three methods.

## Per-Gate visibility source

Ratios use the same six-second target-clock window centered on each designed UAV corridor event. Static and dynamic blockage can overlap.

| Gate | Method | Static LOS blocked | Dynamic LOS blocked | Visible ratio |
|---|---|---:|---:|---:|
| V1 | Native | 0.00% | 10.59% | 89.41% |
| V1 | Gradient | 0.00% | 15.19% | 84.81% |
| V1 | ALP | 5.19% | 8.52% | 86.30% |
| V2 | Native | 6.30% | 10.19% | 83.52% |
| V2 | Gradient | 6.30% | 8.89% | 84.81% |
| V2 | ALP | 6.30% | 9.44% | 84.26% |
| V3 | Native | 0.00% | 11.30% | 88.70% |
| V3 | Gradient | 0.00% | 19.81% | 80.19% |
| V3 | ALP | 0.00% | 26.30% | 73.70% |

Observed static blockers were method-independent scene objects:

- V1: ALP alone crossed a LOS region blocked by `visibility_V1_occluder` in this run (28 decisions); Native/Gradient did not.
- V2: `visibility_V2_occluder` blocked exactly 34 decisions for every method.
- V3: no actual method entered the representative static-occlusion line often enough to register static blockage; visibility loss was dynamic.

Thus the designed LOS mechanism exists geometrically, but this single benchmark does **not** show an ALP visibility advantage from it. ALP's explicit topology improved dynamic safety at V1/V2, yet did not consistently move the full formation into the better-LOS observer region; at V3 its dynamic LOS blockage was the largest.

This negative result is important: target route, yaw, runtime, and frozen scene were identical, so it cannot be explained by an unfair setup. It means the scene provides the intended opportunity, but the current ALP behavior did not convert that opportunity into higher aggregate visibility in this run.

## Executed smoothness

Odometry velocities were linearly resampled to a common 30 Hz grid before finite differencing. This avoids numerical spikes from nearly duplicate CSV timestamps. These are executed finite-difference values, not MINCO analytic jerk.

| Method/UAV | Acc RMS | Acc P95 | Jerk RMS | Jerk P95 | Jerk max |
|---|---:|---:|---:|---:|---:|
| Native UAV1 | 2.912 | 1.784 | 98.799 | 52.348 | 3318.440 |
| Native UAV2 | 0.835 | 1.608 | 22.114 | 43.185 | 208.555 |
| Native UAV3 | 0.843 | 1.391 | 23.149 | 37.401 | 267.966 |
| Native mean | 1.530 | 1.594 | 48.021 | 44.312 | 1264.987 |
| Gradient UAV1 | 0.586 | 1.239 | 14.326 | 30.469 | 117.948 |
| Gradient UAV2 | 0.657 | 1.297 | 16.632 | 32.967 | 149.299 |
| Gradient UAV3 | 0.503 | 0.978 | 11.724 | 23.937 | 122.929 |
| Gradient mean | 0.582 | 1.171 | 14.227 | 29.124 | 130.058 |
| ALP UAV1 | 1.004 | 2.203 | 26.883 | 59.802 | 217.263 |
| ALP UAV2 | 1.145 | 2.822 | 30.420 | 73.678 | 183.117 |
| ALP UAV3 | 0.632 | 1.331 | 16.199 | 33.687 | 140.906 |
| ALP mean | 0.927 | 2.119 | 24.500 | 55.722 | 180.429 |

Gradient is the smoothest by these executed metrics. Native UAV1 contains a large isolated executed velocity-change spike; it is reported rather than clipped.

## Final fields

`VISIBILITY_STRESS_SCENE:` `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`

`TARGET_STATIC_COLLISION_FREE:` YES

`TARGET_MIN_STATIC_CLEARANCE:` `0.344444 m` continuous geometry; runtime minimum `0.344471–0.344483 m`

`STATIC_COLLISION_FREE_BYPASS_EXISTS:` YES for V1/V2/V3 under the `0.65 m` inflated-UAV contract

`VISIBILITY_GATES:` V1 `LEFT_VISIBILITY_OPEN`; V2 `RIGHT_VISIBILITY_OPEN`; V3 `DWELL_OCCLUSION/LEFT_OPEN`

`NATIVE_VISIBILITY:` UAV1 98.75%, UAV2 96.00%, UAV3 96.55%, all-3 92.43%

`GRADIENT_VISIBILITY:` UAV1 95.63%, UAV2 96.44%, UAV3 96.85%, all-3 91.13%

`ALP_VISIBILITY:` UAV1 94.21%, UAV2 96.36%, UAV3 96.90%, all-3 88.88%

`ALP_VISIBILITY_ADVANTAGE_FROM_LOS_TOPOLOGY:` NO

`FAIR_SCENE_FROZEN_BEFORE_BENCHMARK:` YES; SHA256 identical in all three run metadata records

`NATIVE_RESULT:` complete; 3 dynamic collision episodes / 40 samples; all-3 visibility 92.43%; static collision 0

`GRADIENT_RESULT:` complete; 4 dynamic collision episodes / 58 samples; all-3 visibility 91.13%; static collision 0

`ALP_RESULT:` complete; 1 dynamic collision episode / 7 samples; all-3 visibility 88.88%; static collision 0

