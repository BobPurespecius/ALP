# SPEED50 Repeatability and Metrics Report

Date: 2026-09-01. Scene: `long_cylinder_forest.json`, obstacle 9 period 12.0 s, target-facing yaw ON, RViz OFF. Native n=2, Gradient n=2, ALP n=4; all runs serial and reached the final target.

## Cutoff and measurement contract

The runner detected `(target_x,target_y)=(36.0,0.25)` with zero target velocity for all three UAV records for six consecutive samples, then sent SIGINT. Metric cutoffs were 83.589–84.140 s elapsed. Post-target WAIT_TARGET/hover data are excluded.

`native_egov2_rviz_scene.py:665-704` writes pose/twist directly from UAV odometry. No saved artifact contains complete active MINCO polynomial coefficients and piece partitions. Therefore planned analytic MINCO smoothness cannot be reconstructed for any version. All jerk/acceleration values below are explicitly **executed-odometry finite-difference** metrics, not planned jerk. The previous ALP jerk P95=42.79 was this same odometry metric; it does not contradict planner `max_jer=22`.

## Repeated summary (mean; min/max/std in parentheses)

| Metric | Native n=2 | Gradient n=2 | ALP n=4 |
|---|---:|---:|---:|
| Task completion | 2/2 | 2/2 | 4/4 |
| Collision episodes/run | 2.00 (2/2/0) | 3.00 (3/3/0) | 0.00 (0/0/0) |
| Collision samples/run | 33.5 (31/36/2.5) | 40.5 (38/43/2.5) | 0 (0/0/0) |
| Obstacle 9 safe runs | 0/2 | 0/2 | 4/4 |
| Unsafe episodes/run | 6.50 (6/7/0.5) | 6.00 (5/7/1.0) | 2.50 (1/4/1.12) |
| Min dynamic clearance (m) | 0.000 | 0.000 | 0.200 (0.076/0.395/0.125) |
| Static collision episodes/run | 0 | 0 | 0 |
| Min static clearance (m) | 0.210 (0.206/0.214) | 0.207 (0.195/0.218) | 0.186 (0.178/0.195) |
| Tracking mean (m) | 1.549 (1.474/1.625/0.076) | 1.458 (1.455/1.461/0.003) | 1.443 (1.430/1.455/0.009) |
| Tracking P95 (m) | 2.094 (1.655/2.533) | 1.631 (1.626/1.635) | 1.659 (1.626/1.730) |
| Tracking max (m) | 4.352 | 2.145 | 2.171 |
| Mean path length (m) | 77.012 (76.489/77.535) | 76.391 (76.137/76.645) | 77.253 (76.622/77.523) |
| Latency P95 (ms) | 0.659 (0.614/0.705) | 0.773 (0.755/0.790) | 0.780 (0.768/0.791) |
| UAV1 visibility | 97.67% | 98.01% | 97.82% |
| UAV2 visibility | 98.46% | 98.19% | 99.37% |
| UAV3 visibility | 96.90% | 98.13% | 99.40% |
| All-3 visibility | 94.10% | 95.26% | 96.88% |
| At-least-2 visibility | 98.94% | 99.07% | 99.70% |
| At-least-1 visibility | 100.00% | 100.00% | 100.00% |
| None-visible | 0.00% | 0.00% | 0.00% |

Values are computed after the per-run target-stop cutoff. Dynamic collision is clearance `<=0`; unsafe is `<0.5 m`.

## Smoothness (executed odometry only)

| Metric (mean of UAV values per run) | Native | Gradient | ALP |
|---|---:|---:|---:|
| Velocity max (m/s) | 2.82 | 1.83 | 2.36 |
| Acceleration max (m/s²) | 8.26 | 6.21 | 6.90 |
| Jerk RMS (m/s³) | 37.07 | 20.23 | 25.75 |
| Jerk P95 (m/s³) | 43.66 | 44.37 | 55.04 |
| Jerk max (m/s³) | 931.6 | 188.4 | 225.6 |

`PLANNED_JERK_RMS/P95/MAX`: N/A for all versions because full polynomial samples were not recorded. `SMOOTHNESS_MEASUREMENT_VALID`: YES for execution, NO for planned polynomial.

## Obstacle 6/9 per-UAV results

Native obstacle 9: UAV1 min 1.414 m, UAV2 min 0.447 m (unsafe only), UAV3 collision (min 0, 16–19 samples/run). Gradient obstacle 9: UAV1 min 1.398 m, UAV2 min 0.315 m (unsafe only), UAV3 collision (min 0, 18–19 samples/run). ALP obstacle 9: no collision in 4/4; minimum per run remained positive (UAV3 minimum 0.257–0.498 m; UAV2 minimum 0.108–0.964 m).

Obstacle 6 had no collision in any run. Native and Gradient had unsafe-only UAV1/UAV2 encounters; ALP had unsafe-only UAV2 in some runs, no collision.

## Collision Context

| Run | UAV | Obstacle | Window (elapsed s) | Min clearance | Relative motion | Planning state |
|---|---:|---:|---:|---:|---|---|
| native_1 | 3 | 8 | 51.662–52.128 | 0 | CROSSING | normal tracking; risk/SIDE disabled |
| native_1 | 3 | 9 | 73.595–74.095 | 0 | CROSSING | normal tracking; risk/SIDE disabled |
| native_2 | 3 | 8 | 51.670–52.169 | 0 | CROSSING | normal tracking; risk/SIDE disabled |
| native_2 | 3 | 9 | 73.569–74.169 | 0 | CROSSING | normal tracking; risk/SIDE disabled |
| gradient_1 | 2 | 6 | 11.982–12.241 | 0 | CROSSING | normal tracking; gradient cost only |
| gradient_1 | 3 | 8 | 51.641–52.107 | 0 | CROSSING | normal tracking; gradient cost only |
| gradient_1 | 3 | 9 | 73.541–74.141 | 0 | CROSSING | normal tracking; gradient cost only |
| gradient_2 | 2 | 6 | 12.089–12.223 | 0 | CROSSING | normal tracking; gradient cost only |
| gradient_2 | 3 | 8 | 51.659–52.123 | 0 | CROSSING | normal tracking; gradient cost only |
| gradient_2 | 3 | 9 | 73.489–74.058 | 0 | CROSSING | normal tracking; gradient cost only |

Closest-approach positions/velocities and obstacle states are recorded in `speed50_repeatability_20260901/repeatability_metrics.json`. ALP has no collision episodes; its obstacle-9 encounters were handled by risk-triggered SIDE/SCP candidates (NOMINAL→SIDE/KEEP-PREVIOUS-SAFE as logged), with positive executed clearance.

## Final labels

TARGET_STOP_METRIC_CUTOFF: PASS  
POST_TARGET_IDLE_INCLUDED: NO  
SMOOTHNESS_SOURCE: executed odometry twist + finite differences; planned MINCO analytic derivatives unavailable  
SMOOTHNESS_MEASUREMENT_VALID: YES for execution / NO for planned polynomial  
OLD_ALP_JERK_42_79_EXPLAINED: YES  
NATIVE_OBSTACLE9_SAFE: 0/2  
GRADIENT_OBSTACLE9_SAFE: 0/2  
ALP_OBSTACLE9_SAFE: 4/4  
NATIVE_ALL3_VISIBILITY: 94.10%  
GRADIENT_ALL3_VISIBILITY: 95.26%  
ALP_ALL3_VISIBILITY: 96.88%  
NATIVE_NONE_VISIBLE: 0.00%  
GRADIENT_NONE_VISIBLE: 0.00%  
ALP_NONE_VISIBLE: 0.00%  
BUILD: NOT NEEDED (no source changes)  
RUNTIME: PASS (8 serial runs, target-stop cutoff, exit code 0)  
