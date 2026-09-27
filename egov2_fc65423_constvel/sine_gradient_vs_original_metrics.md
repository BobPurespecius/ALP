# EGOv2 baseline vs sinusoidal gradient metrics

Date: 2026-08-20. Both experiments used the isolated `fc65423` workspace,
the same long-cylinder-forest scene, three UAVs, target waypoints, speed,
and a 30-second evaluation window. The baseline is the original EGOv2
planning chain with current moving-obstacle point clouds only. The gradient
run adds the sinusoidal future-position cost. `plan_success` is intentionally
not used as an evaluation metric.

The visibility evaluator and odometry recorder are external to EGOv2 and do
not publish planning commands or alter the map. Trajectory derivatives below
exclude the first 2 seconds of takeoff transient; visibility uses the full
target-motion window.

## Visibility and outages

| Metric | Original EGOv2 | Sinusoidal gradient |
|---|---:|---:|
| Visibility samples | 900 | 900 |
| Mean visible UAVs | 2.679 | 2.744 |
| All three visible ratio | 78.11% | 79.22% |
| All three visible samples | 703 | 713 |
| All three invisible ratio | 2.78% | 1.11% |
| All three invisible samples | 25 | 10 |
| Total all-invisible duration | 0.767 s | 0.267 s |
| Longest all-invisible outage | 0.433 s | 0.167 s |
| All-invisible outage count | 2 | 2 |
| All-invisible outages/minute | 4.00 | 4.00 |
| UAV1 visibility ratio | 96.67% | 98.22% |
| UAV2 visibility ratio | 92.44% | 95.89% |
| UAV3 visibility ratio | 78.78% | 80.33% |
| UAV3 longest outage | 1.967 s | 1.903 s |

The key robustness result is the reduction in complete team loss of target
visibility: 25 to 10 samples, and 0.767 s to 0.267 s total duration. UAV3
remains the weakest individual sensor viewpoint in both modes.

## Tracking and formation

| Metric | Original EGOv2 | Sinusoidal gradient |
|---|---:|---:|
| Team centroid mean target error | 2.645 m | 1.719 m |
| Team centroid RMS target error | 3.349 m | 1.970 m |
| Team centroid max target error | 8.115 m | 5.309 m |
| Mean pairwise UAV spacing | 1.220 m | 1.145 m |
| Pairwise spacing standard deviation | 0.115 m | 0.122 m |
| Target path length in window | 25.934 m | 25.936 m |
| Target straight-line displacement | 23.687 m | 23.677 m |

Per-UAV RMS target-distance error was 3.426/3.405/3.461 m for the original
mode and 2.100/1.981/2.187 m for the gradient mode (UAV1/UAV2/UAV3).

## Path and smoothness

| UAV | Mode | Path length | Path inefficiency | RMS acceleration | RMS jerk | Max jerk |
|---|---|---:|---:|---:|---:|---:|
| 1 | Original | 29.45 m | 1.089 | 1.237 m/s2 | 33.26 m/s3 | 289.64 m/s3 |
| 2 | Original | 30.40 m | 1.108 | 1.501 m/s2 | 38.90 m/s3 | 282.00 m/s3 |
| 3 | Original | 28.92 m | 1.091 | 1.278 m/s2 | 37.07 m/s3 | 354.04 m/s3 |
| 1 | Gradient | 29.39 m | 1.094 | 1.139 m/s2 | 31.00 m/s3 | 206.57 m/s3 |
| 2 | Gradient | 30.46 m | 1.109 | 1.499 m/s2 | 41.21 m/s3 | 360.30 m/s3 |
| 3 | Gradient | 29.49 m | 1.115 | 1.169 m/s2 | 31.95 m/s3 | 527.27 m/s3 |

Path efficiency is nearly unchanged. The gradient mode lowers UAV1/UAV3 RMS
acceleration and jerk, but increases UAV2 jerk and the UAV3 peak jerk. Peak
values are sensitive to controller/sample transitions, so RMS jerk and
integrated squared jerk are the more stable comparison quantities. Full
values, including integrated squared jerk, are in `metrics_*json`.

## Safety and clearance

Clearance is geometric distance from the odometry point to the nearest
cylinder surface; the 0.5 m threshold is reported as a near-obstacle
indicator, not a collision claim.

| Metric | Original EGOv2 | Sinusoidal gradient |
|---|---:|---:|
| Team minimum static clearance | 0.334 m | 0.307 m |
| Team minimum moving clearance | 0.339 m | 0.474 m |
| Lowest static clearance time below 0.5 m | 3.07 s (UAV1) | 2.20 s (UAV1) |
| Lowest moving clearance time below 0.5 m | 1.43 s (UAV2) | 1.27 s (UAV2) |

No physical collision counter is inferred from planner rejection messages.
The clearance values are the appropriate external safety indicator for this
non-Gazebo fake-drone simulation.

## Reproduction

```bash
python3 ros_ws/src/multi_uav_formation/scripts/analyze_native_egov2_metrics.py \
  --visibility-csv metrics_baseline_visibility.csv \
  --trajectory-csv metrics_baseline_trajectory.csv \
  --output metrics_baseline.json

python3 ros_ws/src/multi_uav_formation/scripts/analyze_native_egov2_metrics.py \
  --visibility-csv metrics_sine_visibility.csv \
  --trajectory-csv metrics_sine_trajectory.csv \
  --output metrics_sine.json
```

The recorder is enabled by `NATIVE_EGOV2_TRAJECTORY_CSV` in
`run_constvel_gradient_rviz.sh`. It records odometry only; it is not part of
the EGOv2 planner or controller.

## Post-fix validation

The remaining complete-visibility loss was traced to two separate effects:

1. Visibility was sampled at 30 Hz while `dynamic_cylinders` was refreshed at
   5 Hz. The evaluator could therefore use a stale moving-cylinder pose and
   report a one-sample false reacquisition. Visibility now recomputes moving
   cylinder positions at the sample timestamp. The EGOv2 point-cloud update
   rate and planner inputs are unchanged.
2. The target waypoint path crossed moving obstacle 6. The target coordinator
   now loads the same sinusoidal obstacle model and precomputes per-segment
   speeds at target start. Waypoints are unchanged; the closest safe speed is
   selected for each segment. In the tested run only segment 4 was adjusted,
   from `0.928 m/s` to `0.436 m/s`, with a computed target surface clearance
   of `0.309 m` (`0.25 m` target radius plus `0.05 m` margin).

After these changes, the 35-second sinusoidal-gradient validation produced
zero samples with all three UAVs invisible. The visibility CSV contained 954
samples, with `visible_count=0` occurring zero times. No obstacle geometry was
changed and no EGOv2 planner source was modified for this fix.

An initial post-fix comparison was invalid because ROS children from an earlier
run remained alive. EGOv2's UDP bridge binds fixed port 8081 with
`SO_REUSEPORT`; changing `ROS_MASTER_URI` therefore does not isolate two bridge
instances. A stale bridge can consume part of the swarm trajectory broadcast,
which caused UAVs to miss sequential startup. The invalid run is not used
below.

Both modes were rerun only after verifying that no earlier EGOv2 master,
planner, or bridge process remained. Each run also exited fully before the
next started. These are the controlled post-fix results:

These controlled runs remove cross-run process contamination, but they are
still diagnostic rather than a final benchmark. The target coordinator sets
its start clock before synchronously computing the segment-speed schedule.
That computation delayed the first moving-target publication by about two
seconds in the controlled runs, and UAV pursuit began 3.2--5.4 seconds after
the target clock. In the earlier catastrophic rerun the computation took
13.14 seconds under load, after which the target jumped from its initial point
to approximately `(-21.57, 2.71)`; UAV2 and UAV3 never began moving. This is a
target-start synchronization defect in the integration, not evidence of an
original EGOv2 planning limitation.

| Metric | Original EGOv2 | Sinusoidal gradient |
|---|---:|---:|
| Samples | 841 | 843 |
| All three invisible ratio | 0.00% | 0.00% |
| Mean visible UAVs | 2.655 | 2.594 |
| All three visible ratio | 69.08% | 60.74% |
| Team centroid RMS target error | 3.258 m | 1.970 m |
| Mean pairwise spacing | 1.786 m | 1.503 m |
| Team minimum static clearance | 0.135 m | 0.196 m |
| Team minimum moving clearance | 0.551 m | 0.160 m |
| Mean per-UAV path length | 30.401 m | 30.068 m |
| Mean path inefficiency | 1.221 | 1.188 |
| Mean per-UAV RMS acceleration | 2.108 m/s2 | 2.348 m/s2 |
| Mean per-UAV acceleration P95 | 4.105 m/s2 | 4.618 m/s2 |
| Mean per-UAV RMS jerk | 72.20 m/s3 | 106.02 m/s3 |
| Mean per-UAV jerk P95 | 119.97 m/s3 | 138.91 m/s3 |
| Mean per-UAV RMS target distance | 3.477 m | 2.233 m |

Neither controlled run had a geometric cylinder-surface violation. The
gradient mode substantially reduced target tracking error and slightly
reduced path length/inefficiency, but did not improve every visibility or
smoothness measure in this single run: original EGOv2 had a higher all-visible
ratio and lower average acceleration/jerk. Repeated trials are required for a
statistical claim because target/obstacle phase and EGOv2 replanning are not
deterministic across process starts.

## Synchronized-start rerun

The coordinator now waits 2 seconds after all three initial odometry streams
are present, performs the speed preparation while the target remains at the
first waypoint, and only then publishes `target_start_time`. This removes the
target jump and makes the startup sequence identical between modes.

The synchronized original run completed normally:

| Metric | Original EGOv2 |
|---|---:|
| Samples | 858 |
| All three invisible ratio | 0.00% |
| All three visible ratio | 70.63% |
| Team centroid RMS target error | 1.654 m |
| Mean pairwise spacing | 1.250 m |
| Mean per-UAV path length | 27.214 m |
| Mean RMS acceleration | 1.181 m/s2 |
| Mean jerk P95 | 59.3 m/s3 |

The corresponding gradient process generated only 486 visibility samples and
ended after repeated IPOPT line-search failures (`Return = -1005`) and planner
shutdown. Its partial trajectory is therefore not a valid comparison window;
the partial data is retained as `settled_gradient_metrics.json` for diagnosis,
not presented as a performance result.

A second synchronized gradient run reached a 27-second common evaluation
window. On that common window, the original/gradient results were:

| Metric | Original EGOv2 | Sinusoidal gradient |
|---|---:|---:|
| Samples | 810 | 810 |
| All three invisible ratio | 0.00% | 4.69% |
| Longest all-invisible outage | 0 s | 1.233 s |
| All three visible ratio | 68.89% | 72.35% |
| Mean visible UAVs | 2.606 | 2.599 |
| Team centroid RMS target error | 1.665 m | 1.712 m |
| Mean pairwise spacing | 1.264 m | 1.165 m |
| Mean path inefficiency | 1.079 | 1.095 |
| Mean RMS acceleration | 1.211 m/s2 | 1.266 m/s2 |
| Mean jerk P95 | 63.5 m/s3 | 65.9 m/s3 |
| Mean RMS target distance | 1.839 m | 1.852 m |

The gradient run had no static-cylinder clearance violation, while its minimum
moving clearance was `0.070 m`; the baseline also had no static violation but
reached `0 m` moving clearance for UAV2. These are single-run measurements,
not a claim that the gradient method dominates the original in every metric.
