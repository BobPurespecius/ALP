# ALP Body-Yaw Position-Effect and FOV-Animation Report

Date: 2026-08-31  
Scene: `long_cylinder_forest.json`  
Comparison: identical current launch configuration, `enable_target_facing_yaw=false` (OFF) versus `true` (ON), RViz disabled.

## Result

`BODY_YAW_POSITION_EFFECT = NEGLIGIBLE` in the engineering sense required for this decision. The measured odometry traces differ between the two short runs, but the difference has no consistent degradation direction, no collision regression, and no tracking regression. The position planner remains independent of yaw, so the current body-yaw implementation is retained.

## A/B measurements

The trajectory CSV records executed odometry, not a separate position-command stream. Source inspection provides the planner-side result: `traj_server` evaluates position (`p/v/a/j`) from the same trajectory in both modes and only changes the yaw fields; the position planner does not read yaw. Therefore:

`PLANNER_POSITION_TRAJECTORY_CHANGED = NO` (source-level; direct command RMSE was not emitted by the existing CSV schema).

For executed odometry, traces were aligned by relative tracking time over each pair's common interval:

| UAV | Odom position RMSE (m) | P95 (m) | Max (m) | Path OFF (m) | Path ON (m) | Matched-progress path change |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 0.264 | 0.624 | 1.042 | 35.276 | 35.714 | +1.35% |
| 2 | 0.298 | 0.767 | 0.861 | 36.553 | 36.048 | −1.26% |
| 3 | 0.298 | 0.684 | 1.293 | 34.374 | 35.285 | +2.65% |

These are run-to-run executed-state differences, not a commanded-path change. Their signs are mixed and they do not coincide with a safety or tracking deterioration.

The A/B was still necessary: the SO3 simulator consumes external yaw when constructing the attitude/force model, so a body-yaw change could in principle perturb executed position through controller transients even though the planner's position command is unchanged. In this run that coupling did not produce a repeatable directional or safety/tracking penalty.

## Safety, tracking, dynamics, and runtime

| Metric | OFF | ON | Assessment |
|---|---:|---:|---|
| Minimum static clearance (m) | 0.200 | 0.202 | no static collision in either run |
| Minimum moving clearance (m) | 0.0046 | 0.0225 | no sampled dynamic collision; both are close approaches |
| Mean target distance, UAV1/2/3 (m) | 1.666 / 1.758 / 1.746 | 1.615 / 1.768 / 1.692 | mixed, no systematic tracking regression |
| Target-distance P95, UAV1/2/3 (m) | 2.405 / 2.493 / 2.383 | 2.411 / 2.442 / 2.259 | mixed |
| Max speed observed (m/s) | 2.06 / 1.87 / 2.06 | 2.10 / 1.86 / 2.05 | below the 3 m/s configured limit |
| Plan-success duration mean (s) | 0.882 | 0.842 | not a controlled latency comparison; no yaw-specific stall evidence |
| Lost-heartbeat messages | 0 | 0 | no runtime stall |

Acceleration and jerk values reconstructed by finite-differencing odometry velocity are estimator/sampling-sensitive (and are not the planner's analytic extrema). Their maxima did not show a consistent ON increase: OFF/ON max acceleration was 6.32/6.41, 8.31/5.88, 6.63/7.35 m/s² for UAV1/2/3; finite-difference jerk was 58.4/59.8, 88.7/63.5, 64.1/70.0 m/s³. These numbers are not used to claim a continuous-time dynamics guarantee.

## Visibility effect

The ON run produced target-facing commands and executed odometry yaw; OFF retained the legacy zero command. Visibility samples:

| Metric | OFF | ON |
|---|---:|---:|
| Samples | 1074 | 1162 |
| UAV1 visibility ratio | 90.41% | 96.99% |
| UAV2 visibility ratio | 94.04% | 94.23% |
| UAV3 visibility ratio | 83.33% | 100.00% |
| All three visible | 70.30% | 93.46% |
| Zero visible | 0 | 0 |

ON command-to-target bearing error was small (mean absolute 1.90°, 1.15°, 2.06° for UAV1/2/3 in the new run). OFF command yaw and yaw rate were exactly zero. No static or dynamic collision regression was observed.

## FOV animation audit

The implementation is a live ROS visualization path:

- `visualization_msgs/MarkerArray` is published on `/native_egov2/camera_fov` every 1/30 s.
- Each UAV uses a stable namespace (`alp_camera_fov/uav1`, `uav2`, `uav3`) and marker ID `0`; updates replace the same marker rather than creating an unbounded stream of IDs.
- Marker origin is the current UAV odometry position.
- Forward ray uses executed odometry yaw; boundary rays are yaw `+42.5°` and `−42.5°`.
- Range is fixed at 8 m with a 16-segment horizontal arc.
- RViz subscribes to the same MarkerArray topic via the `Camera FOV (executed yaw)` display.
- Visibility uses the same executed odometry yaw and wrapped relative bearing, so the displayed sector and metric use one camera-axis definition.

The shortest-angle wrap is used in both target-facing control and visibility. An offline check of the wrap at `179° → −179°` yields a `+2°` step (and the reverse yields `−2°`), so the representation does not command a one-turn jump. The ON trajectory's largest observed wrapped sample-to-sample yaw step was 6.48°; this is consistent with bounded updates, not a π-wrap discontinuity.

RViz was disabled for the smoke runs, so this report verifies the marker topic/configuration and real-time publication path statically and through the running node; it does not claim a GUI screenshot capture.

## Decision

No independent-gimbal rollback is justified by the evidence. The current target-facing body-yaw layer is retained because position commands are isolated by design, executed-path differences are mixed/no-directional, safety and tracking are not degraded, and visibility improves materially.

```text
BODY_YAW_POSITION_EFFECT: NEGLIGIBLE
PLANNER_POSITION_TRAJECTORY_CHANGED: NO
COLLISION_REGRESSION: NO
TRACKING_REGRESSION: NO
KEEP_BODY_TARGET_FACING_YAW: YES
INDEPENDENT_GIMBAL_IMPLEMENTED: NO
BODY_YAW_RESTORED_TO_LEGACY: NO
VISIBILITY_USES_CAMERA_YAW: YES (current camera axis = executed body yaw)
RVIZ_FOV_USES_CAMERA_YAW: YES
FOV_ANIMATION_REALTIME: YES
FOV_ANIMATION_WRAP_CONTINUOUS: YES
BUILD: PASS
RUNTIME: PASS
```
