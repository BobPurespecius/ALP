# ALP Command–Odometry 0.84 m Root-Cause Report

Date: 2026-09-01
Scenario: `long_cylinder_forest.json`
Scope: read-only audit of the existing obstacle-9 execution artifact. No planner,
SIDE/A*/SFC, MINCO/SCP, trust-region, OSQP, jerk, yaw/FOV, controller gain, or
safety parameter was changed, and no new simulation was required.

## Executive conclusion

The reported approximately `0.840 m` command–odometry difference is **not a
validated persistent control-tracking error**. It was produced by associating
records with the wrong time/trajectory identity (and, in the earlier analysis,
by relying on a reusable integer `trajectory_id`). The strict association is:

```text
absolute command time = TrajServer command_time
absolute odometry time = target_start_time + trajectory_csv.time_s
target_start_time = 1788256472.986711 s
trajectory identity = start_time (+ generation when available), never ID alone
```

With this association, the command and odometry streams are normally close. The
global synchronized position-error statistics are approximately P50 `0.038 m`,
P90 `0.068 m`, P95 `0.090 m`, and a robust maximum `0.359 m` (malformed
concatenated throttled-log records are excluded). The best delay scan result is
about `+0.022 s`, with RMSE changing from about `0.0598 m` at zero delay to
`0.0553 m`; this does not support a 0.25–0.5 s temporal lag.

During the obstacle-9 encounter (approximately elapsed `72.5–74.8 s`), the
synchronized position error is approximately P50 `0.153 m`, P95 `0.301 m`,
maximum `0.321 m`, with RMSE `0.172 m`. The previously quoted `0.840 m` at the
old “closest odometry” sample becomes approximately `0.043 m` after strict
target-clock alignment.

The remaining 0.04–0.32 m execution difference is real sampled tracking error,
but the available artifact does not identify whether it is caused by controller
force/attitude saturation, simulator motor/thrust response, or another internal
closed-loop dynamic effect. No controller-internal diagnostic fields were
recorded in this run.

## 1. Data and time-base audit

Primary files:

- `obstacle9_chain_20260901.launcher.log`
- `obstacle9_chain_20260901_visibility_trajectory.csv`
- ROS logs under `/home/bob/.ros/log/1d8d3834-a5eb-11f1-a7e5-b74b79e726c8/`
- `ros_ws/src/multi_uav_formation/scripts/native_egov2_rviz_scene.py`
- `ros_ws/src/ego-planner/src/uav_simulator/so3_control/src/so3_control_nodelet.cpp`
- `ros_ws/src/ego-planner/src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp`

The target coordinator explicitly logged:

```text
Target clock started at 1788256472.986711
```

The trajectory CSV `time_s` is elapsed time from that target clock. The dynamic
scene has a separate motion epoch, published through `/dynamic/motion_start_time`;
it must not be substituted for the target clock when matching command and odom
samples. The CSV's moving-clearance field is sampled/collector-derived and is
clipped at zero in this artifact, so negative collision depth was recomputed
offline rather than inferred from that column alone.

## 2. Planner polynomial → PositionCommand

`traj_server.cpp` stores the received polynomial, `start_time_`, duration,
piece count, and trajectory identity in `polyTrajCallback()`. `cmdCallback()`
evaluates the same polynomial at:

```text
t_cur = ros::Time::now() - start_time_
```

and publishes position, velocity, acceleration, jerk-related state, and yaw in
the `world` frame. Planner publish and TrajServer receive records in the chain
match on start time, duration, and piece count. No stale-trajectory, local-time,
sample-overrun, transport, or frame conversion discrepancy was found.

The throttled `[traj-server-command]` log is emitted at approximately 9 Hz
(`ROS_INFO_THROTTLE(0.1)`); it is not the true command publication rate. The
actual timer path is approximately 100 Hz. A small number of launcher lines are
interleaved by concurrent ROS logging; those malformed records were excluded
from numerical statistics.

Observed rates for this artifact are therefore:

```text
PositionCommand diagnostic log: ~9.1 Hz (throttled observation only)
TrajServer command timer:       ~100 Hz
scene odometry CSV recorder:    ~30 Hz
simulator odometry publisher:   200 Hz (launch parameter)
controller update:              odometry callback rate; no independent counter logged
```

The lower CSV/log rates are observers, not evidence that the command topic ran
at 9 Hz. No command-drop or queue-backlog evidence appears in the ROS
master/topic records.

```text
PLANNER_TO_POSITION_COMMAND_ERROR_P95: approximately 0 m (same polynomial/time base)
PLANNER_TO_POSITION_COMMAND_ERROR_MAX: no measured non-zero discrepancy; exact
  residual requires an unthrottled command recording
```

This is a source/runtime consistency result, not a claim of a strict continuous
bound from the throttled text log.

## 3. PositionCommand → SO3 controller input

The active controller source directly assigns the incoming command fields:

```text
des_pos_ = cmd->position
des_vel_ = cmd->velocity
des_acc_ = cmd->acceleration
des_yaw_ = cmd->yaw
des_yaw_dot_ = cmd->yaw_dot
```

The odometry callback updates the controller state and then publishes the SO3
command. Topic inspection for this run shows the expected chain:

```text
/drone_0_planning/pos_cmd
  publisher /drone_0_traj_server
  subscriber /drone_0_so3_control
/drone_0_so3_cmd
  publisher /drone_0_so3_control
  subscriber /drone_0_quadrotor_simulator_so3
/drone_0_visual_slam/odom
  publisher /drone_0_quadrotor_simulator_so3
  subscriber /drone_0_so3_control
```

No frame transform or command-field rewrite exists between PositionCommand and
the controller desired state. However, this run has no controller-side log of
the received desired vector, so an empirical per-message residual cannot be
computed from the artifact alone.

```text
POSITION_COMMAND_TO_CONTROLLER_INPUT: MATCH (source/topic audit)
```

All four layers use the `world`/ENU position convention in the inspected path;
no ENU/NED, body/world, or quaternion-to-position conversion is present.
Frame audit: `PASS`.

## 4. Controller desired → odometry

After strict absolute-time interpolation, the command/odom statistics are:

| Window | Position error P50 | P90 | P95 | max | RMSE |
|---|---:|---:|---:|---:|---:|
| Full synchronized run | 0.038 m | 0.068 m | 0.090 m | 0.359 m | 0.060 m |
| Obstacle-9 encounter | 0.153 m | 0.277 m | 0.301 m | 0.321 m | 0.172 m |

Velocity error over the full synchronized run is approximately P50 `0.035 m/s`,
P95 `0.165 m/s`, max `1.34 m/s`; in the obstacle-9 window it is approximately
P50 `0.222 m/s`, P95 `0.376 m/s`, max `0.431 m/s`.

The obstacle-9 sample used by the old report (`t≈73.883918 s`, UAV1) is
approximately:

```text
odometry position  = (28.8333, 0.2754, 1.5309)
aligned command    = (28.7954, 0.2736, 1.5316)
position difference ≈ 0.043 m
```

This directly falsifies the earlier 0.840 m pairing as a same-time command/odom
measurement.

## 5. Delay scan and error mode

The scan compared the synchronized command trajectory against odometry for
`Δt ∈ [-0.5,+0.5] s`:

```text
BEST_COMMAND_ODOM_DELAY: +0.022 s (approximately)
RMSE_ZERO_DELAY:         0.0598 m (approximately)
RMSE_BEST_DELAY:         0.0553 m (approximately)
```

The small optimum is consistent with sampling and callback scheduling. It does
not support a large command/odometry temporal lag. The error is not a long-lived
0.5–0.8 m offset. It is best classified as a moderate high-dynamics execution
tracking difference near the encounter, with the historical 0.840 m value being
a time/identity association artifact.

## 6. Obstacle-9 clearance

Obstacle 9 is `long_forest_moving_9`:

```text
base center = (29.0, 1.2)
radius      = 0.28 m
axis        = (0, 1)
amplitude   = 1.2 m
period      = 6.0 s
phase       = 5.76 rad
```

Using the execution chain's established motion-phase association and the same
sampled center-distance-minus-radius contract used by the prior collision
analysis:

```text
COMMAND_MIN_DYNAMIC_CLEARANCE: approximately +0.006 m (sampled)
ODOM_MIN_DYNAMIC_CLEARANCE:    approximately -0.1828 m (sampled)
```

The command path was therefore marginally outside the moving obstacle while the
executed odometry entered it. The command clearance is only about 6 mm, so it
has essentially no robustness margin; this report does not change that margin.
Because the CSV's `moving_clearance_m` field is clipped at zero, the negative
odom value comes from the offline geometric recomputation, not from the clipped
CSV column.

## 7. Yaw coupling and unresolved internal cause

The run has target-facing yaw enabled in the scene/visualization layer, and the
collector records actual yaw. It does not record the commanded yaw together with
the trajectory samples, nor does it record controller roll/pitch, thrust, force,
motor response, or saturation flags. Therefore a correlation between the
remaining 0.04–0.32 m position error and yaw rate cannot be established from
this artifact.

The active controller/simulator executable was resolved from the companion
`/home/bob/ALP/guidance/ros_ws` devel tree; the corresponding ALP source was
inspected for the same callback/data-flow semantics. This matters for any future
diagnostic rebuild, but does not alter this read-only conclusion.

```text
BODY_YAW_COUPLING_CORRELATED: UNRESOLVED
```

This must not be promoted to a causal yaw/controller diagnosis.

## 8. First-error-layer classification

| Layer | Result | Basis |
|---|---|---|
| Planner polynomial → TrajServer command | PASS | Same start time/duration/pieces; same polynomial evaluation rule |
| TrajServer → controller desired input | MATCH by source/topic audit | Direct field copy; no transform/rewrite found |
| Controller desired → odometry | Real sampled tracking error remains | 0.04–0.32 m after strict alignment |
| Timestamp/trajectory association | **FAIL in the old analysis** | Reused integer ID / wrong epoch produced the 0.840 m pairing |
| Controller internal saturation/attitude/motor cause | UNRESOLVED | Required fields were not logged |

```text
FIRST_ERROR_LAYER: TIME_ALIGNMENT (historical analysis artifact)
DOWNSTREAM_REAL_ERROR_LAYER: CONTROLLER_OR_SIMULATOR_EXECUTION (unresolved)
TRACKING_ERROR_MODE: HIGH_DYNAMICS (moderate encounter-local error), not sustained 0.84 m
SOURCE_LEVEL_ROOT_CAUSE: wrong time/trajectory association in the 0.840 m comparison
```

There is no evidence in this artifact for a planner-to-command bug, ROS
transport loss, stale TrajServer activation, frame mismatch, or a large fixed
command delay.

## 9. Required final fields

```text
STRICT_TIME_ALIGNMENT: PASS
PLANNER_TO_POSITION_COMMAND_ERROR_P95: approximately 0 m (same polynomial/time base)
PLANNER_TO_POSITION_COMMAND_ERROR_MAX: not observable from throttled command text; no discrepancy found
POSITION_COMMAND_TO_CONTROLLER_INPUT: MATCH
COMMAND_ODOM_ERROR_P50: approximately 0.038 m
COMMAND_ODOM_ERROR_P95: approximately 0.090 m
COMMAND_ODOM_ERROR_MAX: approximately 0.359 m (robust, malformed log rows excluded)
BEST_COMMAND_ODOM_DELAY: approximately +0.022 s
RMSE_ZERO_DELAY: approximately 0.0598 m
RMSE_BEST_DELAY: approximately 0.0553 m
COMMAND_MIN_DYNAMIC_CLEARANCE: approximately +0.006 m
ODOM_MIN_DYNAMIC_CLEARANCE: approximately -0.1828 m
FIRST_ERROR_LAYER: TIME_ALIGNMENT (historical association); downstream sampled execution error is CONTROLLER_OR_SIMULATOR
TRACKING_ERROR_MODE: HIGH_DYNAMICS / MODERATE_ENCOUNTER_LOCAL, not sustained 0.84 m
BODY_YAW_COUPLING_CORRELATED: UNRESOLVED
SOURCE_LEVEL_ROOT_CAUSE: wrong time base and reusable trajectory-ID association created the 0.840 m pseudo-error
CODE_BUG_FOUND: NO (no production code bug established by this audit)
CODE_BUG_FIXED: NOT_NEEDED
OBSTACLE9_COLLISION_AFTER_FIX: not rerun in this read-only audit; historical chain remains collision
BUILD: NOT_NEEDED
RUNTIME: PASS (existing artifact was cleanly shut down; no new run required)
```

## Final answer to the requested questions

1. The approximately 0.84 m value is primarily a **measurement/association
   artifact**, not a proven sustained command–odometry tracking failure.
2. A smaller real error remains near obstacle 9, but the existing logs do not
   prove controller saturation, thrust limitation, attitude lag, or motor
   response as its internal cause.
3. A large temporal lag is not supported; the best delay is only about 22 ms.
4. No planner or TrajServer execution bug was found in the strict audit.
5. The obstacle-9 collision remains real at the odometry layer, while the
   command trajectory is only marginally clear. This report does **not** claim
   that the collision itself is solved.

`NO EVIDENCE OF A SUSTAINED 0.84 m COMMAND–ODOMETRY ERROR.`
