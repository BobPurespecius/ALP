# SIDE Lifecycle and Tracking Error Report

Date: 2026-09-01  
Scenario: `long_cylinder_forest.json`  
Scope: lifecycle fix first, then command/odometry audit. No SIDE/A*/SFC/MINCO/SCP/OSQP or safety parameters were changed.

## Phase A — lifecycle fix

The source-level defect was in `TrajContainer::setGlobalTraj()`. Every global-guide update reset `local_traj.duration` to `0.0`, reset `local_traj.traj_id` to `0`, and cleared `drone_id`. The currently executing local trajectory itself was not removed, but the persistence validator used those reset fields as its validity gate. Consequently an accepted/published SIDE trajectory could be reported as `NO_PREVIOUS_ACTIVE` during a later risk cycle.

The minimal fix removes those local-state resets. Global trajectory replacement now updates only `global_traj`; the local execution object retains its own start time, duration, polynomial, and ID until it naturally expires or is explicitly invalidated. Expiry, current-phase slicing, dynamics, static, dynamic, and swarm revalidation remain unchanged.

Planner lifecycle identity is now logged as `(local start_time, generation)` rather than relying on the reusable integer trajectory ID. Added lifecycle events cover commit, accepted, published, activated, validated, retained, superseded, and invalidated paths.

Relevant source files:

- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/include/traj_utils/plan_container.hpp`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp`

## Build

```text
catkin build traj_opt ego_planner path_searching -j2 \
  --cmake-args -Dosqp_DIR=/home/bob/ALP/egov2_fc65423_constvel/local_osqp_debs/extracted/opt/ros/noetic/lib/cmake/osqp
BUILD: PASS
OSQP linkage: OK
```

## Runtime validation

`lifecycle_fix_final_smoke_20260901.launcher.log` (23.2 s, RViz off, isolated ROS master, SIGINT) produced:

```text
COMMITTED=465
ACCEPTED=464
PUBLISHED=471
ACTIVATED=464
VALIDATED=11
RETAINED=7
SUPERSEDED=460
NO_PREVIOUS_ACTIVE=0
KEEP_PREVIOUS_SAFE=7
```

The log contains SIDE follow-up cycles with `previous_type=SIDE_*` followed by successful validation and `KEEP_PREVIOUS_SAFE`; there is no post-fix `NO_PREVIOUS_ACTIVE` event for an active SIDE trajectory. No state contamination, dimension mismatch, segmentation fault, double free, or OSQP crash occurred. SIGINT cleanup left no `roscore`, `rosmaster`, `roslaunch`, planner, TrajServer, simulator, or scene processes.

After the final logging rebuild, `lifecycle_fix_postbuild_smoke_20260901.launcher.log` (20 s) produced `COMMITTED=271`, `ACCEPTED=271`, `PUBLISHED=280`, `ACTIVATED=277`, `VALIDATED=22`, `RETAINED=9`, `SUPERSEDED=268`, `INVALIDATED=14`, and `NO_PREVIOUS_ACTIVE=0`. This confirms the complete selected → published → received/activated → validated/retained trace in the rebuilt executable.

The longer post-fix run (`lifecycle_fix_full_20260901.launcher.log`, 112.9 s recorded trajectory) also showed repeated SIDE persistence and clean shutdown. Its only `NO_PREVIOUS_ACTIVE` was the initial nominal startup case (`previous_type=NOMINAL`), not a lost SIDE trajectory.

## Phase B — command versus odometry

The existing obstacle-9 chain artifact was re-associated by absolute execution time using `start_time + local_time`, with start time (not integer `trajectory_id`) as the identity key. Planner publish and TrajServer receive records matched on start time, duration, and piece count. The first downstream discrepancy was execution tracking, not ROS transport or TrajServer activation:

```text
command trajectory min obstacle-9 clearance: approximately +0.006 m (sampled)
odometry min obstacle-9 clearance:          approximately -0.1828 m (sampled)
closest command/odometry position error:    approximately 0.840 m
```

The command stream was therefore marginally outside the moving obstacle while the simulated odometry entered it. This classifies the remaining discrepancy as `CONTROLLER_TRACKING_FAILURE` / dynamic tracking limitation, after the lifecycle bug, rather than a publish/receive loss or trajectory-ID association error. The command path itself has only millimetres of clearance at closest approach, so it has little execution robustness margin; no safety threshold was relaxed.

The full post-fix trajectory artifact reports no static-clearance regression (minimum static clearance 0.125 m in the 112.9 s run). Dynamic collision behavior remains scenario-dependent and is not claimed solved by this lifecycle change.

## Final classification

```text
SIDE_ACTIVE_STATE_LOSS_ROOT_CAUSE: GLOBAL_TRAJ_UPDATE_RESET_LOCAL_TRAJ_VALIDITY_FIELDS
ACTIVE_ID_RESET_BY_GLOBAL_TRAJ_RESET: YES (fixed)
SAFE_SIDE_RETAINED_AFTER_RECOVERY_FAILURE: YES (observed)
NO_PREVIOUS_ACTIVE_FOR_ACTIVE_SIDE_AFTER_FIX: 0
FIRST_TRACKING_DISCREPANCY: CONTROLLER_TRACKING_FAILURE (command vs odom)
LOG_TIME_ALIGNMENT_ERROR: NO EVIDENCE
TRAJ_SERVER_SAMPLING_ERROR: NO EVIDENCE
BODY_YAW_CONTROL_COUPLING: NOT ESTABLISHED
SCP_CORE_CHANGED: NO
TRUST_CHANGED: NO
OSQP_CHANGED: NO
JERK_LIMIT_CHANGED: NO
BUILD: PASS
RUNTIME: PASS
CLEAN_SHUTDOWN: PASS (SIGINT)
ROS_PROCESS_RESIDUE: NONE
```
