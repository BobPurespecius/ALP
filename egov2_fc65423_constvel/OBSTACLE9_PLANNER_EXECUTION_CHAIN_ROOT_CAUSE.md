# Obstacle 9 Planner → Execution Chain Root-Cause Report

Date: 2026-09-01  
Scenario: `long_cylinder_forest.json`  
Run: `obstacle9_chain_20260901.launcher.log` (RViz off, isolated ROS master `11361`)

## Executive result

Obstacle 9 / UAV 1 (UAV 1 is `drone_id=0` in the planner) still collided in the
new run. The first broken link is not a missing publish or a TrajServer receive
failure. A safe SIDE trajectory was generated, selected, committed, published,
and received, but a subsequent recovery cycle selected and published NOMINAL
after both SIDE candidates failed (`NO_STATIC_FEASIBLE_SIDE`). The safe SIDE
trajectory therefore remained active only briefly and was superseded before the
collision window. The command stream at closest approach was nominal and had a
small positive clearance; the odometry lagged it into collision.

## Event chain around obstacle 9

Absolute ROS times below are from the launcher log. `trajectory_id` is included
for message-level matching, but is not globally unique: `setGlobalTraj()` resets
the local counter, so `start_time` is the authoritative identity.

| Time (s) | Planner event | ID | Duration (s) | Predicted clearance |
|---:|---|---:|---:|---:|
| 1788256543.35584 | SIDE_MINUS `SCP_FINAL_OK`, selected | 1 | 6.211631 | 1.779322 |
| 1788256543.35840 | SIDE_MINUS published | 1 | 6.211631 | — |
| 1788256543.35853 | SIDE_MINUS received by `/drone_0_traj_server` | 1 | 6.211631 | — |
| 1788256543.62155 | both SIDE candidates failed; selected NOMINAL | 1 | 5.354621 | 1.066225 |
| 1788256543.62192 | NOMINAL published | 1 | 5.354621 | — |
| 1788256543.62326 | NOMINAL received | 1 | 5.354621 | — |
| 1788256546.39175 | SIDE_MINUS `SCP_FINAL_OK`, selected | 1 | 3.705767 | 0.435072 |
| 1788256546.39332 | SIDE_MINUS published | 1 | 3.705767 | — |
| 1788256546.39499 | SIDE_MINUS received | 1 | 3.705767 | — |
| 1788256546.40944 | SIDE failed (`DYNAMICS_TRUST_RETRY_EXHAUSTED`), selected NOMINAL | 2 | 5.444083 | 0.288202 |
| 1788256546.40996 | NOMINAL published | 2 | 5.444083 | — |
| 1788256546.41007 | NOMINAL received | 2 | 5.444083 | — |
| 1788256547.00644 | NOMINAL command sample at closest command clearance | 1 | — | — |
| 1788256547.44372 | odometry closest-approach sample | — | — | — |

The same pattern repeats in the intervening cycles: SIDE success is followed by
another risk-triggered cycle in which both SIDE candidates fail and NOMINAL is
committed. The log explicitly records `previous=SIDE_MINUS selected=NOMINAL` and
`NO_STATIC_FEASIBLE_SIDE`; the fallback validator reports
`PREVIOUS_INVALID reason=NO_PREVIOUS_ACTIVE` in the immediately following cycle.

## Four-layer clearance comparison

Values are sampled maxima/minima, not continuous-time guarantees.

- **Planner prediction:** selected NOMINAL clearance fell to approximately
  `0.282 m` in the collision approach (`0.288 m` on the final nominal commit).
- **Selected trajectory re-evaluation:** no evidence of a missing or stale
  message; the trajectory received by TrajServer has the same `start_time`,
  duration, piece count, and ID as the published message.
- **Command trajectory vs. obstacle:** minimum sampled clearance was about
  `+0.006 m` (near contact but not negative) using the recorder's motion epoch
  (`motion_start` is about 3.5 s before trajectory CSV time).
- **Actual odometry vs. obstacle:** minimum sampled clearance was `-0.1828 m`
  at trajectory time `73.883918 s`; static clearance at that sample was
  `2.0071 m`, so this is a dynamic collision, not a static-map collision.

At the odometry closest sample (`x=28.8333, y=0.2754`), interpolated command
position was approximately `(29.6727, 0.3028)`, giving a command/odometry
tracking error of `0.840 m`. Thus the command path was marginally clearing while
the executed state entered the moving cylinder.

## First failure layer

`OTHER_SOURCE_LEVEL_CAUSE` — **SAFE_TRAJECTORY_ACTIVATED_BUT_SUPERSEDED_BY_NOMINAL_AFTER_FOLLOW-UP_REPLAN**.

This is earlier than the command/odometry tracking error: the planner had a safe
SIDE result, but did not retain it when the next risk-triggered cycle had no new
SIDE solution. The subsequent NOMINAL command is the active trajectory at the
obstacle-9 closest approach. There is no evidence that a published safe message
was dropped by ROS or ignored by TrajServer.

The downstream execution symptom is additionally classified as
`CONTROLLER_TRACKING_FAILURE` (command clearance positive, odometry clearance
negative), but it is not the first broken link in the safe-candidate chain.

## Required boolean conclusions

```text
SAFE_CANDIDATE_GENERATED_BEFORE_COLLISION: YES
SAFE_CANDIDATE_SELECTED_BEFORE_COLLISION: YES
SAFE_TRAJECTORY_PUBLISHED: YES
SAFE_TRAJECTORY_RECEIVED_BY_TRAJ_SERVER: YES
SAFE_TRAJECTORY_ACTIVATED: YES (briefly)
ACTIVE_TRAJECTORY_AT_COLLISION: NOMINAL (drone_id=0 / UAV1)
PLANNER_PREDICTED_MIN_CLEARANCE: ~0.282 m (final nominal approach)
COMMAND_TRAJECTORY_ACTUAL_OBSTACLE_MIN_CLEARANCE: ~+0.006 m (sampled)
ACTUAL_ODOM_MIN_CLEARANCE: -0.1828 m (sampled)
PREDICTION_ERROR_AT_CLOSEST_APPROACH: ~0.28 m vs command (and ~0.47 m vs odometry)
COMMAND_TRACKING_ERROR_AT_CLOSEST_APPROACH: ~0.840 m
FIRST_FAILURE_LAYER: OTHER_SOURCE_LEVEL_CAUSE
SOURCE_LEVEL_ROOT_CAUSE: successful SIDE result was superseded by a later NOMINAL commit after SIDE recovery failure
CODE_BUG_FOUND: YES — planner lifecycle state is inconsistent during follow-up replanning (`previous=SIDE_MINUS` while `active_traj_id=0` and validator=`NO_PREVIOUS_ACTIVE`), allowing a NOMINAL overwrite; no transport/TrajServer bug was found
CODE_BUG_FIXED: NO (diagnostic-only scope; requires lifecycle serialization/state fix)
OBSTACLE9_COLLISION_AFTER_FIX: no planner fix was made; historical repeat set remains 3/3 (this chain run also collided)
```

## Stability and shutdown

The run completed with an explicit `SIGINT` to the isolated `roslaunch`, followed
by normal shutdown. No `roslaunch`, `ego_planner`, `traj_server`, simulator, or
port-11361 `rosmaster` remained afterward. Unrelated processes under
`/home/bob/RRCT/reopen` were not part of this run and were intentionally not
counted as residue.

No segmentation fault, double free, dimension mismatch, or OSQP crash occurred.

## Build

```text
catkin build traj_opt ego_planner path_searching -j2 \
  --cmake-args -Dosqp_DIR=/home/bob/ALP/egov2_fc65423_constvel/local_osqp_debs/extracted/opt/ros/noetic/lib/cmake/osqp
BUILD: PASS (all requested packages; OSQP linkage OK)
RUN_EXIT_MODE: SIGINT_CLEAN
```
