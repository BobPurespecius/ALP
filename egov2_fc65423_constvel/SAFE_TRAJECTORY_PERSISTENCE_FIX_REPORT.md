# ALP Safe-Trajectory Persistence Fix Report

## Root cause before fix

Each replanning cycle initialized the selected result as NOMINAL. When the
current cycle's SIDE/A*/SCP recovery failed, that default remained selected,
even if the previously published SIDE/repair trajectory was still safe. The
actual executing trajectory was therefore discarded and replaced by a known-
risk nominal trajectory.

## Source change

The minimal change is in `planner_manager.cpp`, with state declarations in
`planner_manager.h` and safe initialization/time bookkeeping in
`plan_container.hpp`.

- Track provenance for the trajectory that was actually committed/published.
- Slice the published quintic trajectory at the current execution phase using
  coefficient translation; the old trajectory is never replayed from time 0.
- Revalidate the remaining segment against current dynamics, static occupancy,
  current dynamic prediction, and swarm safety.
- If a new candidate is accepted, commit it and report `NEW_SAFE_SELECTED`.
- If recovery fails while the remaining published trajectory is valid, return
  without calling `setLocalTrajFromOpt()` and report `KEEP_PREVIOUS_SAFE`.
- Commit success is checked before updating active metadata; failed commits can
  retain a valid previous trajectory.

No A*, SFC, SCP/MINCO mathematics, OSQP setting, safety threshold, or planner
parameter was changed.

## Fallback semantics

| Action | Meaning |
|---|---|
| `NEW_SAFE_SELECTED` | A new SIDE/repair candidate passed the existing checks and was committed. |
| `KEEP_PREVIOUS_SAFE` | New recovery was not usable, but the currently executing remaining segment passed current-cycle revalidation. |
| `PREVIOUS_INVALID` | Revalidation failed; the reason is logged (`DYNAMIC`, `SWARM`, `DYNAMICS`, `STATIC`, `EXPIRED`, or `NO_PREVIOUS_ACTIVE`). |
| `NORMAL_NOMINAL` | No active risk-triggered recovery is required; nominal behavior remains unchanged. |

## Safety revalidation and time behavior

The remaining trajectory is constructed as `p_remaining(u)=p(elapsed+u)`
with shifted quintic coefficients. The current wall-clock execution phase is
used, and the original `start_time` is left untouched when retaining a
trajectory. Dynamic checks query the current prediction epoch; the persistence
check uses the existing moving-object safety clearance contract. Velocity,
acceleration, and jerk checks use the existing limits/checker. A seven-case
offline test (`trajectory_persistence_unit_test.py`) passed, including
coefficient equivalence, current-phase extraction, and expiry rejection.

## Build

Command:

```text
catkin build ego_planner traj_opt path_searching -j2 \
  --cmake-args \
  -Dosqp_DIR=/home/bob/ALP/egov2_fc65423_constvel/local_osqp_debs/extracted/opt/ros/noetic/lib/cmake/osqp
```

Result: all 5 packages succeeded, no warnings, no failures.

## Runtime validation

All runs used `long_cylinder_forest.json`, RViz OFF, risk candidates and Hard
Corridor SCP enabled, target-facing yaw enabled, and unchanged production
parameters. Every run was terminated with SIGINT and ended with roslaunch's
`done` cleanup marker.

### Final smoke runs

The short smoke logs contained Local-SFC final records for both successful and
failed candidates, with no dimension mismatch, double free, segmentation fault,
or OSQP crash. The two short smokes had 3 and 2 `side-local-sfc-final` records;
all were explicit (`SCP_FINAL_OK` or a classified failure).

### Three complete post-fix runs

The trajectory recordings covered 117.40 s, 116.72 s, and 114.26 s,
respectively—beyond the approximately 95 s completion time of the reference
scenario.

| Run | KEEP_PREVIOUS_SAFE | NEW_SAFE_SELECTED | PREVIOUS_INVALID | Obstacle 6 / UAV1 | Obstacle 9 / UAV1 |
|---|---:|---:|---:|---|---|
| `full_run1` | 2 | 42 | 104 | unsafe-only, min +0.0716 m | collision, min -0.2393 m |
| `full_run2` | 1 | 38 | 119 | collision, min -0.2452 m | collision, min -0.0460 m |
| `full_run4` | 0 | 49 | 151 | unsafe-only, min +0.2895 m | collision, min -0.1644 m |

Aggregate: 3 observed `KEEP_PREVIOUS_SAFE` decisions, 129 new safe selections,
and 374 previous-invalid decisions. A concrete obstacle-9 recovery-failure
case in run 1 shows `risk-candidate ... obs=9 ... selected=NOMINAL` immediately
followed by `KEEP_PREVIOUS_SAFE`; the previous SIDE_MINUS trajectory had
`static_valid=1`, `dynamic_valid=1`, `dynamics_valid=1`, and 1.3798 m minimum
dynamic clearance.

The absence of KEEP in a cycle is not treated as a bug: the logs show explicit
revalidation failures such as dynamic clearance below the existing 1.1 m
safety contract, swarm conflicts, or `NO_PREVIOUS_ACTIVE`.

## Obstacle 9 and obstacle 6 outcome

The pre-fix reference reproduced obstacle-9 collision in 4/4 runs. The three
post-fix complete runs still produced 3/3 obstacle-9 collisions for UAV1
(minimum clearances -0.2393, -0.0460, and -0.1644 m). Thus persistence control
flow is fixed and observed, but obstacle-9 safety is not solved by this change.
Obstacle 6 was collision-free in two of the three post-fix runs and collided in
one; this is timing-sensitive and not a regression claim.

The remaining obstacle-9 blocker is downstream recovery: candidate generation
and SCP still fail or provide insufficient improvement in the encounter window
(including QP/max-iteration and secondary MINCO/SCP dynamics mismatch cases),
or the previous trajectory is correctly rejected by current safety checks. The
data do not show that a valid previous trajectory was retained and then
rewound or overwritten by this fix.

## Regression and stability checks

Across the three complete runs:

- No segmentation fault, double free, dimension mismatch, or OSQP crash.
- No planner-process residue from these runs after SIGINT cleanup.
- Static clearance remained positive in recorded odometry (minimum about
  0.19--0.21 m across runs).
- Tracking distance mean was approximately 1.65--1.74 m per UAV; path lengths
  were approximately 78--81 m per UAV.
- Optimization latency P95 was approximately 0.98--1.08 ms (maximum individual
  samples were workload spikes, not a new control-flow failure).
- Visibility ratios remained in the existing range (UAV1 0.933--0.956,
  UAV2 0.962--0.978, UAV3 0.980--0.989).
- Finite-difference jerk remains subject to the already-known secondary
  MINCO/SCP mismatch; it was not modified in this task.

## Final assessment

`KEEP_PREVIOUS_SAFE` is now a real fallback based on the currently executing
trajectory, not on a stale candidate object. It is revalidated every risk
cycle, extracted from the current execution time, and does not reset the
trajectory start time. New safe candidates can still replace it. The main
known-risk NOMINAL overwrite bug is therefore fixed, while obstacle-9 collision
requires a separate recovery/dynamics investigation.

```text
PRIMARY_ROOT_CAUSE_FIXED: YES
PREVIOUS_ACTIVE_SAFE_TRAJECTORY_PERSISTED: YES
PREVIOUS_TRAJECTORY_REVALIDATED_EACH_CYCLE: YES
PREVIOUS_TRAJECTORY_REUSED_FROM_CURRENT_EXECUTION_TIME: YES
TIME_REWIND: NO
KNOWN_RISK_NOMINAL_AUTOMATIC_FALLBACK_REMOVED: YES
NORMAL_NOMINAL_BEHAVIOR_PRESERVED: YES
NEW_SAFE_CANDIDATE_CAN_REPLACE_PREVIOUS: YES
OBSTACLE9_COLLISION_BEFORE: 4 / 4
OBSTACLE9_COLLISION_AFTER: 3 / 3 (post-fix complete runs)
OBSTACLE9_IMPROVEMENT: NO (persistence fix alone)
OBSTACLE6_COLLISION_AFTER: 1 / 3
STATIC_COLLISION_REGRESSION: NO EVIDENCE
TRACKING_REGRESSION: NO EVIDENCE
JERK_REGRESSION: NO (secondary mismatch unchanged)
VISIBILITY_REGRESSION: NO EVIDENCE
PLANNING_LATENCY_REGRESSION: NO EVIDENCE
SECONDARY_SCP_JERK_MISMATCH_CHANGED: NO
NEXT_BLOCKER_IF_COLLISION_REMAINS: recovery candidate/SCP dynamics and QP feasibility around obstacle 9
BUILD: PASS
RUNTIME: PASS
```
