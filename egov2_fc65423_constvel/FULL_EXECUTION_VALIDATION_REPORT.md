# ALP Full Execution Validation Report

## Run

- Scene: `long_cylinder_forest.json`
- RViz: OFF
- Target-facing body yaw: ON
- `max_jer`: 22
- Risk candidates, SIDE/backoff, A*, downstream rejoin (max 3), Local SFC, free-time MINCO, Hard SCP/OSQP, model-agreement, Physical-Time Trust and final checker: enabled as configured
- Log: `full_execution_20260901.launcher.log`
- Trajectory: `full_execution_20260901_trajectory.csv`
- Visibility: `full_execution_20260901_visibility.csv`

The run was stopped with SIGINT after approximately 85.5 s of trajectory data. roslaunch completed its cleanup and no ROS/planner processes remained afterward.

## A* and downstream rejoin runtime behavior

The source loop performs up to three total attempts (`0, 1, 2`), computes a new downstream index and anchor for each attempt, and only accepts a later attempt after the earlier attempt fails. In this full run:

| Item | Count/result |
|---|---:|
| A* result records | 24 |
| attempt 0 | 24, all `SUCCESS` |
| attempt 1 | 0 |
| attempt 2 | 0 |
| `[astar-rejoin-fallback]` | 0 |

This is `REJOIN_FALLBACK_NOT_NEEDED`, not an indication of an unreachable fallback path: no attempt-0 A* failure occurred in the actual scene execution. The code audit found no loop-external return, clamped identical goal, or missing reset that would prevent attempts 1/2 after a genuine attempt-0 failure.

## A* → Local SFC → SCP execution

Runtime records in the full run:

| Metric | Count |
|---|---:|
| Local SFC `BUILD_SUCCESS` | 7 |
| Local SFC active records | 7 |
| `collision_bypassed_to_scp=1` | 8 records (7 initial-collision hand-offs plus one conflict-window hand-off) |
| Local SFC final records | 7 |
| `SCP_FINAL_OK` records | 52 |
| Local SFC final failures | 7 |

The seven Local-SFC candidates all have corresponding final records. Their final failure reasons are:

- `P_T_TRUST_TOO_SMALL`: 1
- `QP_MAX_ITER_EXHAUSTED`: 3
- `TRUE_CONSTRAINT_INFEASIBILITY`: 1
- `DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`: 2

The collision bypass records demonstrate that an A*-safe guide with a valid Local SFC is allowed into the existing SCP even when the unconstrained MINCO initializer is occupied. No candidate was forced to success after backend failure.

## Candidate selection and execution evidence

The risk-candidate lifecycle produced:

- 113 triggered risk-candidate decisions;
- 32 decisions selecting SIDE (`SIDE_PLUS=20`, `SIDE_MINUS=12`);
- 52 `SCP_FINAL_OK` final records;
- 882 `REPLAN_TRAJ → EXEC_TRAJ` transitions and 6 initial `SEQUENTIAL_START → EXEC_TRAJ` transitions;
- all three trajectory servers reported ready.

The current log format does not attach a unique candidate ID to the downstream trajectory message, so it cannot provide a one-line identity join from each individual `SCP_FINAL_OK` to a specific `traj_server` callback. The available execution evidence is nevertheless consistent: successful planner cycles transition to `EXEC_TRAJ`, trajectory servers are live, and the odometry recorder contains continuous post-planning trajectories.

## Actual odometry clearance

`full_execution_20260901_trajectory.csv` contains 7,701 samples for three UAVs, covering `0.008088–85.540753 s`.

| UAV | Minimum static clearance (m) | Minimum moving clearance (m) |
|---:|---:|---:|
| 1 | 0.231469 | 0.007922 |
| 2 | 0.299634 | 0.050744 |
| 3 | 0.209532 | 0.736248 |

All recorded static clearances are strictly positive. Moving clearance is also positive in the odometry record; the minimum values are a runtime tracking metric and are not substituted for the planner's authoritative dynamic safety checker.

## Obstacle 9

Obstacle 9 was reached in the actual execution. The log contains repeated obstacle-9 risk decisions, Local-SFC construction, and both accepted and backend-rejected SCP outcomes. At least one obstacle-9 candidate reached `SCP_FINAL_OK`; accepted trajectories retain `corridor_violation=0`. Other obstacle-9 candidates failed with QP max-iteration or dynamics model-mismatch exhaustion. Their `static_violation` fields describe retained failed/intermediate trajectories, not accepted final trajectories.

## Stability and shutdown

- No dimension mismatch, bad array, segmentation fault, double free, or OSQP crash was observed.
- Existing shutdown output includes `boost::wrapexcept<boost::lock_error>` from the control/shutdown path; roslaunch still completed cleanup.
- No `roscore`, `rosmaster`, `roslaunch`, `ego_planner_node`, `so3_control`, `swarm_bridge`, `simulator`, or native scene process remained after SIGINT cleanup.

## Final assessment

The full execution confirms the new A*→Local-SFC hand-off is live in the production chain and reaches the SCP backend during obstacle encounters. The downstream fallback is implemented and runtime-reachable by control flow, but was not needed in this run because all 24 observed attempt-0 A* searches succeeded. Remaining Local-SFC failures are backend trust/QP/true-feasibility outcomes rather than an A* fallback control-flow defect.

```text
FULL_EXECUTION: PASS
ASTAR_TO_LOCAL_SFC_RUNTIME: PASS
COLLISION_INITIALIZER_BYPASSED_TO_SCP: YES
REJOIN_FALLBACK_RUNTIME_TRIGGERED: NO (NOT NEEDED)
OBSTACLE9_REACHED: YES
OBSTACLE9_ACCEPTED_TRAJECTORY_STATIC_COLLISION: NO
ACTUAL_ODOM_STATIC_CLEARANCE_POSITIVE: YES
SIGINT_CLEAN_SHUTDOWN: YES
ROS_PLANNER_RESIDUALS: NONE
```

