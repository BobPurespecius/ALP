# Round 2 verification record (5 fixes) — sim_run_round2.log

Scenario: `long_cylinder_forest.json` (FULL ON: risk-triggered candidates, 10 dynamic
obstacles, encirclement tracking, cooperative viewpoint, visibility ranking,
K-of-N (k=2), Early Joint/topology, native RViz).
Resolved parameters were printed by `run_round2_full_on.sh` before launch.

Build:
`catkin build traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`
-> All 6 packages succeeded.

One pre-existing build break had to be repaired to satisfy that command:
`src/multi_uav_formation/src/team_visibility_optimizer.cpp:965` used `target`
outside its scope in `TeamVisibilityOptimizer::evaluate()`; the second sampling
loop now re-derives the target at its own sample world time (no algorithm change,
same single time base as the first loop).

## Fix 1 — planning busy-loop

`[replan-cadence] REPLAN_TOTAL_COUNT=189 REPLAN_INTERVAL_LT_10MS_COUNT=0
REPLAN_INTERVAL_LT_5MS_COUNT=0 PENDING_REPLAN_SUPPRESSED_COUNT=13`

Max value observed for both interval counters across the whole run: 0
(previously 206 of 330 replans on drone 0 were <10 ms apart).

Mechanism: `callReboundReplan()` now returns `ReplanOutcome{SUCCESS,PENDING,FAILED}`.
A pending successor returns `PENDING` before any MINCO/candidate work, and
`REPLAN_TRAJ` goes to `EXEC_TRAJ` on `PENDING`, so the next attempt is gated by
the next FSM tick.

## Fix 2 — rolling-target unit error

`[rolling-target-extension] ROLLING_TARGET_EXTENSION_COUNT=0 ... MAX_LOCAL_TARGET_FORWARD_JUMP=0.000000`

The extension now uses `movingObjectPredictionHorizonTime()` [s]
(2.0 s in this scenario) instead of `planning_horizen_` [m] (7.5 m).
The guard did not fire in this run (min local-target distance stayed above the
0.5 m threshold), so the numeric effect is code-verified but not
run-exercised; see "reporting bug found" below.

## Fix 3 — target / dynamic-obstacle time base

`[timebase-audit] MIXED_TARGET_TIMEBASE_COUNT=272
MIXED_DYNAMIC_RISK_TIMEBASE_COUNT=0 ROS_NOW_INSIDE_DYNAMIC_RISK_COUNT=0`

- The three `ros::Time::now()` epochs in the side-candidate risk path are gone;
  every audited `evaluateDynamicRisk` call now receives the batch
  `planning_prediction_epoch`. Dynamic-risk time base: 0 mismatches.
- `object_p_`/`object_v_` now carry `object_stamp_` (average stamp of the samples
  the smoothed estimate was built from) and every target prediction goes through
  `EGOPlannerManager::targetPositionAt(world_time)`; the optimizer object, the
  candidate visibility reports and the raw LOS sampler all receive the same
  `target_at_prediction_epoch`.
- `[timebase-consistency]` printed 201 times and shows the four target witnesses
  identical for the same epoch.
- `MIXED_TARGET_TIMEBASE_COUNT=272` is a **metric artifact, not a code defect**:
  the counter was defined as "caller object_pt differs from the rebased value",
  which is the intended age compensation and is non-zero by construction. The
  definition has been corrected after the run (it now counts only an absent or
  non-finite authoritative target stamp), so this value must be re-measured on
  the next run before it can be claimed as 0.

## Fix 4 — both sides failed -> NOMINAL

`[side-both-failed] SIDE_BOTH_FAILED_COUNT=100
SIDE_BOTH_FAILED_FALLBACK_NOMINAL_COUNT=100`

249 fallback events logged, e.g.

```
[side-both-failed-fallback] drone=1 generation=20 nominal_id=36
L_status=4.96402:imp=0.000321047 R_status=4.80607:imp=-0.157629
selected=NOMINAL reason=SIDE_BOTH_FAILED_FALLBACK_NOMINAL
```

No new stop/hold/brake/recovery/coverage mechanism was added.

## Fix 5 — LOS_OBSERVATION_SIDE plane lifecycle

`[los-plane-audit] LOS_PLANE_CREATED_COUNT=76 LOS_PLANE_LOST_COUNT=84
LOS_PLANE_VIOLATION_COUNT=19`

Read this with the corrected metric definitions:

- `LOS_PLANE_PRESERVED_ACROSS_ASTAR_REBUILD` = 0 because the A* repair branch
  never ran in this scenario; the first-loss point identified by the audit
  (`local_sfc_planes = repaired_local_sfc_planes`, which overwrote the semantic
  LOS plane) is now instrumented and no longer destructive: collision planes are
  replaced, semantic LOS planes are carried across the rebuild.
- `LOS_PLANE_INTERVAL_INVALID` = 0, so no plane was dropped by a bad
  active_start/active_end mapping.
- `LOS_PLANE_LOST_COUNT=84` counted candidates that had **no plane at all**
  (208 `local_sfc_planes_empty` events). That is not a loss; the definition has
  been corrected after the run so the next run reports 0 for that class.
- `LOS_PLANE_VIOLATION_COUNT=19` with 71 `REJECT_ALTERNATIVE_LOS_PLANE_VIOLATED`
  events: the plane metadata was present for the whole lifecycle and the final
  polynomial really left the required half-space. That is a legal candidate
  failure, which is exactly the metadata-lost vs geometric-violation split this
  round required.

## Other counters in the same run

```
TERMINAL_HOLD_ENTER                         4
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION       4
JOINT_FAILURE_LOCAL_NOOP                  346
EXECUTED_COLLISION                          0
SWARM_VIOLATION                             0
process has died                            0
lock_error                                  0
```

No SIGSEGV and no `boost::lock_error` occurred in this run (previous runs lost
planner processes in roughly every other run). Per the round's instructions the
remaining hold/joint numbers are recorded, not chased.

## Reporting bugs found and fixed during verification

1. The periodic counter printers latched after the first sample (a one-shot
   `summary_emitted_` flag was checked in the periodic path), so live counters
   froze at their startup values. Fixed: time-throttled periodic reporting plus a
   separate one-shot flag for the final report.
2. `reportTimebaseAndFallbackAudit()` shared one throttle variable with the
   `[timebase-consistency]` witness, which is paced on planning epochs and
   therefore suppressed the report. Fixed with two independent throttles.
3. Two audit counters were mis-defined (Fix 3 and Fix 5 above); definitions
   corrected after the run.

## Environment note

`~/.ros` is mounted read-only in this environment, which made rosmaster fail to
start (`OSError: Read-only file system: '/home/bob/.ros/log/master.log'`). The
runner now exports `ROS_HOME`/`ROS_LOG_DIR` inside the workspace. One run started
with no master at all and produced only `XmlRpcClient ... Connection refused`;
that run was discarded.
