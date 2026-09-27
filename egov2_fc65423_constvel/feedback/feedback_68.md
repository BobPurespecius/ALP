# Feedback 068 - Frequent Braking Root Cause: Dynamic Successor Supply Block

## Scope

This turn is a read-only audit of the retained encirclement run. No production
planner, lifecycle, optimizer, safety threshold, launch, or controller behavior
was changed. The simulation was not restarted and no ROS process was left by
this turn.

Evidence sources:

- `rolling_guide_decouple_encirclement_20260915/full.log`
- `encirclement_run_20260915/full.log`
- current `planner_manager.cpp` and `poly_traj_optimizer.cpp`

## User-visible symptom

The frequent brake/stop is confirmed as a successor-supply failure. It is not a
mission-end terminal condition and it is not the finite recovery guide shrinking
the rolling horizon.

In the first acute event around `1789436069.93`, drone 0 still had trajectory
136 active with roughly `0.57 s` of validated coverage and a previously
validated dynamic clearance of about `1.26 m`. Replanning repeatedly generated a
fresh full-horizon initializer (`4.38 s` in representative cycles), but no
executable successor was admitted. The previous trajectory then expired and the
trajectory server entered `TERMINAL_HOLD`.

Retained run counts:

```text
TERMINAL_HOLD_ENTER: 2
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION: 2
MOVING_SUCCESSOR_STARVATION: 0 (the run stopped before this label was emitted)
SHORT_STATIONARY_HYPOTHESIS: 1
UNVALIDATED_EXECUTION: 0
EXECUTED_COLLISION: 0
```

The longer encirclement run retained in `encirclement_run_20260915` shows the
same failure family at scale (`TERMINAL_HOLD_ENTER=17`,
`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION=17`, `SHORT_STATIONARY_HYPOTHESIS=958`).

## Closed causal chain

For the representative drone-0 event:

1. The active predecessor remains safe while it has coverage. It is retained as
   `KEEP_PREVIOUS_SAFE` and has nonzero motion; there is no terminal-velocity
   zeroing at this point.
2. A new nominal fresh initializer is constructed with a normal multi-second
   duration. This proves the guide-decoupling fix is active and that the fresh
   initializer is not the source of the stop.
3. The nominal path is dynamically unsafe (`min_dynamic_clearance` around
   `0.47-0.60 m`, while the existing body threshold is `1.10 m`).
4. `SIDE_MINUS` is statically free, but is rejected immediately by
   `DYNAMIC_SAFETY_CLEARANCE` / `DYNAMIC_INVALID_AFTER_BACKOFF`.
5. `SIDE_PLUS` is rejected by the static inflated-map check (`STATIC_COLLISION`)
   before it can produce a usable candidate.
6. The candidate set therefore contains no executable successor. The planner
   keeps the old trajectory until its validated end, then the server has no
   successor authority and enters `TERMINAL_HOLD`.

The log repeats this same pair of rejections every few milliseconds while the
remaining predecessor coverage decreases. This is the visible braking event:
the command is allowed to reach the end of the old authority, then the server
has no replacement to activate.

## First failure layer

The first dynamic rejection is in the SIDE frontend of
`EGOPlannerManager::reboundReplan`, not in the final preflight:

```text
planner_manager.cpp:6722-6730  evaluateDynamicRisk(side_init_mjo.getTraj(), ...)
planner_manager.cpp:6900-6915  side_dynamic_valid / DYNAMIC_SAFETY_CLEARANCE
planner_manager.cpp:6914       break before MINCO/SCP
```

The effective condition for a BODY conflict is:

```text
trial_risk.min_distance >= getMovingObjClearance()
```

When that condition is false, the code breaks out of side generation. The
candidate never reaches the full `optimizeTrajectory()` call that follows the
frontend. Therefore:

```text
EARLY_DYNAMIC_GATE_EXISTS: YES
EARLY_GATE_BLOCKS_P_OPTIMIZATION: YES
EARLY_GATE_BLOCKS_T_OPTIMIZATION: YES
EARLY_GATE_CAN_REJECT_REPAIRABLE_SEEDS: YES (at least potentially)
```

This is a seed/admission gate in front of the optimizer, but it behaves as a
hard candidate rejection. It is not merely a diagnostic precheck.

## What the optimizer can and cannot repair

The optimizer does have free-time variables. `optimize_time=true`, virtual-T
updates are accepted, and the retained log contains nonzero time steps. The
moving-object gradient path (`movingObjGradCostP`) computes `gradp`, `gradt`, and
`grad_prev_t` when `use_time_aware_moving_obj_cost` is enabled.

However, that path is a soft cost only. The body dynamic safety contract is
enforced later by `EGOPlannerManager::evaluateDynamicRisk`; there is no dynamic
obstacle hard SCP row in the current optimizer. The standalone
`PolyTrajOptimizer::checkMovingObjSafety` exists but is not called from the
candidate pipeline.

```text
LOCAL_DYNAMIC_OBSTACLE_TIME_REPAIR_ACTUALLY_EXISTS: PARTIAL
LOCAL_DYNAMIC_OBSTACLE_SPACE_REPAIR_ACTUALLY_EXISTS: PARTIAL
DYNAMIC_BODY_HARD_SCP_CONSTRAINT: NO
```

Consequently, removing the gate alone would not be a safe fix: an unsafe seed
could enter MINCO, receive only a soft moving-object cost, and still fail the
authoritative checker. Any future repair path must remain execution-ineligible
until the complete current-revision dynamic preflight passes.

The existing time-only feasibility routine is for swarm/inter-UAV temporal
separation. It does not solve local moving-obstacle body conflicts.

## Candidate diversity and physical interpretation

The failing set does not establish a globally impossible scene. It establishes
that the current local search did not produce an executable solution. In the
representative event, one side is statically blocked and the other side has a
dynamic clearance deficit before optimization. Since no dynamic-aware hard
repair stage is entered, the evidence cannot distinguish physical infeasibility
from planner search starvation.

```text
PHYSICAL_GLOBAL_INFEASIBILITY: NOT PROVEN
LOCAL_SEARCH_FAILURE: CONFIRMED
SEARCH_BLOCK_CLASSIFICATION: CASE_B / PARTIAL SEARCH BLOCK
```

The same dynamic blocker is repeatedly reported (`obstacle_id=0`) with the same
side outcome, so recovery is retrying the same local basin rather than expanding
the feasible P/T search space.

## Time alignment and terminal state

The dynamic query convention is internally consistent in the audited path:

```text
world_time = planning_prediction_epoch + local_candidate_time
```

`planning_prediction_epoch` is set to the scheduled activation epoch for the
candidate, and `evaluateDynamicRisk` uses that epoch for every obstacle query.
No stale-snapshot or double-elapsed-time bug was required to explain the stop.

The predecessor terminal velocity is not the primary cause. The retained
`KEEP_PREVIOUS_SAFE` predecessor remains moving until its finite authority ends;
the hold is a consequence of missing successor authority, not an intentional
planner brake.

## Root-cause ranking

1. **Primary:** dynamic SIDE precheck rejects a dynamically unsafe seed before
   any MINCO/SCP P/T repair can run.
2. **Secondary:** the existing optimizer has no local dynamic-obstacle hard SCP
   constraint; its moving-object path is soft-cost only, so the precheck cannot
   simply be deleted.
3. **Resulting lifecycle effect:** repeated identical retries consume the
   predecessor's validated coverage and produce `TERMINAL_HOLD`.

The previously fixed finite-guide duration coupling is not present in this
chain. Representative guide logs show `guide_remaining` near `0.16 s` while
`rolling_seed_duration` remains `1.50 s`.

## Recommended fixes (not implemented in this turn)

1. Add a strictly non-executable `UNSAFE_SEED_FOR_REPAIR_ONLY` path for a
   statically valid SIDE seed, then add authoritative dynamic body constraints
   with correct P/T Jacobians to the repair stage. Only a final hard dynamic
   preflight may grant execution authority.
2. Alternatively, implement a bounded local P/T repair routine that evaluates
   the existing dynamic model at every trial and accepts only a fully hard-safe
   result. Keep the existing absolute clearance unchanged.
3. Add a regression fixture proving that a dynamic-unsafe but statically valid
   seed can be repaired without executing the seed, and that a truly blocked
   local neighborhood is reported as search failure rather than silently held.

Neither recommendation permits unsafe execution, lowers the `1.10 m` contract,
executes an unvalidated suffix, or makes Local wait for Joint.

## Final status

```text
PRODUCTION_SOURCE_CHANGED: NO
PLANNER_BEHAVIOR_CHANGED: NO
LIFECYCLE_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
SIMULATION_RUN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
