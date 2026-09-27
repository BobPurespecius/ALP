# Feedback 069 - Repair-only Dynamic Seed Follow-up on Many-Obstacle Encirclement

## Scope

This turn used the ALP scene `long_cylinder_forest.json`, which contains 10
dynamic obstacles.  It was launched with the two-phase RViz startup and
`enable_encirclement_tracking=true`.  The run was stopped immediately after a
real terminal hold, as required.  No RRCT path was accessed.

## Production change

`EGOPlannerManager::reboundReplan` now distinguishes a statically free SIDE
seed with insufficient dynamic clearance from a statically infeasible seed.
The former is marked `side_repair_only` and may enter the existing MINCO/SCP
path with the explicit log action
`ENTER_OPTIMIZER_NO_EXECUTION_AUTHORITY`.  This is not an execution admission:
the existing candidate classification and current-revision hard preflight must
still prove dynamic, static, dynamics, and swarm safety before selection and
commit.  No threshold, brake, fallback, guide, Joint, or FREE_TIME behavior was
changed.

## Run evidence

The run directory is `encirclement_many_dynamic_20260915/`.

Startup confirmed `prediction/obj_num=10`, `moving_obj_clearance=1.1`, and
time-aware moving-object cost enabled.  The repair-only branch was reached 808
times (807 complete `REPAIR_ONLY_TO_OPTIMIZER` records).  In the acute drone-0
event, statically free SIDE seeds entered the optimizer with seed dynamic
clearance around `0.32 m`, while the hard required clearance remained `1.10 m`.
The optimizer returned trajectories with final dynamic clearance around
`0.35-0.40 m`; the authoritative safety classifier correctly marked them
`INVALID`, so they were never executable.

The run was stopped at the first observed failure window.  Retained counts:

```text
TERMINAL_HOLD_ENTER: 3
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION: 3
MOVING_SUCCESSOR_STARVATION: 0
SHORT_STATIONARY_HYPOTHESIS: 0
side-repair-only: 808
REPAIR_ONLY_TO_OPTIMIZER: 807
UNVALIDATED_EXECUTION: 0 (no such event observed)
EXECUTED_COLLISION: 0 (no such event observed)
SWARM_VIOLATION: 0 (no such event observed)
```

Representative hold:

```text
drone=0
trajectory_id=62
TERMINAL_HOLD_ENTER ~= 1789438053.69965
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION ~= 1789438053.69979
predecessor end ~= 1789438053.69541
```

## Causal result

The repair-only admission removed the first frontend starvation gate, but it did
not create a dynamic-obstacle repair mechanism.  The ordinary MINCO/SCP path
has moving-object terms available, yet the acute candidate diagnostics show
`moving_cost=0` and `moving_grad=0`; the optimized path therefore stays in the
same dynamic conflict basin.  Its final hard risk remains below `1.10 m`, and
the existing hard classifier rejects it.  Both sides can consequently remain
non-executable until the predecessor authority expires, producing the observed
terminal hold.

This run therefore disproves the idea that allowing an unsafe seed into the
ordinary optimizer alone is sufficient.  It does not justify lowering the
clearance threshold or executing the seed.  The next production fix must wire
a real dynamic body repair objective/constraint into the repair path, or run a
bounded P/T trial loop with the existing hard dynamic evaluator after every
trial.  Final hard preflight remains mandatory.

## Status

```text
PRODUCTION_SOURCE_CHANGED: YES
REPAIR_ONLY_STATE_ADDED: YES
FREE_TIME_FROZEN_REINTRODUCED: NO
SAFETY_THRESHOLD_CHANGED: NO
UNSAFE_SEED_EXECUTED: NO
PLANNER_BRAKE_ADDED: NO
STOP_FALLBACK_ADDED: NO
BODY_LOS_TOPOLOGY_CHANGED: NO
JOINT_LIFECYCLE_CHANGED: NO
SIMULATION_STOPPED_AFTER_HOLD: YES
FULL_RUN_COMPLETE: NO (stopped on first acute hold)
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

