# Feedback 064 - RViz authority marker and short-trajectory audit

Date: 2026-09-14

BASELINE: feedback_63

## Change

`planning_visualization.cpp` now renders the validated execution-authority
prefix in the historical red color with marker scale `0.07` (previously green
with scale `0.15`).  The unvalidated future suffix remains on its separate
topic and orange/transparent style, so the authority distinction is preserved.
The legacy `optimal_list` topic remains unchanged.

`traj_utils` was rebuilt successfully and the ALP encirclement launch was
restarted to load the new library.  No planner, lifecycle, horizon, safety
threshold, or target-tracking logic was changed.

## Short trajectory audit

The old `encirclement_rviz.log` contains selected trajectories below one
second, but the first causal chain is lifecycle/successor supply rather than
the RViz marker code:

- `SHORT_STATIONARY_HYPOTHESIS` rejects short, low-speed candidates;
- repeated rejection is followed by `NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE`
  / `MOVING_SUCCESSOR_STARVATION`;
- the trajectory server then reports
  `TRAJECTORY_EXPIRED_NO_REPLACEMENT` and
  `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`;
- the selected trajectories immediately before/after these events are often
  about `0.6-1.0 s`, while fresh initializers are commonly longer.  Thus a
  short visible authority prefix can also reflect the validated-prefix
  contract or an expired successor, not a uniform planner duration clamp.

The restarted log (`encirclement_rviz_after_marker_fix.log`) shows selected
durations of approximately `1.4-2.4 s` in the observed interval and no
`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` or `MOVING_SUCCESSOR_STARVATION` in
that interval.  This is evidence that the current observation was transient
and workload-dependent; it is not sufficient justification to alter the
existing lifecycle contract or its `SHORT_STATIONARY_HYPOTHESIS` policy.

Therefore no production lifecycle fix was applied in this round.  A separate
task should instrument and address successor starvation if it persists under a
complete run; changing its threshold or adding a stop/retiming fallback here
would be an unrelated behavior change.

## Verification

`catkin build traj_utils -j2 --no-status --workspace ros_ws`: PASS.

The ALP encirclement simulation and native RViz are currently running with the
same scene and cooperative/encirclement parameters.  The unvalidated suffix
topic remains enabled and visually distinct.

PRODUCTION_SOURCE_CHANGED: YES (visualization marker color/scale only)
PLANNER_SOURCE_CHANGED: NO
LIFECYCLE_SOURCE_CHANGED: NO
LAUNCH_PARAM_CHANGED: NO
SIMULATION_RUN: YES (existing requested ALP run, restarted after rebuild)
SIMULATION_LEFT_RUNNING: YES (current ALP run intentionally left available for observation)
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
