# Feedback 80

## Root cause

`EGOPlannerManager::lifecycleSuccessorDue()` always called
`validatePreviousRemainingTrajectory(now, true, ...)`. This upgraded every
installed rolling trajectory to mission-terminal validation during lifecycle
revalidation. A trajectory could therefore be safe over its rolling authority
prefix while a static or dynamic conflict several seconds later invalidated the
whole suffix and removed successor coverage.

## Fix

- Added `active_execution_touch_goal_` to carry the terminal/rolling semantic of
  the installed trajectory.
- Set it when a local trajectory is committed and when a team trajectory is
  adopted.
- Lifecycle revalidation now passes that stored semantic to
  `validatePreviousRemainingTrajectory()`.
- Rolling trajectories continue to use the bounded execution-authority horizon;
  only true mission-terminal trajectories use full-duration validation.

No collision threshold, N/L/R geometry, Joint, controller, or hold fallback was
changed.

## Build and simulation

Build:

```text
catkin build ego_planner traj_opt -j2 --no-status --workspace ros_ws
```

Result: successful.

Scenario: `long_cylinder_forest.json`, FULL ON, 10 dynamic obstacles,
encirclement tracking, cooperative viewpoint, visibility/team selection,
Joint/topology enabled.

Run artifact: `sim_run_84/sim.log`.

Observed in the sampled ~65 s simulation:

```text
TERMINAL_HOLD_ENTER: 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION: 0
ACTIVE_PVA_MISMATCH: 0
execution-safety-reject: 0
EXECUTED_COLLISION: 0
UNVALIDATED_EXECUTION: 0
SWARM_VIOLATION: 0
ACTIVE_TRAJECTORY_INVALIDATED: 10
```

The 10 active invalidations were genuine static or dynamic physical hard-safety
events. Candidate hard-collision rejection remained enabled; no physical hard
clearance was lowered. The run was stopped by the external bounded runtime
timeout after the verification window, with no ROS processes left running.

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
