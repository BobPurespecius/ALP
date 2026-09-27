# Feedback 82

## Quick card-stop diagnosis and fix

The apparent all-UAV freeze in `sim_run_95` was amplified by stale nodes from
the previous RViz/scene run sharing ROS master `11361`. After cleaning the
workspace's simulation nodes, the remaining planner-side failure was a simple
zero-progress rolling target: `getLocalTarget()` occasionally returned a point
within a few centimetres of the measured UAV. This produced 0.49 s (or shorter)
trajectories; when side candidates failed, persistence expired before a normal
successor could be installed.

## Fix

In `EGOReplanFSM::callReboundReplan()`, tracking-only local targets closer than
0.5 m are extended using the already authoritative target velocity for one
normal rolling horizon (minimum 1 s). This is target prediction, not a hard
safety constraint, stop fallback, planner brake, or mission-terminal change.

Build:

```text
catkin build ego_planner -j2 --no-status --workspace ros_ws
```

Successful.

Clean validation run: `sim_run_97.log`, fixed `long_cylinder_forest.json`, FULL
ON, 10 dynamic obstacles, cooperative/visibility/Joint/topology enabled,
native RViz. The run produced 62 commits and 2 hold/end-before-next events in
the bounded 32 s window; all workspace simulation nodes were cleaned afterward.

The previous run's 35 holds in 45 s was therefore partly stale-process
contention; the clean run is substantially lower. No LOS/BODY architecture,
Joint, safety threshold, or dynamic clearance was changed.

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
