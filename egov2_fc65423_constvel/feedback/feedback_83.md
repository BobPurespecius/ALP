# Feedback 83 — sim_run_98 crash and target-obstacle audit

SIMULATION: `sim_run_98.log`
SCENARIO: `long_cylinder_forest.json`

## Planner stop / crash

`drone_0_ego_planner_node`, `drone_1_ego_planner_node`, and
`drone_2_ego_planner_node` all exit with code `-11`; the
`team_target_reachability` process also exits with code `-11`.  Therefore the
observed all-UAV stop is a process crash, not an ordinary successor hold.

The log also contains:

```
terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'
what(): boost: mutex lock failed in pthread_mutex_lock: Invalid argument
```

No core/backtrace is present in the ROS log directory, so the exact source
line of the SIGSEGV is not proven by this run.  The last planner activity is
still trajectory activation/replanning and `KEEP_PREVIOUS_SAFE`/candidate
fallback traffic.  This is a crash-path symptom, not evidence that the
recent target-distance guard alone caused it.

Observed in the run:

- `TERMINAL_HOLD` records: 6
- `END_BEFORE_NEXT` records: 3
- planner SIGSEGV exits: 3
- reachability SIGSEGV exits: 1

The hold counters are downstream effects of planner death and loss of normal
trajectory supply; they must not be interpreted as a clean lifecycle hold.

## Target route collision audit

The target coordinator uses `target_radius=0.25` and
`target_obstacle_clearance=0.05` (0.30 m required center-to-obstacle-radius
clearance) and its pre-pass only evaluates `movingObstacleData`.  There is no
static-cylinder check in `prepare_dynamic_safe_speeds()`.

Replaying the waypoint interpolation at the configured 0.928 m/s against all
static cylinders gives these minimum clearances (target radius and obstacle
radius subtracted):

| obstacle | minimum clearance | route time (s) |
|---|---:|---:|
| static 15 | -0.609 m | 74.33 |
| static 5 | -0.461 m | 27.04 |
| static 0 | -0.366 m | 7.62 |
| static 22 | -0.056 m | 69.76 |
| static 3 | +0.026 m | 17.20 |

Thus the red target route is geometrically colliding with at least four static
cylinders under the scene's own radii; this is independent of planner
selection and is the primary confirmed cause of the visible target collision.

For the coordinator's dynamic timing model (motion clock synchronized through
`/dynamic/motion_start_time`), replaying its speed schedule gives a minimum
dynamic target clearance of approximately `+0.211 m` (moving obstacle 6,
about `t=9.56 s`).  This does not prove the runtime model is perfectly aligned
with Gazebo, but it shows the confirmed route collision is static, while any
additional dynamic collision would require checking the actual obstacle clock,
radius, and target odometry against the published motion start.

## Authority gaps

`target_state_coordinator.py` currently protects target-vs-dynamic geometry
only during speed selection.  It does not protect target-vs-static geometry,
and the fallback when no dynamic speed scale is safe selects the least-bad
dynamic clearance rather than rejecting the route.  Consequently target
collision prevention is not a final physical authority gate.

## Recent-fix causality

`TARGET_PROGRESS_GUARD_CAUSED_COLLISION`: **UNPROVEN**.  The deterministic
static route collision exists in the scene waypoint chain before planner
execution and is not created by the guard.  The SIGSEGV is also not localized
to a source line because this run has no core/backtrace; do not attribute it to
the guard or raw-LOS changes without a reproducible stack trace.

CODE_MODIFIED_THIS_TURN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
