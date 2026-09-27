# ALP Dynamic Topology Obstacle-Pressure Map

## Deliverable

`ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates.json`

The production `long_cylinder_forest.json` was not overwritten.  The new file
keeps the target route, UAV initial states, obstacle physical dimensions, map
scale, planner parameters, and the ten-object moving-obstacle interface.  It
rearranges only a few nearby forest cylinders and adds four small static gate
blockers so that the two C gates have an explicit asymmetric bypass.

The generator was also checked for unique static model names; the two blockers
at each C gate now have distinct names (`blocker_0`/`blocker_1`), and the
generated world parser accepts all 40 static and 10 moving models.

## Route-local construction

For each gate, `t` is the route tangent and `n` is the left horizontal normal.
Dynamic motion is the existing runtime model:

```text
p(t) = centerENU + axisENU * A * sin(2*pi*t/T + phase)
v_peak = A * 2*pi/T
```

For A gates, the phase is analytically chosen so the obstacle crosses the
nominal centerline at the expected encounter time.  For B/C gates, the baseline
is shifted by `+A*n` and the phase places the obstacle at its zero-speed
turning point on the nominal centerline at the encounter time.

| Gate | Type | Encounter time (s) | A (m) | T (s) | v_peak (m/s) | Warning @ 1.1 m | Dwell/zone time | Intended topology |
|---|---|---:|---:|---:|---:|---:|---:|---|
| A1 | CROSSING | 16.0 | 1.10 | 12.566 | 0.550 | 2.00 s | crossing | either side |
| B1 | DWELL | 31.0 | 1.00 | 12.000 | 0.524 | 2.10 s | 3.54 s (`|y|<=0.4`) | either side |
| C1 | LEFT_OPEN | 46.0 | 0.90 | 11.500 | 0.492 | 2.24 s | 3.59 s (`|y|<=0.4`) | LEFT |
| C2 | RIGHT_OPEN | 58.0 | 0.90 | 11.500 | 0.492 | 2.24 s | 3.59 s (`|y|<=0.4`) | RIGHT |
| A2 | CROSSING | 72.0 | 1.10 | 12.566 | 0.550 | 2.00 s | crossing | either side |

The warning estimate uses the planner's 1.1 m dynamic center-distance contract,
`T_warning ~= 1.1/v_peak`, rather than the smaller physical radius sum.  Gate
separations are 15, 15, 12, and 14 seconds, so encounters are intentionally
staggered.

## Geometry feasibility sanity

The scene metadata records `r_uav=0.30 m`, `r_dynamic=0.28 m`, and static C-gate
blocker radius `0.55 m`.

- Dynamic inflated radius: `rho_dyn = 0.30 + 0.28 = 0.58 m`.
- Static blocker inflated radius: `rho_sta = 0.30 + 0.55 = 0.85 m`.
- C-gate blocked-side blockers are at normal offset 1.05 m.  They do not
  physically overlap the moving cylinder (`1.05 - (0.55+0.28) = 0.22 m`),
  while their UAV-inflated regions overlap the dynamic inflated region
  (`1.05 - (0.85+0.58) = -0.38 m`), making that side unattractive/blocked
  without a collision-producing overlap.
- At a 1.50 m open-side bypass centerline, dynamic surface clearance is
  `1.50 - 0.58 = 0.92 m`.  The minimum static surface margin at that same
  centerline is 1.95 m (C1/C2), so the open side is not a grazing passage.
- A 0.10 m grid BFS using all static cylinders plus the gate dynamic footprint
  found a local start-to-goal bypass for A1, B1, C1, C2, and A2.  The intended
  C1/C2 open-side sample points are free.
- With `D=1.5 m` lateral displacement and the available warning/occupancy
  times, the rough lateral estimates remain within the existing limits:
  `a ~= 4D/tau^2` is about 1.5--2.0 m/s^2 for the A gates and below 1 m/s^2
  for the dwell gates; the corresponding rough jerk scale is below 10 m/s^3,
  under `max_jer=22`.

Therefore no gate is intentionally centimeter-wide, instant-stop, or
geometrically impossible.

## Runtime sanity run

Run artifact (RViz enabled, target-facing yaw enabled, `max_jer=22`, current ALP
risk candidates/Local-SFC/Hard-Corridor settings):

- `dynamic_gates_sanity_20260901.launcher.log`
- `dynamic_gates_sanity_20260901_visibility.csv`
- `dynamic_gates_sanity_20260901_visibility_trajectory.csv`

The run reached the target stop state and traversed the whole route.  The
first risk-trigger observations were approximately 15.9 s (A1, obstacle 0),
30.6 s (B1, obstacle 1), 43.8 s (C1, obstacle 2), 57.5 s (C2, obstacle 3),
and 68.8 s (A2, obstacle 4).  The run log shows SIDE selections for the
encounters and no parser, dimension, or planner process crash during flight.
The forced test teardown emitted several Boost mutex warnings from child-node
shutdown; these occurred after the target had stopped and are teardown-only,
not a gate-geometry failure.  All ROS/RViz processes were subsequently
cleaned up.

## Required final classification

```text
SCENE_FILE: ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates.json
A1_WARNING_TIME: 2.00 s
B1_DWELL_TIME: 3.54 s
C1_OPEN_SIDE: LEFT
C2_OPEN_SIDE: RIGHT
A2_WARNING_TIME: 2.00 s
MAX_DYNAMIC_OBSTACLE_SPEED: 0.55 m/s
MIN_OPEN_SIDE_SURFACE_CLEARANCE: 0.92 m (at 1.50 m bypass centerline)
MIN_GATE_SEPARATION_TIME: 12.0 s
ALL_GATES_HAVE_FEASIBLE_BYPASS: YES
ANY_GATE_REACTION_TIME_LIMITED: NO
ANY_GATE_GEOMETRICALLY_IMPOSSIBLE: NO
RVIZ_SANITY: PASS (teardown warnings only)
```

No planner, A*, SIDE, Local-SFC, MINCO, SCP, OSQP, trust-region, jerk,
clearance, yaw, or target-route parameter was changed for this scene design.
