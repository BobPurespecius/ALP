# Feedback 81

## Fixes

- BODY-only conflict no longer enters the visibility/topology side selector.
  `LEXICOGRAPHIC_VISIBILITY_IMPROVEMENT` and preferred topology tie selection
  now require a current-authority LOS descriptor. BODY-only side selection is
  reported as BODY authority.
- Added independent raw LOS truth sampling in `EGOPlannerManager::reboundReplan()`.
  Static and dynamic ray checks use the same absolute world time for UAV,
  target, and dynamic obstacle prediction. Raw witnesses are merged into the
  existing `ConflictDescriptor` before selection, without changing Joint,
  Local-SFC, observation-plane, or safety-threshold architecture.

## First loss point

The previous LOS path depended on optimizer visibility support diagnostics.
In the failing run those diagnostics were zero, so the later candidate
visibility evaluator could observe an occlusion but no LOS witness was ever
created for the conflict descriptor. The first loss was therefore in
`EGOPlannerManager::reboundReplan()`, before descriptor construction. The raw
geometric witness is now created before that diagnostic gate.

## Build and simulation

Build:

```text
catkin build traj_opt ego_planner -j2 --no-status --workspace ros_ws
```

Result: successful.

Scenario: `long_cylinder_forest.json`, FULL ON, 10 dynamic obstacles,
encirclement tracking, cooperative viewpoint, visibility/team selection,
Early Joint/topology, native RViz.

Run artifact: `sim_run_94.log`.

Observed:

```text
RAW_STATIC_LOS_BLOCKED_COUNT: 207
RAW_DYNAMIC_LOS_BLOCKED_COUNT: 113
LOS_WITNESS_CREATED_COUNT: 210 audit events
LOS_DESCRIPTOR_COUNT: 210 audit events
LOS_DETECTION_MISS_COUNT: 0
RAW_LOS_BLOCKED_AND_DESCRIPTOR_MISSING_COUNT: 0
BODY_ONLY_VISIBILITY_TOPOLOGY_OVERRIDE_COUNT: 0
```

Descriptor masks observed:

```text
BODY: 2 events
LOS: 162 events
BODY+LOS: 186 events
```

Side-authority logs showed BODY, LOS, and BODY_AND_LOS selections; no
BODY-only selection was attributed to visibility/topology authority.

The run had 3 `TERMINAL_HOLD_ENTER` / 3
`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` events during the bounded run. No
`ACTIVE_PVA_MISMATCH`, `EXECUTED_COLLISION`, or `SWARM_VIOLATION` was logged.
Those lifecycle events are outside this round's two requested fixes and were
not changed.

BODY_ONLY_AUTHORITY_OVERREACH_FIXED: YES
LOS_DETECTION_FIX_APPLIED: YES
STATIC_LOS_RUNTIME_ACTIVE: YES
DYNAMIC_LOS_RUNTIME_ACTIVE: YES
BODY_PLUS_LOS_RUNTIME_ACTIVE: YES
WRONG_SIDE_OCCLUSION_STILL_OBSERVED: not established by this audit

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
