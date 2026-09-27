# Feedback 072 — Rolling dynamic authority / hard-clearance update

本轮固定运行 `long_cylinder_forest.json`（10 dynamic obstacles，FULL ON/native RViz）。未访问或修改 `/home/bob/RRCT`。

## Production change

`moving_obj_clearance=1.1 m` remains the preferred avoidance trigger/soft repair target. It is no longer used as the physical execution gate. Execution rejection now uses a separate live-geometry hard center-distance threshold:

`d_dynamic_hard = deployed iris horizontal footprint + 0.5*max(live obstacle marker scale.x, scale.y)`.

The deployed `iris.sdf` rotor outer extent is `sqrt(0.13²+0.22²)+0.128 = 0.3835 m`, represented by the production parameter `0.384 m`. Static `grid_map/obstacles_inflation=0.099 m` is no longer reused as the dynamic body radius. No scene obstacle radius was hard-coded; obstacle radius is read from the live predictor marker scale.

Dynamic manager validation and optimizer/SCP hard rows remain capped by the same rolling prediction/authority horizon (`2.0 s`); the 1.1 m soft objective remains unchanged and FREE TIME remains enabled.

## Fixed-scene evidence

The final run loaded:

```text
[moving_obj] optimizer params: preferred_clearance=1.10,
hard_body_radius=0.384, horizon=2.00
```

Observed in the retained run (`dynamic_hard_clearance_run_20260915c/constvel_rviz.log`):

```text
TERMINAL_HOLD_COUNT: 2
END_BEFORE_NEXT_COUNT: 2
TRAJECTORY_EXPIRED_NO_REPLACEMENT: 2
POST_DEADLINE_RECOVERY: 2
ACTIVATION events: 513
HARD_DYNAMIC_COLLISION_FAIL observations: 452
DYNAMIC_PREDICTION_INVALID observations: 61 (startup/stale prediction episode)
EXECUTED_COLLISION_COUNT: 0 observed
UNVALIDATED_EXECUTION_COUNT: 0 observed
traj-server handoff rejects: 0 observed
```

Representative first hold (drone 2, trajectory 90): persistence revalidation measured `clearance=0.61285` against `hard_clearance=-0.05115` (center distance below the physical body+obstacle threshold), so this was a genuine hard dynamic collision rejection, not a 1.1 m margin-only rejection. The predecessor validated end was `1789465051.881930113`; candidate generation continued after the deadline and terminal hold occurred at `1789465051.890646696` before a safe successor activation. A second hold occurred for drone 1 trajectory 168 for the same expiry/no-replacement pattern.

The run also contains many candidates with dynamic distances below the preferred 1.1 m objective while still producing normal successor activations. Thus the preferred margin is no longer a global hard gate. However, no retained line emitted a dedicated `DYNAMIC_MARGIN_VIOLATION` label, so the count of such executions is not separately measurable from this run.

## Status

```text
DYNAMIC_AVOIDANCE_CLEARANCE: 1.1 m
DYNAMIC_HARD_CLEARANCE: 0.384 m + live obstacle radius (about 0.28 m in this scene; approximately 0.664 m center distance)
MOVING_OBJ_CLEARANCE_NO_LONGER_GLOBAL_HARD_GATE: YES
ROLLING_DYNAMIC_REPAIR_HORIZON: 2.0 s optimizer prediction/authority horizon
ROLLING_DYNAMIC_HARD_VALIDATION_HORIZON: same 2.0 s rolling horizon
REPAIR_AND_VALIDATION_HORIZON_ALIGNED: YES
TOUCH_LOCAL_TARGET_SEPARATED_FROM_MISSION_TERMINAL: PARTIAL (rolling dynamic check is capped; no new terminal-mode path added)
FUTURE_UNAUTHORIZED_SUFFIX_CAN_KILL_CURRENT_PREFIX: NO for dynamic rolling check

TERMINAL_HOLD_COUNT: 2
END_BEFORE_NEXT_COUNT: 2
COVERAGE_HOLE_COUNT: 2
DYNAMIC_MARGIN_VIOLATION_EXECUTABLE_COUNT: observed qualitatively, exact count unavailable
DYNAMIC_HARD_COLLISION_REJECT_COUNT: 452 logged hard-fail observations
EXECUTED_COLLISION_COUNT: 0
UNVALIDATED_EXECUTION_COUNT: 0
SWARM_VIOLATION_COUNT: 0 explicit execution violations observed

USER_VISIBLE_STOP_PROBLEM: IMPROVED, NOT SOLVED
UAV_LARGE_OSCILLATION: UNKNOWN from this run
TEAM_POSITION_SWAP_OBSERVED: not assessed
MEDIUM_DENSITY_STUCK_OBSERVED: YES, at genuine hard-collision windows

SCENARIO: long_cylinder_forest.json
SAFETY_THRESHOLD_PHYSICAL_HARD_LOWERED_ARBITRARILY: NO
FREE_TIME_FROZEN_REINTRODUCED: NO
BODY_LOS_TOPOLOGY_CHANGED: NO
JOINT_CHANGED: NO
STOP_FALLBACK_ADDED: NO
PLANNER_BRAKE_ADDED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

The remaining two holds are not caused by the 1.1 m preferred-margin gate: both are preceded by physical hard-collision rejection (and one startup prediction-invalid episode). Further elimination requires a separate local-candidate/hard-collision feasibility or prediction-readiness fix; this round did not weaken physical collision authority.
