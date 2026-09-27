# Feedback 067 - Guide Lifetime Decoupled From Rolling Horizon

## Scope

This run changed one production behavior: `EGOPlannerManager::buildRecoveryGuideSeed()` no longer uses the finite recovery-guide remainder as the total duration of the next rolling trajectory. The prior `FREE_TIME_FROZEN` rollback was preserved. No safety threshold, topology, Joint, lifecycle protocol, or execution authority rule was changed.

The first launch attempt was stopped because it used ordinary tracking flags (`enable_encirclement_tracking=False` and cooperative viewpoint disabled); its temporary directory was discarded during cleanup. The production verification run was then relaunched with the existing encirclement contract: cooperative viewpoint reference, encirclement tracking, visibility/team selection, Joint P/T/yaw, topology coordination, risk candidates, and native RViz all enabled.

## Implementation

Source: `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:5279-5363`.

The guide remainder is now used only to sample valid guide geometry. The rolling seed duration is constructed from the existing local geometry horizon and vehicle limits (`rolling_horizon`, 1.5 s in the production run). Samples after guide expiry hold the guide terminal relative offset while following the current target motion; they do not extrapolate the expired guide. Dynamics validation and the existing retiming loop remain mandatory before the seed is accepted.

## Verification

Build:

```text
catkin build traj_utils plan_env path_searching traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
All 6 packages succeeded
```

Evidence directory: `rolling_guide_decouple_encirclement_20260915/`.

The correct run logged all of the following:

```text
/cooperative_viewpoint_manager/enable: True
/cooperative_viewpoint_manager/enable_encirclement_tracking: True
/drone_[0-2]_ego_planner_node/optimization/enable_encirclement_tracking: True
```

Representative guide samples show the required separation:

```text
guide_remaining=0.361351 rolling_seed_duration=1.500000 fresh_duration=1.500000
guide_remaining=0.161351 rolling_seed_duration=1.500000 fresh_duration=1.500000
guide_remaining=0.161351 rolling_seed_duration=1.500000 fresh_duration=4.564553
```

The run produced 310 `GUIDE_SEED_USED` records and 2,593 nonzero free-time SCP records. A representative free-time step had `trial_delta_virtual_T_max_abs=0.15`, `actual_delta_real_T_max_abs=0.174269`, and `accepted=1`. Thus rolling time optimization remained enabled and was not re-frozen.

## Runtime result

The run was stopped immediately after the first observed acute hold, as requested. Raw counts in the retained log are:

```text
TERMINAL_HOLD_ENTER: 2
TERMINAL_HOLD_EXIT: 2
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION: 2
MOVING_SUCCESSOR_STARVATION: 0
NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE: 0
SHORT_STATIONARY_HYPOTHESIS: 1
UNVALIDATED_EXECUTION: 0
EXECUTED_COLLISION: 0
```

The first hold was not a guide-duration collapse. UAV0 predecessor 136 had a normal `duration=1.500000` persistence trajectory, but its validated end was reached while the new candidate set had no executable candidate. At the failure point:

```text
nominal dynamic clearance = 0.4864 m (< 1.1 m required threshold)
SIDE_MINUS = DYNAMIC_SAFETY_CLEARANCE failure
SIDE_PLUS = STATIC_COLLISION failure
candidate result = DYNAMIC_FAIL
source = PERSISTENCE_FALLBACK
```

The planner then entered post-deadline recovery and the server entered `TERMINAL_HOLD`. This is an independent dynamic feasibility/successor-supply failure; it is not evidence that the finite guide still controls rolling duration. No unsafe trajectory was executed.

## Acceptance fields

```text
GUIDE_REMAINING_DECOUPLED_FROM_ROLLING_HORIZON: YES
BUILD_RECOVERY_GUIDE_SEED_FIXED: YES
FREE_TIME_REMAINS_ENABLED: YES
ROLLING_TIME_OPTIMIZATION_ACTIVE: YES
GUIDE_REFRESH_WHEN_REMAINING_SHORT: NO (existing target expiry/refresh lifecycle remains unchanged)
RECURSIVE_GUIDE_DURATION_COLLAPSE: NO (not observed after the fix)
FIRST_COLLAPSE_STAGE_AFTER_FIX: NONE
SHORT_STATIONARY_COUNT: 1
STARVATION_COUNT: 0
END_BEFORE_NEXT_COUNT: 2
TERMINAL_HOLD_COUNT: 2
DYNAMIC_TIME_ADJUSTMENT_OBSERVED: YES
DYNAMIC_FAIL_TO_SAFE_SUCCESSOR_OBSERVED: NO (run stopped at first hold)
UNVALIDATED_EXECUTION_COUNT: 0
EXECUTED_COLLISION_COUNT: 0
SWARM_VIOLATION_COUNT: NOT OBSERVED IN RETAINED RUN
PRODUCTION_SOURCE_CHANGED: YES
FREE_TIME_FROZEN_REINTRODUCED: NO
SAFETY_THRESHOLD_CHANGED: NO
PLANNER_BRAKE_ADDED: NO
STOP_FALLBACK_ADDED: NO
BODY_LOS_TOPOLOGY_CHANGED: NO
JOINT_LIFECYCLE_CHANGED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

## Conclusion

The confirmed rolling-duration bug was the direct assignment of finite guide lifetime to fresh rolling seed duration. That assignment has been removed. A short guide now yields a normal 1.5 s seed (or a dynamics-retimed longer seed), while the guide is sampled only over its valid interval. The correct encirclement run still exposes a separate dynamic conflict with no safe local candidate; that issue remains unmodified and requires a separate successor-search audit.
