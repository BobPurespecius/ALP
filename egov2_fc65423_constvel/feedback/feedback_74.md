# Feedback 74 — NOMINAL target isolation and cooperative viewpoint progress guard

本轮只修改 `EGOReplanFSM::callReboundReplan()` 的 target wiring，以及 cooperative viewpoint 已完成后的 zero-progress alternative guard。未修改 duplicate publication/lifecycle、Joint、dynamic SCP、clearance、FREE TIME、N/L/R、BODY/LOS、controller 或 traj_server。

## Root cause and fix

NOMINAL_TARGET_OVERWRITE_ROOT_CAUSE:
`callReboundReplan()` 原先把 `local_target_pt_ / local_target_vel_` 直接传给 `teamReferenceGoal()`；该函数会原地写入 cooperative viewpoint，导致 team reference 越权替代正常 rolling local target。

FIX_APPLIED:
`getLocalTarget()` 结果保留为 `normal_local_target` / `normal_local_target_vel`；cooperative target 改用独立的 `team_reference_target` / `team_reference_vel` / `team_relative_tracking`。hypothesis-0 明确强制使用 normal target；只有显式 alternative 才可使用 team/reference 偏移。

NOMINAL_USES_ONLY_NORMAL_LOCAL_TARGET: YES
TEAM_REFERENCE_HAS_SEPARATE_STORAGE: YES
TEAM_VIEWPOINT_CAN_OVERRIDE_NOMINAL: NO

TEAM_VIEWPOINT_ZERO_PROGRESS_GUARD: YES
TEAM_VIEWPOINT_COMPLETION_BLOCKS_LOCAL: NO

Guard 复用现有 `visibility_min_target_distance`（本次场景为 0.20 m）。当 authoritative start 到 cooperative reference 的距离不超过该 tolerance，alternative 被丢弃并记录 `TEAM_VIEWPOINT_ZERO_PROGRESS_REPLAN`；正常 NOMINAL rolling target 继续存在。

## Simulation evidence

NOMINAL_TARGET_OVERWRITTEN_COUNT: 0 (wiring invariant; 980 个 `[nominal-baseline-semantics]` 事件均保持 `source=AUTHORITATIVE_PVA_TO_LOCAL_TARGET`)
TEAM_VIEWPOINT_ZERO_PROGRESS_REPLAN_COUNT: 4

TERMINAL_HOLD_COUNT: 1
END_BEFORE_NEXT_COUNT: 1

该唯一 hold 事件发生在 trajectory 147 到期时，前后紧邻 `Received drone 0's trajectory out of order or duplicated, abandon it.`；本次仿真共记录约 2895 条该类告警。它属于用户明确要求本轮只统计、不修的 duplicate publication/lifecycle 问题，因此不能把该 hold 归因于 NOMINAL/team target wiring。

USER_VISIBLE_STOP: IMPROVED, but not eliminated because duplicate/out-of-order publication remains
UAV_LARGE_OSCILLATION: UNKNOWN
SHARP_DIRECTION_SWITCH: UNKNOWN
MEDIUM_DENSITY_STUCK: YES (仍有 persistence/recovery 事件)
TEAM_POSITION_SWAP_OBSERVED: UNKNOWN

DUPLICATED_TRAJECTORY_WARNING_COUNT: 2895 (仅记录，本轮未修改)

Hypothesis/alternative 运行仍存在（`HIGH_VISIBILITY_CANDIDATE_GENERATED=1` 记录 198 次），说明 cooperative/team reference 功能未被关闭；同时 zero-progress alternatives 被单独丢弃。

SCENARIO: long_cylinder_forest.json

JOINT_CHANGED: NO
DYNAMIC_SCP_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
FREE_TIME_CHANGED: NO
TRAJ_SERVER_CHANGED: NO
DUPLICATE_PUBLICATION_FIX_APPLIED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
