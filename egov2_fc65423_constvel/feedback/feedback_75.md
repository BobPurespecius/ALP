# Feedback 75 — Trajectory duplicate / out-of-order publication audit

本轮只处理 trajectory publication 去重。N/L/R、BODY/LOS、team reference、dynamic SCP、FREE TIME、Joint 架构、controller、traj_server hold 策略和安全阈值均未修改。

## Root cause

PRIMARY_DUPLICATE_ROOT_CAUSE:
存在两处 production 发布端重复广播：

1. `EGOReplanFSM::execFSMCallback()` 在 `EXEC_TRAJ` 中每 100 ms 调用 `publishCurrentTrajectory("lifecycle_validation_update")`。该调用只做 revalidation，却重复发送完全相同的 `trajectory_id + start_time`。
2. retained-previous / persistence 路径可能返回 `plan_success=true` 而没有采用新 trajectory；上层仍调用 `publishCurrentTrajectory()`，再次发送原 identity。

FIRST_DUPLICATE_PUBLICATION_FUNCTION:
`EGOReplanFSM::execFSMCallback()` 的 `lifecycle_validation_update` 定时发布路径（随后统一发布函数也会被 retained-previous 路径重复调用）。

FIRST_DUPLICATE_PUBLICATION_CONDITION:
revalidation 或 retained-previous 仍保持相同 `traj_id` 和 `start_time`，但 publication 没有区分“状态更新”与“新 successor adoption”。

## Fix

FIX_APPLIED:

- 移除 EXEC_TRAJ 中每 100 ms 的 `lifecycle_validation_update` 重广播；revalidation 继续运行，但只更新状态。
- 在 `publishCurrentTrajectory()` 增加统一 identity guard：当 `trajectory_id` 和 `start_time` 均与上次实际发布相同，直接抑制广播并记录诊断；只有 identity 变化才发送 PolyTraj/MINCOTraj。
- 接收端原有 stale/duplicate 拒绝逻辑保留不变。

SAME_TRAJECTORY_REPUBLISHED_BY_REVALIDATION: NO (修复后)
TRAJECTORY_ID_REVISION_MONOTONIC: YES（实际安装/广播的 successor identity 单调；retained identity 不再重复广播）
COMMIT_TO_PUBLICATION_ONE_TO_ONE: YES（本次仿真中每个新 identity 一次实际 publication；相同 identity 的 2 次重复调用被 guard 抑制）
PUBLICATION_TO_RECEIVE_ONE_TO_ONE: YES（本次仿真无 receiver duplicate/out-of-order rejection）

## Fixed-scene simulation

SCENARIO: long_cylinder_forest.json

DUPLICATED_TRAJECTORY_WARNING_COUNT_BEFORE: 2895
DUPLICATED_TRAJECTORY_WARNING_COUNT_AFTER: 0

重复 identity 抑制计数：2。该计数发生在发布端，消息没有送到 receiver；receiver 的 duplicate/stale protection 仍然有效。

TERMINAL_HOLD_COUNT: 2
END_BEFORE_NEXT_COUNT: 2

两次 hold 均不是 duplicate 导致：

- predecessor trajectory_id=3 正常被接收并激活；
- 下一条 Joint trajectory_id=4 的 handoff 日志为 `accepted=0`；
- trajectory_id=3 到期后没有可安装 successor，随后进入 hold。

因此 duplicate 修复已闭合，但该场景仍暴露一个独立的 Joint successor acceptance/coverage 问题。本轮按用户要求不修改 Joint。

USER_VISIBLE_STOP: IMPROVED, duplicate-induced stop removed; 仍有 Joint acceptance 导致的 hold
VISUAL_SWITCH_JITTER: UNKNOWN

NOMINAL_TARGET_LOGIC_CHANGED: NO
N_L_R_CHANGED: NO
DYNAMIC_SCP_CHANGED: NO
FREE_TIME_CHANGED: NO
JOINT_ARCHITECTURE_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
STOP_FALLBACK_ADDED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
