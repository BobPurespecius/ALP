# Feedback 76 — Restart simulation hold-cause audit after duplicate fix

本轮只审查，不修改 production code。

## Result

DUPLICATE_FIX_CAUSED_HOLD: NO

证据：

- 本次 restart 日志中 `Received drone ... out of order or duplicated` 为 0。
- identity guard 只抑制了 1 次相同 identity 的重复发布；没有吞掉新的 trajectory identity。
- 两次 hold 都有明确的非 duplicate 原因。

## Hold event 1 — drone 0

predecessor:
trajectory_id=6, start=1789478224.155589342, end=1789478229.356340170

successor candidate:
trajectory_id=7, source=EARLY_JOINT_PRIMARY, activation=1789478224.569224119

planner-side handoff:
`handoff-gate accepted=1`
`planner-traj-commit trajectory_id=7`

executor-side first failure:
`1789478224.099189487 [traj-server-handoff-reject] trajectory_id=7 reason=ACTIVE_PVA_MISMATCH`

因此 planner/manager 认为 Joint successor 已 commit，但 drone-0 traj_server 没有安装它。旧 trajectory 6 继续执行并于 `1789478229.356340170` 到期，随后：

`TERMINAL_HOLD_ENTER trajectory_id=6`
`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`

FIRST_CAUSAL_FAILURE:
`TRAJ_SERVER_SWITCH_DELAY / ACTIVE_PVA_MISMATCH`（Joint successor 与 executor 当前 P/V/A 不一致）

这不是本次 duplicate publication guard 引起的；guard 只发生在发布端完全相同 identity 上，不能改变该 successor 的 P/V/A 内容。

## Hold event 2 — drone 1

predecessor:
trajectory_id=24, start=1789478229.851839066, end=1789478231.789161921

planning evidence:

- `JOINT_FAILURE_LOCAL_NOOP reason=JOINT_TIMEOUT` at 1789478230.410661150
- 随后开始 bounded moving replan，直到 validated coverage 仅剩 1.231431 s。
- 没有在 predecessor 到期前产生/commit trajectory 25。
- `traj-server-tracking-ready ready=0` 后 trajectory 24 到期并进入 hold。

FIRST_CAUSAL_FAILURE:
`NO_CANDIDATE_GENERATED / JOINT_TIMEOUT` followed by insufficient execution coverage

随后 ego planner 进程还出现：

`boost::lock_error: boost: mutex lock failed in pthread_mutex_lock: Invalid argument`
`drone_1_ego_planner_node process has died, exit code -6`

这是另一个独立的进程/线程生命周期故障，发生在 hold 之后或其附近；本轮未修改。

## Counts

DUPLICATED_TRAJECTORY_WARNING_COUNT: 0
IDENTITY_GUARD_SUPPRESSION_COUNT: 1
TERMINAL_HOLD_COUNT: 2
END_BEFORE_NEXT_COUNT: 2

结论：

1. 本次 duplicate 修复有效，receiver duplicate/out-of-order 告警已消失。
2. 频繁卡死的当前主因不是 duplicate publication。
3. 首要真实断点是 Joint successor 在 traj_server 端 `ACTIVE_PVA_MISMATCH` 被拒绝；另一例是 Joint timeout 后 successor 未及时形成。
4. `boost::lock_error` 是额外的 planner 进程崩溃风险，需要后续独立审查。

本轮未修改：

NOMINAL_TARGET_LOGIC_CHANGED: NO
N_L_R_CHANGED: NO
DYNAMIC_SCP_CHANGED: NO
FREE_TIME_CHANGED: NO
JOINT_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
TRAJ_SERVER_CHANGED: NO

SCENARIO: long_cylinder_forest.json
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
