# Feedback 070 — Local dynamic repair wiring and hard-SCP verification

本轮使用固定合围场景：`long_cylinder_forest.json`（10 dynamic obstacles，`enable_encirclement_tracking=true`）。未访问或修改 RRCT。

## 结果

PRIMARY_ROOT_CAUSE: moving-object cost/gradient 诊断字段没有在 `addPVAGradCost2CT()` 独立累加，且 SIDE hard-SCP 原先没有动态 body clearance row；因此 repair-only seed 虽进入优化器，动态项在日志中显示为 0，优化器也没有动态硬可行性约束。

FIX_APPLIED:

- `addPVAGradCost2CT()` 增加独立 `moving_cost_accum` / `moving_grad_sq_accum`，写入 `current_cost_snapshot_.moving_cost` 和 `current_moving_grad_sq_`。
- `runCandidateHardCorridorSCP()` 增加基于当前预测 epoch 的动态 body clearance 线性化 row，保留 P 与 virtual-T Jacobian；未改变 1.1 m 阈值。
- SCP trial merit/acceptance 增加动态非线性 violation，避免接受动态安全变差的 trial。
- SCP 最终阶段增加动态 violation 检查；unsafe candidate 仍无 execution authority，最终 hard preflight 仍是执行准入条件。

## 真实运行证据

编译：`catkin build traj_opt ego_planner -j2 --no-status --workspace ros_ws` PASS。

`dynamic_repair_validation_20260915/full_encirclement_fix4.log` 中确认：

- `minco-gradient-audit` 的 `moving_cost` / `moving_grad` 出现非零，例如 `moving_cost=1.46e-06`, `moving_grad=1.03e-03`。
- 动态冲突 candidate 出现 `SCP_FINAL_DYNAMIC_VIOLATION`，说明动态 hard 检查已进入 SCP 终态路径。
- 运行在首次 20 s 观察窗口内未出现 `TERMINAL_HOLD_ENTER` 或 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`；该窗口中仍有大量 `DYNAMIC_FAIL`（279 次，按日志关键字计数），因此不能宣称完整闭环已完成。
- 未观察到 `UNVALIDATED_EXECUTION`、`EXECUTED_COLLISION` 或 `SWARM_VIOLATION`。

TERMINAL_HOLD_COUNT: 0（fix4 观察窗口）
END_BEFORE_NEXT_COUNT: 0（fix4 观察窗口）
EXECUTED_COLLISION_COUNT: 0
UNVALIDATED_EXECUTION_COUNT: 0
SWARM_VIOLATION_COUNT: 0

DYNAMIC_REPAIR_CHAIN_COMPLETE: NO
MOVING_COST_ACTIVE_ON_CONFLICT: YES
MOVING_GRAD_ACTIVE_ON_CONFLICT: YES
DYNAMIC_TIME_REPAIR_ACTIVE: PARTIAL（virtual-T row 已接入，但当前冲突窗口仍有失败）
DYNAMIC_SPACE_REPAIR_ACTIVE: PARTIAL（P row 已接入；部分 candidate 仍未达到 1.1 m）
DYNAMIC_FAIL_TO_SAFE_SUCCESSOR_OBSERVED: NO
STATIC_ASTAR_DYNAMIC_REPAIR_CHAIN_WORKS: PARTIAL（静态 A* 与 Local-SCP 可运行；动态 hard preflight 仍拒绝若干 candidate）

## 未完成项

本轮没有降低安全阈值、冻结 FREE_TIME、执行 unsafe seed、修改 Joint/BODY-LOS topology、增加 stop/brake fallback。真实日志显示部分 `SCP_FINAL_OK` 记录仍伴随低于 1.1 m 的 manager dynamic-clearance 值，说明 optimizer epoch/最终动态 clearance 记录之间仍需下一轮专门对齐审查；本轮不再扩大修改范围。

PRODUCTION_SOURCE_CHANGED: YES
SAFETY_THRESHOLD_CHANGED: NO
FREE_TIME_FROZEN_REINTRODUCED: NO
BODY_LOS_TOPOLOGY_CHANGED: NO
JOINT_ARCHITECTURE_CHANGED: NO
STOP_FALLBACK_ADDED: NO
PLANNER_BRAKE_ADDED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
