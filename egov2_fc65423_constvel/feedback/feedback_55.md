# Feedback 055 — post-deadline fresh-state recovery、mandatory supply 与 Scenario A 复验

日期：2026-09-13  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_54.md`  
场景：`long_cylinder_forest_visibility_stress.json`

## 0. 结论

本轮已修复 Feedback054 确认的永久 liveness trap：过期 predecessor 不再把以后每轮 mandatory planning budget 永久压成 0。系统会如实记录 coverage failure，结束旧 handoff deadline authority，从真实当前 odometry P/V（沿用现有 acceleration convention）进入同一套 NOMINAL/LEFT/RIGHT、A*/Local-SFC、MINCO/SCP 与 current-revision final preflight。旧 suffix 没有被延长，unsafe candidate 没有被执行。

真实运行证明这个软件死锁已经消失：所有发生 hold 的 UAV 都能在后续 hard-safe candidate 出现后重新激活 moving trajectory，不再像 Feedback054 那样永久停在 trajectory 177。但七次保留式 FULL ON 尝试均至少出现一次 terminal hold、starvation、unvalidated execution、collision sample 中的一项，因此没有取得用户定义的健康 run。用户随后要求停止继续仿真、清理前三轮数据并立即报告；本轮在最终保守源码完成 build/contracts 后停止。

```text
FINAL_ASSESSMENT = PARTIAL

POST_DEADLINE_LIVENESS_TRAP_FIXED = YES
PERMANENT_ZERO_BUDGET_LOOP_REPRODUCED_AFTER_FIX = NO
POST_DEADLINE_RECOVERY_EVENTUALLY_REACTIVATES = YES
HEALTHY_FULL_RUN_FOUND = NO
FULL_RUN_COUNT = 7
```

`PARTIAL` 的含义是：代码级永久 deadlock、fresh-state authority、mandatory/optional 分离和 executor identity合同已经闭合；但“任何一次完整运行都不发生 hold/starvation/unvalidated/collision”的运行级验收没有闭合。

## 1. 原始 failure 与代码根因

Feedback054 中 UAV0 trajectory 177 的第一层物理失败是正确 hard reject：NOMINAL/LEFT/RIGHT 的 dynamic clearance 约 `0.35–0.93 m`，低于 `1.1 m`。真正的软件永久失效发生在其后：

```text
now + activation_margin >= predecessor validated_end
→ predecessor 已无 handoff 能力
→ planning_budget_remaining < 0
→ ensureExecutionCoverage() 仍用旧 deadline 计算 fast_budget=0
→ optionalRefinementAllowed() 同时剪掉 mandatory supply
→ candidate_seen=0
→ 下一周期继续继承同一个过期 authority
→ permanent terminal hold
```

因此：

```text
FIRST_ORIGINAL_FAILURE:
  EGOPlannerManager::reboundReplan dynamic final preflight < 1.1 m
  （正确的物理拒绝）

ROOT_CAUSE:
  EGOPlannerManager::ensureExecutionCoverage 把已经过期的 predecessor
  继续当作以后 mandatory planning 的 wall-deadline authority，且 optional
  quality gate 同时控制 first-safe supply。
```

## 2. Production 修改

### 2.1 明确区分 predecessor handoff 与 post-deadline recovery

`local_execution_contract.h` 新增纯合同：

```text
PREDECESSOR_HANDOFF
  budget = min(pipeline estimate, predecessor remaining budget)

POST_DEADLINE_RECOVERY
  predecessor deadline authority结束
  budget = bounded current pipeline runtime estimate
```

该 budget 仍由已有 execution runtime estimate、optimizer cancellation/checkpoint 和 FSM cadence 限界；没有拍脑袋新增 0.3/0.5 s 固定阈值。

### 2.2 从真实当前状态复用同一 Planner

`planner_manager.cpp` 与 `ego_replan_fsm.cpp` 在进入 recovery epoch 时：

- 记录过期 predecessor identity，不改写历史 failure；
- 读取现有 authoritative odometry position/velocity；
- 沿用生产中的 zero-acceleration fresh-start convention；
- 重新生成 fresh moving initializer；
- 继续使用原 NOMINAL、SIDE_PLUS、SIDE_MINUS、A*、Local-SFC、MINCO/SCP；
- candidate 必须重新通过 dynamics/static/dynamic/swarm/current-revision/handoff/final preflight。

这不是第二套 Planner，也不是 brake、stop 或旧 suffix extension。

### 2.3 mandatory supply 与 optional quality 分离

新增 `mandatoryPlanningAttemptAllowed()`：在还没有 executable candidate 时，NOMINAL 与第一轮必要 LEFT/RIGHT 属于 supply，可使用当前有界 recovery deadline；出现 first-safe reserve 后，额外 quality search 才受 `optionalRefinementAllowed()` 的 uninterruptible reserve约束。

同时修复两处直接 candidate-entry 缺口：

1. shared visibility support 已经存在且 trajectory/diagnostics finite 时，即使 nonlinear NOMINAL solve 本轮失败，也允许触发 SIDE generation；
2. 在 first-safe 出现前，必要 SIDE 不能因 optional quality gate 关闭而被提前裁掉。

### 2.4 Executor fresh-state handoff

`trajectory_lifecycle.h` 与 `traj_server.cpp` 只在以下条件同时成立时接受 post-deadline takeover：

- planner 显式标记 `POST_DEADLINE_RECOVERY_VALIDATED`；
- full hard preflight 已通过；
- new generation 严格大于 expired predecessor generation；
- activation 不早于旧 trajectory 的有限终点；
- recovery identity 与新 trajectory ID 一致。

因此它不会把弱 P/V/A proximity、raw A* 或未验证候选冒充安全 successor。

### 2.5 被撤回的实验入口

Run5–Run7 前曾试验允许“动态仍不安全、但 risk finite”的 visibility SIDE seed进入 A*。真实结果把 LEFT/RIGHT A* 尝试放大到数百次，并产生 9–36 s 的异常长重定时候选；A*只修 static observation topology，并不能提供动态避障证明。该实验在最终源码中已撤回：visibility SIDE 的 A* base重新要求原 `1.1 m` 动态绝对安全，最终 hard checker始终未变。

这项撤回避免把 CPU/搜索负载当成动态 supply；没有删除原来已验证的“动态安全 seed 的 same-topology static A* rescue”。

## 3. 修改文件

Production：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  plan_manage/include/plan_manage/local_execution_contract.h
  plan_manage/include/plan_manage/local_visibility_preference.h
  plan_manage/include/plan_manage/planner_manager.h
  plan_manage/src/planner_manager.cpp
  plan_manage/src/ego_replan_fsm.cpp
  plan_manage/src/traj_server.cpp
  traj_utils/include/traj_utils/trajectory_lifecycle.h
```

Tests：

```text
  plan_manage/test/local_execution_contract_test.cpp
  plan_manage/test/trajectory_lifecycle_wiring_test.py
  plan_manage/test/recovery_nonblocking_wiring_test.py
```

没有修改 visibility-first 排序、J_vis deep-risk、Q_dir、25°/170°定义、1.1 m dynamic clearance、static/swarm阈值、brake/stop、ACK/commit协议或 Stage2/3A/3B 主公式。

## 4. 合同与构建

覆盖的确定性 Case：

| Case | 合同 | 结果 |
|---|---|---|
| A | predecessor仍可handoff时继续使用原 validated-end deadline | PASS |
| B | predecessor过期后旧deadline结束，mandatory recovery仍有非零机会 | PASS |
| C | obstacle仍阻塞时hard reject，下一周期仍有规划机会 | PASS |
| D | obstacle离开后fresh-state candidate经full preflight并由executor激活 | PASS |
| E | optional quality耗尽不能阻止mandatory first-safe supply | PASS |
| F | 无安全解时fail closed，不生成brake/stop或伪safe | PASS |
| G | visibility-first non-strict candidate仍可参与执行 | PASS |

最终保守源码：

```text
catkin build ego_planner -j2 --no-status --workspace ros_ws = PASS
local_execution_contract_test                         = PASS
visibility_topology_production_test                   = 8/8 PASS
trajectory_lifecycle_wiring_test.py                   = 17/17 PASS
recovery_nonblocking_wiring_test.py                   = 7/7 PASS
adaptive_execution_contract_test.py                   = 12/12 PASS
git diff --check                                      = PASS
```

第一次 topology binary 运行因没有启动 roscore 而等待 master；这不是合同失败。使用临时 roscore 重跑后 8 项全部通过，临时 master随后已停止。构建通过后按用户清理要求删除了可完全重建的 `ros_ws/build` 与 catkin logs；上述结果来自删除前的最终源码构建。

## 5. 七次 Scenario A FULL ON

所有运行均使用 `long_cylinder_forest_visibility_stress.json`、FULL ON、native RViz；没有 OFF/A-B，没有降低安全阈值，也没有因结果差而从计数中删除。全部 target route完成。

| Run | 源码阶段 | Hold | Starvation | End-before-next | Unvalidated | Collision | Swarm violation | Camera-time | K2 | All3 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | post-deadline core | 2 | 旧分析0* | 2 | 305 | 0 | 0 | 221.887 | 0.9683 | 0.7999 |
| 2 | post-deadline core | 1 | 1 | 2 | 39 | 16 | 0 | 224.141 | 0.9851 | 0.8131 |
| 3 | post-deadline core | 1 | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | recorder incomplete | UNKNOWN | UNKNOWN |
| 4 | +shared trigger/+mandatory SIDE；最终保守语义 | 2 | 2 | 4 | 170 | 0 | 0 | 225.987 | 0.9929 | 0.8260 |
| 5 | experimental broad A* entry | 2 | 2 | 4 | 99 | 0 | 0 | 223.515 | 0.9790 | 0.8098 |
| 6 | experimental broad A* entry | 1 | 1 | 2 | 52 | 15 | 0 | 228.536 | 0.9871 | 0.8632 |
| 7 | experimental broad A* entry | 5 | 6 | 10 | 405 | 26 | 0 | 211.241 | 0.9750 | 0.6677 |

`*` Run1 的旧 pairwise starvation analyzer漏掉右删失末端事件；Hold/End-before-next已经证明该 run不健康，不能按 starvation=0判PASS。

Run3 的原始 recorder不完整；可确认一次约8.85 s terminal hold及随后恢复，但其余数值写 `UNKNOWN`，不补造。

```text
HEALTHY_RUN_CRITERION:
  hold=0, starvation=0, end-before-next=0, unvalidated=0,
  collision=0, swarm_violation=0

HEALTHY_RUNS = 0 / 7
```

因此没有依据按“健康 run 后再评价 visibility”的口径，对 Feedback053 或历史 OFF给出正式收益结论。Run4的K2较高、Run6的All3较高，都被 lifecycle/collision失败污染，不能作为验收样本。

## 6. 修复后 failure 的真实边界

### 6.1 永久软件 trap 已消失

每个 post-deadline episode均继续记录新的 `POST_DEADLINE_RECOVERY_ATTEMPT`；若 obstacle尚未离开，unsafe candidate继续被hard reject；当后续出现safe candidate时，日志形成：

```text
POST_DEADLINE_RECOVERY_SUCCESS
→ POST_DEADLINE_EXECUTOR_HANDOFF_ACCEPTED
→ POST_DEADLINE_RECOVERY_EXIT
→ new moving trajectory activation
```

Run4–Run7 的 hold均为有限时长后恢复，不再是旧 predecessor永久把 `candidate_seen` 压成0。

### 6.2 为什么仍未达到 zero-hold 验收

剩余的第一个断点发生在 predecessor到期之前的 candidate supply：

- Run4：moving obstacle窗口内best dynamic clearance约 `0.65–0.82 m < 1.1 m`；无 hard-safe successor，约5.3 s后障碍离开才恢复；
- Run5：一个episode有动态安全候选但找不到同时static-safe的完整candidate；另一个best dynamic约 `0.811 m`；分别约2.24/1.04 s后恢复；
- Run6：NOMINAL失败，SIDE A*/Local-SFC产生的长重定时candidate继续在static/dynamic/SCP/final preflight失败；约1.63 s后恢复；
- Run7：broad A*入口显著增加昂贵且最终无效的candidate，出现5次hold和26个collision samples，证明该实验不能保留。

这些 episode的 hard reject本身正确。post-deadline恢复解决的是“暂时无解不能演变成软件永久死锁”，并不能倒推 predecessor到期前物理上必然存在安全moving solution。

```text
SECONDARY_FAILURE:
  predecessor到期前没有及时形成同时通过static/dynamic/dynamics/swarm/final
  preflight的moving successor。

SECONDARY_ROOT_CAUSE:
  各episode分别是dynamic clearance真实不足、static-safe path supply不足，或
  A*/retiming/SCP生成端未在deadline内产生可被final checker接纳的trajectory；
  不是executor接受了unsafe/unvalidated candidate。

SECONDARY_FIX:
  保留shared-risk触发和mandatory SIDE supply；撤回动态不安全seed进入A*的
  过宽实验。没有证据支持进一步降低阈值或绕过preflight，因此本轮停止。
```

## 7. Telemetry

新增/完善：

```text
PREDECESSOR_HANDOFF_DEADLINE_EXPIRED
POST_DEADLINE_RECOVERY_ENTER
POST_DEADLINE_RECOVERY_STATE
POST_DEADLINE_RECOVERY_START_AUTHORITY
POST_DEADLINE_RECOVERY_ATTEMPT
POST_DEADLINE_RECOVERY_NO_SAFE_CANDIDATE
POST_DEADLINE_RECOVERY_SUCCESS
POST_DEADLINE_EXECUTOR_HANDOFF_ACCEPTED
POST_DEADLINE_RECOVERY_EXIT
```

记录 recovery epoch、expired predecessor ID、current P/V/A source、wall budget、N/L/R attempts、candidate count、best clearance、selected candidate/revision、新trajectory ID与activation。`NO_SAFE_CANDIDATE`按episode节流，不再把一个failure解释成数千个独立failure。

## 8. 清理

按用户追加要求，报告前已执行：

- 永久删除本任务最早三轮原始仿真目录：`run1`、`run2`、`run3`；
- 保留 `analysis/run1/run1_metrics.json`、`analysis/run2/run2_metrics.json` 等compact汇总；
- Run3原本没有完整metrics，报告仅保留可证实的hold事实；
- 删除可完全重建的 `ros_ws/build` 与 `ros_ws/logs`；
- 保留 Run4–Run7 原始证据及全部源码；
- 清理前约291 MiB旧仿真数据与约770 MiB build cache，文件系统可用空间从约1.8 GiB恢复到约2.8 GiB。

被删除原始数据不可直接恢复，但均属于可重新生成的仿真/build产物；未删除production或关键汇总报告。

最终进程检查没有 roscore、rosmaster、roslaunch、RViz、Gazebo或recorder残留。

## 9. 最终字段

```text
POST_DEADLINE_LIVENESS_TRAP_FIXED: YES

EXPIRED_PREDECESSOR_STILL_CONTROLS_MANDATORY_BUDGET: NO

POST_DEADLINE_FRESH_MOVING_RECOVERY_IMPLEMENTED: YES
POST_DEADLINE_RECOVERY_USES_REAL_CURRENT_STATE: YES
POST_DEADLINE_FULL_HARD_PREFLIGHT: YES

OPTIONAL_QUALITY_CAN_BLOCK_MANDATORY_SUPPLY: NO

DYNAMIC_SUPPLY_MODIFIED: YES
DYNAMIC_SUPPLY_CHANGE:
  shared finite visibility-risk evidence survives a failed nominal solve;
  necessary LEFT/RIGHT remain mandatory until first-safe supply exists.

SAFETY_THRESHOLD_RELAXED: NO
HARD_SAFETY_BYPASSED: NO
OLD_VALIDATED_SUFFIX_EXTENDED: NO
PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO

HEALTHY_FULL_RUN_FOUND: NO
FULL_RUN_COUNT: 7

TERMINAL_HOLD_COUNT: no healthy run; per-run = 2/1/1/2/2/1/5
MOVING_SUCCESSOR_STARVATION_COUNT: no healthy run; per-run = 0*/1/UNKNOWN/2/2/1/6
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: no healthy run; per-run = 2/2/UNKNOWN/4/4/2/10
UNVALIDATED_EXECUTED_SAMPLES: no healthy run; per-run = 305/39/UNKNOWN/170/99/52/405
COLLISION_SAMPLES: no healthy run; per-run = 0/16/UNKNOWN/0/0/15/26
SWARM_CLEARANCE_VIOLATION_SAMPLES: known runs = 0

VISIBILITY_FIRST_SEMANTICS_PRESERVED: YES
STRICT_25_170_REMAINS_NON_HARD: YES

CAMERA_TIME: NO_HEALTHY_RUN
K2: NO_HEALTHY_RUN
ALL3: NO_HEALTHY_RUN
ACTUAL_Q_DIR_MEAN: NO_HEALTHY_RUN
EXECUTED_ENCIRCLEMENT_RATIO: NO_HEALTHY_RUN

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

最终判断：旧 deadline 导致的永久 terminal hold已经修复为可恢复的fresh-state、full-preflight planning；但在用户要求结束时，Scenario A 仍未出现一条同时满足全部 zero-failure条件的完整run，因此本轮不能写成健康验收通过。
