# Feedback 131 — 冻结激活 + validation-only handoff（第一轮闭环）

日期：2026-09-27。工作区：`/home/bob/ALP/egov2_fc65423_constvel`（dirty worktree 唯一权威）。本 feedback 覆盖修复模式的第一轮：架构修复 + 编译 + 一次有效 FULL + 真实结果分析。

## 本轮根因

Feedback130 定位的主故障在源码中确认：`finalizeCapturedCandidates()` 在候选**优化完成之后**把 activation 重算为 `now + local_activation_margin_`，随后 `prepareLocalHandoff()` 对每个候选用新 activation 重采样 predecessor 的 P/V/A，`opt.reset(head,tail,...)` + `opt.generate(...)` 重新生成五次多项式——保留旧 inner points/durations/tail、替换 head——不再进入任何 P/T 优化，然后才交给 hard preflight。optimizer 用旧 head 保证的 PVAJ 在新多项式上不成立，jerk 超限 → `DYNAMICS_FAIL` → SIDE 成批死于终检（F130 的 8/12 E 类），NOMINAL 作为唯一 hard-safe 幸存者被提交，载具以惯性穿轴产生 K3 loss。

## 代码修改（single frozen activation 架构）

- `prepareFutureActivation()`：batch 的唯一 activation 在**生成开始前**冻结：
  `frozen = plan_start + execution_budget_.estimate(wall) + local_activation_margin_`，
  并被 predecessor 的 `planning_deadline_ros_`（= validated_end − margin）上界约束、`plan_start + margin` 下界约束。reserve 复用 `canStart` 同一个 budget authority，未新增任何 timing estimator。restart 模式（odometry authority）不加 reserve。
- 生成侧自动共享同一世界时原点：generation head（`start_pt_/vel_/acc_`，prepareFutureActivation 在冻结时刻采样）、target epoch（`setTargetState(..., local_activation_time_)`）、`planning_prediction_epoch = local_activation_time_`（已存在的单 epoch 契约）。
- `finalizeCapturedCandidates()`：`activation = local_activation_time_`（冻结值），不再从 `now` 重推导。
- 新增 `validateLocalHandoffWindow()`：validation-only 的调度检查（冻结槽是否错过/前驱覆盖是否耗尽）。restart 模式直接放行（odometry authority 无 predecessor 窗口，连续性由 `checkActiveHandoff` 的 restart 分支验证）。**该函数不触碰多项式。**
- finalize 逐候选路径改走 `validateLocalHandoffWindow` + 既有 `checkActiveHandoff`（残差容差验证，日志打开），并新增 payload hash 前后一致性 tripwire：`POST_OPT_PAYLOAD_MUTATED`。
- `prepareLocalHandoff` 重命名为 `rebuildLocalCandidateAtActivation`，注释明确为**外部 payload 专用 rebuild 契约**（Team relay anchor probe、防御性 commit），Local N/L/R 生产契约永不进入。两个保留调用点同步改名。
- 新日志：`[candidate-batch-activation]`（含 plan_start/planning_reserve/frozen_before_optimization）、`[handoff-validation]`、`[FROZEN_CANDIDATE]` 扩展 `handoff_mutation=0 hash_before/after`、`[future-activation]`（activation_frozen）。

## 三原则核对

- 不阻塞：错过冻结槽的整批拒绝（MISSED_FROZEN_ACTIVATION），安全 incumbent 继续执行，下一 rolling tick 重冻结；无任何等待。
- 各司其职：activation scheduler（prepareFutureActivation 冻结）→ optimizer（唯一多项式权威）→ handoff（纯验证）→ SafetyKernel（不变）→ selector（不变）。rebuild 权威只留给外部 payload（Team/防御路径），且每次 rebuild 后必须显式验证。
- 架构简洁：无新 FSM/dwell/hysteresis/magic margin；单一不变量 `generated = optimized = validated = committed payload`，由 hash tripwire 强制。

## build

`git diff --check` 未运行（保持与既有会话相同口径：无 git 状态操作）；`catkin build -j2 --no-status` **25/25 succeeded、Warnings: None**。

## FULL run id 与结果

第一此尝试 `20260927_040458_899486` 因**我传入相对场景路径**导致场景节点 `FileNotFoundError` 自杀（`pose_histories=0/11` 全程）、drone0 无预测永久重试、BOOT-08 超时——基础设施错误，与补丁无关；保留 `SCENE_NODE_FATAL_TRACEBACK_EXCERPT.txt`。

有效 run：`20260927_041442_918315`，**BOOT-12、FINAL_EXIT_CODE=0、CLEANUP OK（29 进程全退）**。

| 指标 | 基线 (20260926_225042) | 本 run | 判定 |
|---|---:|---:|---|
| K2 | 0.994771 | **1.000000** | ✓ K2 loss=0 |
| ALL3/K3 | 0.892375 | **0.882789** | 略降 0.96pp |
| blackout | 0.400s loss | **0** | ✓ |
| mean_visible | — | 2.8828 | — |
| SIDE DYNAMICS_FAIL | 92 | **46** | 减半，未清零 |
| `SUCCESSOR_REANCHORED`（finalize mutation） | 4667 | **0** | ✓ 变异已消灭 |
| `CANDIDATE_REANCHORED`（restart mutation） | 27732 | **0** | ✓ |
| `POST_OPT_PAYLOAD_MUTATED` / `FROZEN_ACTIVATION_MUTATED` | — | **0 / 0** | ✓ 不变量成立 |
| TERMINAL_HOLD_ENTER | 29 | **194** | ✗ churn 上升 |
| MOVING_SUCCESSOR_STARVATION | 5 | **53** | ✗ |
| NO_EXECUTABLE_SUCCESSOR | 158 | 274 | ✗ |
| PVA mismatch / unvalidated commit | 0 / 0 | **0 / 0** | ✓ |
| 接触/碰撞 | 0 | **0**（min_dynamic_clearance=0.3243）| ✓ |

全盲结构：8 段（uav1×4=4.85s、uav3×3=3.0s、uav2×1=1.0s），与基线 7 段同构（同一场景遮挡走廊）。K3 事件 6 个；46/112 个 K3 batch 三方 hard-safe 全灭（DYNAMICS_FAIL 46 + HARD_DYNAMIC_COLLISION_FAIL 15 是主因）。

## 是否真正改善

**机制层：是。** post-optimization polynomial mutation 从架构上消灭（两处 REANCHORED 计数 4667+27732→0，hash tripwire 零触发），SIDE 因终检 DYNAMICS_FAIL 的死亡率减半，K2 达到 100%、blackout 0、执行安全性无回退。

**liveness 层：出现新回退。** coverage < 2·margin（0.20s）的"死区"内，冻结下界（plan_start+margin）与验证下界（finalize_now+margin）结构性不可同时满足——任何正 latency 必然 MISS（实测晚 1.5-2.5ms），2245 次拒绝、295 次 restart churn（drone1 占 2081 次，对应 UAV2 两次减速 6.36s）。基线的"平滑"实际上是靠 27732 次 finalize/restart mutation 掩盖了这个死区——移除变异后死区暴露，但 restart authority 以 ~60ms 恢复且 K2 保持 100%，安全性未破。

## 剩余第一根因（下一轮处理）

K3 loss 的第一 production 根因：**优化器与 kernel 的动力学判定不同物理**。`SIDE_MINCO success=1` 的 payload 在终检 `max_j=22.99 > 20` 被杀：优化器的行约束/验收用均匀 α 采样 lattice（每 piece cps_num 个样本），jerk 为分段二次多项式，采样之间的峰值逃逸（实测 15% 超限）；kernel 的 jerk 检查也是采样（dt≤0.05s），只是更密。`side_retimed=false` 已确认 retiming 不是变异源。修复方向：两侧统一为**解析精确**的 jerk 极值（jerk 二次 → |j|² 四次 → 导数三次，闭式+RootFinder），行约束落在极值点上。

## 下一步

1. 库层 `Piece::getMaxJerRate()/getJerStationaryAlphas()`（解析极值）+ `Trajectory::getMaxJerRate()`；
2. kernel `checkTrajectoryDynamics` 的 jerk 改用解析极值；
3. optimizer `forEachCandidateDynamicsSample` 与 `predictDynamicsForStep` 增加 jerk 极值点采样（行=验收=kernel 同一物理量）；
4. 离线验证后再次 FULL，目标：SIDE hard-safe DYNAMICS_FAIL→0、ALL3 回升。

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES  # 25/25, Warnings: None
FULL_RUN_COUNT_THIS_TASK: 1  # 20260927_041442_918315 (另有 1 次基础设施失败尝试，未计入有效 run)
REPEATED_FULL_SIMULATION_USED: NO

POST_OPT_HANDOFF_MUTATION: 0  # REANCHORED 4667+27732 -> 0; hash tripwire 零触发
SIDE_DYNAMICS_FAIL: 46  # baseline 92
SIDE_HARD_SAFE_RATE: 提升但未量化到 per-transition 口径
K2: 1.000000
K3_ALL3: 0.882789
K3_LOSS_TOTAL: 8.97s (8 episodes: 4.85+3.0+1.02)
BLACKOUT: 0
CONTACTS: 0 (min_dynamic_clearance 0.3243)
UNVALIDATED_COMMITS: 0
PVAJ_VIOLATION_COMMITS: 0
HANDOFF_FAILURES: 0 硬失败; 2245 MISSED_FROZEN_ACTIVATION(死区 churn)
PLANNER_DEADLOCK: 0
STARVATION: 53 (baseline 5)
TERMINAL_HOLD_ENTER: 194 (baseline 29, restart 60ms 快速恢复)

FIRST_CAUSAL_FAILURE: 优化器采样 lattice 与 kernel 动力学判定不同物理，jerk 峰值在采样间逃逸(22.99>20)
NEXT_STEP: 解析精确 jerk 极值统一两侧判定后再次 FULL

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[run exit](../runs/20260927_041442_918315/exit_status.txt)、[visibility CSV](../runs/20260927_041442_918315/visibility.csv)、[执行轨迹 CSV](../runs/20260927_041442_918315/visibility_trajectory.csv)、[run1 场景节点自杀摘录](../runs/20260927_040458_899486/SCENE_NODE_FATAL_TRACEBACK_EXCERPT.txt)。
