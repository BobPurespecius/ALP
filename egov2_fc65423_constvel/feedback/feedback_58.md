# Feedback 058 — Unified Rolling Team Planning Transaction：合同完成、真实集成未闭合

日期：2026-09-13  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_57.md`  
场景：`long_cylinder_forest_visibility_stress.json`

## 0. 结论

本轮已把 `LOCAL_RESERVE` 与 `EARLY_JOINT_QUALITY` 建模为同一 transaction 内的两个候选值，并加入 immutable identity、统一 common activation/boundary、一次 decision/commit、team-local swarm compatibility、abort 后 local authority 恢复，以及统一 proposal→3 ACK→commit 合同。确定性 Case A–H、Feedback57 race fixture、build 和既有相关测试均通过。

但真实集成没有闭合。第一版 Run1 仍由 coordinator 重新选择 activation/rehead，造成 19 个不同 team solution 出现 `TRAJECTORY_BOUNDARY_MISMATCH`；随后改为 planner 预声明 boundary。Run2 暴露三机预声明 slot 不一致后只等待、不释放旧 transaction 的 livelock，已改为 fail-closed abort。最终 Run3 又暴露一个更具体的 producer/consumer 合同错误：coordinator 要求 `target_prediction_epoch == common_activation`，而 planner 实际发布 `target_prediction_epoch=current->planning_epoch`、`dynamic_prediction_epoch=common_activation`。即使日志中的三机 transaction ID 和 activation 完全相同，也会被 `PREDECLARED_TRANSACTION_NOT_ALIGNED` 拒绝。

用户要求立刻停止后，没有继续修复该条件、重新 build 或启动第四次仿真。最终源码因此是“确定性合同通过、真实 transaction 入口尚未可用”的中间状态，不能写成实现完成或健康验收通过。

```text
FINAL_ASSESSMENT: PARTIAL / RUNTIME FAIL
UNIFIED_TRANSACTION_ARCHITECTURE_PROVEN: CONTRACT_ONLY
HEALTHY_FUSED_TRANSACTION_RUN_FOUND: NO
FULL_ON_ATTEMPTS: 3 (Run2 deliberately stopped at 69.47 s)
```

## 1. 架构审计与修改

### 1.1 修改前

```text
local first-safe may commit
while early joint works on an older future boundary
→ local revision / future owner advances
→ later joint takeover attempts ACK against stale boundary
→ TRAJECTORY_BOUNDARY_MISMATCH
```

local commit authority位于 `EGOPlannerManager::setLocalTrajFromOpt()` 及其 handoff/trajectory generation 更新链；Feedback57 的 early joint context、seed、optimizer 和 preflight在异步 coordinator 链中运行，proposal 前使用的 predecessor可能已不再是 seed 创建时的 owner。

### 1.2 修改后的目标结构

```text
transaction G
→ one predeclared common activation and per-UAV predecessor P/V/A boundary
→ TEAM_LOCAL_RESERVE (liveness candidate)
→ EARLY_JOINT_PRIMARY (quality candidate)
→ one final visibility-first decision
→ one proposal
→ 3 ACK
→ one commit
→ common activation
```

`LOCAL_RESERVE` 在 eligible transaction 内只进入 pending candidate，不立即改变 predecessor；joint失败、超时、context失配或 coverage不足时 transaction abort，原 local commit authority恢复。三条 individually safe local reserve还必须通过既有 team swarm compatibility，才能形成 `TEAM_LOCAL_RESERVE`。

本轮没有加入 lease、reservation、takeover、rebase或新 recovery FSM。pending期间 Planner仍执行新的 candidate/SIDE/A*/SFC/MINCO/preflight计算；同一 transaction boundary不匹配的更新被跳过或触发abort，不是暂停Planner。

## 2. 修改文件与生产合同

主要 production 修改：

```text
ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  unified_team_transaction.h                              (new)
  team_planning_context.h

ros_ws/src/multi_uav_formation/src/
  multi_uav_topology_coordinator.cpp

ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  plan_manage/include/plan_manage/planner_manager.h
  plan_manage/src/planner_manager.cpp
  plan_manage/src/ego_replan_fsm.cpp
  plan_manage/src/traj_server.cpp
  traj_utils/msg/TopologyCandidateBundle.msg
  traj_utils/msg/TeamTrajectorySolution.msg

ros_ws/src/multi_uav_formation/CMakeLists.txt
```

测试修改/新增：

```text
multi_uav_formation/test/unified_team_transaction_contract_test.cpp (new)
multi_uav_formation/test/early_joint_primary_contract_test.cpp
plan_manage/test/trajectory_lifecycle_wiring_test.py
```

`UnifiedTeamPlanningTransaction` 的 production contract提供：

- `PREPARING→RESERVE_READY→QUALITY_SEARCH→DECIDED→COMMITTED`，任意未提交阶段可 `ABORTED`；
- immutable transaction identity；
- local reserve team hard-safe/swarm-compatible admission；
- joint只作为visibility-first quality replacement；
- `decision_count <= 1`、`commit_count <= 1`；
- stale generation/revision/activation fail closed；
- coverage不足时不进入 team quality transaction。

## 3. 确定性合同、build 与测试

最终仿真前执行：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws
```

结果：PASS。随后完整执行18个既有相关C++ binaries、4个transaction目标、38项Python wiring/contracts；全部PASS，`git diff --check` PASS。测试包含既有analytic/finite-difference gradient、visibility topology、lifecycle、recovery、adaptive execution与early-joint contracts。

四个transaction目标：

```text
unified_team_transaction_contract_test       PASS
feedback57_boundary_race_fixture             PASS
one_transaction_one_commit_contract          PASS
transaction_abort_liveness_contract          PASS
```

Case A–H 均PASS：joint优胜的一次提交、joint失败的team-local fallback、同boundary local reserve更新、mandatory abort/liveness、team-local swarm冲突拒绝、较差joint不替换、individual sacrifice但team更优可替换、stale identity fail closed。

确定性 fixture 未删除 ACK boundary check：旧控制流可复现 local commit 后 mismatch；新控制流中 reserve不提前commit，one decision/one commit且 mismatch=0。

这些测试仍漏掉了真实 producer 语义差异：fixture把 `target_prediction_epoch` 和 activation构造成同一值，而 runtime planner发布的是不同概念。这是测试通过、Run3入口全拒绝的直接原因。

构建结果对应 Run3 源码。用户要求停止后没有进行后续代码修改，因此不需要也没有再次build。

## 4. 三次 FULL ON 结果

三次均为同一场景SHA256：

```text
430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
```

| Run | 源码阶段 | 路线 | transaction/joint结果 | Hold / end-before-next | unvalidated | collision / swarm violation |
|---:|---|---|---|---:|---:|---:|
| 1 | 初始统一对象，但coordinator仍rehead | 80.2045 s完成 | joint success 21；23 proposals；2次3ACK/commit；1 joint + 1 team-local三机激活；19个mismatch solution IDs | 2 / 2 | 56 | 0 / 0 |
| 2 | planner预声明activation；未对齐仍等待 | 69.47 s主动结束诊断 | 116 unique transactions，0 joint success/proposal；115 abort；117条misalignment reason、314条timeout reason | 0 / 0（仅已运行窗口） | 0 | 0 / 0 |
| 3 | 未对齐立即abort的最终源码 | 80.2045 s完成 | 153 unique transactions，151 abort；0 joint success/proposal/commit/activation；695条misalignment reason | 2 / 2 | 193 | 0 / 0 |

Run1 的两个完整三机激活证明 unified proposal/3ACK/commit payload链能够执行，但它不属于最终predeclared-boundary架构，且运行本身不健康。Run2不是完整运行。Run3是最终源码的完整运行，也是最终runtime authority。

所有运行均为 FULL ON + native RViz；没有OFF/A-B，没有调权重、场景或安全阈值。

### 4.1 最终 Run3 健康性

```text
TERMINAL_HOLD_COUNT:                         2
MOVING_SUCCESSOR_STARVATION_COUNT:           2 (由2个end-before-next episode计)
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 2
UNVALIDATED_EXECUTED_SAMPLES:                193
COLLISION_SAMPLES:                           0
SWARM_CLEARANCE_VIOLATION_SAMPLES:           0
MIN_STATIC_CLEARANCE:                        0.124440 m
MIN_DYNAMIC_CLEARANCE:                       0.793298 m
MIN_SWARM_DISTANCE (elliptical):             0.599195 m
```

Run3 的两次 terminal hold 后均由既有 post-deadline recovery恢复，但这不满足零hold/零unvalidated的健康门禁。不能用0 collision掩盖transaction churn与validated coverage失败。

### 4.2 最终 Run3 可见性

```text
CAMERA_TIME:            218.275 camera·s
MEAN_VISIBLE:           2.720700
K2:                     0.983375
ALL3:                   0.737323
NONE:                   0
LONGEST_K2_LOSS:        1.165928 s
LONGEST_BLACKOUT:       0 s
ACTUAL_Q_DIR_MEAN:      0.843880
ENCIRCLEMENT_RATIO:     0.223692
```

相对 Feedback57 Run9：camera-time `-11.243` camera·s，mean-visible `-0.1425`，K2 `-0.00665`，All3 `-0.13586`，Q_dir和strict encirclement也更低。该结果受两次hold和transaction入口abort污染，不是干净的visibility算法对照。

## 5. 最终代码级失败点

最终第一次 transaction 断点发生在 `MultiUavTopologyCoordinator::evaluateSnapshot()` 与 `attemptTeamOptimization()` 的 predeclared alignment gate：

```cpp
abs(bundle.target_prediction_epoch - common_activation) <= 1e-6
abs(bundle.dynamic_prediction_epoch - common_activation) <= 1e-6
```

而 `EGOPlannerManager::stageCapturedTopologyCoordination()` 发布：

```cpp
target_prediction_epoch  = current->planning_epoch;
dynamic_prediction_epoch = local_activation_time_;
earliest_activation      = local_activation_time_;
```

`current->planning_epoch` 是target prediction snapshot/source epoch，不是执行activation。两者本应通过“相同snapshot authority + 在common activation处一致评价”的合同关联，不能被要求数值等同。Run3中甚至出现三个transaction ID及activation完全相同，仍被该隐藏子条件判为 `PREDECLARED_TRANSACTION_NOT_ALIGNED`。

因此真实链为：

```text
three local reserves ready on a common activation
→ coordinator sees unified bundles
→ incorrect target_epoch == activation predicate fails
→ fail-closed terminal negative to all planners
→ local fallback / next transaction
→ repeated abort churn
→ optimizer never reached
→ no proposal / no fused execution
```

Run2 的“不同slot后一直等待”已通过 terminal negative修复；Run3证明abort能释放旧slot，但未解决如何由三机共同发布同一个target snapshot authority。后续最小修复应是：显式携带并比较真正共享的target snapshot identity/epoch，分别验证dynamic epoch与activation；同时增加一个fixture，使用真实producer字段语义而非把所有epoch人为设成activation。用户要求停止后，本轮未实施该修复。

## 6. Transaction统计与失败分类

最终 Run3：

```text
TEAM_TRANSACTION_COUNT:                   153 unique transaction IDs
TRANSACTION_FINAL_LOCAL_COUNT:            0
TRANSACTION_FINAL_JOINT_COUNT:            0
TRANSACTION_ABORT_COUNT:                  151 unique transaction IDs
ONE_TRANSACTION_MULTIPLE_COMMIT_VIOLATION_COUNT: 0
BOUNDARY_CHANGED_INSIDE_VALID_TRANSACTION_COUNT: 0

JOINT_SUCCESS_COUNT:                      0
JOINT_SELECTED_COUNT:                     0
JOINT_PROPOSAL_COUNT:                     0
JOINT_3ACK_COUNT:                         0
JOINT_ACTIVATION_COUNT:                   0
JOINT_SUCCESS_TO_ACTIVATION_RATE:         UNDEFINED (optimizer never entered)

TRAJECTORY_BOUNDARY_MISMATCH_COUNT:       0 in Run3, but vacuous: no proposal
PREDECLARED_TRANSACTION_NOT_ALIGNED:      695 repeated observations
```

```text
EARLY_JOINT_ATTEMPT_FAILURE_BREAKDOWN (Run3):
  common activation / snapshot identity invalid: 151 aborted transaction IDs
  transaction aborted:                            151
  no constructible tuple:                         NOT_REACHED
  local-SFC infeasible:                           NOT_REACHED
  dynamics/static/dynamic/swarm fail:             NOT_REACHED
  optimizer convergence fail:                     NOT_REACHED
  wall-budget exhausted inside optimizer:         NOT_REACHED
  other optimizer failure:                        NOT_REACHED
```

Run3的 mismatch=0不能用来宣称Feedback57 race已完成真实消除；它只说明更早的context gate阻止了所有proposal。运行级 conversion 无法评价。

performance 的 transaction preparation/local-ready/joint/decision/proposal→ACK/total p50/P95/max同样不可提供：最终Run3没有任何transaction通过入口到decision。只报0或把abort latency冒充成功transaction latency都会失真。

## 7. 要求字段

```text
BASELINE: feedback_57

ARCHITECTURE_BEFORE:
  local first-safe may commit while early joint works on an older future
  boundary -> later joint takeover -> boundary race

ARCHITECTURE_AFTER:
  same transaction -> same generation/common activation/boundary
  -> local reserve + joint quality -> one final decision -> one commit

UNIFIED_TEAM_PLANNING_TRANSACTION_IMPLEMENTED: PARTIAL
LOCAL_RESERVE_IS_CANDIDATE_NOT_SEPARATE_EXECUTION_AUTHORITY: YES (eligible transaction contract)
EARLY_JOINT_IS_TEAM_QUALITY_STAGE: YES (contract/source)
ONE_TRANSACTION_ONE_COMMIT: CONTRACT PASS; FINAL RUN NOT EXERCISED

FEEDBACK57_BOUNDARY_RACE_REPRODUCED: YES
FEEDBACK57_BOUNDARY_RACE_ELIMINATED: DETERMINISTIC YES; PRODUCTION NOT PROVEN
TRAJECTORY_BOUNDARY_MISMATCH_COUNT: 0 in final Run3 (vacuous; proposals=0)

TEAM_TRANSACTION_COUNT: 153
TRANSACTION_FINAL_LOCAL_COUNT: 0
TRANSACTION_FINAL_JOINT_COUNT: 0
TRANSACTION_ABORT_COUNT: 151

JOINT_SUCCESS_COUNT: 0
JOINT_SELECTED_COUNT: 0
JOINT_ACTIVATION_COUNT: 0
JOINT_SUCCESS_TO_ACTIVATION_RATE: UNDEFINED

EARLY_JOINT_ATTEMPT_FAILURE_BREAKDOWN:
  PREDECLARED_TRANSACTION_NOT_ALIGNED before optimizer: 151 aborted IDs

JOINT_FAILURE_BLOCKS_LOCAL_COVERAGE: NOT_CLOSED (Run3 hold/end-before-next = 2/2)

TERMINAL_HOLD_COUNT: 2
STARVATION_COUNT: 2
END_BEFORE_NEXT_COUNT: 2
UNVALIDATED_COUNT: 193
COLLISION_COUNT: 0
SWARM_VIOLATION_COUNT: 0

CAMERA_TIME: 218.275 camera.s
MEAN_VISIBLE: 2.720700
K2: 0.983375
ALL3: 0.737323
Q_DIR: 0.843880
ENCIRCLEMENT: 0.223692
VISIBILITY_VS_FEEDBACK57: WORSE (transaction-abort/hold-confounded)

FEEDBACK56_RECOVERY_GUIDE_FIX_PRESERVED: YES
TIME_BUDGET_CLOSED_LOOP_PRESERVED: YES
POST_DEADLINE_RECOVERY_PRESERVED: YES
EARLY_JOINT_PRIMARY_PRESERVED: YES, but final runtime entry rejected

HANDOFF_LEASE_ADDED: NO
TAKEOVER_RESERVATION_ADDED: NO
LOCAL_PLANNER_FROZEN_FOR_JOINT: NO
POST_JOINT_REBASE_ADDED: NO

NEW_GLOBAL_ASTAR_ADDED: NO
THREE_UAV_JOINT_ASTAR_ADDED: NO
DYNAMIC_SFC_ADDED: NO
SAFETY_THRESHOLD_RELAXED: NO
DYNAMIC_CLEARANCE_1P1_CHANGED: NO

RAW_ASTAR_EXECUTED: NO
JOINT_SEED_DIRECTLY_EXECUTED: NO
FINAL_PREFLIGHT_BYPASSED: NO
PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO
OLD_SUFFIX_EXTENDED: NO
STRICT_25_170_REINTRODUCED_AS_HARD: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

## 8. 清理与最终说明

用户要求停止后未再修改production或启动仿真。最终检查无ROS、roscore、rosmaster、roslaunch、RViz、Gazebo或recorder残留。

已删除本轮三次运行中可重建、重复的 `ros_logs/` 和约705 MiB `ros_ws/build` cache；保留三轮 `launcher.log`、trajectory/visibility CSV、run metadata、确定性race/contracts源码、production diff和本报告。被删除内容不可直接恢复，但均为可重建日志/build产物。

Local 与 Joint 在目标源码结构中不再存在“先local执行、后joint接管”的执行权交接；最终选择应在 `multi_uav_topology_coordinator.cpp` 的 unified transaction decision 中完成，并由统一 proposal→3ACK→commit 只提交一次。然而最终Run3在该decision之前被错误的target-epoch alignment gate全部abort，因此这一“一次选择、一次commit”尚未取得真实运行证明。

```text
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_58.md
```
