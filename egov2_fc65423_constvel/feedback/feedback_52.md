# Feedback 052 — validated moving coverage 时间预算闭环与 Scenario A FULL ON 验证

日期：2026-09-12  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
承接基线：`feedback_51.md`

## 0. 结论

本轮把 predecessor 的有限 validated moving suffix、successor 规划 deadline、candidate-ready activation、P/V/A 重锚、current-revision final preflight 和新 validated coverage 串成了同一时间合同。Feedback51 的两类断链都得到针对性修复：candidate 不再沿用 plan-start 的过期 activation；低覆盖期 NOMINAL 失败后不再提前跳过 SIDE supply，也不会让 team/joint enhancement 阻塞已形成的安全 local successor。

唯一一次 Scenario A FULL ON + native RViz 运行正常完成路线，三机全程没有 terminal hold、trajectory-end gap、runtime starvation 或 unvalidated execution。因第一次运行已经没有复现 successor-coverage 系统失效，按要求没有启动第二次仿真。

```text
TIME_BUDGET_CLOSED_LOOP_IMPLEMENTED: YES
SUCCESS_TOO_LATE_FIXED: YES
SUCCESSOR_SUPPLY_DEADLINE_AWARE: YES
CANDIDATE_READY_REANCHOR_ACTIVE: YES
METADATA_RECOMPUTED_AFTER_REANCHOR: YES
TEAM_ENHANCEMENT_CAN_BLOCK_LOCAL_COVERAGE: NO
OLD_STARVATION_ACCOUNTING_FIXED: YES

FULL_ON_RUN_COUNT: 1
SYSTEM_FAILURE: NO
```

## 1. 两个真实根因

### 1.1 SUCCESS TOO LATE

旧链把 `plan_start + estimated planning budget + activation margin` 当作候选 activation，并在 candidate ready 后继续沿用。实际求解耗时与估计不一致时，这个假设已经失效。更严重的是，旧 `checkActiveHandoff()` 先对 predecessor 做 terminal-clamped sample，再判断有限 suffix，因而能先打印一次误导性的 accepted，随后才被 `validateActivePrefixUntil()` 以 `PREDECESSOR_PREFIX_MOVING_SUFFIX_EXHAUSTED` 拒绝。

当前语义是：candidate ready 时以真实 `now + local_activation_margin` 重新计算 activation；先检查它严格早于 validated end，再从 predecessor 的该时刻采样真实 P/V/A，重建 MINCO head。任何重锚都会产生新 execution revision，并重新执行完整 hard preflight 和 metadata 计算。

### 1.2 SUPPLY NOT FORMED

旧低覆盖分支会因为 `execution_reserve_limited_` 直接跳过 `optimize_side`。这恰好在 NOMINAL 连续失败、最需要另一条安全 observation topology 时切断了 SIDE supply。optional refinement、重复 alternative 和 team enhancement 也没有共同服从 predecessor 的真实有限 deadline。

当前低覆盖策略不伪造 candidate，也不降低任何 safety gate：

```text
NOMINAL fully executable
  → 立即采用第一条完整安全 local successor

NOMINAL failed
  → 在 deadline 内尝试优先 SIDE
  → 若仍失败且尚有时间，再尝试另一 SIDE
  → 第一条通过统一 final hard preflight 的 moving successor 即停止 optional search
```

A*/Local-SFC、MINCO/SCP、current-revision final preflight 都检查同一 wall deadline；optimizer 内层沿用已有 cancellation/checkpoint。local successor commit 后，低覆盖期的 team enhancement 延后，不等待 proposal/ACK/commit 消耗 predecessor suffix。

## 2. 统一时间合同

### 2.1 三个时间量

```text
VALIDATED_END_SOURCE:
  min(active.start_time + active.duration,
      freshly revalidated execution_safe_until)

ACTIVATION_EARLIEST_SOURCE:
  candidate_ready_ros_time + local_activation_margin
  当前配置 local_activation_margin = 0.100 s

PLANNING_DEADLINE_FORMULA:
  t_planning_deadline = t_validated_end - local_activation_margin
```

每个 planning batch 都发布：

```text
validated_coverage_remaining = t_validated_end - now
planning_budget_remaining    = t_planning_deadline - now
```

wall-clock optimizer deadline取历史 pipeline estimate与真实 planning-budget remaining 的较小者，绝不把 deadline 延伸到 predecessor validated end 以后。

### 2.2 candidate-ready 重锚与重新验证

`prepareLocalHandoff()` 当前先做有限 end 检查，再采样 predecessor。满足 `t_activation_new < t_validated_end` 时，用实际：

```text
P_predecessor(t_activation_new)
V_predecessor(t_activation_new)
A_predecessor(t_activation_new)
```

重建 candidate head 并重新生成 polynomial。之后重新计算/验证：

- dynamics；
- static、dynamic、swarm；
- Local-SFC；
- handoff continuity；
- role/local geometry；
- visibility、binary camera-time 与任务 metadata；
- current-revision final execution preflight。

只有该最终 revision 可以提交。`checkActiveHandoff()` 也改为在任何 terminal-clamped sampling 之前拒绝 activation 越过 validated end。

### 2.3 starvation 语义

production 现在能区分：

- `VALIDATED_COVERAGE_LOW`：已进入优先第一条安全 local successor 的窗口；
- `VALIDATED_COVERAGE_DEADLINE_MISSED`：当前已经没有可接管 activation；
- `EXECUTABLE_SUCCESSOR_READY_TOO_LATE`：candidate ready，但已无法接上 predecessor；
- `NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE`：该 batch 到 deadline 仍未完成 handoff；
- `SUCCESSOR_REANCHORED_TO_LATEST_ACTIVATION`：按 ready-time 重建 head；
- `VALIDATED_COVERAGE_HANDOFF_SUCCESS`：新安全 trajectory 已形成新的 validated coverage。

`MOVING_SUCCESSOR_STARVATION` 现在由“能否在 predecessor validated end 前实际接管”触发，不再由“planner 是否仍在运行/是否有中间 candidate”代替。Feedback51 已修正的 right-censored terminal-tail 离线计数继续保留。

## 3. 修改文件与边界

production：

- `plan_manage/include/plan_manage/local_execution_contract.h`
  - 增加 `ValidatedMovingCoverageWindow` 与 deadline-aware alternative 合同。
- `plan_manage/include/plan_manage/planner_manager.h`
  - 保存 validated end、planning deadline、coverage/budget remaining、batch handoff 状态和 execution revision/target context。
- `plan_manage/src/planner_manager.cpp`
  - 统一 coverage deadline；candidate-ready activation/PVA 重锚；current-revision 全量重验证；低覆盖 SIDE supply；A*/Local-SFC deadline；新 telemetry。
- `plan_manage/src/ego_replan_fsm.cpp`
  - 已安全 commit 的 local successor 在低覆盖期优先发布；team enhancement 延后。

tests：

- `plan_manage/test/local_execution_contract_test.cpp`
  - Case A、Case B、genuinely-no-safe-solution 纯合同。
- `plan_manage/test/continuous_motion_contract.h`
  - 真实 manager/MINCO fixture：ready-time 重锚、exact predecessor P/V/A、真正过期 fail-closed。

仿真结束后只修正了一项 telemetry 顺序：`VALIDATED_COVERAGE_HANDOFF_SUCCESS` 原在 `setLocalTraj()` 自增 ID 前打印，导致日志中的 successor_id 与 predecessor_id 相同；现在改为自增后打印真实新 ID。此改动不改变规划、trajectory、安全或执行结果，已重新 build 并复跑定向合同，没有为它启动第二次仿真。

以下均未修改：safety thresholds、25°/170°、Q_dir、shared-risk-support、LEFT/RIGHT 语义和 offsets、J_vis/All3 排序、controller、planner brake、stop/hold fallback。

## 4. Build 与测试

构建：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
6 requested/dependency packages: PASS
failed packages: 0
```

17 个相关可执行测试全部 PASS：

```text
fresh_moving_initializer_contract_test
local_execution_contract_test
recovery_probe_production_test
trajectory_lifecycle_contract_test
visibility_topology_production_test
adaptive_viewpoint_generator_contract_test
cooperative_viewpoint_contract_test
encirclement_geometry_contract_test
multiview_contract_test
persistent_recovery_target_contract_test
static_los_wall_contract_test
team_solution_commit_contract_test
team_visibility_optimizer_contract_test
topology_coordinator_contract_test
elastic_visibility_contract_test
time_only_feasibility_contract_test
time_only_swarm_temporal_contract_test
```

其中 analytic/finite-difference P、tau、piece boundary、K2、blackout、K2-continuity、J_acc、joint yaw 与 deep-risk non-decaying gradient contracts 均通过。新增定向输出：

```text
SUCCESS_TOO_LATE_READY_TIME_REANCHOR_CONTRACT=PASS
SUPPLY_DEADLINE_FIRST_SAFE_CONTRACT=PASS
GENUINELY_NO_SAFE_SOLUTION_REMAINS_FAILURE=PASS
```

telemetry ID 顺序修正后再次执行 `ego_planner` build，以及 local execution、trajectory lifecycle、recovery production 三项合同，均 PASS。`git diff --check` 与分析脚本 compile 通过。

## 5. 唯一一次 Scenario A FULL ON

证据：

```text
run    = validated_coverage_closed_loop_20260912/run1
audit  = validated_coverage_closed_loop_20260912/analysis/run1/run1_metrics.json
scene  = long_cylinder_forest_visibility_stress.json
SHA256 = 430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
mode   = FULL ON + native RViz
route  = normal completion, 80.205 s
```

同一运行没有调参、没有 OFF/A-B，也没有第二次仿真。

### 5.1 validated coverage 与连续性

```text
VALIDATED_MOVING_COVERAGE samples        = 921
minimum validated coverage remaining    = 0.554726 s
P05 / median coverage remaining          = 0.977796 / 3.377732 s
VALIDATED_COVERAGE_LOW                   = 97 (UAV0/1/2 = 45/27/25)
FIRST_EXECUTABLE_LOCAL_SELECTION         = 41 (23/7/11)
TEAM_ENHANCEMENT_DEFERRED_LOCAL_COVERAGE = 69 (19/29/21)
SUCCESSOR_REANCHORED_TO_LATEST_ACTIVATION= 2911
CANDIDATE_CURRENT_REVISION_PREFLIGHT_PASS= 1940
CANDIDATE_METADATA_RECOMPUTED_AFTER_REANCHOR = 805
VALIDATED_COVERAGE_HANDOFF_SUCCESS       = 835 (276/276/283)

VALIDATED_COVERAGE_DEADLINE_MISSED       = 0
EXECUTABLE_SUCCESSOR_READY_TOO_LATE       = 0
NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE  = 0
PREDECESSOR_VALIDATED_END_EXCEEDED       = 0
PREDECESSOR_PREFIX_MOVING_SUFFIX_EXHAUSTED = 0
MOVING_SUCCESSOR_STARVATION_COUNT        = 0
TERMINAL_HOLD_COUNT                      = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT = 0
UNVALIDATED_EXECUTED_SAMPLES             = 0
```

规划 batch latency：921 次，mean 35.546 ms、median 7.942 ms、P95 162.136 ms、max 827.188 ms。即便存在长尾，最小 observed validated coverage 仍覆盖真实 0.1 s activation margin，并成功形成下一条 validated trajectory。

三机 path length 为 88.042 / 72.944 / 88.531 m；最长低速 episode 仅 0.368 s。任务末端 UAV2 虽处于既有 `PERSISTENCE_FALLBACK` source，但速度约 1.18 m/s、`safety_validated=True`，没有 terminal hold 或 unvalidated tail。

### 5.2 team/joint 与 topology

```text
NOMINAL / LEFT / RIGHT actual activation = 708 / 32 / 31
TEAM_VIS_OPT_ATTEMPT / SUCCESS            = 109 / 11
3-ACK commit                              = 10
3-UAV adoption                            = 10
complete optimizer→proposal→3ACK→commit→3adoption→optimized-yaw chains = 10
OPTIMIZED_YAW_EXECUTED complete IDs       = 10
```

这说明本地 coverage 优先没有关闭 FULL ON team path：正常窗口仍完成 10 条完整 joint P/T/yaw 执行链；只在 coverage 紧张时把 enhancement 延后。

### 5.3 可见性与合围

```text
ACCUMULATED_CAMERA_VISIBLE_TIME = 190.875964 camera-s
MEAN_VISIBLE                    = 2.380726
K2                              = 0.961141
ALL3                            = 0.420005
NONE                            = 0.000420
LONGEST_K2_LOSS                 = 2.068052 s
LONGEST_BLACKOUT                = 0.033673 s
ACTUAL_Q_DIR_MEAN               = 0.966712
HIGH_QUALITY_DIRECTIONAL_RATIO  = 0.944537
EXECUTED_ENCIRCLEMENT_RATIO     = 0.779889
SAME_SEMICIRCLE_RATIO           = 0.192181
```

该运行的连续性已健康，因此可见性数字有效；但它仍低于 Feedback49 好运行的 camera-time/All3 均值（约 227.48 camera-s / 0.853）。本轮只修 lifecycle coverage，没有修改可见性目标或 All3 排序，因此不能把一次 liveness PASS 写成 All3 已完成优化。

### 5.4 安全

```text
COLLISION_SAMPLES                 = 0
SWARM_CLEARANCE_VIOLATION_SAMPLES = 0
MIN_STATIC_CLEARANCE              = 0.217379 m
MIN_DYNAMIC_CLEARANCE             = 0.814345 m
MIN_SWARM_DISTANCE                = 0.966994 m
EMERGENCY_STOP / HEARTBEAT_TIMEOUT= 0 / 0
```

没有观察到 safety 或连续性回归。

## 6. 最终回答

```text
TIME_BUDGET_CLOSED_LOOP_IMPLEMENTED: YES

VALIDATED_END_SOURCE: min(active nominal end, freshly revalidated execution_safe_until)
ACTIVATION_EARLIEST_SOURCE: candidate ready ROS time + 0.100 s local handoff margin
PLANNING_DEADLINE_FORMULA: t_validated_end - required_activation_margin

SUCCESS_TOO_LATE_FIXED: YES
SUCCESSOR_SUPPLY_DEADLINE_AWARE: YES
CANDIDATE_READY_REANCHOR_ACTIVE: YES
METADATA_RECOMPUTED_AFTER_REANCHOR: YES
TEAM_ENHANCEMENT_CAN_BLOCK_LOCAL_COVERAGE: NO

OLD_STARVATION_ACCOUNTING_FIXED: YES

TERMINAL_HOLD_COUNT: 0
VALIDATED_COVERAGE_DEADLINE_MISSED: 0
EXECUTABLE_SUCCESSOR_READY_TOO_LATE: 0
NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE: 0
UNVALIDATED_EXECUTED_SAMPLES: 0

ALL3: 0.420005
K2: 0.961141
CAMERA_TIME: 190.875964 camera-s
ACTUAL_Q_DIR_MEAN: 0.966712
EXECUTED_ENCIRCLEMENT_RATIO: 0.779889

COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0

PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

