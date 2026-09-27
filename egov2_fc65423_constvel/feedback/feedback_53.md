# Feedback 053 — validated coverage 两阶段供给/质量策略与单次 Scenario A FULL ON 验证

日期：2026-09-13  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_52.md`  
场景：`long_cylinder_forest_visibility_stress.json`  

## 0. 结论

本轮确认了 Feedback52 中的两个提前终止点：系统在 coverage 紧张时一旦找到首个安全本地候选，就会跳过尚未生成/尚未 current-revision 检查的其余 NOMINAL/LEFT/RIGHT；本地 successor commit 后，又会仅因 `executionReserveLimited()` 无条件推迟异步 team enhancement。它们发生时通常仍有足够的真实 deadline 余量，并非 deadline 已耗尽。

当前实现改为两阶段：先保留第一个通过统一 hard preflight 的安全候选作为 validated-moving-coverage reserve，再只在原有真实 planning deadline 与 optimizer checkpoint 允许时继续本地拓扑和异步 team quality 搜索；质量失败或 deadline 到达时直接提交已保留的安全候选。没有新增经验时间阈值。

唯一一次 FULL ON 运行保持了 Feedback52 的连续性结果，同时显著恢复可见性：All3 从 `0.420005` 提高到 `0.857320`，camera-time 从 `190.875964` 提高到 `228.612786 camera-s`，K2 从 `0.961141` 提高到 `0.994232`。运行中有 20 个首个安全候选之后的更优候选完成实际 activation，其中 5 个有明确的“同 K2、更高 mean-visible/All3”日志证据。

但总体结论仍是 `PARTIAL`：`ACTUAL_Q_DIR_MEAN` 仅小幅下降（`0.966712→0.955469`），而 `EXECUTED_ENCIRCLEMENT_RATIO` 从 `0.779889` 明显降到 `0.456784`，`SAME_SEMICIRCLE_RATIO` 从 `0.192181` 升到 `0.464905`。因此本轮证明了“不因 first-safe 过早停止质量搜索且不破坏 rolling coverage”，但不能宣称 All3 改善同时完整保持了 25°/170° 合围表现。

```text
FIRST_SAFE_EARLY_TERMINATION_CONFIRMED: YES
TEAM_DEFER_SUPPRESSED_QUALITY_IMPROVEMENT: YES
TWO_STAGE_COVERAGE_QUALITY_POLICY_IMPLEMENTED: YES

ALL3_BEFORE: 0.420005
ALL3_AFTER: 0.857320
CAMERA_TIME_BEFORE: 190.875964 camera-s
CAMERA_TIME_AFTER: 228.612786 camera-s
K2_BEFORE: 0.961141
K2_AFTER: 0.994232

TERMINAL_HOLD_COUNT: 0
STARVATION_COUNT: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
```

## 1. Feedback52 的提前终止证据

### 1.1 本地 first-safe 提前终止

旧逻辑有三层提前退出：

1. reserve-limited 时 NOMINAL 一旦 executable，LEFT/RIGHT 都不生成；
2. NOMINAL 不可执行而第一侧 executable 时，另一侧不再生成；
3. `finalizeCapturedCandidates()` 在第一条 current-revision safe candidate 后停止检查后续候选。

对 Feedback52 原始日志重建：

```text
FIRST_EXECUTABLE_LOCAL_SELECTION                    = 41
其中真正已有 safe candidate                        = 14
其中 NOMINAL safe 后 LEFT/RIGHT 均跳过              = 10
其中 first SIDE safe 后 opposite SIDE 跳过          = 1
其中其实没有 safe candidate                         = 27

首个 safe 时剩余 planning deadline：
min / median / max = 0.444076 / 0.848381 / 1.134901 s
positive remaining = 14 / 14
```

旧 telemetry 名称 `FIRST_EXECUTABLE_LOCAL_SELECTION` 混入了 27 个“没有安全候选”的 batch；本轮没有把这 27 个误算为 first-safe 证据。直接证据是其余 14 个事件中 11 个确实因 first-safe 跳过后续 observation topology，且 14/14 均仍有正 deadline 余量。

进一步按原事件的 planning deadline 检查后续日志：10/14 在 deadline 内后来出现多候选 batch，2/14 在 deadline 内出现同 K2 且 mean-visible/All3 更高的候选。这证明提前退出确实删除过可观测的质量机会，而不只是理论上的候选可能性。

### 1.2 team enhancement 被无条件 defer

Feedback52 有 71 次 `TEAM_ENHANCEMENT_DEFERRED_LOCAL_COVERAGE`。其中 69 次能可靠绑定 validated-coverage window，68 次在扣除既有 50 ms uninterruptible reserve 后仍有正余量，剩余时间中位数约 `0.938 s`。

旧判断只检查 `executionReserveLimited()`，没有在本地安全 successor 已 commit 后再次询问真实 deadline。因此“coverage 紧张”被错误地等同于“任何剩余 team quality 工作都会阻塞本地 coverage”。这会推迟 joint P/T/yaw 的质量提升，即使该工作在本地 commit 之后异步进行。

### 1.3 与实际 All3 loss 的关系

Feedback52 中 `All3:1→0 且 K2=1` 共 13 个事件、累计 `6.640004 s`：

| 第三架失视原因 | 事件数 |
|---|---:|
| FOV | 8 |
| static LOS | 4 |
| FOV + range | 1 |

其中 7/13 在此前 3.5 s 内出现 first-safe 提前终止，13/13 在此前 3.5 s 内出现 team defer。准确结论是：提前终止抑制了可用的质量修正机会，但实际失视仍由 FOV/static LOS 等物理条件产生；它不是所有 All3 loss 的唯一原因。

## 2. 修改逻辑

### 2.1 保留首个安全候选，再按真实 deadline 搜索质量

`local_execution_contract.h` 新增 `CoverageQualityAction`：

```text
尚无安全候选                         -> FIND_SAFE_SUPPLY
已有安全 reserve 且 deadline 允许     -> SEARCH_QUALITY_WITH_RESERVED_SAFE
已有安全 reserve 且 deadline 不允许   -> COMMIT_RESERVED_SAFE
```

`deadlineAwareAlternativeAllowed()` 不再把“已找到 first-safe”本身当作停止条件；唯一时间 authority 仍是 Feedback52 已验证的 wall planning deadline、activation margin 和 optimizer 的 uninterruptible checkpoint。

在 `planner_manager.cpp`：

- NOMINAL/LEFT/RIGHT 仍逐个走原 generation、A*/MINCO/SCP 和安全分类；
- 第一个 executable candidate 只被保留为 reserve，不再自动跳过尚未尝试的 observation topology；
- `finalizeCapturedCandidates()` 对每条候选重新 reanchor，重新计算 current-revision metadata，并通过相同 Local-SFC、handoff、dynamics、static/dynamic/swarm final preflight；
- 后续 quality solve 失败不会删除 reserve；deadline 不允许继续时立即选择 reserve；
- `BETTER_CANDIDATE_ACTIVATED` 只在 executor 实际激活对应 trajectory ID 后发布，不用 selected/committed 冒充执行。

### 2.2 修复 finalization 中 comparator 旁路

审计还发现 Feedback52 的 `finalizeCapturedCandidates()` 虽重算了 current-revision visibility，却最终退回 geometry/own-camera-time 比较，旁路了 Feedback51 已锁定的完整排序。

现改为：

```text
hard safety / executable set unchanged
-> blackout / exact K2 protection
-> equal K2: mean-visible / All3
-> weakest camera / max loss
-> diversity / spread / min-pairwise-angle and geometry as later ties
```

All3 没有成为 hard gate，也不允许用 All3 换取任何 K2 下降。运行日志中的 `LOCAL_SELECTED_LOWER_MEAN_VISIBLE` 为 0。

### 2.3 本地 commit 后的异步 team quality

`ego_replan_fsm.cpp` 仍先完成本地 current-revision safe successor commit。commit 后只要 `optionalRefinementAllowed()` 证明真实 deadline 仍有余量，就允许 `stageCapturedTopologyCoordination()`；team solve/proposal 仍是异步 enhancement，不能撤销或阻塞已提交的本地 coverage。deadline 不足时仍 fail closed defer。

outer hypothesis 循环也只服从相同 deadline，不再仅因 reserve-limited 在第一组后退出。

## 3. 修改文件

Production：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/local_execution_contract.h
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp
```

Tests/analysis：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/test/local_execution_contract_test.cpp
coverage_quality_two_stage_20260913/analyze_two_stage.py
```

没有修改 J_vis/deep-risk、Q_dir、25°/170° 定义、shared-risk-support、SIDE offsets/A*、Stage2/3A/3B 主公式、controller、brake/stop 或任何 static/dynamic/swarm 安全阈值。

## 4. Build 与测试

构建：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

6 个请求/依赖包全部成功，0 failure。

17 个相关 unit/gradient/contract/production-chain 测试可执行文件全部 PASS，包括：

```text
fresh_moving_initializer_contract_test
local_execution_contract_test
recovery_probe_production_test
trajectory_lifecycle_contract_test
visibility_topology_production_test
adaptive_viewpoint_generator_test
cooperative_viewpoint_test
encirclement_geometry_test
multiview_test
persistent_recovery_target_test
static_los_wall_test
team_solution_commit_test
team_visibility_optimizer_test
topology_coordinator_test
elastic_visibility_contract_test
time_only_feasibility_test
time_only_swarm_temporal_test
```

其中 team optimizer 的 analytic/finite-difference P、tau、piece boundary、K2、blackout、K2-continuity、J_acc、joint yaw 与既有 deep-risk gradient contracts 保持通过。新增本地合同覆盖：first-safe reserve 后继续质量搜索、deadline 到达提交 reserve、quality solve failure 保留 reserve、late team enhancement 不撤销本地 reserve。

```text
FIRST_SAFE_RESERVED_QUALITY_SEARCH_CONTRACT = PASS
FIRST_SAFE_DEADLINE_COMMIT_CONTRACT = PASS
QUALITY_FAILURE_PRESERVES_RESERVED_SAFE_CONTRACT = PASS
LATE_TEAM_ENHANCEMENT_PRESERVES_LOCAL_RESERVE_CONTRACT = PASS
git diff --check = PASS
analysis py_compile = PASS
```

## 5. 唯一一次 Scenario A FULL ON

证据目录：

```text
coverage_quality_two_stage_20260913/run1
coverage_quality_two_stage_20260913/analysis/run1/run1_metrics.json
coverage_quality_two_stage_20260913/analysis/two_stage_audit.json
```

```text
RUN_COUNT = 1
mode = FULL_ON
scene SHA256 = 430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
target route = normally completed
route duration = 80.204541 s
exit code = 0
```

Stage2、3A、3B、visibility candidate ranking、K-of-N、team visibility optimizer、joint P/T、joint yaw、topology coordination、cooperative viewpoint、encirclement、risk-triggered candidates、SIDE hard-corridor SCP、predicted-attitude FOV、Elastic tracking 与既有 deep-risk J_vis 均保持 ON。没有运行 OFF 或 A/B，没有第二次仿真；仿真后 production 源码未再改变。

### 5.1 可见性

| 指标 | Feedback52 | 本轮 | Feedback49 Run1/2/5 均值 | 旧 OFF |
|---|---:|---:|---:|---:|
| camera-time (camera-s) | 190.875964 | **228.612786** | 227.483786 | 227.317129 |
| mean-visible | 2.380726 | **2.851552** | 2.837703 | 2.905279 |
| K2 | 0.961141 | **0.994232** | 0.985196 | 0.982248 |
| All3 | 0.420005 | **0.857320** | 0.852768 | 0.923316 |
| NONE | 0.000420 | **0** | — | — |
| longest K2 loss (s) | 2.068052 | **0.265703** | 0.611448 | 0.877114 |
| longest blackout (s) | 0.033673 | **0** | — | 0.022395 |

All3 已超过 Feedback49 三次好运行均值，但仍低于旧 OFF；camera-time 和 K2 均超过两个参考，mean-visible 略低于旧 OFF。因此这里的 `ALL3_IMPROVED=YES` 是相对 Feedback52 和健康 ON 均值，不应扩张为“超过旧 OFF 的 All3”。

失视累计：

| 原因 | Feedback52 | 本轮 |
|---|---:|---:|
| static LOS loss | 33.703865 s | **6.052932 s** |
| dynamic LOS loss | 4.819175 s | **1.357907 s** |
| HFOV loss | 1.566846 s | **1.103293 s** |
| VFOV loss | 1.506640 s | 3.619023 s |
| range loss | — | 0.333685 s |

本轮仍有 24 个 `All3:1→0 且 K2=1` 事件，累计 `9.458432 s`，原因为 FOV 14、static LOS 8、FOV+range 1、dynamic LOS 1。事件数/累计时长高于 Feedback52，但长时间 All3 丢失被消除，所以完整路线 All3 比例显著更高；不能用 onset 计数替代时间比例。

### 5.2 两阶段机制的真实执行证据

```text
FIRST_SAFE_SUPPLY_HELD                         = 65
FIRST_SAFE_RESERVED                            = 151
QUALITY_SEARCH_AFTER_FIRST_SAFE                = 237
BETTER_CANDIDATE_FOUND_AFTER_FIRST_SAFE        = 33
BETTER_CANDIDATE_SELECTED_AFTER_FIRST_SAFE     = 20
BETTER_CANDIDATE_ACTIVATED                     = 20
QUALITY_SEARCH_ABORTED_FOR_DEADLINE             = 1
TEAM_ENHANCEMENT_AFTER_LOCAL_RESERVE           = 150
TEAM_ENHANCEMENT_DEFERRED_LOCAL_COVERAGE        = 4 (全部 reason=DEADLINE)
VALIDATED_COVERAGE_DEADLINE_MISSED              = 0
NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE         = 0
LOCAL_SELECTED_LOWER_MEAN_VISIBLE_COUNT         = 0
```

20 个 activated upgrade 中，5 个能由完整 telemetry 明确证明在同 K2 下提高 mean-visible/All3；其余主要是 visibility 等价后的后级 tie-break，另有一条 found 日志被并发输出破坏但 activation identity 完整。该证据证明 quality work 实际执行，而不只是“生成过候选”。

当前 first-safe reserve 的剩余 planning deadline：151 个样本，min/median/max 为 `0.031455/0.977514/1.630716 s`。只有 1 次本地 quality search 和 4 次 team enhancement 因真实 deadline 停止；没有新增固定等待时间。

### 5.3 多方向、合围与 team 闭环

| 指标 | Feedback52 | 本轮 | Feedback49 好运行均值 | 判定 |
|---|---:|---:|---:|---|
| ACTUAL_Q_DIR_MEAN | 0.966712 | 0.955469 | 0.972633 | 小幅下降，基本保持 |
| HIGH_QUALITY_DIRECTIONAL_RATIO | 0.944537 | 0.906860 | — | 小幅下降 |
| HIGH_QUALITY_MULTI3_RATIO | — | 0.787073 | 0.807588 左右 | 接近好运行水平 |
| EXECUTED_ENCIRCLEMENT_RATIO | 0.779889 | **0.456784** | 0.729853 | 未保持 |
| SAME_SEMICIRCLE_RATIO | 0.192181 | **0.464905** | 约0.184 | 明显变差 |

实际 topology activation 从 Feedback52 的 NOMINAL/LEFT/RIGHT `708/32/31` 变为本轮 `649/49/97`。更多 observation SIDE 执行与 All3 提升同时出现，也与合围率下降同时出现；单次运行只能支持这种关联，不能严格证明全部合围下降均由两阶段策略造成。由于本轮禁止第二次运行且不允许再调参，源码在该结果后冻结。

team 路径仍完整工作：

```text
TEAM_VIS_OPT_ATTEMPT / SUCCESS = 82 / 33
proposal -> 3 ACK -> commit -> 3-UAV adoption -> optimized yaw execution = 28 complete chains
joint planning time mean/p50/p95/max = 2.282/2.258/6.965/10.117 ms
OPTIMIZED_YAW_EXECUTED = YES (28 solution IDs)
```

### 5.4 连续性与安全

```text
MOVING_SUCCESSOR_STARVATION_COUNT        = 0
TERMINAL_HOLD_COUNT                      = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION    = 0
UNVALIDATED_EXECUTED_SAMPLES             = 0
COLLISION_SAMPLES                        = 0
SWARM_CLEARANCE_VIOLATION_SAMPLES        = 0
EMERGENCY / HEARTBEAT_TIMEOUT            = 0 / 0

MIN_STATIC_CLEARANCE                     = 0.121771 m
MIN_DYNAMIC_CLEARANCE                    = 0.709592 m
MIN_SWARM_DISTANCE                       = 0.737682 m
```

因此 Feedback52 的 validated-moving-coverage 合同没有回归，且本轮样本中没有观察到安全回归。

## 6. 最终判定与剩余建议

已闭合的部分：

- 首个安全候选被保留而不是丢弃；
- 剩余质量搜索严格受原真实 deadline 约束；
- quality/team 失败不撤销本地安全 reserve；
- current-revision finalization 使用完整 Feedback51 visibility comparator；
- 更优候选从 found、selected 到 actual activation 有 identity 证据；
- 单次 FULL ON 无 hold、starvation、unvalidated execution 或安全样本回归。

未闭合的部分：

- 合围率与 same-semicircle 明显退化，所以“在所有协作几何指标不明显退化的前提下提高 All3”尚未完全满足；
- 当前证据只有一次真实运行，不能把可见性提升声明为重复稳定结果；
- 后续应离线比较相同 K2/All3 等级候选的 25°/170° admissible geometry，并在现有 comparator 的后级 tie-break 内恢复合围，不应回退 first-safe 修复，也不应改安全阈值或用 All3 hard gate。该建议本轮未实现，以免改变已经完成的唯一实验版本。

```text
ALL3_IMPROVED_VS_FEEDBACK52: YES
ALL3_ABOVE_FEEDBACK49_GOOD_ON_MEAN: YES
ALL3_ABOVE_OLD_OFF: NO
K2_MAINTAINED: YES
Q_DIR_MAINTAINED: YES (small numerical decrease)
ENCIRCLEMENT_MAINTAINED: NO
CAMERA_TIME_IMPROVED: YES
SAFETY_REGRESSION_OBSERVED: NO
CONTINUITY_REGRESSION_OBSERVED: NO

FULL_MULTI_OBJECTIVE_ACCEPTANCE: PARTIAL
ON_RUNS_THIS_AGENT: 1
OFF_RUNS_THIS_AGENT: 0
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

