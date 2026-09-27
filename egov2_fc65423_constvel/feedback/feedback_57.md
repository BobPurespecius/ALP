# Feedback 057 — Early Joint / Primary Joint Trajectory Planning

日期：2026-09-13  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_56.md`  
场景：`long_cylinder_forest_visibility_stress.json`

## 0. 结论

本轮已把团队联合优化的生产入口从“三架 local executable trajectory 定型后再做 refinement”前移到 pre-final topology seed 层：三架 Planner 在相同 topology hypothesis 下导出 `JOINT_SEED`、A*/guide 与 Local-SFC，Coordinator 建立统一 `TeamPlanningContext`、枚举最多 27 个 N/L/R tuple，再把三架 seed 的 P/T 和现有 yaw 一起交给原 `TeamVisibilityOptimizer`。Joint solution 只有通过 team hard preflight、三机 current-revision ACK 和 common activation 合同后才可执行；seed 本身没有 execution authority。

Local first-safe 路径保留且不等待 joint 成功。Run9 是第一条同时满足以下两个条件的完整运行，因此按要求立即停止：

1. terminal hold、starvation、end-before-next、unvalidated、collision、swarm violation 全为 0；
2. early-primary solution 5、8 均完成 `pre-final seed → early joint P/T/yaw → full preflight → proposal → 3 ACK → commit → 3 UAV activation → optimized yaw execution`。

```text
FINAL_ASSESSMENT = PASS

EARLY_JOINT_PRIMARY_TRAJECTORY_PLANNING_ACTIVE = YES
JOINT_INPUT_IS_PRE_FINAL_LOCAL_SEED = YES
LOCAL_FINAL_WINNERS_ARE_NOT_REQUIRED_BEFORE_JOINT = YES
TEAM_PLANNING_CONTEXT_IMPLEMENTED = YES
TEAM_TOPOLOGY_TUPLE_USED = YES

HEALTHY_FULL_RUN_FOUND = YES
FIRST_HEALTHY_EARLY_JOINT_RUN = Run9
```

相对 Feedback56 healthy Run8，本次 camera-time `+0.617532 camera-s`、All3 `+0.011413`、mean-visible `+0.007307`；K2 从 `0.994127` 降至 `0.990022`，仍维持约 0.99 且无 blackout，但 longest K2 loss 从 `0.400443 s` 增至 `0.599683 s`。Q_dir、Multi2/Multi3 与 strict encirclement 比基线低，因此不能把本次写成所有质量维度全面改善；25°/170°仍按既定设计保持 non-hard。

## 1. 修改前后的代码级结构

### 1.1 修改前

```text
team viewpoint hypothesis
→ UAV0/1/2 各自 N/L/R
→ 各自 A*/Local-SFC/MINCO/SCP/final preflight
→ 三条 local-finalized executable polynomial
→ Coordinator 组合
→ TeamVisibilityOptimizer 做 post-selection refinement
→ proposal / 3ACK / commit
```

原 `TeamVisibilityOptimizer` 已联合优化三架 P/tau，并使用现有 joint yaw stage、team visibility、pairwise swarm gradient、trust region 与 hard validation；问题不是缺少联合数学，而是输入已是 local-finalized polynomial，团队梯度介入太晚。

### 1.2 修改后

```text
common TeamPlanningContext
→ 各机 NOMINAL / LEFT / RIGHT pre-final construction
→ simple SIDE / same-topology A* / Local-SFC
→ JOINT_SEED(P,T,head PVA,guide,SFC,identity)
→ 最多 27 个 team topology tuple cheap ordering
→ existing TeamVisibilityOptimizer primary joint P/tau/yaw
→ team full hard preflight
→ proposal / 3ACK / commit / common activation
```

并行的 liveness 路径仍是：

```text
local first-safe successor
→ current-revision final preflight
→ normal rolling activation
```

early joint 超时、没有 common activation、optimizer/hard preflight/ACK 失败时，只回退到该 reserved local successor；不会产生 pending block，也不会执行 `JOINT_SEED`。

### 1.3 三架第一次真正进入同一个问题的位置

第一次真正进入同一个联合优化问题的位置是 `multi_uav_topology_coordinator.cpp::attemptTeamOptimization(..., early_joint_primary=true)` 调用既有 `TeamVisibilityOptimizer::optimize()` 时：输入的三项 `TeamTrajectoryInput` 来自同一个 `TeamPlanningContext` 下三架 `TopologyCandidate.joint_seed`，同时携带各自 Local-SFC，联合变量和梯度一次覆盖三架 P/tau、pairwise swarm 项及现有 yaw stage。

它与旧代码的本质区别是：旧路径从 `candidate.trajectory` 读取已经 local-finalized、已具 execution authority 的 polynomial；新路径从 `candidate.joint_seed` 读取尚无 execution authority 的 pre-final MINCO initializer，在三条本地轨迹成为团队 quality 决策之前就让团队 visibility/swarm gradient共同塑形。

## 2. TeamPlanningContext 与 identity

新增 `team_planning_context.h`，统一保存/检查：

- team generation、snapshot、target/dynamic prediction epoch；
- static-map revision 与 visibility-model version；
- 三机 planning/execution generation、active trajectory ID；
- earliest activation、validated end、common activation；
- planning horizon 与 joint wall deadline。

`common_activation >= max(earliest_activation_i)`，且 predecessor handoff 时必须仍早于三机 validated end。proposal 前及每机 ACK 时重新检查 generation、trajectory identity、context/model revision、head P/V/A、hard safety与 common activation。任一 UAV 已进入更新 generation 时旧 solution fail closed。

Run9：

```text
TEAM_PLANNING_CONTEXT_CREATED = 79
JOINT_MISSED_COMMON_ACTIVATION_COUNT = 0
```

两条执行链的 context：

| solution | team generation | snapshot | prediction epoch | common activation | horizon |
|---:|---:|---:|---:|---:|---:|
| 5 | 27 | 1789293619.590195 | 1789293619.656874 | 1789293619.830195 | 1.5 s |
| 8 | 52 | 1789293628.753083 | 1789293628.835553 | 1789293628.993083 | 1.5 s |

两次 proposal remaining lead 分别为 235.592/234.893 ms，高于既有 85 ms activation margin。

## 3. JOINT_SEED、tuple 与 primary optimizer

### 3.1 Seed export

`TopologyCandidate.msg` 增加：pre-final MINCO seed、construction status、dynamic evidence、binary visibility trace、Local-SFC planes/active intervals和 start time。Planner 在 N/L/R 的 guide/A*/SFC 已构造、但 local candidate 最终定型之前发布 seed。

Run9：

```text
JOINT_SEED_EXPORTED = 976
  UAV0 / UAV1 / UAV2 = 270 / 334 / 372
JOINT_SEED_DIRECT_EXECUTION_AUTHORITY = 0
```

### 3.2 Tuple construction

Coordinator 为每机收集最多 N/L/R 三类 constructible seed，以相同 hypothesis/generation 构造最多 `3^3=27` 个 tuple。cheap ordering复用 predicted camera time、dynamic evidence、transition burden；继续搜索多少个 tuple由 joint wall budget决定，没有固定 top-K。

```text
TEAM_TOPOLOGY_TUPLE_ENUMERATED snapshots = 39
tuple count p50 / p95 / max = 6 / 27 / 27
TEAM_TOPOLOGY_TUPLE_SCREENED = 80
enumeration→first-screen timestamp proxy p50/p95/max
  = 0.047 / 0.108 / 0.131 ms
```

### 3.3 Optimizer 与 hard constraints

复用原 `TeamVisibilityOptimizer` 的：

- joint P/tau variables、existing yaw stage 与 trust region；
- J_acc、J_K2、J_K2-cont、J_blackout、Q_dir/deviation/yaw prior；
- P/tau Jacobian与pairwise swarm gradient；
- static/dynamic/swarm、v/a/jerk 与 final metric checks。

新增的是从 pre-final seed 进入上述 optimizer，以及把 seed Local-SFC 作为 active-interval joint constraint传入。没有新建另一套 optimizer、global A*或dynamic SFC。

## 4. 开发中发现并修复的明确实现错误

Run8 在第一次 early optimize 中触发 Eigen assertion。调用链为：

```text
TeamVisibilityOptimizer::fixedTimeSampleGradient
→ MinJerkOpt::addPropCtoT
→ DenseCoeffsBase index assertion
```

原因是 common-horizon crop + dynamics retime 可能把 seed变为单 piece，而 `MinJerkOpt` 单-piece closed-form `generate()` 没有初始化 `getGrad2TP()` 所需的 T2–T5/adjoint A。修复为 `ensureJointDifferentiableSeed()`：仅在 one-piece seed 时按原 minimum-jerk polynomial 中点拆成两 piece，并在 21 个样点验证 P/V/A误差分别不超过 `1e-6/1e-5/1e-4`，然后才暴露给 joint SCP。它不改变几何路径或安全阈值。

定向复现修复后：

```text
retime scale = 12.2439
piece_count = 2
single-piece-derived gradient valid = YES, gradient size = 5
```

此前还修正了 retime 后完整 12–18 s seed未重新裁剪到 common horizon的问题；当前 full retime 后再次严格裁到 1.5 s，再验证 dynamics。

## 5. Deterministic contracts 与 build

最终源码构建：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 \
  --no-status --workspace ros_ws

6 requested/dependency packages succeeded; 0 failed
```

18 个相关 C++ binary全部 PASS：

```text
fresh_moving_initializer_contract_test
local_execution_contract_test
recovery_probe_production_test
trajectory_lifecycle_contract_test
feedback53_regression_fixture_test
adaptive_viewpoint_generator_contract_test
cooperative_viewpoint_contract_test
early_joint_primary_contract_test
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

额外结果：

```text
visibility_topology_production_test = 8/8 PASS
trajectory_lifecycle_wiring_test.py = 19/19 PASS
recovery_nonblocking_wiring_test.py = 7/7 PASS
adaptive_execution_contract_test.py = 12/12 PASS
git diff --check = PASS
analysis scripts py_compile = PASS
```

gradient suite覆盖 P、tau、piece boundary、K2、blackout、K2 continuity、J_acc、visibility P/tau chain和pairwise separation；本次输出最大绝对误差约 `5.64e-10`。

合同结论：

```text
EARLY_JOINT_ARCHITECTURE_PROVEN_IN_FIXTURE = YES
TEAM_BENEFIT_REQUIRING_INDIVIDUAL_SACRIFICE = PASS
JOINT_CREATED_TRAJECTORY_NOT_JUST_MICRO_REFINEMENT = PASS
JOINT_RESPECTS_LOCAL_SFC = PASS
JOINT_FAILURE_BLOCKS_LOCAL_COVERAGE = NO
STALE_CONTEXT_AND_NEWER_LOCAL_GENERATION_REJECTED = PASS
```

这里的 fixture authority是组合后的 deterministic production contracts：context/27 tuple/individual sacrifice由 early-joint contract锁定；joint P/tau/yaw的显著变化与team objective改善由现有 optimizer production contract锁定；SIDE→A*→Local-SFC及joint constraint wiring分别由 production topology test与lifecycle wiring锁定。不是用仿真日志倒推合同。

## 6. Scenario A FULL ON Run9

证据：

```text
early_joint_primary_validation_20260913/run9
early_joint_primary_validation_20260913/analysis/run9_metrics.json
early_joint_primary_validation_20260913/analysis/run9_early_joint.json

scene SHA256 = 430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
mode = FULL_ON + native RViz
route = normally completed
route duration = 80.204541 s
exit code = 0
```

### 6.1 Healthy gate

```text
TERMINAL_HOLD_COUNT = 0
MOVING_SUCCESSOR_STARVATION_COUNT = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT = 0
UNVALIDATED_EXECUTED_SAMPLES = 0
COLLISION_SAMPLES = 0
SWARM_CLEARANCE_VIOLATION_SAMPLES = 0

actual activation rate UAV0/UAV1/UAV2
  = 2.2069 / 2.2567 / 2.1320 Hz
terminal recorder identities = 174 / 181 / 184, all safety_validated=True
```

实际 sampled minimum：static `0.171358 m`、dynamic `0.733997 m`、inter-UAV `0.685233 m`。这里 recorder 的 actual-body dynamic metric与 Planner的 1.1 m future moving-prediction admission metric不是同一量；1.1 m hard threshold未改。两条 early solution的 joint final preflight minimum dynamic分别为 26.874/18.258 m。

### 6.2 Visibility 与方向质量

| metric | Feedback56 Run8 | Run9 | delta |
|---|---:|---:|---:|
| camera-time | 228.900393 | 229.517925 | +0.617532 |
| mean-visible | 2.855893 | 2.863200 | +0.007307 |
| K2 | 0.994127 | 0.990022 | -0.004105 |
| All3 | 0.861765 | 0.873178 | +0.011413 |
| NONE | 0 | 0 | 0 |
| longest K2 loss | 0.400443 s | 0.599683 s | +0.199240 s |
| longest blackout | 0 s | 0 s | 0 |

Run9 loss totals：

```text
static LOS = 5.571836 camera-s
dynamic LOS = 1.265765 camera-s
HFOV = 0.033369 camera-s
VFOV = 5.560959 camera-s
range = 0

ACTUAL_Q_DIR_MEAN = 0.918449
HIGH_QUALITY_DIRECTIONAL_RATIO = 0.803838
HIGH_QUALITY_MULTI2_RATIO = 0.793863
HIGH_QUALITY_MULTI3_RATIO = 0.727747
EXECUTED_ENCIRCLEMENT_RATIO = 0.254374
SAME_SEMICIRCLE_RATIO = 0.672711
```

Run9的 camera-time/All3提高且系统健康；K2仍约0.99、无blackout。Q_dir、Multi2/Multi3和strict encirclement未保持Feedback56水平，因此 `VISIBILITY_VS_FEEDBACK56=PARTIAL_IMPROVEMENT`，不是全面优于基线。

### 6.3 Early 与 legacy 分开计数

```text
EARLY_JOINT_ATTEMPT_COUNT = 79
EARLY_JOINT_SUCCESS_COUNT = 11
EARLY_JOINT_PREFLIGHT_PASS_COUNT = 11
EARLY_JOINT_PROPOSAL_COUNT = 11
EARLY_JOINT_3ACK_COMMIT_COUNT = 2
EARLY_JOINT_ACTIVATION_COUNT = 2 team solutions / 6 UAV activations
EARLY_JOINT_COMPLETE_SOLUTION_IDS = 5, 8

LEGACY_POST_SELECTION_REFINEMENT_ATTEMPT/SUCCESS = 24 / 9
LEGACY_COMPLETE_3UAV_CHAINS = 5
TOTAL_COMPLETE_CHAINS = 7
```

另外 9 条 early proposal被各机 current-revision ACK以 `TRAJECTORY_BOUNDARY_MISMATCH` fail closed（UAV0/1/2 = 3/2/4），没有覆盖更新的 local plan。

两条实际 early chain：

| solution | tuple | local baseline camera → early | seed All3 before→after | K2 | P/T/yaw norm | final min swarm/static/dynamic m |
|---:|---|---:|---:|---:|---|---|
| 5 | L/R/R | 4.5→4.5 | 1.000→1.000 | 1→1 | .129564/.029082/.000578 | 1.931/.954/26.874 |
| 8 | L/R/L | 3.8→3.9 | .533333→.600000 | 1→1 | .087790/.017192/.000360 | 2.224/.437/18.258 |

两条均为 nonzero P/T/yaw变化并实际执行 optimized yaw。累计：

```text
P_CHANGE_NORM_SUM = 0.217354
T_CHANGE_NORM_SUM = 0.046275
YAW_CHANGE_NORM_SUM = 0.000939
EARLY_JOINT_CAMERA_TIME_GAIN_SUM = +0.100000 camera-s
EARLY_JOINT_SEED_LEVEL_ALL3_GAIN_SUM = +0.066667
```

### 6.4 Individual sacrifice 与旧结构反事实

Run9两条已执行 early tuple的 `transition_burden=0`，即本次自然运行没有选择偏离三机各自 local-best topology 的 tuple：

```text
EARLY_JOINT_SELECTED_NONLOCAL_BEST_TUPLE_COUNT = 0
TEAM_BENEFIT_REQUIRING_INDIVIDUAL_SACRIFICE_COUNT = 0
```

这不等同于能力缺失；确定性 fixture 已证明一架 individual visibility下降、team value显著上升时可识别为 team-beneficial sacrifice，并拒绝数值噪声伪改善。但不能把 fixture事件冒充Run9自然事件。

对 actual executed early event，源码用三机当时 committed local suffix构造旧 A，并以 early solution作为 B；Run9保留了 camera/K2 before/after：

```text
EARLY_JOINT_VS_OLD_COMBINATION_COUNT = 2
EARLY_JOINT_BETTER_CAMERA_TIME_COUNT = 1
EARLY_JOINT_EQUAL_CAMERA_TIME_COUNT = 1
EARLY_JOINT_WORSE_CAMERA_TIME_COUNT = 0
K2_B - K2_A = 0, 0
```

严格的 `All3_B-All3_A` 不能对两条都从旧 A重建：Run9记录了 committed-local camera/K2/Q_dir，却没有记录 committed-local All3字段。表中的 All3是 seed initializer→optimized的同-context对比；因此不将其伪装成完整旧架构反事实。

```text
EARLY_JOINT_VS_OLD_COMBINATION =
  CAMERA/K2 STRICTLY AVAILABLE; ALL3 STRICT COUNTS UNKNOWN
```

## 7. 性能

Run9可解析 telemetry：

| stage | count | p50 ms | p95 ms | max ms |
|---|---:|---:|---:|---:|
| tuple enumeration→first screen timestamp proxy | 39 | 0.047 | 0.108 | 0.131 |
| joint optimization（完整可解析 solve） | 49 | 5.180 | 11.707 | 14.848 |
| coordinator start→proposal ready | 10 | 4.935 | 6.318 | 6.730 |

实际执行的 solution 5/8：joint solve `3.918/4.732 ms`，full coordinator-to-proposal `4.408/5.107 ms`。

`joint_seed_construction_ms` 已在 bundle message中存在，但 Run9的 `JOINT_SEED_EXPORTED` 文本没有打印该字段，原始 bundle topic也未被 recorder保存，所以 seed construction p50/p95/max为 `UNKNOWN`；不从local total latency伪推。后续只需补telemetry，不需要改变规划语义。

## 8. 运行次数、证据与清理

本任务目录有9个technical launch attempt，其中6个完成约80 s路线（Run1/3/6/7/8/9）；Run2为约3.1 s partial，Run4没有有效recorder数据，Run5为RViz startup timeout。开发阶段运行未因结果差删除。Run6证明早期闭环但后段发生物理动态无解后的可恢复hold；Run7暴露长seed retime；Run8精确复现one-piece gradient assertion；修复并重跑全部contracts后，Run9成为第一条healthy+early-activation run，随后停止。

```text
FULL_RUN_COUNT = 6
TECHNICAL_LAUNCH_ATTEMPT_COUNT = 9
FIRST_HEALTHY_EARLY_JOINT_RUN_FOUND = YES
FIRST_HEALTHY_EARLY_JOINT_RUN_INDEX = 9
```

没有rosbag。本轮保留Run9、关键失败日志、compact metrics、fixture和源码；当前磁盘仍有约15 GiB可用，未为腾空间删除关键证据。最终检查无 roscore、rosmaster、roslaunch、RViz、Gazebo或recorder进程。

## 9. 修改文件

Production主要文件：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  plan_manage/include/plan_manage/planner_manager.h
  plan_manage/src/planner_manager.cpp
  traj_utils/msg/TopologyCandidate.msg
  traj_utils/msg/TopologyCandidateBundle.msg
  traj_utils/msg/TeamTrajectorySolution.msg

ros_ws/src/multi_uav_formation/
  include/multi_uav_formation/team_planning_context.h
  include/multi_uav_formation/team_visibility_optimizer.h
  src/multi_uav_topology_coordinator.cpp
  src/team_visibility_optimizer.cpp
  CMakeLists.txt
```

Tests/evidence：

```text
ros_ws/src/multi_uav_formation/test/early_joint_primary_contract_test.cpp
ros_ws/src/multi_uav_formation/test/team_visibility_optimizer_contract_test.cpp
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  plan_manage/test/trajectory_lifecycle_wiring_test.py
early_joint_primary_validation_20260913/analyze_early_joint.py
early_joint_primary_validation_20260913/analysis/run9_early_joint.json
```

## 10. 最终字段

```text
BASELINE: feedback_56

ARCHITECTURE_BEFORE:
  local-finalized -> team combination -> joint refinement
ARCHITECTURE_AFTER:
  team context -> topology tuple -> local pre-final seed/corridor
  -> primary joint P/T/yaw -> team preflight -> 3ACK/common activation

EARLY_JOINT_PRIMARY_TRAJECTORY_PLANNING_ACTIVE: YES
JOINT_INPUT_IS_PRE_FINAL_LOCAL_SEED: YES
LOCAL_FINAL_WINNERS_ARE_NOT_REQUIRED_BEFORE_JOINT: YES
TEAM_PLANNING_CONTEXT_IMPLEMENTED: YES
TEAM_TOPOLOGY_TUPLE_USED: YES

LOCAL_FIRST_SAFE_FALLBACK_PRESERVED: YES
JOINT_FAILURE_BLOCKS_LOCAL_COVERAGE: NO

JOINT_P_T_ACTUALLY_SHAPES_TRAJECTORY: YES
TEAM_BENEFIT_REQUIRING_INDIVIDUAL_SACRIFICE_CONTRACT: PASS
JOINT_RESPECTS_LOCAL_SFC: YES
TEAM_COMMON_ACTIVATION_VALIDATED: YES
THREE_ACK_PROTOCOL_PRESERVED: YES

EARLY_JOINT_ATTEMPT_COUNT: 79
EARLY_JOINT_SUCCESS_COUNT: 11
EARLY_JOINT_ACTIVATION_COUNT: 2 team solutions / 6 UAV activations
EARLY_JOINT_SELECTED_NONLOCAL_BEST_TUPLE_COUNT: 0
TEAM_BENEFIT_REQUIRING_INDIVIDUAL_SACRIFICE_COUNT: 0

EARLY_JOINT_CAMERA_TIME_GAIN_SUM: +0.100000 camera-s
EARLY_JOINT_ALL3_GAIN_SUM: +0.066667 seed-level;
  strict committed-local counterfactual UNKNOWN (missing A-side All3 telemetry)

HEALTHY_FULL_RUN_FOUND: YES
FULL_RUN_COUNT: 6 route-complete / 9 technical launches

TERMINAL_HOLD_COUNT: 0
MOVING_SUCCESSOR_STARVATION_COUNT: 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0

CAMERA_TIME: 229.517925 camera-s
MEAN_VISIBLE: 2.863200
K2: 0.990022
ALL3: 0.873178
NONE: 0
LONGEST_K2_LOSS: 0.599683 s
LONGEST_BLACKOUT: 0 s
ACTUAL_Q_DIR_MEAN: 0.918449
EXECUTED_ENCIRCLEMENT_RATIO: 0.254374

VISIBILITY_VS_FEEDBACK56:
  camera-time/mean-visible/All3 higher; K2 slightly lower but ~0.99;
  Q_dir/Multi2/Multi3/strict encirclement lower
EARLY_JOINT_VS_OLD_COMBINATION:
  2 executed comparisons; camera better/equal/worse = 1/1/0;
  K2 delta = 0/0; strict All3 comparison unavailable

VISIBILITY_FIRST_SEMANTICS_PRESERVED: YES
STRICT_25_170_REMAINS_NON_HARD: YES
FEEDBACK56_RECOVERY_GUIDE_FIX_PRESERVED: YES
TIME_BUDGET_CLOSED_LOOP_PRESERVED: YES
POST_DEADLINE_RECOVERY_PRESERVED: YES

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

现在三架机第一次真正进入同一个联合优化问题，是 Coordinator 将同一 `TeamPlanningContext` 下的三条 pre-final `JOINT_SEED` 连同 Local-SFC送入既有 `TeamVisibilityOptimizer::optimize()` 的时刻；以前传入的是三架各自已经完成本地 MINCO/SCP/finalization 的 executable polynomial，团队层只能在既成结果上做后置微调。
