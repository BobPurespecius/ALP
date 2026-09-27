# 提前可见性 topology 修复与 Scenario A 五次 FULL ON 验证

日期：2026-09-12  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
承接基线：`feedback_47.md`（编号扫描时 `feedback_48.md` 已存在，故本报告 exclusive create 为 49）

## 1. 结论

受控 production-chain 证据已闭合提前绕行问题：当前仍可见且 NOMINAL 机体安全时，已有 shared visibility risk support 在预测 LOS loss 前触发；NOMINAL、SIDE_PLUS、SIDE_MINUS 均进入真实生产链；有利侧可完成 A*/Local-SFC/MINCO/SCP/final preflight，被选择并在损失前激活；镜像场景自然选择相反侧。简单 SIDE offsets 全部静态失败时，同侧 topology 不再被提前杀死，A* rescue 能继续进入完整链。

Scenario A 的首次有效 FULL ON 是高质量运行，因此按要求冻结源码/参数继续到 5 个有效运行。五次结果没有稳定复现首次质量：Run1/2/5 的 K2 为 0.979/0.981/0.995，而 Run3/4 为 0.557/0.575，并发生 terminal hold；Run3 还有 30 个零静态余量样本。入口修复成立，但完整系统重复运行的可见性、连续性和安全稳定性不成立。

```text
EARLY_VISIBILITY_TOPOLOGY_PROBLEM_FIXED: YES
TARGET_SIDE_VALIDATION_PASS: YES
MIRROR_VALIDATION_PASS: YES
SIMPLE_SIDE_ASTAR_RESCUE_PASS: YES
VISIBILITY_TRIGGER_METHOD: SHARED_RISK_SUPPORT

FIRST_ON_RESULT_PROMISING: YES
VALID_ON_RUN_COUNT: 5
INVALID_ON_RUN_COUNT: 1
```

## 2. 原始根因与修复

`ROOT_CAUSE_IF_NOT_INITIALly_FIXED`：

1. shared visibility risk 已能触发 SIDE dispatch，但 `DynamicRiskInfo.conflict_time` 仍来自动态碰撞 evaluator 的最近 moving-obstacle 时刻。纯 visibility 触发时，这个时刻可能属于无关动态物体或落在轨迹末端，导致 SIDE guidance window 错位或 `WINDOW_NO_OVERLAP`；它不是实际第一个 LOS/FOV risk witness。
2. visibility-only topology 的简单 SIDE seed 与 A* base 仍沿用“必须比 NOMINAL 的 moving clearance 改善”的旧动态语义。NOMINAL 和 SIDE 对无关动态物体都已经绝对安全时，零 improvement 会错误阻止 A* 入口。
3. A* 已找到静态无碰撞同侧 guide 后，旧 handoff 仍会用下游本来要修正的 unconstrained rebuilt polynomial 的动态结果提前拒绝；Local-SFC 为空时（扫描证明无需平面）也缺少明确的成功 handoff 身份。

`FIX_APPLIED`：

- 在 optimizer 诊断快照中记录 directional/Elastic shared visibility support 的最早非零 trajectory time、observer position 和 predicted target position。
- visibility-only trigger 用该最早 witness 重写 SIDE risk window；不改变 shared risk 公式或阈值。
- visibility-triggered SIDE seed/A* base 使用既有 moving-obstacle absolute clearance；非 visibility 触发继续保留原 improvement 语义。
- A* guide + static/side semantics + Local-SFC build 成功即可进入既有 SIDE-SCP；dynamic safety 仍由完整 P/T SCP 和 authoritative final checker重新验证。
- `SFC_NOT_REQUIRED` 作为成功的 Local-SFC scan/handoff，而不是伪装成使用了平面。

没有修改 J_vis deep-risk、25°/170°、Q_dir、SIDE offsets、A* 参数、安全距离、brake/stop、Stage2/3A/3B 主公式或 joint/coordinator 生命周期。

## 3. 修改文件

- `planner/traj_opt/include/optimizer/poly_traj_optimizer.h`
  - `CostGradientSnapshot` 增加最早 visibility-support witness。
- `planner/traj_opt/src/poly_traj_optimizer.cpp`
  - directional J_vis 与 Elastic risk 在原采样环内记录最早非零 witness。
- `planner/plan_manage/include/plan_manage/planner_manager.h`
  - candidate 增加 `local_sfc_handoff_valid` 诊断身份。
- `planner/plan_manage/src/planner_manager.cpp`
  - 正确绑定 visibility witness；修复 visibility-only simple/A* 动态入口语义；允许有效 A*/Local-SFC handoff 进入既有 SCP/final checker。
- `planner/plan_manage/test/visibility_topology_production_test.cpp`
  - target-side、镜像和 simple-offset-fail/A*-rescue 三组 production-chain 合同。
- `planner/plan_manage/test/visibility_topology_fixture_scene.json`
  - 两个镜像柱体的确定性测试场景。
- `planner/plan_manage/CMakeLists.txt`
  - 注册 production test。
- `early_visibility_topology_validation_20260912/analyze_run.py`、`aggregate.py`
  - 离线审计与五次统计；不参与 production。

## 4. 受控 production-chain 证据

原场景 observer x=-2：

```text
trigger=1, SIDE_PLUS=1, SIDE_MINUS=1, executable=1
selected topology enum=1
TRIGGER_LEAD_TIME=1.550 s
ACTIVATION_LEAD_TIME约1.42 s
```

镜像 observer x=+2：相同链路成立，selected topology enum=2，与原场景相反；没有 preferred sign 或手工 case。

simple SIDE offsets 全部 static fail 的子例：

```text
NO_STATIC_FEASIBLE_SIDE
-> ASTAR_REPAIR_ACCEPT
-> local-sfc-handoff status=SFC_NOT_REQUIRED handoff_allowed=1
-> SIDE MINCO/SCP
-> final preflight
-> executable candidate
```

production test 的 8 条断言全部 PASS：production batch、loss 前 trigger、双侧 dispatch、完整 executable chain、loss 前 activation、镜像相反选择、正 activation lead、simple seed -> A*/Local-SFC/MINCO rescue。

## 5. Build 与测试

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

结果：6 个请求/依赖包全部成功，无失败。

17 个相关可执行测试全部 PASS：

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

`git diff --check`、分析脚本 Python compile：PASS。

## 6. 运行有效性与历史比较

场景 SHA256：`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`。五次有效运行均为相同 production、参数、`long_cylinder_forest_visibility_stress.json`、FULL ON、native RViz，target route 正常完成约 80.20 s；多次序列中没有调参或修改 production。

第一次技术尝试因继承 `/home/bob/catkin_ws` 的 Python message binding，无法导入当前 `PolyTraj`，导致主要 lifecycle/trajectory 记录缺失，标为 `INVALID_RUN` 并保留；随后显式 source ALP `ros_ws/devel` 后取得 Run1。结果差的 Run3/4/5 均保留为有效运行。

最近的 feedback_42 Scenario A 使用相同 scene hash、route、camera 与 safety/main flags，但其保留目录中 `run_meta/commands/activated/logs` 的 epoch 与 `trajectory.csv` 相差约 1623 s，属于混合 raw artifact，无法进行严格全指标重放：

```text
NO_STRICTLY_COMPARABLE_BASELINE
```

feedback_42 报告本身记录 terminal hold=2、trajectory-end-before-next=2、约386次 starvation、collision=0。Run1 绝对质量为 K2=0.979、无 blackout、无 collision/swarm violation/starvation/hold、且 LEFT/RIGHT 均实际执行，故 `FIRST_ON_RESULT_PROMISING=YES`；这不是依赖单一指标的判定。

## 7. Run1 完整指标

### 7.1 可见性

```text
ACCUMULATED_CAMERA_VISIBLE_TIME: 221.602251 camera-s
MEAN_VISIBLE: 2.763630
K2: 0.978905
ALL3: 0.784725
NONE: 0
LONGEST_K2_LOSS: 0.701253 s
LONGEST_BLACKOUT: 0 s
```

| UAV | visible ratio | static LOS loss s | dynamic LOS loss s | HFOV loss s | VFOV loss s | range loss s |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.942933 | 1.642575 | 1.376731 | 0.632793 | 0.890004 | 0.033809 |
| 1 | 0.853976 | 6.360200 | 3.329399 | 0.307652 | 3.377343 | 0 |
| 2 | 0.966721 | 1.934760 | 0.733695 | 0 | 0 | 0 |

```text
VISIBILITY_TRIGGERED_TOPOLOGY_COUNT: 649
STATIC_LOS_TRIGGER_COUNT: 609
DYNAMIC_LOS_TRIGGER_COUNT: 237
FOV_TRIGGER_COUNT: 63
TRIGGER_LEAD_TIME p50/p95/min: 0 / 2.604602 / 0 s
TRIGGER_TO_READY p50/p95/max: 0.016836 / 0.285971 / 0.827497 s
TRIGGER_TO_ACTIVATION p50/p95/max: 0.139085 / 0.409607 / 0.927078 s
VISIBILITY_TRIGGER_FALSE_NEGATIVE_COUNT: 0
```

`TRIGGER_TO_READY` 最小值可为负（-0.239762 s），原因是 risk witness 在 candidate 优化采样中产生、trigger 日志在 ready log 后输出；以上 p50/p95 使用真实 message `candidate_ready_time`，activation 使用 executor `start_time`，没有再用 selector 时间冒充 activation。

Run1 有 5 个 K2-loss onset；每个 onset 前 3.5 s 内均有 trigger 且已有 SIDE activation。因此请求的失败类为：NOT_TRIGGERED=0、TRIGGER_TOO_LATE=0、SEED_FAILED=0、ASTAR_FAILED=0、MINCO_FAILED=0、FINAL_SAFETY_FAILED=0、NOT_SELECTED=0、ACTIVATION_TOO_LATE=0；另记 `MITIGATION_EXECUTED_BUT_TEAM_K2_LOSS_REMAINED=5`，避免把团队仍有短时 K2 loss 伪装成入口失败。

### 7.2 绕行 funnel

```text
NOMINAL_ATTEMPT_COUNT: 318 planning batches
LEFT_ATTEMPT_COUNT: 454
RIGHT_ATTEMPT_COUNT: 465
LEFT_SIMPLE_SEED_FAIL / RIGHT_SIMPLE_SEED_FAIL: 225 / 231
LEFT_ASTAR_ATTEMPT / RIGHT_ASTAR_ATTEMPT: 98 / 55
LEFT_ASTAR_SUCCESS / RIGHT_ASTAR_SUCCESS: 74 / 50
SAFE_NOMINAL_COUNT: >=647（selector-admitted telemetry lower bound）
SAFE_LEFT_COUNT / SAFE_RIGHT_COUNT: 428 / 439
LOCAL_SELECTED_NOMINAL / LEFT / RIGHT: 647 / 71 / 41
ACTUAL_NOMINAL_ACTIVATION / LEFT / RIGHT: 649 / 53 / 28
TARGET_SIDE_ROUTE_SELECTED_COUNT: UNAVAILABLE
FAR_SIDE_ROUTE_SELECTED_COUNT: UNAVAILABLE
```

production telemetry 只携带 SIDE_PLUS/SIDE_MINUS topology，没有“target-side/far-side”标签；因此两项不可无证据推断。受控场景已经证明 target-side 选择和镜像反向选择，但不能把 fixture 标签套到全场运行。

### 7.3 合围、多方向与 joint

```text
EXECUTED_ENCIRCLEMENT_RATIO: 0.757973
SAME_SEMICIRCLE_RATIO: 0.183701
GAP_MIN deg p50/p95/min: 79.715410 / 108.372181 / 20.248829
GAP_MAX deg p50/p95/max: 156.803769 / 216.075062 / 314.141851
ACTUAL_Q_DIR mean/p50/p95: 0.973189 / 1 / 1
HIGH_QUALITY_DIRECTIONAL_RATIO: 0.956598
HIGH_QUALITY_MULTI3_RATIO: 0.746717
HIGH_QUALITY_MULTI2_RATIO: 0.935918
TEAM_REFERENCE_COVERAGE_RATIO: 0.626605
MULTIVIEW_JOINT_ATTEMPT_COUNT: 94
MULTIVIEW_JOINT_SUCCESS_COUNT: 54
MULTIVIEW_COMMIT_COUNT: 45
MULTIVIEW_ACTUAL_3UAV_ADOPTION_COUNT: 45
THREE_ACK_COUNT: 45
COMPLETE optimizer->proposal->3ACK->commit->3adoption->optimized-yaw execution chains: 45
joint optimization time ms mean/p50/p95/max: 2.7785 / 2.9625 / 7.06695 / 14.136
```

### 7.4 连续性与安全

```text
MOVING_SUCCESSOR_STARVATION_COUNT: 0
TERMINAL_HOLD_COUNT: 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 0
ACTUAL_ACTIVATION_RATE UAV0/UAV1/UAV2: 3.777841 / 3.553415 / 3.840182 Hz
LOW_SPEED episode count/p50/p95/max: 3 / 0.367300 / 0.636609 / 0.666533 s

COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
MIN_STATIC_CLEARANCE: 0.146065 m
MIN_DYNAMIC_CLEARANCE: 0.201524 m
MIN_SWARM_DISTANCE: 0.765695 m
MAX_PLANNER_SPEED: 3.019587 m/s
MAX_COMMAND_SPEED: 3.000181 m/s
MAX_ODOM_SPEED: 3.167959 m/s
```

### 7.5 实际 activated trajectory 质量

| source | curvature p50/p95/max | heading Δ deg p50/p95/max | jerk p50/p95/max | min piece s |
|---|---|---|---|---:|
| NOMINAL | 0.1572 / 2.3171 / 502.3940 | 0.1917 / 1.6644 / 179.9835 | 1.1223 / 5.7061 / 23.0254 | 0.002400 |
| LEFT | 0.7246 / 6.9905 / 405.1011 | 1.4283 / 7.8687 / 179.7748 | 3.3128 / 8.9036 / 21.6305 | 0.192930 |
| RIGHT | 0.6060 / 5.7980 / 606.1978 | 1.1352 / 7.3076 / 140.0416 | 2.8844 / 8.5277 / 21.3075 | 0.856685 |
| OPTIMIZER_SUCCESS | 0.1892 / 2.7456 / 606.1978 | 0.2389 / 2.6635 / 179.9835 | 1.2623 / 6.4871 / 23.0254 | 0.002400 |
| OPTIMIZER_FAILED_BUT_SAFE | 0.6267 / 13.0413 / 292.4378 | 1.0048 / 8.5491 / 152.6889 | 4.0602 / 16.7046 / 22.3356 | 0.110274 |

极大 curvature/heading 样本发生在接近零速或极短 piece，不能用来替代 p50/p95；它们仍如实保留。`ROUGH_RED_TRAJECTORY_EVENT_COUNT=0`。没有独立 rough/red execution identity；`ROUGH_SAFE_CANDIDATE_USED_COUNT=31` 只能由 `FEASIBLE_FALLBACK` source identity 绑定，无法补造 generation/candidate/revision/hash 级 rough 事件。

## 8. 五次有效运行（全部纳入）

| Run | camera-s | mean visible | K2 | All3 | None | longest K2 loss s | blackout s | static/dynamic LOS loss total s | triggers | actual N/L/R | Qdir | HQ-dir / HQ-3 | encircle / same-semicircle | hold | collision / unvalidated | complete chains |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---|---|---:|---|---:|
| 1 | 221.6023 | 2.7636 | 0.9789 | 0.7847 | 0 | 0.7013 | 0 | 9.9375 / 5.4398 | 649 | 649/53/28 | 0.9732 | 0.9566/0.7467 | 0.7580/0.1837 | 0 | 0/0 | 45 |
| 2 | 231.3207 | 2.8865 | 0.9813 | 0.9061 | 0.0008 | 0.7669 | 0.0628 | 4.4659 / 2.1990 | 593 | 675/41/26 | 0.9768 | 0.9588/0.8712 | 0.7216/0.1904 | 0 | 0/0 | 32 |
| 3 | 135.2534 | 1.6867 | 0.5575 | 0.1759 | 0.0467 | 25.1500 | 2.6002 | 96.3530 / 12.2520 | 672 | 368/29/34 | 0.8774 | 0.6460/0.1448 | 0.6792/0.1949 | 2 | 30/2816 | 4 |
| 4 | 150.6744 | 1.8792 | 0.5749 | 0.3122 | 0.0079 | 31.4816 | 0.6342 | 60.7941 / 22.9066 | 545 | 411/25/18 | 0.7853 | 0.3971/0.2777 | 0.3163/0.4709 | 2 | 0/2795 | 12 |
| 5 | 229.5284 | 2.8629 | 0.9954 | 0.8675 | 0 | 0.3662 | 0 | 7.6533 / 3.2762 | 562 | 666/42/36 | 0.9679 | 0.9365/0.8049 | 0.7100/0.1765 | 1 | 0/56 | 34 |

五次 moving successor starvation 和 trajectory-end-before-next-activation 均为 0；terminal hold 分布为 0/0/2/2/1。LEFT、RIGHT 在每一次都真实 activation，不只是生成。

## 9. 跨 run mean/std/min/max

| metric | mean | std | min | max |
|---|---:|---:|---:|---:|
| camera visible time | 193.675832 | 41.820404 | 135.253416 | 231.320714 |
| mean visible | 2.415794 | 0.521935 | 1.686706 | 2.886542 |
| K2 | 0.817584 | 0.205434 | 0.557469 | 0.995433 |
| All3 | 0.609291 | 0.303835 | 0.175947 | 0.906075 |
| None | 0.011081 | 0.018062 | 0 | 0.046710 |
| longest K2 loss s | 11.693194 | 13.719875 | 0.366166 | 31.481592 |
| longest blackout s | 0.659440 | 0.999296 | 0 | 2.600218 |
| static LOS loss total s | 35.840758 | 36.699571 | 4.465886 | 96.352998 |
| dynamic LOS loss total s | 9.214711 | 7.686729 | 2.198988 | 22.906578 |
| visibility topology triggers | 604.2 | 49.019996 | 545 | 672 |
| actual LEFT activation | 38.0 | 10.0 | 25 | 53 |
| actual RIGHT activation | 28.4 | 6.374951 | 18 | 36 |
| actual NOMINAL activation | 553.8 | 135.096114 | 368 | 675 |
| trigger lead run-mean s | 0.568398 | 0.135844 | 0.388238 | 0.801392 |
| actual Qdir mean | 0.916119 | 0.075147 | 0.785293 | 0.976778 |
| high-quality directional | 0.778985 | 0.224592 | 0.397090 | 0.958766 |
| high-quality Multi3 | 0.569058 | 0.297767 | 0.144798 | 0.871178 |
| executed encirclement | 0.637016 | 0.162326 | 0.316302 | 0.757973 |
| same semicircle | 0.243298 | 0.113959 | 0.176537 | 0.470877 |
| starvation | 0 | 0 | 0 | 0 |
| terminal hold | 1.0 | 0.894427 | 0 | 2 |
| end before activation | 0 | 0 | 0 | 0 |
| rough red event | 0 | 0 | 0 | 0 |
| safe fallback used | 34.4 | 4.758151 | 30 | 43 |
| collision samples | 6.0 | 12.0 | 0 | 30 |
| swarm violations | 0 | 0 | 0 | 0 |
| unvalidated executed samples | 1133.4 | 1365.433206 | 0 | 2816 |

`TARGET_SIDE_ROUTE_SELECTED_COUNT` 和 `FAR_SIDE_ROUTE_SELECTED_COUNT` 在所有运行均因 production telemetry 缺字段而为 UNAVAILABLE，未参与统计。

```text
ACCUMULATED_CAMERA_VISIBLE_TIME_MEAN: 193.675832
MEAN_VISIBLE_MEAN: 2.415794
K2_MEAN: 0.817584
ALL3_MEAN: 0.609291
NONE_MEAN: 0.011081
LONGEST_K2_LOSS_MEAN: 11.693194 s
LONGEST_BLACKOUT_MEAN: 0.659440 s
LEFT_ACTIVATION_MEAN: 38.0
RIGHT_ACTIVATION_MEAN: 28.4
NOMINAL_ACTIVATION_MEAN: 553.8
TRIGGER_LEAD_TIME_MEAN: 0.568398 s
ACTUAL_Q_DIR_MEAN_ACROSS_RUNS: 0.916119
HIGH_QUALITY_MULTI3_RATIO_MEAN: 0.569058
EXECUTED_ENCIRCLEMENT_RATIO_MEAN: 0.637016
MOVING_SUCCESSOR_STARVATION_MEAN: 0
TERMINAL_HOLD_TOTAL: 5
COLLISION_SAMPLES_TOTAL: 30
SWARM_CLEARANCE_VIOLATION_SAMPLES_TOTAL: 0
UNVALIDATED_EXECUTED_SAMPLES_TOTAL: 5667
ROUGH_RED_TRAJECTORY_EVENT_MEAN: 0
```

## 10. 稳定性与剩余问题

```text
VISIBILITY_IMPROVEMENT_STABLE: NO
TOPOLOGY_BEHAVIOR_STABLE: PARTIAL
CONTINUITY_STABLE: NO
SAFETY_STABLE: NO

BEST_RUN: Run2
MEDIAN_RUN: Run1（按K2排序）
WORST_RUN: Run3
```

- 提前 visibility 绕行入口是否真正解决：**YES**，受控原/镜像 production chain 与 A* rescue 均有 loss 前 activation 证据。
- shared risk support 是否足够提前：受控链 **YES**；完整场景为 **PARTIAL**。五次 false-negative 均为 0，run-mean lead 0.388–0.801 s，但大量 witness 已处于深风险区，Run1 lead p50=0，不能声称每次都有正预测余量。
- LEFT/RIGHT 是否实际执行：**YES**，五次分别 LEFT 25–53、RIGHT 18–36 次 actual activation。
- target-side 绕行是否减少 LOS loss：受控 target-side/mirror 对照 **YES**；全场 production 缺 target-side label，不能做归因统计。
- 第一次改善是否稳定复现：**NO**。3/5 为高 K2，但 Run3/4 是严重退化，不可用均值掩盖。
- 当前最大剩余问题：不是 topology 入口，而是长路线中部分 UAV 在后半程失去持续的已验证 successor 并进入 terminal hold；这随后造成巨量 unvalidated samples、静态 LOS loss、K2/合围崩塌。Run3 的零静态余量样本还构成明确安全回归证据。按本轮范围未继续修改 recovery/lifecycle、安全或参数。

```text
PLANNER_LEVEL_BRAKE_REINTRODUCED: NO
NEW_STOP_FALLBACK_INTRODUCED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

原始有效运行：

- `scenario_a_20260911_234722/early_visibility_20260912_run1_valid`
- `scenario_a_20260911_234722/early_visibility_20260912_run2_valid`
- `scenario_a_20260911_234722/early_visibility_20260912_run3_valid`
- `scenario_a_20260911_234722/early_visibility_20260912_run4_valid`
- `scenario_a_20260911_234722/early_visibility_20260912_run5_valid`

统计证据：`early_visibility_topology_validation_20260912/analysis/aggregate.json`。

```text
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_49.md
```
