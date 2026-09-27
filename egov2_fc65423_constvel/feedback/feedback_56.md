# Feedback 056 — Feedback53→54/55 successor regression、deterministic fixture 与健康 Scenario A 闭环

日期：2026-09-13  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
known-good：`feedback_53.md`  
bad range：`feedback_54.md`–`feedback_55.md`  
场景：`long_cylinder_forest_visibility_stress.json`

## 0. 结论

本轮把问题作为 **Feedback53→54/55 的 production regression** 处理，没有用新 dynamic-SFC、新 A*、新恢复 FSM 或安全阈值调整补能力。第一处分叉已由 deterministic production fixture 锁定：Feedback54 在让 visibility-first NOMINAL 不再受 strict 25°/170° objective shaping 时，错误地同时撤掉了 hypothesis 0 的 planner-validated recovery guide，并让 accepted-trajectory warm start 抢在仍可用的 obstacle-aware guide 前面。于是 Feedback53 会沿 guide 绕墙进入原 MINCO/SCP 的输入，在 Feedback54/55 中变成从 current PVA 到 terminal reference 的直线 chord；同一 fixture 中该 chord 穿墙，后续自然表现为 static/dynamic/final reject 和 successor supply 断裂。

本轮还沿失败链修复了三个同属 authority/data-flow 的直接缺口：短或失效 committed-team suffix 不再遮住独立有效的 recovery guide；validated coverage 已过期时，traj_server 不再用旧 polynomial 的 nominal end 重新建立 PVA authority、拒绝显式 full-safe fresh recovery；worker 会在“下一 monitor tick + activation margin”将越过既有 guide 证书时提前刷新。所有修复都复用既有 Target/Guide→N/L/R→A*/Local-SFC→MINCO/SCP→current-revision hard preflight 链。

第 8 次 FULL ON 取得首个严格健康运行，随即停止仿真：六项门禁全部为 0，三机全程保持 rolling validated moving execution，27 条 optimizer→proposal→3 ACK→commit→三机 adoption→optimized-yaw actual execution 链闭合。

```text
FINAL_ASSESSMENT = PASS

REGRESSION_TREATED_AS_BUG_NOT_NEW_FEATURE = YES
KNOWN_GOOD_BASELINE = feedback_53
BAD_RANGE = feedback_54_to_feedback_55
DETERMINISTIC_REGRESSION_FIXTURE_CREATED = YES

HEALTHY_FULL_RUN_FOUND = YES
FIRST_HEALTHY_FULL_RUN_INDEX = 8
FULL_RUN_COUNT = 8
```

## 1. 保护、差分范围与第一处分叉

开始修改前已保存完整 dirty-worktree diff：

```text
regression_fix_20260913/pre_fix_current.diff
```

没有 commit、reset、覆盖或丢弃用户/此前 AI 的未提交工作。最终报告编号通过实际扫描 `feedback/` 后 exclusive create 为 56。

### 1.1 Feedback53 为什么能工作

Feedback53 的 normal tracking initializer authority 是：可用 committed team suffix，否则使用 worker 已经通过地图/reachability 验证的 recovery guide，再进入原 MINCO/SCP 和 final preflight。guide 是空间初始化证据，不是 strict encirclement hard gate；因此它既能绕开墙/柱，也不要求最终进入 25°/170°。

### 1.2 Feedback54/55 的回归 hunk

第一处分叉位于：

```text
REGRESSION_CAUSING_FILE:
  plan_manage/src/planner_manager.cpp

REGRESSION_CAUSING_FUNCTION:
  EGOPlannerManager::reboundReplan

REGRESSION_CAUSING_HUNK:
  visibility-first hypothesis 0 通过 !visibility_first_nominal 条件跳过
  buildRecoveryGuideSeed；同时旧 warm-start 条件把
  visibility_first_nominal 当成绕过 recovery_target_.usable(...) 的理由。
```

原意仅是“不向 NOMINAL 注入 strict gap/recovery objective”。实际却把 objective authority 与 collision-free initializer authority 绑在一起：

```text
Feedback53:
  valid recovery guide -> obstacle-aware P/T initializer -> MINCO/SCP

Feedback54/55:
  hypothesis 0 drops/bypasses guide
  -> accepted warm start or fresh direct chord
  -> wall/column crossing initializer
  -> static/dynamic/final reject
  -> predecessor coverage expires
```

```text
FIRST_DIVERGENCE_FROM_FEEDBACK53:
  VISIBILITY_FIRST_NOMINAL_DROPPED_OR_BYPASSED_VALIDATED_RECOVERY_GUIDE

REGRESSION_PHYSICAL_EFFECT:
  obstacle-aware detour initializer became a direct or stale-polynomial-biased
  trajectory; the fixture's direct chord has signed static clearance -0.619039 m.
```

Feedback55 修复了 predecessor 过期后的永久 zero-budget loop，但没有恢复上述 guide authority，所以能从 hold 恢复，却仍频繁先失去 moving successor。

## 2. Deterministic Feedback53-vs-current fixture

新增：

```text
plan_manage/test/feedback53_regression_fixture_test.cpp
```

fixture 使用已知 free-map 与一面挡住 start→goal 直线的墙，固定 current P/V/A、target、trajectory 53、future activation、recovery target/guide 和 planner 参数；它直接调用 production `EGOPlannerManager`、真实 MINCO 和 finalization，而不是手写替代优化器。

### 2.1 BEFORE

```text
REGRESSION_FIXTURE_BEFORE:
  fresh direct chord generated = YES
  direct chord static safe = NO
  signed static clearance = -0.619039 m
  result = no executable direct successor
```

合同断言：`PASS feedback54_direct_chord_reproduces_static_candidate_loss`。

### 2.2 AFTER

```text
REGRESSION_FIXTURE_AFTER:
  validated recovery guide available for hypothesis 0 = YES
  guide wins before accepted warm start = YES
  guide detour static safe = YES
  guide enters existing MINCO/SCP chain = YES
  current-revision hard preflight = PASS
  committed successor trajectory = 54
  duration = 6.632550 s
  dynamics/static/swarm valid = 1/1/1
  handoff dP/dV/dA = 0/0/0
```

日志：`regression_fix_20260913/fixture_after_guide_priority.log`。fixture 同时覆盖 validated-end 先于 nominal polynomial end、explicit recovery identity/full-safety/generation gate，以及下一 monitor+activation 即将越过 guide expiry 时必须刷新。

## 3. 最小 production 修复

### 3.1 恢复 guide 的 initializer authority，不恢复 strict shaping

`planner_manager.cpp::reboundReplan` 当前顺序为：

```text
active committed team reference
-> usable planner-validated recovery guide
-> accepted-trajectory warm start（仅无 guide 时）
-> fresh moving initializer
```

hypothesis 0 仍不注入未提交 strict recovery target/gap objective；恢复的是 Feedback53 已有的空间初始化数据流，不是恢复 25°/170° hard gate。`local_execution_contract.h::acceptedWarmStartEligible` 锁定：usable guide 存在时，accepted warm start 不能越权。

### 3.2 team suffix 与 guide 是独立 authority

`planner_manager.cpp::buildRecoveryGuideSeed` 中，短于 0.5 s、无 piece 或 fresh retime 后 dynamics invalid 的 team suffix只使该来源失败；它随后继续尝试独立的 recovery guide，而不是直接返回并落到穿障碍 direct chord。合法 team suffix仍按 authoritative activation P/V/A 重建时长和末端状态。

### 3.3 validated coverage expiry 不等于 nominal polynomial end

`trajectory_lifecycle.h::postDeadlineRecoveryHandoff` 与 `traj_server.cpp::executionHandoffGate` 保留以下 hard gates：

- explicit `POST_DEADLINE_RECOVERY_VALIDATED` identity；
- `safety_validated=true`；
- strictly newer generation；
- finite activation time。

但不再要求 fresh recovery activation 晚于旧 polynomial nominal end。旧 safety certificate 已经过期后，该 nominal end 不具有重新建立 predecessor PVA authority 的资格；Run2 中 fully-safe recovery 195 的 `ACTIVE_PVA_MISMATCH` 因而不再永久阻止 activation。

### 3.4 guide refresh 不制造周期性 freshness 缝隙

`target_guide_lifecycle.h::targetGuideRefreshDue` 与 `team_target_reachability.cpp::tick` 使用既有 worker monitor period、activation margin 和 guide policy推导刷新时机：若当前 tick 可用、但下一 tick+activation 将不可用，则尝试 same-target refresh；刷新失败时仍保留旧证书直到其真实 expiry。没有新增生命周期阈值。

### 3.5 修改文件

Production：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  plan_manage/include/plan_manage/local_execution_contract.h
  plan_manage/include/plan_manage/target_guide_lifecycle.h
  plan_manage/src/planner_manager.cpp
  plan_manage/src/team_target_reachability.cpp
  plan_manage/src/traj_server.cpp
  traj_utils/include/traj_utils/trajectory_lifecycle.h
```

Tests/analysis：

```text
  plan_manage/CMakeLists.txt
  plan_manage/test/feedback53_regression_fixture_test.cpp
  plan_manage/test/local_execution_contract_test.cpp
  plan_manage/test/recovery_nonblocking_wiring_test.py
regression_fix_20260913/analyze_bad_candidates.py
```

## 4. Build 与合同

最终源码执行并通过：

```text
catkin build ego_planner -j2 --no-status --workspace ros_ws = PASS
git diff --check = PASS
```

18 个相关 C++ production/contract/gradient binaries 全部 PASS：

```text
feedback53_regression_fixture_test
fresh_moving_initializer_contract_test
local_execution_contract_test
recovery_probe_production_test
trajectory_lifecycle_contract_test
visibility_topology_production_test (8/8)
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

这些 binary 覆盖已有 P/tau/piece-boundary/yaw/J_vis deep-risk/Elastic/Q_dir/gap 的 analytic-vs-finite-difference gradients，以及 N/L/R→A*/Local-SFC→MINCO/SCP、team/joint 和 lifecycle contracts。

Python：

```text
adaptive_execution_contract_test.py = 12/12 PASS
trajectory_lifecycle_wiring_test.py = 17/17 PASS
recovery_nonblocking_wiring_test.py = 7/7 PASS
analyze_bad_candidates.py py_compile = PASS
```

## 5. FULL ON 迭代与首个健康运行

所有真实运行均使用同一 scene hash `430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`、FULL ON 和 native RViz；没有 OFF/A-B、没有降低安全阈值，也没有把差运行从 run count 删除。Run1–Run7 每次用于暴露上述下一处直接 authority bug或确认真实 hard-safety 无解窗口；只有完成对应 fixture/contracts 后才进入下一 run。

| Run | 结果 | 首个代码/物理断点 |
|---:|---|---|
| 1 | FAIL | 短/invalid team suffix 遮住仍有效 guide，UAV1 traj140 后 hold约8.79 s |
| 2 | FAIL | fully-safe fresh recovery 被 traj_server 以旧 nominal-end PVA authority判 `ACTIVE_PVA_MISMATCH` |
| 3 | FAIL | guide 当前 tick可用，但在下一 monitor+activation 前过期，形成确定性 freshness gap |
| 4 | FAIL | guide 被当前 map真实判 `GUIDE_MAP_BLOCKED`；1 hold、108 unvalidated、26 collision samples |
| 5 | FAIL | 2 hold、2 end-before-next、52 unvalidated；无 collision/swarm violation |
| 6 | FAIL | 1 brief hold、1 starvation、9 unvalidated；随后 fresh recovery恢复 |
| 7 | FAIL | 1 hold、1 starvation、121 unvalidated；当时 N/L/R 均未过原 hard safety |
| 8 | **PASS** | 六项健康门禁全零，route正常完成，随即停止 |

Run4–Run7 没有继续诱发 production 回滚：它们证明 post-deadline deadlock已消失，剩余 episode 在当时确实没有同时通过 static/dynamic/dynamics/swarm/final preflight 的 candidate。Run8 则证明最终数据流能在同一压力场景保持完整 rolling coverage。

### 5.1 Run8 健康门禁

证据：

```text
regression_fix_20260913/run8
regression_fix_20260913/analysis/run8/run8_metrics.json
regression_fix_20260913/analysis/run8/bad_candidates.json
```

```text
route = normally completed
route duration = 80.204541 s
exit code = 0

TERMINAL_HOLD_COUNT = 0
MOVING_SUCCESSOR_STARVATION_COUNT = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT = 0
UNVALIDATED_EXECUTED_SAMPLES = 0
COLLISION_SAMPLES = 0
SWARM_CLEARANCE_VIOLATION_SAMPLES = 0

actual activation rate UAV0/UAV1/UAV2 = 3.6781 / 3.6033 / 3.6532 Hz
minimum static clearance = 0.215430 m
minimum dynamic distance = 0.716069 m
minimum swarm distance = 0.927251 m
```

任务结束 recorder 的三机样本均绑定不同的新鲜 trajectory ID（297/292/295），且 `safety_validated=True`；不是以最终静止或旧 trajectory 冒充 rolling pass。

### 5.2 Team/joint 完整闭环

```text
TEAM_VIS_OPT_ATTEMPT / SUCCESS = 93 / 43
proposal -> 3 ACK -> commit = 27
3-UAV actual adoption = 27
optimized-yaw actual execution = 27 solution IDs
complete chains = 27
joint planning ms mean/p50/p95/max = 3.597 / 3.427 / 10.668 / 16.816
```

## 6. 坏候选与实际执行分离

计数合同保存在 `analysis/run8/bad_candidates.json`：静态项按 SIDE construction attempt统计；dynamic collision只计 `candidate-ground-truth` 中 optimizer success且几何 clearance≤0 的轨迹；after-SCP只计 `scp_success=1 && final_success=0`；after-reanchor只计重算 metadata后 dynamics/static/swarm invalid。

```text
BAD_CANDIDATE_STATIC_COLLISION_COUNT = 413
  STATIC_COLLISION = 392
  STATIC_COLLISION_CONFLICT_WINDOW = 21
  of which explicit pre-init static rejects = 104

BAD_CANDIDATE_DYNAMIC_COLLISION_COUNT = 0
DYNAMIC_CLEARANCE_THRESHOLD_REJECT_COUNT = 123
BAD_CANDIDATE_AFTER_SCP_COUNT = 0
BAD_CANDIDATE_AFTER_REANCHOR_COUNT = 0
REANCHOR_METADATA_RECOMPUTE_COUNT = 812
```

这 413 是生成/筛选阶段的 SIDE seed/offset 尝试，不是 413 条 executable candidate，也没有进入 executor。它们解释了 RViz/日志中仍可能看到朝墙或贴墙的试探性 candidate visualization；不能因 final checker拦住就称候选池完美。另一方面，本轮无法用同口径从 Feedback55 保存产物得到严格 before count，所以不声称“静态坏 seed 数显著下降”。可证实的改善是：当前没有 optimizer-success 的动态碰撞 candidate、没有 SCP 成功后被 final checker发现碰撞、没有 reanchor 后沿用旧 safety metadata，也没有任何实际 collision/unvalidated execution。123 次 dynamic reject是低于既有 1.1 m planning clearance的正确 hard reject，不是几何碰撞。

## 7. 仅用健康 Run8 评价 visibility

| metric | Feedback53 healthy | Run8 healthy | 差值 | 历史 OFF |
|---|---:|---:|---:|---:|
| camera-time (camera-s) | 228.612786 | **228.900393** | +0.287607 | 227.317129 |
| mean-visible | 2.851552 | **2.855893** | +0.004341 | 2.905279 |
| K2 | 0.994232 | **0.994127** | -0.000105 | 0.982248 |
| All3 | 0.857320 | **0.861765** | +0.004445 | 0.923316 |
| NONE | 0 | **0** | 0 | 约0 |
| longest K2 loss (s) | 0.265703 | **0.400443** | +0.134740 | 0.877114 |

```text
static LOS loss total = 7.504827 camera-s
dynamic LOS loss total = 2.107147 camera-s
HFOV loss total = 0
VFOV loss total = 1.435169 camera-s
range loss total = 0.503087 camera-s

ACTUAL_Q_DIR_MEAN = 0.964813
HIGH_QUALITY_DIRECTIONAL_RATIO = 0.932627
HIGH_QUALITY_MULTI2_RATIO = 0.926762
HIGH_QUALITY_MULTI3_RATIO = 0.794874
EXECUTED_ENCIRCLEMENT_RATIO = 0.351334
SAME_SEMICIRCLE_RATIO = 0.493984
```

结论：相对 Feedback53，Run8 的 camera-time、mean-visible 和 All3小幅提高，K2仅差0.000105、属于保持；Q_dir从0.955469提高到0.964813。strict encirclement ratio从0.456784降至0.351334，未保持，但 strict 25°/170°按当前已验证设计仍是 visibility/safety之后的 diagnostic/soft tie-break，不是执行 hard gate。相对历史 OFF，Run8 的 camera-time和K2更高，mean-visible和All3仍较低。

## 8. 安全边界与清理

```text
NEW_ALGORITHM_ADDED = NO
DYNAMIC_SFC_ADDED = NO
NEW_ASTAR_VARIANT_ADDED = NO
SAFETY_THRESHOLD_RELAXED = NO
1.1m_DYNAMIC_CLEARANCE_CHANGED = NO
STATIC_THRESHOLD_RELAXED = NO
SWARM_THRESHOLD_RELAXED = NO
RAW_ASTAR_EXECUTED = NO
FINAL_PREFLIGHT_BYPASSED = NO
PLANNER_BRAKE_REINTRODUCED = NO
STOP_FALLBACK_REINTRODUCED = NO
OLD_SUFFIX_EXTENDED = NO

VISIBILITY_FIRST_SEMANTICS_PRESERVED = YES
STRICT_25_170_REMAINS_NON_HARD = YES
POST_DEADLINE_RECOVERY_PRESERVED = YES
TIME_BUDGET_CLOSED_LOOP_PRESERVED = YES
FIRST_SAFE_RESERVE_AND_QUALITY_SEARCH_PRESERVED = YES
SHARED_VISIBILITY_RISK_PRESERVED = YES
SIDE_OBSERVATION_TOPOLOGY_PRESERVED = YES
LOCAL_SFC_MINCO_SCP_PRESERVED = YES
```

结束后已确认无 roscore、rosmaster、roslaunch、RViz、Gazebo、recorder 或 run script残留。删除了 Run7/Run8可重建 ROS node logs和已完成验证的 `ros_ws/build` cache；Run5–Run8大 launcher/odom/command/polynomial证据采用 gzip压缩。保留 deterministic fixture、compact metrics、关键日志、pre-fix diff、所有源码和本报告。磁盘可用空间约2.0 GiB。

## 9. 最终字段

```text
REGRESSION_TREATED_AS_BUG_NOT_NEW_FEATURE: YES
KNOWN_GOOD_BASELINE: feedback_53
BAD_RANGE: feedback_54_to_feedback_55

DETERMINISTIC_REGRESSION_FIXTURE_CREATED: YES

FIRST_DIVERGENCE_FROM_FEEDBACK53: VISIBILITY_FIRST_NOMINAL_DROPPED_OR_BYPASSED_VALIDATED_RECOVERY_GUIDE
REGRESSION_CAUSING_FILE: plan_manage/src/planner_manager.cpp
REGRESSION_CAUSING_FUNCTION: EGOPlannerManager::reboundReplan
REGRESSION_CAUSING_HUNK: visibility_first_nominal incorrectly gated guide use and warm-start precedence
REGRESSION_PHYSICAL_EFFECT: obstacle-aware detour became a wall-crossing direct/stale-biased initializer

ROOT_CAUSE: objective authority and collision-free initializer authority were accidentally coupled in Feedback54; Feedback55 fixed post-deadline budget liveness but retained that guide/data-flow regression

MINIMAL_FIX: retain visibility-first objective semantics while restoring guide-first initializer authority; let invalid team suffix fall through to guide; use validated-end rather than nominal-end authority for explicit full-safe fresh recovery; refresh guide before existing cadence crosses expiry

NEW_ALGORITHM_ADDED: NO
DYNAMIC_SFC_ADDED: NO
NEW_ASTAR_VARIANT_ADDED: NO
SAFETY_THRESHOLD_RELAXED: NO

VISIBILITY_FIRST_SEMANTICS_PRESERVED: YES
STRICT_25_170_REMAINS_NON_HARD: YES
POST_DEADLINE_RECOVERY_PRESERVED: YES
TIME_BUDGET_CLOSED_LOOP_PRESERVED: YES

REGRESSION_FIXTURE_BEFORE: direct chord generated; static hard reject; signed clearance -0.619039 m; no executable successor
REGRESSION_FIXTURE_AFTER: guide-first real MINCO/SCP; current-revision hard preflight PASS; trajectory54 committed

HEALTHY_FULL_RUN_FOUND: YES
FULL_RUN_COUNT: 8

TERMINAL_HOLD_COUNT: 0
MOVING_SUCCESSOR_STARVATION_COUNT: 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0

BAD_CANDIDATE_STATIC_COLLISION_COUNT: 413 construction attempts; 0 activated
BAD_CANDIDATE_DYNAMIC_COLLISION_COUNT: 0
BAD_CANDIDATE_AFTER_SCP_COUNT: 0
BAD_CANDIDATE_AFTER_REANCHOR_COUNT: 0

CAMERA_TIME: 228.900393 camera-s
MEAN_VISIBLE: 2.855893
K2: 0.994127
ALL3: 0.861765
NONE: 0
LONGEST_K2_LOSS: 0.400443 s
LONGEST_BLACKOUT: 0 s
ACTUAL_Q_DIR_MEAN: 0.964813
EXECUTED_ENCIRCLEMENT_RATIO: 0.351334

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

一句话回答：**Feedback53 能正常生成轨迹，是因为 visibility/quality work 仍以 planner-validated obstacle-aware recovery guide 初始化原 MINCO/SCP；Feedback54 把“移除 strict geometry shaping”错误扩大为“删除/绕过 guide authority”，Feedback55 只修了过期后的预算死锁而未恢复这条数据流，所以从那时开始反复产生穿墙或失去安全 successor 的 candidate。**
