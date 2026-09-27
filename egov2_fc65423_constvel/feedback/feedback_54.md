# Feedback 054 — visibility-first 任务语义、25°/170° 降级与单次 Scenario A 验证

日期：2026-09-13  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_53.md`  
场景：`long_cylinder_forest_visibility_stress.json`

## 0. 结论

本轮已经把生产链的任务层级统一为：

```text
hard executability
> blackout / NONE / K2 / longest K2 loss
> camera-time / mean-visible / All3 / weakest-camera / max-loss
> Q_dir / pairwise diversity / g_min / g_max / strict 25°/170°
```

严格合围定义仍完整保留，但不再作为 Target、recovery completion、本地候选、team combination、joint acceptance 或 trajectory execution 的 hard qualification。安全且高可见的非严格布局可以进入同一个 Target→Guide→Trajectory、NOMINAL/LEFT/RIGHT、team selection、joint P/T/yaw 和 3-ACK commit 链。visibility-first NOMINAL 仍属于 FULL ON production pipeline，只是不再被未提交的合围 Target/gradient预先塑形。

定向合同、gradient 和原有 17 项生产相关测试全部通过。唯一一次 Scenario A FULL ON 也证明完整 team 闭环仍工作：32 次 joint success、27 次 proposal→3 ACK→commit→三机 adoption→optimized-yaw actual execution。

但是这唯一一次运行发生了独立的 UAV0 rolling-lifecycle 失效：trajectory 177 之后没有及时形成满足原动态安全阈值的 successor，旧 predecessor deadline 过期后又把所有后续 mandatory supply 的 wall budget 压成 0，最终进入 terminal hold 并留下 910 个 unvalidated samples。因此本轮不能通过整体可见性、安全/连续性验收，也不能用受 terminal hold 污染的 All3 数字评价新排序的收益。

```text
FINAL_ASSESSMENT = PARTIAL

VISIBILITY_FIRST_SEMANTICS_CONTRACT = PASS
STRICT_25_170_HARD_GATE_REMOVAL = PASS
EXISTING_PIPELINE_REUSE = PASS
BUILD_AND_TESTS = PASS
FULL_ON_RUN_COUNT = 1
FULL_ON_SYSTEM_HEALTH = FAIL
SECOND_SIMULATION = NO
```

`PARTIAL` 的含义是：任务语义和确定性合同已经闭合，但唯一真实 run 没有保持 Feedback53 的 validated-moving-coverage/liveness，故不能宣称本轮 production 获得运行级 PASS。

## 1. 当前生产代码审计与修复后的 authority

### 1.1 25°/170° 在各层的当前状态

严格合围仍定义为：

```text
g_min >= 25° && g_max <= 170°
```

`encirclement_geometry.h::eligible()` 没有删除，`EXECUTED_ENCIRCLEMENT_RATIO`、`SAME_SEMICIRCLE_RATIO`、`g_min/g_max`、gap cost 和 `E_enc` 也继续产生。但各生产层的 authority 已改为：

| 层 | 当前语义 | 是否仍为 strict hard gate |
|---|---|---:|
| adaptive Target generation | `RecoveryTargetCandidate::eligible()` 只要求 screened、数值有限和圆周几何可计算；调用点仍存在，但 25°/170° 参数不再淘汰 Target | NO |
| Target wire validity | `readRecoveryTarget()` 保留 identity/freshness/finite geometry；忽略 strict threshold | NO |
| recovery completion | `recoveryGuidesReached()` 要求三条 guide 的 arc progress 和 terminal proximity；不要求最终 strict encirclement | NO |
| local candidate/execution | `geometryRecoveryDue()` 恒为 false；退化只记 `GEOMETRY_DEGRADED`，日志显式为 `urgent_recovery=0 geometry_execution_gate=0` | NO |
| team combination | hard-safe executable set 先形成；visibility continuity、总可见性、弱相机质量先比较，geometry 后置 | NO |
| joint acceptance | dynamics/static/dynamic/swarm/handoff 等 hard check保留；strict ratio只作 metric/soft progress | NO |
| actual trajectory execution | current revision、identity、deadline 和 final preflight保留；strict geometry不阻止 commit/activation | NO |

运行中可直接观察到多条 `g_max=190–318°`、`encirclement_ratio=0` 的 `LOCAL_COMMIT`/`SUFFIX_REVALIDATION`，且同时记录 `geometry_execution_gate=0`。这证明非严格布局不只存在于单元测试，确实拥有 production execution 资格。

### 1.2 J_enc / gap restoring / recovery Target

当前仍保留的 geometry 路径有三类：

1. adaptive generator 可生成缩小 gap cost 的候选；
2. Target/Guide 可把某个已筛选的 team layout 作为 soft reference；
3. local/joint objective 仍可计算 gap、diversity、Q_dir 和 recovery intent cost。

这些项没有被删除，因为它们仍是“可见性相当时改善多方向结构”的有用质量信号。改变的是其 authority：

- 不再先以 strict 25°/170°裁掉 Target/trajectory；
- 不再因 active geometry degraded 单独调度强制 recovery；
- comparator/selector 在 hard safety、blackout/K2、camera-time/All3/weakest-camera 之后才看 geometry；
- joint objective 可以沿 geometry 方向优化，但 final replacement acceptance 不允许它覆盖明确 visibility regression。

因此本轮没有机械修改 25°/170° 数值，也没有仅缩小一个 `J_enc` 权重来伪装层级修复。

### 1.3 Q_dir 的当前作用

Q_dir/方向多样性继续存在于：

- generator 的后级 hypothesis/Target tie-break；
- local candidate 在 visibility 指标相当后的 tie-break；
- team combination 在 safety、continuity、camera/All3、weakest/max-loss 之后的 tie-break；
- joint objective 的 soft multi-view quality，以及 visibility不退化时的 measurable progress。

所以在 All3、K2、安全相当时，Q_dir更好的候选仍能赢；但 Q_dir 的小幅收益不能覆盖 camera-time/All3 的明确损失。

### 1.4 shared-risk、SIDE 和局部绕行链

这些既有主干全部保留：

```text
shared static/dynamic/FOV visibility risk
→ NOMINAL + SIDE_PLUS + SIDE_MINUS
→ simple SIDE seed
→ 必要时 same-topology A* repair
→ Local-SFC
→ MINCO/SCP
→ current-revision final hard preflight
→ two-stage first-safe reserve / remaining-budget quality search
→ team bundle / joint P/T/yaw
→ proposal / 3 ACK / commit / adoption
```

本轮 run 中 visibility trigger 710 次，LEFT/RIGHT 均生成并实际激活；简单 SIDE seed 失败后仍观察到 A* repair 94/126 次尝试和 83/112 次成功。A* 仍只负责同一 observation topology 下的安全路径修复，最终 observation quality仍由后续 optimizer/selector决定。Local-SFC/SIDE corridor只约束对应局部绕行区间，没有新增永久观察侧锁。

### 1.5 visibility-first NOMINAL

hypothesis 0 保持 normal tracking、J_vis/FOV/yaw、MINCO/SCP、deadline和final preflight，但在没有已提交 team reference 时：

- 不使用未提交 recovery Target seed；
- 不注入 strict-gap recovery gradient；
- 不要求 terminal 进入 25°/170°。

它不是 OFF 模式，也不是第二 planner；它是 FULL ON 候选池中始终存在、从当前真实 P/V/A 出发的 visibility-first baseline。LEFT/RIGHT 和 team candidate在同一 hard-safe set 内与它竞争。

## 2. 修改内容

### 2.1 Target generation、排序与 completion

`multi_uav_formation`：

- `include/multi_uav_formation/adaptive_viewpoint_generator.h`
  - hypothesis 增加 `all3`、`min_uav_visibility`、`max_loss_duration`，公开统一 comparator。
- `src/adaptive_viewpoint_generator.cpp`
  - 对每个安全 hypothesis 计算 All3、逐机 visibility 和最长 loss；排序改为 continuity→total visibility→weakest→geometry。
  - target pool 不再由 strict 25°/170°过滤。
- `include/multi_uav_formation/team_recovery_target.h`
  - Target eligibility 不再要求 strict geometry；Target comparator visibility-first。
  - wire decode只保留 finite/identity/freshness；completion 改为 guide progress + terminal neighborhood。
  - 增加 terminal proximity 是为了防止仅靠 path projection 误把远离终点的点判为完成。
- `src/cooperative_viewpoint_manager.cpp`
  - 所有安全 hypothesis 均可更新/发布，不再只让 strict layout拥有长期 Target资格；不重排已由 generator 给出的 visibility-first pool。

### 2.2 local planning 与真正 NOMINAL

- `plan_manage/include/plan_manage/local_execution_contract.h`
  - strict geometry退化改为 diagnostic；`geometryRecoveryDue()` 不再把它变成执行 replan authority。
- `plan_manage/src/planner_manager.cpp`
  - 统一本地 visibility metrics/comparator；保留全部 hard final checker。
  - NOMINAL 不再被未提交 strict recovery seed塑形；runtime日志明确记录 geometry不具有 execution authority。
- `traj_opt/src/poly_traj_optimizer.cpp`
  - hypothesis 0 在无 committed team reference 时跳过 recovery-target/strict-gap shaping；其他 hypothesis 仍可把 geometry作为 soft quality。
- `traj_opt/include/optimizer/poly_traj_optimizer.h`
  - visibility诊断快照保持与上述 metric/provenance一致。

### 2.3 team selection 与 joint acceptance

- `include/multi_uav_formation/topology_coordinator_core.h`
  - team metric增加 weakest-camera 和 maximum-camera-loss。
- `src/topology_coordinator_core.cpp`
  - 排序明确为 hard safety→continuity→camera/All3→weakest/max-loss→Target/intent/Q_dir/gaps/clearance等后级 tie。
  - selection reason更新为 `HARD_SAFE_VISIBILITY_FIRST_GEOMETRY_TIE`。
- `include/multi_uav_formation/team_visibility_optimizer.h`
- `src/team_visibility_optimizer.cpp`
  - acceptance加入 binary NONE/All3 和 visibility-continuity contract；trial/final不再 hard require strict encirclement。
- `src/multi_uav_topology_coordinator.cpp`
  - live replacement删除 strict/relative encirclement的独立 hard admission；ACK/commit协议本身未改。
- `plan_manage/src/team_target_reachability.cpp`
  - recovery candidate仍必须通过完整 P/V/A、dynamics/static/dynamic/swarm和team safety；strict geometry不再是 final reject或completion条件。

### 2.4 测试与离线分析

更新：

```text
multi_uav_formation/test/adaptive_viewpoint_generator_contract_test.cpp
multi_uav_formation/test/persistent_recovery_target_contract_test.cpp
multi_uav_formation/test/encirclement_geometry_contract_test.cpp
multi_uav_formation/test/topology_coordinator_contract_test.cpp
plan_manage/test/local_execution_contract_test.cpp
multi_uav_formation/test/adaptive_execution_contract_test.py
plan_manage/test/trajectory_lifecycle_wiring_test.py
```

本轮证据分析：

```text
visibility_first_geometry_tie_20260913/analyze_visibility_first.py
visibility_first_geometry_tie_20260913/analysis/run1_metrics.json
visibility_first_geometry_tie_20260913/analysis/two_stage_audit.json
visibility_first_geometry_tie_20260913/analysis/visibility_first_audit.json
```

仿真后只修正了离线 analyzer 的一个 accounting bug：pairwise activation-gap统计无法识别“最后一条 trajectory后再也没有 next activation”的右删失事件。现在把 executor 的显式 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` identity并入 episode集合；production源码和本次实验版本没有因此改变。

## 3. Case A–F 合同

| Case | 覆盖 | 结果 |
|---|---|---|
| A 非严格高可见 vs strict低All3 | adaptive Target、team selector和local comparator均允许 `g_max>170°` 的高K2/All3候选击败strict低visibility候选 | PASS |
| B visibility相当、geometry更好 | camera/K2/All3相同后，Q_dir/encirclement更好的候选获胜 | PASS |
| C visibility risk→SIDE→A*→SFC→MINCO/SCP/preflight | 复用受控原场景、镜像和simple-side-static-fail/A* rescue production test | PASS |
| D geometry improvement伴随visibility regression | local/team/joint acceptance拒绝用geometry improvement覆盖camera-time/All3明显下降 | PASS |
| E 非strict长期健康执行 | Target可用、identity/freshness、guide reuse、non-strict execution和guide-based completion均不触发强制strict recovery | PASS |
| F visibility不降且geometry更好 | 后级geometry/Q_dir tie-break及真实joint soft recovery仍可选择、3-ACK commit | PASS |

Case E 还额外覆盖：仅有 guide path projection、但实际点远离 guide terminal 时不能错误完成 Target。

## 4. Build 与测试

构建：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

结果：6 个请求/依赖包全部成功，0 failure。唯一编译 warning 是既有未使用变量 `base_delta_phi`。

17 个相关 C++ unit/gradient/contract/production-chain测试全部 PASS：

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

这些测试包含现有 analytic/finite-difference P、tau、piece boundary、yaw、J_vis deep-risk、Elastic、diversity/gap gradient和本轮 visibility-first comparator contracts。

Python contracts：

```text
adaptive_execution_contract_test.py       = 12 passed
trajectory_lifecycle_wiring_test.py       = 15 passed
git diff --check                          = PASS
analysis scripts py_compile               = PASS
```

## 5. 唯一一次 Scenario A FULL ON

证据：

```text
visibility_first_geometry_tie_20260913/run1
scene SHA256 = 430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
mode = FULL_ON + native RViz
target route = normally completed
route duration = 80.204541 s
exit code = 0
```

Stage2/3A/3B、visibility ranking、K-of-N、team visibility optimizer、joint P/T、joint yaw、predicted-attitude FOV、topology coordination、cooperative viewpoint、shared risk、SIDE hard-corridor SCP、Elastic 和既有 deep-risk J_vis 全部保持 ON。未运行 OFF、未做 A/B、未启动第二次仿真。

### 5.1 可见性

| 指标 | 本轮唯一 run | Feedback53 | 历史 OFF参考 |
|---|---:|---:|---:|
| ACCUMULATED_CAMERA_VISIBLE_TIME | 203.502210 camera-s | 228.612786 | 约233（旧严格表为227.317129） |
| MEAN_VISIBLE | 2.538308 | 2.851552 | 2.905279 |
| K2 | 0.981245 | 0.994232 | 0.982248 |
| ALL3 | 0.557064 | 0.857320 | 0.923316 |
| NONE | 0 | 0 | 约0 |
| LONGEST_K2_LOSS | 0.333995 s | 0.265703 s | 0.877114 s |
| LONGEST_BLACKOUT | 0 s | 0 s | 0.022395 s |

本轮数值显著低于 Feedback53，且 camera-time/All3低于历史 OFF；K2与历史 OFF几乎相同但略低约0.0010。关键是这些 aggregate被 UAV0 后约30.6 s terminal hold严重污染，不能用来否定或证明 visibility-first排序本身的收益。

逐机失视累计：

| UAV | static LOS | dynamic LOS | HFOV | VFOV | range |
|---:|---:|---:|---:|---:|---:|
| 0 | 3.698808 s | 10.563168 s | 22.236284 s | 0.864760 s | 22.205325 s |
| 1 | 1.971698 s | 0.634088 s | 0 | 1.070330 s | 0 |
| 2 | 2.841885 s | 1.333830 s | 0 | 0.168457 s | 0 |
| total camera-seconds | 8.512391 | 12.531086 | 22.236284 | 2.103547 | 22.205325 |

`All3:1→0 且 K2=1` 共18个事件，累计19.748016 s：FOV 8、static LOS 7、static+FOV 1、dynamic LOS 1、FOV+range 1。最后一个事件持续9.103 s、失视者为UAV0，已处于 terminal-hold扩散阶段；不能把它算成正常 selector质量失败。

### 5.2 multi-view、topology 与 joint闭环

```text
ACTUAL_Q_DIR_MEAN                  = 0.932502
HIGH_QUALITY_DIRECTIONAL_RATIO    = 0.863194
HIGH_QUALITY_MULTI2_RATIO         = 0.844442
HIGH_QUALITY_MULTI3_RATIO         = 0.483975
EXECUTED_ENCIRCLEMENT_RATIO       = 0.320259
SAME_SEMICIRCLE_RATIO             = 0.573145

g_min deg mean/p50/p95/min/max    = 65.783/66.429/99.041/1.347/117.128
g_max deg mean/p50/p95/min/max    = 184.150/182.491/236.175/122.287/318.323
```

Q_dir仍显示明显多方向性，但 strict encirclement和Multi3低于Feedback53；由于同一次 run 存在一机长hold，这些数字不能作为纯粹几何层级变更的稳定因果结果。

```text
visibility-triggered topology      = 710
NOMINAL/LEFT/RIGHT attempts        = 3211 / 520 / 490
safe NOMINAL/LEFT/RIGHT            = 500 / 463 / 452
actual NOMINAL/LEFT/RIGHT          = 499 / 64 / 84
LEFT/RIGHT A* attempts             = 94 / 126
LEFT/RIGHT A* successes            = 83 / 112

TEAM_VIS_OPT_ATTEMPT / SUCCESS     = 60 / 32
proposal / THREE_ACK / commit      = 27 / 27 / 27
3-UAV actual adoption              = 27
optimized yaw actual execution     = 27 solution IDs
joint planning ms mean/p50/p95/max = 2.509 / 2.946 / 7.090 / 13.911
```

因此现有 observation topology、A*/Local-SFC/MINCO/SCP 和完整 optimizer→proposal→3ACK→commit→adoption→optimized-yaw execution 链均没有被本轮语义修改切断。

### 5.3 two-stage coverage/quality

```text
FIRST_SAFE_SUPPLY_HELD                      = 25
FIRST_SAFE_RESERVED                         = 60
QUALITY_SEARCH_AFTER_FIRST_SAFE             = 93
BETTER_CANDIDATE_FOUND_AFTER_FIRST_SAFE     = 11
BETTER_CANDIDATE_SELECTED_AFTER_FIRST_SAFE  = 8
BETTER_CANDIDATE_ACTIVATED                  = 8
QUALITY_SEARCH_ABORTED_FOR_DEADLINE          = 0
TEAM_ENHANCEMENT_AFTER_LOCAL_RESERVE        = 59
TEAM_ENHANCEMENT_DEFERRED_LOCAL_COVERAGE     = 1
```

说明 Feedback53 的 first-safe reserve 与剩余预算质量搜索仍实际工作。后续 terminal hold不是因为本轮回退了two-stage逻辑，而是 active predecessor deadline已错过后，mandatory supply仍与过期deadline绑定的liveness缺口。

### 5.4 continuity与Safety

```text
TERMINAL_HOLD_COUNT                         = 1
MOVING_SUCCESSOR_STARVATION_COUNT           = 1
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION       = 1
UNVALIDATED_EXECUTED_SAMPLES                = 910
VALIDATED_COVERAGE_DEADLINE_MISSED          = 1 unique episode / 156 throttled logs
NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE     = 1 unique episode / 5922 repeated logs
READY_TOO_LATE                              = 0

COLLISION_SAMPLES                           = 0
SWARM_CLEARANCE_VIOLATION_SAMPLES           = 0
MIN_STATIC_CLEARANCE                        = 0.245097 m
MIN_DYNAMIC_CLEARANCE                       = 0.529428 m
MIN_SWARM_DISTANCE                          = 0.832048 m
```

没有碰撞或swarm violation样本，但一机进入未验证terminal hold，因此 `SAFETY_AND_CONTINUITY_PRESERVED=NO`。不能用 collision=0 掩盖 liveness/validation失败。

## 6. strict / non-strict 条件统计

使用同一world time下的三机trajectory geometry与最近visibility sample，将执行时间按真实 `g_min>=25° && g_max<=170°` 划分：

| 条件 | duration | camera-rate / mean | camera-time | K2 | All3 | static loss | dynamic loss | FOV loss | Q_dir |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| STRICT_ENCIRCLED | 25.675757 s | 2.597506 | 66.692944 | 0.980227 | 0.617279 | 1.865521 | 1.499016 | 8.468807 | 0.990967 |
| NOT_STRICT_ENCIRCLED | 54.496093 s | 2.510545 | 136.814904 | 0.981730 | 0.528816 | 6.645645 | 11.031192 | 15.868271 | 0.904913 |

按用户要求直接输出：

```text
ALL3_WHEN_ENCIRCLED          = 0.617279
ALL3_WHEN_NOT_ENCIRCLED      = 0.528816
K2_WHEN_ENCIRCLED            = 0.980227
K2_WHEN_NOT_ENCIRCLED        = 0.981730
CAMERA_RATE_WHEN_ENCIRCLED   = 2.597506
CAMERA_RATE_WHEN_NOT_ENCIRCLED = 2.510545
```

这只是条件相关，不是“strict geometry导致visibility”的因果证据。尤其 terminal hold在两组中分布不均。为揭示该混杂，hold前的条件值为：

| hold前 | K2 | All3 | camera-rate | Q_dir |
|---|---:|---:|---:|---:|
| strict | 1.000000 | 0.867360 | 2.867360 | 0.989238 |
| non-strict | 0.985198 | 0.732892 | 2.718089 | 0.899961 |

hold后，strict子段 All3=0、non-strict子段 All3=0.247225。它再次说明本次单run主要受execution failure支配，不能从整段平均推导 strict threshold应该重新升级为 hard gate。

## 7. terminal hold 的第一个代码级断点

### 7.1 物理候选首先被正确的dynamic safety拒绝

UAV0最后一条有效运动轨迹：

```text
trajectory_id                 = 177
source                        = PERSISTENCE_FALLBACK
activation                    = 1789240319.971170664
duration                      = 1.675464 s
validated_end                 = 1789240321.646634340
terminal hold entered         = 1789240321.654433
```

规划在 `1789240320.142648` 已开始，validated coverage约1.504 s、planning budget约1.404 s。此前 suffix仍是 absolute-safe，其 min dynamic distance约1.168 m，高于production门槛1.1 m。

随后cycle 133–237 共105个unique attempts中，NOMINAL新候选全部在 `EGOPlannerManager::reboundReplan()` 的 `trial_dynamic_valid`/最终dynamic preflight失败：

```text
first observed candidate min distance = 0.928051 m
window minimum                       = 0.349293 m
last pre-deadline value              = 0.520834 m
required moving clearance            = 1.100000 m
```

LEFT/RIGHT确实被打开。该窗口中side拒绝主要为：

```text
DYNAMIC_SAFETY_CLEARANCE              = 100
INSUFFICIENT_DYNAMIC_IMPROVEMENT      = 42
other/static/not-applicable           = 60
```

部分SIDE已经完成MINCO/SCP并记录 `SCP_FINAL_OK`，但final dynamic clearance仍只有约0.65–0.82 m，所以保持 `available=0 dynamic_ok=0`。`finalizeCapturedCandidates()` 有88个 `safe=0 reason=DYNAMIC_FAIL`，没有一条 current-revision final preflight通过。

这一层的拒绝是正确hard safety行为，不能通过降低1.1 m阈值或执行raw A*绕过。A*是static same-topology repair；这里简单side在static层可行、真正失败的是dynamic clearance，因此窗口内 `side-astar-repair=0` 也不是A*入口丢失。

### 7.2 deadline错过后的持久liveness trap

第一个deadline miss：

```text
now / activation-earliest     = 1789240321.549655 / 1789240321.649655
validated_end                 = 1789240321.646634
physical suffix remaining     = 0.096979 s
required activation lead      = 0.100000 s
planning_budget_remaining     = -0.003021 s
```

此时即使还有约97 ms位置suffix，也已经无法满足100 ms activation lead。`EGOPlannerManager::ensureExecutionCoverage()` 保持“不能把deadline延长到validated predecessor之后”的正确原则，但随后执行：

```text
fast_budget = max(0, min(estimate, negative planning_budget_remaining)) = 0
planning_deadline_wall = current wall time
```

接着 `optionalRefinementAllowed()` 为false；`ego_replan_fsm.cpp` 的candidate/hypothesis loops因此在生成mandatory first-safe supply前退出。日志从此为 `candidate_seen=0`。旧 predecessor deadline始终在过去，每一轮又把wall budget重置为0，于是系统无法从actual odom/held state启动一条新的、完整hard-gated recovery trajectory，最后traj_server进入terminal hold并一直保持ID 177。

因此应区分：

```text
FIRST_PHYSICAL_CANDIDATE_FAILURE_FUNCTION
  = EGOPlannerManager::reboundReplan
FIRST_PHYSICAL_FAILURE_CONDITION
  = trial_dynamic_valid / final dynamic clearance < 1.1 m

FIRST_PERSISTENT_CODE_FAILURE_FUNCTION
  = EGOPlannerManager::ensureExecutionCoverage
FIRST_PERSISTENT_FAILURE_CONDITION
  = expired predecessor deadline -> zero wall budget;
    optionalRefinementAllowed suppresses mandatory supply
```

planner“仍在正常循环”但executor没有validated moving successor，正是因为循环继续运行、旧suffix也继续被revalidate/publish（101次），然而 deadline authority已经不允许任何新candidate完成。telemetry中的大量 warning 是同一个 episode的重复观测，不是5922个独立failure。

### 7.3 为什么旧starvation指标可能报0

旧离线指标只在相邻两次activation之间找gap。最后一次activation后永远没有“下一次activation”，因此这个右删失episode没有pair可计算，会被漏报为0。本轮仿真后只修了离线accounting：将executor显式的 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` identity并入episode集合。修正后：

```text
MOVING_SUCCESSOR_STARVATION_COUNT = 1
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT = 1
```

production lifecycle没有在实验后被偷偷改变。

### 7.4 推荐的后续lifecycle修复

应把：

- optional quality deadline；
- predecessor仍可handoff时的normal successor deadline；
- predecessor已过期后的mandatory recovery budget

分成不同authority。过期后不能延长或伪造旧validated suffix，但也不能继续让过期predecessor把mandatory supply预算永久压成0；应从actual odom/held state重新生成一条运动candidate，并重新通过完整dynamics/static/dynamic/swarm/handoff/final preflight后才激活。

这不是恢复planner brake/stop fallback，也不允许放宽Safety。它需要独立的lifecycle设计、contracts和真实验证；本轮在唯一实验结束后只做诊断，没有继续大改或启动第二次仿真。

## 8. 最终字段

```text
VISIBILITY_FIRST_TASK_SEMANTICS_IMPLEMENTED: YES

STRICT_25_170_STILL_HARD_TARGET_GATE: NO
STRICT_25_170_STILL_HARD_EXECUTION_GATE: NO
STRICT_25_170_RETAINED_AS_DIAGNOSTIC: YES

NON_ENCIRCLED_HIGH_VISIBILITY_TARGET_ALLOWED: YES
VISIBILITY_FIRST_NOMINAL_RETAINED: YES

SHARED_VISIBILITY_TRIGGER_REUSED: YES
SIDE_OBSERVATION_TOPOLOGY_REUSED: YES
ASTAR_SAME_TOPOLOGY_REPAIR_REUSED: YES
LOCAL_SFC_REUSED: YES
MINCO_SCP_REUSED: YES
FINAL_HARD_PREFLIGHT_REUSED: YES

J_ENC_CAN_OVERRIDE_MAJOR_VISIBILITY_GAIN: NO
Q_DIR_CAN_OVERRIDE_MAJOR_VISIBILITY_GAIN: NO

TIME_BUDGET_CLOSED_LOOP_PRESERVED: YES
TWO_STAGE_COVERAGE_QUALITY_PRESERVED: YES

FULL_ON_RUN_COUNT: 1

CAMERA_TIME: 203.502210 camera-s
MEAN_VISIBLE: 2.538308
K2: 0.981245
ALL3: 0.557064
NONE: 0

ACTUAL_Q_DIR_MEAN: 0.932502
HIGH_QUALITY_MULTI2_RATIO: 0.844442
HIGH_QUALITY_MULTI3_RATIO: 0.483975
EXECUTED_ENCIRCLEMENT_RATIO: 0.320259
SAME_SEMICIRCLE_RATIO: 0.573145

ALL3_WHEN_ENCIRCLED: 0.617279
ALL3_WHEN_NOT_ENCIRCLED: 0.528816
K2_WHEN_ENCIRCLED: 0.980227
K2_WHEN_NOT_ENCIRCLED: 0.981730
CAMERA_RATE_WHEN_ENCIRCLED: 2.597506
CAMERA_RATE_WHEN_NOT_ENCIRCLED: 2.510545

TERMINAL_HOLD_COUNT: 1
MOVING_SUCCESSOR_STARVATION_COUNT: 1
UNVALIDATED_EXECUTED_SAMPLES: 910
COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0

VISIBILITY_VS_FEEDBACK53: WORSE (terminal-hold-confounded; not a clean semantic comparison)
VISIBILITY_VS_HISTORICAL_OFF: WORSE (K2 approximately tied, camera-time/All3 lower)
MULTIDIRECTIONAL_QUALITY_PRESERVED: PARTIAL (Q_dir high; strict encirclement/Multi3 lower)
SAFETY_AND_CONTINUITY_PRESERVED: NO

FINAL_ASSESSMENT: PARTIAL

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

最终 production 语义已成为：现有绕行、优化和硬约束链继续回答“能不能安全飞”，visibility continuity与有效相机时间回答“值不值得飞”，Q_dir/25°/170°仅在前两者质量相当时回答“哪个多方向结构更漂亮”。唯一真实运行未通过的原因不是strict geometry被降级，而是另一个明确的post-deadline lifecycle liveness trap。

