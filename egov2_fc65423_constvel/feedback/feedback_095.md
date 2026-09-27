# Feedback 095 — Predictive Visibility Relay + Extra Team Spatiotemporal SCP

日期：2026-09-22  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
审计基线：`AUDIT_PREDICTIVE_VISIBILITY_RELAY_20260922.md`  
最终场景：`long_cylinder_forest_visibility_stress.json`

## 0. 结论

本轮完成了 production 代码中的 Predictive Visibility Relay 与额外 Team
Spatiotemporal SCP 接入：Local planner 仍先独立生成并 hard-preflight 一条可执行未来
轨迹；Team 只在预测到第二观察通道 `M2` 下降时创建跨 rolling cycle 的 handoff
contract，并把 T-only 或 P/T SCP 作为不改变 Local authority 的 speculative refinement。

恢复的旧 `runCandidateHardCorridorSCP()` 没有被重新打开为全局 SIDE policy。FULL 模式中：

```text
OLD candidate hard-corridor SCP = OFF
NEW team spatiotemporal SCP      = ON
```

T-only 与 P/T 均已实现、均可由 production contract 入口到达。较早的真实 production
分支运行中，T-only 有 2 次 PASS，P/T 有 1 次 PASS；均完成 exact MINCO regeneration、
nonlinear margin check 和 Local hard preflight。最终源码的正式压力运行中创建了 4 个持久
contract，触发 7 次 Team-SCP：5 次因没有可用连续线性化而 fail closed，2 次进入 P/T
后由 OSQP 与去除 P/T trust 的诊断共同确认当前约束集真实不可行。4 个 contract 均在 Local
轨迹自身恢复至 release margin 后释放，没有 relay refinement 形成三机 realized handoff。

因此，代码与 production reachability 已闭合，但运行级 make-before-break handoff 尚未闭合，
不能写成完整验收通过。最终 K2 为 1.0、零 hold、零执行碰撞，也不能反推 relay 成功，因为
本次最终运行没有 contract-specific Team refinement 被三机共同采用。

```text
FINAL_ASSESSMENT: PARTIAL
PRODUCTION_IMPLEMENTATION: COMPLETE
PRODUCTION_REACHABILITY: YES
FINAL_RUN_HANDOFF_COMPLETION: NO
NEW_GLOBAL_CANDIDATE_SCP_ENABLED: NO
```

## 1. Build status after SCP restoration

开始时先核对当前工作树中人工恢复的 P/T 数值骨架，没有从 tar、旧 commit 或旧报告覆盖
当前文件。恢复代码适配当前树后能正常构建；实现 Team 入口、SIDE certificate 与最终
`+inf` LOS 语义修正后，最终执行：

```text
env CMAKE_PREFIX_PATH=/opt/ros/noetic \
  catkin build traj_opt ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws
```

结果：6 个涉及包及依赖全部 PASS，最终无编译 warning；`git diff --check` PASS。

最后一个直接修正是 producer/consumer 语义对齐：共享圆柱与 LOS 线段不存在垂直重叠时，
现有几何函数返回 `valid=true, clearance=+inf`，表示该动态障碍对该 sample 无约束。coordinator
本来已正确跳过，Team-SCP 却把非有限 clearance 误判为 linearization invalid。现在 Team-SCP
对这个有效的 no-overlap case 直接 `continue`，没有伪造梯度，也没有放松任何有限 clearance。

本轮主要 production 文件：

```text
ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  tracking_visibility_geometry.h
  realized_team_validator.h

ros_ws/src/multi_uav_formation/src/
  multi_uav_topology_coordinator.cpp
  realized_team_validator.cpp

ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  traj_opt/include/optimizer/poly_traj_optimizer.h
  traj_opt/src/poly_traj_optimizer.cpp
  plan_manage/include/plan_manage/planner_manager.h
  plan_manage/src/planner_manager.cpp
  traj_utils/msg/TeamReferenceSchedule.msg
  traj_utils/msg/TeamTrajectoryAck.msg

  plan_manage/launch/advanced_param.xml
  plan_manage/launch/run_in_sim.launch

ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch
```

## 2. Final production architecture

最终实际控制链是：

```text
authoritative odom / target snapshot / map / dynamic prediction
  -> 原 Local N/L/R -> A* -> Local-SFC -> MINCO/LBFGS
  -> existing Local feasibility/SCP -> current-revision hard preflight
  -> Local guaranteed successor（authority 不等待 Team）

三条 ACTIVE / Local guaranteed future trajectories
  -> fixed-world-time visibility margins
  -> M2 = second-largest(m0,m1,m2)
  -> imminent M2 loss
  -> persistent HandoffContract
  -> speculative TEAM_T_ONLY 或 TEAM_PT
  -> exact MINCO regeneration
  -> nonlinear visibility/topology/physics validation
  -> existing RealizedTeamValidator
  -> PREPARE -> 3 ready -> atomic commit -> common activation
```

Team 失败时只丢弃 speculative result。它不会清 Local successor、不会改 authoritative
predecessor、不会等待 Team 才提交 first-safe，也不会把 reference phase 变成时间 authority。

## 3. Restored P/T SCP infrastructure reuse

新入口为 `runTeamContractSCP()`，复用了恢复后的单套 OSQP/MINCO 基础设施：

- `[vec(Delta P); Delta virtual_T]` decision layout；
- `MinJerkOpt` 的 P/T parameterization 与 `realTimeJacobianAt()`；
- fixed world-time sample 重新定位 piece/local time 的 Jacobian；
- QP sparse matrix、OSQP、P trust、physical T trust 与 adaptive trust diagnostics；
- Local-SFC、native static、dynamic body、v/a/j rows；
- exact `MinJerkOpt::generate()`、model agreement、nonlinear postcheck。

没有建立第二套 OSQP wrapper，也没有把旧 candidate hard-SCP acceptance policy当 Team policy。
普通 candidate 仍走原 native Local optimizer；Team SCP 只在 active handoff contract 下执行。

## 4. Unified visibility margin / M2

`tracking_visibility_geometry.h` 统一提供同一套 signed margin 语义：

```text
m_i(t) = min(static LOS, dynamic LOS, FOV, range)
M2(t)  = second-largest(m_0(t), m_1(t), m_2(t))
```

每个 sample 保留 limiter：`STATIC / DYNAMIC_LOS / FOV / RANGE`。只有实际提供连续位置梯度的
component 才进入 Team soft gradient 与 SCP hard row；binary/discrete 或当前点数值奇异的
component只参与 trigger、exact nonlinear validation 与 RealizedTeamValidator，不伪造解析梯度。

baseline authority 优先使用 ACTIVE 和 Local guaranteed successor，不使用未验证 speculative
Team future。Team requirement 定义在绝对 `t_world`；T 变化后重新进行
`world time -> piece index -> local piece time`，不是冻结 piece phase，也不是改 waypoint
timestamp。

最终运行产生 189 个 `TEAM_M2_MIN` telemetry marker，其中 185 条未受并发日志交织、可完整
解析；7 次低于 trigger `0.3`，可解析全局最小值为 `-3.070256`。这些事件创建了 4 个 contract。

## 5. Handoff Contract

coordinator 中的 `PersistentHandoffContract` 保存：contract identity、task/world/target/dynamic/
static/visibility revision、三机 baseline trajectory/generation/lineage、outgoing/incoming/stable
UAV、acquire/preserve world time、required overlap/margin、created/expire time及当前 topology。
contract 跨 rolling callback 复用，lineage freshness 与 revision 不一致时 fail closed。

最终运行的 4 个 contract：

| ID | outgoing -> incoming / stable | acquire | preserve | predicted M2 min | 结束 |
|---:|---|---:|---:|---:|---|
| 1 | 1 -> 2 / 0 | 1790022773.226559 | 1790022773.726559 | -1.123369 | M2=0.483159，自然 release |
| 2 | 2 -> 0 / 1 | 1790022773.372658 | 1790022773.872658 | 0.250235 | M2=0.580788，自然 release |
| 3 | 1 -> 2 / 0 | 1790022782.064616 | 1790022782.564616 | -0.324079 | M2=1.0，自然 release |
| 4 | 1 -> 2 / 0 | 1790022804.648787 | 1790022805.148787 | -2.741220 | M2=1.0，自然 release |

统计：`CREATE=4`、`REUSE=3`、`RELEASE=4`、`COMPLETE=0`、`STALE=0`。make-before-break 的
计划与 realized overlap 验证已实现，但最终运行没有 contract-specific 三机 realization，
因此 realized overlap 为不可评价，而不是把 `0` 写成一次成功 overlap。

## 6. TEAM_T_ONLY

T-only 模式严格固定 P，只优化 `Delta virtual_T`。QP 仍保留 visibility、v/a/j、positive
duration、physical T trust 与必要的 time/swarm条件；trial 必须 exact regenerate 后再验收。

最终正式运行：T-only trigger/pass/fail = `0/0/0`，因为两次可构造 continuous rows 的
deficit 均明显超过 T gain bound，直接选择 P/T。

较早的同一 production 入口分支运行 `runs/20260922_035534_752273` 提供了实际 T-only
执行证据：

| contract/UAV | `||Delta P||` | `||Delta tau||` | `||Delta T||` | linear/nonlinear margin | v/a/j before -> after |
|---|---:|---:|---:|---:|---|
| 1 / UAV2 | 0 | 0.213984 | 0.253797 | 1.919208 / 1.0 | 1.384907/1.315639/4.929129 -> 1.264048/1.295059/4.933192 |
| 2 / UAV1 | 0 | 0.122281 | 0.103433 | 1.138280 / 1.0 | 1.179833/1.428541/6.561245 -> 1.107981/1.375054/6.356547 |

两次均 `Delta P=0`，证明它是实际 T subspace，不是给 P 一个很小 trust box。

## 7. TEAM_PT

P/T decision vector 为：

```text
x = [vec(Delta P); Delta virtual_T]
```

目标以最小必要 deformation 为主，Team soft linear term只帮助朝 margin 增大方向投影；hard
guarantee仍由 linearized rows、exact regeneration、nonlinear validation 和 realized validator
共同给出。

TEAM_PT 保留：

- Team visibility hard rows；
- Local-SFC/native static/dynamic body；
- velocity/acceleration/jerk；
- swarm authoritative nonlinear check；
- P trust、physical T trust、positive duration；
- revision/generation/lineage freshness；
- zero-plane SIDE hard topology rows。

zero-plane SIDE 没有另造 topology。Planner 把本次实际 `CandidateSideScope` 的 origin、normal、
sign、offset、active interval 和 topology identity作为 exact certificate放入 ACK；coordinator与
RealizedTeamValidator消费同一证据。这样 `plane_count==0` 时 P/T 也不能从 LEFT 穿到 RIGHT，
有 SFC 时则与 SFC 同时成立。nonlinear final check再次按实际 polynomial 验证 side sign。

最终运行进入 P/T 2 次，均为 contract 2：UAV0 deficit `4.132060`、T gain bound
`0.186583`；UAV2 deficit `1.565991`、T gain bound `0.138946`。两者在原 trust、去 P trust、
去 T trust、同时去 P/T trust的诊断中均为 primal infeasible，分类为
`TRUE_CONSTRAINT_INFEASIBILITY`，speculative result被丢弃，Local 不变。

较早 production 分支运行曾取得一次 P/T PASS：

```text
contract/UAV                     = 1 / UAV0
||Delta P||                      = 0.230460193
||Delta tau||                    = 0.278763598
||Delta T||                      = 0.430765076
linear margin / nonlinear margin = 0.983037986 / 1.000000000
model error                      = 0.016962014
max v                            = 1.381198 -> 1.246220
max a                            = 1.171105 -> 0.886082
max jerk                         = 4.547185 -> 2.821483
```

这证明 P/T 联合变量、exact regeneration 与 nonlinear acceptance真实可达；但该次三机中
另两机失败，所以仍未形成 completed handoff。

## 8. T vs PT decision rule

对 critical fixed-world-time samples计算：

```text
deficit      = required_margin - min(nominal margins)
T_gain_bound = sum_j |dm/dT_j| * DeltaT_trust_j
```

只有 T-gradient norm超过数值阈值、T trust仍有空间，且
`deficit <= team_scp_t_gain_ratio * T_gain_bound` 时选择 T-only；否则直接 P/T。FULL 参数的
gain ratio 为 `0.8`。T-only若失败，不宣告 topology infeasible；实现会从原 Local seed进入
P/T retry。只有 P/T 在当前 side/SFC/safety/Team margin约束共同作用下确认为真不可行时，
才允许返回当前 topology infeasible语义。

## 9. RealizedTeamValidator

没有创建第二套 validator。现有 `RealizedTeamValidator` 现在使用三条最终 realized
polynomial在统一 world time复查：unified margin、M2、incoming acquire、outgoing preserve、
overlap、exact SIDE certificate、existing safety以及 transaction freshness。

最终运行有 122 个 validator marker；并发 ROS 输出交织导致其中 2 行的 decision token被别的
日志截断，完整可解析结果为 67 PASS、53 REJECT，其中 6 个是 exact side certificate拒绝。
这些大部分属于普通 committed-prefix Team reference，并非 handoff contract。最终没有
contract refinement组成三机 ready，因此 handoff-specific realized overlap分支没有得到
一次完整运行证明。

SIDE certificate修复后，普通 Team reference已有大量 `side_ok=1` 的 realized PASS，说明
consumer不再根据历史 joint seed猜测 SIDE；同时仍有6次真正不满足 exact side geometry的
候选被fail closed。

## 10. Lifecycle safety

最终运行证明 Team失败没有破坏 Local liveness：

```text
TERMINAL_HOLD_ENTER                         = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION       = 0
MOVING_SUCCESSOR_STARVATION                 = 0
POST_DEADLINE_RECOVERY_ENTER                = 0
UNVALIDATED_EXECUTED_SAMPLES                = 0
PREDECESSOR_MISMATCH                        = 0
PARTIAL_TEAM_ACTIVATION                     = 0
```

普通 Team reference链有67次三机 ready。日志中可识别56个 unique transaction commit ID；
49个 unique ID最终各出现严格3次 activation，共147条 activation event；另外7个 unique ID
在 activation前因 current-revision final preflight失败被原子 revoke，三份 speculative copy
全部取消并记录 `local_authoritative_predecessor_unchanged=1`。没有单机先激活的 partial
activation。

这部分不是 relay成功证据，但证明新 contract/SCP失败路径没有清空 Local guaranteed
successor，也没有破坏已有 executor identity与原子提交边界。

## 11. Parameters

最终 FULL 运行实际解析参数：

| 参数 | 值 |
|---|---:|
| `enable_team_spatiotemporal_scp` | true（三机） |
| `enable_candidate_hard_corridor_scp` | false（三机） |
| predictive relay horizon | 2.0 s |
| relay sample dt | 0.1 s |
| trigger / required / release margin | 0.30 / 0.20 / 0.45 |
| overlap / stable time | 0.50 / 0.20 s |
| contract timeout | 1.0 s |
| P trust | 0.30 |
| T trust ratio | 0.15 |
| max SCP iterations | 6 |
| Team soft weight | 10.0 |
| T gain ratio | 0.8 |

三机启动日志均明确打印：

```text
[TEAM_SPATIOTEMPORAL_SCP_ENABLED]
value=1 candidate_hard_policy=0
```

active handoff path固定打印 `TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF=0`；handoff timing authority
只有 world-time contract与物理 MINCO duration T。

## 12. Production run

### 12.1 最终权威运行

```text
RUN_DIR:
/home/bob/ALP/egov2_fc65423_constvel/runs/20260922_043207_778930

scenario      = long_cylinder_forest_visibility_stress.json
ablation mode = FULL
headless      = YES
BOOT          = BOOT-12
CORE_EXIT     = 0
CLEANUP       = OK
```

最终 relay/SCP统计：

```text
TEAM_M2_MIN markers                 = 189
parseable M2 below trigger          = 7
parseable global M2 minimum         = -3.070256

HANDOFF CREATE / REUSE / RELEASE    = 4 / 3 / 4
HANDOFF COMPLETE / STALE            = 0 / 0

TEAM_SCP_START                      = 7
TEAM_T_ONLY trigger/pass/fail       = 0 / 0 / 0
TEAM_PT trigger/pass/fail           = 2 / 0 / 2
pre-mode linearization fail         = 5
CURRENT_TOPOLOGY_TEAM_INFEASIBLE    = 2 true P/T infeasible results

contract-specific realized overlap = NOT_REACHED
```

最终运行没有 accepted Team-SCP step，所以最终 run的 accepted `Delta P/Delta T` distribution
为空；两个 P/T failure均保持 `Delta P=Delta tau=Delta T=0`。这不是把零步长冒充优化结果。

用于证明已实现分支的较早真实 run为：

```text
/home/bob/ALP/egov2_fc65423_constvel/runs/20260922_035534_752273
```

其3个 accepted SCP样本的分布：

```text
||Delta P||     min/median/max = 0 / 0 / 0.230460193
||Delta tau||   min/median/max = 0.122280617 / 0.213984051 / 0.278763598
||Delta T||     min/median/max = 0.103432596 / 0.253796926 / 0.430765076
|linear-nonlinear margin error|
                 min/median/max = 0.016962014 / 0.138280253 / 0.919208211
```

T样本中的最大 model error较大，但 exact nonlinear margin仍为1.0，所以没有把 linear model
当最终证明。三次 accepted样本中，max velocity与max acceleration均下降；jerk有两次下降，
一次从4.929129轻微升到4.933192。没有出现旧“所有 SIDE常态 hard-SCP”造成的系统性
jerk恶化。

### 12.2 最终可见性

任务统计窗口 `81.128181 s`、2434组同步样本：

```text
CAMERA_TIME       = 235.351720 camera.s
MEAN_VISIBLE      = 2.900986031
K2                = 1.000000000
ALL3              = 0.900986031
VISIBLE 0/1/2/3   = 0 / 0 / 241 / 2193
LONGEST_K2_LOSS   = 0 s
LONGEST_BLACKOUT  = 0 s
UAV visibility    = 0.920296 / 1.000000 / 0.980690
```

这些是整个最终运行的实际执行指标；由于4个 relay contract都未完成，不能把它们归因为
relay refinement收益。

### 12.3 最终执行安全

从 `visibility_trajectory.csv` 的20900量级执行样本重算：

```text
UNVALIDATED_EXECUTED_SAMPLES = 0
MIN_STATIC_SURFACE_CLEARANCE = 0.204824 m
MIN_MOVING_SURFACE_CLEARANCE = 0.418486 m
MIN_UAV_TARGET_CENTER        = 0.447076 m
MIN_INTER_UAV_CENTER         = 1.001148 m
EXECUTED_OBSTACLE_COLLISION  = 0（surface clearance <= 0口径）
SWARM_<0.5M_SAMPLES          = 0
```

日志中的“collision”字样包含 planner candidate检查，不能当作实际执行碰撞；实际口径使用
同步 odom/scene surface clearance与机间距离。

### 12.4 运行过程中的直接修复

最终 run前还完成两项直接 producer/consumer修正并重建：

1. ACK携带本次真实 CandidateSideScope certificate，coordinator不再从旧 joint seed推断SIDE；
2. dynamic LOS的`valid + clearance=+inf`明确解释为垂直无重叠并跳过，不再误判无效。

未为这些问题新建 fallback/FSM，也未降低任何安全阈值。

## 13. Remaining problems

1. 最终运行5次 `TEAM_VISIBILITY_LINEARIZATION_INVALID`。`+inf` no-overlap误判已修后仍存在，
   剩余情况是当前 critical limiter处没有可用连续梯度或处于数值奇异点。当前正确行为是
   fail closed；后续应先按 component/world-time记录缺失梯度证据，不能给binary visibility
   伪造梯度。
2. contract 2的两条 P/T问题被诊断为真不可行。去掉P trust、T trust及两者后仍primal
   infeasible，说明不是简单放大trust即可解决；也不能因此放松side/SFC/safety。
3. 4个contract全部由Local未来自身恢复到release margin，没有一次形成三机 relay realized
   transaction。因此 persistent contract、T/PT、validator各自production可达，不等于完整
   make-before-break闭环已被实跑证明。
4. 普通 Team reference仍有7个 unique transaction在提交后、激活前被final preflight revoke。
   原子取消正确且无partial activation，但表明 committed-prefix quality链仍存在revision churn；
   这是已有 Team lifecycle问题，不应通过扩大relay authority处理。
5. 并发ROS stdout有少量行交织，2条validator decision token不完整。原始marker、transaction
   identity与CSV仍保留；后续如需论文统计，应让关键Team telemetry写独立结构化流，而不是
   继续增加高频ROS_INFO。

## 14. Final conclusion

本轮不是把恢复的旧 hard-SCP全局打开，而是把它收敛为 contract触发、最小修正、可回滚的
额外Team projection。T-only、P/T、fixed-world-time Jacobian、unified margin/M2、persistent
handoff contract、zero-plane hard SIDE authority、exact regeneration和现有realized validator
已经接入FULL production；Local planner与executor authority保持不变。

真实运行同时给出了两类必要证据：较早production run证明T与P/T都能产生非零修正并通过
exact nonlinear acceptance；最终源码run证明FULL默认开关、M2、contract、P/T触发和失败
回滚全部在真实压力场景工作，且零hold、零unvalidated、零partial activation。

但最终run没有completed handoff，也没有contract-specific realized overlap。故最终只能判
`PARTIAL`：实现完成、运行级relay闭环未完成。下一步应针对“连续梯度为什么缺失”和“当前
约束集为何真不可行”做直接数值诊断，不应再增加第三套planner、fallback或时间authority。

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES
FULL_SIMULATION_RUN: YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO

RESTORED_PT_SCP_BUILD_VALIDATED: YES

EXISTING_LOCAL_PLANNER_PRESERVED: YES

TEAM_M2_IMPLEMENTED: YES
HANDOFF_CONTRACT_IMPLEMENTED: YES

TEAM_T_ONLY_IMPLEMENTED: YES
TEAM_PT_IMPLEMENTED: YES

TEAM_SPATIOTEMPORAL_SCP_PRODUCTION_ENABLED: YES

ZERO_PLANE_HARD_TOPOLOGY_AUTHORITY: YES

REALIZED_HANDOFF_VALIDATION_IMPLEMENTED: YES

TEAM_T_ONLY_TRIGGERED: 0
TEAM_PT_TRIGGERED: 2

BRANCH_EVIDENCE_TEAM_T_ONLY_TRIGGERED: 2 (PASS=2, FAIL=0)
BRANCH_EVIDENCE_TEAM_PT_TRIGGERED: 9 (PASS=1, FAIL=8)

TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF: 0
HANDOFF_CONTRACT_COMPLETED: 0

RUN_DIR:
/home/bob/ALP/egov2_fc65423_constvel/runs/20260922_043207_778930

COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
SIMULATION_LEFT_RUNNING: NO
```
