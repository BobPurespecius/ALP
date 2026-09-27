# Feedback 086 — NOMINAL guide authority 直接修复与 ALP 架构减法审计

日期：2026-09-19  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
范围：Phase 1 只修 NOMINAL recovery-guide authority hole；Phase 2 冻结 production 后只读审计  

## 0. 结论

本轮没有再为 `SHORT_STATIONARY_HYPOTHESIS`、`NO_STATIC_FEASIBLE_SIDE` 或
terminal hold 发明新的 fallback。先修复的直接逻辑洞只有一个：

```text
NOMINAL 仍拥有正常 local target / visibility-first objective
但已经由 Planner 验证的 recovery guide 可以作为其数值初始化路径
```

此前代码把“不能让 recovery target 改写 NOMINAL 目标”错误扩大成“不能让 NOMINAL
使用 recovery guide 初始化”。结果是：guide 明明是唯一绕过静态障碍的安全路径，
NOMINAL 却只能退回 fresh direct chord，继而进入短时长、高 jerk、自身拒绝的自锁。

修复后真实运行中，原来发生 168.6 s 停摆的 drone0 没有复现该故障：其 guide 分支
真实进入 837 次，发布 827 条轨迹，轨迹 ID 从旧故障的 257 持续推进到 812；全程
最长 `<0.05 m/s` 低速段为 0.567 s，路径长度 175.131 m。

但整个系统仍不能判健康：同一运行中 drone2 有 130 条 `TERMINAL_HOLD_ENTER` / 
`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` 日志和 11 条 starvation 日志。它们全部发生
在另一个 UAV，并且 148 次 post-deadline recovery 都随后成功，说明本轮直接洞已修，
但当前多套 planning/recovery/authority 并存的架构仍在制造 rolling liveness 压力。

Phase 2 的核心判断是：当前复杂度不是算法本身不可避免，而是多轮修复把替代方案都
保留成了 production 路径。现在至少存在四套 endpoint/guide authority、四层 visibility
排序、三层 rolling recovery、两种互相矛盾的 team transaction 模型，以及一个复制三套
完整 Planner 的 shadow planning 进程。下一步应做删除和合并，不应继续加机制。

```text
PHASE1_DIRECT_HOLE_FIXED: YES
ORIGINAL_UAV0_168P6S_STALL_REPRODUCED: NO
PHASE1_BUILD: PASS
PHASE1_REAL_BRANCH_EXERCISED: YES
PHASE1_WHOLE_SYSTEM_HEALTH: FAIL（独立的 drone2 rolling hold）

PHASE2_PRODUCTION_CHANGED: NO
ARCHITECTURE_SUBTRACTION_AUDIT: COMPLETE
NEW_FALLBACK_OR_RECOVERY_MODULE_ADDED: NO
```

## 1. Phase 1：真实代码问题与唯一修改

### 1.1 第一个真实分叉

文件：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
plan_manage/src/planner_manager.cpp
```

原语义同时成立：

```cpp
nominal_baseline = (encirclement_hypothesis_id == 0);
recovery_guide_usable = recovery_target_.usable(...);

// guide usable 时，不允许 accepted trajectory warm start
warm_start_allowed = ... acceptedWarmStartEligible(..., recovery_guide_usable);

// 但 NOMINAL 又不允许用 guide
seed_ok = !nominal_baseline && buildRecoveryGuideSeed(...);
```

这不是 optimizer、Safety 或 static checker 的问题，而是 initializer authority 的空洞：

```text
NOMINAL + usable guide
→ accepted warm start 被 guide 正确压后
→ guide 又被 nominal_baseline 错误禁止
→ 只能构造 fresh direct chord
→ 静止/近目标采样时 duration 被压短、jerk 爆表
→ SIDE 静态不可行且 NOMINAL 自身拒绝
→ 零 successor
```

“NOMINAL 不接受 recovery target 改写目标”与“NOMINAL 不可使用已验证路径作初始化”是
两件不同的事。旧条件把 target authority 和 numerical initializer authority 错绑在一起。

### 1.2 修改

本轮 production 增量只有这一处语义变化：

```cpp
bool seed_ok = buildRecoveryGuideSeed(
    start_pt, start_vel, start_acc, object_pt, object_vel, initMJO);
```

保留不变的内容：

- hypothesis 0 仍是 NOMINAL；
- `local_target_pt`、正常 target tracking 和 visibility-first objective 不变；
- recovery target / strict gap 不重新取得目标权威；
- guide 失败后仍退回原 `buildFreshMovingInitializer()`；
- MINCO、SCP、static/dynamic/swarm/dynamics final preflight 全部不变；
- Safety clearance、NOMINAL/LEFT/RIGHT、joint、ACK/commit、executor 均未改。

没有增加 `SHORT_STATIONARY_HYPOTHESIS && NO_STATIC_FEASIBLE_SIDE` 专用状态、恢复器、
brake、stop 或新的阈值。

## 2. Phase 1：构建与真实验证

### 2.1 构建

```text
catkin build ego_planner -j2 --no-status --workspace ros_ws
```

结果：`plan_env / traj_utils / path_searching / traj_opt / ego_planner` 全部成功。
唯一 warning 是既有未使用变量 `target_state_epoch`。`git diff --check` 通过。

当前 workspace 是 `CATKIN_ENABLE_TESTING=OFF`；一个遗留的
`feedback53_regression_fixture_test` 二进制与当前库 ABI 不一致，不能把旧二进制运行结果
冒充当前源码测试。本轮没有为这一个条件新建合同或测试框架，验证 authority 是源码分支
和真实运行。

### 2.2 有效运行

```text
命令：./run_on.sh --headless --timeout 200 --pt
目录：runs/20260919_171153_221343
场景：long_cylinder_forest.json
结果：BOOT-12，CORE_EXIT_CODE=0，CLEANUP_STATUS=OK
```

一次更早启动因未带 `--pt`、与故障基线不一致，在启动日志阶段即终止并清理，目录为
`runs/20260919_171121_218497`，不计为验证运行。

运行时配置包含 FULL local visibility/encirclement/cooperative path、joint P/T；joint yaw、
Elastic、hard-corridor SCP、A* visibility cost 均按当前生产配置关闭。

### 2.3 分支和物理执行证据

drone0：

```text
nominal=1, guide_usable=1, warm_start_allowed=0       857 次
GUIDE_SEED_USED                                       837 次
planner trajectory publish                            827 次
TERMINAL_HOLD_ENTER                                     0 次
最终 trajectory_id                                    812
路径长度                                           175.131 m
最长 speed < 0.05 m/s                               0.567 s
```

首次真实分支可见：原 guide seed 初始 jerk 为 `205.63`、直接 dynamics invalid；按现有
guide rolling retime 后，实际交给后端的 seed duration 为 `3.424790 s`、max jerk 为
`9.775454`、`dynamics_valid=1`。这正是旧条件屏蔽掉的已有路径。

三机轨迹摘要：

| CSV uav_id / 实机 | path | 最长 `<0.05 m/s` | 最终 traj ID |
|---|---:|---:|---:|
| 1 / drone0 | 175.131 m | 0.567 s | 812 |
| 2 / drone1 | 179.003 m | 0.734 s | 722 |
| 3 / drone2 | 115.231 m | 1.367 s | 470 |

目标运动统计窗口：

```text
CAMERA_TIME                = 216.194990 camera·s
MEAN_VISIBLE              = 2.825784
K2                        = 0.994338
ALL3                      = 0.832317
NONE                      = 0.000871
LONGEST_K2_LOSS           = 0.429248 s
LONGEST_BLACKOUT          = 0.065507 s
```

上述数值只证明 direct hole 修复后 drone0 能持续运动，不作为整套复杂架构的健康验收。

## 3. Phase 2：代码区规模与真实 production 边界

Phase 2 开始后没有再改 production。当前主要文件规模：

| 文件 | 行数 |
|---|---:|
| `planner_manager.cpp` | 11,739 |
| `poly_traj_optimizer.cpp` | 9,084 |
| `multi_uav_topology_coordinator.cpp` | 2,962 |
| `ego_replan_fsm.cpp` | 2,855 |
| `team_visibility_optimizer.cpp` | 2,104 |
| `team_target_reachability.cpp` | 326 |

仅这六个文件已约 29k 行；`planner_manager.cpp` 加载 81 个参数、包含 303 个 ROS 日志点，
`poly_traj_optimizer.cpp` 再加载 72 个参数。FSM 仍有约 101 行明显的注释掉控制流。

工作区还保留一套完整的 `ros_ws/src/ego-planner`，但根目录含 `CATKIN_IGNORE`；实际
`rospack find ego_planner` 指向 `EGO-Planner-v2/.../plan_manage`。因此旧树是明确的历史
副本，不属于 build，却继续增加搜索和维护歧义。

## 4. 当前机制清单：哪些活着，哪些重复，哪些已经死了

### 4.1 本地候选生产：活跃，但层数过多

当前每个 UAV 的真实 local 链为：

```text
normal/cooperative/hypothesis endpoint
→ team reference / recovery guide / accepted warm start / fresh direct 四级 seed
→ NOMINAL + SIDE_PLUS + SIDE_MINUS
→ SIDE offset/backoff
→ 可选 same-topology A*
→ Local-SFC
→ MINCO
→ 可选 hard-corridor SCP
→ candidate acceptance + side hysteresis + local visibility comparator
→ current-revision final preflight
```

有效运行的 initializer 使用量：

```text
recovery GUIDE_SEED_USED             4559
fresh direct initializer             1728
accepted warm start                   30
accepted SIDE warm start               11
NO_ACCEPTED_STATE                    6314
```

accepted warm-start 分支为了约 41 次真实使用保留了大量 cache、identity、反事实和排序
条件；而 recovery guide 已经成为主 seed。这是一个明显的删减候选。

### 4.2 endpoint / guide authority：四套同时存在

当前至少有四个上游几何来源：

1. 正常 rolling local target；
2. cooperative viewpoint per-UAV reference / encirclement hypothesis；
3. `TeamRecoveryTarget` 的三机 guide；
4. committed `TeamTrajectoryReference` / joint suffix。

Planner 内部又分别决定这些对象能否改 endpoint、能否提供 seed、能否覆盖 warm start、
是否只作 soft cost。Phase 1 的直接 bug 就来自这些 boolean authority 组合，而不是缺一套
机制。

### 4.3 `team_target_reachability`：活跃的 shadow planner

该进程不是轻量 checker。它：

- 复制三套 `EGOPlannerManager` 及其参数、地图和 predictor；
- 每 0.5 s 对三机分别调用 `probeRecoveryTarget()`；
- 再跑 NOMINAL/SIDE/A*/Local-SFC/MINCO；
- 枚举三机 candidate 组合；
- 再做 team final preflight；
- 产出 guide，真实 Planner 随后又把 guide 重新送入 MINCO/final preflight。

运行中 `TARGET_PROPOSED=286`、`GUIDE_REUSED=230`、guide seed 使用 4559 次。它是
production active，却与三台真实 Planner 做了同一件事两遍，是当前最重的结构性重复。

### 4.4 team transaction：源码描述与真实执行相互矛盾

`UnifiedTeamPlanningTransaction` 类、状态机和 tests 都存在，`PendingTeamSolution` 甚至
持有一个实例；但 production 中没有任何一次
`create/localReserveReady/beginQualitySearch/jointResult/decide/commit/abort` 调用。

真实路径在 `ego_replan_fsm.cpp` 明确写死：

```text
finalizeCapturedCandidates(..., reserve_for_team_transaction=false)
→ publishCurrentTrajectory("local_first_safe_direct")
→ LOCAL_DIRECT_COMMIT
→ 提交后才 stageCapturedTopologyCoordination()
→ joint 作为 committed-prefix 后台质量替换
```

有效运行：

```text
LOCAL_DIRECT_COMMIT             4606
TEAM_PENDING_START              1888
EARLY_JOINT_OPT_ATTEMPT          266
TRANSACTION_JOINT_SUCCESS        114
TRANSACTION_COMMIT                32
TRANSACTION_ACTIVATED             24
JOINT_FAILURE_LOCAL_NOOP          304
```

因此当前真正有效的是“local first-safe 立即提交 + optional background joint”，不是
“local reserve 和 joint 在一个 unified transaction 中二选一”。后者是历史实验留下的
dead facade；继续同时维护两种架构只会制造 PENDING、boundary、generation 和 timeout
语义冲突。

### 4.5 visibility / geometry：同一决策在四层重复

同一组 K2、mean-visible、All3、weakest camera、loss、Q_dir、gap 指标至少在以下层
重复判断：

1. local optimizer 的 `J_vis` / bearing / gap soft cost；
2. `LocalVisibilityPreference` 的 local candidate 排序；
3. cooperative/adaptive viewpoint generator 的 target/hypothesis 排序；
4. topology coordinator comparator 与 `TeamVisibilityOptimizer` 的 objective、
   `measurableBenefit` 和 final acceptance。

这些层的时间窗、epsilon、binary/continuous metric 和 authority 并不完全相同。即使每个
局部条件单独合理，也可能出现 optimizer 朝一个方向走、local comparator 不接受、team
comparator又按第三种口径改选的情况。Q_dir/25°/170° 已降为后级 soft/diagnostic，但
相关生成、恢复、cost、排序、日志仍散落在各层。

### 4.6 rolling continuity：三套恢复同时工作

当前同时存在：

- previous-safe suffix / persistence revalidation；
- first-safe reserve + remaining-budget quality search；
- post-deadline fresh-state recovery；
- executor terminal hold；
- recovery target guide refresh/replan；
- moving-horizon renewal。

运行日志中：

```text
PERSISTENCE_REVALIDATION          5179
FIRST_SAFE_RESERVED                229
QUALITY_SEARCH_AFTER_FIRST_SAFE     62
POST_DEADLINE_RECOVERY_ENTER       148
POST_DEADLINE_RECOVERY_SUCCESS     148
TERMINAL_HOLD_ENTER                130（全部 drone2）
```

post-deadline recovery 能恢复，说明它不再永久死锁；但 recovery、persistence、quality
search、joint 和 guide probe 都消费同一个滚动窗口/CPU。系统在“保覆盖”上不是缺机制，
而是多个机制争用 deadline。

### 4.7 明确 dead / flag-off / 近似无效代码

| 机制 | 当前状态 |
|---|---|
| `ros_ws/src/ego-planner` 整树 | `CATKIN_IGNORE`，历史副本 |
| `UnifiedTeamPlanningTransaction` 状态迁移 | production 无调用，tests-only facade |
| `reserve_for_team_transaction=true` 路径 | 当前 production call 明确传 false |
| `predictionWindowCovers` / `committedFrontierStillCurrent` / `sameLocalRollingTrace` | tests-only；production只用另两个 prefix helper |
| Elastic tracking region | runtime OFF |
| candidate hard-corridor SCP | runtime OFF |
| A* visibility cost | runtime OFF |
| early avoidance | runtime OFF |
| relative temporal safety elasticity | runtime OFF |
| warm-start counterfactual | runtime OFF |
| post checks | runtime OFF |
| joint yaw | 本次 runtime OFF |
| prefix progress cost | weight 0 |
| team K2-continuity / blackout objective | weight 0 |

这些不是说应当一次性盲删，而是应当停止让它们继续参与主代码的条件组合。先做 production
引用证明，再按小批次移除；不要再给 flag-off 分支修新功能。

## 5. 互相打架的具体地方

### 5.1 target authority 与 initializer authority

Phase 1 已实证：为了保护 NOMINAL endpoint 而禁用 guide seed，会把唯一静态可行路径
一并删除。今后不应继续用 `nominal_baseline`、`visibility_first_nominal` 这类目标标签
间接控制 seed、warm-start 或执行权威；每种 authority 必须是独立、单向的数据。

### 5.2 local first-safe 与 unified transaction

真实系统要求 local first-safe 不等 joint；dead unified transaction 又假定 local reserve
应等待 team decision。两种模型不能同时作为 production 设计。当前运行证据支持保留前者：
local 立即保覆盖，joint 只能及时替换，失败必须 no-op。

### 5.3 shadow guide planner 与真实 local planner

shadow worker 先用完整 Planner 证明 guide，真实 Planner 再从 guide 重建并做同样 hard
preflight。它增加的不是独立安全事实，而是第二套候选生成 authority。两者地图快照、
dynamic epoch、deadline 或 activation 只要稍有不同，就会产生“guide usable 但 real plan
不可用”以及反方向的不一致。

### 5.4 多层 quality gate 与 hard executability

candidate acceptance、clearance gain、side hysteresis、local visibility comparator、team
comparator、joint measurable-benefit 都能拒绝已经 hard-safe 的替换。它们的合理位置只能
有一个：hard-safe executable set 形成之后的统一 quality comparator。把相同偏好散在
producer、optimizer admission 和 team acceptance 中，会出现“有安全候选但每层都认为
别人会负责”的空集。

### 5.5 persistence / recovery / joint 共用 deadline

previous-safe revalidation、guide refresh、local quality search 和 joint 都需要时间；当前
代码分别计算自己的预算，却最终争用同一个单线程 Planner/ROS callback 和 predecessor
coverage。130 条 drone2 hold 是这种结构仍未收敛的直接运行证据。

## 6. 建议保留的最小算法链

建议目标不是继续修补现有所有分支，而是收敛到以下单一主链：

```text
authoritative odom P/V/A
+ one target snapshot/world epoch
+ static map / dynamic prediction
+ peer committed trajectories

→ 一个 local endpoint（cooperative viewpoint只给 soft endpoint，不给执行权）
→ 一个 local candidate producer
   NOMINAL direct seed
   direct seed静态不可行时：同一 Planner 的 A* → Local-SFC seed
   当前 BODY/LOS risk存在时才生成 SIDE±
→ MINCO（必要时同一条明确的 corridor refinement）
→ 一次 current-revision hard preflight
→ 一个统一 local quality comparator
→ first-safe 立即提交
→ executor identity / handoff check

可选：在已提交 future prefix 上做一次后台 team P/T quality pass
→ 一次 team cross-UAV hard preflight
→ 3 ACK / atomic commit
→ 超时、失败或过期一律 local no-op
```

需要保留的 hard 条件只有：dynamics、static、dynamic、swarm、current revision、handoff
identity 和 executor activation。它们在 local final preflight 与 team/ACK 边界各检查一次
有意义；同一候选在更早层反复做等价 admission 则应删除。

## 7. 建议的删减顺序（后续任务，不在本轮实施）

### 第 1 批：零算法风险的清场

1. 从工作区移出/归档 `ros_ws/src/ego-planner` 忽略副本；
2. 删除 tests-only 的 `UnifiedTeamPlanningTransaction` facade 和当前永远传 false 的
   reserve/PENDING alternative，保留真实 committed-prefix joint 模型；
3. 删除 flag-off 且无 production 计划的 Elastic、hard-SCP、A* visibility cost、
   relative elasticity、counterfactual、post-check 分支；
4. 清掉 FSM 注释代码和把高频诊断降为受控 telemetry level。

### 第 2 批：合并 authority

1. cooperative manager 只发布 soft desired endpoint/hypothesis；
2. 移除 `team_target_reachability` 的三套 Planner replica；
3. 将“direct chord 静态不可行时的 guide”变成本机 Planner 内同一次 A*/Local-SFC seed，
   不再经过外部 recovery target execution authority；
4. 保留一个 target snapshot identity 和一个 activation epoch，不再让各层自行 rebase。

### 第 3 批：统一 selection

1. 建立一次 hard-safe executable set；
2. local 和 team 共用同一个 visibility-first comparator 定义；
3. clearance gain、SIDE hysteresis、Q_dir/gap 只作该 comparator 的后级 tie-break；
4. 删除 optimizer 外的重复 measurable-benefit/gate，或让其只检查数值噪声，不重做排序。

### 第 4 批：收敛 lifecycle

1. predecessor 尚有效：只允许正常 rolling successor + bounded persistence；
2. predecessor 过期：仅重置 start authority 到真实 odom 并重新走同一 Planner，不建立
   第二条 recovery pipeline；
3. terminal hold 只属于 executor 的 fail-closed 结果，不应成为 Planner 策略；
4. joint 永远不得占用 local first-safe 的 deadline。

每一批都应先删除、build、用同一场景验证，再进入下一批；不要在删减过程中同时调权重
或增加补偿机制。

## 8. 最终字段

```text
NOMINAL_RECOVERY_GUIDE_AUTHORITY_HOLE_FIXED: YES
NOMINAL_TARGET_AUTHORITY_CHANGED: NO
RECOVERY_GUIDE_INITIALIZER_ALLOWED_FOR_NOMINAL: YES
NEW_RECOVERY_EXIT_ADDED: NO
NEW_FSM_ADDED: NO
SAFETY_THRESHOLD_CHANGED: NO

BUILD_PASS: YES
REAL_RUN_COUNT: 1 valid + 1 invalid bootstrap not counted
ORIGINAL_DRONE0_STALL_REPRODUCED: NO
DRONE0_GUIDE_SEED_USED: 837
DRONE0_FINAL_TRAJECTORY_ID: 812
DRONE0_LONGEST_LOW_SPEED: 0.567 s

WHOLE_SYSTEM_HEALTHY: NO
INDEPENDENT_DRONE2_TERMINAL_HOLD_LOGS: 130

PRODUCTION_ACTIVE_LOCAL_AUTHORITY:
  local first-safe direct commit
PRODUCTION_ACTIVE_TEAM_AUTHORITY:
  optional committed-prefix background joint P/T
UNIFIED_TRANSACTION_STATE_MACHINE_ACTIVE: NO
SHADOW_THREE_PLANNER_REACHABILITY_ACTIVE: YES

ARCHITECTURE_SUBTRACTION_REQUIRED: YES
RECOMMENDED_MINIMUM:
  one local rolling planner + one final local comparator + one optional
  nonblocking committed-prefix team quality pass + one executor contract

PHASE2_PRODUCTION_CHANGED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

最终判断：本轮应修的直接条件已经修完，原 drone0 自锁得到真实闭环证据。接下来最重要的
工作不是继续追一个日志就补一个 recovery，而是按上述顺序删除 inactive/dead facade、
移除 shadow Planner、统一 endpoint/initializer/selection authority，并让 joint 回到真正
可选且不阻塞 local rolling 的位置。

