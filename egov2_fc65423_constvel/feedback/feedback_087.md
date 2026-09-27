# Feedback 087 — ALP 架构减法重构与单次最终运行

日期：2026-09-19  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
基线：`feedback/feedback_086.md`  
场景：`long_cylinder_forest.json`

## 0. 结论

本轮没有增加 recovery、fallback、transaction 或新 FSM。production 已收敛为三类权威：

```text
Local Planner
  → one cooperative soft reference
  → ConflictDescriptor(STATIC/DYNAMIC BODY, LOS)
  → N/L/R
  → valid same-topology warm start 或 fresh direct/A*/Local-SFC initializer
  → one MINCO → current-revision hard preflight
  → shared visibility-first comparator → local first-safe commit

optional committed-prefix Team P/T
  → team preflight → proposal / 3 ACK / common activation
  → fail / timeout / stale = local no-op

Executor
  → predecessor identity / handoff / activation / fail-closed HOLD
```

明确退出 production 的是：shadow `team_target_reachability`、external recovery
target/guide authority、等待 team decision 的 unified transaction facade、reserve-for-team
alternative、velocity continuation、Joint yaw，以及 Elastic、hard-corridor SCP、A* visibility
cost、relative temporal elasticity、counterfactual、legacy post-check 等 flag-off 分支。旧
`ros_ws/src/ego-planner` 已移至 `_archive/legacy_ego_planner`，并增加
`ARCHIVE_STATUS.md` 明确 `NOT_BUILT / NOT_PRODUCTION / DO_NOT_EDIT`。

核心四文件从 26,640 行降到 19,671 行，减少 6,969 行（26.16%）；按 Feedback086 相同
口径，Planner/Optimizer ROS 参数从 153 降到 105，减少 31.37%。最终构建无 warning，
14 个当前有效 C++ contract binary、12 个 adaptive execution contracts、20 个 lifecycle
wiring contracts 及 tracking visibility contract 全部通过。

唯一最终运行的目标运动阶段表现良好：三机均无 `<0.05 m/s` 持续低速 episode，
`K2=100%`、`ALL3=85.4466%`、blackout=0。但完整运行不健康：目标停止后 UAV0 的
trajectory 276 耗尽，进入一次 terminal hold，此后没有新 activation，记录结束时已持续
147.274 s，并留下 4,419 个 `safety_validated=false` 样本。

```text
FINAL_ASSESSMENT: PARTIAL
SIMPLIFICATION_COMPLETE: PARTIAL
CORE_ALGORITHM_PRESERVED: YES
FULL_RUNTIME_HEALTHY: NO
FINAL_REAL_RUN_COUNT: 1
```

`PARTIAL` 不是因为还缺一套 recovery，而是同一个 Local Planner 在 terminal hold 后没有
继续产生/激活 successor。按本轮原则，这一结果只报告，不在最终运行后再发明模块或启动
第二次仿真。

## 1. 四批架构减法

### Batch 1：dead / flag-off 清场

- 历史 `ros_ws/src/ego-planner` 移出 ROS source 搜索语义，源码保留在项目内 archive；
- 删除 inactive Elastic、hard-SCP、A* visibility cost、early avoidance、relative temporal
  elasticity、counterfactual、post-check、零权重 prefix progress、Joint yaw 等分支和参数；
- yaw 继续由 target-facing 语义导出；canonical runner 只暴露 scene、headless/RViz、
  timeout、team P/T。

以下名称在非 test、非 archive production 源码内均为 0 matches：

```text
UnifiedTeamPlanningTransaction / reserve_for_team_transaction
team_target_reachability / TeamRecoveryTarget
enable_elastic_tracking_region_ / hard_corridor_scp
warm_start_counterfactual / relative_temporal_safety
a_star_visibility_cost / enable_post_checks
```

### Batch 2：只保留真实 team transaction

删除了“local reserve 等待 team decision”的 facade。当前唯一模型是 local hard-safe
candidate 先提交，Team P/T 只处理 committed future；失败、超时、过期只丢弃 joint。
3 ACK、共同 activation、payload identity 和 stale/CAS 检查保留，因为它们解决三机
future-tail 原子替换，不是第二套 local execution authority。

### Batch 3：移除 shadow Planner 与 recovery guide

原 `team_target_reachability` 复制三套 `EGOPlannerManager`，生成 guide 后真实 Planner
又跑一次 MINCO/preflight。该进程及 guide lifetime/guide-vs-warm authority 已退出。

真实 Planner 现在只有两类 initializer：

1. `SAME_TOPOLOGY_VALID_WARM_START`；
2. `FRESH_GEOMETRIC_INITIALIZER`：direct；若 direct 静态不可行，则同一 Planner 内
   A* → simplify → Local-SFC，再进入同一条 MINCO。

最终 run 中 `STATIC_INFEASIBLE` 触发 82 次、`ASTAR_REPAIR_ACCEPT` 141 次，证明
same-Planner repair 确实进入 runtime。

### Batch 4：统一 selection、safety 与 lifecycle

- Local `J_vis(P,T)` 保持 continuous objective，不因 soft cost 非零触发 SIDE；
- raw BODY/LOS 统一进入 `ConflictDescriptor`，物理 witness 仍分开；
- Local、cooperative 与 team 共用 `betterTeamVisibilityWithinNearBestK2()`：K2 near-best
  set 后比较累计 camera time；All3、NONE、weakest、blackout、Q_dir、diversity只作 metric；
- soft 120° slot、radius 0.35 m deadband、height 0.20 m deadband和 weak bearing slack保留；
- Team P/T objective 只保留 K2、accumulated visibility、一个 geometry regularizer、
  deviation/smoothness；
- candidate classification 收敛为 `ABSOLUTE_SAFE / INVALID`，旧 `IMPROVED_ONLY` 不再
  拥有全局 authority；
- predecessor 有效时继续执行并由同一 Planner 下一 tick 重试；过期时 start authority
  切到真实 odom，仍调用同一 producer；executor 只负责 fail-closed HOLD。

源码结构符合上述语义，但最终 run 证明最后一项的恢复闭环仍有缺口，见第 6 节。另有
少量无 runtime consumer 的 metric/test helper（如 `blackoutGradient`）仍在源码中；它们
无 production authority，但属于纯源码清理残留，所以不把减法写成绝对完成。

## 2. 复杂度变化

| 项 | Feedback086 前 | 当前 | 变化 |
|---|---:|---:|---:|
| `planner_manager.cpp` | 11,739 | 9,402 | -2,337 (-19.91%) |
| `poly_traj_optimizer.cpp` | 9,084 | 4,914 | -4,170 (-45.90%) |
| `ego_replan_fsm.cpp` | 2,855 | 2,609 | -246 (-8.62%) |
| `multi_uav_topology_coordinator.cpp` | 2,962 | 2,746 | -216 (-7.29%) |
| 四文件合计 | 26,640 | 19,671 | -6,969 (-26.16%) |
| Planner + Optimizer 参数 | 153 (81+72) | 105 (55+50) | -48 (-31.37%) |

机制计数采用“具有独立决策/供给 authority 的 production 路径”口径，不把 telemetry 和
executor HOLD 当 Planner：

| authority / mechanism | before | after |
|---|---:|---:|
| active recovery mechanism | 3（persistence/guide/post-deadline） | 1（同一 Local Planner retry/odom restart） |
| endpoint authority | 4 | 1（cooperative soft geometry 由 Local 消费） |
| initializer types | 4 | 2（valid warm / fresh geometric） |
| visibility comparator definitions | 4 | 1 shared policy |
| team transaction models | 2 | 1 committed-prefix background P/T |
| shadow Planner process | 1 | 0 |

## 3. 构建与测试

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws

6 packages succeeded; warnings=0; failures=0

traj_opt C++ contracts                    2/2 PASS
ego_planner C++ contracts                 4/4 PASS
multi_uav_formation C++ contracts         8/8 PASS
合计                                    14/14 PASS
adaptive_execution_contract_test.py      12/12 PASS
trajectory_lifecycle_wiring_test.py      20/20 PASS
tracking_visibility_contract_test.py     PASS
```

`catkin test` 因工程使用普通 `add_executable/add_test` 而报告 0 个 catkin XML tests；没有
把空结果冒充通过，而是直接运行上述当前 CMake 清单中的 14 个二进制。

测试阶段发现并修正两项真实不一致：local geometry 已按 horizon 计算 violation，selector
又以 terminal endpoint 重算；现只消费一次 horizon metric。另一个是 authority horizon
外的 future LOS 会覆盖更早 BODY/LOS witness；现只允许 horizon 内 raw LOS 取得 topology
权威并按最早 witness 合并。两条 Python 旧断言仍匹配无 `touch_goal` 参数的历史签名，
也已同步到当前真实接口。

```text
git diff --check                    PASS
launch/XML parse                    13/13 PASS
Python py_compile                   3/3 PASS
production dead-symbol check        0 matches
```

## 4. 唯一最终运行

```text
RUN_ID:          20260919_205929_346132
RUN_DIR:         runs/20260919_205929_346132
SCENE_SHA256:    7585c2c2392a2dcc0914add7418d60f9a7475edee53fcffff678e1d6e4111faf
COMMAND:         ./run_on.sh --headless --timeout 200 --pt
BOOT_12:         YES
CORE_EXIT_CODE:  0
CLEANUP_STATUS:  OK
SECOND_RUN:      NO
```

### 4.1 目标运动阶段（76.500307 s）

CSV `uav_id=1/2/3` 对应 `drone0/1/2`。

| UAV | path | speed P10 / P50 | `<0.05 m/s` episodes / longest |
|---|---:|---:|---:|
| UAV0 | 77.144 m | 0.658 / 0.946 m/s | 0 / 0 s |
| UAV1 | 70.561 m | 0.580 / 0.930 m/s | 0 / 0 s |
| UAV2 | 75.603 m | 0.666 / 0.948 m/s | 0 / 0 s |

```text
CAMERA_TIME:             218.368 camera·s
MEAN_VISIBLE:            2.854466
K2:                      1.000000
ALL3:                    0.854466
NONE / BLACKOUT:         0 / 0 s
LONGEST_K2_LOSS:         0 s
LONGEST_BLACKOUT:        0 s

BEARING_ERROR_P50/P90:   0.607° / 1.960° (三机汇总)
  UAV0:                  0.81° / 2.42°
  UAV1:                  0.19° / 1.15°
  UAV2:                  0.90° / 2.15°
RADIUS_P50/P90:          1.967 / 2.138 m
HEIGHT_ERROR_P90/MAX:    0.227 / 2.368 m
ALL_THREE_AHEAD_RATIO:   0.000000
TWO_FAR_AHEAD_RATIO:     0.961186
```

相对 Feedback086：mean-visible `+0.028682`，K2 `+0.5662 pp`，All3
`+2.2149 pp`，NONE `-0.0871 pp`，最长 K2 loss 从 0.429 s 降到 0。这里只说明目标
运动窗口没有明显 visibility 回归；不能覆盖后续 liveness failure。高度 max 2.368 m 也
说明仍有一次明显 vertical excursion。

### 4.2 Team P/T

```text
JOINT_ATTEMPT:            464
JOINT_SUCCESS:            2 optimizer successes
JOINT_COMMIT:             0
JOINT_ACTIVATED:          0
JOINT_FAILURE_LOCAL_NOOP: 466
```

Team P/T 确实进入 optimizer，但两个成功结果均未通过最终 replacement acceptance/identity
时序进入 commit。local 未等待 joint，no-op 合同正确；0 activation 说明 Team P/T 的真实
有效性本轮未闭合，不能用 attempt 数冒充算法收益。

### 4.3 Safety

```text
EXECUTED_COLLISION_SAMPLES (scene surface <= 0): 0
SWARM_VIOLATION_FRAMES (elliptical < 0.5 m):      0
MIN_SWARM_DISTANCE elliptical / Euclidean:       1.005381 / 1.005417 m
MIN_STATIC_CLEARANCE:                             0.156820 m
MIN_DYNAMIC_CLEARANCE:                            0.018465 m
```

日志另有 37 条 grid-map `the drone is in obstacle` warning，集中于约 37.14–37.37 s；
同窗 recorder 的 scene surface 没有穿透，但 moving clearance 最低仅 0.018 m。主字段按
统一 scene geometry 为 collision=0，同时保留这 37 条 inflated-grid 告警，不把本轮写成
无条件 safety PASS。

## 5. 完整运行的 liveness failure

完整 234.106 s recorder 窗口：

| UAV | full path | speed P10 / P50 | `<0.05` episodes / max | 最终 source |
|---|---:|---:|---:|---|
| UAV0 | 83.872 m | 0 / 0 m/s | 2 / 146.938 s | `TERMINAL_HOLD` |
| UAV1 | 166.320 m | 0.096 / 0.816 m/s | 1 / 0.201 s | `FEASIBLE_FALLBACK` |
| UAV2 | 172.761 m | 0.107 / 0.816 m/s | 1 / 0.334 s | `FEASIBLE_FALLBACK` |

```text
UAV0/1/2 CONTINUOUS_MOTION (full run): NO / YES / YES
TERMINAL_HOLD_EPISODES:                 1
TERMINAL_HOLD_TOTAL_DURATION:           147.274 s in recorded window
TERMINAL_HOLD_MAX_DURATION:             147.274 s (right-censored at shutdown)
TRAJECTORY_END_BEFORE_NEXT:             1
STARVATION_EPISODES:                    3 unique trajectory identities
UNVALIDATED_EXECUTED_SAMPLES:           4419 (all UAV0 terminal hold)
```

唯一 hold 链：

```text
drone0 trajectory 276
last publish          = 1789822861.958315
activation            = 1789822862.058113
validated end         = 1789822862.595960
last planning tick    = 1789822862.280391
candidate 2701        = SHORT_STATIONARY_HYPOTHESIS
terminal hold enter   = 1789822862.601291
next activation       = none
recording end         = 1789823009.875431
```

最后一个 planning tick 正确保留当时尚有效的 predecessor，但此后 drone0 没有新的
`PLANNING_START`、publish 或 activation；planner heartbeat 也不再出现。日志没有显式
process-died/exception 行，所以 FIRST REAL DIVERGENCE 只能定位到“drone0 local planning
supply 停止”，不能伪造为已证明的 timer、node exit、callback 阻塞或 FSM 条件。executor
fail-closed 是正确结果，但同一 Planner 从真实 odom 重试并恢复的运行合同没有闭合。

这不是恢复 shadow planner 的理由。若继续，只应查清 normal planning tick 为什么消失，
而不是新增 terminal-hold recovery pipeline。

## 6. 九项直接回答

1. **当前 ALP 是否只剩一个 Local planning authority？** 源码和 launch 上是。只有三架
   实机的同构 Local Planner 能生成 local executable trajectory；cooperative 只给 soft
   reference，Team 只替换 committed future。
2. **shadow Planner 是否彻底退出 production？** 是。进程、launch、CMake、consumer 和
   recovery guide authority 均移除；历史源码仅在 archive。
3. **recovery 是否变成同一个 Planner 的 initializer 问题？** 是。static direct chord
   失败由同一 Planner 的 A*/Local-SFC 生成 fresh initializer；runtime 有 141 次 accept。
4. **Team 是否只剩 committed-prefix background P/T？** 是。3ACK/common activation仅
   服务 future-tail atomic replacement。
5. **visibility 是否收敛为 continuous objective + one comparator？** production decision
   是；其它指标只作 telemetry。少量无 runtime call 的 metric helper 是清理残留。
6. **hard safety 是否形成 shared physical definition + final executable decision？** 源码与
   contracts 是；Local commit和Team payload边界各作必要 preflight。但37条grid告警和
   18 mm minimum dynamic clearance使运行只能保守写 PARTIAL。
7. **rolling 是否只剩 predecessor有效继续、下一tick同Planner重试、耗尽executor HOLD？**
   控制流设计是，运行结果否。UAV0 hold 后 planning tick 消失且没有恢复。
8. **减法后是否仍保持四条创新主线？** 源码和目标运动阶段保持：软合围、持续可见性、
   BODY/LOS统一N/L/R、local-first-safe + background Team P/T均存在；K2=100%、blackout=0。
   但完整 safe rolling execution失败，所以只能回答 PARTIAL。
9. **还有哪些复杂机制必须保留？** 只有三类有不可替代证据：predecessor/handoff/executor
   identity保证未验证payload不执行；Team 3ACK/common activation保证三机原子替换；
   bounded suffix revalidation在单tick失败时保留已验证覆盖。无 production consumer 的
   helper属于清理残留，不以“以后可能有用”为理由。

## 7. 最终字段

```text
SIMPLIFICATION_COMPLETE: PARTIAL
CORE_ALGORITHM_PRESERVED: YES
SOFT_ENCIRCLEMENT_PRESERVED: YES
LOCAL_VISIBILITY_PT_PRESERVED: YES
UNIFIED_THREE_THREAT_TOPOLOGY_PRESERVED: YES
TEAM_PT_PRESERVED: YES (runtime attempts=464; activation=0)
JOINT_YAW_PRODUCTION_ACTIVE: NO

SHADOW_PLANNER_REMOVED: YES
RECOVERY_GUIDE_AUTHORITY_REMOVED: YES
UNIFIED_TRANSACTION_FACADE_REMOVED: YES
LOCAL_FIRST_SAFE_MODEL_ONLY: YES

ACTIVE_RECOVERY_MECHANISM_COUNT_BEFORE: 3
ACTIVE_RECOVERY_MECHANISM_COUNT_AFTER: 1
ACTIVE_ENDPOINT_AUTHORITY_COUNT_BEFORE: 4
ACTIVE_ENDPOINT_AUTHORITY_COUNT_AFTER: 1
ACTIVE_INITIALIZER_TYPES_BEFORE: 4
ACTIVE_INITIALIZER_TYPES_AFTER: 2
ACTIVE_VISIBILITY_COMPARATOR_COUNT_BEFORE: 4
ACTIVE_VISIBILITY_COMPARATOR_COUNT_AFTER: 1 shared policy
ACTIVE_TEAM_TRANSACTION_MODELS_BEFORE: 2
ACTIVE_TEAM_TRANSACTION_MODELS_AFTER: 1
SHADOW_PLANNER_PROCESS_COUNT_BEFORE: 1
SHADOW_PLANNER_PROCESS_COUNT_AFTER: 0
ACTIVE_ROS_PARAMS_BEFORE: 153
ACTIVE_ROS_PARAMS_AFTER: 105
CODE_LINES_BEFORE: 26640
CODE_LINES_AFTER: 19671

RUN_ID: 20260919_205929_346132
BOOT_12: YES
CORE_EXIT_CODE: 0
UAV0/1/2_CONTINUOUS_MOTION_TARGET_PHASE: YES / YES / YES
UAV0/1/2_CONTINUOUS_MOTION_FULL_RUN: NO / YES / YES
TERMINAL_HOLD_EPISODES: 1
TERMINAL_HOLD_TOTAL_DURATION: 147.274 s recorded, right-censored
TERMINAL_HOLD_MAX_DURATION: 147.274 s recorded, right-censored
STARVATION_EPISODES: 3
TRAJECTORY_END_BEFORE_NEXT: 1
COLLISION_COUNT: 0 by scene-surface execution audit
SWARM_VIOLATION_COUNT: 0
GRID_OCCUPANCY_COLLISION_WARNINGS: 37

MEAN_VISIBLE: 2.854466
K2: 1.000000
ALL3: 0.854466
BLACKOUT: 0 s
LONGEST_K2_LOSS: 0 s
BEARING_ERROR_P50/P90: 0.607 / 1.960 deg
RADIUS_P50/P90: 1.967 / 2.138 m
HEIGHT_ERROR_P90/MAX: 0.227 / 2.368 m
JOINT_ATTEMPT: 464
JOINT_SUCCESS: 2
JOINT_COMMIT: 0
JOINT_ACTIVATED: 0
ALL_THREE_AHEAD_RATIO: 0.000000
TWO_FAR_AHEAD_RATIO: 0.961186

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

最终判断：主要架构减法已经完成，且目标运动阶段证明核心算法没有因删除而消失；但 UAV0
进入不可恢复 terminal hold，Team P/T 也没有实际 commit/activation。因此不能把代码减少
26%直接写成系统健康。下一步唯一合理方向是定位 normal planning tick 在
1789822862.280391 后为何停止，不是恢复任何已删除的 shadow/recovery/transaction 机制。
