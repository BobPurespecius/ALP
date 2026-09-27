# Feedback 066 - Rollback of FREE_TIME_FROZEN and Root Cause of Rolling Duration Collapse

## 0. Scope and protection

本轮只回退了最近为抑制 rolling duration collapse 引入的 `FREE_TIME_FROZEN` 语义，然后完成必要构建和一次当前生产场景的 FULL ON + native RViz 运行观察。没有修改 BODY/LOS unified topology、blocker witness、observation geometry、persistence、Early Joint、CAS/ACK/COMMIT、atomic team activation 或 final hard safety。没有访问 `/home/bob/RRCT`，没有使用 reset、checkout 或整仓覆盖。

本轮保留的运行证据：

```text
rolling_time_restore_20260914/full.log
rolling_time_restore_20260914/scene.log
```

仿真、RViz、Gazebo、planner、recorder 均已停止；预先存在的 ROS master 未被清理。

## 1. Required result fields

```text
FREE_TIME_FROZEN_ROLLED_BACK: YES
ROLLING_TIME_OPTIMIZATION_RESTORED: YES
DYNAMIC_TIME_AVOIDANCE_OBSERVED: PARTIAL
DURATION_COLLAPSE_REPRODUCED: YES

FIRST_COLLAPSE_UAV: 2
FIRST_COLLAPSE_TIME: 1789400817.758594424
FIRST_COLLAPSE_GENERATION: candidate 1779 / topology generation 79
FIRST_DURATION_COLLAPSE_STAGE: MINCO_FREE_TIME

PERSISTENCE_FALLBACK_CAUSAL: PARTIAL
PREVIOUS_OPTIMIZED_T_REUSED_AS_REFERENCE: NO EVIDENCE
REANCHORED_SUFFIX_REUSED_AS_FULL_REFERENCE: NO EVIDENCE FOR CANDIDATE 1779
VALIDATED_PREFIX_REUSED_AS_PLANNING_HORIZON: NO EVIDENCE
DOUBLE_TIME_CONSUMPTION: NO EVIDENCE
TOUCH_GOAL_MODE_CHANGED_BY_REFACTOR: NO DIRECT EVIDENCE
FREE_TIME_MODE_CHANGED_BY_REFACTOR: YES FOR THIS ROLLBACK ONLY; NOT THE HISTORICAL ROOT CAUSE
TIME_COST_WAS_ALREADY_PRESENT_BEFORE_COLLAPSE: YES
BODY_LOS_UNIFIED_TOPOLOGY_ITSELF_CAUSAL: PARTIAL

DYNAMIC_HOLD_AFTER_ROLLBACK: 16
END_BEFORE_NEXT_AFTER_ROLLBACK: 8
STARVATION_AFTER_ROLLBACK: 4

PRODUCTION_SOURCE_CHANGED: YES (FREE_TIME_FROZEN rollback only)
PLANNER_BEHAVIOR_CHANGED: YES (rolling T/tau optimization restored)
LIFECYCLE_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
SIMULATION_RUN: YES, current production scene only
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

必要构建已通过：`traj_utils plan_env path_searching traj_opt ego_planner multi_uav_formation`。

## 2. What was rolled back

当前 `PolyTrajOptimizer::optimizeTrajectory()` 在普通 rolling 路径中重新使用 `current_durations` 计算 physical time trust region；`optimize_time` 仍为真，时间变量继续进入 SCP QP，accepted step 继续更新 `current_virtual_t` 和 `current_durations`。相关代码位于：

```text
poly_traj_optimizer.cpp:1256-1267   optimize_time/current_durations
poly_traj_optimizer.cpp:1354-1391   physical time trust initialization
poly_traj_optimizer.cpp:2630-2641   time columns and trust box
poly_traj_optimizer.cpp:2840-2978   trial duration update/acceptance
poly_traj_optimizer.cpp:3324-3351   accepted real-duration change
```

回滚没有删除 moving-object 的 `gradt`/`grad_prev_t`，也没有改变 `moving_obj_clearance`、动态硬检查、final preflight 或 unsafe trajectory 的执行资格。日志中出现了真实的：

```text
[scp-free-time-step]
trial_delta_virtual_T_max_abs > 0
actual_delta_real_T_max_abs > 0
trial_duration changed
```

所以 rolling T/tau 自由度确实恢复。不能把 candidate 1779 的短时长归因于 T 优化本身，因为该候选没有对应的 `scp-free-time-step` 记录。

## 3. First observed collapse

代表事件为 drone 2、candidate 1779：

```text
active trajectory 281 duration = 1.260929 s
warm_start_used = 0
source = FRESH_MOVING_DIRECT
fresh initializer duration = 1.000000 s
start = (39.895111, 0.618464, 1.459633)
target = (39.928005, 0.613633, 1.458200)
target distance ~= 0.0333 m
required speed = 0.033277 m/s
max velocity = 0.172072 m/s
candidate duration = 0.440468 s
SHORT_STATIONARY_HYPOTHESIS required = 0.490000 s
```

这条候选的目标是 rolling local target，不是最终任务目标 `(36, 0, 1.5)`。它的位移和 required speed 都非常小，候选被短/静止策略拒绝具有事实依据；本条不能证明合法 rolling successor 被错误拒绝。

更重要的是，同一运行中 `drone=2 / plan_id=4 / reachability_generation=42` 的真实 guide horizon 连续下降：

```text
1.137007, 1.124887, 1.112045, 1.099905, ...
0.909860, 0.898382, 0.886541, ...
0.780159, 0.768278, ...
0.605184, 0.593751, ...
0.540585, 0.528642, 0.516582, 0.504981
```

对应生成的 candidate duration 也从约 `0.478360`、`0.461523`、`0.443323` 下降到 `0.274957`、`0.182886`、`0.101493`，随后大量触发：

```text
SHORT_STATIONARY_HYPOTHESIS
NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE
MOVING_SUCCESSOR_STARVATION
TERMINAL_HOLD
```

日志中的 `[target-guide-local]` 同时打印了逐次下降的 `guide_horizon` 和 `fresh_duration`。因此这不是把 validated prefix 误读成 full polynomial duration；有限 guide/local-target horizon 本身在反复 rolling replan 中被消费。

## 4. First code-level mechanism

`EGOPlannerManager::buildRecoveryGuideSeed()` 位于：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:5197-5341
```

关键语义是：

```cpp
const double epoch = local_activation_time_;
const double remaining =
    guide.times.back() - (epoch - recovery_target_.guide_epoch);
if (remaining < 0.5)
  return false;
const int pieces = ceil(remaining / 0.5);
durations = Constant(pieces, remaining / pieces);
```

随后 guide 的 sample 使用 `epoch + t - guide_epoch`，tail 也使用这个 `remaining`。这意味着同一 finite guide 在每次新的 rolling activation/replan 中按 world time 消耗，新的 fresh initializer 总时长直接随 `remaining` 递减。

这不是 accepted trajectory suffix 被执行，也不是 unvalidated suffix 被放行；它是一个新的 fresh seed，但其几何来源仍受旧 finite guide 的绝对 expiry 约束。

`EGOReplanFSM::callReboundReplan()` 在：

```text
ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:756-922
```

先取 local target、调用 `prepareFutureActivation()`，再进入 recovery/topology hypothesis 的 `reboundReplan()`。hypothesis path 使用：

```cpp
hypothesis_local_target = local_target_pt_ + (reference - current_reference);
```

对 hypothesis 0，reference 与 current_reference 相同，因此目标仍为 `local_target_pt_`。这解释了第一条短候选为何是一个很近的 rolling target，而不是 mission-end。

## 5. Historical comparison: Feedback 061 -> 063

### Feedback 061

Feedback 061 的 FULL ON run 是健康的：`TERMINAL_HOLD=0`、`STARVATION=0`、`END_BEFORE_NEXT=0`、`UNVALIDATED=0`。但源码和日志已经有孤立短候选，例如 `SHORT_STATIONARY_HYPOTHESIS duration=0.194435`。因此不能写成“Feedback 061 完全没有短轨迹”。准确结论是：当时没有形成当前这种连续的 same-guide horizon consumption -> short successor -> coverage exhaustion 链。

### Feedback 063

Feedback 063 同期引入 unified BODY/LOS topology、observation geometry、blocker persistence 和 geometry-driven candidate lifecycle。其 run6 已有 `TERMINAL_HOLD=17`、`STARVATION=3`、`END_BEFORE_NEXT=17`，并出现大量 residual short successors。源码还明确支持跨 rolling cycle persistence：同一 blocker、已接受 observation side、当前 active safety trajectory 继续有效时复用当前 side，避免每轮重新求解两侧。

当前证据没有显示“previous optimized T 被直接复用为下一轮 T reference”，也没有显示 candidate 1779 先经过 accepted warm suffix、reanchor 或 crop 才缩短。当前更强、可直接从日志和源码证明的交互是：

```text
finite recovery-guide/local-target horizon
  -> rapid rolling/persistence/geometry replans
  -> same guide remaining is consumed before refresh
  -> fresh initializer duration shrinks
  -> short candidates are rejected or cannot provide coverage
  -> successor supply fails
  -> terminal hold
```

因此 `PERSISTENCE_FALLBACK_CAUSAL` 只能写 `PARTIAL`：它是高概率增加重规划频率和 refresh 竞争的生命周期交互，但本轮没有单独的历史 baseline instrumentation 能把“第一处增加重规划频率的具体提交”闭合到唯一 commit。

### Feedback 064

Feedback 064 主要是 RViz validated-prefix 颜色/大小调整；其短窗口把问题描述成 transient。当前运行中相同 guide horizon 的密集递减和最终 hold 被再次重现，因此该 transient 结论不成立。RViz 改动不是 duration 根因。

## 6. Checks for alternative duration causes

### Previous optimized T reuse

未找到 candidate 1779 使用 previous optimized T vector 作为 fresh guide reference 的证据。`buildRecoveryGuideSeed()` 从 guide remaining 重新构造 `durations`；这是几何 guide 时间消耗，不是 `T_optimized(k)` 递归赋值。

### Reanchor/crop and validated prefix

candidate 1779 明确 `warm_start_used=0`、`source=FRESH_MOVING_DIRECT`。没有证据显示 accepted reanchored suffix 或 validated/cropped prefix 被作为该 candidate 的 full planning horizon。因而不能把本事件归因于 suffix recursive reuse 或 validated-prefix/full-polynomial 混用。

### Double time consumption

源码审计和保留日志没有确认同一个 elapsed 被两个层重复扣除。`remaining` 的单一公式明确消耗 guide world time，但未发现另一个对该同一 guide horizon 的确定性重复扣除。结论为 `NO EVIDENCE`，不是已证明“不可能”。

### touch_goal/free_time mode

普通 rolling candidate 的 `touch_goal` 模式没有直接证据显示被 Feedback 063 的 topology/persistence 改写。`FREE_TIME_MODE_CHANGED` 仅指本轮从冻结恢复到可优化，不能把它写成 Feedback 063 的历史根因。

### J_T/time cost

time objective/gradient 在 collapse 之前已经存在；本轮日志也证明恢复后它确实能改变 rolling T。仅有 `J_T` 不足以解释为何 Feedback 061 健康而 Feedback 063 之后出现当前规模的 collapse。关键差异是 finite guide 的 remaining 被高频 rolling lifecycle 消耗，而不是新建一个 time cost。

## 7. Dynamic time avoidance after rollback

`DYNAMIC_TIME_AVOIDANCE_OBSERVED: PARTIAL`。

恢复后可观察到 rolling SCP 的虚拟和实际 duration 都发生变化，例如日志中的 `predicted_delta_real_T_max_abs`、`actual_delta_real_T_max_abs` 非零；这证明 FREE_TIME_FROZEN 曾经确实抑制了原有 T/tau 通道。

但当前一次 FULL ON 运行同时发生了大量 finite-guide collapse、短候选拒绝和 dynamic/coverage 失败。没有一条足够完整的证据链可以声称“某个动态障碍冲突已由 T 调整、通过 hard dynamic preflight 并实际激活”。因此只能报告为 `PARTIAL`，不能把 T 梯度日志误写成 dynamic avoidance 已完全恢复。

## 8. Counts and run interpretation

回滚后运行在首次明确供给失败后停止，计数为：

```text
TERMINAL_HOLD: 16
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION: 8
STARVATION / NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE: 4
SHORT_STATIONARY_HYPOTHESIS: 524
```

这些数字不是“只由 FREE_TIME_FROZEN 回滚造成”的 A/B 结论；它们描述恢复 rolling T 后当前生产场景中的实际状态。回滚恢复了时间优化自由度，但没有消除 finite-guide consumption 与 refresh cadence 的生命周期交互，反而让原有 time optimization 和 duration collapse 同时可见。

## 9. Root-cause classification

```text
COLLAPSE_PRIMARY_ROOT_CAUSE:
  buildRecoveryGuideSeed() uses the finite recovery-guide remaining horizon as
  the fresh rolling seed duration. Feedback063-era persistence/geometry-driven
  replanning can consume the same guide repeatedly before refresh, so the
  fresh initializer and resulting candidate duration shrink below coverage.

COLLAPSE_FIRST_CODE_LEVEL_FUNCTION:
  EGOPlannerManager::buildRecoveryGuideSeed()

COLLAPSE_FIRST_CODE_LEVEL_ASSIGNMENT_OR_CONDITION:
  remaining = guide.times.back() - (epoch - recovery_target_.guide_epoch);
  durations = Constant(pieces, remaining / pieces);

FIRST_SUSPECT_CHANGESET:
  FUNCTION-LEVEL CHANGE, not uniquely attributable to a commit:
  Feedback063 persistence/geometry-driven repeated rolling lifecycle combined
  with the pre-existing finite recovery-guide expiry semantics.

BODY_LOS_UNIFIED_TOPOLOGY_ITSELF_CAUSAL: PARTIAL
```

### Why Feedback 061 did not collapse at this scale

Feedback 061 already tolerated isolated short candidates, but its healthy run did not enter the observed high-frequency same-target persistence/replan loop before the guide refresh boundary. Consequently a finite guide was not consumed into a long chain of sub-0.5 s successors. This is a difference in lifecycle timing and replan density, not proof that the old free-time objective was intrinsically safe.

### Why Feedback 063 started collapsing

Feedback 063 added the persistence/geometry candidate lifecycle around the existing rolling authority. The retained generation-42 trace shows multiple guide reuses between refreshes and then a dense sequence of fresh seeds whose duration follows `remaining` down to approximately `0.5 s` and below. The exact first commit that changed replan frequency is not recoverable from the current dirty worktree and logs; claiming a unique commit would be fabricated.

## 10. Recommended minimal fixes (not implemented)

本轮不实施以下建议，只记录审计结果：

1. Keep a finite recovery guide as geometric authority, but do not silently use its consumed remaining time as the next rolling coverage horizon. Build a fresh rolling duration/reference from current authoritative P/V/A and target progress; retain the guide only for geometry and event identity. This must not execute an unsafe seed and must preserve final hard dynamic/static/swarm checks.
2. Add explicit guide refresh/consumption telemetry and a lifecycle guard that distinguishes “guide geometry still valid” from “guide has enough remaining execution horizon”. A stale/short guide may trigger a new guide request or ordinary safe local planning; it must not create an executable short successor merely to satisfy a guide hint.
3. Add a regression contract that runs several rolling replans before guide refresh and asserts that full polynomial duration, candidate duration, and validated authority are reported separately; it must reject truly stationary junk while preserving a moving successor and must leave FREE_TIME optimization available.

None of these recommendations lowers dynamic clearance, freezes T again, executes unvalidated suffix, adds a planner brake/stop fallback, or makes Local wait for Joint.

## 11. Final answers

1. **FREE_TIME_FROZEN 回退后 dynamic time avoidance 是否回来？** 部分回来。日志证明 rolling T/tau 重新变化，但本次运行没有足够证据证明一条动态冲突已经通过 T 修复并完成实际激活。
2. **collapse 是否重新出现？** 是。drone 2 candidate 1779 是首次明确的 MINCO/free-time 短候选，随后同一 finite guide 的 remaining 和 fresh duration 连续递减并造成 starvation/hold。
3. **collapse 第一次真正发生在哪个阶段？** 对 candidate 1779，fresh initializer 为 1.0 s，最终 nominal polynomial 为 0.440468 s，最早可定位在 `MINCO_FREE_TIME` 输出阶段；不是 reanchor、crop 或 validated-prefix 截断。
4. **Feedback 061 之后哪项修改改变了 duration/reference 语义？** 目前最有证据的是 Feedback 063 的 persistence/geometry-driven repeated replan 与 finite recovery-guide remaining 语义的交互。没有证据证明 previous optimized T 被直接递归复用，也没有闭合到唯一 commit。
5. **为什么统一重构前没有、Feedback 063 后才出现？** 不是因为旧版本绝对没有短候选；Feedback 061 已有孤立短候选。区别在于 Feedback 063 后同一 finite guide 在 refresh 前被更密集地重复消费，短 seed 形成连续链，最终耗尽 validated coverage，因而从孤立拒绝升级为 successor starvation 和 TERMINAL_HOLD。

本轮调查完成后停止；未继续设计第三套时间机制，未修改其他生产行为。
