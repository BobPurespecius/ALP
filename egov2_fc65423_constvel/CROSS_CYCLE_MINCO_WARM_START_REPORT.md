# ALP Cross-Cycle MINCO Warm-Start Counterfactual Report

审计目标：验证“上一周期优秀 accepted MINCO 解作为下一周期 warm start”是否能改善 obstacle 9 recovery。

本轮严格只复用现有源码与既有日志；未修改生产代码、未调参数、未运行仿真。

## 1. Phase-A counterfactual

现有日志中可以识别出至少三组“同一 UAV/obstacle，先出现 SIDE 安全结果，随后 recovery 周期失败”的真实 trace pair：

| Case | Source safe cycle | Next failed cycle | Gap | Source evidence | Missing for executable counterfactual |
|---|---|---|---:|---|---|
| 1 | `collision_repeat_20260901/baseline_episode_2_planner_trace.log`：UAV1 / obstacle9 / `SIDE_MINUS` selected；preceding `SCP_FINAL_OK`，duration `4.524832 s`，max jerk `14.300561`，dynamic clearance `2.367328` | 同 UAV/obstacle9；previous `SIDE_MINUS`，两侧失败，最终 selected `NOMINAL`；`SIDE_MINUS` final reason `DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`，duration `3.276875 s`，max jerk `21.957691`，clearance `1.717718` | ~`2.190 s` | planner trace 明确记录 selected/previous/final diagnostics | 上一周期 accepted polynomial coefficients、inner `P`、physical/virtual `T`、piece topology、Local-SFC/SIDE state 未导出 |
| 2 | `free_time_local_sfc_final_20260830.launcher.log`：UAV1 / obstacle9 / `SIDE_MINUS` selected；candidate18 `SCP_FINAL_OK`，duration `3.586966 s`，final jerk `18.003878`，clearance `1.614101`，`accepted=1` | 同 UAV/obstacle9；previous `SIDE_MINUS`，selected `NOMINAL`；candidate19 final reason `SCP_QP_FAILURE`，fresh initial duration `3.214261 s`，initial post-retiming jerk `18.246768`，clearance `-1`，`accepted=0` | ~`0.321 s` | launcher log 有完整 initial/final 与 acceptance 行 | 没有 candidate18 的 polynomial/P/T/topology/SFC 快照，无法把 candidate19 fresh 初值替换为 exact shifted accepted state |
| 3 | `jerk22_20260831.launcher.log`：UAV1 / obstacle9 / `SIDE_MINUS` selected | 约 `0.294 s` 后同 UAV/obstacle9 selected `NOMINAL`，失败 reason `QP_MAX_ITER_EXHAUSTED` | ~`0.294 s` | selected/failure 时序可识别 | 缺上一周期 accepted MINCO 的 exact state 与下一周期同约束 counterfactual 输入 |

另有 `trajectory_persistence_full_run3b_20260901.launcher.log` 中约 `0.288 s` 的同类 selected-safe → selected-NOMINAL/`QP_MAX_ITER_EXHAUSTED` trace，可作为重复时序证据，但同样不能执行严格 counterfactual。

### Why these are not executable Phase-A tests

严格的 Phase-A 要求在完全相同的 start state、target、obstacle prediction、SIDE/SFC/static/dynamics constraints 下，仅替换：

```text
fresh P/T  -> shifted previous accepted P/T/topology
```

当前日志没有保存 accepted candidate 的完整 polynomial coefficients 或 inner control points、physical/virtual durations、piece partition、SIDE/A*/Local-SFC topology。因此无法重建同一下一周期的 warm-start 输入，也无法公正计算 fresh 与 warm 的对照。

## 2. Warm-start hypothesis verdict

现有证据可以证明相邻周期之间存在高相关的 recovery 失败，但不能证明 warm start 会成功，也不能证明它会失败。不能从“上一周期 selected SIDE”反推出下一周期可复制的 MINCO 状态。

无法诚实计算以下指标：

- `FRESH_INIT_MAX_JERK`
- `WARM_INIT_MAX_JERK`
- `FRESH_SCP_RESULT` vs `WARM_SCP_RESULT`
- recovery feasibility/latency difference

因此本阶段结论是：

```text
WARM_START_COUNTERFACTUAL_SUCCESS: 0 / 0
WARM_START_HYPOTHESIS_CONFIRMED: INCONCLUSIVE
```

`0 / 0` 表示没有可执行的 counterfactual pair，不表示 warm start 已被证伪。

## 3. 源码级现状

### `computeInitState()`

`planner_manager.cpp::computeInitState()` 的 previous-trajectory 分支会重新采样 inner points、按当前距离重新计算 `piece_nums`、重新均匀分配 durations，并对 SIDE candidate 重新施加 side offset，随后 `reset()` + `generate()`。这提供执行起点状态的 continuity，但不是复制上一 accepted MINCO 解。

结论：

```text
EXECUTED_STATE_USED_AS_START: YES (normal local-replan path)
EXACT_ACCEPTED_P_INHERITANCE: NO
EXACT_ACCEPTED_T_INHERITANCE: NO
```

在 local trajectory 过期、tracking error 超阈值或退回 global planning 时，连当前执行轨迹起点也不是绝对保证，故 P/V/A continuity 对所有路径只能记为 `PARTIAL`。

### Current candidate SCP

`runCandidateHardCorridorSCP()` 在 candidate 内部使用 free-time `[P, virtual_T]` continuation；这不是跨 replanning cycle 的 continuation。下一周期的 `initInnerPts/initT` 仍来自 `computeInitState()` 的重建。

### MINCO reset/generate

`MinJerkOpt::reset()` 会重新设置 `N/head/tail` 并 resize 内部矩阵；`generate()` 重新计算 polynomial coefficients。当前类没有“shift previous accepted trajectory”或“inherit previous accepted P/T”接口。赋值 operator 只复制对象内部状态，不构成上层跨周期 warm-start protocol。

### OSQP warm start

`SCPOptimizer::solve()` 开启了 `settings.warm_start = 1`，但每次 solve 都重新 `osqp_setup()`，求解后 `osqp_cleanup()`。因此这是单次 workspace 的开关，不是跨 candidate 或跨 replanning cycle 的 trajectory/MINCO warm start。

## 4. What the existing logs do show

既有 obstacle-9 记录支持：

- persistence 修复后仍有重复的 selected SIDE → 后续 recovery failure 时序；
- 后续失败周期通常重新生成更短或不同的 duration/initial jerk；
- 失败 reason 包括 `DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`、`SCP_QP_FAILURE`、`QP_MAX_ITER_EXHAUSTED`；
- 这些现象与“accepted MINCO 的 P/T/topology 没有跨周期继承”一致。

但这是源码级机制与相关性证据，不是 warm-start 的因果 A/B 证据。当前数据不能区分：

1. exact warm-start 能否降低 initial jerk；
2. warm-start 是否能让 obstacle 9 recovery 进入可行 basin；
3. 失败主要来自重初始化、预测时序变化、几何约束变化，还是 OSQP/max-iter；
4. accepted candidate 的完整状态是否仍适用于下一周期。

## 5. Phase-B decision

由于 Phase-A 为 `INCONCLUSIVE`，按任务要求不进入生产实现，不运行 x3 warm-start 验证，也不改变当前 planner 控制流。

```text
PRODUCTION_WARM_START_IMPLEMENTED: NO
ONLY_ACCEPTED_TRAJECTORY_CACHED: NO
P_INHERITED_ACROSS_CYCLES: NO
T_INHERITED_ACROSS_CYCLES: NO
TOPOLOGY_PREFERRED_ACROSS_CYCLES: NO
CURRENT_EXECUTION_TIME_SHIFT_USED: YES (persistence/start-state path only)
TIME_REWIND: NO EVIDENCE
FRESH_INIT_FALLBACK_PRESERVED: YES
SCP_CORE_CHANGED: NO
TRUST_CHANGED: NO
OSQP_CHANGED: NO
JERK_LIMIT_CHANGED: NO
```

## 6. Required missing artifact for the next step

要使 Phase-A 可执行，下一步只需增加 accepted-candidate state export/cache（不应在本报告阶段实现）：

- accepted polynomial coefficients，或等价 inner `P`；
- accepted physical durations 与 virtual `T`；
- piece count/segment partition；
- SIDE branch、A* guide/turning points、Local-SFC planes/active interval；
- 与下一周期相同的 start/target/prediction/constraint snapshot；
- fresh 与 shifted-warm 两次 SCP 的 initial/final jerk、QP status、最终 acceptance。

拿到这些数据后，才能做一次严格的单变量 counterfactual，再决定是否进入生产 warm-start 实现。

## 7. Final labels

```text
COUNTERFACTUAL_CASES: 0 executable / 3 identified trace pairs
WARM_START_COUNTERFACTUAL_SUCCESS: 0 / 0
WARM_START_HYPOTHESIS_CONFIRMED: INCONCLUSIVE
PRODUCTION_WARM_START_IMPLEMENTED: NO
ONLY_ACCEPTED_TRAJECTORY_CACHED: NO
P_INHERITED_ACROSS_CYCLES: NO
T_INHERITED_ACROSS_CYCLES: NO
TOPOLOGY_PREFERRED_ACROSS_CYCLES: NO
CURRENT_EXECUTION_TIME_SHIFT_USED: YES (persistence only)
TIME_REWIND: NO evidence
FRESH_INIT_FALLBACK_PRESERVED: YES
SCP_CORE_CHANGED: NO
TRUST_CHANGED: NO
OSQP_CHANGED: NO
JERK_LIMIT_CHANGED: NO
OBSTACLE9_COLLISION_BEFORE: 3 / 3 in post-persistence complete runs
OBSTACLE9_COLLISION_AFTER: NOT TESTED
OBSTACLE9_IMPROVEMENT: INCONCLUSIVE
INITIAL_JERK_REDUCTION: NOT MEASURED
STATIC_COLLISION_REGRESSION: NOT TESTED
TRACKING_REGRESSION: NOT TESTED
VISIBILITY_REGRESSION: NOT TESTED
PLANNING_LATENCY_REGRESSION: NOT TESTED
BUILD: NOT RUN (no production change)
RUNTIME: NOT RUN
NEXT_BLOCKER_IF_ANY: missing exact accepted trajectory P/T/polynomial/topology export for Phase-A counterfactual
```

## Bottom line

当前确实存在跨周期 accepted MINCO 状态未继承的问题，但现有日志不足以把它从“合理根因”提升为“已验证改善方案”。因此本轮正确停止点是：保留 fresh initialization，不修改生产 warm-start，先补齐可重放的 accepted P/T/polynomial/topology state，再做唯一缺失的 Phase-A counterfactual。
