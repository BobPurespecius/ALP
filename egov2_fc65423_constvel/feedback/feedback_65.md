# Feedback 065 - Dynamic Candidate Rejection / Successor Starvation Deep Audit

## 0. Scope and protection

本轮只读审查，工作区为 `/home/bob/ALP/egov2_fc65423_constvel`。未访问 `/home/bob/RRCT`，未启动仿真，未修改 planner、lifecycle、dynamic checker、MINCO/SCP、SIDE、recovery、A*、traj_server、controller、launch 或参数。

工作树在审查前已经包含 Feedback 063/064 及更早用户修改，未做 reset、checkout 或覆盖。`rolling_fix_full.log` 保留，大小约 22 MB。当前没有残留 ROS/RViz/planner 进程。

## 1. Executive conclusion

两个真实急停都属于 coverage hole：前一条执行 authority 到期时，下一条没有通过完整执行安全门，因此 traj_server 进入 `TERMINAL_HOLD`。predecessor 仍在运动，且 terminal velocity 不是零；这不是 planner 主动把正常 successor 变成停车轨迹，也不是 traj_server 在已有 successor 到期前主动插入 brake。

对代表性 nominal/recovery candidate，日志中最早可见的 `DYNAMIC_FAIL` 出现在 `finalizeCapturedCandidates()` 的 current-revision final preflight：candidate 已有 fresh/warm initializer、已完成 MINCO，随后 `prepareLocalHandoff()` 重锚，再由 `validateExecutionTrajectory()` 的动态硬门拒绝。源码同时存在一个更早的 SIDE 动态 usefulness/backoff gate；它确实限制了部分 SIDE 搜索，但本次两个 TERMINAL_HOLD 的主证据不足以证明所有失败都在该 gate 前发生。

因此最终分类为：

```text
SEARCH_BLOCK_CLASSIFICATION: CASE_B (PARTIAL SEARCH BLOCK)
PRIMARY_ROOT_CAUSE: dynamic hard final rejection + recovery search not expanding an executable local solution before validated coverage expires
SECONDARY_ROOT_CAUSE: rolling local time is frozen and dynamic obstacle safety is soft-cost/post-check only; no complete local dynamic P/T repair contract
IS_SELF_CREATED_LOGICAL_BLOCK: PARTIAL
```

不能据现有日志证明全局物理无解，也不能证明存在一个已被当前局部搜索覆盖的安全 P/T 解。正确边界是 `NO_SAFE_SOLUTION_FOUND_IN_AUDITED_LOCAL_SEARCH_REGION`。

## 2. Two stop-event timelines

### Event 1: `1789394463.348`

前一条 authority 是 trajectory `149`，start `1789394461.846508265`，validated end `1789394463.346508265`。在 `1789394463.248`，日志已经显示 `validated_coverage_remaining=0.098430`、`planning_budget_remaining=-0.001570`，随后 post-deadline recovery 仍以 odometry P/V/A 继续尝试。真实 recovery head 为：

```text
p=(2.311883,1.536821,1.503264)
v=(0.601984,0.280760,0.000080)
a=(0,0,0)
```

在此前的正常 rolling cycle，candidate `190, 192, 194, ...` 已完成 fresh/warm initializer、MINCO、`SUCCESSOR_REANCHORED_TO_LATEST_ACTIVATION`，但每个 `local-geometry-candidate` 均为 `safe=0 reason=DYNAMIC_FAIL`。围绕事件 1 的日志窗口 `1789394460..1789394464` 共记录 drone 1 的 187 个 local-geometry candidate，candidate id 从 `190` 到 `512`，全部 `DYNAMIC_FAIL`，没有 `safe=1`。

`1789394463.348135608` 发生：

```text
TERMINAL_HOLD_ENTER trajectory_id=149 end_time=1789394463.346508265
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION
```

随后 recovery candidate `446,447,448,...` 继续失败。急停前 predecessor 速度约 `0.6 m/s`，所以分类为 `COVERAGE_HOLE`，不是 `PLANNED_TERMINAL_DECEL`。

### Event 2: `1789394477.208`

前一条 authority 是 trajectory `171`，validated end `1789394477.203114510`。到期前后 coverage 已为零，`1789394477.212` 的 post-deadline recovery head 为：

```text
p=(9.945629,2.390397,1.499917)
v=(0.379015,0.071004,-0.000016)
a=(0,0,0)
```

`1789394477.207781086` 发生 `TERMINAL_HOLD_ENTER`，随后 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`。事件 2 窗口中，drone 1 的 local-geometry candidate 从 `1087` 起持续以 `DYNAMIC_FAIL` 结束；在 post-deadline recovery 的明确样本中，candidate `1292,1293` 等都由 fresh initializer 生成后立即进入相同失败路径。日志抽取窗口 `1789394476..1789394479` 共记录 176 个 drone 1 local-geometry candidate，全部为 `DYNAMIC_FAIL`，没有 `safe=1`。

事件 2 的 recovery fresh initializer 并非零时长垃圾：例如 candidate `1292/1293` 使用 5-piece、约 `5.783502 s` 的 moving initializer，`plan_success=1`，但最终仍被动态硬门拒绝。因此该事件不是 SHORT_STATIONARY 或 terminal velocity 构造问题。

## 3. First dynamic-fail stage and call chain

### Production call chain

```text
candidate construction / fresh or warm initializer
  -> MINCO optimizer (nominal or SIDE path)
  -> prepareLocalHandoff()                 [planner_manager.cpp:1432]
  -> retryTrajectoryDynamics()             [planner_manager.cpp:1494]
  -> retryTrajectoryStaticSafety()         [planner_manager.cpp:1502]
  -> validateRetimedLocalSfc()/handoff
  -> validateExecutionTrajectory()         [planner_manager.cpp:1511]
       -> checkTrajectoryDynamics()
       -> checkTrajectoryStaticSafety()
       -> evaluateDynamicRisk(trajectory, activation, true)
       -> min_distance < moving_obj_clearance
       -> reason = DYNAMIC_FAIL
  -> candidate discarded; no commit/activation
```

### Required fields

```text
STOP_EVENT_1_TIME: 1789394463.348135608 (server log; trajectory expiry 1789394463.346508265)
STOP_EVENT_2_TIME: 1789394477.207781086 (server log; trajectory expiry 1789394477.203114510)

FIRST_DYNAMIC_FAIL_STAGE_EVENT_1: FINAL_PREFLIGHT / POST_REANCHOR_RECHECK
FIRST_DYNAMIC_FAIL_STAGE_EVENT_2: FINAL_PREFLIGHT / POST_REANCHOR_RECHECK
FIRST_DYNAMIC_FAIL_FUNCTION: EGOPlannerManager::validateExecutionTrajectory()
FIRST_DYNAMIC_FAIL_CONDITION: !risk.valid OR risk.min_distance < getMovingObjClearance()
RETURN_OR_CONTINUE_BEHAVIOR: return false; finalizeCapturedCandidates() logs local-geometry-candidate safe=0 and continues to next candidate; no execution authority is created
```

The log does not emit a per-candidate `risk.obstacle_id`, `risk.min_distance`, or `risk.conflict_time` at this final rejection point. Therefore the exact offending obstacle for the two hold events cannot be reconstructed from the retained telemetry alone.

## 4. Early dynamic gates

`planner_manager.cpp:6700-6708` evaluates `trial_risk` for SIDE seeds before the static check and only treats a seed as dynamically useful when it is above the absolute clearance (topology trigger) or improves nominal clearance (non-topology path). Later, `planner_manager.cpp:8441-8444` permits a colliding initializer to reach hard SCP only under the narrow predicate:

```text
hard_corridor_scp_enabled && astar_repair_applied && !local_sfc_planes.empty()
&& side_init_valid && side_dynamic_valid
```

Ordinary dynamic-invalid SIDE seeds do not receive an equivalent `UNSAFE_SEED_FOR_REPAIR_ONLY` channel. A dynamic-invalid seed can therefore be rejected before a full SIDE repair path, even though the final execution gate is correctly hard.

```text
EARLY_DYNAMIC_GATE_EXISTS: YES
EARLY_GATE_BLOCKS_P_OPTIMIZATION: PARTIAL
EARLY_GATE_BLOCKS_T_OPTIMIZATION: PARTIAL
EARLY_GATE_CAN_REJECT_REPAIRABLE_SEEDS: YES/PARTIAL
```

The qualification matters: the two stop windows primarily show nominal/recovery candidates reaching the post-handoff final gate. They do not prove that the early SIDE gate was the first failure for every failed candidate.

## 5. Dynamic safety mechanisms and repair authority

| Mechanism | Stage | Uses P | Uses T | Hard/Soft | Can repair an unsafe seed? |
|---|---|---:|---:|---|---|
| `movingObjGradCostP()` | MINCO/SCP objective | yes | gradient exists | soft | spatially may influence, not guarantee |
| `movingObjGradCostP()` `gradt`/`grad_prev_t` | optimizer derivative | indirect | yes in formula | soft | rolling T trust region is frozen for `!touch_goal` |
| `checkMovingObjSafety()` | post-optimizer sampled checker | yes | via sampled local time | hard post-check | only reject |
| `evaluateDynamicRisk()` | manager precheck/final preflight | yes | query uses activation + local time | hard gate | only reject |
| `validateExecutionTrajectory()` | execution admission | yes | yes | hard | only reject |
| time-only inter-UAV correction | SIDE/team-specific path | fixed inner P | piece durations | hard for its own contract | not a general moving-obstacle repair |
| Local-SFC | static A*/collision corridor | yes | retime mapping | hard geometric rows | static collision only; no dynamic obstacle SFC |

There is no dynamic-obstacle-specific hard SCP row in the audited optimizer. `movingObjGradCostP()` supplies a soft Euclidean hinge and derivatives, while `checkMovingObjSafety()` and `evaluateDynamicRisk()` are posterior gates. The existing time-only correction is not evidence that local dynamic-obstacle crossing time can be repaired: rolling free-time SCP sets duration trust to zero when `!touch_goal` (`poly_traj_optimizer.cpp:1276`, `2458-2461`, `2524-2531`, `2648-2653`).

```text
LOCAL_DYNAMIC_OBSTACLE_TIME_REPAIR_ACTUALLY_EXISTS: NO for rolling successors; PARTIAL only for narrow SIDE/time-only contracts
LOCAL_DYNAMIC_OBSTACLE_SPACE_REPAIR_ACTUALLY_EXISTS: PARTIAL (soft P cost plus bounded post-check repair, no dynamic hard corridor)
```

The optimizer can alter P in its normal spatial optimization. It cannot make an unsafe rolling seed execution-eligible merely because the soft dynamic cost decreased; final clearance is still independently required.

## 6. Recovery search-space audit

The retained log shows recovery attempts repeatedly using the same expired predecessor and nearly unchanged current P/V/A. Event 1 has a dense sequence of candidates roughly every 10–12 ms after candidate 190 and after expiry; event 2 repeats the same fresh initializer construction (including the same 5-piece, 5.783502 s shape in adjacent attempts) after the predecessor is already expired.

```text
RECOVERY_SEARCH_SPACE_EXPANDED_EVENT_1: NO/PARTIAL
RECOVERY_SEARCH_SPACE_EXPANDED_EVENT_2: NO/PARTIAL
```

This is a telemetry limitation as well as a design finding: the current log does not print a stable per-candidate seed hash, guide offset, optimized duration, or dynamic-clearance tuple. It proves repeated failure and repeated recovery context, but not a precise Euclidean delta for every attempt. A future diagnostic should add those fields before claiming quantitative search diversity.

For the same reason:

```text
NLR_SPATIAL_DIVERSITY: UNKNOWN/LOW in the two stop windows (no complete N/L/R per-candidate telemetry)
NLR_TEMPORAL_DIVERSITY: LOW for observed nominal/recovery retries; adjacent candidates preserve nearly identical activation construction
NLR_DYNAMIC_CLEARANCE_DIVERSITY: UNKNOWN (final gate does not log risk distance)
```

The absence of `candidate-final-status`, `side-full-scp-refinement`, or `side-feasible-fallback` records for the representative nominal failures means the retained evidence cannot support the stronger claim that a full SIDE SCP repair was attempted and failed. It does support that nominal/recovery candidates were optimized and then rejected at final dynamic admission.

## 7. Dynamic prediction time semantics

`evaluateDynamicRisk()` (`planner_manager.cpp:4151+`) uses:

```text
query_time = prediction_epoch + sample_time
```

and queries `evaluateConstVel(id, query_time)`. `finalizeCapturedCandidates()` chooses a common activation epoch (`planner_manager.cpp:1371-1376`), `prepareLocalHandoff()` reanchors the candidate, and the final risk check uses that activation. `finalizeTopologyCandidate()` also resets `candidate.prediction_epoch = activation` and reevaluates risk (`planner_manager.cpp:3564-3571`). No concrete stale-snapshot, missing activation addition, or reanchor local/world-time mismatch was found in source or event logs.

```text
DYNAMIC_TIME_ALIGNMENT_BUG: NO EVIDENCE / NOT CONFIRMED
```

However, dynamic failure telemetry is too sparse to independently verify the query time and obstacle pose for each rejected candidate. This is an observability gap, not evidence of a timing bug.

## 8. Dynamic-fail label classification

The source distinguishes missing predictions inside `evaluateDynamicRisk()` by returning an invalid risk, and final admission collapses both invalid risk and insufficient clearance into `DYNAMIC_FAIL` (`planner_manager.cpp:1999-2001`). The retained log does not expose which branch was taken for the two events.

```text
TRUE_DYNAMIC_BODY_FAIL_COUNT: UNKNOWN from current telemetry
PREDICTION_DATA_FAIL_COUNT: UNKNOWN
TIME_ALIGNMENT_FAIL_COUNT: 0 observed / not proven
OTHER_DYNAMIC_FAIL_COUNT: UNKNOWN
DYNAMIC_FAIL_LABEL_TOO_BROAD: YES (at final admission; invalid prediction and clearance violation are merged)
```

The scene had a small number of predicted moving objects, but the final rejection logs do not preserve the offending object ID. It is therefore unsafe to assert that one specific obstacle caused both events.

## 9. Local counterfactual conclusion

No production trajectory was executed or altered by an offline counterfactual in this read-only round. Existing logs provide no per-candidate polynomial samples or obstacle witness sufficient to perform a defensible P/T clearance grid for the two exact events. Consequently:

```text
TIME_SHIFT_SAFE_SOLUTION_EXISTS_EVENT_1: UNKNOWN
TIME_SHIFT_SAFE_SOLUTION_EXISTS_EVENT_2: UNKNOWN
SPATIAL_SAFE_SOLUTION_EXISTS_EVENT_1: UNKNOWN
SPATIAL_SAFE_SOLUTION_EXISTS_EVENT_2: UNKNOWN
LOCAL_PT_SAFE_SOLUTION_EXISTS_EVENT_1: UNKNOWN
LOCAL_PT_SAFE_SOLUTION_EXISTS_EVENT_2: UNKNOWN
```

The only defensible statement is `NO_SAFE_SOLUTION_FOUND_IN_AUDITED_LOCAL_SEARCH_REGION`; this is not a claim of global physical infeasibility.

## 10. Budget and starvation

Event 1 had only `0.098430 s` validated coverage remaining when recovery authority was already late. Event 2 entered recovery with zero validated coverage and increasingly negative planning budget. The individual optimizer calls shown in the log are short (sub-millisecond to a few milliseconds), but repeated candidate attempts continue after the predecessor deadline and no candidate is admitted. Thus:

```text
DYNAMIC_FAILURE_BUDGET_LIMITED: YES
```

The dominant budget failure is not one expensive MINCO call; it is repeated dynamic rejection after the available validated execution authority is exhausted. Event 2 additionally demonstrates that post-deadline recovery can keep spending effort on the same class of candidate for seconds without producing an executable successor.

## 11. Relationship to Feedback 052-056 and 061-064

- Feedback 052-053 addressed validated coverage supply, deadline and first-safe/quality separation; healthy runs showed zero hold, but did not prove dynamic-conflict recovery.
- Feedback 054-055 recorded the first dynamic-safety rejection and post-deadline liveness family. Feedback 055 removed a permanent recovery trap but explicitly retained cases where dynamic candidates remain unsafe.
- Feedback 056 separated validated-prefix authority from full polynomial duration and repaired guide/initializer authority. The current events respect that distinction: the hold occurs at validated end, not full unvalidated suffix.
- Feedback 061-062 concern atomic Joint/visibility topology and design audit; they do not add dynamic-obstacle hard repair.
- Feedback 063 unified BODY/LOS topology but reported dynamic/hold regressions. Its new topology path does not remove the final dynamic hard gate.
- Feedback 064's short-window conclusion is insufficient for these events: the current log contains two real hold episodes with long repeated post-deadline failure sequences.

Therefore this is the same broader successor/liveness family as Feedback 52-56, with a new dynamic-rejection mechanism dominating the two observed episodes. It is not evidence that terminal P/V/A or the traj_server itself is the primary root cause.

## 12. Recommended fixes (design only, not implemented)

### RECOMMENDED_FIX_1: explicit repair-only dynamic candidate state

涉及 `planner_manager.cpp` candidate lifecycle and `CandidateResult` metadata. Preserve the hard final admission unchanged, but distinguish `UNSAFE_SEED_FOR_REPAIR_ONLY` from `EXECUTION_ELIGIBLE`. A seed below dynamic clearance may enter a bounded repair path only; it must have no commit, publication, validated prefix, or executor authority until a fresh activation-time hard preflight passes. Prediction validity/staleness remains a hard rejection. This is the smallest way to prevent an early seed gate from being confused with execution safety.

### RECOMMENDED_FIX_2: make local dynamic repair capability explicit

Either add a narrowly scoped dynamic hard SCP row with correct fixed-world-time P Jacobian and an explicitly bounded activation-time sample set, or document that only spatial soft repair exists and add bounded alternative seed generation. Do not simply delete the early gate, do not use A* for dynamic obstacles, and do not add a dynamic collision SFC without proving it is needed. Rolling T must remain frozen unless a separate validated contract proves that time changes preserve activation, prediction epoch, and all hard checks.

### RECOMMENDED_FIX_3: telemetry and bounded recovery diversity

Add read-only diagnostics first: per candidate stage, `risk.valid`, `obstacle_id`, `min_distance`, `conflict_time`, prediction query time, seed/optimized duration, P/V/A head, guide side/offset, and rejection stage. Bound repeated recovery attempts by changing a documented existing spatial or temporal dimension, while preserving the same hard safety gate and Local-first nonblocking lifecycle. This is required before claiming physical infeasibility or N/L/R diversity.

Regression requirements for all options: retain Feedback 55 post-deadline recovery, validated-prefix authority, current-revision revalidation, no unsafe execution, no unvalidated suffix, no Local waiting for Joint, BODY/LOS topology contracts, dynamic prediction stale/missing tests, and a deterministic `TERMINAL_HOLD` starvation fixture.

## 13. Required final fields

```text
EARLY_DYNAMIC_GATE_EXISTS: YES
EARLY_GATE_BLOCKS_P_OPTIMIZATION: PARTIAL
EARLY_GATE_BLOCKS_T_OPTIMIZATION: PARTIAL
EARLY_GATE_CAN_REJECT_REPAIRABLE_SEEDS: YES/PARTIAL

LOCAL_DYNAMIC_OBSTACLE_TIME_REPAIR_ACTUALLY_EXISTS: NO (rolling); PARTIAL (narrow time-only path)
LOCAL_DYNAMIC_OBSTACLE_SPACE_REPAIR_ACTUALLY_EXISTS: PARTIAL

RECOVERY_SEARCH_SPACE_EXPANDED_EVENT_1: NO/PARTIAL
RECOVERY_SEARCH_SPACE_EXPANDED_EVENT_2: NO/PARTIAL

NLR_SPATIAL_DIVERSITY: UNKNOWN/LOW
NLR_TEMPORAL_DIVERSITY: LOW
NLR_DYNAMIC_CLEARANCE_DIVERSITY: UNKNOWN

DYNAMIC_TIME_ALIGNMENT_BUG: NO EVIDENCE / NOT CONFIRMED
DYNAMIC_FAIL_LABEL_TOO_BROAD: YES

TRUE_DYNAMIC_BODY_FAIL_COUNT: UNKNOWN
PREDICTION_DATA_FAIL_COUNT: UNKNOWN
TIME_ALIGNMENT_FAIL_COUNT: 0 observed / not proven

DYNAMIC_FAILURE_BUDGET_LIMITED: YES
SEARCH_BLOCK_CLASSIFICATION: CASE_B

PRODUCTION_SOURCE_CHANGED: NO
PLANNER_BEHAVIOR_CHANGED: NO
LIFECYCLE_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
SIMULATION_RUN: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

## 14. Six direct answers

1. `DYNAMIC_FAIL` 第一次出现在本次代表性 candidate 的 `FINAL_PREFLIGHT / POST_REANCHOR_RECHECK`，函数是 `validateExecutionTrajectory()`；SIDE 另有更早 gate，但不是两个事件全部失败的已证实首点。
2. nominal candidate 在被拒前有 P 优化机会；rolling successor 的 T 优化实际被冻结。动态安全没有进入一个能保证修复的 hard SCP row，最终只能由 soft cost影响、由后验硬 checker裁决。
3. recovery 在两个事件中主要重复相似的 current-state/fresh-initializer 失败候选；事件 2 的重复性尤其明确，但缺少 seed hash 和风险数值，不能伪造精确 delta 统计。
4. 源码中的 world-time 关系是 `prediction_epoch + local sample_time`，reanchor/finalization 会以 activation 重算；目前没有具体 time-alignment bug 证据。
5. 现有证据支持“planner 在审计局部搜索区域内没有找到安全 successor”，不支持“物理全局无解”。
6. 是，部分构建了“先要求动态安全才允许进入可能修复阶段”的逻辑限制：SIDE early gate 明确存在；但本次主路径还叠加了完整优化后的 final hard rejection，因此严格判定为 `CASE_B / PARTIAL SEARCH BLOCK`，不能用移除一个 gate 概括解决。

## 15. Status

```text
PRODUCTION_SOURCE_CHANGED: NO
PLANNER_BEHAVIOR_CHANGED: NO
LIFECYCLE_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
SIMULATION_RUN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
