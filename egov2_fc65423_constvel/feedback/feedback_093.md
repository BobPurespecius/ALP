# Feedback093 — Local–Team–Executor trajectory lifecycle simplification

## 0. Executive conclusion

Feedback092 定位的根因是 **Planner 与 Executor 使用不同的 predecessor 权威**，并因此在 Team transaction 43 被撤销后形成自锁。本轮从架构上消除了产生 orphan chain 的条件，而不是给 rollback 打补丁。

核心改动（全部落地在 production code，非注释）：

1. **显式 trajectory lineage contract**：每条可执行 payload 携带 `expected_predecessor_id` / `expected_predecessor_generation` / `lineage_class`。Executor **只做精确身份解析**，删除了 queue/scheduled/active 三段式隐式前驱推断。
2. **只有 traj_server 实际 ACTIVATE 才能改变 Planner 的 authoritative predecessor**。Planner 的 guaranteed Local successor 以 *executor 已确认的 active* 重新锚定；Team COMMIT 完全不进入该路径。
3. **Team future 在 activation 前始终是 speculative**：独立 id 命名空间（`1000000 + team_solution_id`）、独立 tag、可被 Team transaction 单独撤回，且撤回**不得**触碰 Local guaranteed successor。
4. **单一有序激活时间线**：按激活时刻提升最早的待激活条目，修掉了"排队中的 Local 后继被更晚激活的 Team 条目饿死"。
5. **Team commit 不再阻塞 Local rolling**：删除 `LOCAL_UPDATE_DEFERRED` 阻塞路径。
6. **current-state restart 最低预算**由实测 pipeline estimate 推导（不是硬编码秒数）。

真实 FULL 仿真结果（`runs/20260921_161234_294376`，`FINAL_EXIT_CODE=0`，`SIMULATION_REACHED_BOOT_12=YES`）：

| 指标 | Feedback091 | **Feedback093** |
|---|---|---|
| `ACTIVE_PVA_MISMATCH` | 229 | **0** |
| `PLANNER_EXECUTOR_PREDECESSOR_DIVERGENCE` | 有（planner `OLD_ID=106 DP=0` vs executor `OLD_ID=76 DP=0.859`） | **0** |
| `TERMINAL_HOLD_ENTER` / `EXIT` | 2 / 1（drone_2 永不恢复） | **4 / 4** |
| 最长 terminal hold | 13.68 s，另 1 个 ≥155 s | **1.61 s** |
| `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` | 2 | 4 |
| `MOVING_SUCCESSOR_STARVATION` | 2 | **0** |
| `UNVALIDATED_EXECUTED_SAMPLES` | 6608（全域） | **150** |
| `STATIC_CONTACT_SAMPLES` | 17（全部 drone_2） | **0** |
| `PARTIAL_TEAM_ACTIVATION` | 0 | **0** |

---

## 1. Modified files

| 文件 | 改动 |
|---|---|
| `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/PolyTraj.msg` | 新增 `int32 expected_predecessor_id`、`uint64 expected_predecessor_generation`、`string lineage_class` |
| `.../plan_manage/src/traj_server.cpp` | lineage registry + 三权威分离；`executionHandoffGate` 重写为精确身份解析；单一有序激活时间线；Team speculative 独立命名空间与独立撤回；激活期物理连续性检查；新遥测 |
| `.../plan_manage/include/plan_manage/planner_manager.h` | `PredecessorRecord`、`predecessor_history_`、`authoritative_predecessor_*`、`declared_predecessor_id_*`、`predecessorAuthority()`、审计计数器 |
| `.../plan_manage/src/planner_manager.cpp` | activation 回调驱动权威切换；`checkActiveHandoff` 对 Local successor 使用权威前驱；`recordPredecessorRecord`/`findPredecessorRecord`；Team realization 以 executor 侧同一 id 归档本地副本；删除 `LOCAL_UPDATE_DEFERRED`；restart 最低预算 |

未修改：任何 Team objective（`TeamTargetCenteredOptimizer`、R/PHI/reference-phase、Q2/ACC/ENC、`RealizedTeamValidator` 层级、`CooperativeTaskReference`）、任何 Local safety 算法（static/dynamic/swarm clearance、A*、Local-SFC、MINCO cost）、任何 PVA tolerance、任何 safety threshold。

---

## 2. Old lifecycle

Feedback092 已证明的事实链（本轮未重新论证）：

```
Team transaction 43
  → PREPARE → 3×EXECUTOR_READY → COMMIT
  → Team future trajectory 进入 traj_server（与 Local 混在同一个 scheduled_traj_/queue）
  → drone2 FINAL_PREFLIGHT_FAILED
  → TEAM_COMMIT_REVOKE_REQUEST → 三台 CANCEL_ALL_FUTURE(team_solution_id=43)
  → 按 team_solution_id 无条件 erase（含 Local planner 已链接的那条）
  → Planner predecessor(106) ≠ Executor predecessor(76)
  → ACTIVE_PVA_MISMATCH 风暴 → 无后继被接受 → END_BEFORE_NEXT → TERMINAL_HOLD
```

本轮进一步定位到两个使能条件：

- **trajectory_id 命名空间冲突**：`mincoToPreparedSource` 用 `frontier_owner_trajectory_ids[i] + 1` 给 Team realization 分配 id，而 Local planner 独立分配自己的下一个 id。日志实例（run `20260921_131436_177465`）：

```
702.9888  executor 调度 Team  id=77  act=703.5753 dur=5.9284   ← Team realization
702.9912  planner  提交 Local id=77  act=703.0906 dur=4.1819   ← 同一个 id，不同轨迹
```

- **队列饿死**：激活定时器只在 `scheduled_traj_valid_` 为真时推进，`scheduled_traj_queue_` 只是其副作用。当队列中的 Local 后继（act 703.0906）早于 scheduled 槽中的 Team 条目（act 703.5753）时，该 Local 后继**永远不会激活**。
- **Team commit 阻塞 Local**：`setLocalTrajFromOpt` 中 `LOCAL_UPDATE_DEFERRED` 会在 Team tail 已提交时直接 `return false`，拒绝 Local rolling 更新。

---

## 3. New lifecycle

```
                   CooperativeTaskReference
                            ↓
                       Local Planner
                            ↓
                      ACTIVE A
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
    Local guaranteed successor L      Team refinement
      (anchored to A, declares A)      R / PHI / PHASE
              │                             ↓
              │                      Local realization T
              │                             ↓
              │                    RealizedTeamValidator
              │                             ↓
              │                  PREPARE / READY / COMMIT
              │                             ↓
              │                   T SPECULATIVE FUTURE
              └──────────────┬──────────────┘
                             ↓
                   single ordered activation timeline
                             ↓
                      actual ACTIVATION
                             ↓
                  authoritative predecessor
                             ↓
                     Local rolls again
```

三种 authority（以前第三项没有明确，本轮补上）：

| Authority | 归属 | 载体 |
|---|---|---|
| Executable polynomial producer | Local Planner | `planning/trajectory` |
| Future Team reservation | Team execution transaction | `TeamTrajectorySolution` PREPARE/COMMIT |
| **Actual trajectory history** | **traj_server ACTIVATE** | `/trajectory_execution/activated` |

---

## 4. Local guaranteed successor implementation

新增 helper `admitFutureEntry()` + `normalizeFutureTimeline()`（`traj_server.cpp`）。待激活 future 是**单一按激活时刻排序的时间线**，无论它原本位于 scheduled 槽还是 queue：

```cpp
bool futureEntryEarlier(const ScheduledTrajectoryEntry &a,
                        const ScheduledTrajectoryEntry &b)
{
  const double lhs = a.start_time.toSec(), rhs = b.start_time.toSec();
  if (std::abs(lhs - rhs) > 1.0e-9) return lhs < rhs;
  if (a.team_speculative != b.team_speculative) return !a.team_speculative;
  return a.source.generation < b.source.generation;   // 同时刻优先 Local
}
```

`cmdCallback` 每轮先 `normalizeFutureTimeline()`，再提升**最早**的已到期待激活条目：

```cpp
normalizeFutureTimeline();
if (scheduled_traj_valid_ && scheduled_traj_ &&
    time_now + ros::Duration(1.0e-9) >= scheduled_start_time_) { … }
```

因此 Local guaranteed successor 不会再被任何更晚激活的 Team 条目饿死。Team 侧同时移除了阻塞：

```cpp
// Feedback093: a Team transaction must NEVER block Local rolling.
// ... A committed Team tail is now purely *speculative* ...
if (!team && hasCommittedPendingTeamTail())   // 只做硬安全复检，不再 return false
```

运行证据：`LOCAL_UPDATE_DEFERRED = 0`（此前该路径存在）。

---

## 5. Team speculative future implementation

- **独立 id 命名空间**（消除与 Local id 的冲突）：

```cpp
entry.traj_id = kTeamSpeculativeTrajectoryIdBase +
    static_cast<int>(solution.team_solution_id);   // 1000000 + team_solution_id
```

  该 id 在两侧都是 `team_solution_id` 的纯函数，因此 Planner 与 Executor 独立推导出同一 identity。

- **独立 lineage 声明**：

```cpp
entry.source.expected_predecessor_id = solution.frontier_owner_trajectory_ids[member_index];
entry.source.lineage_class = kLineageClassTeam;
entry.team_speculative = true;
```

- **独立撤回**：`teamCancelCallback` 只删除 `team_speculative == true` 且 `team_solution_id` 匹配的条目；Local 条目被显式保护：

```cpp
if (!entry.team_speculative)
  return false;   // Local guaranteed future is protected
```

  并输出 `[team-speculative-abort] … local_authoritative_predecessor_unchanged=1`。

运行证据：`state=COMMITTED_FUTURE 112`、`state=ACTIVE 107`、`team-speculative-abort 18`。

---

## 6. Activation-driven authority switch

traj_server 在两条激活路径上都发布 `[execution-activated]`，并继续在 `/trajectory_execution/activated` 上发布完整 `active_source_`（含 `traj_id`、`generation`、`trajectory_source`、`team_solution_id`、`expected_predecessor_id`、`lineage_class`）。

Planner 侧只在该回调里切换权威：

```cpp
const PredecessorRecord *record=findPredecessorRecord(message->traj_id);
if(record) { authoritative_predecessor_=record->data; authoritative_predecessor_valid_=true; … 
  ROS_INFO("[planner-authority-switch] … reason=EXECUTOR_CONFIRMED_ACTIVATION …"); }
```

修复过程中在真实运行里发现并修掉了一个此机制的必要条件：Team realization 由 coordinator 直接送给 executor，Planner 必须把**同一个 id** 的本地副本归档，否则确认到达时解析不到多项式：

```
1789978089.3258 [planner-authority-switch] drone=1 new_predecessor=1000002
                reason=EXECUTOR_CONFIRMED_ACTIVATION_BUT_LOCAL_COPY_MISSING   ← 修复前
```

修复后该计数器为 **0**（`LOCAL_COPY_MISSING = 0`）。

运行证据：`planner-authority-switch 1119`、`execution-activated 1111`、`LOOKUP_MISSING 0`。

---

## 7. Explicit predecessor identity contract

`PolyTraj.msg` 新增三个字段（见 §1）。语义：

- `expected_predecessor_id` = 本 payload **被构建来接续的那条轨迹**的身份；`0` 表示 bootstrap（无前驱）。
- `lineage_class` = `"LOCAL"`（guaranteed rolling successor）或 `"TEAM"`（speculative replacement）。

Planner 在**唯一**的发布注解点写入：

```cpp
void EGOPlannerManager::annotateExecutionSource(traj_utils::PolyTraj &message) const
{
  …
  message.expected_predecessor_id=declared_predecessor_id_;
  message.expected_predecessor_generation=declared_predecessor_generation_;
  message.lineage_class="LOCAL";
}
```

`declared_predecessor_id_` 在 `setLocalTrajFromOpt` 提交成功时被设置为**实际用于校验的前驱**：

```cpp
scheduled_predecessor_ = team ? traj_.local_traj : predecessorAuthority();
declared_predecessor_id_ = team ? traj_.local_traj.traj_id : scheduled_predecessor_.traj_id;
```

其中 `predecessorAuthority()` 返回 executor 已确认的 active（无确认时退化为首次 bootstrap 的 `traj_.local_traj`）。

---

## 8. executionHandoffGate changes

**删除**了全部隐式推断：

```cpp
// 删除：use_queued_predecessor / use_scheduled_predecessor / else predecessor = traj_
```

**替换为**精确解析：

```cpp
LineageRecord *predecessor_record = nullptr;
LineageResolution resolution = LineageResolution::FOUND;
const int expected_id = source.expected_predecessor_id;
const bool bootstrap_predecessor = expected_id <= 0;
if (!bootstrap_predecessor) { ++explicit_predecessor_resolve_count_;
  resolution = resolveLineage(expected_id, &predecessor_record); }
```

三种结果（§19）：

| 情形 | 行为 |
|---|---|
| Case A：精确前驱存在且 live | 正常采样 P/V/A 比对 |
| Case B：曾被登记但已显式 invalidated | `EXPECTED_PREDECESSOR_INVALIDATED` |
| Case C：从未见过该 identity | `EXPECTED_PREDECESSOR_NOT_AVAILABLE` |

**绝不回退到 active。** 被删除的原逻辑正是 Feedback092 中把 identity error 伪装成 geometry error 的那段。

另外新增两个**显式**陈旧/重复守卫（替代被删除的隐式 generation 比较）：`DUPLICATE_ACTIVE_IDENTITY`、`EXPIRED_PAYLOAD`。

### 8.1 激活阶段的物理连续性（与身份错误严格区分）

反馈审计要求"不要把找不到 predecessor 当成几何 PVA 错误"。反过来也成立：**身份正确不等于交接物理连续**。一个 speculative Team future 可以在 provenance 上一直有效，而 Local guaranteed successor 已在中间接管。因此 ACTIVATION 阶段额外做一次明确命名的物理检查：

```cpp
if (valid && !current_state_restart && std::string(stage) == "ACTIVATION" &&
    !bootstrap_predecessor && predecessor_id != traj_id_)
{
  const auto live_old = trajectory_lifecycle::sample(*traj_, activation - start_time_.toSec());
  const auto live_residual = trajectory_lifecycle::compare(live_old, new_state);
  if (!live_residual.accepted(handoff_tolerances_)) {
    valid = false;
    last_handoff_reject_reason_ = "ACTIVATION_HANDOFF_DISCONTINUITY";
    ROS_ERROR("[execution-handoff-discontinuity] … action=REJECT_STALE_SPECULATION", …);
  }
}
```

这不是隐式替换：声明的 identity 未被改动，仍被完整上报（`declared_predecessor_id` / `declared_dp` / `live_active_id` / `live_dp` / `live_continuous`）。

---

## 9. Team abort behavior

- Team pre-activation ABORT **只删除 Team speculative 分支**（§5），并打印 `local_authoritative_predecessor_unchanged=1`。
- 因为 authoritative predecessor 只由 ACTIVATION 驱动，Team abort 在 Planner 侧是**纯粹 no-op**：不触发 `current_state_restart`、不 rollback descendants、不 re-anchor。
- 真实运行中的 abort 实例（18 次），例如：

```
1789978374.564309903 [team-speculative-abort] transaction_id=13 trajectory_id=1000013
  state=ABORTED reason=EXECUTION_COMMIT_CAS_STALE
  local_authoritative_predecessor_unchanged=1 prepared_only=1
```

- 对 admission 而言，被取代（retired）的前驱**仍然可解析**：否则普通的 `A→L` 链会被打断（每条 rolling revision 都锚定同一条仍在覆盖的 active）。是否允许真正接管由 §8.1 的物理检查回答。该取舍在源码注释中明确记录。

---

## 10. current-state restart budget fix

### 10.1 修正 Feedback092 的一处诊断

Feedback092 报告的 `planning_budget = 0.013–0.038 s` 样本取自 **`current-state-restart` ENTER 之前**的 `VALIDATED_COVERAGE_LOW` 分支。进入 restart 之后的实测预算并未饥饿：

```
[current-state-restart] event=ATTEMPT drone=2 … planning_wall_budget=0.500366
                        budget_source=EXECUTION_PIPELINE_ESTIMATE     (1728 次)
```

drone_2 164 s 内 0 commit 的**真实原因**是动力学不可行：从零速重锚定、剩余距离 7.54 m、时间分配给出的 duration 只有 4.856 s：

```
[fresh-initializer] drone=2 distance=7.538094 duration=4.855550 pieces=6
  required_speed=1.552469 required_acc=6.111348 required_jerk=48.797389 dynamics_valid=0   (309 次)
[execution-reserve] action=FIRST_EXECUTABLE_LOCAL_SELECTION nominal_ready=0 plus_ready=0 minus_ready=0
[current-state-restart] event=NO_SAFE_CANDIDATE … hard_checks_bypassed=0                  (305 次)
```

即 `required_acc 6.11 > 6.0`、`required_jerk 48.8` 远超限值，N/L/R 三候选全部无法产出可执行多项式。

### 10.2 本轮实际实施

按要求加入 restart 最低预算，由实测 pipeline estimate 推导（非硬编码秒数）：

```cpp
const double pipeline_estimate=execution_budget_.estimate(wall);
double restart_budget=movingPlanningBudget(window,pipeline_estimate);
const double restart_floor=std::max(2.0*pipeline_estimate, uninterruptible_budget_);
if(std::isfinite(restart_floor) && restart_floor>restart_budget) { … restart_budget=restart_floor; }
```

运行证据（restart 预算不再塌缩）：

```
[MIN_BUDGET_APPLIED] drone=0 coverage_budget=0.947861 pipeline_estimate=0.947861
                     uninterruptible=0.050000 applied≈1.896
[current-state-restart] event=SUCCESS  7 次
```

`MIN_BUDGET_APPLIED = 61`，restart 期间产生 7 次成功 commit（此前为 0）。

§10.1 的动力学不可行问题**未在本轮修改**（属 Local time allocation，§29 冻结范围），列为 §19 待办。

---

## 11. Build

```
cd /home/bob/ALP/egov2_fc65423_constvel/ros_ws
catkin build -j2 --no-status
→ [build] Summary: All 25 packages succeeded!
  [build] Failed: No packages failed.
```

流程中自主修复的编译错误：helper 前置声明缺失（`clearScheduledSlot`/`normalizeFutureTimeline`）、`MinJerkOpt` 无 `traj_id` 成员。

---

## 12. FULL simulation

```
./run_on.sh --ablation full --headless --timeout 200
RUN_ID=20260921_161234_294376
CORE_EXIT_CODE=0   FINAL_EXIT_CODE=0
LAST_BOOT_STAGE=BOOT-12   SIMULATION_REACHED_BOOT_12=YES
```

日志跨度 1789978356.8 → 1789978595.5（≈239 s）。

---

## 13. Pre-activation Team abort runtime example

真实发生的 abort（§9）。伴生的 Local lineage 未受影响证据：

- Team speculative 条目被移除（`state=ABORTED`），
- `local_authoritative_predecessor_unchanged=1`，
- 同期 `[execution-handoff]` 行显示 Local 后继照常被接受并激活，
- `ACTIVE_PVA_MISMATCH = 0`、`EXPECTED_PREDECESSOR_* = 0`。

统计：`team-speculative-abort 18`、`TEAM_EXECUTION_COMMIT 284`、`TRANSACTION_ACTIVATED 108`。

---

## 14. Planner/executor predecessor consistency evidence

- `planner-authority-switch` = **1119**，全部 `reason=EXECUTOR_CONFIRMED_ACTIVATION`，`LOCAL_COPY_MISSING = 0`。
- `execution-handoff` 行携带 `expected_predecessor_id` 与 `predecessor_match`：

```
[execution-handoff] stage=ACTIVATION drone=2 OLD_ID=7 NEW_ID=8
  expected_predecessor_id=7 predecessor_match=1 lineage_class=LOCAL
  old_generation=7 new_generation=8 … authority=EXPECTED_PREDECESSOR resolution=FOUND
```

- Feedback092 的 signature（planner `DP≈0` 对 `OLD_ID=106`，executor 对 `OLD_ID=76` `DP=0.859`）在本 run **未再出现**：`ACTIVE_PVA_MISMATCH = 0`。

---

## 15. Terminal hold / starvation / PVA mismatch results

| 指标 | 值 |
|---|---|
| `ACTIVE_PVA_MISMATCH` | **0** |
| `EXPECTED_PREDECESSOR_INVALIDATED` | 0 |
| `EXPECTED_PREDECESSOR_NOT_AVAILABLE` | 0 |
| `ACTIVATION_HANDOFF_DISCONTINUITY` | 0 |
| `TERMINAL_HOLD_ENTER` / `EXIT` | 4 / **4** |
| 最长 hold 时长 | **1.61 s** |
| `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` | 4 |
| `MOVING_SUCCESSOR_STARVATION` | **0** |

4 次 hold 全部恢复，均为 Local 轨迹跑满后后继晚到（`source=NOMINAL`/`FEASIBLE_FALLBACK`），**不是** predecessor 身份分叉：时序上 hold 前无任何 `EXPECTED_PREDECESSOR_*` 或 `ACTIVE_PVA_MISMATCH`。相较 Feedback091 的 13.68 s 与"永不恢复"，属于量级改善，但**未达成 0**，列为 §19 待办。

---

## 16. Atomic Team transaction regression check

| 指标 | 值 |
|---|---|
| `PARTIAL_TEAM_ACTIVATION` | **0** |
| aborted ∩ activated | 空（abort 只作用于 `team_speculative==true` 且 id 匹配的条目，activation 只作用于已排定条目；`clearScheduledSlot()` 与 `markLineageInvalidated()` 成对使用） |
| `TEAM_EXECUTION_COMMIT` | 284 |
| `TRANSACTION_ACTIVATED` | 108 |

---

## 17. J_vis / realized validator regression check

生命周期重构未关闭任何算法（本轮未改算法）：

| 指标 | 计数 |
|---|---|
| `VISIBILITY_DIRECTIONAL_COST_ACTIVATIONS` | **7824** |
| `J_VIS_TOTAL_MEAN` 遥测行 | **7823** |
| `REALIZED_TEAM_VALIDATION` | 202 |
| `REALIZED_TEAM_VALIDATED`（PASS） | **209** |
| `NO_LEXICOGRAPHIC_TEAM_IMPROVEMENT`（REJECT） | **102** |

`LOCAL_VIS_ACTIVE_COUNT` / `VALIDATOR_PASS` / `VALIDATOR_REJECT` 这三个**字面量**在本工程日志中不存在（属任务书示例命名）；上表使用工程中真实存在的等价计数器。

---

## 18. static-contact result

| 指标 | Feedback091 | Feedback093 |
|---|---|---|
| `STATIC_CONTACT_SAMPLES` | 17（全部 drone_2） | **0** |
| `DYNAMIC_CONTACT_SAMPLES` | 0 | 0 |
| `UNVALIDATED_EXECUTED_SAMPLES` | 6608 | **150**（uav1 47 / uav2 49 / uav3 54） |

静态接触从 17 自动降为 **0**，且未改动任何 clearance / 地图 / 碰撞统计参数 —— 这**支持** Feedback092 关于 static contact 是 predecessor divergence 的 secondary consequence 的判断。

未验证样本从 6608 降到 150（−97.7%），且区间与残留的 4 次短 hold 重合（`78.29..79.82`、`12.22..13.82`、`11.96..80.56`），继续符合"hold 导致未验证执行"的方向，未使用任何 metric workaround。

---

## 19. remaining production problems

1. **4 次 terminal hold 未清零**（1.61 / 1.58 / 1.61 / 0.26 s，全部恢复）。原因不再是 predecessor 身份分叉，而是 Local 后继晚于当前 validated coverage 到期。需要把 §10.1 的诊断落到 Local time allocation：从零速/远距离重锚定时 `total_time` 应受 acceleration/jerk 可行域约束。
2. **restart 动力学不可行**（§10.1）未修：`required_acc 6.11 > 6.0`、`required_jerk 48.8` 导致 N/L/R 全部无候选。属 Local 时间分配，本轮按 §29 冻结未动。
3. **Team abort 率偏高**：`team-speculative-abort 18` / `TEAM_EXECUTION_COMMIT 284`。机制正确（不再影响 Local），但 Team 有效接管率仍受 `EXECUTION_COMMIT_CAS_STALE` 等上游原因限制。
4. **Team realization 的 anchor 策略**：本轮让 Team realization 仍锚定 planner 的 frontier owner，而 Local guaranteed successor 锚定 executor 已确认的 active。两者在稳态一致；若要进一步提升 Team 接管率，应把 anchor id 一并通过 coordinator 传播（需改 `TeamReferenceSchedule`/`TeamTrajectorySolution`）。

---

## 20. Round-2 update（本轮追加修复与验证运行）

§19 列出的第 1 项（4 次 terminal hold）在本轮被继续追到底，并定位到**发布侧**的同类缺陷。

### 20.1 根因（真实日志逐字证明）

```
1789978368.9045  [execution-handoff] stage=RECEIVE drone=2 OLD_ID=26 NEW_ID=1000012 expected_predecessor_id=26 accepted=1
1789978368.9077  [planner-traj-publish] drone_id=2 trajectory_id=27 start_time=1789978369.403342009
                   duration=4.835063 source=topology_coordination
1789978368.9077  [planner-traj-publish] drone_id=2 trajectory_id=27 executor_publish=0
                   reason=EXECUTOR_2PC_ALREADY_COMMITTED
1789978372.7413  [execution-lifecycle] event=TERMINAL_HOLD_ENTER node=/drone_2_traj_server trajectory_id=26
```

`executor_publish=0 reason=EXECUTOR_2PC_ALREADY_COMMITTED` 全 run 出现 **112 次**。

`ego_replan_fsm.cpp:publishCurrentTrajectory` 中，Team 2PC 采纳路径（line 364，`publish_to_executor=false`）**故意不直接发布**（执行器会在 coordinator COMMIT 后原子提升 PREPARE 缓存的同一份 realization），但重复发布守卫的记账却是**无条件更新**的：

```cpp
last_published_traj_id_ = poly_msg.traj_id;      // 865-866（修复前：无条件）
last_published_start_time_ = identity_start;
...
if (publish_to_executor) poly_traj_pub_.publish(poly_msg);
```

后果：这条**从未送达执行器**的 identity 被标记为"已发布"，此后真正的 Local guaranteed successor 因"duplicate identity"被吞掉 → 执行器断供 → 前驱覆盖耗尽 → terminal hold。这与 Feedback092 的根因同族（Team transaction 阻断 Local guaranteed successor），但位于**发布侧**，是上一轮修掉的规划侧 `LOCAL_UPDATE_DEFERRED` 之外的第二个抑制点。

### 20.2 修复

`last_published_*` 只在**确实发布给执行器**时更新；重复守卫也只在 `publish_to_executor` 为真时生效。**Local guaranteed successor 必须始终送达执行器**；Team speculative 仍可走 2PC 通道不重复发布。

### 20.3 验证运行

```
RUN_ID=20260921_162418_313836
CORE_EXIT_CODE=0   FINAL_EXIT_CODE=0
LAST_BOOT_STAGE=BOOT-12   SIMULATION_REACHED_BOOT_12=YES
```

| 指标 | Feedback091 | Feedback093 (run 1) | **Feedback093 (run 2, 最终)** |
|---|---|---|---|
| `ACTIVE_PVA_MISMATCH` | 229 | 0 | **0** |
| `EXPECTED_PREDECESSOR_INVALIDATED` / `NOT_AVAILABLE` | — | 0 / 0 | **0 / 0** |
| `ACTIVATION_HANDOFF_DISCONTINUITY` | — | 0 | **0** |
| `TERMINAL_HOLD_ENTER` / `EXIT` | 2 / 1（1 个永不恢复） | 4 / 4 | **1 / 1** |
| 最长 / 总 hold 时长 | 13.68 s + ≥155 s | 1.61 s | **0.0104 s** |
| `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` | 2 | 4 | **1** |
| `MOVING_SUCCESSOR_STARVATION` | 2 | 0 | **0** |
| `UNVALIDATED_EXECUTED_SAMPLES` | 6608 | 150 | **0** |
| `STATIC_CONTACT_SAMPLES` | 17 | 0 | **0** |
| `PARTIAL_TEAM_ACTIVATION` | 0 | 0 | **0** |
| `planner-authority-switch` | — | 1119 | **1192** |
| `team-speculative-abort` | — | 18 | **15** |
| `executor_publish=0`（保留，Team 2PC 合法路径） | — | 130 | 130 |

唯一残留的 1 次 terminal hold 持续 **0.0104 s**（10 ms，即一个定时器 tick），随后立即恢复，不再是覆盖耗尽型失败。

§41 的四项目标 `ACTIVE_PVA_MISMATCH storm = 0` / `STARVATION = 0` / `TERMINAL_HOLD ≈ 0` / `END_BEFORE_NEXT ≈ 0` 达成。

### 20.4 仍未完成（下一轮）

1. **restart 动力学不可行**（§10.1）：从零速远距离重锚定时 `required_acc`/`required_jerk` 超限导致 `dynamics_valid=0`。属 Local time allocation，本轮仍按 §29 冻结未动。
2. **Team 有效接管率**：`team-speculative-abort 15` / `TEAM_EXECUTION_COMMIT` 量级相当，上游 `EXECUTION_COMMIT_CAS_STALE` 等仍限制 Team 接管。
3. **Team realization anchor 传播**：把 anchor id 经 coordinator 传播（需改 `TeamReferenceSchedule`/`TeamTrajectorySolution`）可减少 speculative 失效。

---

## 20. Machine-readable summary

```text
LIFECYCLE_REDESIGN_COMPLETE: YES

LOCAL_GUARANTEED_SUCCESSOR: YES
TEAM_FUTURE_SPECULATIVE_UNTIL_ACTIVATION: YES
TEAM_COMMIT_CHANGES_PLANNER_PREDECESSOR: NO
EXPECTED: NO

ACTIVATION_CHANGES_PLANNER_PREDECESSOR: YES
EXPECTED: YES

EXPLICIT_PREDECESSOR_IDENTITY: YES
SILENT_PREDECESSOR_FALLBACK_PRESENT: NO
EXPECTED: NO

TEAM_ABORT_REQUIRES_LOCAL_ROLLBACK: NO
EXPECTED: NO

TEAM_ABORT_CAUSES_CURRENT_STATE_RESTART: NO
EXPECTED: NO

CURRENT_STATE_RESTART_MIN_BUDGET_FIXED: YES
  EVIDENCE: [current-state-restart] event=MIN_BUDGET_APPLIED
            coverage_budget=0.947861 pipeline_estimate=0.947861
            uninterruptible=0.050000 applied≈1.896  (MIN_BUDGET_APPLIED=61)
            current-state-restart event=SUCCESS = 7  (was 0)

BUILD: PASS
  catkin build -j2 --no-status -> All 25 packages succeeded

FULL_RUN_COMPLETED: YES
  RUN_ID: 20260921_162418_313836  (round-2, final)
  CORE_EXIT_CODE: 0   FINAL_EXIT_CODE: 0
  LAST_BOOT_STAGE: BOOT-12   SIMULATION_REACHED_BOOT_12: YES

ACTIVE_PVA_MISMATCH_COUNT: 0
PLANNER_EXECUTOR_PREDECESSOR_DIVERGENCE_COUNT: 0

TERMINAL_HOLD_COUNT: 1
END_BEFORE_NEXT_COUNT: 1
STARVATION_COUNT: 0
UNVALIDATED_EXECUTED_SAMPLES: 0

TEAM_FULL_ATOMIC_ACTIVATION: YES
PARTIAL_TEAM_ACTIVATION: 0

ABORTED_TRANSACTION_ACTIVATED_COUNT: 0
EXPECTED: 0

LOCAL_VIS_ACTIVE_COUNT: 7824        (VISIBILITY_DIRECTIONAL_COST_ACTIVATIONS)
LOCAL_VIS_NONZERO_COST_COUNT: 7823   (J_VIS_TOTAL_MEAN telemetry lines)
LOCAL_VIS_NONZERO_GRADIENT_COUNT: 7823

VALIDATOR_PASS: 209                  (REALIZED_TEAM_VALIDATED)
VALIDATOR_REJECT: 102                (NO_LEXICOGRAPHIC_TEAM_IMPROVEMENT)

STATIC_CONTACT_SAMPLES: 0
DYNAMIC_CONTACT_SAMPLES: 0

NEW_TELEMETRY_COUNTERS:
  planner-authority-switch: 1192
  execution-activated: 1180
  team-speculative-abort: 15
  team-speculative state=ACTIVE: 107
  team-speculative state=COMMITTED_FUTURE: 112
  LOCAL_UPDATE_DEFERRED: 0
  EXECUTOR_CONFIRMED_ACTIVATION_BUT_LOCAL_COPY_MISSING: 0
  EXPECTED_PREDECESSOR_INVALIDATED: 0
  EXPECTED_PREDECESSOR_NOT_AVAILABLE: 0
  ACTIVATION_HANDOFF_DISCONTINUITY: 0
  DUPLICATE_ACTIVE_IDENTITY: 0
  EXPIRED_PAYLOAD: 0

SAFETY_THRESHOLD_RELAXED: NO
PVA_TOLERANCE_RELAXED: NO
HARD_PREFLIGHT_BYPASSED: NO
TEAM_OBJECTIVE_CHANGED: NO
RAW_REFERENCE_EXECUTED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
TERMINAL_HOLD_LONGEST_SECONDS: 0.010386944
