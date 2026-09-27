# Feedback096 — Team T/PT-SCP 与高频 rolling Local planning 的调度共存

日期：2026-09-22
项目：`/home/bob/ALP/egov2_fc65423_constvel`
上游：`feedback/feedback_095.md`
压力场景：`long_cylinder_forest_visibility_stress.json`（26 航点目标 / 39 静态障碍 / 3 动态障碍）

---

# 1. Root engineering problem

任务假设是："Local 高频 rolling（~12 ms/tick），Team T/PT-SCP 耗时更长，因此二者共存时会出现
TEAM-SCP starvation（一有 Local 更新就 cancel/restart）或 SCP 算完即过期"。

**本轮先用真实数据核对这个假设，结论是假设不成立。**

来自 `feedback_095` 权威运行 `runs/20260922_043207_778930` 的实测：

```text
TEAM_SCP_START              = 7      ← 只触发 7 次，不存在反复 cancel/restart
TEAM_SCP_PASS               = 0
HANDOFF_CONTRACT_STALE      = 0      ← 没有任何 contract 因 Local 更新而 stale
HANDOFF_CONTRACT_LINEAGE_ADVANCE = 20 ← 直接后继被正确"推进"而非判 stale
HANDOFF_CONTRACT_LINEAGE_DIVERGED = 0
[TEAM_REFERENCE_ACK] ... latency_ms = 1.366 / 1.470 / 4.110 / 5.151 / 1.956
```

即：

* **不存在 Local 更新导致的 cancel/stale**：coordinator 的 `executionCallback()`
  早已实现 `HANDOFF_CONTRACT_LINEAGE_ADVANCE`（`multi_uav_topology_coordinator.cpp:582-616`）——
  若新 Local 轨迹是 contract frontier 的直接后继（`expected_predecessor_id == execution_[i].traj_id`），
  就**推进** frontier，而不是判 stale。`STALE=0`、`ADVANCE=20` 证明该路径工作。
* **不存在 SCP 耗时更长**：单次 SCP 实测 **1.4–5.2 ms**，而 Local 重规划 tick 的 P50 是 **12.2 ms**。
  Team-SCP 比 Local 更新更快，不是更慢。
* 真正的问题是 **`TEAM_SCP_PASS = 0`（7 次尝试全部失败）**。

因此本轮的实际工程问题是：

> 在 Local rolling 持续更新的前提下，把 Team refinement 的**调度、输入冻结、完成后再校验、
> 采用与防覆盖**做成语义正确的模型，并用真实遥测回答"到底是算不出来、算得太慢、
> rolling 太快、还是问题自己消失"。

---

# 2. Final implemented scheduling model

| 契约 | 实现 |
|---|---|
| **A. Single-flight** | `team_refinement_open_` 从 snapshot 冻结起、到其 transaction 终结止；期间新的 schedule 只做 latest-wins 合并（`team_ref_coalesced_`），绝不启动第二个 competing solve，也不 cancel 正在跑的。同时 `hasCommittedPendingTeamTail()` 为真时同样不发起新的 refinement。 |
| **B. Freeze input, NOT Local** | `TeamRefinementSnapshot` 在 SCP 启动时一次性捕获（refinement_id / contract_id / activation / critical window / 三机 baseline identity / `start_snapshot_id` / required_margin / anchor 残差）。Local rolling 全程继续，不等待。 |
| **C. No cancel-on-normal-update** | 普通 Local 轨迹更新只做 `++latest_team_prediction_snapshot_id_`（在 `setLocalTrajFromOpt` 提交成功后），**不取消** solver、也不把结果判 stale。 |
| **D. Finish then recheck** | Team refinement 完成后不因 generation 变化机械 reject；在**该 transaction 即将真正落地的那一刻**（`processPendingTopologyCoordination()` 的 COMMITTED 分支）用当前 anchor / peer / world 做四道 check。 |
| **E. Prevent immediate quality overwrite** | 被实际采用的 Team future 携带轻量 `TeamImprovementCertificate`；**非安全必需**的 Local quality 更新在其认证窗口内不得覆盖它；**安全必需的 Local successor 永远优先**。 |

**没有引入**：新的 persistent 状态机、第二套 transaction、Team-SCP 队列、thread pool、
online rebase、planner pause、trajectory-generation inheritance。

---

# 3. Exact production source changes

## 3.1 `traj_opt/include/optimizer/poly_traj_optimizer.h`

* `TeamSCPResult` 新增两个**诊断**字段：
  * `double linearization_invalid_world_time{-1.0}`
  * `std::string linearization_invalid_component{"NONE"}`
* 新增私有成员 `int team_margin_invalid_component_{-1}`（0=STATIC,1=DYNAMIC_LOS,2=FOV,3=RANGE,-1=none）。
  刻意只存索引，避免头文件依赖 `multi_uav_formation` 类型。

## 3.2 `traj_opt/src/poly_traj_optimizer.cpp`

1. 匿名命名空间新增 `visibilityComponentName(int)`。
2. `teamMarginAt()` 内：调用 `composeVisibilityMargin` **之前**先判定并记录第一个无效 component。
3. `runCandidateHardCorridorSCP()` 的 visibility 采样循环：
   * `margin.margin_valid == false` → 记录 `linearization_invalid_world_time = activation + t`
     与 component 名，然后 `rows_valid = false`；
   * `locatePieceTime()` 失败 → 记录 `LOCATE_PIECE_TIME`。
4. **关键语义修正**：把原先的一体判定
   ```cpp
   if (!rows_valid || team_visibility_constraint_count <= 0) { fail; }
   ```
   拆成两支：
   * `!rows_valid` → 仍然 `TEAM_VISIBILITY_LINEARIZATION_INVALID`，fail closed（保持）；
   * `rows_valid && count <= 0` → 所有采样点**已经满足** contract margin，`appendLinearUpperBound`
     因其零梯度且 `ub >= 0` 而正确地不生成行。这不是线性化失败，而是"无需修正"。
     现在返回 `success=true, reason=TEAM_VISIBILITY_ALREADY_SATISFIED`，零修改接受 seed，
     并打印 `[TEAM_VISIBILITY_ALREADY_SATISFIED]`。**所有下游 exact nonlinear validation、
   hard preflight、RealizedTeamValidator 照常运行**，没有绕过任何检查。

## 3.3 `plan_manage/include/plan_manage/planner_manager.h`

* `TeamRefinementSnapshot`（冻结输入）。
* `team_refinement_` / `team_refinement_open_` / `team_refinement_serial_` /
  `latest_team_prediction_snapshot_id_` / `latest_handoff_contract_{id,active,seen}_`。
* `team_refinement_solver_pass_` / `team_refinement_solver_reason_` /
  `team_refinement_solve_ms_` / `team_refinement_total_ms_`。
* 延迟样本向量：T-only 与 PT 分开（`team_ref_t_*` / `team_ref_pt_*`）。
* 采用漏斗计数器：started / solver_pass / adopted / activated / coalesced /
  no_longer_needed / stale_anchor / latest_recheck_reject / too_late / solver_infeasible。
* 保护计数器：`local_quality_overwrite_blocked_` / `local_safety_override_team_`。
* 执行时长样本 `team_ref_execution_durations_`。
* `TeamImprovementCertificate`（轻量、自动失效，不是状态机）。
* `PendingTeamTrajectory` 新增 `bool handoff_refinement{false}`。
* 两个方法：`bool localSuccessorIsSafetyRequired(std::string&) const`、
  `void reportTeamRefinementAudit(bool) const`。

## 3.4 `plan_manage/src/planner_manager.cpp`

* `localSuccessorIsSafetyRequired()`：**复用既有 Local authority**，不用新启发式。返回 true 当且仅当
  `current_state_restart_active_` / `lifecycle_invalidated_` / 无 active future /
  有效覆盖 `<= local_activation_margin_ + execution_margin_`（`COVERAGE_EXHAUSTING`）。
* `reportTeamRefinementAudit()`：每 5 s 与 final 打印汇总（含 T/PT 分开的 latency 分位）。
* `teamReferenceScheduleCallback()`：
  * 入口记录最新 contract 状态并 `++latest_team_prediction_snapshot_id_`；
  * `handoff_contract_active` 分支加 single-flight gate（coalesce 并返回
    `TEAM_REFINEMENT_COALESCED`）；
  * 冻结 `TeamRefinementSnapshot`；
  * 对 solve 计时，失败按 `SOLVER_INFEASIBLE` 记录并带上缺失梯度证据；
  * 成功记录 `TEAM_REFINEMENT_START` / `TEAM_REFINEMENT_SOLVE`，按 T/PT 分桶；
  * 成功 ACK 后置 `team_refinement_open_ = true` 且
    `pending_team_trajectory_.handoff_refinement = true`。
* `processPendingTopologyCoordination()` COMMITTED 分支：**只有** `team.handoff_refinement`
  为真时才进入四道 check（见 §5）。普通 Team reference 提交路径**逐字保持原样**并提前返回，
  不受 relay 语义影响。
* `setLocalTrajFromOpt()`：Local 提交成功后 `++latest_team_prediction_snapshot_id_`；
  在原有 team-tail 检查前加 `TeamImprovementCertificate` 保护（见 §6）。
* activation 回调：认证轨迹被 executor 确认激活时 `++team_ref_activated_` 并打印
  `[TEAM_REFINEMENT_ACTIVATED]`。匹配时同时覆盖 planner 本地 id 与 Team speculative
  命名空间 `1000000 + team_solution_id`。

---

# 4. Thread / mutable-state safety

**结论：Team-SCP 与 Local optimization 没有并发，因此不存在共享 mutable optimizer 污染。**

证据：

* `ego_planner_node.cpp:53` 使用单线程 `ros::spin()`；
* `team_reference_schedule_sub_` 用默认 callback queue（`planner_manager.cpp:630-634`），
  `execFSMCallback` 是默认 queue 上的 `ros::Timer`。二者**同一条线程**，串行执行。
* 因此 Team-SCP 运行期间不可能有 Local callback 进入，`updates_during_solve` 结构性为 0
  （实测亦为 0）。这一点在遥测里被显式记录，而不是被掩盖。

§6 的处置：**没有**为了形式强行引入 thread pool。本轮先按要求"证明不会 block Local rolling"，
并用遥测量化（见 §9、§15）：SCP 实测 1.3–2.7 ms，Local tick P50 12.2 ms；
运行级 liveness 全绿（`LOCAL_UPDATE_DEFERRED=0`、`TERMINAL_HOLD=0`、
`MOVING_SUCCESSOR_STARVATION=0`）。§15 据此回答"是否需要异步化"。

> 说明：确实存在一个真实的异步窗口——ACK 之后 coordinator 的
> PREPARE → 3×READY → COMMIT 往返期间 Local rolling 继续。四道 check 正是落在
> 该窗口结束、COMMIT 真正落地的那一刻，因此 recheck 面对的是真正的 latest 状态。

---

# 5. Adoption checks

四道 check 全部落在 `processPendingTopologyCoordination()` COMMITTED 分支的
`setLocalTrajFromOpt(team.trajectory, ...)` **之前**，且仅在 `team.handoff_refinement` 为真时执行。

| Check | 源码落点 | 判据 | 失败 reason |
|---|---|---|---|
| **1 activation / predecessor compatibility** | 用 `prepareLocalHandoff()` 重新锚定到真实 activation boundary，再用 **executor 同一套** `checkActiveHandoff()`（`handoff_position/velocity/acceleration_tolerance_` = 0.02/0.05/0.10）校验连续性 | 不新增 `team_anchor_*` 阈值 | `TEAM_RESULT_STALE_ANCHOR` |
| **2 baseline 是否还需要** | 比较 `latest_handoff_contract_id_/active_` 与该 refinement 的 `contract_id` | contract 已 release/被替换 ⇒ 需求消失（正常 no-op，非 solver failure） | `TEAM_RESULT_NO_LONGER_NEEDED` |
| **3 与最新 peers 重新组合** | 对 refined polynomial 重新跑 `checkTrajectoryDynamics` / `checkTrajectoryStaticSafety` / `evaluateDynamicRisk` / `checkTrajectorySwarmSafety` / `validateExecutionTrajectory`（后者复用 RealizedTeamValidator 同源证据） | peer id 变化本身**不是**拒绝理由 | `TEAM_RESULT_LATEST_RECHECK_REJECT` |
| **4 还赶得上共同 activation** | `now > activation - local_activation_margin_` | 不移动 activation 迁就旧解 | `TEAM_RESULT_TOO_LATE` |

四道全部通过才进入 `setLocalTrajFromOpt`；此后 `pending_team_trajectory_` 冻结为最终 triple，
PREPARE/READY/COMMIT 对应同一份三机轨迹（沿用既有 transaction，未新增第二套）。

---

# 6. Local overwrite protection

实现于 `setLocalTrajFromOpt()`，只对 `!team`（普通 Local 提交）生效。

**阻止 quality replacement 的条件（全部满足）**：

1. `team_improvement_certificate_.valid`
2. `traj_.local_traj.traj_id == certificate.trajectory_id`（认证轨迹仍在 timeline）
3. 当前时间 < `certificate.activation + certificate.critical_end`（认证窗口未结束）
4. `localSuccessorIsSafetyRequired(reason) == false`

⇒ 打印 `[LOCAL_QUALITY_OVERWRITE_BLOCKED]`，`return false`，
**不删除** Local candidate（下一 tick 仍可复用），并让下一轮 rolling Team prediction 重新评价。

**safety successor 无条件优先**：命中 §4 的 `localSuccessorIsSafetyRequired`（restart / lifecycle
invalidated / 无 active future / 覆盖即将耗尽）时：

* 打印 `[TEAM_IMPROVEMENT_OVERRIDE] ... action=ALLOW_SAFETY_SUCCESSOR`
* 立即清空 certificate，允许安全后继提交。

`TeamImprovementCertificate` 自动失效：critical window 结束、认证轨迹离开 timeline、
safety override。它不构成新的 authority，也不要求 planner 等待任何 `HANDOFF_COMPLETE`。

---

# 7. Build

```text
cd /home/bob/ALP/egov2_fc65423_constvel/ros_ws
catkin build -j2 --no-status
→ [build] Summary: All 25 packages succeeded!
  [build] Failed: No packages failed.
```

过程中自主修复的编译错误：头文件缺少 visibility 类型 include（改为只存索引）、
`runCandidateHardCorridorSCP` 内 `result` 不在作用域（改用成员 `team_scp_result_`）、
`team_result` 越作用域使用。

运行 `--scenario` 曾因传**相对路径**导致节点 `No such file or directory` 而 BOOT-06B 失败
（topology coordinator 未打印 `[ablation-config]`）——已改为绝对路径。这是运行器使用失误，
不是产品缺陷。

---

# 8. FULL production run

```text
ON  (enable_team_spatiotemporal_scp=true)  : runs/20260922_204715_37427
OFF (enable_team_spatiotemporal_scp=false) : runs/20260922_204050_33052

scenario       = long_cylinder_forest_visibility_stress.json
ablation mode  = FULL         headless = YES
两者均：FINAL_EXIT_CODE=0, LAST_BOOT_STAGE=BOOT-12, SIMULATION_REACHED_BOOT_12=YES
```

---

# 9. Timing analysis

## ON run（refinement 成功路径）

| 指标 | 值 |
|---|---|
| solve P50 / P95 / MAX | **1.838 / 1.838 / 2.743 ms** |
| total P50 / P95 / MAX | **0.588 / 0.588 / 0.612 ms** |
| 模式分布 | **PT = 3，T-only = 0** |
| anchor ΔP / ΔV / ΔA | 0 / 0 / 0（见 §13：三次均为零修改接受） |

**T-only：本 run 样本数 0**，因此 T-only 的 P50/P95/MAX 无数据，不给数字。

## 与 Local 更新速度的对比（回答 §23 Q1/Q2）

| 量 | 值 |
|---|---|
| Local 重规划 tick P50（同 run） | ~12 ms |
| Team-SCP solve P50 | **1.8 ms** |
| Team-SCP solve MAX | **2.7 ms** |
| 一次 SCP 期间平均 Local 更新次数 | **0**（串行架构下结构性为 0；`updates_during_max=0`） |

⇒ **Team-SCP 比一次 Local rolling 更新快约 5 倍**，不存在"算得太慢"。

---

# 10. Adoption funnel

| 级别 | ON | OFF |
|---|---|---|
| `TEAM_REFINEMENT_STARTED`（SCP 尝试） | **7** | 0 |
| `TEAM_REFINEMENT_SOLVER_PASS` | **3** | 0 |
| `TEAM_REFINEMENT_ADOPTED` | **3** | 0 |
| `TEAM_REFINEMENT_ACTIVATED` | **3** | 0 |
| `TEAM_REFINEMENT_COALESCED` | 0 | 0 |

派生率（ON）：`solver_pass_rate = 3/7 = 42.9%`；
`adoption_given_solver_pass = 3/3 = 100%`；`activation_given_adoption = 3/3 = 100%`。

逐机（final audit）：

```text
drone=2 started=3 solver_pass=1 adopted=1 activated=1 solver_infeasible=2
drone=1 started=2 solver_pass=2 adopted=2 activated=2 solver_infeasible=0
drone=0 started=2 solver_pass=0 adopted=0 activated=0 solver_infeasible=2
```

激活明细（经 Team speculative 命名空间确认）：

```text
[TEAM_REFINEMENT_ACTIVATED] drone=2 team_solution_id=70 trajectory_id=1000070
[TEAM_REFINEMENT_ACTIVATED] drone=1 team_solution_id=70 trajectory_id=1000070
[TEAM_REFINEMENT_ACTIVATED] drone=1 team_solution_id=84 trajectory_id=1000084
```

---

# 11. Reject reason distribution

| reason | ON |
|---|---|
| `TEAM_RESULT_NO_LONGER_NEEDED` | **0** |
| `TEAM_RESULT_STALE_ANCHOR` | **0** |
| `TEAM_RESULT_LATEST_RECHECK_REJECT` | **0** |
| `TEAM_RESULT_TOO_LATE` | **0** |
| `SOLVER_INFEASIBLE` | **4** |
| `TEAM_VISIBILITY_ALREADY_SATISFIED`（子尝试，非拒绝） | 3 |

汇总的 SCP 失败原因：`TEAM_SCP_FAILED`×8、`LOCAL_REALIZATION_FAILED`×2。

**没有一次 discard 来自 staleness、anchor 失配、peer 变化或太晚**——这四项机制存在但本轮未被触发，
因为它们本来针对的风险在本 run 中确实没有发生。

---

# 12. Actual execution lifetime of Team results

3 次 adopted refinement 全部在 ~0.5 s 后被 executor 确认激活（`team_solution_id=70` /
`trajectory_id=1000070` 于 1790081275.02 激活；`team_solution_id=84` / `1000084` 于
1790081283.12 激活），`certificate_valid=1`。

**是否被下一条 Local quality trajectory 立即覆盖：没有。** `LOCAL_QUALITY_OVERWRITE_BLOCKED`
在 ON run 中触发 **449** 次，`TEAM_IMPROVEMENT_OVERRIDE`（safety 覆盖）**0** 次，
说明认证窗口内所有普通 Local quality 更新都被拒绝，而没有任何安全必需后继被阻挡。

> 诚实说明：`team_ref_execution_durations_` 记录的是"激活时刻 − activation"，结构性接近 0，
> 不能作为执行时长。本报告用"认证窗口内 449 次 quality 覆盖被阻止 + 0 次 safety override"
> 作为执行寿命的证据，不虚构一个平均秒数。

---

# 13. OFF/ON visibility result

| 指标 | **ON** | **OFF** |
|---|---|---|
| K2（≥2 机可见） | 98.1923% | **99.1376%** |
| ALL3（3 机可见） | **92.1939%** | 83.5729% |
| BLACKOUT | 0.4108% | **0.2053%** |
| visible 0/1/2/3 | 10 / 34 / 146 / 2244 | 5 / 16 / 379 / 2035 |
| 统计窗口 | 81.12 s / 2434 样本 | 81.16 s / 2435 样本 |

ON 在 **ALL3 上 +8.62 pp**，但 K2 低 0.95 pp、BLACKOUT 高 0.21 pp。
**这是各一次运行的单样本对照，不足以宣称可见性收益**；只能说 ON 没有可见性退化到不可接受，
且在 ALL3 上明显更好。

**必须指出的关键事实**：本 run 的 3 次 refinement **全部是 `TEAM_VISIBILITY_ALREADY_SATISFIED`**：

```text
[TEAM_VISIBILITY_ALREADY_SATISFIED] drone_id=1 contract_id=1 window=[...] margin_min=1.000000 required=0.200000
[TEAM_REFINEMENT_SOLVE] mode=PT margin=-inf->1.000000 dp=0.000000000 dtau=0.000000000 dT=0.000000000
```

即 solver 判定 seed 在契约窗口内**每个采样点都已满足** required margin（`margin_min = 1.0`，
是理论最大值），因此**零修改接受**。**本轮没有任何 Team refinement 真正改变了轨迹几何**，
所以 ON/OFF 的可见性差异不能被归因为 relay refinement 的收益。

同时这是一个值得下一轮追查的真实矛盾：coordinator 因 `M2 < trigger(0.30)` 才创建 contract，
而 optimizer 的 `teamMarginAt` 对同一窗口给出 `margin = 1.0`。两者对"同一世界、同一时间窗"
的 M2/margin 判定不一致（怀疑与 outgoing/incoming 角色对应的 UAV 并非真正的 limiting UAV 有关）。

---

# 14. Safety / liveness

| 指标 | ON | OFF |
|---|---|---|
| `TERMINAL_HOLD_ENTER` | **0** | 1（drone_1，持续 **6.01 s**） |
| `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` | **0** | 1 |
| `MOVING_SUCCESSOR_STARVATION` | **0** | 0 |
| `ACTIVE_PVA_MISMATCH` | 0 | 0 |
| `EXPECTED_PREDECESSOR_INVALIDATED` / `NOT_AVAILABLE` | 0 / 0 | 0 / 0 |
| `ACTIVATION_HANDOFF_DISCONTINUITY` | 0 | 0 |
| `LOCAL_UPDATE_DEFERRED` | **0** | 0 |
| `PARTIAL_TEAM_ACTIVATION` | **0** | 0 |
| `traj-server-handoff-reject` | 0 | 0 |
| `TRANSACTION_ACTIVATED` | 162 | 168 |
| min static clearance | 0.2011 m | 0.2860 m |
| min moving clearance | **0.4062 m** | **0.0000 m（74 个接触样本）** |
| `UNVALIDATED_EXECUTED_SAMPLES` | **0** | **181** |
| 末态距目标 uav1/2/3 | 2.01 / 1.71 / 1.71 m | 1.72 / 1.65 / 1.98 m |

**回答 §23 Q8/Q9：没有任何 safety-required Local successor 被 Team protection 错误阻挡
（`TEAM_IMPROVEMENT_OVERRIDE = 0`，且 ON 组零接触、零未验证、零 hold）。**
Team refinement 未造成 hold / starvation / predecessor mismatch / partial activation / safety regression。

---

# 15. Whether additional future-anchor protection is actually necessary

**基于数据：不需要。**

* `TEAM_RESULT_STALE_ANCHOR = 0`、`LATEST_RECHECK_REJECT = 0`、`TOO_LATE = 0`。
* `HANDOFF_CONTRACT_STALE = 0`，而 `HANDOFF_CONTRACT_LINEAGE_ADVANCE = 64` /
  `LINEAGE_DIVERGED = 0` —— Local rolling 每推进一次 frontier，contract 都能正确跟进。
* 因此"短 future anchor protection"针对的风险在本 run 中没有出现；按 §24 的要求，
  **不提前实现**。

关于异步化：**同样不需要。** Team-SCP solve P50 = 1.8 ms、MAX = 2.7 ms，比 Local tick 的
12 ms 快约 5 倍；串行执行期间 `LOCAL_UPDATE_DEFERRED = 0`、`TERMINAL_HOLD = 0`、
`MOVING_SUCCESSOR_STARVATION = 0`。引入独立线程反而会引入 `grid_map_` / `obj_predictor_` /
`traj_.swarm_traj` 的真实数据竞争。§6 允许的路径已按"先证明不阻塞"执行。

**真正的瓶颈不在调度，而在求解本身**：7 次尝试中 4 次 `SOLVER_INFEASIBLE`、
3 次 `ALREADY_SATISFIED`（零修改），**没有一次产生有效的几何修正**。

---

# 16. 机器可读总结

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES
FULL_SIMULATION_RUN: YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO

TEAM_SINGLE_FLIGHT_IMPLEMENTED: YES
TEAM_CANCEL_ON_NORMAL_LOCAL_UPDATE: NO
TEAM_FINISH_RECHECK_IMPLEMENTED: YES
TEAM_LATEST_PEER_RECHECK_IMPLEMENTED: YES
TEAM_QUALITY_OVERWRITE_PROTECTION: YES
LOCAL_SAFETY_OVERRIDE_PRESERVED: YES

TEAM_REFINEMENT_STARTED: 7
TEAM_REFINEMENT_SOLVER_PASS: 3
TEAM_REFINEMENT_ADOPTED: 3
TEAM_REFINEMENT_ACTIVATED: 3

TEAM_RESULT_NO_LONGER_NEEDED: 0
TEAM_RESULT_STALE_ANCHOR: 0
TEAM_RESULT_LATEST_RECHECK_REJECT: 0
TEAM_RESULT_TOO_LATE: 0
TEAM_RESULT_SOLVER_INFEASIBLE: 4
TEAM_RESULT_ALREADY_SATISFIED: 3

TEAM_T_TOTAL_LATENCY_P50_P95_MAX: NO_DATA (T-only samples = 0)
TEAM_PT_TOTAL_LATENCY_P50_P95_MAX: 0.588 / 0.588 / 0.612 ms
TEAM_PT_SOLVE_LATENCY_P50_P95_MAX: 1.838 / 1.838 / 2.743 ms

LOCAL_UPDATES_DURING_TEAM_REFINEMENT_P50_P95_MAX: 0 / 0 / 0
  (single-threaded serial architecture: structurally 0 during solve)

TEAM_ACTIVATED_EXECUTION_DURATION_P50_P95_MAX: NOT_MEASURABLE
  (certificate window protection held for all 3 activations; 449 quality
   overwrites blocked, 0 safety overrides)

LOCAL_QUALITY_OVERWRITE_BLOCKED: 449
LOCAL_SAFETY_OVERRIDE_TEAM: 0

TEAM_CAUSED_TERMINAL_HOLD: 0
TEAM_CAUSED_STARVATION: 0
PREDECESSOR_MISMATCH: 0
PARTIAL_TEAM_ACTIVATION: 0

RUN_DIR:
  ON  = /home/bob/ALP/egov2_fc65423_constvel/runs/20260922_204715_37427
  OFF = /home/bob/ALP/egov2_fc65423_constvel/runs/20260922_204050_33052

CURRENT_SIMPLE_ASYNC_SCHEME_SUFFICIENT: YES

MINIMAL_NEXT_STEP (based only on this run's timing + adoption data):
  不需要异步化，也不需要 future-anchor protection。
  唯一阻塞点在求解端：7 次 SCP 中 4 次 SOLVER_INFEASIBLE、3 次 ALREADY_SATISFIED 零修改。
  下一步应只做数值诊断：对同一 critical window 同时打印 coordinator 的三机
  per-UAV margin 与 optimizer 的 teamMarginAt，确认
  (a) contract 的 outgoing/incoming 角色是否指向真正的 limiting UAV，
  (b) 两侧 M2/margin 判定为何分别为 <0.30 与 =1.0。
  不应增加新的状态机、第二套 SCP、也不应放宽任何 safety 阈值。
```

---

# 17. 本轮直接修复的生产缺陷（附带收益）

`team_visibility_constraint_count <= 0` 曾被与"线性化无效"合并判定。当 seed 在契约窗口内
**每点都已满足** required margin 时，`appendLinearUpperBound` 会因零梯度且 `ub >= 0` 而
正确地不生成约束行，使计数为 0 —— 旧代码因此把它判成 `TEAM_VISIBILITY_LINEARIZATION_INVALID`
并**拒绝整个 Team reference**。该缺陷在 feedback095 的运行中就以
`TEAM_VISIBILITY_LINEARIZATION_INVALID=23` 的形式存在。本轮拆分后，
`TEAM_VISIBILITY_ALREADY_SATISFIED` 成为一次合法的、零修改的成功结果，
同时保留 `!rows_valid` 时的 fail closed 行为，且不绕过任何下游 exact 校验。
