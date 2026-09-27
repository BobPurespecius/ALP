# Feedback 100 —— M2 只负责发现风险；双候选修复 + 最早交叉触发 + 完整时延链

本轮按任务要求修改 production code：M2 不再固定决定"修谁"，改为发布修复候选集
{ranked[1], ranked[2]}；触发改为 **最早阈值交叉**；并补全 forecast → activation 的
完整时延链遥测。随后 build + 3 次真实 FULL 压力仿真。

硬边界遵守情况：未使用 `git reset` / `git checkout` / `git clean` / 未回滚已有修改；
未重写 MINCO；未重写 SCP 数学主体；未重写 Feedback096/098 已闭合的
single-flight / finish-and-recheck / Local guaranteed successor / speculative Team branch /
PREPARE-READY-COMMIT；**未降低任何 safety threshold**；**未扩大 P/T trust**；
未新增 persistent HandoffContract 状态机；**未加入 J_reserve/team soft shaping**；
未新增大规模测试套件；未访问 RRCT。

---

## 1. Root problem

Feedback099 已证明主链能工作（`margin −0.030110 → +0.652918`，ADOPTED + 证书 + 激活），
但 19 次 refinement decision 只有 1 次成功，且失败样本 deficit 全部 ≥ 1.846。
同时 forecast → action 期间 limiting UAV 的 margin 出现 1.12–5.15 的系统性恶化。

本轮要回答的是：**失败到底来自"修错了 UAV"、"来不及"，还是"物理上不可行"。**

---

## 2. Repair-target selection change

### 2.1 实现前先核实的前提（与原任务描述有出入，必须记录）

任务描述为"当前固定修 ranked[1]"。**代码核实结果：并非如此。**
`handoff_limiting_uav` 在 planner 中只有一处功能性使用
（`planner_manager.cpp` 的 `named_uav` 短路门控），三个 UAV 一直都会各自尝试 SCP。
Feedback099 的成功样本恰恰来自**既不是 ranked[1] 也不是最差**的那一架
（012329：forecast 排名第 0 的 drone2）。

因此若严格按"把 trial 限制在 {b, c}"实现，**会直接消灭 099 唯一的一次成功**。
本轮据此把候选集实现为：

```text
repair candidates = { ranked[1], ranked[2] }        # 发布，用于判定与日志
eligibility       = declared candidate OR own realized-seed window-min < required
```

第二个子句是必要的：forecast 排的是**realization 之前**的基线，
一架 forecast 健康的 UAV 仍可能 realized 出一个有缺陷的种子（099 实证）。
`repair_eligible` 只用于**判定与日志**，不用于禁止求解 —— 求解一次只要 1–13 ms，
而 2026-09-23_020949 的实测显示：禁止非优先候选求解会让两个 event 都失去唯一能解的 trial。

### 2.2 M2 limiter 保留为诊断语义

`limiting_uav = ranked[1]` 保留，但现在明确区分三种含义，日志同时打印：

```text
[RELAY_M2_LIMITING_UAV] ... M2_LIMITING_UAV=0 TEAM_REPAIR_CANDIDATES=[1,0] TEAM_REPAIR_PRIORITY_UAV=1
```

### 2.3 修复优先级（repairability）用真实数据而非启发式

需要恢复到 `required` 的缺口最小的通道 = 当前 margin 最高的**缺陷**通道。
该值直接取自发布的 forecast，因此每个节点推导出同一个顺序，不需要新的协议往返。

---

## 3. Two-candidate SCP implementation

- 新增 schedule 字段：`handoff_repair_candidates[]`、`handoff_repair_candidate_margins[]`、
  `handoff_repair_priority_uav`、`handoff_forecast_margins[]`、
  `handoff_t_cross / t_recover / t_star`、`handoff_detect_wall / request_wall`。
- 每个 candidate 跑**当前真实**的 `TEAM_T / TEAM_PT` 完整求解（同一 forecast snapshot、
  同一 critical window、同一 required margin；各自用自己的 topology 与 Local baseline）。
- **试验判定不等于 solver status**。`trial_feasible` 要求：
  SCP success ∧ exact nonlinear margin ≥ required ∧ (M2_after ≥ required ∨ 既有 Team acceptance 条件)
  ∧ 非零修改；并沿用既有的 static/dynamic/swarm/vxax/拓扑/latest-world recheck 检查。
- `M2_after` = 把本机 `margin_after` 代入该机通道、其余两架用发布 forecast margin 后的第二大值。
- deformation cost 用**已有的 trust 尺度**归一化，不裸加"米²+秒²"：

```text
deformation_cost = dp_norm / team_scp_p_trust_  +  dt_norm / team_scp_t_trust_ratio_
```

  为此新增只读访问器 `PolyTrajOptimizer::teamSCPTrustScale()`，**不改变任何调参值**。

### 3.1 单 winner：实测证明严格版会致命，已按证据放宽

第一版实现"非 winner 一律撤销"。首次实测（020309）暴露 **互相谦让**：
drone2 算出 winner=0，drone0 算出 winner=2，两边都撤销 → 谁都不修。
第二版收紧为"仅在发布的 priority 目标 == M2 limiting UAV 时才撤销"。
第二次实测（020949）又暴露互补缺陷：**两个 event 里 priority 候选根本没有跑 trial**
（它携带契约的 reference 在 realization/identity 校验就失败了），
而真正解出来的是另一个候选 → 唯一能解的 trial 被压掉。

**结论：在无新增协议往返的前提下，严格单 winner 在 2/2 事件中导致零修复。**
现规则为：eligible 且 trial feasible 即可提交；仅当"发布 priority 目标恰为 M2 limiting UAV"
时才允许撤销他人。该限制已写入代码注释并在此明确记录。

---

## 4. Earliest-crossing trigger

- 扫描 horizon 时记录**第一段连续** `M2(t_k) < trigger(=0.30)` 区间：
  `t_cross` = 首个交叉样本，`t_recover` = 该段结束；后续交叉不再延长它。
- `t_star = argmin M2` 保留，仅用于严重程度诊断。
- 契约窗口改为锚定在交叉区间：`acquire = max(activation, t_cross)`，
  `preserve = min(horizon_end, t_recover)`，再按既有 `relay_overlap_` 补足最小长度。
- **窗口语义仍是"全窗口 minimum"**，未退回单点（099 已证明单点 +1.0 / 窗口 −2.54 会误判）。
  实测交叉区间长度 0.49 / 0.56 s。

---

## 5. Full latency chain

统一由 `logRelayLatency(outcome)` 在**每个终态**输出（ADOPTED / SOLVER_INFEASIBLE /
TRIAL_INFEASIBLE / NOT_SELECTED），因此"没有可行候选"的 event 也可测量。

实测（3 次运行，全部有效 event）：

| run / contract / drone | L_detect→receive | L_receive→scp | L_scp | L_scp→outcome | L_forecast→outcome | time_to_cross | **INTERVENTION_SLACK** |
|---|---|---|---|---|---|---|---|
| 020949 c2 drone0 | 21.470 ms | 0.094 ms | 1.858 ms | 0.067 ms | **23.489 ms** | 0.607 s | **+0.583 s** |
| 020949 c1 drone2 | 31.140 ms | 0.044 ms | 13.515 ms | 0.074 ms | **44.772 ms** | 1.406 s | **+1.361 s** |
| 021544 c1 drone0 | 30.310 ms | 0.071 ms | 4.463 ms | 0.090 ms | **34.934 ms** | 0.745 s | **+0.710 s** |
| 021544 c1 drone1 | 27.169 ms | 0.072 ms | 2.990 ms | 0.060 ms | **30.291 ms** | 0.745 s | **+0.714 s** |

`INTERVENTION_SLACK = time_to_cross_at_forecast − L_forecast_to_outcome`，**4/4 全部为正**。

---

## 6. Activation lead analysis

**结论：activation lead 不是瓶颈，本轮不修改它。**（符合 §13 的"若已不是瓶颈就不要改"）

证据：

1. 从"检测到交叉"到"SCP 出结果"总耗时 **23.5 – 44.8 ms**；
2. 距离真正的 crossing 还有 **0.607 – 1.406 s**；
3. `INTERVENTION_SLACK` = **+0.583 … +1.361 s**，最坏情况仍有一个数量级余量；
4. 失衡的原因不在管道：管道只用掉了可用时间的 **2.4% – 5.4%**。

因此按 §14 的要求，**没有**把 activation lead 改成任何新数字，
也没有新增第三套 guard；`local_activation_margin` / `execution_margin` / 既有 transaction guard 原样保留。

---

## 7. Build

```text
catkin build -j2 --no-status
[build] Summary: All 25 packages succeeded!
[build] Warnings: None.
[build] Failed: None.
```

`git diff --check` → **PASS**。`ego_planner` / `multi_uav_formation` / `traj_opt` 均 0 warning。
新增 `.msg` 字段触发了 `--force-cmake` 路径，已正常生成。未使用任何 git 破坏性命令。

---

## 8. FULL production runs

命令（正式压力场景，绝对路径）：

```bash
./run_on.sh --ablation full --headless --timeout 200 \
  --scenario <abs>/ros_ws/src/multi_uav_formation/scenes/\
long_cylinder_forest_visibility_stress.json
```

| RUN_ID | 事件 | EXIT | BOOT-12 | contract | repair event | trial | ADOPTED |
|---|---|---|---|---|---|---|---|
| 20260923_020309_72622 | 规则 v1（严格单 winner） | 0 | YES | 1 | 1 | — (日志未覆盖不可行路径) | 0 |
| 20260923_020949_78945 | 规则 v2 | 0 | YES | 2 | 2 | 3 | 0 |
| 20260923_021544_86729 | 规则 v2 | 0 | YES | 1 | 1 | 3 | 0 |

三次均 `CORE_EXIT_CODE=0`、`SIMULATION_REACHED_BOOT_12=YES`、
`CLEANUP_FORCE_KILLED=0`、`CLEANUP_REMAINING_OWNED=0`、`CLEANUP_KILLED_FOREIGN_PROCESSES=0`。

> run 020309 用的是 telemetry 尚未覆盖不可行路径的中间版本，
> 该缺口在同轮内已修复（`logRelayLatency` + 不可行路径的 `[TEAM_REPAIR_TRIAL]`），
> 后续两次运行数据完整。

---

## 9. Repair option table（逐 event）

| event | M2 (min) | stable | ranked1 / **M2 limiting** | worst | candidates | priority | trials run | ranked1: deficit / feasible | other: deficit / feasible | selected | activated |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 020309 c1 | −2.956338 | 0 | 2 / **2** | 0 (−1.034@cross) | [2, 0] | 2 | drone0, drone2 | 4.841 / **no** | 5.172 / **no** | uav2 | no |
| 020949 c1 | −1.000357 | 1 | 0 / **0** | 2 (−1.000) | [0, 2] | 0 | drone2（drone1 zero-mod） | 4.994 / **no** | 未参与 | uav0（未参与） | no |
| 020949 c2 | −3.849160 | 1 | 2 / **2** | 0 (−4.346) | [2, 0] | 2 | drone0 | 4.725 / **no** | 未参与 | uav2（未参与） | no |
| 021544 c1 | −3.313218 | 2 | 0 / **0** | 1 (+0.087@cross) | [1, 0] | **1** | drone0, drone1（drone2 zero-mod） | 4.823 / **no** | 5.290 / **no** | uav1 | no |

逐 event 补充：

- **stable observer 被正确排除**：020949 c1 的 drone1（seed window-min = +1.000）与
  021544 c1 的 drone2（+1.000）都得到 `eligible=0 reason=NOT_ELIGIBLE`，未成为修复目标 ✅
- **修复目标 ≠ M2 limiting UAV**：021544 c1，`M2_LIMITING_UAV=0` 但
  `TEAM_REPAIR_PRIORITY_UAV=1` —— 因为 uav1 在交叉时刻 margin = +0.087（只差 0.11 就达标），
  而 uav0 是 −1.741。**规则按"最可修复"选中了 uav1**，这正是 §2 想要的行为 ✅（1/4 事件）
- 所有 4 个 event 的两个候选在 trial 时都已深度为负，`deficit` 4.72–5.29。

---

## 10. Success rate vs Feedback099

| 指标 | Feedback099 | Feedback100 |
|---|---|---|
| TEAM_REPAIR_EVENTS | 19 decisions | **4 events** |
| CANDIDATE_TRIALS_TOTAL | 19 | **9 refinement results** |
| RANKED1_TRIAL_FEASIBLE | 0 | **0** |
| OTHER_TRIAL_FEASIBLE | 0 | **0** |
| SELECTED_RANKED1 | — | 0 |
| SELECTED_OTHER | — | 0 |
| BOTH_FEASIBLE | 0 | **0** |
| BOTH_INFEASIBLE | — | **4 / 4** |
| NONZERO_TEAM_REFINEMENT_ADOPTED | 1 | **0** |
| REALIZED_M2_IMPROVEMENT | 1 | **0** |
| repair_event_success_rate | 5.3% (1/19) | **0 / 4 = 0%** |

**必须诚实说明**：本轮成功率更低，但这**不能归因于双候选机制无效**。
099 的 19 次决策覆盖了 deficit 0.230（成功）到 5.473 的整个范围；
本轮 4 个 event 的 deficit **全部落在 4.72–5.29**，即全部位于 099 已证明
"即使无信赖域也 infeasible"的区间。本轮样本里**没有出现过小缺口事件**，
因此测的是"目标选择"而不是"能力边界"。

按 §22 要求以 event（而非 candidate trial）为分母：`repair_event_success_rate = 0/4`。

---

## 11. Forecast → activation margin decay

| event | forecast 时的 M2 | 实测 realized-seed window-min | 衰减 |
|---|---|---|---|
| 020949 c2 | −3.849160 | −4.407788 | **+0.559** |
| 021544 c1 | −3.313218 | −4.126779 / −4.788614 | **+0.814 / +1.475** |
| 020309 c1 | −2.956338 | −4.380297 / −4.971717 | **+1.424 / +2.015** |
| 020949 c1 | −1.000357 | −4.794869 | **+3.795** |

**衰减 0.56 – 3.80，4/4 event 全部为正**，与 Feedback099 的 1.12–5.15 一致。

**恶化发生的阶段（§20 Q5 的答案）**：
管道只占 23–45 ms（可用时间的 2.4–5.4%），而衰减量是 0.56–3.80 ——
**恶化几乎全部发生在 forecast 与 realized seed 之间的 Local/团队参考滚动里，
不在 detect → request → SCP → commit 这条链上。**

---

## 12. Whether activation lead was changed

**ACTIVATION_LEAD_WAS_MODIFIED: NO。**

不是瓶颈，证据见 §6：`INTERVENTION_SLACK` 4/4 为正（+0.583 … +1.361 s），
管道耗时比可用时间小 1–2 个数量级。按 §13/§14 的要求，
未把 lead 改为 50/100/200 ms 之类的拍脑袋值，也未新增重复 guard。

**下一轮若仍要处理"来不及"，正确目标不是 activation lead，而是 Local 基线本身。**

---

## 13. Safety/liveness

| 指标 | 020309 | 020949 | 021544 |
|---|---|---|---|
| `TERMINAL_HOLD_ENTER` | 2 | **0** | **0** |
| `TERMINAL_HOLD_STARVATION` | 0 | 0 | 0 |
| `MIN_BUDGET_APPLIED` | 1 | 52 | 82 |
| `ACTIVE_PVA_MISMATCH` | 0 | 0 | 0 |
| `PARTIAL_TEAM_ACTIVATION` | 0 | 0 | 0 |
| `EXPECTED_PREDECESSOR_INVALIDATED` | 0 | 0 | 0 |
| `ACTIVATION_HANDOFF_DISCONTINUITY` | 0 | 0 | 0 |
| `END_BEFORE_NEXT` | 2 | **0** | **0** |
| static/dynamic collision | 0 | 0 | 0 |
| swarm violation / unvalidated execution | 0 / 0 | 0 / 0 | 0 / 0 |
| 可见性 uav1 / uav2 / uav3 | 0.7555 / 0.9766 / 0.9638 | 0.9261 / 0.9741 / 0.9659 | 0.9371 / 0.9823 / 0.9762 |
| `fov_mismatch_rate_team` | 0.00452 | 0.01684 | **0.00000** |
| `executed_longest_blackout_s` | 0.1665 | 0.1998 | **0.0000** |
| 几何退化占比 | 5622/6239 = **90.1%** | 904/8119 = 11.1% | 447/8330 = 5.4% |

- **无任何安全违规**：碰撞 0、swarm 0、未验证执行 0、PVA mismatch 0、部分激活 0。
- `TRANSACTION_ACTIVATED` = 87 / 140 / 161 → Feedback096 事务契约仍在工作。
- `TERMINAL_HOLD_ENTER` 从 2 降到 0（后两次运行），`TERMINAL_HOLD_STARVATION` 三次全 0。
- 需要注意并已记录：**run 020309 是几何离群点**（退化占比 90.1%，uav1 可见性 0.7555，
  黑障 0.167 s）；后两次运行几何正常（11.1% / 5.4%），可见性 0.93–0.98。
  该离群与本轮改动无因果关系（本轮只改了修复目标选择与窗口锚定，未触碰安全语义），
  但按"不能用可见性收益掩盖问题"的原则在此明确标出。

### 13.1 计算开销（§24）

单个 trial 的 SCP 本体 **1.858 – 13.515 ms**；同一 event 内的 candidate trial 是**串行**的，
`L_receive→scp` 仅 0.044–0.094 ms，`L_scp→outcome` 0.060–0.090 ms。
双候选总耗时最坏约 **13.5 ms**，远低于 Local tick，**没有观察到阻塞 Local rolling**，
因此保持简单串行实现，**未引入线程**。

---

## 14. Remaining problem

1. **主因已定位：物理不可行 + Local 基线恶化，不是目标选择、也不是时延。**
   4 个 event 的 deficit 全在 4.72–5.29，而 099 已证明该区间内即使无信赖域也 infeasible。
   本轮的双候选机制**从未获得一个"小缺口"的 event 去证明自己**。

2. **契约只在几何已经崩坏时才产生。** 020309 的退化占比 90.1%、021544 的
   `M2_limiting=m2_min=−3.31` —— 交叉发生时缺口就已经是 3+。
   这解释了为什么"及早触发"本轮没有带来改善：`t_cross` 确实比 `t_star` 早 0.49–0.56 s，
   但那 0.5 s 内的缺口本就已无法修复。

3. **priority 候选参与率不足。** 020949 的两个 event 里，被指名的 priority 候选
   都因为携带契约的 reference 在 realization / identity 校验失败而未进入 trial
   （`LOCAL_MINCO_REALIZATION_FAILED` / `HANDOFF_CONTRACT_IDENTITY_OR_TIME_INVALID`）。
   即"要修的那一架恰好没参与"。

4. **单 winner 在分布式约束下无法严格实现。** 本轮实测了两种尝试并都被真实数据否决
   （互相谦让 / 压掉唯一能解的 trial）。当前规则是有据可依的折中，但
   "至多一个 winner"只能在"发布 priority 目标恰为 M2 limiting UAV"时保证。

5. **`stable_uav` 与候选集的时间基准不一致**：`stable` 取窗口首样本，
   候选集取交叉样本，因此 020309 出现 `stable_uav=0` 而 `candidate_2=0` 的观感矛盾。
   仅为遥测一致性问题，不影响判定。

6. run 020309 的几何离群（90.1% 退化）原因未查，需要更多样本区分方差与机制。

---

## 15. Whether J_team_reserve is still necessary

**YES —— 本轮数据正好构成 §16 所要求的那个前置条件。**

§16 说：只有当"forecast → SCP/commit 时延已经很小，但最新 Local baseline 仍在
几十/几百 ms 内持续大幅恶化"时，下一轮才考虑短时 `J_team_reserve`。实测：

| 条件 | 实测值 | 是否满足 |
|---|---|---|
| 管道时延已经很小 | L_forecast→outcome = **23.5 – 44.8 ms** | ✅ |
| 可用时间远大于管道 | INTERVENTION_SLACK = **+0.583 … +1.361 s** | ✅ |
| 但基线仍在快速恶化 | forecast→seed 衰减 = **0.56 – 3.80** | ✅ |

即：**管道已经不是问题，问题是在这段时间里 Local 基线自己把未来带向了更差的几何。**
本轮按 §16 明确要求**没有**实现 `J_team_reserve`；下一轮它是首选方向。

---

## 16. 结论字段

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES
FULL_SIMULATION_RUN: YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO

M2_USED_ONLY_FOR_RISK_DETECTION: YES
  limiting_uav 保留为诊断；repair candidates 由 {ranked1, ranked2} 明确定义并发布
DUAL_REPAIR_CANDIDATE_SCP_IMPLEMENTED: YES
  每个候选跑真实 TEAM_T/TEAM_PT；判定含 margin_after>=required 与 M2_after；
  deformation_cost = dp/team_scp_p_trust_ + dt/team_scp_t_trust_ratio_（同一 trust 尺度归一化）
EARLIEST_CROSSING_TRIGGER_IMPLEMENTED: YES
  t_cross = 第一段 M2<trigger 连续区间的起点；t_recover = 该段终点；
  t_star = argmin（仅诊断）；窗口锚定在 [t_cross, t_recover]，仍取全窗口 minimum

TEAM_REPAIR_EVENTS: 4
TEAM_REPAIR_SUCCESS: 0
TEAM_REPAIR_SUCCESS_RATE: 0.000   # 0/4，以 event 为分母（§22）

RANKED1_TRIAL_FEASIBLE: 0
OTHER_TRIAL_FEASIBLE: 0

SELECTED_RANKED1: 0
SELECTED_OTHER: 0

BOTH_FEASIBLE: 0
BOTH_INFEASIBLE: 4

NONZERO_TEAM_REFINEMENT_ADOPTED: 0
REALIZED_M2_IMPROVEMENT_COUNT: 0

FORECAST_TO_ACTIVATION_P50_P95_MAX: 34.934 / 44.772 / 44.772 ms
  # 以 L_forecast_to_outcome 度量（detect→trial 终态）；无 adopted event，故无 activation 样本
FORECAST_TO_SCP_START_P50_P95_MAX: 30.594 / 44.899 / 44.899 ms   # L_detect_to_receive + L_receive_to_scp
SCP_TO_COMMIT_P50_P95_MAX: 0.067 / 0.090 / 0.090 ms             # L_scp_to_outcome（无 commit 样本）
COMMIT_TO_ACTIVATION_P50_P95_MAX: N/A（本轮 4/4 event 无可行候选，未进入 commit）

DUAL_CANDIDATE_SCP_TOTAL_P50_P95_MAX: 13.515 / 13.515 / 13.515 ms  # 串行，最坏单次 SCP
  注：双候选并不总是同时求解（见 §9「trials run」列），最坏单次 SCP 即上界

ACTIVATION_LEAD_WAS_MODIFIED: NO
ACTIVATION_LEAD_MODIFICATION_REASON:
  不是瓶颈。INTERVENTION_SLACK 4/4 为正（+0.583 … +1.361 s），
  detect→SCP 终态仅 23.5–44.8 ms，管道只消耗可用时间的 2.4–5.4%。
  按 §13「若当前 activation policy 已经不是瓶颈：不要修改它」处理，
  未新增数值、未新增 guard。

J_TEAM_RESERVE_NEEDED_NEXT: YES
REASON:
  §16 的前置条件已被本轮实测满足：管道时延极小（23.5–44.8 ms）而
  forecast→realized-seed 衰减仍有 0.56–3.80，说明恶化来自 Local 基线滚动本身，
  而非 detect→commit 链路。

TEAM_CAUSED_HOLD: 2          # 020309 的 TERMINAL_HOLD_ENTER；后两次为 0
TEAM_CAUSED_STARVATION: 0    # 三次运行全部为 0
PARTIAL_TEAM_ACTIVATION: 0
SAFETY_REGRESSION: NO
  # 碰撞 0、swarm 违规 0、未验证执行 0、PVA mismatch 0、EXPECTED_PREDECESSOR_INVALIDATED 0、
  # ACTIVATION_HANDOFF_DISCONTINUITY 0；TRANSACTION_ACTIVATED = 87/140/161（096 契约仍在工作）

RUN_DIRS:
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_020309_72622
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_020949_78945
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_021544_86729

CURRENT_MAIN_BOTTLENECK:
  PHYSICAL_INFEASIBILITY  (primary)
  + LOCAL_BASELINE_DETERIORATION  (primary, co-cause)
  证据：
    (a) 4/4 event 的 trial deficit = 4.72–5.29，全部落在 Feedback099 已证明
        「无信赖域探测也不可行」的区间；RANKED1 与 OTHER 的 trial_feasible 均为 0。
    (b) INTERVENTION_SLACK 4/4 为正（+0.583 … +1.361 s），管道仅 23.5–44.8 ms
        → PIPELINE_LATENCY 已可排除。
    (c) forecast→realized-seed 衰减 0.56–3.80（4/4 为正）而管道只占其中 2.4–5.4%
        → 恶化发生在 Local 基线滚动，不在 detect→SCP→commit 链路。
    (d) REPAIR_TARGET_SELECTION 已被实现并生效（stable observer 被正确排除，
        1/4 event 选中了非 M2-limiting 但更可修复的通道），但本轮样本中
        没有任何 event 存在可修复的候选，因此该机制未被区分性检验。

REPAIR_TARGET_SELECTION_IMPLEMENTED: YES
REPAIR_TARGET_SELECTION_PROVEN_BENEFICIAL: NOT_YET   # 本轮 0 个可修复候选可供区分
PIPELINE_LATENCY_IS_BOTTLENECK: NO
STRICT_SINGLE_WINNER_ACHIEVABLE_WITHOUT_NEW_PROTOCOL: NO
  # 两种实现均被实测否决：v1 互相谦让（020309），v2 压掉唯一能解的 trial（020949）

MINCO_REWRITTEN: NO
SCP_REWRITTEN: NO
FEEDBACK096_SINGLE_FLIGHT_REWRITTEN: NO
PERSISTENT_CONTRACT_STATE_MACHINE_ADDED: NO
SAFETY_THRESHOLD_LOWERED: NO
PT_TRUST_ENLARGED: NO
J_TEAM_RESERVE_ADDED: NO
GIT_RESET_USED: NO
GIT_CHECKOUT_OVERWRITE_USED: NO
GIT_CLEAN_USED: NO
EXISTING_MODIFICATIONS_ROLLED_BACK: NO

FORECAST_AUTHORITY_FIXED: YES
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
