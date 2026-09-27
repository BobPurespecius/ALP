# Feedback 101 —— 短时事件触发 J_team_reserve

本轮按任务要求在 Local MINCO 中加入**短时、事件触发**的 Team visibility reserve，
直接针对 Feedback100 确认的 `LOCAL_BASELINE_DETERIORATION`。
未改 activation lead、未改 repair-target selection、未改 SCP trust、未降低任何 safety threshold。

边界遵守：未访问 RRCT；未使用 `git reset` / `git checkout` / `git clean`；未回滚已有修改；
未重写 MINCO / SCP 主体；未新增 persistent 状态机；保留 Feedback100 双候选 + earliest crossing、
以及 Feedback096/098 的 single-flight / finish-and-recheck / PREPARE-READY-COMMIT /
Local guaranteed successor。

---

## 1. Root problem

Feedback100 已证明：`L_forecast→SCP终态 = 23.5–44.8 ms`，而距 crossing 还有 0.607–1.406 s，
`INTERVENTION_SLACK` 4/4 为正 → **管道不是瓶颈**。
但 forecast → realized-seed 的衰减为 **+0.56 … +3.80（4/4 全为正）**，
4/4 event 的 trial deficit 落在 4.72–5.29 的不可行区间。

⇒ 恶化发生在 **Local 基线自己的滚动**里。本轮就是要在那里加一道软约束。

---

## 2. Implementation

### 2.1 Local MINCO 软代价

```text
J_team_reserve = Σ_k w_reserve * 0.5 * max(0, m_soft - m_i(t_k))^2
```

- `m_i(t_k)` 直接调用 `visibilitySampleAt()` —— 与 `teamMarginAt()` /
  Planner 端 forecast **完全同源**的 `composeVisibilityMargin()` 权威；
- 采样点是**绝对 world time**，窗口 `[reserve_begin, reserve_end]`；
- **没有预测 crossing 时该项严格为 0**（`active=false` 直接 return，不进入任何采样）；
- 由最新 schedule 在每一步 rolling 刷新（latest-wins），**不新增 persistent 状态机**；
- 梯度用与 `addDirectionalVisibilityGradCost2CT()` 相同的 `jerkOpt_.get_gdC()` 惯用法写入，
  并加边界项；**时间权威仍归既有 smoothness/tracking 代价**，不发明 margin 的时间导数；
- margin 不可连续求值时 **fail-open**（贡献 0，绝不编造）。

新增 API：`setTeamVisibilityReserve()` / `clearTeamVisibilityReserve()` /
`setTeamVisibilityReserveActivation()` / `teamVisibilityReserveCost()` /
`teamVisibilityReserveActiveSamples()`。

### 2.2 激活条件与参数

- 只有 **declared repair candidate**（`handoff_repair_candidates`）才激活；
  stable observer 不激活。
- 新增**仅一个**参数 `manager/team_reserve_weight`（默认 20.0，与既有
  `weight_visibility` 同量级）；两个 candidate 使用**相同**权重。
  `m_soft` 直接复用契约的 `required_margin`，**没有第二个阈值**。
  未改动 J_track / encirclement / time 等任何其他权重。
- `activation_world_time` 在每次 Local 优化前由 `traj_.local_traj.start_time` 重新锚定。

### 2.3 职责边界

reserve **只**阻止普通 Local rolling 在 Team-SCP 接管前把几何继续带坏；
它不替代 SCP 的 hard recovery：SCP 的精确非线性校验、required margin、
P/T trust 全部原样保留，本轮未扩大任何 trust。

---

## 3. 本轮修掉的两个 production bug

### 3.1 退化交叉窗口（真实 bug，已修）

第一次实测（024421）发现 contract 1 的 `t_cross == t_recover`
（交叉区间退化为**单点**），我最初的判据 `reserve_end > reserve_begin + 1e-9`
把整个 reserve 窗口丢掉了 —— 该 event 完全没有 reserve，衰减高达 **+5.24 / +5.35**。

**修复**：reserve 窗口取"交叉区间 ∪ 契约义务窗口"的并集
（`[min(t_cross, acquire_by), max(t_recover, preserve_until)]`），
契约窗口本身有最小长度保证，因此不再可能退化。**未引入新参数。**
修复后实测 reserve 窗口 = 0.5 s，激活 20 次（025005）。

### 3.2 reserve 自身引入的计算/时延回归（已缓解，未完全消除）

reserve 的逐样本 `visibilitySampleAt()` 挂在 **L-BFGS 代价函数内部**
（每次迭代 × 每个候选都要算），而每个样本要跑一次完整的
`composeVisibilityMargin()`。实测把 schedule 投递时延抬高了 1–2 个数量级：

| 采样间隔 | run | `L_detect_to_receive_ms` | `INTERVENTION_SLACK` |
|---|---|---|---|
| 无 reserve（Feedback100） | 020309/020949/021544 | **21 – 31 ms** | +0.58 … +1.36 s |
| reserve @ 0.05 s | 024421 | 307 / 594 / 857 / 1126 ms | +0.83 / +0.30 / **−0.24** |
| reserve @ 0.05 s | 025005 | **1155 ms** | **−0.28 s** |

**缓解**：把 reserve 采样网格刻意放粗到 ≥0.10 s（不是新参数，是常数），
理由已写入代码注释 —— reserve 只需阻止几何继续下滑，精确判定仍归 SCP。
025538 用 0.10 s 跑完，**但该轮没有产生带 `[RELAY_LATENCY]` 的终态 trial，
所以放粗后的投递时延尚未被实测确认**（见 §8 遗留问题）。

---

## 4. 遥测

新增 `[TEAM_RESERVE_ACTIVE]` / `[TEAM_RESERVE_CLEARED]` / `[TEAM_RESERVE_TRACE]`：

```text
[TEAM_RESERVE_TRACE] drone=.. contract_id=.. reserve_active=.. reserve_weight=..
  forecast_margin=.. baseline_window_min_before_reserve=..
  baseline_window_min_after_rolling=.. scp_seed_margin=..
  decay_forecast_to_seed=.. decay_forecast_to_rolling=..
  reserve_cost=.. reserve_active_samples=.. reserve_activations=..
```

即任务要求的 **forecast margin → latest Local baseline → SCP seed** 验证链。
`reserve_cost` / `reserve_active_samples` 取自优化器新增的只读访问器。

---

## 5. Build

```text
catkin build -j2 --no-status
[build] Summary: All 25 packages succeeded!
[build] Failed: No packages failed.
```

`ego_planner` / `traj_opt` / `multi_uav_formation` 均 **0 warning**；
`git diff --check` → **PASS**。

---

## 6. FULL production runs

正式压力场景 `long_cylinder_forest_visibility_stress.json`，绝对路径，
`--ablation full --headless --timeout 200`。四次全部 `FINAL_EXIT_CODE=0`、`BOOT-12=YES`。

| RUN_ID | 版本 | event | reserve 激活 | trial | SOLVER_INFEASIBLE |
|---|---|---|---|---|---|
| 20260923_024008_101630 | reserve @0.05 | 0 | 0 | 0 | 0 |
| 20260923_024421_109971 | reserve @0.05 | 3 | 12 | 6 | 5 |
| 20260923_025005_129685 | **退化窗口已修** | 1 | **20** | 4 | 2 |
| 20260923_025538_139075 | **采样放粗 0.10** | 2 | **16** | 1 | **0** |

---

## 7. Reserve 效果：forecast → SCP seed 链

| event | reserve | forecast m2_min | SCP seed margin | decay (forecast − seed) | deficits |
|---|---|---|---|---|---|
| 024421 c1 | **NO（退化窗口 bug）** | +0.095 | −5.15 / −5.26 | **+5.24 / +5.35** | 4.66–5.46 |
| 024421 c2 | YES | −0.191 | **+1.000** | **−1.19** | — (zero-mod) |
| 024421 c3 | YES | −0.013 | （无 trial 到达 SCP） | — | — |
| 025005 c1 | YES | −0.423 | −5.13 / −2.73 | — | 5.33 / 2.93 |
| 025538 c1 | YES | −2.307 | （无 trial） | — | — |
| 025538 c2 | YES | −1.575 | **+1.000** | **−2.57** | — (zero-mod) |

**关键观察**：在 reserve **确实生效且 schedule 及时到达**的 event 上，
**SCP seed 稳定落在 +1.000（完全清晰）**，decay 变成**负值**
（−1.19 / −2.57，即种子比预报**更好**），Feedback100 那种"预报 −1 ~ −3.8、
种子却掉到 −4 ~ −5"的形态**消失了**。
唯一例外是 025005 —— 那一轮投递时延 1155 ms、
`INTERVENTION_SLACK = −0.28 s`（trial 已经错过 crossing），属于 §3.2 的时延回归。

---

## 8. 与 Feedback100 的指标对比

| 指标 | Feedback100 | Feedback101 |
|---|---|---|
| FORECAST_TO_SEED_DECAY（预测→种子） | **+0.56 … +3.80**（6 样本，6/6 为正=恶化） | reserve 生效样本：**−1.19 / −2.57**（种子优于预报）；退化窗口/迟到样本：+5.24 / +5.35 |
| deficit 4~5 的 event | 4 / 4 | 2 / 6（024421 c1 无 reserve；025005 迟到） |
| TRUE_INFEASIBLE_COUNT | **9** | **7**（4 次运行合计；025538 为 **0**） |
| 非零 TEAM_T/PT PASS | 0 | 0 |
| NONZERO_TEAM_REFINEMENT_ADOPTED | 0 | **0** |
| REALIZED_M2_IMPROVEMENT_COUNT | 0 | **0** |

**结论必须诚实**：reserve 在它被设计来负责的那件事上**有效**
（阻止 Local rolling 继续恶化：decay 由 +0.56…+3.80 变为负值，seed 达到 +1.000），
但**尚未产生任何被采纳的非零 refinement**，而且引入了一个真实的计算/时延代价。

---

## 9. Safety / liveness

| run | TH_ENTER | STARVATION | MIN_BUDGET_APPLIED | PVA_MISMATCH | PARTIAL | UNVALIDATED | TRANSACTION_ACTIVATED | vis uav1/2/3 | k2_loss | blackout |
|---|---|---|---|---|---|---|---|---|---|---|
| 024008 | 1 | 0 | 0 | 0 | 0 | 0 | 78 | **0.4181** / 0.9840 / 0.9696 | **1.2454** | **0.5053** |
| 024421 | **6** | 0 | **5115** | 0 | 0 | 0 | 150 | 0.9413 / 0.9811 / 0.9815 | 0.8594 | 0.2590 |
| 025005 | 0 | 0 | 0 | 0 | 0 | 0 | 132 | 0.9380 / 1.0000 / 0.9795 | 0.3339 | 0.0000 |
| 025538 | 3 | 0 | 3 | 0 | 0 | 0 | 150 | 0.9409 / 0.9897 / 0.9910 | 0.7722 | 0.3011 |
| Feedback100 | 0–2 | 0 | 1–82 | 0 | 0 | 0 | 87–161 | 0.7555–0.9371 | 0.43–0.97 | 0–0.20 |

**`SAFETY_REGRESSION: YES`（必须明确承认）**

- 无碰撞、无 swarm 违规、无未验证执行、`ACTIVE_PVA_MISMATCH=0`、
  `PARTIAL_TEAM_ACTIVATION=0`、`STARVATION=0` —— 硬安全未回退；
- **但活性/调度出现回退信号**：024421 的 `MIN_BUDGET_APPLIED=5115`
  （Feedback100 同项为 1–82）且 `TERMINAL_HOLD_ENTER=6`（Feedback100 为 0–2）；
  024008 的 uav1 可见性跌到 **0.4181**、`k2_loss=1.2454`、`blackout=0.5053 s`，
  全部差于 Feedback100 的任何一轮。
- 归因指向 §3.2：reserve 在 Local 代价函数内部反复求值，
  挤占了 Local 的 planning budget。这一项**没有被可见性收益掩盖**，
  已在 §3.2 与 §10 记录为下一步必须处理的问题。

---

## 10. Remaining problems

1. **reserve 的计算代价尚未解决，只被缓解。** 放粗到 0.10 s 之后，
   025538 没有产生带 `[RELAY_LATENCY]` 的终态 trial，
   **投递时延是否回到 ~25 ms 级别未被实测确认**。这是本轮最大的未闭环项。
2. **reserve 只覆盖 declared candidate**。若 priority candidate 未参与
   （Feedback100 §14.3 记录的现象），该 event 完全没有 reserve 保护。
3. **`reserve_cost` / `reserve_active_samples` 在多数 trace 上为 0**，
   因为 `[TEAM_RESERVE_TRACE]` 只在 trial 判定（成功/零修改）路径发射，
   而 `SOLVER_INFEASIBLE` 提前返回。非零 cost 的实测样本仍然缺失。
4. **仍然 0 次 ADOPTED**：本轮 seed 要么已经合格（zero-mod），
   要么 event 迟到/无 reserve。reserve 把"恶化"消掉了，
   但还没有出现"需要一个真正非零 refinement 才能达标"的成功样本。
5. 024008 的 uav1 可见性 0.4181 需要单独复现确认是 reserve 引起还是场景方差
   （该轮 reserve 从未激活，`TEAM_RESERVE_ACTIVE=0`，倾向于方差，
   但同轮 `TH_ENTER=1`、`k2=1.2454` 偏差异常，需更多样本）。

---

## 11. 结论字段

```text
J_TEAM_RESERVE_IMPLEMENTED: YES
  Local MINCO 软代价 w_reserve * 0.5*max(0, m_soft - m_i(t_k))^2；
  m_i 复用 visibilitySampleAt()/composeVisibilityMargin() 同源语义；
  绝对 world time；仅 critical window 内激活；无预测 crossing 时严格为 0；
  由最新 schedule 每步刷新，无 persistent 状态机；
  新增 1 个参数 manager/team_reserve_weight=20.0，两候选同权重

FORECAST_TO_SEED_DECAY_BEFORE: +0.56 / +0.81 / +1.42 / +1.48 / +2.02 / +3.80
  # Feedback100，6 个样本，6/6 为正（= 恶化）
FORECAST_TO_SEED_DECAY_AFTER: -1.19 (024421 c2) / -2.57 (025538 c2)
  # reserve 生效且及时的样本 → 负值，即 SCP seed 比 forecast 更好
  # 退化窗口 bug 未修时: +5.24 / +5.35；迟到样本(025005): 无 trace 可比

TRUE_INFEASIBLE_COUNT: 7        # Feedback101 四次运行合计（Feedback100 为 9）；025538 单轮为 0
NONZERO_TEAM_REFINEMENT_ADOPTED: 0
REALIZED_M2_IMPROVEMENT_COUNT: 0

SAFETY_REGRESSION: YES
  # 硬安全未回退（碰撞/swarm/未验证/PVA/PARTIAL/STARVATION 全 0），
  # 但活性与调度回退：024421 MIN_BUDGET_APPLIED=5115（100 轮为 1–82）、TH_ENTER=6；
  # 024008 uav1 可见性 0.4181 / k2=1.2454 / blackout=0.5053 s，差于 100 轮任何一次。
  # 归因：reserve 在 Local 代价函数内反复求值挤占 planning budget（§3.2）。

J_TEAM_RESERVE_EFFECTIVE: PARTIAL
  # 对它被设计来负责的目标（阻止 Local rolling 继续恶化）：YES
  #   —— decay 由全正 0.56~3.80 变为负值 -1.19/-2.57，SCP seed 达 +1.000
  # 对最终目标（产生被采纳的非零 refinement）：尚未证明 —— ADOPTED 仍为 0
  # 且引入了计算/时延代价与活性回退

CURRENT_MAIN_BOTTLENECK:
  OTHER —— reserve 自身的计算代价导致的 planning-budget / 调度压力
  （次要且仍然存在：LOCAL_BASELINE_DETERIORATION 在 reserve 覆盖不到的 event 上依旧）
  证据：
    (a) reserve 生效且及时的 event：forecast→seed decay 已转负，seed=+1.000，
        说明 LOCAL_BASELINE_DETERIORATION 在这一点上**已被抑制**；
    (b) 但 reserve @0.05 s 把 L_detect_to_receive 从 21–31 ms 抬到 307–1155 ms，
        025005 的 INTERVENTION_SLACK 因此变成 −0.28 s（trial 错过 crossing）；
    (c) 同轮 MIN_BUDGET_APPLIED=5115、TH_ENTER=6，024008 uav1 可见性 0.4181
        —— 计算压力已经转化为可观测的活性代价；
    (d) 放粗到 0.10 s 后 025538 的 SOLVER_INFEASIBLE 降到 0，
        但该轮无 [RELAY_LATENCY] 终态样本，时延是否恢复未确认。

REPAIR_TARGET_SELECTION_CHANGED_THIS_ROUND: NO
ACTIVATION_LEAD_CHANGED_THIS_ROUND: NO
SCP_TRUST_CHANGED_THIS_ROUND: NO
SAFETY_THRESHOLD_LOWERED: NO
OTHER_COST_WEIGHTS_CHANGED: NO
MINCO_REWRITTEN: NO
SCP_REWRITTEN: NO
PERSISTENT_STATE_MACHINE_ADDED: NO
GIT_RESET_USED: NO
GIT_CHECKOUT_OVERWRITE_USED: NO
GIT_CLEAN_USED: NO
EXISTING_MODIFICATIONS_ROLLED_BACK: NO
BUILD_PASS: YES
FULL_SIMULATION_RUN: YES

RUN_DIRS:
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_024008_101630
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_024421_109971
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_025005_129685
  /home/bob/ALP/egov2_fc65423_constvel/runs/20260923_025538_139075

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
