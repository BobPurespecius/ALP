# Feedback 127 — Topology candidate selection 专项只读审计

日期:2026-09-26。工作区:`/home/bob/ALP/egov2_fc65423_constvel`。
性质:**只读**。未修改 production、未编译、未运行测试、未启动仿真、未调参;未访问 /home/bob/RRCT;未执行任何 git reset/checkout/restore/clean。
审计对象:当前最新 dirty source + 唯一有效可分析 run `runs/20260926_225042_830329`(BOOT-12、FINAL_EXIT_CODE=0、306 MB stdout、visibility.csv 2347 样本)。执行基线:K2=99.4771、ALL3=89.2375、K2 最长丢失=0.3994 s、blackout=0、K2 丢失总量=0.400 s、ALL3 丢失总量=8.233 s、K3 recovered=9 事件。

---

## 1. "正确 topology" 的可审计定义与本 run 的可测边界

几何定义(按 mandate):对每个 K3/LOS 事件,以 primary blocker identity、target 预测、blocker center/radius、LEFT/RIGHT observation frame 定义真实观察侧;正确候选 = 实际轨迹从 blocker 阴影侧脱离更快的一侧。

**遥测边界(诚实声明)**:run 遥测不记录逐 candidate 的多项式系数/坐标,因此 `actual_geometric_side`(候选相对 blocker 的真实通过方向)与逐候选 `predicted_first_binary_recovery_time` 无法从日志重建——该项对绝大多数 batch 记为 `UNRESOLVED_DUE_TO_MISSING_TELEMETRY`。本审计的 oracle 降级为**事件级真值**:visibility.csv 的 executed binary truth + K3_EVENT_RESULT 的 recovery_observed_world_time/time_to_recovery + commit kind 时间线。这足以回答"selection 是否选错"的方向性问题,不足以逐候选重排。

可比的替代判据(源码+遥测):候选的评价指标本身(K2/C3/D3)是在同一 world-time snapshot、同一 activation、同一 peer 冻结契约下由 `evaluateLocalK3WindowMetrics` 计算的,左侧/右侧差异反映的就是 evaluator 对两条轨迹的预测可见性差异——evaluator 一致性(§7)有源码级答案。

## 2. Comparator 真实控制流(从当前 dirty source 重建,非历史报告)

`finalizeCapturedCandidates()`(planner_manager.cpp:1482)逐 set( NOMINAL set 在前,SIDE set 在后)逐 candidate:

| # | 层 | INPUT | 比较/条件 | TOLERANCE | EARLY_RETURN | 能否淘汰候选 |
|---|---|---|---|---|---|---|
| 0 | batch 冻结 | activation=now+margin,全候选共享 | `[candidate-batch-activation]` | — | 否 | — |
| 1 | 构造 | candidate.success | false→skip | — | continue | 是(构造) |
| 2 | handoff | prepareLocalHandoff(reanchor_to_now=**false**) | 失败→`[moving-rehead-reject]` | — | continue | 是 |
| 3 | 静止假设 | duration<coverage 且速度<0.2 | SHORT_STATIONARY | — | continue | 是 |
| 4 | **统一 hard preflight** | validateRetimedLocalSfc(连续 checker) ∧ checkActiveHandoff ∧ validateExecutionTrajectory(dyn/static/dynamic-hard/swarm 0.5m) | safe=0→`[local-geometry-candidate]` | 连续 checker ≤1e-6 | continue | 是(唯一 hard-safety 出口) |
| 5 | 证书 | hardCheckedUntil>activation | UNKNOWN→skip | 1e-6 | continue | 是 |
| 6 | **freeze** | revision++ / payload_hash / activation / checked_until 固定,`[FROZEN_CANDIDATE]` | FROZEN_ACTIVATION_MUTATED 防御 | 1e-9 | — | — |
| 7 | K3 窗口指标 | evaluateLocalK3WindowMetrics on `[max(ev_begin,activation), min(ev_end, baseline.start+baseline.duration)]` | 窗口必须 ⊆ [candidate.start, candidate.end] 否则 invalid | sample dt=0.1s | — | 指标 invalid=比较劣势 |
| 8 | **K2 gate** | candidate.K2 ≥ baseline.K2 | K2_GATE_REJECT_CANDIDATE / K3_WINDOW_INVALID / K2_BASELINE_UNAVAILABLE(SIDE 无 baseline 时全禁) | 1e-9 | 禁止进入 C3/D3 | 是(eligibility) |
| 9 | K2 对决 | 双方 one-ok-one-not | ok 者胜 | 1e-9 | — | — |
| 10 | **C3** | C3 更高者胜 | C3_HIGHER/OPPONENT_C3_HIGHER | **1e-9** | — | — |
| 11 | **D3** | D3 更低者胜 | D3_LOWER/OPPONENT_D3_LOWER | **1e-9** | — | — |
| 12 | tie→side preference | last_committed_side(SIDE_PREFERENCE_HOLD) | 跨 cycle 轻偏好,非 hard lock | — | — | — |
| 13 | tie→ORIGINAL_ORDER 链 | target_side_static_topology 保持 → visibility_better(LocalVisibilityPreference 字典序)→ geometryBetter(cost) | — | — | — | — |
| 14 | 兜底 | best=null 且有 reserved_first_safe 且 safety-required | SAFETY_SUCCESSOR_QUALITY_VETO_BYPASS | — | — | — |

指标定义(7790 区域):**K2 = 窗口内 ≥2 机 binary 可见样本占比;C3 = 窗口内 3 机全可见样本占比;D3 = Σ dt·max(0, required_margin − margin(t))**。三者同 primitive、同 yaw 推进、同 target/peer world-time。**没有任何"首次恢复时间"项。**

## 3. 数据与漏斗(run 830329,逐 batch 遥测 90 行,事件 23 个/闭合 21)

```
TOTAL_COMPARABLE_BATCHES(K3-active)             = 90
  ├─ 有 ≥1 hard-safe SIDE 候选                   = 60 (67%)
  │    ├─ SIDE 被选中                            = 36 (60%)
  │    │     胜因: ORIGINAL_ORDER=8, C3_HIGHER=4, D3_LOWER=4, SIDE_PREFERENCE_HOLD=1
  │    └─ NOMINAL 被选中(尽管 side hard-safe)    = 24 (40%)
  │          ├─ 严格 C3/D3 落败(side 更差)       = 19  (其中 ≥10 例差距 ≤5e-3 ≈ 单样本分辨率)
  │          ├─ 全精度平局 → ORIGINAL_ORDER 链    = 4   (指标打印完全相同)
  │          └─ 无法解释(聚合 vs 胜者矛盾)        = 1   → UNRESOLVED_DUE_TO_MISSING_TELEMETRY
  └─ 无 side hard-safe(NOMINAL 唯一/构造失败)     = 30

事件层(21 闭合 + 2 未闭合):
  binary recovered = 9   (TTR 0.023–1.856 s,全部落在比较窗内)
  M2_PREEMPTED     = 4   (Team 让位序,非 selection)
  NO_SIDE_DISPATCH = 6   (side 从未进入构造/比较——上游,非 selection)
  ACTIVATION_NOT_REACHED = 1; PROGRESS_BUT_NOT_RECOVERED = 1; 无闭合 = 2
执行真值: K2 丢失总量 = 0.400 s(t≈73.6–74.2,UAV2+UAV3 瞬时遮挡自恢复,endgame,与 selection 无关)
```

## 4. 七个重点的逐项结论

**§4 K2 gate 是否在选错 topology —— 不是主因。** 全 run K2 gate 只在 1 个 (batch,kind) 上拦截过 side(且该行聚合自相矛盾:k2 高于 baseline 却 k2adm=0——drone 归属噪声,记 UNRESOLVED)。0 例"正确 side 因 K2 暂跌被淘汰"。CORRECT_SIDE_REJECTED_BY_K2_COUNT = **0 可证**(候选 1 例 UNRESOLVED)。

**§5 C3/D3 时间窗 —— 结构性缺陷确认。** 比较窗 = `[max(ev_begin,activation), min(ev_end, baseline.start+baseline.duration)]`,即**被"当前已执行轨迹"的剩余视界(≈1–2.3 s)截断**;事件窗本身在检测时也以 baseline 视界为界。后果在本 run 的表现不是"错过恢复"(9 次 TTR 0.02–1.86 s 均在窗内),而是**窗口太短使 PLUS/MINUS/NOMINAL 在窗内尚未几何分化 → 指标大量相等/近相等**(24 例 NOMINAL 胜出中 19 例差距 ≤5e-3 ≈ 单个 0.1 s 样本格;4 例完全相等)。C3_D3_WINDOW_MISSES_RECOVERY_COUNT = **0 可证**;真正的问题是分辨率而不是错过。

**§6 selection 是否最后 commit 的同一 candidate —— 是。** FROZEN_ACTIVATION_MUTATED=0、moving-rehead-reject=0;freeze 契约全 run 零违例;910 次 activation handoff 的 DP 均值 0.0038(max 0.558 为 current-state-restart 预期)。STALE_QUALITY_REVISION_COUNT = 0 可证;SELECTED_PAYLOAD_CHANGED_AFTER_SCORING_COUNT = 0 可证。

**§7 visibility evaluator 是否区分左右 —— 源码一致,几何验证缺失。** K2/C3/D3、三机报告、binary trace 在 F126 后统一在 `LocalVisibilitySample` 原语(同 static LOS/dynamic LOS/FOV/range、同 world-time、同 yaw 推进),与 executed truth 同口径;候选之间不存在 anchor/时基差(peer 冻结契约 + activation 冻结)。但因无逐候选几何真值,VISIBILITY_EVALUATOR_RANK_REVERSAL_COUNT 只能给 **0 可证 / UNRESOLVED(几何)**。

**§8 指标缺少恢复速度 —— 确认,这是核心。** C3=全可见占比、D3=margin 缺口积分,都是**窗口内质量/亏空量**,不含首次恢复时间排序。当两侧在 ~1–2 s 窗内尚未分化(本 run 23/24 的 NOMINAL 胜出属于此类),指标要么相等(→tie 链),要么以单样本格噪声差分出胜负(≥10 例差距 ≤5e-3)。C3_D3_FAIL_TO_RANK_TTR_COUNT = **4 例完全平局 + ≥10 例噪声级差分**(这些 batch 中 side 的真实收益在窗外,无法进入比较)。

**§9 side preference/persistence —— 只在 tie 层,且只触发 1 次。** SIDE_PREFERENCE_HOLD=1 且那次是 side 胜出。WRONG_SIDE_SELECTED_BY_TIE_PREFERENCE_COUNT = **0**。blocker identity/target 旋转导致的"同 PLUS 语义漂移"本 run 无证据。

**§10 NOMINAL 是否占不合理优势 —— 是,在 tie 层,机制性存在但本 run 未造成可证伤害。** CORRECT_SIDE_LOST_TO_NOMINAL_COUNT = **0 可证**(9 次恢复事件中没有"保持 NOMINAL 导致恢复更慢"的可证案例;d0 ev9 保持 NOMINAL 12 cycles 后照常恢复);但平局链(13 层)在指标相等时天然偏向与 incumbent 近似的 NOMINAL——这就是"persistence 无显式状态机却存在"的实现位置,一旦未来窗口内的两侧指标再度相等且真实收益在窗外,它会成为错误保持点。

## 5. 逐 batch 第一分歧分类(A–J)

| 类别 | 计数 | 证据 |
|---|---|---|
| A CORRECT_CANDIDATE_NOT_ACTUALLY_ELIGIBLE | 0 可证 / 1 UNRESOLVED | K2 gate 仅 1 例拦截,聚合自相矛盾 |
| B K2_GATE_RANK_REVERSAL | 0 | — |
| C C3_RANK_REVERSAL | 0 | 2 例"疑似"实为 C3 字典序正确行为(d2 ev6 cy10:minus D3 更优但 C3 更差→NOMINAL 胜) |
| D D3_RANK_REVERSAL | 1 UNRESOLVED | d2 ev9 cy5:NOMINAL 以 D3_LOWER 获胜但聚合 D3 高于 minus——聚合遥测无法定位逐 candidate |
| E C3_D3_TIE_THEN_SIDE_PREFERENCE | 0 错误(1 次触发且为 side 胜) | — |
| F C3_D3_TIE_THEN_NOMINAL_COST | 4 确认(全平局) + ≥10 噪声级差分 | 全部经 ORIGINAL_ORDER 链 |
| G STALE_REVISION_METRICS | 0 | freeze 契约零违例 |
| H VISIBILITY_EVALUATOR_ERROR | 0 可证 / 几何 UNRESOLVED | 无逐候选几何真值 |
| I COMMIT_PAYLOAD_CHANGED | 0 | DP 均值 0.0038 |
| J OTHER_EXPLICIT | — | 非 selection 层的事件失败:NO_SIDE_DISPATCH=6、M2_PREEMPT=4、ACTIVATION=1 |

## 6. 六个最终问题的回答

1. **正确 topology candidate 是否真的进入了同一 comparator?** 是——60/90 batch 有 hard-safe side 且全部通过了与 NOMINAL 完全相同的 freeze→preflight→指标→比较链(同一 activation、同一 peer 冻结、同一 evaluator)。
2. **它第一次在哪一层输掉?** 不在 eligibility(K2 gate 全 run 仅 1 例)、不在 rank 反转(0 可证)、不在 stale payload(0);它输在 **C3/D3 在被 baseline 视界截断的 ~1–2 s 窗内无法区分两条 topology**(23/24 的 NOMINAL 胜出差距 ≤ 单样本格),随后在**全精度平局链**(target-side→visibility→geometry/cost)中被 NOMINAL 的"贴近 incumbent"性质压过。
3. **主要错误是哪个?** 主因:**时间窗绑定为 incumbent 视界 → 指标分辨率不足以表达 topology 差异**(§5);次因:平局链的 NOMINAL 结构性偏置 + C3 先于 D3 的字典序(2 例 D3 更优 side 被 C3 压制)。上游方面,事件失败的主要计数在 selection 之外(NO_SIDE_DISPATCH=6、M2_PREEMPT=4)。
4. **comparator 在优化什么?** 窗口内(≤~2 s)的"K2 地板 + 全可见占比 + margin 亏空积分",平局后是全程总质量/几何/cost——即"可见性总质量"。
5. **与"最快恢复 K3"是否同一目标?** 不是。comparator 中不存在首次 binary 恢复时间、恢复后保持时长或任何 TTR 语义;D3 是亏空积分,可以由"晚但深"与"早但浅"以近似值相等。
6. **7–8 s K3 platform 能否由 selection rank reversal 解释?** 本 run 不存在该 platform(K2 丢失总量 0.400 s,9 次恢复 TTR ≤1.86 s),因此**当前数据不能把历史 platform 归因于 rank reversal**;若 platform 复现,本审计指出的"窗口截断 + 平局链 NOMINAL 保持"是其最短的机制路径——但这是推断,不是本 run 的观测。

## 7. 修改方向(仅建议,不实施;与 §15 的证据约束一致)

1. **窗口绑定事件,而非 incumbent(修 §5 主因,不加固定秒数)**:把 C3/D3/K2 比较窗的右端绑到 `k3_event_.predicted_binary_recover_time`(blocker-crossing 预测)与 candidate 自身时长的组合上,允许候选展示"incumbent 结束之后"的恢复段;指标 validity 已支持窗口 ⊆ 候选区间,只需放宽与 baseline 的交集约定并保持三机同窗。
2. **平局层引入显式恢复次序,不加权重(修 tie 链)**:仅在 C3/D3 全精度平局(现有 1e-9 判定)时,以"预测首次 binary 恢复时间早者胜"作为 tie-break,其后才进入 target-side/visibility/geometry;不新增 gate、不新增状态机。
3. **C3/D3 的分辨率护栏(修噪声差分)**:当 |C3 差| 与 |D3 差| 均小于 1 个样本格(1/sample_count 或 dt×margin)时视为平局进入上述 tie-break,避免以单样本噪声在 topology 间判决;以及遥测补齐:在 commit/activation 日志输出 payload_hash 与逐候选预测恢复时间,闭合本审计的 oracle 缺口。

---

## 8. 最终字段

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

CORRECT_TOPOLOGY_DEFINITION: blocker-target geometry + earliest binary-shadow escape;
  per-candidate geometric ground truth NOT reconstructable from run telemetry (no
  per-candidate coefficients logged) -> event-level executed-truth oracle used
COMPARATOR_CURRENT_ORDER: success -> handoff(frozen activation) -> stationary ->
  unified hard preflight -> certificate -> FREEZE(rev/hash) -> K3 window metrics
  [clipped to baseline horizon] -> K2 gate vs baseline -> K2 pair -> C3(desc, 1e-9)
  -> D3(asc, 1e-9) -> side-preference-hold -> target-side keep -> visibility -> geometry/cost
  -> (safety-required veto bypass)

TOTAL_COMPARABLE_BATCHES: 90
CORRECT_TOPOLOGY_CONSTRUCTED: 60 batches with >=1 hard-safe SIDE (per-kind geometric
  correctness UNRESOLVED_DUE_TO_MISSING_TELEMETRY)
CORRECT_TOPOLOGY_HARD_SAFE: 60
CORRECT_TOPOLOGY_K2_ELIGIBLE: 59 (1 blocked, self-contradictory aggregate)
CORRECT_TOPOLOGY_SELECTED: 36/60 (60%)

WRONG_TOPOLOGY_SELECTION_COUNT: 0 provable (24 NOMINAL wins: 19 strict-metric losses,
  4 exact ties, 1 UNRESOLVED)

CORRECT_SIDE_REJECTED_BY_K2_COUNT: 0
C3_RANK_REVERSAL_COUNT: 0
D3_RANK_REVERSAL_COUNT: 1 UNRESOLVED_DUE_TO_MISSING_TELEMETRY (d2 ev9 cy5)
C3_D3_FAIL_TO_RANK_TTR_COUNT: 4 exact ties (+ >=10 sub-resolution margins <=5e-3)
C3_D3_WINDOW_MISSES_RECOVERY_COUNT: 0 provable (all 9 TTRs 0.023-1.856s inside window);
  structural clip to baseline horizon confirmed in source
WRONG_SIDE_SELECTED_BY_TIE_PREFERENCE_COUNT: 0 (preference fired once, side won)
CORRECT_SIDE_LOST_TO_NOMINAL_COUNT: 0 provable (all 9 recoveries unaffected);
  structural NOMINAL tie-bias present in 4+ batches

STALE_QUALITY_REVISION_COUNT: 0 (FROZEN_ACTIVATION_MUTATED=0, rehead=0)
SELECTED_PAYLOAD_CHANGED_AFTER_SCORING_COUNT: 0 (910 handoffs, DP mean 0.0038)
VISIBILITY_EVALUATOR_RANK_REVERSAL_COUNT: 0 provable / geometric UNRESOLVED

PRODUCTION_ORACLE_DISAGREEMENT: not computable per-candidate (telemetry gap);
  event-level: 9/9 recoveries consistent with committed kind timeline
PLUS_MINUS_RANK_REVERSAL: 0 provable
SIDE_NOMINAL_RANK_REVERSAL: 0 provable (1 UNRESOLVED)

FIRST_SELECTION_DIVERGENCE: myopic comparison window (clipped to incumbent horizon)
  -> C3/D3 cannot separate topologies -> ties/noise margins
PRIMARY_SELECTION_ROOT_CAUSE: window resolution (event unbound to blocker-crossing)
SECONDARY_SELECTION_ROOT_CAUSE: ORIGINAL_ORDER tie-chain NOMINAL bias + C3-before-D3
  lexicographic suppression (2 cases of better-D3 sides kept unselected)

IS_COMPARATOR_OPTIMIZING_FAST_K3_RECOVERY: NO (no TTR term anywhere)
IS_COMPARATOR_OPTIMIZING_SOMETHING_ELSE: YES — in-window visibility mass
  (K2 floor + C3 all-visible fraction + D3 margin deficit), then total quality/cost
CAN_SELECTION_EXPLAIN_K3_7_8S_PLATFORM: NO for current run (K2 loss total 0.400s,
  9 recoveries TTR<=1.86s); historical platform predates current comparator and
  cannot be attributed from present telemetry; window+tie mechanisms are the
  plausible recurrence path (inference, not observation)

RECOMMENDED_CHANGE_1: bind comparison window right edge to the event's predicted
  binary-recovery / blocker-crossing time combined with candidate duration (not
  incumbent end); keep the 3-UAV common-window contract
RECOMMENDED_CHANGE_2: at exact C3/D3 tie (existing 1e-9), add explicit
  first-binary-recovery-time tie-break before target-side/visibility/geometry
RECOMMENDED_CHANGE_3: treat metric differences below one sample cell as ties
  (resolution guard); log per-candidate predicted recovery time + payload hash at
  commit/activation to close the oracle gap

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
