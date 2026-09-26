# feedback_128 — 「正确 topology 已生成，但 selection 选错」专项审计（只读）

- Run: `runs/20260926_225042_830329`（FULL，BOOT-12，exit 0，78.2 s，唯一被逐事件复盘的 run）
- 方法：comparator 源码逐行审计 + 日志逐事件/逐 batch/逐 candidate 复盘。无编译、无仿真、无参数改动、未访问 /home/bob/RRCT。
- 结论先行：**没有实锤「正确 topology 已生成但 selector 选错」。** comparator 从未违背它自己的窗口指标（0 例）。真正的问题是：(a) 比较/事件窗口被基线 horizon **结构性截断**（源码实锤），C3/D3 在 topology 分化出现之前就停止观察；(b) 31% 的 batch 根本没有 hard-safe SIDE（生成问题，不是选择问题）；(c) 指标精确平局时落入生成顺序/旧 comparator（7 例，无一生成可证明选错）。

---

## 1. Comparator 源码真实顺序（审计基准）

`plan_manage/src/planner_manager.cpp finalizeCapturedCandidates()`（L1482-1975），对每个 batch：

| 级 | 层 | 源码位置 | 语义 |
|---|---|---|---|
| 0 | hard preflight | L1653-1658 | validateRetimedLocalSfc + checkActiveHandoff + validateExecutionTrajectory；进入比较的候选全部 hard-safe |
| 0b | certificate | L1660-1668 | hardCheckedUntil 有限且 > activation |
| 1 | K2 gate | L1769-1778 | 候选窗口 K2 ≥ 基线(已执行轨迹)窗口 K2 − 1e-9，否则该候选出局（`K2_GATE_REJECT_CANDIDATE`/`K3_WINDOW_INVALID`）|
| 2 | C3 | L1803-1810 | \|ΔC3\| > 1e-9 严格更高者胜 |
| 3 | D3 | L1811-1819 | \|ΔD3\| > 1e-9 严格更低者胜 |
| 4 | SIDE preference | L1820-1841 | C3/D3 全平时保留上次为该 event 提交过的 side |
| 5 | target_side_static_topology | L1850-1859 | 布尔保持（本 run 从未触发）|
| 6 | 旧 visibility comparator | L1861-1868 | betterLocalVisibilityCandidate 十元组 |
| 7 | geometry/cost | L1866-1871 | geometryBetter；K3 激活时获胜记 `ORIGINAL_ORDER` |

关键细节：`ORIGINAL_ORDER` 有两种含义——(i) 真的由第 5-7 层决出；(ii) **第一个可准入候选直接成为 best、无人挑战成功**（k3_selected_reason 为空时日志打印 ORIGINAL_ORDER，L1929-1931）。batch 内生成顺序 NOMINAL → SIDE_PLUS → SIDE_MINUS（L8710、L13278、L13284），因此精确平局天然偏向 NOMINAL。

## 2. 窗口语义（§6A 的结构基础）

- 采样 dt = `manager/visibility_sample_dt` 默认 **0.10 s**（L643、L664）。
- K2 = 窗口内三机二值可见 ≥2 的样本占比；C3 = 三机全可见占比；D3 = 自身 margin 欠量 `max(0, required_margin − margin)` 的时间积分（`evaluateLocalK3WindowMetrics` L7676-7826）。
- **窗口有效性要求整个窗口落在被评轨迹 span 内**（L7693-7695）：窗口超出候选末端 → 该候选整只无效。
- **比较窗终点 = min(事件窗终点, 基线轨迹末端)**（L1516-1523）。
- **事件窗终点本身锚在基线上**：`scan_end = baseline.start + min(baseline.duration, prediction_horizon)`（L3579-3585）；基线在自己 horizon 内不自预测恢复时，`window_end = scan_end + dt`（L3767-3769）。刷新只延不缩（L3770-3783）。

即：**comparator 的可见时域被 incumbent 的剩余 horizon 封顶；候选在基线末端之后的行为，对选择器不存在。**

## 3. 逐事件时间线（22 个事件 = d0×9 + d1×6 + d2×7）

时间统一为 run 内秒（wall − 1790434250.99 ≈ CSV time_s）。`Δrecov` = 执行二值恢复时刻 − 初始事件窗终点。

| 事件 | limiter | 初始窗 (s) | 窗长 | 执行 loss@ | 执行 recov@ | Δrecov | 执行轨迹 | blocker | SIDE请求/实际侧 | 结果 |
|---|---|---|---|---|---|---|---|---|---|---|
| d0 E1 | STATIC | [7.91, 8.26] | 0.35 | 7.91 | 8.81 | **+0.55** | SIDE_MINUS | 36 | MIN/MIN ✓ | MARGIN_RECOVERED |
| d0 E2 | STATIC | [7.71, 8.81] | 1.10 | 7.91 | 8.81 | +0.00 | NOMINAL | 26 | MIN/MIN (NOMINAL 飞行) | RECOVERED ttr=1.005 |
| d0 E3 | STATIC | [10.41, 10.76] | 0.35 | 10.68 | 11.95 | **+1.19** | NOMINAL | 26 | MIN/PLUS (NOMINAL) | RECOVERED ttr=1.480 |
| d0 E4 | STATIC | [24.41, 24.90] | 0.49 | 25.22 | 26.51 | **+1.61** | NOMINAL | 28 | PLUS/MINUS (NOMINAL) | M2_PREEMPTED |
| d0 E5 | STATIC | [26.01, 26.61] | 0.60 | 25.81* | 26.51 | — | NOMINAL | 28 | PLUS | RECOVERED ttr=0.434 |
| d0 E6 | STATIC | [26.51, 28.40] | 1.89 | — | — | — | NOMINAL | — | — | RECOVERED ttr=0.005 |
| d0 E7 | DYN_LOS | [32.01, 32.62] | 0.61 | — | — | — | NOMINAL | — | — | NO_SIDE_DISPATCH |
| d0 E8 | FOV | [53.01, 53.34] | 0.33 | — | — | — | NOMINAL | — | — | NO_SIDE_DISPATCH |
| d0 E9 | STATIC | [63.51, 63.61] | 0.10 | 63.95 | 65.05 | **+1.43** | NOMINAL | 13 | PLUS/MINUS (NOMINAL) | RECOVERED ttr=1.449 |
| d1 E1 | STATIC | [8.41, 8.56] | 0.15 | — | — | — | NOMINAL | 36 | MIN | MARGIN_RECOVERED |
| d1 E2 | STATIC | [11.61, 11.88] | 0.27 | — | — | — | NOMINAL | — | — | MARGIN_RECOVERED |
| d1 E3 | STATIC | [26.21, 26.53] | 0.32 | — | — | — | NOMINAL | — | — | M2_PREEMPTED |
| d1 E4 | DYN_LOS | [73.71, 74.14] | 0.43 | 73.48 | 74.05 | −0.09 | NOMINAL | 10(动) | PLUS/MINUS(动障碍,不可靠) | RECOVERED ttr=0.279 |
| d1 E5 | STATIC | [75.11, 76.66] | 1.55 | — | — | — | NOMINAL | — | — | RECOVERED ttr=0.023 |
| d1 E6 | RANGE | [79.31, 80.01] | 0.70 | — | — | — | NOMINAL | — | — | NO_SIDE_DISPATCH |
| d2 E1 | STATIC | [8.31, 8.52] | 0.21 | — | — | — | NOMINAL | 36 | MIN | M2_PREEMPTED |
| d2 E2 | STATIC | [11.61, 11.87] | 0.26 | — | — | — | NOMINAL | — | — | MARGIN_RECOVERED |
| d2 E3 | STATIC | [26.21, 26.49] | 0.28 | — | — | — | NOMINAL | — | — | M2_PREEMPTED |
| d2 E4 | STATIC | [35.01, 35.25] | 0.24 | 35.51 | 36.48 | **+1.23** | NOMINAL | 6 | PLUS/PLUS (NOMINAL) | RECOVERED ttr=1.371 |
| d2 E5 | DYN_LOS | [70.21, 70.37] | 0.16 | 70.68 | 72.48 | **+2.11** | SIDE 谱系 FALLBACK | 14(动) | 不可靠(动障碍) | PROGRESS_BUT_NOT_RECOVERED |
| d2 E6 | STATIC→刷新 | [70.51, 70.92]→[70.0,73.58] | 0.41→3.58 | 70.68 | 72.48 | **+1.56**(初始窗) | SIDE 谱系 FALLBACK | 10→14(动) | 不可靠(动障碍) | RECOVERED ttr=1.856 |
| d2 E7 | STATIC | [73.51, 74.41] | 0.90 | 73.65 | 74.38 | −0.03 | NOMINAL | 37 | PLUS/PLUS (NOMINAL) | RECOVERED ttr=0.805 |

\* d0 E5 的执行损失早于事件窗（事件由 margin 预测触发，二值损失未在其窗内发生）。

要点：
- **10 个有完整损失/恢复片段的事件中，7 个的真实二值恢复落在初始事件窗终点之后 0.55–2.11 s**（d0 E1/E3/E4/E9、d2 E4/E5/E6）；只有 3 个落在窗内（d0 E2、d1 E4、d2 E7）。
- 窗口刷新（只延不缩）确实最终覆盖了恢复（如 d2 E6 从 0.41 s 刷到 3.58 s），但那发生在前若干 cycle 已经在盲窗里做完选择之后。
- SIDE 谱系真正被执行的片段只有 3 个（d0 E1、d2 E5/E6）。静态 blocker 可验证的 1 例（d0 E1）requested=actual=MINUS，**side 契约一致**；d2 E5/E6 为移动 blocker，侧别不可可靠验证（标注 UNRESOLVED，移动障碍相位模型未验证）。
- NOMINAL 执行的片段有 3 例绕到了 SIDE 请求的**对侧**（d0 E3/E4/E9）——NOMINAL 无 side 契约，不是违约，但它说明「选 NOMINAL」与「选 SIDE」在实际几何上确实是两条不同的 homotopy。

## 4. 候选身份恢复（§2）

- `FROZEN_CANDIDATE` 全 run 3192 条可解析（2492 NOMINAL / 402 SIDE_PLUS / 298 SIDE_MINUS），含 candidate_id、revision、payload_hash、activation、checked_until、active_end。同一 batch 全体候选共享同一 activation（冻结契约成立，未发现 FROZEN_ACTIVATION_MUTATED）。
- selected/committed/activated 链完整：K3_SIDE_SOFT_LOS_COMMIT → trajectory_id → active-traj-lifecycle → visibility_trajectory.csv 的 trajectory_source/trajectory_id。
- **未选中候选的多项式坐标在 run 中不存在**（无 bag、无 candidate 导出）。因此一切「未选中候选的实际拓扑/更早恢复」判断均为：
  **UNRESOLVED_DUE_TO_MISSING_CANDIDATE_TRAJECTORY**。
  不用 kind 标签假装几何真值；唯一可用的替代证据是 production 自己在该候选多项式上采样出的 K2/C3/D3（有日志），它表达「窗口内二值可见性恢复」，不表达几何侧别。

## 5. 是否存在「hard-safe 且更优的候选没被选」（§4/§5）

135 个 K3 batch（4 行因多进程 stdout 交错残缺，2 行可部分解析）。在「同一 batch 存在 ≥2 个 hard-safe 候选」的 61 个中：

- **严格违反 comparator 自身指标序的选错：0 例。**（ 曾在中途报告过 3 例"指标矛盾"，经查是我方解析脚本的 offsetX bug——正则捕获组错位导致字段错读；修正后归零。以此为准。）
- SIDE 严格更优（C3 更高或 C3 平且 D3 更低）的 30 个 batch，**全部**由 C3_HIGHER/D3_LOWER 选出 SIDE。
- SIDE 严格更差的 10 个 batch，全部正确保留 NOMINAL。
- **精确平局 9 个 batch**（C3、D3 在容差下全等）：生产指标无法区分，落入第 4-7 层：

| t | 事件 | 决策 | selected | NOMINAL (C3,D3) | SIDE (C3,D3) | FIRST_SELECTION_DIVERGENCE |
|---|---|---|---|---|---|---|
| 257.591 | E1c1 | ORIGINAL_ORDER | NOMINAL | (0.000, 0.273) | MINUS (0.000, 0.273) | **OLD_VISIBILITY_ORDER** |
| 257.635 | E1c1 | ORIGINAL_ORDER | SIDE_MINUS | (0.000, 0.142) | PLUS (0.000, 0.142) | OLD_VISIBILITY_ORDER |
| 257.888 | E1c2 | ORIGINAL_ORDER | NOMINAL | (0.000, 1.001) | MINUS (0.000, 1.001) | **OLD_VISIBILITY_ORDER** |
| 259.554 | E2c5 | D3_LOWER | SIDE_MINUS | (0.667, 0.062) | MINUS (0.667, 0.062) | D3（打印精度以下的真实差）|
| 287.245 | E4c12 | D3_LOWER | SIDE_PLUS | (1.000, 0.059) | PLUS (1.000, 0.059) | D3（同上）|
| 302.256 | E8c1 | ORIGINAL_ORDER | SIDE_MINUS | (1.000, 0.000) | PLUS (1.000, 0.000) | OLD_VISIBILITY_ORDER |
| 312.791 | E9c1 | ORIGINAL_ORDER | NOMINAL | (1.000, 0.000) | PLUS (1.000, 0.000) | **OLD_VISIBILITY_ORDER** |
| 315.746 | E9c12 | ORIGINAL_ORDER | NOMINAL | (0.667, 0.153) | MINUS (0.667, 0.153) | **OLD_VISIBILITY_ORDER** |
| 323.340 | E6c17 | SIDE_PREFERENCE_HOLD | SIDE_PLUS | (1.000, 0.000) | PLUS (1.000, 0.000) | SIDE_PREFERENCE（按设计保侧）|

9 例中 SIDE 胜 5、NOMINAL 胜 4。NOMINAL 胜的 4 例是唯一的「hard-safe SIDE 存在、指标不吃亏、却没被选」——但没有任何证据表明该 SIDE 实际更优（几何 UNRESOLVED），因此它们是**不可判定**，不是**实锤选错**。

## 6. 两个可疑结构的核查（§6）

**A. 短窗口问题 —— 成立（结构性）**
- 源码：比较窗与事件窗均被基线 horizon 封顶（§2）。
- 数据：7/10 个有完整损失/恢复片段的事件，真实恢复发生在初始窗终点之后（0.55–2.11 s）；候选 active_end（activation+2.5–4 s）普遍超出窗终点 1–3 s，超出部分不参与比较。
- `WINDOW_TRUNCATED_BEFORE_TOPOLOGY_DIFFERENCE = YES`。
- `WINDOW_CAUSED_AMBIGUOUS_SELECTION_COUNT = 7`（9 个平局中的 7 个真平局；「由截断导致」是结构推断，非逐例实证——逐例实证需要未选中候选轨迹，UNRESOLVED）。

**B. 旧排序接管问题 —— 存在但仅限平局**
- `TARGET_SIDE_LABEL_OVERRULE_COUNT = 0`（`[target-side-topology-selection]` 全 run 未出现一次）。
- `OLD_VISIBILITY_OVERRULE_COUNT = 7`（9 个精确平局全部由第 4-7 层决出；其中 4 次对 SIDE 不利、3 次有利）。
- `NOMINAL_TIE_BIAS_COUNT = 4`（平局偏向先生成的 NOMINAL）。
- K2 gate：**从未拒绝任何 hard-safe SIDE**（61 个有 side 的 batch 全部 admissible；`K3_K2_GATE_UNAVAILABLE` 0 次）。K2_GATE_WRONG_SELECTION = 0。

## 7. C3/D3 能否识别正确 topology（§7）

61 个含 side 的 batch 中：严格由 C3/D3 决出 32（全部与指标自洽）、真平局 7、亚分辨率 D3 差 2（打印 3 位小数不可见、1e-9 容差可见的真实 D3 差，均判给了 SIDE）、rank reversal 0。

- C3_D3_DISTINGUISH_CORRECTLY = 32
- C3_D3_TIE = 7
- C3_D3_SUBRESOLUTION = 2
- C3_D3_RANK_REVERSED = 0

对 §7 四选一的回答：**是组合，主因是 1（窗口太短），次因是 3（平局时旧排序接管）**。C3/D3 在窗内忠实表达二值恢复结果（无 rank reversal），但窗口在 topology 分化出现前就结束了——指标本身没排名排反，是**没看到**。第 2 条（指标不表达 topology outcome）部分成立：C3/D3 只表达「窗口内二值可见占比/欠量」，完全不表达侧别与 homotopy；第 4 条（candidate 实际没更好）不可判定。

## 8. 「SIDE 存在」≠「正确 SIDE 存在」（§8）

| 层 | 数值 |
|---|---|
| TOTAL_K3_BATCHES | 135 |
| HARD_SAFE_SIDE_EXISTS | 61（45%）|
| ACTUAL_TOPOLOGY_RESOLVABLE | 候选级 0（无坐标）；执行级 11/22 事件（10 完整 + 1 部分）|
| CORRECT_TOPOLOGY_EXISTS（可证明）| 1（d0 E1：SIDE_MINUS 执行，requested=actual=MINUS，恢复被 latched）；其余 UNPROVEN/UNRESOLVED |
| CORRECT_TOPOLOGY_SELECTED（可证明）| 1（+ d2 E5/E6 SIDE 谱系已执行但动障碍侧别不可验证）|
| CORRECT_TOPOLOGY_NOT_SELECTED（可证明）| **0** |

之前「60/90 batch 有 hard-safe SIDE」若被读成「60 个正确 topology」，就是本轮要纠正的混淆。

## 9. 真实 selection funnel（§9）

```
TOTAL_K3_BATCHES                = 135
HARD_SAFE_SIDE_EXISTS           = 61
  ├─ SIDE 严格更优 → 被选       = 30   (C3_HIGHER/D3_LOWER, 全部自洽)
  ├─ 精确平局                   = 9    (SIDE 胜 5 / NOMINAL 胜 4)
  ├─ SIDE 严格更差 → 正确拒绝    = 10
  └─ 归属残缺(日志交错/跨机ID碰撞)= 12
NO_HARD_SAFE_SIDE(构造/生成失败) = 42   (全部 selected=NONE, 保留 incumbent)
NOMINAL 等无侧对照 batch         = 32
SIDE_SELECTED 总计               = 38
NOMINAL_DESPITE_SIDE             = 13   (10 指标更差=正确, 3 平局, 0 实锤选错)
K2_GATE 拒绝 hard-safe SIDE      = 0
执行恢复落在初始窗外的事件       = 7/10
```

未选原因分解（对 61 个含 side batch）：

```
WINDOW_TRUNCATION（结构因果,非逐例实证） = 7 (平局案例与截断一致)
K2_GATE            = 0
C3                 = 0
D3                 = 0
SIDE_PREFERENCE    = 0 (唯一一次使用,判给 SIDE)
TARGET_SIDE_LABEL  = 0
OLD_VISIBILITY     = 4 (平局+生成顺序,对 SIDE 不利;不可证明为错)
GEOMETRY_COST      = 0 (无法与 OLD_VISIBILITY 区分,计入上条)
STALE_METRICS      = 0
UNRESOLVED         = 12
```

## 10. 判定（§10）

1. **是否实锤「正确 topology 已生成，但 selector 选错」？——否。** 0 个实锤案例。comparator 在全部 135 个 batch 上与自己的窗口指标完全一致；9 个平局案例因缺失候选轨迹不可判定（UNRESOLVED_DUE_TO_MISSING_CANDIDATE_TRAJECTORY）。
2. **第一主因：** 不是 selector 排序错误。是**比较时域被 incumbent horizon 结构性截断**（源码实锤 + 7/10 执行恢复在窗外），使 C3/D3 恰好在 topology 分化出现前失明，平局落入无拓扑信息的生成顺序/旧 comparator。**更大的单一损失在生成侧**：42/135（31%）batch 根本没有 hard-safe SIDE 可选——这是生成/构造问题，selector 无从选起。
3. **最小修改位置：** 若目标是让选择看到拓扑分化——只改**比较时域**（比较窗/事件窗与基线末端解耦，例如以本 batch 候选的最大 active_end 为窗下界），selector 各层顺序与 C3/D3 公式一律不动。现有证据**不足以**支持：SIDE 强制优先、新 TTR cost、新权重、新 FSM、新 hysteresis、删除旧 visibility authority（它只在平局出场，且 4/7 次判给了 SIDE）。

## 最终字段

```
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

TOTAL_K3_BATCHES: 135
HARD_SAFE_SIDE_EXISTS: 61
ACTUAL_TOPOLOGY_RESOLVABLE: 0 (candidate-level; executed-episode-level: 11/22 events)
CORRECT_TOPOLOGY_EXISTS: 1 proven (61 undecidable: UNRESOLVED_DUE_TO_MISSING_CANDIDATE_TRAJECTORY)
CORRECT_TOPOLOGY_SELECTED: 1 proven (+2 SIDE-lineage flown, moving-blocker side unverifiable)
CORRECT_TOPOLOGY_NOT_SELECTED: 0 proven

WINDOW_TRUNCATED_BEFORE_TOPOLOGY_DIFFERENCE: YES
WINDOW_CAUSED_AMBIGUOUS_SELECTION_COUNT: 7

K2_GATE_WRONG_SELECTION: 0
C3_WRONG_SELECTION: 0
D3_WRONG_SELECTION: 0
SIDE_PREFERENCE_WRONG_SELECTION: 0
TARGET_SIDE_LABEL_WRONG_SELECTION: 0
OLD_VISIBILITY_WRONG_SELECTION: 4 (metric-exact-tie, order-decided; not provably wrong)
GEOMETRY_COST_WRONG_SELECTION: 0
STALE_METRICS_WRONG_SELECTION: 0
UNRESOLVED_SELECTION_COUNT: 12

C3_D3_DISTINGUISH_CORRECTLY: 32
C3_D3_TIE: 7
C3_D3_SUBRESOLUTION: 2
C3_D3_RANK_REVERSED: 0

PROVEN_CORRECT_TOPOLOGY_REJECTED: 0
PRIMARY_SELECTION_ROOT_CAUSE: comparison/event window structurally capped by incumbent horizon; C3/D3 blind before topology divergence materializes (7/10 executed recoveries beyond initial window end)
SECONDARY_SELECTION_ROOT_CAUSE: generation-side supply — 42/135 batches had no hard-safe SIDE at all (not a selection problem)

SHOULD_CHANGE_SELECTOR: NO
MINIMAL_CHANGE_LOCATION: comparison-time-domain only (decouple k3_comparison_end / event window_end from baseline trajectory end)
SHOULD_CHANGE_WINDOW: YES
SHOULD_CHANGE_C3_D3: NO
SHOULD_REMOVE_OLD_ORDER_AUTHORITY: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

## 附：审计可信度说明

- 135 行 batch 遥测中 4 行因 roslaunch 多进程 stdout 交错残缺（2 行部分可解析）；FROZEN_CANDIDATE 3210→3192 条可解析。所有跨 drone 的 candidate_id/trajectory_id 关联均以 (activation, wall) 精确匹配，不用模糊时间窗。
- 本审计中途一次「3 例指标矛盾」为我方解析器正则捕获组错位所致，修正后为 0；最终结论以修正后数据为准。
- 移动 blocker（id 10/14）的 actual-side 判定依赖其相位模型与观察面法向刷新，未验证，相关条目标注 UNRESOLVED；静态 blocker（id 6/13/26/28/36/37）位置取自 scene 文件，为精确值。
- 执行真值来源：visibility.csv（生产 tracker 的逐样本 range∧static∧dynamic∧fov 二值）与 K3_BINARY_RECOVERY_OBSERVED（EXECUTED_ODOMETRY latch），两者一致（校验事件 d0 E2：latch 与 CSV 恢复时刻差 < 1 个采样格）。
