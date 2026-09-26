# feedback_130 — SIDE → NOMINAL rolling transition 专项只读审计

- 工作区：`/home/bob/ALP/egov2_fc65423_constvel`（dirty worktree 唯一权威）
- Run：`runs/20260926_225042_830329`；只消费该 run + scene + 已有 CSV/log + feedback_128/129。
- 方法：先闭合 EP1 单链（逐 cycle 证据），再批量验证 EP2-7 的全部 source/topology transition。未编译、未仿真、未改代码。feedback_129 的“selector 翻回”解释本轮视为 HYPOTHESIS 重新检验。

---

## 0. 一句话结论

**12 次 SIDE→NOMINAL 型翻转中，8 次的根本机制是 `SIDE_REGEN_HARD_SAFETY_FAILED`：SIDE 每一轮都重新 dispatch、重新构造（MINCO success=1），却在最终 hard 预检被 `DYNAMICS_FAIL` 击杀，NOMINAL（常靠 feasible-initializer 兜底）作为唯一 hard-safe 幸存者被提交；只有 4 次是 comparator 真实击败（D3/C3 边际差 0.002–0.05）。`trajectory_source=NOMINAL` 从不意味着几何跳回 nominal homotopy——s(t) 跨标签连续，状态继承、拓扑意图丢弃。**

---

## 1. EP1 完整因果链（loss 7.91–8.81 s，uav1/drone0，blocker 36，STATIC_LOS）

drone0 在 EP1 窗口的 planning cycle 表（wall−1790434250.99=sim；每个 cycle 都有 `[threat-dispatch] reason=LOS los=1` 与 `side_attempts=2`）：

| traj | commit (sim) | activation | source | 所在 cycle 证据 |
|---|---|---|---|---|
| 18/19 | 5.24/5.78 | — | NOMINAL | `risk-candidate triggered=0`（LOS 威胁未触发期） |
| 20 | 6.06 | 6.16 | SIDE_MINUS | LOS witness 1.8s；E1 创建前一 cycle |
| 21 | 6.35 | 6.45 | SIDE_PLUS | E1c1，D3_LOWER（0.528<0.565/0.568） |
| 22 | 6.63 | 6.73 | SIDE_PLUS | E1c2（candidate27） |
| 23 | 6.90 | 6.99 | SIDE_PLUS | E1c3（candidate30） |
| **24** | **7.17** | **7.27** | **NOMINAL** | **T1**：E1c4 全三方 success=1；N C3=0.100 D3=2.477 vs P 2.527 / M 2.494 → NOMINAL D3 最低真实获胜（ORIGINAL_ORDER＝NOMINAL 首位+无挑战成功） |
| 25 | 7.46 | 7.59 | FEASIBLE_FALLBACK（handoff 标 SIDE_MINUS） | M2 合同期提交；candidate=initializer 兜底 |
| 26 | 7.60 | 7.70 | SIDE_MINUS | E2c1（candidate40）SIDE 胜出 |
| **27** | **7.73** | **7.83** | **NOMINAL** | **T2**：E2c2 `PLUS={success=0} MINUS={success=0}`；cand42/43 `safe=0 reason=DYNAMICS_FAIL`（`[side-minco] success=1` 均已解算成功）→ NOMINAL 唯一 hard-safe 幸存者 |
| 28 | 8.00 | 8.25 | FEASIBLE_FALLBACK（=SIDE_PLUS initializer，candidate45） | E2c3：NOMINAL success=0，PLUS success=1 胜出——方向相反的翻转（NOMINAL 死、SIDE 活） |
| **29** | **8.28** | **8.38** | **NOMINAL** | **T4**：E2c4 cand48/49 `DYNAMICS_FAIL`，NOMINAL 唯一幸存 |
| 30 | 8.6x | 8.72 | SIDE_MINUS | E2 后段 SIDE 重新构造成功并胜出（恢复侧） |

**逐 transition 归因：**

- **T1＝F TRUE_SELECTOR_REVERSION**。同 batch、同 blocker、三方 hard-safe、全部进入 comparator；NOMINAL 以 D3 2.477 vs 2.527/2.494（差 0.017/0.05，C3 全平 0.100）获胜。comparator 忠实（f128 已证语义），但注意：三方 C3=0.100 意味着在 E1 窗口 [258.90,259.25] 内三方同样盲——没有任何候选（含两侧）能证明恢复，NOMINAL 只是“最不坏”。二值损失 7.91 紧随其后如期发生（predicted_binary_loss=258.999）。
- **T2＝E SIDE_REGEN_HARD_SAFETY_FAILED**。E2c1 刚 commit SIDE_MINUS（traj26），下一 cycle E2c2 两侧 MINCO 仍 `success=1`，但终检 `safe=0 reason=DYNAMICS_FAIL`——SIDE 无法保持 hard-safe，NOMINAL（candidate41 safe=1）作为唯一幸存者提交。**这正是 §16 要求找的具体布尔条件：不是 selector 选了 NOMINAL，是 SIDE 歚在了预检。**
- **T4＝E**（同 T2：cand48/49 DYNAMICS_FAIL）。
- T3（8.00-8.25 的 NOMINAL→FALLBACK）方向相反：该轮 NOMINAL `success=0`、SIDE_PLUS 胜出——证明翻转是双向的存活竞争，不是单向“NOMINAL 抢权”。

**K3 事件生命周期（EP1）**：d0E1（创建 6.34，窗 [7.91,8.26]）在 T1 时仍 active；~7.44 被 **M2_PREEMPTED**（Team M2 合同，两行 M2 结果 258.436/258.441 之一；另外两次 E1 结果 258.135 MARGIN_RECOVERED 属 d1/d2 的同名事件——它们的 SIDE 预测 dodge 清了 margin）。M2 结束后 d0E2 创建（7.60，窗 [7.71,8.81]）。E2 期间 authority 连续（grant×3），事件未在真实遮挡中提前关闭。**B/I 类在 EP1 = 0；A（authority dropout）= 0**（`current_raw_los_blocked_` 每 cycle 重算且持续命中 blocker36，`[current-los-blocked]` ×2；witness world time 始终落在事件窗 ±0.5s 内）。

## 2. NOMINAL 如何被种子（§9，SOURCE PROVEN）

`planner_manager.cpp L8490-8551`：
- `nominal_warm_cache = nominal_baseline && accepted_state_cache_kind_ == CandidateKind::NOMINAL`——**NOMINAL 只允许从上一条 NOMINAL cache warm-start**（注释原文："Never reuse an alternative (SIDE/team/recovery) cache for hypothesis zero"）。
- SIDE 之后的新 NOMINAL 走 `buildStaticFeasibleInitializer`（fresh direct/A\* 种子）。
- 但种子起点 = 当前执行状态 start_pt/start_vel/start_acc（handoff DP=DV=DA=0 强制连续）。

**结论：新 NOMINAL 继承 SIDE 已产生的位置/速度（状态继承），但不继承其 topology 意图（bulge/观察面）。** 这直接否定了“NOMINAL＝几何跳回原 homotopy”的默认解读，也否定了“NOMINAL 从原始 tracking reference 重新出发”的猜测。

## 3. EP1 几何检验：N1 还是 N2（§8）

blocker36-target frame 下 uav1 的 s(t)（+ = PLUS 侧）与真实 LOS36 净空，按执行段：

```text
t=6.92 SIDE_PLUS  s=+1.22  los=+0.737
t=7.28 SIDE_PLUS  s=+1.19  los=+0.519     ← T1 前
t=7.42 NOMINAL    s=+1.15  los=+0.422     ← traj24（T1 后）：无回跳
t=7.88 NOMINAL    s=+0.84  los=+0.084     ← traj27（T2 后）：无回跳
t=8.12 NOMINAL    s=+0.51  los=−0.097     ← 进入阴影
t=8.36 FALLBACK   s=−0.01  los=−0.298     ← 穿轴，最深遮挡
t=8.48 NOMINAL    s=−0.36  los=−0.175     ← traj29（T4 后）：继续同向
t=8.68 NOMINAL    s=−0.87  los=−0.007     ← 即将穿出
t=8.82 SIDE_MINUS s=−1.20  los=+0.109     ← 恢复
```

- **N2 成立**：s(t) 是跨全部 source 标签的单值连续曲线（+1.22→−1.49），每次 label 切换处无一像素级跳变——`NOMINAL` 绝不是几何回归。
- **N1 部分成立**：NOMINAL 不提供 SIDE 的逃逸横向加速度，只是让载具以既有速度继续漂移穿轴（8.36 处最深 −0.298）。损失 0.90s vs 场景 fixed-trace 设计窗 0.47s（≈1.9 倍，INFERENCE）——拖长来自“意图丢弃后的惯性穿行”，不是“几何回退”。
- **判定：`trajectory_source=NOMINAL` = 状态延续 + 拓扑意图丢弃（G 类不成立为根因，但作为解释性发现成立）。**

## 4. FALLBACK 拆解（§13）

`FEASIBLE_FALLBACK` ＝ `candidate.feasible_initializer_fallback==true`（L6528）：优化器失败、但 feasible initializer（NOMINAL→A\* 直连种子；SIDE→正弦 bulge 种子）被作为 payload 提交。逐例：
- EP1 traj25：handoff 标 SIDE_MINUS、执行标 FALLBACK → **SIDE_MINUS 的 initializer payload**（M2 合同期）。
- EP1 traj28：candidate45＝SIDE_PLUS 胜出 + initializer payload → **SIDE initializer**。
- EP2 的 FALLBACK（cand68/71 标 NOMINAL）→ **NOMINAL 的 A\* initializer payload**。
即 FALLBACK 不等于 SIDE 也不等于 NOMINAL——它是“该 kind 的种子几何”，必须逐例看 candidate kind。

## 5. current_raw_los_blocked_ 生命周期（§10，SOURCE PROVEN + RUN）

- 每个批次开头无条件 reset 并重算（L9015-9082）：static 用当前 odom（start_pt）优先，dynamic 兜底；命中即钉 recovery 窗 [0,horizon] 并打 `[current-los-blocked]`。
- EP1 全程 drone0 持续命中（blocker36 ×2 节流日志；threat-dispatch los=1 每个 cycle、conflict_time 从 1.8 递减到 0.0）。
- **未发现 one-cycle hole**：witness world time 始终落在 K3 窗 ±0.5s 内；dispatch grant 每 episode 都在。`raw_los_occlusion_interval_observed_` 也持续（`[los-occlusion-interval]` 连续打印）。§10.7 的“窗口重锚导致 witness 瞬间掉出”未出现于 EP1——事件窗刷新只延不缩。

## 6. EP2-7 全部 transition 总表（§14）

| # | EP | time (sim) | 前→后 | LOS 仍 blocked? | 同事件 | SIDE 重派 | SIDE 生成 | SIDE hard-safe | 进 comparator | NOMINAL comparator 胜 | NOMINAL 几何回归 | root class |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 7.27 | SIDE_PLUS→NOMINAL | 前 0.64s（witness 活跃） | E1c4 | YES | YES（双方） | **YES** | YES | **YES**（D3 −0.017/−0.05） | NO（s 连续） | **F** |
| 2 | 1 | 7.83 | SIDE_MINUS→NOMINAL | **YES**（损失中） | E2c2 | YES | YES（MINCO=1） | NO（DYNAMICS_FAIL×2） | NO | — | NO | **E** |
| 3 | 1 | 8.38 | FALLBACK→NOMINAL | **YES**（损失中） | E2c4 | YES | YES（MINCO=1） | NO（DYNAMICS_FAIL×2） | NO | — | NO | **E** |
| 4 | 2 | 10.66 | SIDE_PLUS→FALLBACK(NOM-init) | 前 0.02s | E3c6 | YES | YES | NO（69/70 DYNAMICS_FAIL） | NO | — | NO | **E** |
| 5 | 2 | 10.90 | FALLBACK→NOMINAL | **YES**（损失中） | E3c7 | YES | YES | NO（72/73 DYNAMICS_FAIL） | NO | — | NO | **E** |
| 6 | 3 | 25.26 | SIDE_MINUS→FALLBACK | **YES**（损失中） | E4 末期 | YES | YES | NO（cand148 DYNAMICS_FAIL） | NO | — | NO | **E** |
| 7 | 3 | 26.10 | FALLBACK→NOMINAL | **YES**（损失中，恢复前 0.4s） | E5c2 | YES | YES | NO（157/158 DYNAMICS_FAIL） | NO | — | NO | **E** |
| 8 | 4 | 35.56 | SIDE_MINUS→NOMINAL | **YES**（损失中） | E4c10 | YES | YES | NO（154/155 DYNAMICS_FAIL） | NO | — | NO | **E** |
| 9 | 5 | 63.54 | SIDE_PLUS→NOMINAL | 前 0.41s（witness 活跃） | E9c8 | YES | YES（M=1） | **YES（MINUS）** | YES | **YES**（D3 4.295 vs 4.326，−0.031） | NO | **F** |
| 10 | 6 | 70.50 | FALLBACK→NOMINAL | 前 0.18s | E6c10 | YES | YES（双方） | **YES** | YES | **YES**（C3 0.267 vs 0.000；注意 M 的 D3 3.349 更低但 C3 先决） | NO | **F** |
| 11 | 7 | 73.34 (d1) | SIDE_PLUS→NOMINAL | 前 0.14s | E4 重试风暴 c7-c26 | YES | YES | NO（N/P/M 全 0，连续 ~20 批全失败，baseline K2=0.2） | NO | — | NO（执行＝persistence） | **E** |
| 12 | 7 | 73.37 (d2) | SIDE_MINUS→NOMINAL | 前 0.28s | E7c4 | YES | YES（P=1） | **YES（PLUS）** | YES | **YES**（D3 2.093 vs 2.095，−0.002） | NO | **F** |

（EP6 @71.06 的 NOMINAL→FALLBACK 为三方全失败后的 executor persistence，非 SIDE→NOMINAL，不计入。）

**共性事实**：12/12 transition 的 cycle 里 `side_attempts=2`（SIDE 从未被停止 dispatch）；`[side-minco] success=1`（构造从不失败）；失败全部发生在最终 hard 预检；DYNAMICS_FAIL 是唯一失败原因（f125 已知的 jerk 类签名：pre-retiming `side-authoritative-dynamics pass=1`，retiming 后超限）。

## 7. 统计（§20）

```text
TOTAL_SIDE_TO_NOMINAL_TRANSITIONS = 12

LOS_STILL_BLOCKED_AT_TRANSITION = 8/12（4 次在二值损失中、4 次在起始前 0.02-0.64s 且 raw witness 活跃；
                                     12/12 的 raw LOS witness / threat-dispatch LOS 活跃）

LOS_AUTHORITY_DROPOUT        = 0
PREMATURE_EVENT_CLOSE        = 0 （d0E1 被 M2 抢占 1 次＝合法优先级，0.15s 后 sibling 事件接管）
SIDE_REGEN_NOT_DISPATCHED    = 0 （每 cycle side_attempts=2）
SIDE_REGEN_CONSTRUCTION_FAILED = 0 （MINCO success=1 12/12）
SIDE_REGEN_HARD_SAFETY_FAILED  = 8 （DYNAMICS_FAIL，唯一失败模式）
TRUE_SELECTOR_REVERSION        = 4 （D3 边际 0.002/0.017/0.031/0.05 或 C3 扫 sweep）
NOMINAL_LABEL_ONLY             = 0 作为根因（但作为解释：12/12 无几何回跳）
HIGHER_PRIORITY_PREEMPTION     = 0 （未直接制造任何一次 transition）
BLOCKER_IDENTITY_CHURN         = 0 （EP1 blocker36 全程稳定）
OTHER                          = 0

SIDE_REDISPATCH_RATE          = 12/12 = 100%
SIDE_REGEN_SUCCESS_RATE(MINCO) = 12/12 = 100%
SIDE_HARD_SAFE_RATE(transition cycles) = 4/12 = 33%
SIDE_AND_NOMINAL_COMPARATOR_RATE        = 4/12 = 33%

TRUE_GEOMETRIC_TOPOLOGY_REGRESSION_COUNT = 0
PROVEN_NOMINAL_COMPARATOR_OVERRULE_COUNT = 4
```

## 8. 五问（§19）

- **Q1**：LOS 仍 blocked 时 SIDE 是否每轮重派？——**是**。12/12 transition cycle `side_attempts=2`；threat-dispatch LOS 每 cycle 在场；无 authority dropout。
- **Q2**：第一处让 SIDE 消失的 production condition？——**`validateExecutionTrajectory` 终检 DYNAMICS_FAIL（8/12）**：retiming 后 jerk 类超限，`[local-geometry-candidate] safe=0 reason=DYNAMICS_FAIL`，SIDE 被 hard gate 剔除，NOMINAL 成唯一 hard-safe。不是 dispatch 条件、不是 K3 窗、不是事件关闭。
- **Q3**：SIDE 重新生成后是否持续 hard-safe？——**否，仅 4/12**。构造 100% 成功，hard-safe 仅 33%。
- **Q4**：NOMINAL 是否真的通过 comparator 击败 SIDE？——**只有 4/12**；且全部是边际差（D3 0.002–0.05 或 C3 sweep，三方窗内同盲 C3≤0.27）。
- **Q5**：`NOMINAL` 标签是否意味着几何回归？——**否**。s(t) 跨标签连续（EP1 逐样本验证），NOMINAL 继承 SIDE 状态（DP/DV=0），只丢弃 bulge 意图；12/12 无一例几何回跳。

## 9. Feedback129 假设判定（§17）

```text
FEEDBACK129_REVERSION_CAUSALITY: PARTIALLY_CONFIRMED
```
- 确认部分：存在真实 comparator reversion（4/12），且其与截断窗内指标无优势相关的解释自洽（F 类 4 例的全部 C3/D3 差都在窗内分辨率边缘）。
- 修正部分：**主导机制（8/12）是 SIDE 终检 DYNAMICS_FAIL——SIDE 根本没活到 comparator**。Feedback129 把执行标签翻转统一归因为“selector 在短窗内无法证明 SIDE”，对 2/3 的 transition 不成立；且“NOMINAL 放弃 topology＝几何回归”的隐含解读不成立（标签连续性，§8/§3）。

## 10. 主根因（§21）

```text
PRIMARY_ROOT_CAUSE:  E — SIDE_REGEN_HARD_SAFETY_FAILED（8/12）：retimed SIDE 多项式在最终
                     hard 预检反复以 DYNAMICS_FAIL 被拒（MINCO 构造 100% 成功、pre-retiming
                     动力学 pass），NOMINAL 作为唯一 hard-safe 幸存者被提交；损失段因此由
                     状态延续、意图丢弃的 NOMINAL 惯性穿轴执行。
SECONDARY_ROOT_CAUSE: F — TRUE_SELECTOR_REVERSION（4/12）：三方同 batch hard-safe 时，
                     NOMINAL 在截断窗内以 0.002–0.05 的边际 D3/C3 差真实获胜。

SIDE_ROLLING_PERSISTENCE_STATUS: BROKEN — 非因 selector、非因 authority 断线，而是 hard 预检
                                 无法稳定复现上一轮已验证过的 SIDE 的动力学可行性
K3_EVENT_LIFECYCLE_STATUS:       HEALTHY（EP1 无提前关闭；M2 抢占 1 次为合法优先级且有接管）
LOS_AUTHORITY_CONTINUITY_STATUS: HEALTHY（12/12 transition cycle witness/grant 在场）
SIDE_REGENERATION_STATUS:        DISPATCH+CONSTRUCTION 100%；HARD-SAFE 33%（DYNAMICS_FAIL 为主）
SELECTOR_REVERSION_STATUS:       REAL BUT MARGINAL（4/12，全部窗内分辨率边缘）
NOMINAL_PROVENANCE_STATUS:       LABEL≠GEOMETRY：状态继承、意图丢弃、零回跳
```

---

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

**最终一句话回答：真实 LOS 持续遮挡时，上一轮的 SIDE 下一轮变成 NOMINAL，12 次里 8 次是因为 SIDE 每轮都照常重派、照常解算成功（MINCO success=1），却在最终 hard 预检被 `DYNAMICS_FAIL` 拒掉、NOMINAL 作为唯一 hard-safe 候选被提交（4 次是 NOMINAL 在 comparator 里以 0.002–0.05 的边际指标差真实获胜）；而执行标签变 NOMINAL 从不等于几何回归——新 NOMINAL 继承 SIDE 的侧向状态（s(t) 跨标签连续）、只丢掉了绕行意图，于是载具以惯性漂移穿轴完成那段约 1 s 的阴影穿越。**
