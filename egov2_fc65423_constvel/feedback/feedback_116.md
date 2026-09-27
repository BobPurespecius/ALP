# Feedback 116 — LOS observation plane 软化(semi-hard slack):消除 SIDE 的最大单点杀手

日期:2026-09-25
工作区:/home/bob/ALP/egov2_fc65423_constvel(RRCT 未访问/未修改)
前置:Feedback115(K3 SIDE 失败只读审计,CAUSE 1 = 已知遮挡区间的 LOS
observation plane 在当前阴影内立即生效,QP `primal_infeasible` 直接杀死
SIDE candidate,发生在 C3/D3 比较之前)
最终 run:`runs/20260925_224743_436366`(FULL ON,BOOT-12,exit 0,78.3 s)

---

## 1. 修改前的 LOS plane 接线(§1 审计结论)

链路:`reboundReplan` 生成 N/L/R 时把 observation half-space 打包成
`LocalSfcPlane(source=LOS_OBSERVATION_SIDE)` → `optimizeTrajectory` 主 MINCO
成功后调用 `enforceCandidateLosPlanesSCP`
(`poly_traj_optimizer.cpp:225`)→ 每 plane 25 行 `m_s^T(p(t_k)−p_T(t_k)) ≥
−margin` 的**硬行**(lower_bound=−margin)+ trust 盒 → OSQP;`primal_infeasible`
或 MAX_ITER → return false → `last_candidate_final_status_reason_=
"LOS_PLANE_SCP_FAILED"` → `optimizeTrajectory` 返回 false → `result.success=
false` → candidate 死亡,永不进入 `finalizeCapturedCandidates` 的 C3/D3。
第二条路径:`planner_manager.cpp` 前端平面恢复检查在最终多项式离开
observation half-space 时 `result.success=false`
(`REJECT_ALTERNATIVE_LOS_PLANE_VIOLATED`)。
**LOS 与真实安全 plane 确曾共用硬行逻辑**:`STATIC_COLLISION_CORRIDOR` 与
`LOS_OBSERVATION_SIDE` 在 hard-corridor SCP 的 Local-SFC 行循环里不区分 source
(2964-3033)。全部行都是硬约束,无 slack。

## 2. 修改内容(只软化 LOS,不动任何安全约束)

| 文件 | 修改 |
|---|---|
| `poly_traj_optimizer.h` | 新增 `solveExecutionQPWithSlack(g,A,lo,hi,h,soft_rows,w1,w2,&slack_max,&slack_sum)`;新增 `LosSoftPlaneTelemetry` 结构 + `lastLosSoftPlaneTelemetry()`;`los_observation_slack_weight_` |
| `poly_traj_optimizer.cpp` | `solveExecutionQPWithSlack`:决策变量扩展 `[x; ξ]`(ξ≥0),前 soft_rows 行变为 `a·x+ξ≥lo`,目标 `w1·Σξ+0.5·w2·Σξ²`(对角 Hessian 上 w2);其余行硬;返回 step 截回原 n 维。`enforceCandidateLosPlanesSCP` 重写:LOS 行经 slack-QP 求解;QP solved 即接受(非线性复核 `los_max_violation` 结果只进遥测 `SOFT_ACCEPTED final_violation=…`),不再 return false;`qp.success=false`(EXECUTION_DEADLINE/solver 失败/其他 hard rows 无解)仍失败,状态改为 `HARD_ROWS_INFEASIBLE:<status>`;新增参数 `optimization/los_observation_slack_weight=1e3` |
| `planner_manager.cpp` | ① 前端平面恢复检查:LOS violation 不再 `success=false`,改 `KEEP_ALTERNATIVE_LOS_PLANE_SOFT`(遥测);② `side_both_failed` 判定从 classify 前的默认 INVALID 改为构造阶段事实(success),新增 `[side-construction]` 行与 `side_construction_failed_count_`;③ 新增遥测 `[LOS_SOFT_PLANE]`(逐候选)与 `[K3_SIDE_SOFT_LOS]/[K3_SIDE_SOFT_LOS_COMMIT]`(逐事件因果,含 c3/d3_improved);④ 新计数 `side_construction_failed/side_hard_preflight_failed/side_valid_not_selected/side_selected/side_activated` |
| 未动 | C3/D3 comparator、M2 Team repair、SCP trust、required margin、SIDE offset、安全阈值、A*、controller、Team K3 optimizer |

**语义**:ξ=0 ⇔ 原硬约束(§5:已满足时仍保持约束,防止穿回错误侧);ξ>0 ⇔
candidate 尚未完全绕出阴影,但 optimizer 主动缩小 LOS violation;slack>0 绝不
导致 INVALID。执行安全仍由 STATIC/DYNAMIC BODY、SWARM、动力学、真实 collision
SFC 与 current-revision preflight 决定;K3 恢复仍只由统一 binary visibility
(range∧static∧dynamic∧FOV)确认。

**权重理由**(唯一新参数):trust 盒把 Δp 限制在 O(0.1~1.5)m,原 QP 单位
Hessian 下走步代价 O(1);w2=1e3 使 1cm slack 与 1m 走步同量级、1m slack 代价
≈500,既允许"尚未绕出"的渐进解,又强推动压零。w1=0.5·w2·1e-3 为微小 L1 项。

## 3. 离线确定性 QP sanity(非仿真)

构造 Feedback115 UAV1/e4 类矛盾(LOS 行要求 x0≥1.0,trust 盒 |x0|≤0.1):

```
HARD: success=0 status=primal_infeasible      ← 原行为(115 的 8/8 失败复现)
SOFT: success=1 status=solved step=[0.1000 0.0000 0.9000]   ← ξ=0.9>0
CHECK: slack>0=1 trust_satisfied=1
```

## 4. 最终 FULL ON 结果(RUN 20260925_224743_436366)

**LOS 硬失败完全消失**:`LOS_PLANE_SCP_FAILED`=0(115:62 次侧尝试);
`primal_infeasible` 仅存于 telemetry 侧计数快照;`HARD_ROWS_INFEASIBLE`=0。
534 次 slack-QP 迭代全部 `solved`;`SOFT_ACCEPTED` 78 个候选(另有 76 次
SATISFIED 即 ξ=0 提前收敛,§5 语义生效)。

**SIDE 生成与漏斗**(19 个 Local K3 事件,163 个比较 batch):
- 事件创建 19(全部 Local 自主,`M2_PREEMPTED` 让位 7);
- 有 SIDE 存活的比较 batch:PLUS success=1 于 19 batch / MINUS 于 19 batch
  (115 的 157 batch 中大量双侧全灭);
- 进度选择 28(SIDE_PLUS 12 / SIDE_MINUS 6 / NOMINAL 10),
  **D3 改善 28/28 次 commit**;C3/D3 改善分布:`c3&d3`=11、`仅d3`=7、`均无`=6;
  commit 28、激活确认 24;
- 事件结果:RECOVERED 6(ttr 0.003–1.38 s)、M2_PREEMPTED 7(Team 优先,
  无竞争)、PROGRESS_BUT_NOT_RECOVERED 4、NO_SIDE_DISPATCH 2。

**剩余 SIDE 杀手(全部真实安全几何,非 LOS)**:
- 静态前端:`NO_STATIC_FEASIBLE_SIDE` 197 / `SIDE_INIT_STATIC_COLLISION` 28 /
  conflict-window 4(115 的 CAUSE 2,本轮按要求只记录);
- 预算:`side-minco status=EXECUTION_DEADLINE` 171(走 feasible-initializer
  fallback 的仍可活);
- 未细分 scp_success=0 的 SIDE±80(多为静态 preinit 失败的下游)。

**执行可见性**(vs Feedback114 / 113):

| 指标 | F116(本轮) | F114 | F113 |
|---|---|---|---|
| ALL3 | 0.9016 | 0.8896 | 0.9037 |
| K2 | **0.9974** | 0.9906 | 0.9979 |
| K3 loss 总时长 | 7.493 s | 7.892 s | 7.359 s |
| LONGEST_ALL3_LOSS | 1.732 s | 1.365 s | 1.765 s |
| LONGEST_K2_LOSS | **0.200 s** | 0.733 s | 0.167 s |
| BLACKOUT | 0 | — | — |

(单 run 方差内,K2/longK2 明显改善,K3 总时长介于两者之间;按任务要求不做
方差级调参比较。)

**安全/liveness**:STATIC_CONTACT 0、DYNAMIC_CONTACT 0、SWARM_VIOLATION 0、
PVA_MISMATCH 0、UNVALIDATED_EXECUTED 0、PARTIAL_TEAM_ACTIVATION 0、
STARVATION 1(M2 seam)、TERMINAL_HOLD 0、MIN_BUDGET_APPLIED 34。
**软化 LOS 后没有任何安全指标回退**;软 candidate 全部经同一硬 preflight。

## 5. §13 三问

1. **LOS_SOFT_CONSTRAINT_WORKS: YES(机制层面完全生效)**——原来必然
   `primal_infeasible` 的 e4 类几何现在 QP solved、候选以 hard-safe 身份进入
   比较层(确定性 sanity + 534/534 次 solved + 0 次 LOS 硬失败);28 次进度
   commit 全部 D3 改善,24 次激活。注意:本 run 的 LOS 硬失败基数与 115 的
   113/114 run 不同(事件时序/场景分岔),机制收益以"LOS 硬失败归零 +
   soft candidate 全部过 preflight + 28/28 D3 改善"为准。
2. **K3_PROGRESS_IMPROVED: YES**——28/28 commit D3 下降,18/24 有 C3 或 D3
   双改善;6 个事件二值恢复(RECOVERED)。
3. **K3_EXECUTED_IMPROVED: PARTIAL**——K2 0.9906→0.9974、longest-K2-loss
   0.733→0.200、K3 总时长 7.892→7.493(对 114);ALL3 0.8896→0.9016;
   但对 113(0.9037/7.359)持平略低。单 run 方差内正向。

## 6. FIRST CAUSAL FAILURE(剩余)

软化 LOS 后,SIDE 的剩余终止全部来自:(a)**静态前端**
`NO_STATIC_FEASIBLE_SIDE`/`SIDE_INIT_STATIC_COLLISION` 共 229 次——115 已证
A* raw path 安全而平滑 MINCO seed 切角碰撞(A*→SFC→MINCO 几何保持);
(b) **EXECUTION_DEADLINE** 171 次(部分由 feasible-initializer fallback 兜底)。
LOS 不再出现在任何首次终止原因里。

NEXT_STEP(仅记录,未实施):A* raw path → SFC/MINCO 的静态可行性保持
(Feedback115 §8.2)。

---

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES (25/25, git diff --check 干净)

LOS_OBSERVATION_PLANE_SOFTENED: YES
LOS_HARD_EXECUTION_GATE_PRESENT: NO
COLLISION_SFC_REMAINS_HARD: YES
SCP_TRUST_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO

LOS_PLANE_QP_ATTEMPTS: 534 slack-QP 迭代(+76 次 SATISFIED 提前收敛,740 条 [LOS_SOFT_PLANE])
LOS_PLANE_PRIMAL_INFEASIBLE: 0(候选终止路径;115 为 62 次侧尝试失败)
LOS_SOFT_SLACK_CANDIDATES: 78(SOFT_ACCEPTED;slack_max p50=0.64, mean=0.80, max=3.51, 零 slack 26)
LOS_SOFT_SLACK_VALID: 19+19 个 batch 存在 hard-safe SIDE(PLUS/MINUS)
LOS_SOFT_SLACK_SELECTED: 28(18 SIDE / 10 NOMINAL;SIDE_PLUS 12,SIDE_MINUS 6)
LOS_SOFT_SLACK_ACTIVATED: 24
LOS_SOFT_SLACK_RECOVERED: 6(RECOVERED,ttr 0.003–1.38s;M2 让位 7 未计入)

SIDE_CONSTRUCTION_FAILED: 111(batches with both sides failed;全部因静态 preinit/预算,LOS=0)
SIDE_HARD_PREFLIGHT_FAILED: 0 证实的 SIDE 专属拒绝(拒绝计数不再混分类前默认值)
SIDE_VALID_NOT_SELECTED: 10(NOMINAL 胜出的比较 batch)
SIDE_SELECTED: 18(SIDE 进度选择)
SIDE_ACTIVATED: 24(激活确认,含同事件多轨迹)

K3_LOCAL_EVENTS: 19
K3_RECOVERED_EVENTS: 6(binary,统一 visibility authority 确认)
C3_IMPROVED_EVENTS: 11(c3&d3 双改善 commit)
D3_IMPROVED_EVENTS: 28/28(全部进度 commit)

EXECUTED_ALL3: 0.901618
EXECUTED_K2: 0.997445
K3_LOSS_TOTAL_TIME: 7.493 s
LONGEST_ALL3_LOSS: 1.732 s
LONGEST_K2_LOSS: 0.200 s

STATIC_CONTACT: 0
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0
PVA_MISMATCH: 0
UNVALIDATED_EXECUTED: 0
PARTIAL_TEAM_ACTIVATION: 0
STARVATION: 1(M2 恢复 seam)
TERMINAL_HOLD: 0

LOS_SOFT_CONSTRAINT_EFFECTIVE: YES
K3_METRIC_IMPROVED: PARTIAL(vs F114 全面正向;vs F113 方差内持平)
SAFETY_REGRESSION: NO
FIRST_CAUSAL_FAILURE: 静态前端 NO_STATIC_FEASIBLE_SIDE/SIDE_INIT_STATIC_COLLISION(229 次)
  与 EXECUTION_DEADLINE(171 次,fallback 部分兜底);LOS 已从首次终止原因中消失
ASTAR_TO_MINCO_NEXT_STEP_JUSTIFIED: YES(数据充分,另行立项)

FULL_ON_RUN_COMPLETED: YES(20260925_224743_436366,BOOT-12,exit 0)
FULL_RUN_COUNT_THIS_TASK: 1
REPEATED_FULL_SIMULATION_USED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
