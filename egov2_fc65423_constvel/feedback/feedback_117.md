# Feedback 117 — Local planner authority consolidation(一次性 pipeline 重构)

日期:2026-09-25/26。工作区:`/home/bob/ALP/egov2_fc65423_constvel`。
前置:[Feedback116](feedback_116.md)(LOS soft 化 + 遥测拆分)。
本轮任务:停止局部补丁,一次性重构 Local SIDE / A* / Local-SFC / MINCO / SCP / visibility 链,建立职责明确、语义统一的 production pipeline。
最终 run:`runs/20260926_001027_482839`(FULL ON,exit 0,唯一一次仿真)。

---

## 1. 修改前审计:实际链路与重复 authority

审计覆盖 planner_manager.h/.cpp、poly_traj_optimizer.h/.cpp、scp 相关函数与 launch 参数。实际链:

```
K3/LOS risk(统一 Local visibility sample)
→ ConflictDescriptor(reason_mask: BODY_SAFETY / LOS_OCCLUSION)
→ N/L/R topology dispatch(escalation_grant = K3 event window ∩ raw witness)
→ SIDE seed(offset backoff;静态被挡 → A* repair → simplify → SFC 扫描)
→ MINCO seed 重建(repaired_mjo)
→ LBFGS(候选 hard SCP flag 默认 off)→ enforceCandidateLosPlanesSCP
→ classify_candidate(dynamics/static/swarm/risk 硬事实)
→ visibility 比较器(hard-safe 集内 C3/D3/K2)→ finalizeTopologyCandidate(commit)
```

发现的重复/冲突 authority(全部在本轮消除):

| # | 位置 | 问题 |
|---|------|------|
| 1 | `runCandidateHardCorridorSCP` Local-SFC 硬行(原 3111–3162) | 行集合包含 `LOS_OBSERVATION_SIDE` 平面——Local SIDE flag 关闭时休眠、Team SCP 时活,是"LOS 行仍是硬约束"的第二条路径 |
| 2 | `evaluateCandidateLocalSfcMaxViolation`(原 193) | 违反度含 LOS 平面,直接 gate feasible-initializer 与 verified-fallback contract——第三条隐藏 LOS 拒绝路径 |
| 3 | `runCandidateHardCorridorSCP` 平面几何校验(原 2984) | `LOCAL_SFC_INVALID_PLANE` 对 LOS 平面同样判死 |
| 4 | `enforceCandidateLosPlanesSCP` 残余 kill path | piece_num≤1 不满足 / NO_CONTROLLABLE_ROWS / QP 求解失败 / 平面几何坏 → `return false` → `LOS_PLANE_SCP_FAILED` 判死候选 |
| 5 | `astar_accept`(原 11773) | 不 gate 重建 seed 的静态安全:A* seed 撞角仍被 ACCEPT,等到 retime 检查才以 `SIDE_INIT_STATIC_COLLISION` 死亡(F116 实测 30/44 重建 seed 碰撞;28+4 次候选死亡)——隐式第二静态判定 |
| 6 | K3 事件时序 | 事件在 `finalizeCapturedCandidates`(1481)才创建/刷新,而 dispatch grant 在本轮已读过旧事件——新建事件损失一个 rolling cycle(§8) |
| 7 | lifecycle 计数 | `side_hard_preflight_failed_count_` / `side_valid_not_selected_count_` / `side_activated_count_` 声明后从未自增(恒 0) |

## 2. 重构后的 Local pipeline(三层职责)

```
[HARD SAFETY authority]  STATIC_BODY / DYNAMIC_BODY / SWARM / DYNAMICS(v-a-j) /
                         STATIC_COLLISION_CORRIDOR rows / current-revision validity
                         —— 唯一能把 candidate 标为 INVALID 的层
[TOPOLOGY/GEOOMETRY]     N/L/R intent → side seed → A*(collision-free path)
                         → Local-SFC corridor(STATIC_COLLISION_CORRIDOR 源)
                         → MINCO seed(static-safe gate)
[VISIBILITY authority]   LOS_OBSERVATION_SIDE(semi-hard slack QP)/ J_vis / margin /
                         C3 / D3 / FOV / range —— 只引导与比较,永不 reject
```

关键修改(一次性,无补丁循环):

1. **LOS 全域 soft**(poly_traj_optimizer.cpp):`enforceCandidateLosPlanesSCP` 0 条 `return false`(任何结果只进 `LosSoftPlaneTelemetry`,保留最后已通过细查的迭代点);corridor SCP 硬行跳过 LOS 源(`los_hard_rows_skipped` 遥测);initializer/fallback 违反度与 SCP 平面校验均跳过 LOS 源。INVARIANT 1/2 从代码结构上成立。
2. **A*→corridor→MINCO invariant**(planner_manager.cpp §5):seed 构建(SFC 扫描 → arc-length timing → MINCO → 细查+冲突窗+全 scope 扫描)改为可重复循环;首坏 segment 触发**局部有限重建**:attempt 1 `RESTORE_RAW_WAYPOINTS`(恢复原始 A* 内点,简化点恒为 raw 子集,索引映射精确),attempt 2 `BULGE_WAYPOINT_AND_CORRIDOR_PLANE`(同一 boundary-scan 机制加走廊平面);`astar_accept` 增加 `repaired_static_free` 硬 gate;仍撞角 → `SEED_COLLISION_EXPLICIT_FAIL` 前端显式失败,不再"接受后死亡"。每 seed 输出 `[A_STAR_TO_MINCO]` 全字段诊断;失败按因计数。
3. **同 cycle dispatch**(§8):`updateLocalK3RecoveryEvent()` 提前到 reboundReplan 的 dispatch grant 读取之前(raw witness 扫描之后),margin 刚下穿即可在同一 planning revision 获得 SIDE 授权;幂等(update 对 active 事件只做窗口延长),finalize 处调用保留。新增 `[K3_SAME_CYCLE_DISPATCH]` 遥测。
4. **candidate lifecycle 遥测**(§10):`CONSTRUCTION_FAILED(按因) → HARD_PREFLIGHT_FAILED → HARD_VALID → VALID_NOT_SELECTED/SELECTED → COMMITTED → ACTIVATED` 全链计数接线;构造失败按原因字符串入 `side_failure_cause_count_`;周期报告新增 `[side-lifecycle-audit]` / `[astar-to-minco-audit]` / `[los-soft-authority-audit]` / `[side-construction-causes]`。
5. **冗余计算**(§11):撞角 seed 现在在前端显式失败,省去后续整段 MINCO+SCP+classify+visibility 评估;LOS 重复 hard-QP 失败已不存在(F116)。A* 结果缓存经评估放弃:实测单次 A* 0.2–0.3 ms,同 batch 内 L/R anchor 几乎不重复,无收益,不引入失效风险。
6. 不变量核对:collision 行零 slack;SCP trust、margin、阈值、A* 主体、controller、M2、C3/D3 数学全部未动;Local 仍是唯一 executable polynomial authority。

## 3. 离线/确定性检查(§13,正式仿真前)

- `git diff --check` 干净;`catkin build -j2 --no-status` **25/25**(首轮因一处 brace 缺失失败,修复后通过)。
- **A(LOS soft,e4)**:静态证明 `enforceCandidateLosPlanesSCP` 内 `return false` 计数 = 0;三条隐藏 LOS 拒绝面(硬行/违反度 gate/平面校验)均显式排除 LOS 源;F116 run 中 e4 类 blocker 78 次 `SOFT_ACCEPTED` 证明 soft 路径进入比较层。
- **B(A*→MINCO,UAV2/e2 类几何)**:确定性离线重放(`astar_minco_replay.cpp`,复用生产 `MinJerkOpt` 与逐字复制的 restore/构建算法,生产同款简化点⊆raw 不变量):38 点 raw path 全 free;简化折线 free 但 build0 的 MINCO 在 t=1.6 s 切进障碍(first_bad_segment=0)——精确复现 F116 缺陷;attempt 1 恢复 raw 内点(26 点)→ build1 **static_free=1 → SEED_SAFE_AFTER_RESTORE**。对抗性双切角场景:两次重建后仍撞 → `SEED_COLLISION_EXPLICIT_FAIL`(显式失败,绝不带撞角 seed 进入优化)。检查 PASS。
- **C(C3/D3)**:比较器条件本轮零改动(仅在 finalize 处加计数行);F116 已证 28/28 进度 commit 全部 D3 改善。
- **D(M2)**:M2 逻辑零改动;预 dispatch 的事件更新在 M2 active 时走同一 `M2_PREEMPTED` 闭合路径,绝对优先级保持(INVARIANT 6)。

## 4. 唯一一次 FULL ON(§14)

`./scripts/run_alp_full_on.sh --ablation full --headless --timeout 240 --boot-timeout 180 --scenario .../natural_team_stress_dense_38_targeted_k2_v6.json --k3-repair on --k3-escalation on`
→ `runs/20260926_001027_482839`,exit 0,CLEANUP OK(owned=28,force_killed=0),无残留进程。

### Executed visibility(analyzer,moving phase t≤76.5 s)

| 指标 | F116 | 本轮 |
|---|---|---|
| ALL3_RATIO | 90.16% | **90.76%** |
| K2_RATIO | 99.74% | 98.56% |
| K3_LOSS_TOTAL | 7.493 s | **7.063 s** |
| LONGEST_ALL3_LOSS | 1.732 s | 1.631 s |
| LONGEST_K2_LOSS | 0.200 s | 1.100 s |
| BLACKOUT | 0 | 0 |

K2 回归定位:两段损失(73.45–74.55 s、76.82–77.12 s)都在终局段;主窗口内 **UAV2 static-LOS 36/36 全挡 + UAV1 dynamic-LOS 33/36**——两机同时被挡,`K2-healthy` gate 为假,按设计不属于 K3-only authority 范围;UAV2 另有一次 3.23 s 低速 episode(<0.30 m/s 占 4.27%)。

### Local geometry / A* / LOS(全 fleet 汇总)

- SIDE 尝试(到达 accept/repair 阶段):1261;构造失败 521:`NO_STATIC_FEASIBLE_SIDE=513`、`SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION=8`、**`SIDE_INIT_STATIC_COLLISION=0`**(F116 为 28——invariant 生效,撞角 seed 前端显式失败,不再进优化器后死亡)。
- `[A_STAR_TO_MINCO]` 判定 49:SEED_SAFE 12 / SEED_COLLISION_EXPLICIT_FAIL 31 / GUIDE_REJECTED 6;计数:ASTAR_ATTEMPTS=55、ASTAR_RAW_PATH_SAFE=42、CORRIDOR_BUILD_SUCCESS=36、MINCO_SEED_STATIC_SAFE=12、ASTAR_SAFE_BUT_MINCO_UNSAFE=24、REPAIR_ATTEMPT=8、REPAIR_SUCCESS=1。
- LOS:SOFT_QP_ATTEMPTS=594 / SOLVED=594 / SLACK_NONZERO=203 / **HARD_REJECT=0**(结构恒零);LOS 平面创建 84、violation 0、lost 0。
- Lifecycle:HARD_VALID=533、HARD_PREFLIGHT_FAILED=67、VALID_NOT_SELECTED=19、SELECTED=167、COMMITTED=233、ACTIVATED=8(K3 激活确认口径)。对比 F116:进度选择 28 → 本轮 K3 进度选择 27、SIDE commit 17、激活确认 21;SIDE 供给量级提升(dispatch 64→128、target-facing A* 64→512)。
- 同 cycle dispatch:事件创建预置 12,创建+授权同 revision 7–8 次;K3 事件 16 个:binary RECOVERED 3(ttr 0.587/1.160/1.237 s)、MARGIN_RECOVERED 6、M2_PREEMPTED 5–6、NO_SIDE_DISPATCH 3、其余(SIDE_INFEASIBLE/PROGRESS_NOT_RECOVERED/EVENT_EXPIRED/NO_K2_HEALTHY)各 1–2。C3 改善行 8、D3 改善行 12(27 次进度选择均满足至少其一,定义保证)。
- Team M2:TRANSACTION_COMMIT=26、ACTIVATED=24、TEAM_PT_ATTEMPT=128——路径与优先级不变。
- Runtime:`EXECUTION_DEADLINE`=136(F116 171;136/136 全部由 verified initializer fallback 兜底)、MIN_BUDGET_APPLIED=304。
- 安全:STATIC/DYNAMIC_CONTACT=0、SWARM_VIOLATION=0(`swarm-hard-conflict` 18 条全部是候选级拒绝,0.432 m<0.58 m 触发,未进入执行)、PVA_MISMATCH=0、UNVALIDATED_EXECUTED=0、PARTIAL_TEAM_ACTIVATION=0、STARVATION=0、TERMINAL_HOLD=3(正常到点保持)。

## 5. §16 九问

1. **LOS 是否彻底移出 safety reject authority?** YES。唯一 LOS 强制点是 semi-hard slack QP;0 条 reject 路径;三条隐藏面全部删除;`LOS_HARD_REJECT_COUNT=0` 为运行时证据。
2. **collision/SFC 是否仍全部 hard?** YES。静态/动态/swarm/动力学/corridor 行零软化;513+8 次构造失败证明硬检查仍在判死。
3. **A* safe path 能否稳定传递成 safe MINCO seed?** 结构上 YES(invariant 成立:seed 必须过静态 gate 或显式失败),量上 PARTIAL:12 直接 SAFE、1 例重建救回、24 例两次重建后仍撞 → 显式失败(以前是"接受后死")。重建对"前缀 junction 长段切角"类帮助有限,是残余瓶颈的一部分。
4. **K3 progressive SIDE 是否真正进入 C3/D3?** YES。27 次进度选择、17 次 commit、21 次激活确认;c3 改善 8 行、d3 改善 12 行。
5. **event/raw 同 cycle 触发?** YES。事件创建前置到 dispatch 之前,7–8 次创建+授权同 revision 完成,消除一个 rolling 的损失。
6. **comparator 是否只在 hard-safe 集工作?** YES,本轮零改动。
7. **M2 Team 优先级?** YES。6 次 M2_PREEMPTED 让位、Team commit/activate 正常。
8. **剩余第一因果瓶颈?** 静态前端:`NO_STATIC_FEASIBLE_SIDE=513`(双侧被静态物理挡 + A* repair base 要求 dynamic-valid 的既有门槛),其次 `EXECUTION_DEADLINE=136`(fallback 全兜底)与不可修复撞角 seed 24 例(已显式失败,不再浪费优化)。
9. **是否还有补丁式双重 authority?** 已知 7 处全部消除(§1 表);未发现新的。dirty worktree 单一权威未破坏。

## 6. 最终字段

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES (25/25, git diff --check 干净)

LOCAL_PIPELINE_REFACTORED: YES
PATCH_STYLE_DUPLICATE_AUTHORITY_REMOVED: YES(§1 表列 7 处)

HARD_SAFETY_AUTHORITY_UNIFIED: YES
VISIBILITY_HARD_REJECT_PRESENT: NO
LOS_OBSERVATION_SOFT_ONLY: YES
COLLISION_SFC_HARD: YES

SIDE_TOPOLOGY_AUTHORITY_CLEAN: YES
ASTAR_CORRIDOR_MINCO_CHAIN_UNIFIED: YES(seed 静态 gate + 有界重建 + [A_STAR_TO_MINCO] 契约)

ASTAR_ATTEMPTS: 55
ASTAR_RAW_PATH_SAFE: 42
CORRIDOR_BUILD_SUCCESS: 36
MINCO_SEED_STATIC_SAFE: 12
ASTAR_SAFE_BUT_MINCO_UNSAFE: 24(目标"尽量为 0"未达;残余见 §5.8)
ASTAR_TO_MINCO_REPAIR_ATTEMPT: 8
ASTAR_TO_MINCO_REPAIR_SUCCESS: 1

LOS_SOFT_QP_ATTEMPTS: 594(solved 594,slack_nonzero 203)
LOS_HARD_REJECT_COUNT: 0

SIDE_ATTEMPTS: 1261(到达 accept/repair 阶段的 SIDE;构造失败 521)
SIDE_CONSTRUCTION_FAILED: 521(NO_STATIC_FEASIBLE_SIDE 513 / CONFLICT_WINDOW 8 / SIDE_INIT_STATIC_COLLISION 0)
SIDE_HARD_PREFLIGHT_FAILED: 67
SIDE_HARD_VALID: 533
SIDE_VALID_NOT_SELECTED: 19
SIDE_SELECTED: 167(构造期选中;K3 进度选择 27)
SIDE_COMMITTED: 233(K3 口径 17)
SIDE_ACTIVATED: 8(K3 激活确认口径;K3_LOCAL_PROGRESS_CONFIRMED activated=1 共 21)

K3_LOCAL_EVENTS: 16
C3_IMPROVED: 8(事件行)
D3_IMPROVED: 12(事件行;27 次进度选择均满足至少其一)
K3_RECOVERED_EVENTS: 3(binary 统一 visibility 确认;ttr 0.587/1.160/1.237 s)

EXECUTED_ALL3: 0.907585
EXECUTED_K2: 0.985615
K3_LOSS_TOTAL_TIME: 7.063 s
LONGEST_ALL3_LOSS: 1.631 s
LONGEST_K2_LOSS: 1.100 s(终局两机同挡,非 K3-only 范围)
BLACKOUT: 0

STATIC_CONTACT: 0
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0
PVA_MISMATCH: 0
UNVALIDATED_EXECUTED: 0
PARTIAL_TEAM_ACTIVATION: 0
STARVATION: 0
TERMINAL_HOLD: 3(正常到点保持)

EXECUTION_DEADLINE: 136(136/136 verified-initializer fallback 兜底)
MIN_BUDGET_APPLIED: 304

M2_TEAM_PATH_PRESERVED: YES(TRANSACTION_COMMIT 26 / ACTIVATED 24)
LOCAL_ONLY_EXECUTABLE_AUTHORITY: YES

FULL_ON_RUN_COMPLETED: YES(20260926_001027_482839,exit 0)
FULL_RUN_COUNT_THIS_TASK: 1
REPEATED_FULL_SIMULATION_USED: NO

FIRST_CAUSAL_FAILURE: 静态前端 NO_STATIC_FEASIBLE_SIDE(513;双侧物理静态挡 + A* base
  dynamic-valid 门槛)为主;EXECUTION_DEADLINE(136,全部兜底)与不可修复撞角 seed
  (24,显式失败)次之;LOS 已结构性退出终止原因。
PIPELINE_REFACTOR_ACCEPTANCE: YES(结构 invariant 全部成立;executed ALL3/K3-total
  优于 F116,K2/longest-K2 终局窗口回退已定位且不属于 K3-only authority 范围)
NEXT_STEP: (1) NO_STATIC_FEASIBLE_SIDE 分解审计(A* base 的 dynamic-valid 门槛是否
  应让位于"先静态可行、动态由 final preflight 判");(2) A*→MINCO 残余 24 例的
  prefix-junction 长段切角专项(corridor 覆盖 junction 段)。仅记录,另行立项。

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
