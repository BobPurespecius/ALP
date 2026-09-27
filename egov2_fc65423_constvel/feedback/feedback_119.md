# Feedback 119 — Local 前端一次性收敛:A* admission 解耦 / 有界静态 corridor / MINCO-SCP 合同 / LOS 彻底 soft 化

日期:2026-09-26。工作区:`/home/bob/ALP/egov2_fc65423_constvel`。
性质:基于 Feedback 118 新鲜审计(RC1/RC2/RC4)的一次性 production 重构,不再打补丁。
唯一 FULL ON run:`runs/20260926_031106_562007`(exit 0,BOOT-12,cleanup OK,`FULL_RUN_COUNT=1`)。
未访问 /home/bob/RRCT;未执行任何 git reset/checkout/restore/clean。

---

## 0. 一句话结论

前端合同已经按 mandate 重排:**A* admission 与 1.1 m 动态软余量解耦(74/74 静态不可行 SIDE 全部进入 A*,raw path 74/74 静态安全)、Local-SFC 变成逐 segment 有界凸 corridor(10 建 10 成、0 构建失败、整箱审计)、restore/bulge 修补链与三个 seed 静态拒绝标签结构性删除、LOS 在所有接口上退出 hard reject(847 次 soft QP 全 solve,HARD_REJECT=0)**。K2 执行质量显著改善(98.56→99.48,最长丢失 1.10→0.40 s)、NO_STATIC_FEASIBLE_SIDE 513→87(−83%)、MIN_BUDGET 304→78(−74%)。**但 corridor SCP 链 10 试 10 败**,根因不是几何而是 seed 数值质量:A*-repair seed 的 initial_jerk 实测高达 20731(F117/F118 遥测早已预告 2534),硬 SCP 的 trust region 与 OSQP(2000 iter,solved_inaccurate)无法收敛;另有 63/74 条 A* guide 被 side 语义检查(GUIDE_REJECTED)拦在 corridor 之前——这是 mandate 三类拒绝结构之外唯一存活的拓扑类拒绝,本轮已定位、记录、列为 NEXT_STEP,未在 FULL 后回改 production。

---

## 1. 修改清单(一次性,全部落地)

| # | 文件 | 修改 | 对应 mandate |
|---|---|---|---|
| 1 | planner_manager.cpp(10452 区域) | `astar_base_valid` 资格从 BODY 分支 `min_distance ≥ getMovingObjClearance(1.1m)` 改为 `!hard_collision`(hard-clearance 真阈值保持);1.1 m 仅保留在 DIRECT_ACCEPT 的 usefulness 判定 `side_dynamic_valid`(非 A* 资格) | §1 |
| 2 | planner_manager.cpp(corridor 块,~11390–11790) | Local-SFC 重写:逐 segment 有界凸区域(左右墙=inflated map 双向边界扫描 + 前后帽=±1 cell 重叠);三站扫描取每侧最小自由距离;整箱采样审计 corridor ⊂ static free;单段失败只允许**几何中点局部细分**(depth≤2);时间窗=seed 时标 ± cap 穿越时间;只对"必要 segment"(有墙者及其邻段)建域,开阔段保持旧 free-probe 语义 | §2 |
| 3 | planner_manager.cpp(seed 块) | MINCO seed 单遍生成(时标表:曲率限速 0.8/0.8/0.10 与 corridor 窗共享同一时间轴);seed 静态检查降级为 64 点 telemetry,不再是 gate;**RESTORE_RAW_WAYPOINTS / BULGE_WAYPOINT_AND_CORRIDOR_PLANE / ×≤3 重建循环整块删除**;一个 SIDE = 1 A* + 1 corridor + 1 条优化链 | §3 |
| 4 | planner_manager.cpp(12104 区域) | `astar_accept` 删除 `repaired_static_free`;verdict 词汇改为 SEED_SAFE / SEED_CUT_CORNER_SCP_REPAIRS / GUIDE_REJECTED;SIDE_INIT_STATIC_COLLISION 与 SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION 两个 seed 再检 gate 删除(保留 checker ERR 中止与 telemetry) | §3/§6 |
| 5 | poly_traj_optimizer.cpp(optimizeTrajectory dispatch) | 携带 STATIC_COLLISION_CORRIDOR 平面的 SIDE 候选强制走 runCandidateHardCorridorSCP(现有 P/T hard SCP);成功后仍执行 enforceCandidateLosPlanesSCP(soft);NOMINAL 与无 corridor SIDE 保持 LBFGS fast path | §4/§8 |
| 6 | poly_traj_optimizer.cpp(runCandidateHardCorridorSCP) | Local-SFC 硬行改 **active-set 协议**:初始每平面=端点+中点+seed 连续违反 argmax;每轮 SCP 用 **五次多项式连续 violation(端点+f'(t)=0 实根,Durand-Kerner)**解析定位真实违反点 t*,追加进下一轮行集;trial 验收对 corridor 项从绝对 ≤2e-3 改为单调不恶化(与其余各项一致),绝对干净由出口 LOCAL_SFC_FINAL_VIOLATION(连续 ≤2e-3)强制;**plane×25 均匀暴力行扩张删除**(只保留为诊断 metric);新增 CorridorScpTelemetry + `[corridor-scp-active-set]` 遥测 | §4 |
| 7 | planner_manager.cpp(validateRetimedLocalSfc) | **RC4 修复**:LOS_OBSERVATION_SIDE 平面完全跳过(不再经 retime 路径拒绝候选);STATIC 平面拒绝从静默 reason="" 改为 `[retime-sfc-reject]` 带因 WARN | §5 |
| 8 | planner_manager.h/.cpp | 新计数:ASTAR_REQUIRED / ASTAR_NO_PATH / CORRIDOR_BUILD_FAILED / CORRIDOR_SCP_ATTEMPT / SUCCESS / INFEASIBLE / CONTINUOUS_CORRIDOR_VIOLATION + `[corridor-contract-audit]` 审计行;NO_STATIC catch-all 分因(ASTAR_DISABLED / NO_VALID_SIDE_SEED / ASTAR_REPAIR_CHAIN_FAILED);post-optimize 逐候选 `[corridor-scp-contract]` | 统计 |

明确不做(遵守 §7/§8):未新增 FIRI/space-time A*/第二优化器/FSM/fallback/cache/线程/budget 参数;SCP trust、安全阈值、MINCO 主体、OSQP、N/L/R、A* 本体、simplification、动态预测、swarm、J_vis、C3/D3、K3 event、M2、lifecycle、binary recovery 全部保持。

### 离线确定性验证抓出的 3 个 production bug(上 FULL 前修复)

离线 replay(A–E,/tmp/f119_offline_check.cpp,F117 角点案例,17/17 PASS)在合入前抓出并修复:
1. **墙平面 normal/point 配对镜像错误**(左墙用了右墙方向)——约束会把轨迹允许进障碍侧;离线 B("corridor ⊇ raw path")失败暴露;
2. **caps 同样镜像**(front cap 变成 s≤−0.1);
3. **`getCoeffMat()` 是降幂排列**(col(5)=常数项)——连续 violation 的五次系数必须取 col(5−k);离线 C(violation 66 m vs brute 4 m)暴露。
另:seed 时间窗重叠从"半段时长"改为"cap 区穿越时间 ×4"——半段重叠会让墙窗覆盖到轨迹已合法离开该 box 的时段(离线 C 定位)。
最终离线结果:**A(admission 解耦:hard 与 1.1 之间 seeds 全放行、sub-hard 仍拒)/ B(corridor ⊂ free、⊇ raw path、邻接重叠、细分可用)/ C(seed 切角 0.309 m 由连续解析精确定位、t\* 代入+驻点验证、quartic 求根 200 随机样例通过)/ D(LOS 违反/畸形不再经 retime 拒绝,STATIC 仍硬拒且带日志)全部 PASS;E(M2/C3/D3 数学零改动,由 diff 范围证明)。**

---

## 2. FULL ON 结果(run 20260926_031106_562007,canonical analyzer + 既定计数聚合)

### 2.1 执行质量(moving phase t≤76.5 s,scripts/analyze_run.py + visibility.csv 同法派生)

| 指标 | F117 | **F119** | Δ |
|---|---|---|---|
| TEAM K2_RATIO | 98.5615 | **99.4771** | +0.92 pp |
| TEAM ALL3_RATIO | 90.7585 | **89.1503** | −1.61 pp |
| TEAM BLACKOUT_RATIO | 0 | **0** | = |
| TEAM MEAN_VISIBLE | 2.8932 | **2.8863** | −0.007 |
| K3 loss total | 1.100 s | **0.401 s** | −64% |
| LONGEST K2 loss | 1.100 s | **0.401 s** | −64% |
| ALL3 loss total | 7.036 s | **8.300 s** | +18% |
| LONGEST ALL3 loss | 1.598 s | **1.766 s** | +11% |

### 2.2 前端合同(fleet 计数,3 planner node 求和)

```
ASTAR_REQUIRED          = 74      (静态不可行且到达 repair 阶段的 SIDE)
ASTAR_ACTUALLY_RUN      = 104     (含 rejoin 重试;F117=55)
ASTAR_NO_PATH           = 0
ASTAR_RAW_PATH_SAFE     = 74/74   (每次要求的搜索都成功且逐点静态自由)

GUIDE_REJECTED          = 63      (side 语义 gate —— 见 §3.2)
corridor verdict        : SEED_SAFE=9, SEED_CUT_CORNER_SCP_REPAIRS=1
CORRIDOR_BUILD_SUCCESS  = 10      CORRIDOR_BUILD_FAILED = 0
plane audit guide_margin= 0.05–0.52 m(墙几何健全)
CORRIDOR_SCP_ATTEMPT    = 10      SUCCESS = 0        INFEASIBLE = 10
  失败因: TRUE_CONSTRAINT_INFEASIBILITY=4, DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED=2,
          P_T_TRUST_TOO_SMALL=2, P_TRUST_TOO_SMALL=1, QP_MAX_ITER_EXHAUSTED=1
CONTINUOUS_CORRIDOR_VIOLATION = 0  (无任何"带违反接受")
MINCO_SEED_STATIC_SAFE  = 24      ASTAR_SAFE_BUT_MINCO_UNSAFE = 1(仅遥测)
ASTAR_TO_MINCO_REPAIR_* = 0 / 0   (restore/bulge 链结构性死亡 ✓)
SIDE_INIT_STATIC_COLLISION / _CONFLICT_WINDOW / SEED_COLLISION_EXPLICIT_FAIL:全 run 0 次
```

### 2.3 失败分解(NO_STATIC 513→87,catch-all 分因生效)

```
NO_STATIC_FEASIBLE_SIDE = 87   (F117 513, −83%)
├─ ASTAR_REPAIR_CHAIN_FAILED = 66   (= GUIDE_REJECTED 63 + 其它 3)
├─ NO_VALID_SIDE_SEED        = 21   (所有 offset 的 seed 均无效:纯几何)
└─ (ASTAR_DISABLED = 0)
```

### 2.4 LOS / K3 / M2 / 预算 / 安全

```
LOS_SOFT_QP_ATTEMPTS ≈ 847 (385+188+274, per-drone finals)  SOLVED=100%
LOS_SOFT_SLACK_NONZERO ≈ 655                                LOS_HARD_REJECT_COUNT = 0
[retime-sfc-reject] 带 WARN 的 STATIC 拒绝存在;LOS 拒绝路径 0 条(离线 D + 源码审计)

K3: events created=14 (9+3+2), same-cycle dispatch=10 (6+3+1)
    RECOVERED=5 (ttr 0.377/0.858/1.143/1.345/1.407 s; F117: 3 次 0.587/1.160/1.237)
    关闭因: MARGIN_RECOVERED=4, NO_SIDE_DISPATCH=4, NO_K3_PROGRESS_CANDIDATE, ACTIVATION_NOT_REACHED=1
M2: M2_PREEMPTED=8 (F117 5-6), TRANSACTION_COMMIT=21 / ACTIVATED=21 (26/24)
Lifecycle: SIDE_HARD_VALID=725, HARD_PREFLIGHT_FAILED=87, VALID_NOT_SELECTED=18,
           SELECTED=222, COMMITTED=366, ACTIVATED=10

EXECUTION_DEADLINE(reason 口径)= 188   (F117 136, +38.6% ✗)
MIN_BUDGET_APPLIED            = 78    (F117 304, −74% ✓)
FEASIBLE_INITIALIZER_FALLBACK:EXECUTION_DEADLINE = 185 (兜底率 100%)
TEAM_PT_ATTEMPT_COUNT=256, TARGET_FACING_ASTAR=256, LOS_DISPATCH=32

安全: STATIC_CONTACT=0, DYNAMIC_CONTACT=0, SWARM_VIOLATION=0, PVA_MISMATCH=0,
      UNVALIDATED_EXECUTED=0;STARVATION=2(全部在 t≈137 s 终局,moving phase 之外),
      TERMINAL_HOLD=8(终局心跳,同上)
```

---

## 3. 机制归因(为什么 SCP 10/10 败、为什么 GUIDE_REJECTED=63)

### 3.1 corridor SCP:seed 动力学病态,不是几何错误

trace(candidate_id=6,SIDE_MINUS,24 planes,10 pieces):
- `[repair-time-init]` corridor seed 的 initial_jerk 全 run 最大 **20731**;SCP 迭代 0 实测 `dyn_jerk_violation=654、dyn_acc_violation=18.7、native_cost=1.5e11、grad~1e13`;
- OSQP 每轮 2000 iter `solved_inaccurate`(dua_res≈1e11),QP step 造成 actual jerk 1585 → MODEL_MISMATCH → PT_SHRINK → trust 耗尽 / TRUE_INFEASIBLE;
- 同时这批 QP 正是 EXECUTION_DEADLINE 136→188 的增量来源(SIDE 串行链里最贵的一段)。
- 而 corridor 几何本身全部健全:0 构建失败、guide_margin 0.05–0.52 m、10 条成功 SCP 的 `corridor_violation` 出口检查全 0。
结论:**违反 mandate 预期的不是 corridor,而是"seed 不需要 static-safe"被误推为"seed 不需要数值 sane"。** F117 的 [repair-time-init](initial_jerk=2534)与 F118 §5 早已给出该信号,本轮未在离线检查中覆盖"seed 动力学"维度,是本轮的验证盲区(离线 A–E 均不含动力学)。

### 3.2 GUIDE_REJECTED=63/74:side 语义 gate 是最后一处"拓扑类拒绝"

admission 解耦后 A* 在 BODY/BOTH 冲突上也大量运行;A* 为绕静态树给出的 guide 有 85% 无法通过 `side_semantic_valid`(对冲突障碍的最近点符号 + 60% 加权占优)。该 gate 是 F118 §7 认定的 topology authority,按 mandate §6 的三类拒绝结构它不属于任何一类——几何(A* static-free)已证明安全,语义只应表达"先试哪侧"。本轮未改动它(属于 §7 "path simplification/side 语义保留"的保守读法),FULL 后如实记录:**它与 mandate 的拒绝类别收敛要求冲突,是 NO_STATIC 87 中 66 例的直接来源。**

### 3.3 执行质量的两面

K2/最长丢失大幅改善(99.48%/0.40 s)与 NO_STATIC −83%、MIN_BUDGET −74%、K3 恢复 3→5 次一致——前端供给变稳;ALL3 −1.61 pp 的增量集中在双侧 bulge seed 都静态可行、但第三机观察被遮挡的窗口(与 K3 无关的 pure-visibility 损失),待 corridor 链打通后由更强的绕行供给回收。

---

## 4. mandate 七问

1. **static repair 是否已与 dynamic soft margin 解耦?** 是。A* 资格=`!hard_collision`(唯一保留的真实安全阈值);1.1 m 仅存于 DIRECT_ACCEPT 的 usefulness 判定。证据:ASTAR_REQUIRED=74 全部实际搜索(104 次,含 rejoin),F117 时 90.8% 的静态不可行 SIDE 根本进不了 A*。
2. **Local-SFC 是否真正变成 static-free bounded corridor?** 构造层是:逐 segment 有界凸区域、双向边界扫描、整箱审计(corridor ⊂ static free)、cap 重叠、局部细分、0 构建失败;SCP 内为硬行 + 连续 quintic 出口检查(CONTINUOUS_CORRIDOR_VIOLATION=0)。但 74 条 A* guide 只有 10 条到达 corridor 构建(63 条被 side 语义 gate 拦截,§3.2)。
3. **A* safe 后是否还会因"初始 MINCO 切角"直接死亡?** 不会。切角 seed 只产生 verdict=SEED_CUT_CORNER_SCP_REPAIRS(1 例)与遥测,不再有任何 seed 静态拒绝(SIDE_INIT_* 全 0,REPAIR 链计数恒 0)。当前死亡发生在 trajectory 类(corridor SCP 10/10),根因是 seed 动力学病态(§3.1),属可修复的数值初始化问题,不是切角一票否决。
4. **LOS 是否彻底退出 hard reject?** 是。`validateRetimedLocalSfc` 跳过 LOS(RC4 修复)、enforceCandidateLosPlanesSCP 恒真、SCP 硬行显式排除、retime/preflight/fallback 全链路审计无 LOS 拒绝;run 内 LOS_HARD_REJECT_COUNT=0,soft QP ≈847/847 solved、slack 非零 ≈655 次全部正常进入比较层。离线 D 给出单元证明。
5. **restore/bulge 式补丁链是否已删除?** 是。RESTORE_RAW_WAYPOINTS/BULGE_WAYPOINT_AND_CORRIDOR_PLANE 代码块删除;ASTAR_TO_MINCO_REPAIR_ATTEMPT=0/SUCCESS=0;SIDE_INIT_STATIC_COLLISION、SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION、SEED_COLLISION_EXPLICIT_FAIL 三个标签全 run 0 次出现。
6. **deadline 是否自然下降?** 没有:reason=EXECUTION_DEADLINE 136→188。删除修补链节省的计算被 corridor SCP 的 OSQP(2000 iter solved_inaccurate × 多轮)与 A* 调用量翻倍(55→104)抵消并反超——按 §3.1 修复 seed 数值质量后,该增量应随 SCP 快速收敛而消失。同期 MIN_BUDGET_APPLIED 304→78(−74%)说明重启正反馈环已被打断。
7. **剩余失败是否具有唯一、明确的物理含义?** 是,互斥且各有独立计数:(a) NO_VALID_SIDE_SEED=21(几何:所有 offset seed 无效);(b) GUIDE_REJECTED=63(拓扑:side 语义 gate,mandate 类别外遗留);(c) CORRIDOR_SCP_*=10(轨迹:seed 动力学病态 → trust/OSQP 耗尽);(d) EXECUTION_DEADLINE=188(预算,100% 兜底);(e) K3 NO_SIDE_DISPATCH=4(K3 窗口内无双侧授权)。不再有 catch-all 折叠。

---

## 5. NEXT_STEP(仅记录,另行立项;未在 FULL 后改动 production)

1. **seed 动力学时间膨胀**:corridor seed 生成后按 v/a/j 超差比做一次时长 dilation(durations×=max(1, v/vmax, √(a/amax), ∛(j/jmax))),corridor 时间窗同步;仍是 1 个 seed + 1 条优化链。预期直接修复 SCP 10/10 与 EXECUTION_DEADLINE 增量。
2. **side 语义 gate 的类别归位**:按 mandate §6 将 GUIDE_REJECTED 从硬拒绝改为比较器偏好/尝试顺序(INV4:SIDE topology 只表达 LEFT/RIGHT)。预期回收 66/87 的 NO_STATIC。
3. corridor SCP 的 OSQP 迭代上限与 seed 预热(superbase 化)作为兜底护栏。

---

## 6. 最终字段

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES (catkin build 25/25, 0 warnings)
GIT_DIFF_CHECK: CLEAN

LOCAL_PIPELINE_REFACTORED: YES
PATCH_STYLE_DUPLICATE_AUTHORITY_REMOVED: YES (restore/bulge/seed-gate 链删除)
ASTAR_ADMISSION_DECOUPLED_FROM_DYNAMIC_SOFT_MARGIN: YES (hard-clearance only)
CORRIDOR_BOUNDED_CONVEX_STATIC_FREE_AUDITED: YES (10/10 built, 0 failed)
CORRIDOR_HARD_SCP_CHAIN_ACTIVE: YES (attempt=10; success=0, infeasible=10)
SCP_ACTIVE_SET_PROTOCOL: YES (endpoints+mid+argmax -> t* augmentation)
PLANE_X25_BRUTE_ROW_EXPANSION_REMOVED: YES (诊断保留)
CONTINUOUS_QUINTIC_VIOLATION_CHECK: YES (endpoints + f'(t)=0 real roots)
RESTORE_BULGE_CHAIN_DELETED: YES (REPAIR_ATTEMPT=0 / SUCCESS=0)
SEED_STATIC_REJECTS_DELETED: YES (SIDE_INIT_* / SEED_COLLISION_EXPLICIT_FAIL = 0 次)
VISIBILITY_HARD_REJECT_PRESENT: NO
LOS_OBSERVATION_SOFT_ONLY: YES
RETIME_SFC_LOS_SKIP: YES (Feedback118 RC4 修复)
LO_SOFT_QP_SOLVED_RATE: 100% (≈847/847, slack_nonzero ≈655)
LOS_HARD_REJECT_COUNT: 0
COLLISION_SFC_HARD: YES (STATIC corridor planes only)
NO_STATIC_FEASIBLE_SIDE: 87 (F117 513; 分因: CHAIN_FAILED=66 / NO_VALID_SEED=21)
GUIDE_REJECTED_SEMANTIC_GATE: 63 (mandate 类别外遗留, NEXT_STEP-2)

EXECUTED (moving phase t<=76.5s):
  K2_RATIO: 99.4771        ALL3_RATIO: 89.1503
  K3_LOSS_TOTAL: 0.401s    LONGEST_K2_LOSS: 0.401s
  ALL3_LOSS_TOTAL: 8.300s  LONGEST_ALL3_LOSS: 1.766s
  MEAN_VISIBLE: 2.8863     BLACKOUT_RATIO: 0.0000
K3_RECOVERED: 5 (ttr 0.377/0.858/1.143/1.345/1.407s)  K3_EVENTS: 14
K3_SAME_CYCLE_DISPATCH: 10   M2_PREEMPTED: 8
TRANSACTION_COMMIT: 21       TRANSACTION_ACTIVATED: 21
SIDE_HARD_VALID: 725  HARD_PREFLIGHT_FAILED: 87  SELECTED: 222
SIDE_COMMITTED: 366   ACTIVATED: 10
EXECUTION_DEADLINE: 188 (F117 136)   MIN_BUDGET_APPLIED: 78 (F117 304)
STATIC_CONTACT: 0  DYNAMIC_CONTACT: 0  SWARM_VIOLATION: 0
PVA_MISMATCH: 0    UNVALIDATED_EXECUTED: 0
STARVATION: 2 (终局, moving phase 外)   TERMINAL_HOLD: 8 (终局心跳)

OFFLINE_CHECKS_A_TO_E: PASS (17/17; 抓出并修复 3 个 production bug)
FULL_RUN_COUNT_THIS_TASK: 1
REPEATED_FULL_SIMULATION_USED: NO
SIM_RUN_COMMAND: canonical runner, scenario natural_team_stress_dense_38_targeted_k2_v6.json, --k3-repair on --k3-escalation on
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
