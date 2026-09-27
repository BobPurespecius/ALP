# Feedback 118 — Local planner 全链路新鲜只读审计(不继承任何既往结论)

日期:2026-09-26。工作区:`/home/bob/ALP/egov2_fc65423_constvel`。
性质:**全新、独立、只读**源码审计。不继承 feedback_113–117 的任何结论;把当前 dirty worktree 当作第一次看到,从 production source 重新画 pipeline,再用 run log 对证。未修改任何 production 文件,未编译,未运行仿真。

运行证据主 run:`runs/20260926_001027_482839`(即 F117 的唯一 FULL ON run);历史对照 F113–F117 文本与其引用的 4 个 run。

---

## 0. 一句话回答(根本问题)

**当前系统处于 PATCH_ACCUMULATED 状态。** 多轮修复确实各自消灭了被点名的 label(LOS 硬拒绝→0、SIDE_INIT_STATIC_COLLISION→0),但失败总量没有下降,而是**改名与前移**:同一物理失败先后叫 `LOS_PLANE_SCP_FAILED` → `SIDE_INIT_STATIC_COLLISION` → `SEED_COLLISION_EXPLICIT_FAIL`/`NO_STATIC_FEASIBLE_SIDE`。本轮从源码定位到两个此前所有 feedback 都没有点透的结构根因:

1. **A* 资格被一个与静态搜索无关的动态余量条件(`astar_base_valid` 要求 seed 对全部动态障碍 ≥1.1 m)劫持**——run 内 524 次静态不可行 SIDE 构造中 **476 次(90.8%)根本没进 A***,直接统一记为 `NO_STATIC_FEASIBLE_SIDE`。513 这个数不是"森林里没路",而是"A* 被自己的资格 gate 缴械后的 catch-all"。
2. **A*→MINCO 之间不存在表示层面的 contract**,只有"采样—检查—局部修补—再检查"的循环:49 次 A* 修复中 29 次以 `SEED_COLLISION_EXPLICIT_FAIL` 显式死亡,12 次直接 SAFE——所谓 CHAIN_UNIFIED 是"检查更严了",不是"几何保真了"。

---

## 1. 当前真实执行的控制流(以 if/return 为准,非注释)

```
FSM planning loop (ego_replan_fsm.cpp:1055/1118,rolling)
└─ EGOPlannerManager::reboundReplan (planner_manager.cpp:8468,≈5,200 行)
   ├─ ensureExecutionCoverage (1302):coverage→wall deadline;MIN_BUDGET/current-state-restart
   ├─ STEP1 NOMINAL initializer (8598):
   │    warm-start(accepted_state_cache) || buildStaticFeasibleInitializer
   │    → finelyCheckAndSetConstraintPoints(8713,ERR 即 return false)
   ├─ STEP2 NOMINAL optimizeTrajectory (8801,LBFGS≤200 iter × ≤3 restart + LOS slack QP)
   │    → nominal_result(safe_seed_retention 失败时回退 initializer,8858)
   ├─ raw LOS truth 扫描(9036–9108,static LOS + 每动态障碍圆柱,world-time)
   ├─ current-LOS 判定(9121–9206,无条件,observer=当前 odom)
   ├─ K3 updateLocalK3RecoveryEvent(9307,dispatch 之前,同 cycle 授权)
   ├─ ConflictDescriptor 组装(reason_mask: BODY_SAFETY / LOS_OCCLUSION)
   │    body 证据: inflate 占据采样(9462) + evaluateDynamicRisk(9572)
   │    LOS 证据: raw_static/dynamic_los_current + escalation_grant(K3 window∩witness)
   ├─ task_topology_trigger = body_topology_trigger || spatial_visibility_trigger(9811)
   ├─ 触发时:attempt_side(±preferred_side) —— optimize_side lambda(10111,≈3,000 行)
   │    ① offset backoff 0.7→0.4(步长=map_resolution,10330):
   │        computeInitState(正弦侧向 bulge seed) → trial_risk(动态)
   │        → astar_base 记录[要求 trial_dynamic_valid!](10452–10472)
   │        → finelyCheck 静态预检(10480) → LOS 观察面构建(10544–10755)
   │        → side_dynamic_valid 判定 → DIRECT_ACCEPT / repair_only / 继续 backoff
   │    ② 全部 offset 失败且 !repair_only → A* repair 块(10824–12264):
   │        资格 gate: enable_side_local_astar_repair_ && astar_base_valid && …(10824)
   │        25 点扫描 base 轨迹找静态碰撞区间 → anchor → AstarSearch ×≤3 rejoin
   │        → raw_path_usable(占据+side 语义) → simplify(≤3 边 LOS 捷径+20°拐点+2 重插)
   │        → Local-SFC boundary scan(逐 guide) → MINCO seed 重建 ×≤3
   │          ( finelyCheck+冲突窗+全 scope+64 点坏段定位;restore-raw / bulge 修补 )
   │        → astar_accept = 5 条件 AND(1205–12108,含 repaired_static_free)
   │    ③ !astar_repair_applied → log_side_preinit_failure("NO_STATIC_FEASIBLE_SIDE") return(12265)
   │    ④ seed 再检:finelyCheck AGAIN(12376)+ 冲突窗采样 AGAIN(12412)
   │       → SIDE_INIT_STATIC_COLLISION / SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION
   │    ⑤ feasible-initializer contract(动力学格+static+local-SFC+risk,12590–12622)
   │    ⑥ optimizeTrajectory(12656,LBFGS+LOS slack QP)
   │       失败且有 ABSOLUTE_SAFE initializer → fallback(12700–12783)
   │    ⑦ classify_candidate(8912: dynamics+static+swarm+risk→ABSOLUTE_SAFE/INVALID)
   ├─ classify_candidate(plus/minus)(13283–13284)+ visibility 三份报告(13286–13296)
   ├─ 构造期选择:NOMINAL-safe > plus_good > minus_good(13386–13391)
   └─ finalizeCapturedCandidates(local_sets,…)(13651 → 1461)
        ├─ 冻结 batch activation;逐候选:
        │   prepareLocalHandoff(reanchor,改多项式!) → SHORT_STATIONARY gate(1620)
        │   → validateRetimedLocalSfc(1631) && checkActiveHandoff && validateExecutionTrajectory
        │   → geometry/visibility/K3-window 重算 → K2 gate → 比较器(K2→C3→D3→side偏好
        │     →target_side偏好→visibility→geometry/cost)(1717–1828)
        ├─ best → finalizeTopologyCandidate(1940 → 6440):
        │   prepareLocalHandoff AGAIN → validateRetimedLocalSfc AGAIN(6505)
        │   → dynamics/static/swarm/risk 全部 AGAIN(6513–6524)→ ABSOLUTE_SAFE 复核(6546)
        │   → checkActiveHandoff AGAIN(6577) → setLocalTrajFromOpt → COMMITTED
        └─ 无 best → validatePreviousRemainingTrajectory → RETAINED_PREVIOUS / FAILED
```

N/L/R 的语义(源码事实):
- **preferred_side**(9943–9978):由标称轨迹在冲突点的切向 × ẑ 的左右、障碍横向偏移的逃逸方向与 (target−start)×ẑ 的对齐符号决定——只是**尝试顺序 hint**(13276–13280 两侧都解)。
- **SIDE seed**(buildFreshMovingInitializer,817–824):`start→local_target` 直线 + `side·0.7·sin(πu)·side_dir` **单拱侧向 bulge**。L/R 表达的是一个**局部偏移方向与幅值**,不是 homotopy 类、不是路径类、不是半空间内的自由规划。
- **LOS 观察面**(10544–10681):`target→blocker` 轴的切锥半空间,side_sign ∈ {+1,−1},事件窗口 = 真实遮挡区间,world_time_anchored。

---

## 2. Candidate 死亡点全表(SIDE 从创建到执行)

标 H=硬安全 / Q=质量 / T=拓扑 / B=预算;FALLBACK 列指该 gate 失败后同 batch 内还能去哪。

| # | STAGE | FILE:LINE(planner_manager.cpp 除注明) | 函数/lambda | 条件 | REASON | 类 | FALLBACK |
|---|---|---|---|---|---|---|---|
| 1 | batch 入口 | 8517 | reboundReplan | !optional && !mandatory | (整批放弃) | B | 无 |
| 2 | NOMINAL init | 8667 | reboundReplan | seed_ok=false | return false | H(动力) | 无(整批) |
| 3 | NOMINAL init | 8713 | reboundReplan | finelyCheck ERR | return false | H | 无 |
| 4 | NOMINAL opt | 5074/5033*(opt) | optimizeTrajectory | budget/`EXECUTION_DEADLINE` | flag_success=false | B | safe_seed_retention→initializer(8858) |
| 5 | NOMINAL 精查 | 5191*(opt) | finelyCheck after LBFGS | still occupied | restart≤3 后失败 | H | #4 同 |
| 6 | SIDE init | 10374 | optimize_side | computeInitState false | SIDE_INIT_FROM_PREVIOUS_FAILED→重试;INVALID_SIDE_INIT | H(动力) | 下一 offset |
| 7 | SIDE 时长 | 10392–10398 | 〃 | pieces≤0/时长≤0 | INVALID_SIDE_INIT_DURATION | H | 下一 offset |
| 8 | SIDE 时长窗 | 10400–10445 | 〃 | duration≤window_start | WINDOW_NO_OVERLAP | T | duration fallback→INVALID_SIDE_INIT_FALLBACK |
| 9 | SIDE 静态 | 10483–10537 | 〃 | finelyCheck≠OBS_FREE | STATIC_COLLISION → continue | H | 下一 offset;observation→path-frame 重试一次 |
| 10 | SIDE 动态 | 10764–10793 | 〃 | BODY: min_dist≥1.1m 不满足 / LOS: hard_collision | DYNAMIC_SAFETY_CLEARANCE 等 | H(余量) | repair_only 进优化器(无执行权) |
| 11 | **A\* 资格** | **10824** | 〃 | **!astar_base_valid** 等 | **不进 A\*→NO_STATIC_FEASIBLE_SIDE** | **B/T(错位)** | **无** |
| 12 | A\* 预算 | 11088 | 〃 | side_work_allowed false | EXECUTION_COVERAGE_DEADLINE | B | 无→#14 |
| 13 | A\* 搜索 | 11116–11168 | 〃 | INIT/SEARCH_ERR、anchors 占据 | ASTAR_SEARCH_FAILED | H | rejoin ×≤3 |
| 14 | A\* 收尾 | 12265–12269 | 〃 | !astar_repair_applied | **NO_STATIC_FEASIBLE_SIDE(catch-all)** | 混合 | return(SIDE 死) |
| 15 | raw path | 11058–11067 | raw_path_usable | 占据体素或 side 语义违反 | 换 rejoin/first_success→#16 | H/T | rejoin |
| 16 | 简化后 | 11370–11526 | repaired_side_valid | path_static_free/side/SFC build | GUIDE_REJECTED | H/T | 无→#14 |
| 17 | SFC build | 11379–11523 | Local-SFC scan | 坏平面/窗口退化/deadline | LOCAL_SFC_BUILD_FAILED | H | 无→#14 |
| 18 | seed 重建 | 11636–12062 | seed rebuild ×≤3 | 重建后仍占据 | SEED_COLLISION_EXPLICIT_FAIL | H | 2 次修补后显式失败→#14 |
| 19 | seed 再检 | 12376–12404 | optimize_side | finelyCheck AGAIN | SIDE_INIT_STATIC_COLLISION | H | 无(SIDE 死) |
| 20 | 窗口再检 | 12411–12444 | 〃 | 冲突窗占据采样 | SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION | H | 无 |
| 21 | MINCO | 12656 | optimizeTrajectory | deadline/仍然占据/LOS QP 硬行 | EXECUTION_DEADLINE 等 | B/H | feasible-initializer fallback(需 ABSOLUTE_SAFE) |
| 22 | fallback 复核 | 12727–12742 | 〃 | dynamics/static/SFC/risk | fallback_usable=false | H | 无(SIDE 死) |
| 23 | 分类 | 8912–8959 | classify_candidate | dynamics/static/swarm/risk | safety_class=INVALID(→SIDE_HARD_PREFLIGHT_FAILED) | H | 上级 candidate |
| 24 | finalize 准备 | 1595–1602 | finalizeCapturedCandidates | prepareLocalHandoff false | moving-rehead-reject(MOVING_SUFFIX_EXHAUSTED 等) | B/H | 其他 candidate |
| 25 | 驻留 gate | 1620–1625 | 〃 | duration<coverage 且 <0.2 m/s | SHORT_STATIONARY_HYPOTHESIS | Q | 其他 candidate |
| 26 | **retime SFC** | **1631 / 6505** | **validateRetimedLocalSfc(2287)** | **全部平面(含 LOS_OBSERVATION_SIDE)采样违反** | **safe=0 reason=空(静默)** | **H/LOS 残留** | 其他 candidate |
| 27 | handoff | 1632 / 6577 | checkActiveHandoff(2177) | PVA 残差>容差/激活越过 validated_end | safe=0 / PREDECESSOR_VALIDATED_END_EXCEEDED | H | 其他 candidate |
| 28 | 执行校验 | 1633 | validateExecutionTrajectory(2308) | dynamics/static/dynamic-hard/swarm | DYNAMICS_FAIL 等 41+19 次 | H | 其他 candidate |
| 29 | K2 gate | 1724–1731 | 〃 | k3_window.k2 < baseline | K2_GATE_REJECT_CANDIDATE | Q(K3) | NOMINAL |
| 30 | K3 无基线 | 1713–1721 | 〃 | baseline 无效且非 NOMINAL | K2_BASELINE_UNAVAILABLE | Q(K3) | NOMINAL |
| 31 | 比较 | 1717–1828 | comparator | 输给其他 candidate | VALID_NOT_SELECTED | Q | — |
| 32 | commit 复核 | 6505–6568 | finalizeTopologyCandidate | retime-SFC/dyn/static/swarm/risk AGAIN | NOT_ABSOLUTE_SAFE 等 | H | KEEP_PREVIOUS_SAFE |
| 33 | commit 写入 | 6586 | setLocalTrajFromOpt | points-to-check 失败 | POINTS_TO_CHECK_FAILED | H | KEEP_PREVIOUS_SAFE |
| 34 | 兜底 | 2010–2029 | finalize 尾部 | previous 也不安全 | FAILED(整批) | H | 下一 cycle |
| 35 | 持久侧 | 9854–9867 | preserve_active_observation_side | 同 blocker 已有 accepted side | NOMINAL 置 INVALID(反向 gate) | T | 保留旧 SIDE |

\* 为 poly_traj_optimizer.cpp 行号。

**同一"静态可行"被独立判断的位置(单条 A*-repair SIDE 最多)**:①每 offset finelyCheck 预检;②A* raw 逐点占据;③simplify 的 segment_free 逐段;④Local-SFC boundary scan;⑤seed 重建 finelyCheck+冲突窗+全 scope(+64 点坏段,×≤3 次);⑥SIDE_INIT finelyCheck AGAIN;⑦SIDE_INIT 冲突窗采样 AGAIN;⑧feasible-initializer checkTrajectoryStaticSafety;⑨classify checkTrajectoryStaticSafety;⑩finalize validateExecutionTrajectory;⑪finalizeTopologyCandidate AGAIN。**= 11 处独立实现,互不共享状态,判定窗口还不同(2/3 前缀 vs 冲突窗 vs 全时长)。**

**同一"visibility/LOS"仍拥有的 authority(12 处)**:raw LOS truth 扫描 / current-LOS 判定 / 方向 J_vis cost / LOS 观察面(frontend) / enforceCandidateLosPlanesSCP(soft) / side_semantic_valid(A* 语义) / evaluateCandidateVisibility(三机报告) / fillCandidateBinaryVisibility / evaluateLocalK3WindowMetrics(C3/K2/D3) / K2 gate / visibility 比较器 / Team visibilitySampleAt。其中 K3 系已统一到 LocalVisibilitySample 原语,但 J_vis、三机报告、binary trace、side 语义仍是各自独立采样。

---

## 3. NO_STATIC_FEASIBLE_SIDE 源树(F117 run 实测分解)

对 `runs/20260926_001027_482839/roslaunch_stdout.log` 的逐条归因(513 次 preinit 计数;524 条 side-static-aware-failure):

```
NO_STATIC_FEASIBLE_SIDE (513)
├─ A. A* 从未被调用(astar_base_valid=false)────────── ≈471 次(91.8%)
│    条件链:全部 4 个 offset 的 seed 都不满足 trial_dynamic_valid
│    (planner_manager.cpp:10452–10472)
│    BODY/BOTH 冲突 → 要求 seed 对**全部**动态障碍 min_distance ≥ 1.1 m
│    (getMovingObjClearance = optimization/moving_obj_clearance, 默认 1.1)
│    实测旁证:155 个"静态自由但动态不足"的 repair-only seed 的
│    seed_dynamic_clearance 100% < 1.1 m(78 个在 0.5–0.8,69 个在 0.8–1.1)
│    按触发类型分布(BODY=242, BOTH=258, LOS=22, N=2)——95% 是
│    带 1.1 m 强条件的 BODY/BOTH
├─ B. A* 被调用但修复链失败 ────────────────────────── ≈42 次(8.2%)
│    B1 搜索/anchor 失败、rejoin 全败(ASTAR_SEARCH_FAILED)
│    B2 raw path 不可用(占据或 side 语义违反)→ GUIDE_REJECTED=5
│    B3 Local-SFC build 失败
│    B4 MINCO seed 重建仍切角 → SEED_COLLISION_EXPLICIT_FAIL=29
│       (SEED_SAFE=12;ASTAR_SAFE_BUT_MINCO_UNSAFE=24)
├─ C. A* 被调用但 base 上没有静态碰撞区间 ───────────── ≈2 次
│    (static_collision_found=0 → ASTAR_NO_COLLISION_INTERVAL 日志)
└─ D. deadline 打断 A*/SFC/seed(1188/11193 处 side_work_allowed)→ 同一 catch-all
   [当前 telemetry 无法区分 A/B/C/D——只有一个字符串]
```

关键结论:**513 ≈ "A* 资格 gate 拒绝了 91.8% 的静态修复请求" + "A*→MINCO 表示失真杀掉其余"。不是森林无路。** 同一 run 中 DIRECT_ACCEPT=549、REPAIR_ONLY=155、ASTAR_REPAIR_ACCEPT=24:凡是 seed 能静态自由的场景,SIDE 正常产出——失败集中在"侧向 bulge 撞树"这一种几何,而这恰是 A* 的本职。

**telemetry 无法区分的来源**(都折叠进同一 reason):A* 未资格 / 搜索失败 / anchor 占据 / 路径语义违反 / SFC build 失败 / seed 切角 / deadline。`[side-construction-causes]` 的 cause map 只有 5 个字符串键,且 `ASTAR_NO_COLLISION_INTERVAL` 这类误导性日志(明明没搜过)会落在 NO_STATIC 计数里。

---

## 4. A* 的真实职责与调用资格(问题 4)

A. **三者混合,但三处实现互不相认**:
 - optimizer 内 `a_star_`(poly_traj_optimizer.cpp:5483):`finelyCheckAndSetConstraintPoints` 撞 obstacle segment 后**变形控制点**(EGO 原生机制);
 - planner_manager 的 `side_a_star_`(10824 块):SIDE seed 的**静态 body repair + 绕行路径搜索**;
 - `buildStaticFeasibleInitializer`(NOMINAL 的 FRESH_ASTAR_TARGET_SIDE 初始化器)。
 同一 run 里两套 A* 实例、三种语义(搜路/修 seed/修控制点),共享同一个 inflated grid map,但没有共享的走廊或 contract。

B. **SIDE 明明需要绕行而 A* 不被调用的情形(实测占 91.8%)**:seed 在所有 offset 上都静态撞树,但没有任何 offset 满足"对全部动态障碍 ≥1.1 m"。DENSE 场景动态障碍密集,侧向 0.7 m 的 bulge seed 几乎必然与**某个别的**动态障碍距离 <1.1 m → astar_base_valid 恒 false → 直接判 NO_STATIC_FEASIBLE_SIDE。

C. **A* eligibility gates 全集**(10824):`enable_side_local_astar_repair_`(参数) ∧ `astar_base_valid`(动态有用性!) ∧ `grid_map_` ∧ `side_a_star_` ∧ `astar_base_traj.getPieceNum()>0`;进入后还有 anchor 自由(11104)、side_work_allowed(11088)。

D. **"A* base dynamic-valid"是什么**:不是可行性,是**优化目标语义**——希望修复后的路径仍保持对冲突障碍的避让增益(10381–10472 注释自认)。但 BODY 模式的判据是"对全部动态障碍 ≥1.1 m preferred clearance",这不是安全事实(最终 preflight 只要求 hard clearance≈0.4–0.9 m),而是把**软余量当成了硬资格**。

E. **结构性错误成立**:static SIDE seed 不可行 → 本应 A* 搜索 → 一个与 static path search 无直接关系的 dynamic-margin 条件使 A* 无资格 → 统一记 NO_STATIC_FEASIBLE_SIDE。**这是结构性问题,不是参数问题**(把 1.1 调小只是缓解,资格条件本身错位;且该条件同时决定 side_dynamic_valid 的 repair_only 分支,两处语义纠缠)。

---

## 5. A*→simplify→SFC→MINCO 是否构成 contract(问题 5)

**判定:不存在数学/表示意义上的 contract;存在的是"采样—验证—有界修补—再验证"的工程循环。**

- `raw safe ⇒ simplified safe`:简化每段都做 segment_free 复查(11238–11264),捷径跨度被钳到 3 条 raw 边,拐点≥20°保留,碰撞带重插 2 点——**逐段成立,但这是检查不是保证**;随后 MINCO 用 3 阶多项式过这些顶点,`raw safe ⇒ MINCO seed safe` **不成立**(F115 CAUSE 2、F116 30/44、本轮 SEED_COLLISION_EXPLICIT_FAIL=29/46 verdicts)。
- corridor:每 guide 点一条**单侧** half-space(`normal·(p−q)≥0`),`active_start/end` 只按弧长比例铺在 [collision_start, collision_end]——**segment-local、无宽度语义、不覆盖 junction 前后的长段**(这正是 24 例 prefix/junction 切角残余的几何原因:碰撞区间外的 MINCO 段没有任何平面约束,多项式在区间边界两侧自由弯)。
- corridor 用于 initializer 吗?**不**。MINCO seed 由 guide 折线 + 弧长时间表直接 generate(11796–11804),**不经 corridor 投影**;corridor 平面只在 LBFGS 之后的 `evaluateCandidateLocalSfcMaxViolation`(诊断)与(默认关闭的)hard SCP 中使用。对默认配置,corridor 的实际消费者只有 seed 重建的 bulge 修补和 finalize 的 retime 检查。
- 修复为何只有 2 次尝试/1 次成功:restore_raw_waypoints 受 `merged<24` 点与"必须比原路径多点"限制(11633),bulge 只处理**单个** first_bad_segment(11978)——prefix 长段多角落时两次都不够 → 显式失败。这是有意为之的止损(F117 §2),但暴露了底层表示问题:折线顶点 + 自由多项式之间没有"保真算子"。
- **A*→MINCO 问题的定性:6=多者组合,主序为 (5) contract 不存在 → (2) corridor construction(segment-local)→ (3) simplification(切角擦除)→ (4) optimizer initialization(无投影)**;纯 representation(1)是根,但单改表示不动 corridor 覆盖也不够。

---

## 6. SIDE topology 语义是否正确(问题 6)

L/R = ±1 只实例化三件事:seed bulge 方向、LOS 观察半空间 side_sign、A* 路径的 side 语义检查(相对标称切向)。它是**局部方向 + 种子偏移**,被后续代码当"绕行拓扑"使用——概念错位成立。

森林中 target→blocker A→SIDE_RIGHT 之后还有 B/C 两棵树时,源码行为是:
- A* repair 在同一 side 内自由搜路,**可以**绕 B/C(side_semantic_valid 只约束靠近冲突障碍的加权点,11003–11033);
- 但 A* 常驻资格被 §4 的 dynamic gate 缴械 → 实际是 **(C) seed 失败**(91.8%);
- offset backoff(0.7→0.4)会把 bulge **缩小回冲突障碍方向**,对"侧向另有树"有效、对本 blocker 有害——backoff 与 topology 意图方向相反;
- MINCO 阶段无从"承担"拓扑(它只有 soft side/side_region/preserve 项)。
没有 (A) "A* 自动在同一 topology 内解决"的稳定通道,也没有 (B) 的显式重触发——每个 cycle 重新掷骰子(K3 的 last_committed_side 轻偏好只是 tie-break)。

---

## 7. hard / soft authority 是否统一(问题 7)

**HARD SAFETY(真正可判 INVALID 的)**:静态 body(checkTrajectoryStaticSafety,前缀窗)、动态 body(evaluateDynamicRisk hard_collision,hard_clearance=0.384+半径/2)、swarm(evaluateTrajectorySwarmConflict,0.58 m)、v/a/j(checkTrajectoryDynamics,1+tol)、STATIC_COLLISION_CORRIDOR 平面(validateRetimedLocalSfc / evaluateCandidateLocalSfcMaxViolation)、revision/handoff(checkActiveHandoff、current-revision)。

**VISIBILITY QUALITY**:J_vis、三机报告、C3/K2/D3、FOV/range、LOS 观察面(soft slack QP,恒 return true)、K2 gate、比较器。**除下面一条残留外,均已不再产生 return false / success=false / INVALID。**

**残留的隐蔽 visibility reject 路径(本轮新发现,与 INVARIANT 1 矛盾)**:
`validateRetimedLocalSfc`(2287–2306)**不区分平面 source**。SIDE 候选的 `local_sfc_planes` 按设计携带 LOS_OBSERVATION_SIDE 平面(12891–12963 显式恢复/保留),slack>0 的候选(本轮 203/594 次)在 classify 存活后,于 finalizeCapturedCandidates:1631 与 finalizeTopologyCandidate:6505 被该函数再次采样——违反即 `return false`,候选静默死亡(reason 字符串为空:run 内 **110 条 safe=0 reason=空**)。该函数还**没有 world_time_anchored 处理**,按 duration 比例缩放 LOS 平面窗口,与 prepareLocalHandoff 的世界时间锚定语义直接矛盾。**结论:visibility 仍通过 validateRetimedLocalSfc 这一条隐蔽路径影响 candidate 生死;`LOS_HARD_REJECT_COUNT=0` 只统计了 enforceCandidateLosPlanesSCP 一处。**

**TOPOLOGY**:side sign / observation side / guide 同侧 / K3 side 偏好 / target_side_static_topology 比较偏好——均为选择/偏好,不判生死,符合设计(除 #35 反向把 NOMINAL 置 INVALID 属于持久化语义,可接受但属第二套 topology authority)。

---

## 8. fallback 清单:是否已成"修补式 planner"(问题 8)

| # | FALLBACK | TRIGGER | AUTHORITY | OUTPUT | 可被谁覆盖 | 下一级 |
|---|---|---|---|---|---|---|
| 1 | warm-start(accepted cache) | NOMINAL/SIDE 首试 | 数值初始化 | initMJO | 无 | fresh init |
| 2 | safe_seed_retention | NOMINAL LBFGS 失败 | initializer 即候选 | nominal_result.success=true | 无 | preflight |
| 3 | SIDE duration fallback | seed 时长<冲突窗 | 重排时标 | 新 seed | 无 | 继续试 offset |
| 4 | observation→path-frame | 观察 frame seed 撞静态 | 换 frame 重建 | 新 seed | 无 | 继续 |
| 5 | offset backoff ×≤4 | 静态撞/动态不足 | 缩幅 bulge | 新 seed | 无 | A* 或失败 |
| 6 | repair_only | 静态自由但动态不足 | 无执行权进优化器 | SIDE seed | preflight | — |
| 7 | A* rejoin ×≤3 | 首 rejoins 不可用 | 下游重连 | raw path | 无 | first_success 保留 |
| 8 | first_success_path | 全 rejoin 不可用 | 诊断保留→显式拒绝 | — | — | GUIDE_REJECTED |
| 9 | seed 重建 ×2(restore-raw/bulge) | 重建 seed 撞角 | 局部修补 | 新 seed | 无 | 显式失败 |
| 10 | SIDE feasible-initializer fallback | MINCO 失败/EXECUTION_DEADLINE | ABSOLUTE_SAFE 契约 | result=initializer | 无 | classify |
| 11 | NOMINAL feasible fallback(#2) | 同上(NOMINAL) | 同上 | 同上 | 无 | preflight |
| 12 | preserve_active_observation_side | 同 blocker 已有 side | 反向:NOMINAL→INVALID | 保留旧 SIDE | 新严格更优 | — |
| 13 | SIDE_BOTH_FAILED→NOMINAL | 双侧构造死 | NOMINAL 兜底 | selected=NOMINAL | preflight | KEEP_PREVIOUS |
| 14 | KEEP_PREVIOUS_SAFE / PERSISTENCE | 全部候选死 | 旧 suffix 复验 | RETAINED_PREVIOUS | 无 | 下一 cycle |
| 15 | current-state-restart + MIN_BUDGET | handoff 逾期 | 里程计重启 | restart batch | 无 | — |
| 16 | EXECUTION_DEADLINE initializer 恢复(opt 内) | 求解超时 | 恢复初值 | partial 拒绝 | 无 | #10 |
| 17 | Team speculative solution | Team commit | 并行权威(有让位序) | pending_team_trajectory | Local 安全覆盖 | — |
| 18 | terminal hold(FSM) | 到点 | 驻留 | — | — | — |

判定:**B——多套机制修同一失败,且成串**:静态撞 → backoff(5)→ A*(7/8/9)→ MINCO deadline(16)→ initializer fallback(10)→ classify 死 → KEEP_PREVIOUS(14)→ 下一 cycle 重来。同一物理失败最多被 4–5 层机制先后处理,每层有自己的成功口径(如 #10 要求 ABSOLUTE_SAFE 而 preflight 只要求 hard-safe;#6 无执行权但仍消耗优化与检查)。层间无共享状态、无统一"本次修复已尽力"的出口。

---

## 9. EXECUTION_DEADLINE / MIN_BUDGET 为什么仍高(问题 9)

每个 risk-triggered rolling cycle 的**串行**工作量(全部在同一线程同一 deadline 内):
1. NOMINAL:seed(+可能内部 A*)+ finelyCheck 全采样 + LBFGS(≤200 iter×≤3 restart,每次评估含逐控制点方向 visibility 采样:static-LOS 射线+动态圆柱+FOV,6599 处才每 64 样本一个 checkpoint)+ 收尾 finelyCheck;
2. raw LOS 扫描(≥20 帧 × (static LOS + N 对象圆柱))+ static body 扫描 + classify(NOMINAL):dynamics+static+swarm+risk;
3. PLUS:≤4 offset × (全轨迹动态 risk + 全轨迹静态预检) [+LOS 面构建+reached-side 扫描] → A*≤3 次 → 逐点/逐段/SFC/seed×≤3(每次 64 点定位+finelyCheck)→ SIDE_INIT 再检 ×2 → 动力学格 ×2 → LBFGS + LOS slack QP(≤6 迭代 OSQP)→ post 风险/区域/诊断;
4. MINUS:同 3 整套;
5. finalize:≤3 候选 × (reanchor 改多项式 + retime-SFC 采样 + handoff + dynamics+static+risk+swarm + geometry + 三机 visibility + binary trace + K3 窗口) + 比较器;
6. commit:swarm×2 + retime-SFC + dynamics+static+risk + geometry + visibility AGAIN + setLocalTrajFromOpt。

动态 risk 全轨迹评估单 cycle ≥8 次;静态全轨迹检查 ≥10 次;三份 MINCO 求解。wall 预算却由 coverage 剩余决定(压力下 0.3–0.5 s,加 0.05 s 不可中断预留)。**结构上必然周期性爆预算**:第 2/3 个 candidate 的 LBFGS 一超时,executionCheckpoint 抛异常整候选作废(136 次),全部由 initializer fallback 兜住;coverage 持续被吃 → handoff 逾期 → current-state-restart(304 次 MIN_BUDGET)→ 更小预算 → 更容易 deadline:**正反馈环**。

可结构性减少的重复(不动阈值):
- N/L/R 三解串行 → **时间上分相**(NOMINAL 先提交,L/R 移交下一 cycle 或后台线程;数据结构已支持 joint_seed 异步导出);
- 统一一次静态/动态采样管线,所有 gate 消费同一份 witness(见 §12 MERGE);
- 把 per-offset 的全轨迹动态 risk 换成冲突窗局部采样;
- LBFGS 的方向 visibility 代价只在 risk 邻域激活(已有 witness 窗口,现在全程算)。

---

## 10. Feedback 113–117 对证:failure renaming 链(问题 10)

| 轮次 | 被消灭的 label | 同一物理失败的新去处 |
|---|---|---|
| F116 | `LOS_PLANE_SCP_FAILED`(F115:62)→ **0** | LOS violation 仍经 validateRetimedLocalSfc 在 finalize 静默拒绝(§7 残留);soft slack 候选进入比较层 |
| F117 | `SIDE_INIT_STATIC_COLLISION`(F116:28)→ **0** | 撞角 seed 前移为 `SEED_COLLISION_EXPLICIT_FAIL`(29)/`NO_STATIC_FEASIBLE_SIDE`(197→**513**,因吸收+scene 方差) |
| F117 | F115 的"56/108 误报 SIDE_BOTH_FAILED" | 改为 construction 事实口径;`SIDE_BOTH_FAILED_NO_SUCCESSOR_COUNT=38` 仍在 |
| 恒真 | `EXECUTION_DEADLINE` 171→136 | 全部由 initializer fallback 兜底(计数含义=求解被取消,非不安全) |

计数器口径问题(tele bug 级):
- `SIDE_COMMITTED=233 > SIDE_SELECTED=167`:两个计数器统计**不同阶段的不同总体**(构造期 count_side_selection vs K3 progress selection vs commit),不构成单调漏斗;
- `SIDE_ACTIVATED=8` 只是 K3 激活确认口径,不是完整激活漏斗;
- `side_failure_cause_count_` 只有 5 个字符串键,§3 的 A/B/C/D 全部折叠;
- `[side-astar-repair] result=ASTAR_NO_COLLISION_INTERVAL` 在"没资格进 A*"时不会打,而 astar_base 无效时**静默**——外界只能看到 NO_STATIC。
- "某计数=0"≠机制解决的两个实例:SIDE_INIT_STATIC_COLLISION=0 同时吸收进了 NO_STATIC(F117 自己在 NEXT_STEP 里承认);LOS_HARD_REJECT=0 而 validateRetimedLocalSfc 的 LOS 采样拒绝无任何计数。

---

## 11. 架构判定(问题 11)

```
ARCHITECTURE_STATUS = PATCH_ACCUMULATED
```

依据:
- 重复 authority:静态可行 11 处、visibility 12 处、topology 5–6 处(§2/§7);
- candidate gate ≥35 个(§2 表);
- fallback 18 套,多套串联修同一失败(§8);
- feasibility 定义不唯一:同一候选在不同阶段面对 2/3 前缀 / 冲突窗 / 全 scope / retime 缩放四种窗口与两种阈值(hard≈0.4–0.9 m vs preferred 1.1 m);
- A*/SFC/MINCO contract:不存在(§5);
- topology 与 geometry 职责错位(§6);
- visibility 仍有一条隐蔽 safety 路径(§7);
- runtime 复杂度不可解释(N/L/R 串行全跑,§9);
- failure reason 无唯一物理意义(§3/§10)。

不是 STRUCTURALLY_CONFLICTED:各层之间仍有清晰的优先序与让位协议(M2>K3、Team>Local quality、hard>quality),没有两套硬安全互相打架;骨架(MINCO+LBFGS、inflated grid、A*、lifecycle)本身是成熟且安全的。**病在 Local 的"前端构造层"(seed/SFC/repair/gate 丛林),不在优化器或执行链。**

---

## 12. 根因树

```
ROOT PROBLEM:rolling cycle 反复以 NO_STATIC_FEASIBLE_SIDE / EXECUTION_DEADLINE
             丢候选,修复一直在搬动 label 而非消除物理原因
│
├─ RC1(结构):A* 资格与动态软余量耦合 —— 静态修复器在最需要时被缴械
│  ├─ 源证:planner_manager.cpp:10452–10472(trial_dynamic_valid→astar_base),
│  │        10824(A* 资格),12265(catch-all)
│  ├─ run 证:476/524(90.8%)静态不可行构造未进 A*;BODY/BOTH 占失败 95%;
│  │        155/155 repair-only seed 全部 <1.1 m
│  └─ 影响:NO_STATIC_FEASIBLE_SIDE 513(主)、SIDE 供给不稳、K3 恢复 ttr 波动
│
├─ RC2(设计债):A*→MINCO 无表示级 contract;corridor segment-local、无保真算子
│  ├─ 源证:11636–12062(seed 检查-修补循环),11389–11523(单侧平面、弧长窗),
│  │        corridor 不进 initializer(仅诊断/SCP/修补)
│  ├─ run 证:ASTAR_SAFE_BUT_MINCO_UNSAFE=24;SEED_COLLISION_EXPLICIT_FAIL=29;
│  │        REPAIR 8 试 1 成;prefix/junction 切角 24 例
│  └─ 影响:SIDE 即使有 A* 也近半数显式死亡;修复复杂度(×3 重建)推高 runtime
│
├─ RC3(结构):单线程 batch 串行做 3×MINCO + 全链重验,与 rolling 预算天然冲突
│  ├─ 源证:reboundReplan 主干 + finalize 两轮全量复验(1631/6505)+ LBFGS 内
│  │        visibility 全程采样(6599 checkpoint 稀疏)
│  ├─ run 证:EXECUTION_DEADLINE=136(100% 兜底)、MIN_BUDGET_APPLIED=304、
│  │        FEASIBLE_INITIALIZER_FALLBACK:EXECUTION_DEADLINE=134
│  └─ 影响:死循环式重启倾向(预算收缩→更多 deadline→更多 restart)
│
├─ RC4(实现 bug):validateRetimedLocalSfc 不过滤 LOS 平面 + 无 world_time 锚定
│  ├─ 源证:2287–2306 vs LocalSfcPlane.world_time_anchored 语义(59–69)与
│  │        12891–12963(保留 LOS 平面的三处代码)
│  ├─ run 证:safe=0 reason=空 110 条(该 gate 无日志,只能总量佐证)
│  └─ 影响:INVARIANT 1 名义成立实际有旁路;候选在最后一步静默死亡
│
├─ 设计债(次级):SIDE=局部 bulge 被当 homotopy(§6);backoff 与拓扑意图反向;
│   feasibility 窗口四种口径并存;N/L/R 选择 vs commit 计数口径混用
│
└─ 次级症状(非根因):NO_STATIC_FEASIBLE_SIDE、EXECUTION_DEADLINE、
   MIN_BUDGET_APPLIED、SEED_COLLISION_EXPLICIT_FAIL、GUIDE_REJECTED、
   SIDE_VALID_NOT_SELECTED 抖动
```

分类:RC1/RC3=ROOT CAUSE;RC2/SIDE 语义=DESIGN DEBT;RC4=IMPLEMENTATION BUG;§10 的计数口径=TELEMETRY BUG;真实无路(密林死角)=EXPECTED PHYSICAL INFEASIBILITY(存在但被 513 高估了 ~90%)。

---

## 13. 两个方案(不实施)

### 方案 1:最小结构修正(2–4 处改动,不动架构)
范围:
1. **解开 A* 资格与动态余量的耦合**:astar_base 选择改为"任一 offset 的 seed 静态撞且 `!hard_collision` 即可"(hard-clearance 口径),≥1.1 m 仅作为多 base 时择优键;A* 修复后由 repaired_risk 与最终 preflight 判安全。预计直接命中 513 的 ~90%。
2. **RC4 修复**:validateRetimedLocalSfc 跳过 LOS_OBSERVATION_SIDE 平面(与 evaluateCandidateLocalSfcMaxViolation 同口径),并为 world_time_anchored 平面加锚定换算;给该 gate 增加 reason 字符串。
3. **NO_STATIC 计数拆键**:在 12265 catch-all 处按 (A 无资格/B 搜索失败/C 路径不可用/D SFC 失败/E seed 失败/F deadline) 记 `side_failure_cause_count_` 细分键。
4. (可选)corridor active 区间外延一个 junction 段:把平面铺满 [collision_start−Δ, collision_end+Δ],Δ=相邻段时长。
风险:低。1 的风险=修复后的路径可能丢失避让增益(由既有 repaired_dynamic_valid 遥测与 preflight 兜住);2 需回归 K3 恢复口径。
能解决:NO_STATIC 大头、隐藏 LOS 拒绝、telemetry 盲区、部分 seed 切角。
不能解决:EXECUTION_DEADLINE 的串行结构、SIDE 局部拓扑语义、比较器抖动。

### 方案 2:Local 前端彻底简化(重构)
目标:一条几何管线、一个可行性权威、一个质量比较器,代码与耗时双降。

REMOVE:
- offset backoff ×4 + duration fallback + observation→path-frame 重试(seed 生成只保留"direct + A*"两种);
- repair_only 分支(动态不足的 seed 一律交给唯一 preflight);
- seed 重建 ×3 循环与 bulge 修补(被下述 corridor 投影初始化取代);
- NOMINAL 与 SIDE 各自的 feasible-initializer fallback(统一为一个"任何阶段失败→上一个已验证 suffix"出口);
- validateRetimedLocalSfc 对 LOS 平面的检查(随 RC4 删除);
- preserve_active_observation_side 的 NOMINAL→INVALID 反向 gate(改为比较器偏好);
- 三处 A* 中的控制点变形 A*(finelyCheck 的 5477–5600 EGO 遗产)——SIDE/NOMINAL 统一走路径级 A*+corridor。

MERGE:
- 全部静态检查 → **一次**采样产出 witness(时间戳,位置,占据),所有 gate(预检/A* 审计/SFC/seed/classify/finalize)消费同一 witness;窗口统一为 executionAuthorityHorizon 一个口径;
- 全部动态 risk 评估 → 冲突窗局部采样 + 提交前一次全窗;
- visibility 12 权威 → LocalVisibilitySample 单原语(J_vis/三机报告/binary/K3/比较器全部由它派生);
- 两个 A* 实例 → 一个路径 A* 服务,输出 = 折线 + **有宽 corridor(星形/凸收缩)**,corridor 同时是 initializer 的投影目标(MINCO seed 生成后向 corridor 内投影一次)与 SCP/验证的唯一平面来源。

KEEP:MINCO+LBFGS、inflated grid map、obj_predictor、lifecycle/handoff(Feedback093 链)、K2/C3/D3 数学、M2 Team 协议、LOS soft slack QP(作为唯一 LOS 执行点)、executor/activation 确认。

REORDER:
```
risk(统一 witness)→ topology intent(L/R 记 sign,不再生成 bulge seed)
→ 一个几何 builder:direct 或 A* → corridor(有宽) → MINCO seed=corridor 投影
→ 一个优化器(LBFGS+唯一 LOS slack QP)
→ 一个 hard validator(current-revision,唯一 INVALID 权威)
→ 一个比较器(visibility→geometry→cost)
→ commit/activation
```
L/R 失败不再同 batch 重试:PLUS/MINUS 分相跨 cycle,单 cycle 只做 N+1 个 side。
风险:中高(删减面大,需等价性回归;Team/M2 接口保持不变是安全边界)。收益:NO_STATIC 语义只剩"真无路";EXECUTION_DEADLINE 大幅下降(单 cycle 工作量 ≈1/2);gate 数 35→约 12;fallback 18→约 6。

---

## 14. 结论

SHOULD_CONTINUE_PATCHING = **NO** —— 113→117 五轮已证明:每个新 gate/fallback 都能消灭自己的 label,但失败总量守恒、且最新的 513 恰是"补丁使其不可见"的产物(A* 资格错位 + catch-all 折叠)。继续逐计数修补只会继续改名。
SHOULD_REFACTOR_BEFORE_MORE_TUNING = **YES** —— 先做方案 1 的 1–3(小而准,直指 RC1/RC4),验证 NO_STATIC 分解后再评估是否上方案 2;两者都不需要动阈值、C3/D3 数学与 Team 协议。

---

## 15. 最终字段

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

FILES_REVIEWED:
  plan_manage/include/plan_manage/planner_manager.h
  plan_manage/src/planner_manager.cpp
  plan_manage/include/plan_manage/local_sfc_boundary_scan.h
  traj_opt/include/optimizer/poly_traj_optimizer.h
  traj_opt/src/poly_traj_optimizer.cpp
  path_searching/src/dyn_a_star.cpp
  plan_manage/src/ego_replan_fsm.cpp(调用上下文)
  traj_utils/trajectory_lifecycle(经头文件与调用点)
  runs/20260926_001027_482839/roslaunch_stdout.log(逐条归因)
  feedback/feedback_113–117(仅作运行证据)
FUNCTIONS_REVIEWED:
  reboundReplan; optimize_side(lambda); classify_candidate; attempt_side;
  finalizeCapturedCandidates; finalizeTopologyCandidate; finalizeCapturedLocal;
  prepareLocalHandoff; checkActiveHandoff; validateRetimedLocalSfc;
  validateExecutionTrajectory; validatePreviousRemainingTrajectory;
  ensureExecutionCoverage; optional/mandatoryPlanningAttemptAllowed;
  evaluateDynamicRisk; dynamicHardClearanceForObject; checkTrajectoryDynamics;
  checkTrajectoryStaticSafety; evaluateTrajectorySwarmConflict;
  buildFreshMovingInitializer; computeInitState; buildStaticFeasibleInitializer;
  buildWarmStartFromAccepted; canReuseRemainingSuffix; getLocalTarget;
  updateLocalK3RecoveryEvent; evaluateLocalVisibilitySampleAtWorldTime;
  evaluateLocalK3WindowMetrics; evaluateCandidateVisibility;
  fillCandidateBinaryVisibility; setLocalTrajFromOpt;
  optimizeTrajectory(+WithinBudget); finelyCheckAndSetConstraintPoints;
  enforceCandidateLosPlanesSCP; solveExecutionQPWithSlack;
  evaluateCandidateLocalSfcMaxViolation; runCandidateHardCorridorSCP(接口面);
  runTeamContractSCP(接口面); AStar::AstarSearch/visibilityPenalty;
  scanFirstLocalSfcBoundary; setExecutionDeadline/executionCheckpoint;
  ensureExecutionCoverage 相关预算函数; ego_replan_fsm try_replan/finish_captured

TOTAL_CANDIDATE_REJECT_GATES: 35(§2 表;其中 SIDE 路径 28 个)
TOTAL_FALLBACK_MECHANISMS: 18(§8 表)
DUPLICATE_STATIC_FEASIBILITY_CHECKS: 11(单条 A*-repair SIDE 最坏路径)
DUPLICATE_VISIBILITY_AUTHORITIES: 12(其中 K3 系已统一,3 个采样器仍在原语之外)
DUPLICATE_TOPOLOGY_AUTHORITIES: 6(preferred_side / LOS frame / K3 侧偏好 /
  observation-side 持久化 / target_side 比较偏好 / side 语义检查)

NO_STATIC_FEASIBLE_SIDE_ROOTS: A≈91.8% A* 未获资格(dynamic ≥1.1 m 门槛);
  B≈8% A* 修复链死亡(搜索/路径/SFC/seed 切角);C≈0.4% base 无静态碰撞区间;
  D deadline 打断。当前 telemetry 全部折叠为一个字符串。
ASTAR_ELIGIBILITY_ROOTS: astar_base_valid ← trial_dynamic_valid ←
  BODY: 全障碍 min_dist ≥ 1.1 m(preferred)/ LOS: !hard_collision;
  结构性错位成立(静态修复被动态软余量 gate)。
ASTAR_TO_MINCO_CONTRACT_STATUS: NOT A CONTRACT —— 采样检查+有界修补循环;
  corridor segment-local、不进 initializer、无保真算子;29/46 verdict 显式失败。
SIDE_TOPOLOGY_SEMANTICS: 局部 blocker 切向/种子偏移被当作完整 homotopy;
  backoff 与拓扑意图反向;B/C 树只能靠常被缴械的 A* 在同 side 内解决。
HARD_SAFETY_AUTHORITY_STATUS: 基本统一(static/dynamic/swarm/vaj/collision-SFC/
  revision),唯一例外见下。
VISIBILITY_AUTHORITY_STATUS: SEMI-CLEAN —— LOS 执行点已 soft 化(恒 return true),
  但 validateRetimedLocalSfc 仍采样并因 LOS 平面拒绝候选(110 条静默 safe=0),
  且无 world_time_anchored 处理;LOS_HARD_REJECT_COUNT=0 不覆盖此路径。

EXECUTION_DEADLINE_ARCHITECTURAL_CAUSE: 单线程 batch 串行 3×(LBFGS≤200×≤3 +
  全程方向 visibility 采样) + ≤4 offset×(全轨迹动态+静态采样) + A*/SFC/seed×3
  + finalize 两轮全量复验,而 wall 预算由 coverage 剩余导出;超时抛异常整候选
  作废 → initializer 兜底 → coverage 进一步被吃 → MIN_BUDGET restart 正反馈。

PATCH_ACCUMULATION_EVIDENCE: gate 35 / fallback 18 / 静态检查 11 / visibility 12 /
  topology 6;同物理失败三层机制先后处理(§8 串);两套 A* 三种语义。
FAILURE_RENAMING_EVIDENCE: LOS_PLANE_SCP_FAILED→(soft)→validateRetimedLocalSfc
  旁路;SIDE_INIT_STATIC_COLLISION 28→0 同时 NO_STATIC 197→513(SEED_EXPLICIT_FAIL
  29 为前移);EXECUTION_DEADLINE 171→136 为真实下降(兜底率 100%)。
HIDDEN_REJECT_PATHS: validateRetimedLocalSfc(LOS 平面,无日志无计数);
  checkActiveHandoff 关闭 log 时静默;astar_base 无效时无任何 per-side 日志;
  side_failure_cause_count_ 键过粗。

ROOT_CAUSE_1: A* 资格与动态 preferred 余量(1.1 m/全障碍)耦合 → 静态修复器被缴械
ROOT_CAUSE_2: A*→SFC→MINCO 无表示级 contract(corridor 局部、seed 无投影、修补有界)
ROOT_CAUSE_3: 单线程 N/L/R 全量串行 batch 与 rolling 实时预算结构性冲突(正反馈重启)
(ROOT_CAUSE_4 实现 bug:validateRetimedLocalSfc LOS 旁路;见 RC4)

ARCHITECTURE_STATUS: PATCH_ACCUMULATED

MINIMAL_STRUCTURAL_FIX: §13 方案 1(1. A* 资格解耦→hard 口径;2. RC4 修复+reason;
  3. NO_STATIC 分键;4. 可选 corridor 外延)。范围小、风险低、直击 513 与隐藏拒绝。
FULL_SIMPLIFICATION_PLAN: §13 方案 2(单几何管线+单 witness+单 validator+分相
  N+1 side;删 6 类分支、并 4 类检查、留 8 个成熟模块)。范围大、需等价回归。

WHAT_TO_REMOVE: offset backoff 链、duration/observation-frame 重试、repair_only、
  seed 重建×3 与 bulge、双 feasible-initializer fallback、retime-SFC 的 LOS 检查、
  observation-side 持久化的反向 INVALID、控制点变形 A*
WHAT_TO_MERGE: 静态检查→单 witness 管线;动态 risk→冲突窗局部+提交全窗各一次;
  visibility→LocalVisibilitySample 单原语;两套 A*→一个 corridor-A* 服务
WHAT_TO_KEEP: MINCO/LBFGS、inflated grid、obj_predictor、lifecycle/handoff、
  K2/C3/D3 数学、M2/Team 协议、LOS soft slack QP、executor activation 确认
WHAT_TO_REORDER: risk→intent→一个 builder→一个 optimizer→一个 validator→
  一个 comparator→commit;PLUS/MINUS 分相跨 cycle

SHOULD_CONTINUE_PATCHING: NO —— 五轮修复证明 label 可灭而失败守恒;513 本身是
  补丁产物(A* 资格错位 + catch-all),继续逐计数修补只会继续改名。
SHOULD_REFACTOR_BEFORE_MORE_TUNING: YES —— 先执行方案 1(1–3)并复跑分解 NO_STATIC;
  其结构收益可直接测量,再据其结果决定是否启动方案 2。
```
