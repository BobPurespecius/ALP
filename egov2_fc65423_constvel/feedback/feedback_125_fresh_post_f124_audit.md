# Feedback 125 — F124 之后全新只读架构审计(Local / Swarm / Execution 链)

日期:2026-09-26。工作区:`/home/bob/ALP/egov2_fc65423_constvel`(当前 dirty worktree 为唯一源码权威)。
性质:全新、独立、只读。不继承 Feedback121–124 的结论(F121–124 仅作历史运行证据引用);所有结论重新从 production source + run logs 得出。未修改 production、未编译、未运行测试/仿真、未调参;未访问 /home/bob/RRCT;未执行任何 git reset/checkout/restore/clean。

主 run 证据:`runs/20260926_164803_727379`(F124 FULL 尝试,BOOT 卡死手动终止)、`runs/20260926_135443_696819`(F123 FULL 尝试,证书冻结)。

---

## 0. 结论速览

**连续修复后仍在 BOOT 阶段暴露新死锁,不是巧合:系统的"生成层"从来没有硬约束(swarm、PVAJ 都只靠软代价),"验证层"是纯硬拒绝,两者之间没有任何协调协议。** 配置宽松时软代价恰好能把优化器压在硬判据内(运行看起来健康);配置一旦收紧(起步三机 0.85 m 间距 vs 0.5 m clearance),生成层产出的一切候选都会被验证层整批拒绝,随后由三层彼此独立的 liveness 耦合把局部停滞放大成舰队级永久冻结:

1. **候选死**(swarm/jerk 无逃逸通道,DESIGN);
2. **证书死**(hardCheckedUntil 在 `activation ≥ peer_end` 时 fail-closed——peer 一旦停止 commit,≤一个轨迹周期后**全舰队**的候选都无法获得证书,连本来安全的恢复候选也被禁止提交,LIVENESS);
3. **观测死**(runner 的 wait_for 计数单位是迭代次数而非秒,BOOT-12 单次探测 12–45 s,180"秒"实际≈1–2 小时,永远不会自己失败,RUNNER)。

三个死锁机制相互独立又相互喂料。`DYNAMICS_FAIL`、`SWARM_REJECT`、`TERMINAL_HOLD`、`BOOT-12 卡死` 是同一根因树上的四个症状,不是四个根因。

本轮最重要的**新测量事实**(纠正 F124 的归因比重):在 run 164803 中,
- **`DYNAMICS_FAIL`(jerk 超限)是头号候选杀手:classify 阶段 128,671 次 dynamics=0,其中 128,548 次 max_jerk>35(限值 20,容差 21);finalize 阶段 77,545 次 `reason=DYNAMICS_FAIL`**;
- swarm 物理拒绝只有 182 次 ellip 违规(finalize `reason=swarm` 28 次,全部对 drone 1;classify 纯 swarm 触发 58 次)。F124 把 swarm squeeze 当作 FIRST_CAUSAL_FAILURE 是对的(它是停止 commit 的**首因**),但把 DYNAMICS_FAIL 当次要现象低估了:它是 stall 期间的主力拒绝,且与 swarm squeeze 是**同一物理构型的两个表现**(见 §3)。

---

## 1. 当前真实 pipeline(逐层:INPUT/OUTPUT/时间语义/REJECT 权/安全归属/复检/突变)

```
FSM rolling tick (ego_replan_fsm.cpp, try_replan → reboundReplan)
│
├─ L0 状态输入    odom(p,v,a) + target state(stamp) + peer broadcasts(MINCOTraj, checked_until)
│                 时间:odom=now;target=stamp 外推;peer=各自 start_time 时基
│                 OWNER: FSM/planner 输入层。无拒绝权。
│
├─ L1 批次预算    ensureExecutionCoverage:coverage→wall deadline;MIN_BUDGET(current-state-restart)
│                 时间:wall clock + ros now。REJECT:可拒绝整批(optional/mandatory gates)。
│                 安全归属:无(预算)。突变:execution_deadline、current_state_restart_active_
│
├─ L2 NOMINAL     initializePvaTiming(53–101:8 次迭代膨胀,种子必须过 checkTrajectoryDynamics)
│                 → finelyCheckAndSetConstraintPoints(2/3 前缀) → LBFGS(≤200 iter×≤3 restart)
│                 内部硬检查:仅静态 finelyCheck + swarm 软接受门(1.25×buffer,采样点)
│                 OUTPUT: nominal_result(success 可为 soft 产物)。突变:多项式本尊。
│                 时间:t_now_=solve 时刻(≠activation,偏 ~0.1–0.15 s)
│
├─ L3 风险/拓扑   raw LOS 扫描 + dynamic risk + conflict descriptor → N/L/R dispatch
│                 REJECT:无(只决定是否生成 SIDE)。K3 事件同 cycle 授权。
│                 时间:world time(planning_prediction_epoch)。安全归属:无(触发器)。
│
├─ L4 SIDE seed   offset backoff(0.7→0.4)→ computeInitState(正弦 bulge,PVA-timing 种子)
│                 A* 资格:静态无条件(F119 已解耦;10619 任一 offset 即可作 base)
│                 → A* ×≤3 rejoin → raw_path_usable(锚边+逐点占据)
│                 REJECT:NO_STATIC_FEASIBLE_SIDE(catch-all)。安全归属:静态几何。
│
├─ L5 corridor    try_build_segment:定高 2.5D 六面棱柱(左右墙+前后帽+上下帽),
│                 inset 体素审计(corridor⊂free 证明),细分 depth≤2,leaf 失败整段失败,
│                 piece_id/piece_u 绑定 MINCO piece,junction overlap 校验
│                 REJECT:CORRIDOR_LEAF_FAILED / BUILD_FAILED / PIECE_BINDING_INVALID。
│                 安全归属:静态几何(已认证子集)。
│
├─ L6 优化        分发(5440–5468):带 STATIC corridor 的 SIDE → runCandidateHardCorridorSCP
│                 (硬行:corridor/vaj/动态;LOS 行除外) ;NOMINAL 与无 corridor SIDE → LBFGS
│                 LBFGS 约束:**无任何硬约束**——静态=soft(碰撞点推力),swarm=soft cubic
│                 (1.5×buffer,仅前 2/3 控制点),PVAJ=soft 二次,tracking/side/LOS=soft
│                 LOS polish:enforceCandidateLosPlanesSCP(semi-hard slack,恒 return true)
│                 REJECT:EXECUTION_DEADLINE(budget) / LBFGS 失败。突变:多项式+时长。
│                 seed 明确允许切角(12116–12123 注释);seed repair 链已删除。
│
├─ L7 classify    classify_candidate(9071):dynamics(tol 1.05)+static(全窗)+swarm(**trigger
│                 margin=clearance+0.08=0.58 m**,非物理 0.5)+dynamic hard_collision
│                 → safety_class=ABSOLUTE_SAFE/INVALID
│                 REJECT:HARD(safety_class)。安全归属:单机 hard。复检:L8/L9 全部重做。
│
├─ L8 finalize    finalizeCapturedCandidates:冻结 activation → prepareLocalHandoff
│                 (**重建多项式头为 executed@activation**)→ retime-SFC(连续 checker,
│                 LOS 排除)→ checkActiveHandoff → validateExecutionTrajectory
│                 (dynamics+static+risk+swarm 0.50)→ **hardCheckedUntil 证书门(1689)**
│                 REJECT:DYNAMICS_FAIL/swarm/UNKNOWN 证书等。突变:**多项式**(reanchor)、
│                 revision+1、checked_until。时间:activation=冻结批次锚点。
│
├─ L9 比较/commit comparator(K2 gate→C3→D3→side 偏好→visibility→geometry/cost;
│                 safety-required 时 QUALITY_VETO_BYPASS 保留首个 hard-safe,1960)
│                 → finalizeTopologyCandidate(prepared 不再二次 reanchor;但 6713 重算
│                 hardCheckedUntil 且 |Δ|>1e-6 即弃)→ setLocalTrajFromOpt:
│                 execution_safe_until_=hardCheckedUntil(2934–2960) → COMMITTED
│                 突变:active 轨迹、execution_safe_until_、accepted cache。
│
├─ L10 publish    PolyTraj/MINCO(float64+payload hash+revision+checked_until)→ broadcast
│
├─ L11 executor   traj_server:激活时校验 identity/revision/predecessor checked_until(910–917);
│                 **执行到 checked_until 停止(fail-closed)**;Team speculative 只按已验证
│                 horizon 认证。REJECT:拒绝激活。突变:执行状态、TERMINAL_HOLD。
│
└─ L12 peer/下轮  广播被邻居接收 → 下一个 rolling cycle 的 L0 输入。
```

WHO OWNS SAFETY(答案):**没有任何一层在生成时拥有硬安全;安全完全由 L7/L8/L9 的验证侧拥有**(corridor SCP 只覆盖携带 corridor 的 A*-repair SIDE)。 Team validator(realized_team_validator.cpp:249,`min_pairwise_distance ≥ swarm_min_separation`)与 traj_server 是第 4/5 个 swarm 判定点。

---

## 2. F124 swarm deadlock 独立重审(问题 2)

### 2A. 物理 swarm clearance 的标准到底有几个?

| # | 位置 | 判据 | 阈值 | 时间基准 | peer 模型 |
|---|---|---|---|---|---|
| 1 | LBFGS `swarmGradCostP`(PO:7337+) | soft cubic `(C2−ellip)³` | 1.5×0.5=**0.75 m** | `t_now_`=solve 时刻(**非 activation**) | polynomial,越界 lifecycle hold |
| 2 | LBFGS 接受门(PO:5342 `min_ellip_dist2_>(1.25c)²`) | 采样点软门 | 1.25×=**0.625 m** | 同上 | 同上 |
| 3 | classify `evaluateTrajectorySwarmConflict`(PM:8283) | `triggered=min_dist<clearance+0.08` → swarm_valid | **0.58 m** | candidate epoch | 同上 |
| 4 | `checkTrajectorySwarmSafety`(PM:8313,validateExecutionTrajectory/previous-remaining/Team realized/commit 复核) | **物理硬判据** ellip=dxy²+0.25dz² | **0.50 m** | activation+tt | polynomial,越界 hold@末端 |
| 5 | `hardCheckedUntil` hold 校验(PM:2531–2537) | 同 4 的椭圆 | 0.50 m | activation+tt | **停止点 hold(fly_until)**(第 2 种运动模型) |
| 6 | Team validator(realized_team_validator.cpp:249) | `min_pairwise_distance ≥ swarm_min_separation` | params(0.5) | realized 采样 | 独立实现 |
| 7 | traj_server predecessor gate(TS:910–917) | predecessor.checked_until 门 | – | activation | 证书 |
| 8 | optimizer 旧 `runCandidateTimeOnlySwarmDeconfliction`(PO:1001) | 时长调整 | – | – | **Local 无调用者(死代码)** |

**不是同一个标准**:4 个不同阈值(0.75/0.625/0.58/0.50)、2 个时间基准(solve 时刻 vs activation)、2 种 peer 运动模型(多项式飞行 vs 停止点保持)、≥4 个独立实现。classify(0.58)**比物理硬判据(0.50)更严**——trigger margin 被当成硬门(§7)。

### 2B. 为什么 SIDE 能 MINCO_OK 却死在 0.17–0.25 < 0.25?

生成层根本没有 hard swarm 约束,且其软约束在四个维度上系统性盲视:
1. **无硬约束**:LBFGS 只有 soft cubic;硬 vaj/corridor 行只存在于 corridor SCP 路径,而起步开阔区 SIDE 全部 DIRECT_ACCEPT(无 corridor)→ LBFGS。
2. **覆盖盲区**:swarm 代价只作用于前 2/3 控制点(`two_thirds_id`,PO:7344)——**时长后 1/3 完全没有 swarm 代价**;验证器却查满 authority horizon。
3. **时间错位**:生成用 `t_now_`(solve 时刻),验证用 activation(≈solve+0.10–0.15 s)。peer 速度 ~0.93 m/s → 生成期对 peer 相位采样误差 ~0.1–0.14 m,与 0.85−0.50=0.35 m 的裕度同量级。
4. **阈值梯度**:+50%(生成目标)/+25%(接受门)/+16%(classify)/0(硬判据)。优化器停在 0.625–0.75 之间的解,验证链却在 0.50/0.58 判——0.41–0.50 m 的候选就是从 2/3 覆盖盲区与时间错位里漏出来的。

另有 peer revision 竞态:生成用 solve 时快照,验证用 check 时刻广播;两机同时 replan 时各自基于对方**旧**轨迹,新-新组合**没有任何人验证**(§4)。

### 2C. 起步 0.85 m vs 0.5 m:有逃逸方向吗?

物理上**存在**(0.85>0.5,同步编队飞行本身可行),但当前系统**不具备产生它的机制**:
- Local 的 SIDE/topology 只由 BODY/LOS 触发——**swarm 冲突不触发任何拓扑搜索**;唯一 swarm 生成机制是软代价推挤;
- peer 被冻结为已广播未来:我若减速/绕行,验证时对手的旧未来仍穿过我的新走廊;两机同时 replan 的对称冲突无仲裁(`temporal_owner` 在 PM:8285 计算后**无任何消费者**);
- 一旦在飞 incumbent 对互相进入 <0.5 m(或候选永远过不了 0.5 判据),**系统没有"从已违规状态恢复"的定义**:代码注释自述"已经互相违反的轨迹不能互相证明安全"(PM:8329–8331),NON_WORSENING 被删后没有任何受控放行,所有候选被拒 → 执行旧轨迹直至证书终点 → TERMINAL_HOLD。

**归类:DESIGN(生成层无 swarm 合同、无恢复语义)+ COORDINATION(同时 replan 无协议),由 PHYSICAL_INITIAL_STATE(0.85 m 编队 vs 0.5 m clearance,裕度仅 0.35 m)触发。不是单纯的 checker 太严,也不是单纯实现 bug。** 恢复 NON_WORSENING 只是把硬判据再次打穿(§12)。

---

## 3. NOMINAL DYNAMICS_FAIL 展开(问题 3)

run 164803 实测(参数:max_vel=3.0/max_acc=6.0/max_jer=20.0,tol=1.05 → 门限 3.15/6.3/21.0):

| 违规项 | 观测值 | 判定 |
|---|---|---|
| max_v | 3.007–3.22 | **通过**(3.15 门限内,除个别 3.22) |
| max_a | 3.34–6.04 | **通过**(6.3 门限内,个别 6.00/6.04 压线) |
| **max_jerk** | **21.2–47.1(>35 占 99.9%)** | **FAIL——唯一硬违规项** |

分布:classify 128,671 次 dynamics=0(128,548 次 jerk>35);finalize 77,545 次 `DYNAMICS_FAIL`;时间上从 sim t≈4 s 即开始,stoppage 后稳定 ~60 次/s 持续整个 25 min(规划器在死锁构型上全速重试)。

发生位置(两处,机制不同):
1. **raw MINCO 产物**:`[dynamics-check] stage=final` 显示 SIDE 优化输出 jerk 21.2/46.4/34.9/47.1(SIDE 证据;NOMINAL 同链)。LBFGS 的 feasibility 是 soft 二次项,时长自由(wei_time_)且 swarm 软推挤制造 kink,收敛点是尖峰解——**生成层对 PVAJ 同样没有硬约束**(hard vaj 行只在 corridor SCP,而起步期 SIDE 无 corridor、NOMINAL 永远不走 SCP)。
2. **finalize reanchor**:`prepareLocalHandoff` 把头状态换成 executed@activation 后用**相同内点与时长**重新 generate(PM:1646→PM:2163–2168),头速度差 δv≈a·0.1≈0.6 m/s 注入首段,jerk ~δv/T₁² 放大;validateExecutionTrajectory 在重建后的多项式上判 DYNAMICS_FAIL。

与 F121 的历史证据衔接:F121 已测得固定 PVA 后 97/117 条 timing `j_after>20`——**种子/时标质量弱点在 F122 之前就存在**;F123 的 initializePvaTiming 修了**种子**(fresh/N/SIDE 种子必须 dynamics-valid),但 LBFGS 产物与 reanchor 两个口子没有修。

**判定:与 swarm deadlock 不是独立问题。** 同一起步挤压构型:SIDE 死于硬 swarm 判据,NOMINAL 死于软 swarm 推挤扭曲出的 jerk 尖峰(+慢性 timing 弱点)。它是**同一根因的两个症状**,外加"生成层无 PVAJ 硬约束"这一结构缺口;stall 后的重试风暴把它放大成 12.8 万次计数。

---

## 4. swarm "生成 vs 验证"职责(问题 4)

**当前就是 "generate without swarm-safe guarantee → final checker reject",源码链:**
`swarmGradCostP`(soft,1.5×,2/3 前缀,t_now_)→ `optimizeTrajectory` 接受门(min_ellip>1.25c,采样点)→ `classify`(0.58)→ `validateExecutionTrajectory`→`checkTrajectorySwarmSafety`(0.50 物理)→ `hardCheckedUntil`(0.50 vs hold 点)→ Team validator(独立实现)→ traj_server。

正确职责应当是:**Local optimizer 在已知 peer future 下生成满足物理 clearance 的候选(硬约束或带硬拒绝的修复回路),final preflight 只验证同一谓词。** 现有 Local+Team 分工是否足够?**分工本身足够,不需要新 swarm planner**:peer future 已经可用(swarm_trajs_)、时长自由度已存在(virtual T)、Team 已有同时序协议(PREPARE/COMMIT + 共同 activation)。缺的是三件现有机制的接线,不是新系统:
1. 把物理 clearance 变成**生成层硬约束**(corridor SCP 已证明这条路可走——把 swarm 行加进 SCP,或 LBFGS 后接一次 time-only 去冲突——`runCandidateTimeOnlySwarmDeconfliction` 已写好但 Local 无调用者);
2. **同时 replan 的对称冲突**:需一个确定仲裁(现成的 `temporal_owner = drone_id > other_drone_id` 从未接线)——低 ID 让高 ID 的已提交未来优先,或双方都退回"保持分离分量"的约束;
3. Team transaction 目前只服务 M2/relay 合同,**不**解决普通编队飞行的 simultaneous change。

---

## 5. checked_until 证书模型(问题 5)

F124 的去递归修复本身正确且必要(源码 PM:2489–2506 确认:peer 贡献改为机械事实;hold 点逐 0.03 s 验证;fail-closed 保留)。但独立审查发现**四个残留缺陷**:

1. **peer 过期 fail-closed(PM:2485–2488)**:`activation ≥ peer_end − ε → return activation`。peer 停止 commit 后,其广播多项式地平线冻结;≤一个轨迹周期(~3.4 s)内,**全舰队**所有候选的证书都变 UNKNOWN——即使某个候选物理上安全(对手已停在可验证的 hold 点)。hold-model 分支只在 `activation < peer_end` 时可达,**"peer 已停"这个最需要 hold 模型的场景反而不可达**。这是把"peer 数据可用性"误当"安全证书"的残余:peer 多项式末端过期只是**运动模型退化**(退化为停点),不是安全事实消失。run 164803 的 309 次 UNKNOWN 与 TERMINAL_HOLD=4 与此一致。
2. **commit 竞态**:finalizeTopologyCandidate 在 PM:6713 重算 hardCheckedUntil 且 `|Δ|>1e-6 即弃`——peer 在我的 capture→commit 间隙 commit 会改变我的证书并杀掉我的候选。又一个 liveness 耦合。
3. **双 peer 运动模型并存**:checker 用 polynomial-then-hold@end,证书用 hold@fly_until(checked_until<peer_end 时两者不同)——同一 peer 在同一 cycle 被两套模型校验,验证口径不一致(保守,但会在 fly_until~peer_end 窗口制造假拒绝)。
4. **executor 停在 checked_until 的 liveness**:fail-closed 方向正确;但当证书续期依赖"所有 peer 持续 commit"(缺陷 1)时,executor 停止 → peer_end 冻结 → 互相冻结,构成**cyclic liveness**:任何单机候选枯竭都会传染成舰队永久停机。

**CERTIFICATE_MODEL_STATUS = PARTIAL**(递归已断、fail-closed 语义正确、hold 模型方向正确;但 peer-expiry 应降级为 hold@末端验证而非 UNKNOWN,commit 竞态与双模型需消除)。

---

## 6. SFC / MINCO 当前状态(问题 6,十二项)

| 项 | 状态 | 依据 |
|---|---|---|
| A* route edge/anchor 连续静态安全 | **FIXED(结构性)/ NOT_RUN_VERIFIED** | raw_path_usable 校验 anchor→path 首尾边+逐点占据(PM:11204+);F123 加入 voxel 边检查;但 F124 run 仅 4 次 corridor 事件,无有效 run 验证 |
| SFC all-leaf success | **FIXED** | leaf 失败→整段失败并清空 planes(PM:11974–11987) |
| 2.5D extrusion z 合同 | **FIXED** | 定高棱柱、斜段显式拒绝(PM:11702–11706) |
| corridor/MINCO piece binding | **FIXED** | piece_id/piece_u 绑定+绑定校验(PM:12168–12185, 12018–12030) |
| continuous checker | **FIXED(代码层)/ PARTIAL(证明层)** | 五项 quartic 精确极值(PO:2445+);LOS 排除;F122 解析用例 PASS,无生产级独立复验 |
| static plane local-time mapping | **FIXED** | staticPlaneInterval 按 piece 映射(PO);retime 不再按秒比例缩放 |
| fixed-PVA timing | **PARTIAL** | 种子已强制 dynamics-valid(initializePvaTiming);但 A* seed 仍是一次 λ 且 λ=1.0(PM:12257 日志),LBFGS 产物无 PVAJ 约束(§3) |
| dynamic world-time Jacobian | **FIXED(F122 声明)/ NOT_VERIFIABLE(无 run 证据)** | SCP 时间行 piece-local u;无有效 FULL run 触及 |
| seed tube hard authority | **FIXED(REMOVED)** | PM:12116–12123:repair 链与 SEED_COLLISION_EXPLICIT_FAIL 已删 |
| LOS hidden hard gate | **FIXED** | continuousLocalSfcMaxViolation 跳过 LOS(PO:2253–2256);enforceCandidateLosPlanesSCP 恒 true;F118 RC4 已闭合 |
| optional LOS polish 覆盖 hard-safe incumbent | **PARTIAL** | Local 侧 safety-required bypass 已接线(PM:1957–1970);Team/polish 路径无 run 证据 |
| **(新)corridor 硬约束只在 SCP 分支生效** | **PARTIAL——设计依赖未验证的分支** | 带 corridor 的 SIDE 自动走 hard SCP(PO:5455–5456);NOMINAL 与无-corridor SIDE 仍是"软生成→硬验证";corridor SCP 本身无有效 run 验证(run 164803 仅 4 次 corridor 事件) |

---

## 7. safety authority 表(问题 7)

| STAGE | FUNCTION | CONDITION | CATEGORY | SHOULD_BE_HARD? | DUPLICATED? | 同一谓词? |
|---|---|---|---|---|---|---|
| LBFGS | swarmGradCostP | soft,1.5×,2/3 前缀 | 生成引导 | 不适用(应升级为硬) | 与 #4/#5 重复 | 否(0.75) |
| LBFGS 接受门 | min_ellip>1.25c² | 采样点 | 生成引导 | 否(假门) | 重复 | 否(0.625) |
| corridor SCP | 硬 vaj/corridor/swarm?行 | 仅 corridor SIDE | 生成硬约束 | 是 | – | 部分 |
| classify | checkTrajectoryDynamics | tol 1.05 | HARD | 是 | ×3(classify/finalize/commit) | 是 |
| classify | checkTrajectoryStaticSafety | 全窗 | HARD | 是 | ×4 | 是 |
| classify | **swarm trigger 0.58** | clearance+0.08 | **质量 margin 当 hard** | **否——应回物理 0.5** | 与 #4 重复 | **否** |
| classify | risk.hard_collision | 物理 | HARD | 是 | 与 finalize 重复 | 是 |
| finalize | validateExecutionTrajectory | dyn+static+risk+swarm0.5 | HARD | 是 | 全部重验 | 是 |
| finalize | **hardCheckedUntil 证书门** | checked_until≤activation → 弃 | HARD(liveness) | 是(修正后) | 与 commit 重复 | 是 |
| finalize | continuousLocalSfcMaxViolation | ≤1e-6,LOS 排除 | HARD | 是 | 与 SCP 硬行重复(两套静态) | 近似 |
| commit | hardCheckedUntil 重算 \|Δ\|>1e-6 | peer 竞态 | HARD | **否——竞态非安全** | 重复 | 是 |
| comparator | K2 gate / C3/D3 | 质量 | QUALITY | 否 | – | – |
| dispatch | SIDE 触发(BODY/LOS) | 拓扑 | TOPOLOGY | 否(仅触发) | – | – |
| Team | realized validator | min_pairwise≥separation | HARD | 是 | **第 4 个 swarm 实现** | 近似 |
| executor | traj_server predecessor/证书门 | identity+证书 | HARD | 是 | 与 planner 证书重复 | 是 |
| budget | executionCheckpoint/side_work_allowed | wall 预算 | BUDGET | 是(但应分相) | – | – |

发现的错位:classify swarm 用 trigger margin(0.58)当 hard(§2A);numerical 失败仍可与物理不可行混记(LBFGS 失败 vs STATIC_INFEASIBLE 在 side_static_reason 口径上已部分分离);K2 质量门在 safety-required 时已正确让位;**未发现** optional quality 覆盖 hard-safe 的现存路径(bypass 已接线)。

---

## 8. fallback / restart 堆积盘点(问题 8)

| 机制 | 状态 | 判定 |
|---|---|---|
| initializePvaTiming(种子动力迭代) | 活跃 | NECESSARY |
| accepted-cache warm start(N/S) | 活跃 | NECESSARY |
| offset backoff 0.7→0.4 | 活跃 | REDUNDANT(swarm 死锁下 4 次全采样纯烧预算) |
| observation→path-frame 重试 | 活跃 | LEGACY(应随单一种子管线合并) |
| A* rejoin ×≤3 + first_success 保留 | 活跃 | NECESSARY(但可并入单次搜索) |
| corridor 细分 depth≤2 | 活跃 | NECESSARY(新,正确) |
| **seed 修复链(restore/bulge×3)** | **已删除** | —(F119 正确删除) |
| feasible-initializer fallback(N/S) | 活跃 | NECESSARY 但两个实现应合一 |
| EXECUTION_DEADLINE 初值恢复 | 活跃 | NECESSARY |
| KEEP_PREVIOUS / PERSISTENCE | 活跃 | NECESSARY |
| current-state restart + MIN_BUDGET | 活跃,**26,677 次** | 下游症状;但 restart 全量 N/L/R 重试本身构成第二层循环(自馈负载) |
| terminal hold(executor) | 活跃,4 次 | NECESSARY(fail-closed 正确) |
| Team speculative fallback | 活跃 | NECESSARY(有让位序) |
| time-only swarm deconfliction | **死代码(Local 无调用者)** | LEGACY |
| temporal_owner | **死赋值(无消费者)** | LEGACY |

MIN_BUDGET storm(26,677)与 DYNAMICS_FAIL storm(12.8 万)都是**下游现象**;但 current-state-restart 的"每 tick 全量重试"与证书-peer 竞态是**会自我放大的第二层循环**。

---

## 9. BOOT-07/08 与 runner(问题 9)

- BOOT-07=首帧 odom、BOOT-08=首条本地轨迹(`topic_has_publisher`+首条消息)、BOOT-12=`simulation_running`(target+三 UAV 位移>0.05 m/3 s 或 pos_cmd 在线)。F124 的"BOOT-08 三机激活检查"按源码实为 **BOOT-12 语义**(BOOT-08 实际已通过——三机都有轨迹发布)。
- **死锁时 runner 永远等的原因**:死锁下无人机停在 hold 点、pos_cmd 停发 → `uav_moving` 全失败 → `simulation_running` 永假。
- **`--boot-timeout 180` 未生效的机制(单位 bug)**:`wait_for` 的循环变量是**迭代次数**(`waited++`/秒),而 `simulation_running` 单次评估 = `displacement_over_3s`(≥3 s sleep+rostopic)×4 + `topic_alive`(2 s timeout)回退 ≈ **12–45 s**。180 次"迭代"≈ **1–2.5 小时**,不是 180 秒。BOOT-03–11 的探测便宜(~1–5 s)所以近似秒,BOOT-12 放大 12–45 倍。
- **日志无限刷**:三机规划器在死锁构型上以 ~20–30 cycle/s 全速重试(classify/finalize 每周期几十行 ROS_INFO)×25 min → 887 MB。规划器侧无"持续无解则退避"的节流。
- **应 fail 的位置**:`wait_for(12)` 应按 wall-clock 计时;`simulation_running` 的多秒探测应与循环超时解耦(先快探测后慢证实);finalize/exit_status 未落盘是因为手动终止绕过了 finalize——runner 需要一个信号驱动的 emergency finalize。**修复方向仅此,不改 planner。**

---

## 10. 历史因果对照(问题 10)

RESOLVED_ROOT_CAUSES:
- **F118 RC1(A* 动态资格缴械)**:已真修(F119 解耦,PM:10607–10625;run 164803 无 NO_STATIC 风暴)。
- **F118 RC4(LOS retime 隐藏 gate)**:已真修(continuous checker 排除 LOS;enforceCandidateLosPlanesSCP 恒 true)。
- **F123 证书递归死锁**:已真修(F124;run 164803 证书前移 6.6 s,TERMINAL_HOLD 4≠3 常驻)。
- **F118 RC2(seed 切角 check-repair 循环)**:以"corridor 硬 SCP 接管"方式结构性替换(路径正确),但该分支的有效性无 run 验证。

MASKED_ROOT_CAUSES(一直存在,以前被更早死锁遮住):
- **生成层无 swarm 硬约束**(F122 删 NON_WORSENING 前被隐式放行遮住);
- **生成层无 PVAJ 硬约束 + LBFGS 尖峰解**(F117 前被"种子 dynamics-valid+健康构型"遮住;F121 的 97/117 j_after>20 早已测到);
- **peer 地平线 fail-closed 的舰队耦合**(F123 的递归死锁遮住了它——修好递归才暴露)。

NEWLY_INTRODUCED_ISSUES:
- F122 删除 NON_WORSENING **没有同步提供任何替代逃逸通道**(设计上正确、迁移上不完整);
- F123/P122 的"prepared 候选不再二次 reanchor"只覆盖 commit,**finalize 仍对每个候选 reanchor 重建多项式**(jerk 放大口子);
- runner wait_for 单位 bug 属工具债(F124 首次以 FULL 失败形式暴露)。

STILL_UNRESOLVED_ISSUES:同时 replan 对称冲突无仲裁;swarm 拒绝不触发拓扑/重规划;recover-from-violation 无定义;F118 RC3(N/L/R 串行全量 vs rolling 预算)原样存在(EXECUTION_DEADLINE 35 次、MIN_BUDGET 风暴)。

---

## 11. 根因树(问题 11)

```
ROOT PROBLEM:安全由验证层独家拥有,生成层(软)与协调层(无)从未承接;
             任何紧构型都会把"全部候选被验证层拒绝"放大成舰队冻结
│
├─ ARCHITECTURE(FIRST_STRUCTURAL_CAUSE,confidence HIGH)
│  ├─ 源证:LBFGS 无硬 swarm/PVAJ(PO:7337 soft, 5440–5468 分发);
│  │        swarm 拒绝不触发任何拓扑/重规划;无 recovery-from-violation 语义
│  ├─ run 证:164803——SIDE MINCO_OK 全部死于 finalize swarm/jerk;
│  │        128,671 dynamics=0;commits 冻结于 traj 6/22/7
│  └─ 影响:紧构型下零供给;宽松构型下掩盖
│
├─ COORDINATION(SECOND_STRUCTURAL_CAUSE,HIGH)
│  ├─ 源证:peer 冻结未来+同时 replan 无仲裁;temporal_owner 死赋值(PM:8285);
│  │        Team transaction 只管 M2/relay
│  └─ run 证:三机起步互相把对方旧未来判进 0.41–0.50 m(ellip 182 次)
│
├─ LIFECYCLE / CERTIFICATE(HIGH)
│  ├─ 源证:peer 过期 fail-closed(PM:2485–2488);commit 证书竞态(PM:6713);
│  │        双 peer 运动模型并存
│  └─ run 证:CANDIDATE_CERTIFICATE UNKNOWN=309;TERMINAL_HOLD=4;
│        135443 FINAL_CHECKED_UNTIL 冻结(递归版,已修)
│
├─ IMPLEMENTATION(MEDIUM)
│  ├─ finalize reanchor 重建多项式(jerk 口子);A* seed λ=1.0 无校正;
│  │        classify swarm 0.58≠物理 0.5;time-only 去冲突死代码
│
├─ NUMERICAL(LOW–MEDIUM)
│  ├─ LBFGS 软 feasibility + 时长自由 → jerk 21–47 尖峰;无 run 级 OSQP 归因
│
├─ SAFETY AUTHORITY(MEDIUM)
│  ├─ 4 个 swarm 阈值/2 个时间基准/≥4 个实现(§7 表);trigger margin 当 hard
│
└─ RUNNER / TOOLING(HIGH 独立)
   ├─ wait_for 单位 bug(迭代≠秒);BOOT-12 探测 12–45 s;无 emergency finalize
   └─ run 证:--boot-timeout 180 失效,console 887 MB,无 exit_status.txt

DOWNSTREAM_SYMPTOMS: DYNAMICS_FAIL(12.8万) / SWARM_REJECT(182) /
  TERMINAL_HOLD(4) / BOOT-12 不满足 / MIN_BUDGET 风暴(26,677) /
  日志洪水 —— 同一树上的叶子,不是根。
```

---

## 12. 一次收敛方案(问题 12,只给方案)

KEEP:initializePvaTiming 与 corridor 6 面棱柱+piece 绑定(新合同正确);continuous checker;LOS soft slack QP;safety-required quality bypass;certificate fail-closed 方向与 hold 点验证;executor identity/revision 链;lifecycle/handoff;M2/Team 协议;runner 的 BOOT 分项结构。

FIX(接口,非新增):
1. **hardCheckedUntil 的 peer 过期分支**:activation≥peer_end 时降级为"hold@polynomial 末端 + inflate 校验"(机械事实),返回 min(self_horizon, activation+hold_verified_span),**不再 return activation**;删除 commit 处 |Δ|>1e-6 竞态弃单(改为用新证书)。
2. **swarm 谓词统一**为一个物理判据(dxy²+0.25dz²≥0.25,activation 时基,polynomial-then-hold@checked_until 单一运动模型),classify/validate/证书/Team validator 全部消费同一实现;删除 classify 的 0.58 trigger 门(降为遥测)。
3. **swarm 逃逸在生成层解决**(答案 1:生成层,不是验证层更不是协调层先行):把物理 clearance 作为 LBFGS 采样点的硬拒绝+时长修正(接线现成的 runCandidateTimeOnlySwarmDeconfliction),或把这些 SIDE/NOMINAL 送入已有 corridor SCP 加 swarm 硬行;**同时 replan 对称冲突由 temporal_owner 仲裁**(低 ID 服从高 ID 已提交未来),答案 6:Team 不需要介入普通 avoidance,只需把仲裁规则写进 Local。
4. **finalize 不再重建多项式**:prepareLocalHandoff 只在 current-state-restart 分支重建;正常路径把 reanchor 误差留给下一 tick(或对 prepared 候选做一次时长-only 校正),消除 jerk 放大口子。
5. runner:wait_for 改 wall-clock;BOOT-12 拆快/慢两级探测;信号驱动的 emergency finalize。

REMOVE:classify 的 0.58 swarm 门;LBFGS 的 1.25× 采样接受门(被硬拒绝替代);time-only 去冲突的"死代码"状态(接线后删除重复);MIN_BUDGET/restart 的每-tick 全量重试(同构型连续失败 N 次后指数退避);双 peer 运动模型(保留 checked_until 停点一种)。

MERGE:swarm 7 判定点→1;静态两套(SCP 硬行+continuous checker)→SCP 生成+checker 验证同一平面集;PVAJ 生成硬约束(LBFGS 采样硬拒/SCP)与验证(同一 tol)。

DO_NOT_ADD:不恢复 NON_WORSENING(答案 2:NO——它把"与我已有的违规轨迹比"当安全,重新打穿物理地板;等价目标由生成层硬约束+分离单调的软目标达成);不加新 swarm planner;不加新 fallback/exception/bypass;不加新参数。

答案 3:NOMINAL DYNAMICS_FAIL 应与 swarm 同轮修(同根因:生成层无硬约束;只修一侧会留下另一半候选枯竭)。答案 4:checked_until 需要继续改(peer-expiry 降级+竞态),但改动面小、方向不变。答案 5:SFC/MINCO 本体**不值得再动**(F119–F122 的合同是新代码中最健康的部分),缺的只是有效 run 验证——冻结。答案 7:下一轮 production 修改应覆盖 §12 FIX 1–4(+runner);**冻结不动**:corridor/SCP/continuous checker/K3/C3/D3/M2/证书 wire 格式/阈值。

---

## 13. 最终字段

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

CURRENT_ARCHITECTURE_STATUS: PATCH_ACCUMULATED→CONVERGING(前端静态链已收敛且质量高;
  安全生成层与协调层是最后的结构空洞)

CERTIFICATE_MODEL_STATUS: PARTIAL(递归已断、fail-closed 正确;peer-expiry 应降级为
  hold@末端而非 UNKNOWN;commit 证书竞态与双 peer 运动模型待消除)
SWARM_GENERATION_STATUS: SOFT_ONLY(1.5×buffer/2/3前缀/t_now_ 时基;无硬约束;无逃逸通道)
SWARM_VALIDATION_STATUS: PHYSICAL_0.50_HARD 但 7 个判定点 4 个阈值 2 个时基 2 种
  peer 模型;classify 0.58 比物理更严
SWARM_COORDINATION_STATUS: NONE(冻结 peer+同时 replan 无仲裁;temporal_owner 死赋值;
  Team 只管 M2/relay)
NOMINAL_DYNAMICS_STATUS: JERK_LIMIT_BINDS(限 21,观测 21–47;raw 产物与 finalize
  reanchor 两个口子;与 swarm squeeze 同根因)
SFC_MINCO_STATUS: CORRECT_BUT_UNVERIFIED_BY_RUN(6 面棱柱/piece 绑定/连续 checker
  代码层正确;有效 run 证据为零)
LOS_AUTHORITY_STATUS: CLEAN(soft slack 独占执行;retime checker 已排除)
QUALITY_AUTHORITY_STATUS: CLEAN(safety-required bypass 已接线)
EXECUTION_IDENTITY_STATUS: SOUND_IN_SOURCE(float64+hash+revision;端到端无 run 证据)
RUNNER_BOOT_STATUS: BROKEN(wait_for 迭代≠秒;BOOT-12 探测 12–45 s;无 emergency finalize)

TOTAL_ACTIVE_HARD_REJECT_AUTHORITIES: 16(static×2/dyn×2/swarm×4/PVAJ×2/SFC×1/
  certificate×2/identity×1/budget×1)
TOTAL_ACTIVE_FALLBACKS: 14 活跃 + 2 死代码(time-only 去冲突、temporal_owner)

RESOLVED_ROOT_CAUSES: F118-RC1(A*资格) / F118-RC4(LOS retime 隐 gate) /
  F123 证书递归 / F118-RC2(seed 切角循环→corridor 合同,待 run 验证)
MASKED_ROOT_CAUSES: 生成层无 swarm 硬约束(被 NON_WORSENING 遮) / 生成层无 PVAJ
  硬约束(被健康构型遮) / peer 地平线舰队耦合(被递归死锁遮)
NEWLY_INTRODUCED_ISSUES: 删 NON_WORSENING 未给逃逸通道 / finalize reanchor 口子
  (prepared 豁免只覆盖 commit) / runner 超时单位 bug
STILL_UNRESOLVED_ISSUES: 同时 replan 仲裁 / swarm 触发的重规划 / recovery-from-
  violation 语义 / N-L-R 串行 vs 预算(F118-RC3)

ROOT_CAUSE_1: 安全独家拥有于验证层——生成层(LBFGS)对 swarm/PVAJ 无硬约束,
  紧构型下零候选供给
ROOT_CAUSE_2: 协调层缺位——peer 冻结、同时 replan 无仲裁、无恢复语义,
  局部停滞经证书 peer 地平线耦合放大为舰队冻结
ROOT_CAUSE_3: 证书 liveness——peer 过期 fail-closed 使"peer 停止"反而禁止
  全舰队续期;叠加 commit 证书竞态

NON_WORSENING_SHOULD_RETURN: NO(以生成层硬约束+分离单调软目标替代)
TEAM_REQUIRED_FOR_SWARM_ESCAPE: NO(temporal_owner 本地仲裁即可;Team 留给 M2/relay)
SFC_SHOULD_CHANGE_NOW: NO(冻结;只欠有效 run 验证)
TIMING_SHOULD_CHANGE_NOW: YES(仅两处:finalize 停止重建多项式;A* seed λ 校正)
CERTIFICATE_SHOULD_CHANGE_NOW: YES(peer-expiry 降级+commit 竞态,小改动面)

WHAT_TO_KEEP: corridor 棱柱/piece 绑定/连续 checker/initializePvaTiming/LOS soft/
  safety bypass/证书 wire 格式/executor identity/M2/lifecycle/runner BOOT 分项
WHAT_TO_FIX: hardCheckedUntil peer-expiry 与 commit 竞态;swarm 单谓词接线;
  生成层 swarm 硬约束(硬拒+time-only 修正/SCP 行);temporal_owner 仲裁;
  finalize reanchor;runner wall-clock 超时+emergency finalize
WHAT_TO_REMOVE: classify 0.58 门;LBFGS 1.25× 假门;MIN_BUDGET 每-tick 全量重试;
  双 peer 运动模型之一;死代码转正或删除
WHAT_TO_MERGE: swarm 7 判定点→1 实现 1 阈值 1 时基;静态生成(SCP)与验证
  (checker)同一平面集;PVAJ 生成/验证同 tol
WHAT_NOT_TO_ADD: 新 swarm planner / 新 fallback / 新 bypass / 新参数 /
  NON_WORSENING 复辟

ONE_SHOT_REFACTOR_SCOPE: §12 FIX 1–5 + REMOVE/MERGE 全部;预计净删代码;
  触及 PM(hardCheckedUntil/classify/finalize/仲裁)与 PO(swarm 硬约束接线)、
  runner;冻结 corridor/SCP/LOS/K3/Team 协议/阈值

SHOULD_PATCH_NEXT_FAILURE: NO
SHOULD_REFACTOR_BEFORE_NEXT_FULL: YES —— 三个死锁层(生成供给/证书 liveness/
  runner 超时)都已从源码定位且有 run 证据;不收敛就跑 FULL 只会继续产出
  887 MB 日志的同型失败。收敛后 FULL 的通过条件也变得可判定
  (BOOT-12 会在真实 180 s 内给出失败原因而不是挂死)。

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
