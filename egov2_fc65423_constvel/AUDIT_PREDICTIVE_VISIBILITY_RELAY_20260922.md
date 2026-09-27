# ALPv2 预测式可见性接力 + Extra Team-SCP 只读源码审计报告

工作区:`/home/bob/ALP/egov2_fc65423_constvel`
日期:2026-09-22
基线:当前工作树(含未提交的 production 修改;最后一次提交 8cef241;当前树已验证可编译,catkin build multi_uav_formation ego_planner exit 0)
RRCT_ACCESSED: NO / RRCT_CHANGED: NO

---

# 1. Executive conclusion

**方案在当前 ALP 架构中适合实现,且比预想的更接近可行**:任务书要求的执行闭环(PREPARE → EXECUTOR_READY×3 → COMMIT → common activation,事务级 ABORT)在当前树中**已经真实存在**并经过真实仿真(见 §2、§13 尺度数据);RealizedTeamValidator 已存在并已接入 commit 门(当前树 `multi_uav_topology_coordinator.cpp:1497`),它就是本方案"realized validation"的直接扩展点。

**最自然的实现位置**:
- M2 预测分析:coordinator(`multi_uav_topology_coordinator.cpp`),在 `evaluateSnapshot` 的 `core_->select` 之后、`attemptTargetCenteredTeamReference` 之前——那里已经有三条 committed Local 轨迹(bundle)与统一的 `continuousVisibility` 实现。
- Handoff Contract:coordinator 新增(独立小模块或挂在 target-centered 流程旁),走已有的 `/topology_coordination/team_reference_schedule` 通道或新增专用消息。
- Extra Team-SCP:`poly_traj_optimizer.cpp` 新增一个 `runTeamHandoffRefinement(...)`,组装自既有三个 QP 路径的零件(见 §6)。

**可直接复用的模块**:
1. `mincoSampleGradientWrtDecision`(`poly_traj_optimizer.cpp:109`)——已输出**完整行**(P 列 + T 列,含 `ds_dT` 固定 world-time 修正),TEAM_PT 的行构造直接用它;P-only 的 `enforceCandidateLosPlanesSCP` 只是截掉了 T 列。
2. T-only 骨架:`runCandidateTimeOnlyFeasibilityCorrection`(`:398`,virtual trust 0.15、4 迭代、median 物理信任)与 `runCandidateTimeOnlySwarmDeconfliction`(`:607`)。**但两者当前只有 unit-test 调用,是生产死路径**(全仓 grep 仅 test 命中)——复用其骨架而非其调用语义。
3. `continuousVisibility`(coordinator)+ `directionalClearanceRisk`(κ=1,β=2,深遮挡 continuation)+ `trackingCameraFovDirectionalRisk`(含 yaw 导数)+ `advanceTargetFacingYaw`(executor 同一 yaw contract)——M_i(t) 的四个分量全部有解析梯度。
4. `RealizedTeamValidator::evaluateMetrics`(`realized_team_validator.cpp`)——已经对真实 polynomial 做固定 world-time 网格采样 + yaw rollout + 三机连续可见性,扩展成 margin 版即可。
5. 事务与 lifecycle:PolyTraj 的 `team_prepare_only`/`team_solution_id`/`lifecycle_state`、traj_server 的 prepared 存储 + speculative queue(ABORT 只删 speculative、保护 Local guaranteed,`markLineageInvalidated` + `normalizeFutureTimeline`)。

**最大的 3~5 个实现风险**:
1. **P+T 联合 SCP 刚被重构移除(重要更正与证据)**:用户的设想与本方案的 TEAM_PT 一样是 P/T 联合优化,且**曾经实现过**——`src_20260918_2027.tar.gz` 备份(09-18 20:27,8280 行版 poly_traj_optimizer.cpp)中存在 `runCandidateHardCorridorSCP`(:1183),QP 维度 `dim = position_dim + virtual_T_dim`(:2635),P 列信任 `trust_p`、T 列信任 `trust_t * current_durations(i)`(带 realTimeJacobianAt 折算),硬行含 corridor/SFC/static/v/a/j,并有 `add_p_trust`/`add_t_trust` 开关;09-19 晨审(9084 行版)确认其为 OSQP 闭环(MINCO→SCP→OSQP→P/T 更新→regenerate→recheck)。**09-19~21 的重构按 feedback_86.md §7 第 1 批第 3 条("删除 flag-off 且无 production 计划的 …hard-SCP…")把它移除了**。当前树 4 个 solveExecutionQP 调用点(:372 P-only/:546 T-only/:863,:1091 T-only)无一联合维度。因此 TEAM_PT 的正确路径不是"从零新组装",而是**从备份恢复该 QP 装配、替换行集(Team margin 行)并换入口**——风险由"高"下调为"中",但需要先确认:该移除是最终决定,还是需要为 Extra Team-SCP 恢复该能力(本审计不改代码,仅暴露)。

**关闭/删除时间线与依据强度(2026-09-22 补充核查)**:
1. 出生即 flag-off:09-18 备份与当前 launch 均 `enable_candidate_hard_corridor_scp default="false"`,runner 默认 `NATIVE_EGOV2_HARD_CORRIDOR_SCP:-false`——它从未进过生产默认。
2. 09-14~15(feedback_65/70):作为动态修复链的一部分开发,并在真实实验运行中打开过(full_encirclement_fix4.log 有 SCP_FINAL_OK / SCP_FINAL_DYNAMIC_VIOLATION);结论 **PARTIAL**("virtual-T row 已接入,但当前冲突窗口仍有失败";"optimizer epoch 与最终动态 clearance 记录之间仍需下一轮专门对齐审查"),未宣称闭环、未转正默认。
3. 09-17~18:为解决"SIDE 硬平面进不了求解器"绕过 flag 强开(MY_LOGIC_ERRORS_AUDIT 错误 2);同运行实测 SCP 路径 jerk p50=16.15 / max=75.83(上限 20/21),差于 NOMINAL p50=12.20 / max=21.00;同时发现真根因是 CandidateSideScope 析构清掉平面快照;根因修复后强开完整撤回(alp_params.sh 留有注释)。**撤回有实测依据,但那是根因 bug 在场时的混杂实验。**
4. 09-19 feedback_85 确认撤回在位;feedback_86 §7 以"flag-off 且无 production 计划"列入删除清单——**删除依据是流程性的,不是新的性能否决**。
5. 09-19~21 重构执行(feedback_087:核心四文件 26,640→19,671 行;`hard_corridor_scp` 在非 test/archive 源码 0 matches)。
**结论**:它不是"验证后被打上不可用",而是"没修完就搁置 + 流程性删除"。对 Extra Team-SCP 而言,恢复后做一次干净验证(根因已修、只开 flag、A/B 对照)是有依据支撑的路径。
2. **同 topology 保持的 authority 偏软**:同一 topology 内做 P/T deformation 时,防止 LEFT→RIGHT 翻转的既有机制是 candidate side region/bias(软,`candidateSideRegionGradCostP`,weight 2.0)+ 有 SFC 平面时的硬平面行;**SIDE 候选无 SFC 平面(plane_count=0)时没有硬 topology authority**。必须在 realized validation 里加 side-sign 复核,否则存在翻转风险(不在 SCP 里偷偷新增机制,按任务书三.3 如实暴露)。
3. **visibility metadata 的重算点**:T 改变后,依赖时间的量(candidate 的 visibility 采样、LOS 平面 active 区间)必须重算;RealizedTeamValidator 本身重新采样真实 polynomial(无此坑),但 candidate-level 缓存(FillCandidateBinaryVisibility 产物)在 Team-SCP 后不可复用。
4. **contract 与 rolling churn 的竞速**:contract 必须跨多个 rolling cycle 存活,绑定(task_reference_generation, snapshot revision, committed-local lineage),绝不能绑定单条 candidate id;否则一次普通 rolling replan 就会把 contract 冲掉。
5. **M2 语义统一**:当前至少存在 4 套 visibility 语义(Local J_vis / Team continuousVisibility / candidate binary / scene 度量)。常数一致(occlusion margin 0.08、hfov 85°)但实现分散、相机契约各读一遍参数。M2 必须指定唯一 authority(推荐 coordinator 的 continuousVisibility 分量族),否则 margin 阈值无从校准。

# 2. Current production path(真实链,含 Existing Local SCP 位置)

```text
run_on.sh(默认 --ablation full) → scripts/run_alp_full_on.sh(RESOLVED CONFIG TRUTH + [ablation-config] 对账)
→ native_egov2_rviz.launch → target_state_coordinator / cooperative_viewpoint_manager(唯一任务几何 authority,
   发布 /cooperative_task_reference + /cooperative_viewpoint/uavN/reference)
→ EGOReplanFSM(100Hz exec) → EGOPlannerManager::reboundReplan
   → 统一 world-time 冲突检测 → ConflictDescriptor → N/L/R
   → target-guided A* + Local-SFC(世界时间锚定平面) → PolyTrajOptimizer
      ├─ LBFGS MINCO 软优化(costFunctionWithinBudget:smoothness+PVA+obstacle+swarm+movingObj
      │   +tracking/radial/height/bearing+local gap+directional J_vis+feasibility)
      └─ 【Existing Local SCP】候选硬化:enforceCandidateLosPlanesSCP(P-only,仅当候选带 SFC 平面,
          poly_traj_optimizer.cpp:1323)→ hard preflight(validateExecutionTrajectory:
          dynamics/static/dynamic/swarm)
   → finalizeCapturedCandidates(batch 冻结统一 activation,同 batch 同边界)→ TopologyCandidateBundle
→ multi_uav_topology_coordinator::evaluateSnapshot → core_->select(27 组合整组评价) → top-K
   → attemptTargetCenteredTeamReference(TeamTargetCenteredOptimizer:R/PHI/PHASE 低维,4 阶段 Q2→ACC→COMP→ENC+REG)
   → TeamReferenceSchedule(STATE_PROPOSAL)→ 三机 planner teamReferenceScheduleCallback
      → Local realization(reboundReplan 家族:predecessor head P/V/A + reference 位移 + MINCO 45ms 预算
        + hard preflight)→ TeamTrajectoryAck(realization 证书)
   → STATE_PREPARE → traj_server 预备存储(不激活)→ TeamTrajectoryAck{phase=EXECUTOR_READY}×3
   → commitPendingTeamProposal:CAS + RealizedTeamValidator(realized vs committed-Local baseline,字典序)
      → STATE_COMMIT → traj_server prepared→committed → common activation → rolling execution
失败路径:ACK_TIMEOUT / CAS stale / REALIZED_TEAM_VALIDATION_REJECT → abortTeamProposal
   (TeamReferenceSchedule::STATE_ABORT)→ traj_server 只清 speculative,Local guaranteed 分支不动。
```

注:coordinator `evaluateSnapshot` 的 legacy Joint 主循环(3242-3486 区段)在 2997 行 `consumeCurrentSnapshot(); return;` 之后,已是死代码。

# 3. Proposed minimal-diff path

```text
(A) Team M2 监测(只读分析,不改轨迹)
  bundle_messages_ 的 committed 轨迹 ×3(或 pending realized)
  → 固定 world-time 网格(activation..activation+H_pred)
  → 每机 m_i(t)=1−2·max(r_static, r_dynamic, r_fov, r_range)(§5)
  → 排序取 M2(t)=m_(2)(t);窗口内 min M2 与首次跌破 m_trigger 的 t_break
  → [TEAM_M2_ANALYSIS] telemetry
       ↓ M2 安全:DO NOTHING(每 cycle 重算)
(B) Handoff Contract(generation/revision 绑定,跨 cycle 存活)
  {outgoing,incoming,preserve_until_world_time,acquire_by_world_time,
   required_overlap,m_required,contract_id,task_reference_generation,
   snapshot_revision,lineage(3×committed traj_id)}
       ↓ 经 /topology_coordination/team_reference_schedule 或新 topic 下发
(C) Extra Team-SCP(planner 侧,一次 refinement)
  起点=本机 realized/committed Local 轨迹(inner pts + durations)
  Mode A TEAM_T_ONLY:固定 P,Δvirtual_t 行(margin 梯度·ds_dT)
  Mode B TEAM_PT:完整行(P+T 列,同一构造器)
  + J_team_soft(仅 contract_active 时)
       ↓ exact MINCO regenerate → 非线性 recheck(margin 实测、动力学、SFC、swarm)
(D) RealizedTeamValidator 扩展:margin 版复核
  incoming 在 acquire_by 前达到 m_required;outgoing 保持到 release;
  overlap 实测 ≥ handoff_overlap_min;critical window M2_realized ≥ 阈值;
  原有 identity/certificates/interaction/lineage 全保留
       ↓ PASS → 既有 PREPARE/READY/COMMIT/activation;REJECT → 事务 ABORT,Local 照旧
```

# 4. Exact source insertion map

| Feature | File | Class/function(当前职责) | Recommended change | Reuse/new | Risk |
|---|---|---|---|---|---|
| M2 预测分析 | multi_uav_topology_coordinator.cpp | evaluateSnapshot(2990 前)/新 `analyzeSecondObserverMargin()` | select 后、Team reference 前插入;输出 telemetry+contract 触发判据 | 复用 continuousVisibility、bundle committed 轨迹 | 低(只读分析) |
| Handoff Contract | 新 `handoff_contract.h`(multi_uav_formation/include)+ 新 msg | 无 | contract 结构、stale 判据、生命周期(绑 task_reference_generation+snapshot+lineage) | new(小) | 中(生命周期) |
| Contract 下发 | multi_uav_topology_coordinator.cpp(planner 订阅区 476-489 旁) | team_reference_publisher_ | 新 topic `/topology_coordination/handoff_contract`(latched)或扩展 TeamReferenceSchedule | new msg | 低 |
| TEAM_T_ONLY | poly_traj_optimizer.cpp 新 `runTeamHandoffMarginTOnly(...)` | 骨架 = runCandidateTimeOnlyFeasibilityCorrection(:398) | 行=margin 梯度·mincoSampleGradientWrtDecision(ds_dT≠0);信任=0.15 median 物理折算;迭代 4-6;非线性 recheck | reuse 骨架 + 行构造器 | 中(T-only 增益不足时的升级判据) |
| TEAM_PT | poly_traj_optimizer.cpp 新 `runTeamHandoffRefinementPT(...)` | 骨架 = enforceCandidateLosPlanesSCP(:225) 的 QP 装配 | 行不截断(保留 T 列)+ T trust box;目标 = min ΔP²+ΔT²;trust P 0.30/T 0.15 | reuse 行构造器+solveExecutionQP+recheck | 高(新组装;topology 保持见 §8) |
| J_team_soft | poly_traj_optimizer.cpp costFunctionWithinBudget(:4883) | native 目标组装 | 在 addDirectionalVisibilityGradCost2CT(:2610 调用点 ~4960)之后,按同一 before/after 快照模式累加;仅 team_contract_active_ 时 | reuse 快照模式 | 低 |
| realized margin 复核 | realized_team_validator.cpp evaluateMetrics | 现算 0-1 可见性 | 增加 per-component margin 与 contract 检查(acquire/release/overlap/M2 window) | reuse | 低 |
| topology 保持 | poly_traj_optimizer.cpp candidateSideRegionGradCostP(:3466)/candidateSideBias | 软 region/偏置 | TEAM_PT 的 trust box 中加入 lateral 半空间行(同一 normal/origin/sign 数据,不加新机制);validation 侧 side-sign 复核 | reuse 数据 | 中 |
| 失败回落 | coordinator abortTeamProposal + traj_server teamSolutionCallback(STATE_ABORT) | 事务级取消(speculative only) | Team-SCP 失败 → 现有 ABORT 路径;Local guaranteed 不动 | reuse(已存在) | 低 |
| escalation | coordinator evaluateSnapshot / bundle 请求 | N/L/R 枚举 | TEAM_PT_INFEASIBLE 时在下一 cycle 的 bundle 评价中提高 SIDE 组合优先级(core_->select tie-break),不新增第二套枚举 | reuse | 低 |

# 5. Visibility margin authority

**推荐 authority:coordinator 的 `continuousVisibility` 分量族 + `tracking_visibility_geometry.h` 的两个 risk 函数**,理由:它是唯一同时被 Team 优化器、RealizedTeamValidator 使用的实现,且四个分量均有解析梯度;Local J_vis(directionalVisibilityGradCostP)与之共享同一批几何原语(querySegmentClearance / segmentVerticalCylinderClearance / FOV risk / range smoothstep),常数一致(occlusion margin 0.08、hfov 85°)。

m_i(t) 推荐定义(组合各分量归一化 risk,均已有梯度):
```
r_static(t): directionalClearanceRisk(clearance; c_in=margin, c_out=margin+transition)
             —— 有 dR/dc,含深遮挡 continuation(κ=1, β=2,梯度不饱和)
r_dynamic(t): 每动态障碍 segmentVerticalCylinderClearance 同型 risk —— 有梯度
r_fov(t): trackingCameraFovDirectionalRisk(horizontal/vertical margin)—— 有梯度,
          依赖 yaw;yaw 用 advanceTargetFacingYaw rollout(rate 2π, accel 5π,executor 同一契约),
          分段光滑,饱和点外可微
r_range(t): 近/远距 smoothstep(continuousVisibility 内,带 ∇range)—— 有梯度
m_i(t) = 1 − 2·max(r_static, r_dynamic, r_fov, r_range) ∈ (−∞,1]:
         >0 可靠余量;=0 边界;<0 不可见(risk>0.5,深遮挡时 r→>1 ⇒ m<−1)
```
- 只能 post-validation 的项:`fillCandidateBinaryVisibility` 的二值相机串(binary_valid)、scene 侧 `visible_N` 度量——它们是真实相机判定,不做梯度,不做 SCP 行。
- 语义漂移(现状如实记录):Local J_vis 的相机契约来自 OPT 自身参数(`visibility_camera_*`),Team/validator/scene 各自从 launch 读 `tracking_camera_*`;数值当前一致(85°/1280×720/0.08),但**两份参数读取来源**,须以 `TrackingCameraContract` 结构体单源化(结构体已存在于 tracking_visibility_geometry.h)。

# 6. Team-SCP design

**Mode A:TEAM_T_ONLY**(变量 Δvirtual_t,维度=piece_num)
- 目标:min ½‖Δτ‖²(最小时间修改;`costFunctionCallback` 的 linear/gradient 传零向量 + QP 目标即最小范数,solveExecutionQP 已支持纯约束+对角 Hessian)。
- Hard rows:`m̄_k + ∂m_k/∂T·Δτ ≥ m_required`,固定 world time 网格(t_k = activation + k·dt);行 = margin 梯度经 `mincoSampleGradientWrtDecision(piece,piece_time,grad_m,0,ds_dT≠0,virtual_t)` 的 T 列;`ds_dT` 语义 = 前序 piece 时长变化对固定 world-time 采样点局部时间的传导(**既有实现**,TimeOnly 路径 :509 已这么用)。
- Trust:Δvirtual_t 逐 piece 盒约束,物理折算 = median(0.15·|J|/T)(与 :436-452 同式)。
- 非线性 postcheck:exact MINCO regenerate → 实测 m(t) + 动力学 lattice + SFC violation(evaluateCandidateLocalSfcMaxViolation)+ swarm 间距。
- 失败:非线性不满足连续 2 次 → 升级 TEAM_PT。

**Mode B:TEAM_PT**(变量 [ΔP;Δτ],维度=3(pieces−1)+pieces)
- **首选实现:恢复备份中的联合 SCP 装配**(`src_20260918_2027.tar.gz` 内 runCandidateHardCorridorSCP,:1183 起;joint `dim=position_dim+virtual_T_dim`、`trust_p`/`trust_t` 双信任、regenerate+recheck 闭环已在),换两处:①行集 = Team visibility margin 行(§6 hard row)+ 既有 side-region 半空间行,替代 corridor 行;②入口由 flag-off 的候选硬化改为 contract 触发的一次性 refinement。**不重写优化器**。
- 目标:min ½(‖ΔP‖²·w_P + ‖Δτ‖²·w_T)(最小 P/T 变形;任务书 246-256)。
- Hard rows:同上,但行不截断(P+T 列全保留);另加:既有 side-region 半空间(同一 origin/direction/sign,把软约束写成行,数据复用、机制不新造)、有 SFC 平面时平面行、动力学 v/a/j 行、P/T trust 盒。
- 非线性 postcheck:同 Mode A + side-sign 复核(见 §8)。
- 失败:`TEAM_PT_INFEASIBLE` → coordinator:`TOPOLOGY_ESCALATED`,下一 cycle 走既有 N/L/R alternative search。

**J_team_soft**(两种 mode 共用,只进 LBFGS 版或 SCP 目标):
- 位置:costFunctionWithinBudget 中 addDirectionalVisibilityGradCost2CT 之后,同一 before/after 快照模式(:4952-4966 的 directional_gdC_before/after 就是模板)。
- 仅 `team_contract_active_=true` 时启用;语义与 hard row 完全同一 margin 函数;权重 §10。
- 不修改 time cost、不动 encirclement/tracking 权重(保守;避免破坏 rolling liveness——B2 教训)。

# 7. T-only vs PT decision rule

```text
deficit = m_required − min_k m̄_k          (k ∈ critical window)
if deficit ≤ 0:                        DO NOTHING(不发 contract、不跑 SCP)
T_gain_bound = Σ_pieces |∂m̄/∂T| · ΔT_trust_physical(piece)
grad_T_norm = ‖∂m̄/∂T‖ 在关键样本
if deficit ≤ T_gain_bound 且 grad_T_norm > ε_rows:   TEAM_T_ONLY
elif 关键样本整段 m̄ < 0(阴影)或 grad_T_norm ≤ ε:   TEAM_PT
else:                                  TEAM_T_ONLY(先试)
失败链:TEAM_T_ONLY 非线性 recheck ×2 失败 → TEAM_PT;
       TEAM_PT infeasible → REALIZED_TEAM_VALIDATION reject → topology escalation。
```
判据全部来自既有量(trust 折算、行范数),第一阶段不引入新启发式森林。

# 8. Interaction with N/L/R

- **same-topology 保持**:topology 的离散权威在候选生成(SIDE seed/A*/SFC),Extra Team-SCP 不重新选择;P/T deformation 期间:L→R 翻转被三道既有机制约束——① side region/bias 软约束(与冲突点同侧的横向带,candidateSideRegionViolation);② SFC 平面硬行(仅当候选带平面;**plane_count=0 的直接 SIDE 无硬 authority——风险如实暴露,需在 realized validation 中复核 side sign**:`sign((p−origin)·direction)` 与 candidate_side_sign_ 一致,不一致即 REJECT);③ validateExecutionTrajectory 硬预检(只管安全,不管拓扑,故②的复核不可省)。
- **infeasible 的定义**:TEAM_PT 在 trust 盒内 QP 不可行,或 QP 可行但非线性 recheck 连续失败(§6),或 realized margin 复核失败。此时不是"每个 Team event 重枚举 27 组合",而是:contract 随事务 ABORT 丢弃;下一 cycle 的 N/L/R 本来就会重新枚举,仅在其中把"当前 topology 无法承担 contract"作为 core_->select 的评价输入(既有 27 组合评价已含 per-candidate visibility,无需新状态机)。

# 9. MINCO / native objective interaction

- 普通 Local MINCO/LBFGS **保持原样**(§13.A 风险的直接回答:Team-SCP 产物走独立 refinement 通道,不回灌 LBFGS;commit 后由 traj_server PREPARE/COMMIT 接管,不存在"再跑一次普通 MINCO 把 Team 约束洗掉"的路径——需在实现时保证 Team-SCP 结果不进入 cps_/jerkOpt_ 的下一轮 nominal)。
- J_team_soft 只在 contract active 时进入一次 refinement 求解,见 §6。
- 现有 tracking/encirclement 权重不动;time cost 不动。

# 10. Parameter recommendation table

| Parameter | Existing related value(源码/launch/日志实测) | Recommended initial | Range | Rationale | Sim calib? |
|---|---|---|---|---|---|
| visibility_prediction_horizon | team_prediction_horizon=2.0;moving_obj_prediction_horizon=2.0;local_horizon=1.5 | 2.0 s | 1.5–2.5 | 复用动态预测权威窗口;H_auth=min(T,2.0) 已是 production 语义 | NO(沿用) |
| visibility_sample_dt(M2 分析) | visibility_sample_dt=0.10;team_reference_sample_dt=0.10 | 0.10 s | 0.05–0.15 | 与 Team 采样一致 | NO |
| team_scp_row_sample_dt | temporal_swarm_sample_dt=0.05;LOS SCP 25 样本/平面 | 0.05 s | 0.03–0.10 | 行密度对齐 swarm T-only | NO |
| m_warn | risk 带:c_out−c_in=transition;occlusion margin 0.08 | m=0.6 | 0.4–0.8 | 1−2r 尺度下进入 risk>0.2 上升段,仅诊断/提前分析 | YES |
| m_trigger | 同上 | 0.3 | 0.15–0.45 | risk≈0.35,离边界尚有两个 cycle 余量(p50 replan 0.11s×多) | YES |
| m_required | — | 0.2 | 0.1–0.3 | 必须低于 m_trigger(滞回),高于 0(边界) | YES |
| m_release(滞回) | — | 0.45 | m_trigger+0.05–0.2 | 触发/释放分离,防抖 | YES |
| handoff_overlap_min | 实测 commit→activation lead p50=0.508 s(p10 0.499) | 0.5 s | 0.4–0.8 | 覆盖实际观察重叠 + 一次 activation lead;**不**随事务延迟放大(事务 p50 18 ms) | YES |
| handoff_stable_duration | visibility_sample_dt=0.10 | 0.2 s | 0.2–0.4 | ≥2 个采样周期连续达标 | YES |
| team_scp_p_trust | LOS SCP trust 0.25–1.5(自适应);team_radius_trust=0.30 | 0.30 m | 0.2–0.5 | 与 Team reference 的半径信任同尺度 | YES |
| team_scp_t_trust_ratio | temporal_swarm_time_trust_ratio=0.15;TimeOnly virtual_trust=0.15 | 0.15 | 0.10–0.25 | 三处独立实现同值,直接沿用 | NO |
| team_scp_max_iterations | LOS SCP=6;TimeOnly=4;team_target_centered_max_iterations=10 | 6 | 4–8 | 对齐 LOS-plane SCP | NO |
| team_soft_weight | weight_visibility_=20.0 | 10.0(=0.5×) | 5–20 | 同一 margin 语义、半个权重,给下降方向不与 hard row 对抗 | YES |
| T_ONLY→PT 阈值 | — | deficit>T_gain_bound | — | 判据式(§7),非常数 | 部分(ε_rows≈1e-8 起步) |
| contract_timeout/stale | team_activation_lead=0.24(实测 lead≈0.5);coordination max_bundle_age=0.30 | 1.0 s | 0.6–1.5 | activation lead 实测 0.5 + prepare/commit 0.02 + 余量;同时绑 generation/revision/lineage 任一变化即 stale | YES |

# 11. Lifecycle / transaction safety

- Team-SCP 产物 = 对**已有 speculative Team realization** 的 refinement(继承同一 team_solution_id/transaction_id 与 lineage),不是新 candidate;expected predecessor 仍绑当前 committed Local(执行激活前不变,Feedback093 契约)。
- 失败路径全部走既有 `abortTeamProposal` → `TeamReferenceSchedule::STATE_ABORT` → traj_server 清 prepared + speculative head + queue 项(`markLineageInvalidated` + `normalizeFutureTimeline`,Local guaranteed 显式保护 `entry.team_speculative`)——**不新增取消机制**。
- `last_published_*`/guaranteed Local 分支:Team-SCP 失败不触碰任何 Local 发布路径(LBFGS nominal 不受影响,Team-SCP 是独立函数,不写 cps_/jerkOpt_ 供下一轮使用的状态)。
- 不得重新引入 planner/executor predecessor authority divergence:Team-SCP 产物只经 PREPARE/COMMIT 通道进 executor。

# 12. Implementation order(按风险)

- **Phase A**:coordinator 加 M2 分析 + `[TEAM_M2_*]` telemetry,零行为变化。验收:FULL 仿真中 M2 时间线与可见性 CSV 一致。
- **Phase B**:Handoff Contract 消息 + 生命周期绑定 + stale 规则;仍不改轨迹。
- **Phase C**:TEAM_T_ONLY(复用 TimeOnly 骨架 + 全行构造器);realized validation 加 margin 检查。
- **Phase D**:TEAM_PT(全行 QP + side 半空间 + trust 盒);validation 加 side-sign 复核。
- **Phase E**:escalation 接线 + J_team_soft(最后加,最多可省)。

# 13. Recommended telemetry

TEAM_M2_MIN、TEAM_BREAK_PREDICTED(t)、HANDOFF_OUTGOING/INCOMING、HANDOFF_ACQUIRE_BY/RELEASE_AT、HANDOFF_CONTRACT_GENERATION、TEAM_SCP_MODE、TEAM_SCP_DELTA_P_NORM、TEAM_SCP_DELTA_T_NORM、TEAM_SCP_ITER、TEAM_SCP_LINEAR_PRED、TEAM_SCP_NONLINEAR_ACTUAL、HANDOFF_REALIZED_OVERLAP、M2_BEFORE/M2_AFTER、CURRENT_TOPOLOGY(kinds)、TOPOLOGY_ESCALATED。不加更多。

# 14. Final recommended design

- **MUST**:M2 分析只读化(A);contract 绑 (task_reference_generation, snapshot_revision, lineage) 且跨 cycle;Team-SCP 只在 contract active 时存在;realized margin 复核进 RealizedTeamValidator;失败走既有事务 ABORT;复用 `mincoSampleGradientWrtDecision` 全行(不写第二套 Jacobian)。
- **SHOULD**:TEAM_T_ONLY 先行;side-sign 复核;J_team_soft 半权重;`TrackingCameraContract` 参数单源化(顺手)。
- **OPTIONAL**:escalation 的 select tie-break 调整;T-only 失败计数持久化。
- **DO NOT DO**:重新实现第二套优化器/可见性定义;恢复 full Cartesian Joint;为 Team-SCP 放宽任何 hard-safety;让 contract 绑单条 candidate id;用事务延迟放大 overlap;把 T-only 实现成"改 waypoint 时间戳"(必须走 ds_dT 固定 world-time Jacobian)。

---

## 强制字段

```
AUDIT_ONLY: YES
PRODUCTION_CODE_CHANGED: NO
PARAMETERS_CHANGED: NO
SIMULATION_RUN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO

EXISTING_LOCAL_SCP_PRESERVED: YES(LBFGS+P-only LOS-plane SCP 为现有生产链,本审计未改)
EXTRA_TEAM_SCP_RECOMMENDED: YES(按 §12 分阶段)
TEAM_T_ONLY_REUSE_PATH: runCandidateTimeOnlyFeasibilityCorrection 骨架(poly_traj_optimizer.cpp:398)+
  mincoSampleGradientWrtDecision(ds_dT≠0,:109)+ trust 折算 :436-452;注意 swarm 版 :607 的 arrival/seed 语义不复用
TEAM_PT_REUSE_PATH: 恢复 src_20260918_2027.tar.gz 内 runCandidateHardCorridorSCP(备份 :1183,
  joint dim=position_dim+virtual_T_dim :2635,trust_p/trust_t 双信任列,regenerate+recheck 闭环)→
  换行集为 Team margin 行 + 换入口为 contract 触发;当前树仅有其行构造器
  mincoSampleGradientWrtDecision(:109)与 P-only(:225)/T-only(:398,:607)残件
M2_COMPUTE_LOCATION: multi_uav_topology_coordinator.cpp evaluateSnapshot 内、core_->select 之后
  (新 analyzeSecondObserverMargin;复用 continuousVisibility + bundle committed 轨迹)
HANDOFF_CONTRACT_OWNER: coordinator 新 handoff_contract 模块(结构体+stale 判据),
  经 latched topic 下发;生命周期绑 task_reference_generation/snapshot_revision/lineage
REALIZED_VALIDATION_LOCATION: realized_team_validator.cpp evaluateMetrics 扩展(margin 版),
  在 commitPendingTeamProposal 既有 REALIZED_TEAM_VALIDATION 块(coordinator :1497)内调用

TOP_5_SOURCE_FILES_TO_CHANGE_LATER:
1. ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp(M2 分析+contract+escalation)
2. ros_ws/src/EGO-Planner-v2/.../traj_opt/src/poly_traj_optimizer.cpp(TEAM_T_ONLY/TEAM_PT/J_team_soft)
3. ros_ws/src/multi_uav_formation/src/realized_team_validator.cpp(margin 版复核+side-sign)
4. ros_ws/src/EGO-Planner-v2/.../traj_utils/msg/(HandoffContract.msg;Ack/Chain 已具备 identity 字段)
5. ros_ws/src/EGO-Planner-v2/.../plan_manage/src/planner_manager.cpp(contract 接收+refinement 触发钩子)

TOP_5_PARAMETERS_TO_ADD_LATER:
1. m_trigger / m_release(滞回,默认 0.3/0.45,需仿真标定)
2. m_required(0.2,需标定)
3. handoff_overlap_min(0.5 s,对齐实测 activation lead,需标定)
4. team_scp_p_trust(0.30 m)与 team_scp_t_trust_ratio(0.15,沿用既有)
5. team_soft_weight(10,需标定)
```
