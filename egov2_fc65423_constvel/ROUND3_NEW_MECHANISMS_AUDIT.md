# 第三份报告源码审计：前两轮之后真正新增的算法/规划机制

只读审计，未修改任何源码、未改参数、未跑仿真。
代码区：`/home/bob/ALP/egov2_fc65423_constvel/`
（注：你给的路径 `/home/bob/ALP/egov2_fc65423_` 不完整；实际完整目录是带 `constvel` 后缀的那个，本报告全部结论以该目录为准。）

时间线来源：git（整个工作区是一个库，HEAD=`8cef241`，2026-08-20，**几乎全部改动未提交**，所以 git diff 不能用作切分依据）
→ 因此改用 **文件 mtime + `feedback/feedback_65..84` 时间序 + 根目录三份根因报告 + 生产 run 日志** 构成真实时间线。

---

## 0. 基线边界（必须先钉死，否则会把旧的东西当新成果）

### 0.1 你列的两轮基线之外，还有一层"已存在但未写进前两份报告"的旧代码

这一点必须诚实说明：**代码区里在 2026-09-14 之前就已经存在大量机制，而且它们已经被 2026-09-10 的 supervisor 结构审计完整记录过**。所以判断"新"不能只看你的前两份报告，还要看这些机制是否在 09-14 之前就已在生产代码中。

已在 **2026-09-14 之前** 存在、**不属于第三份报告新增** 的（附证据 mtime / 文档）：

| 机制 | 证据 |
|---|---|
| 120° 槽位 cooperative viewpoint 生成（`phi0 + 2π·slot/3`，槽位 permutation 搜索） | `cooperative_viewpoint_core.h` 09-11；09-10 结构审计 `ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md:29-40` |
| AdaptiveViewpointGenerator（fixed / uniform-120° / previous-accepted seeds，两尺度 1.5 s / 3.5 s，radius±0.35、height±0.20，`angularDiversityCost`） | `adaptive_viewpoint_generator.h/.cpp` 09-13，但其前身 09-10 报告已述 |
| activation scheduling（`B_stage = max(20 ms, 1.25·max_recent(stage latency))`，`lead = max(min_lead, B_prep+B_joint+margin+25 ms)`） | `activation_schedule.h` 09-11；`feedback/ACTIVATION_LIFECYCLE_REPORT_20260910.md` |
| Joint `{P_i, τ_i, Ψ_i}` + `Q2/K2/J_acc/J_K2-cont/J_blackout/J_div` 团队连续优化器 | `team_visibility_optimizer.*`；09-10 报告已给全部公式 |
| 3-ACK / common activation / 原子提交协议 | `team_solution_commit_contract.h`；09-10 报告 §16-17 |
| K-of-N(k=2) / All3 / safety lattice 团队排序 | `topology_coordinator_core.*`；09-10 报告 §12 |
| `multiviewQuality`（minor pair angles，bad=25°，good=60°，smoothstep）、`angularDiversityCost` | `encirclement_geometry.h` mtime **09-12**；`feedback_41.md` 09-12 |
| persistent recovery target / `team_target_reachability.cpp` | 均为 **09-13** |
| `committed_prefix_joint_contract.h` / `unified_team_transaction.h` / `local_execution_contract.h` / `target_guide_lifecycle.h` / `trajectory_lifecycle.h` | 均为 **09-11 ~ 09-13** |
| **方向性 `J_vis` 连续代价**（`directionalVisibilityGradCostP` / `addDirectionalVisibilityGradCost2CT`，gradient → **P 和 T**） | 09-10 结构审计 §8 已完整描述 `w_vis(R_static²+R_dynamic²+R_fov²)` 与 `tracking_visibility_geometry.h` clearance continuation 公式 |
| `local_visibility_preference.h`（字典序可见性比较，只排序无梯度） | mtime **09-13 12:46** |

➡ **所以"visibility 进入连续优化"本身不是新成果，第一轮基线里就已经是连续优化了**（只是当时报告里写成"最终排序中尝试加入 visibility，但效果有限"）。第三份报告必须换一个说法，见第二节。

### 0.2 生产配置（FULL ON）实测打开的开关

由 `run_round2_full_on.sh` → `run_constvel_gradient_rviz.sh` → `native_egov2_rviz.launch` 解析：

| 机制 | 生产值 | 状态 |
|---|---|---|
| `enable_encirclement_tracking` | true | **开** |
| `enable_cooperative_viewpoint_reference` | true | **开** |
| `enable_visibility_candidate_ranking` / `enable_k_of_n_team_visibility` (k=2) | true | **开** |
| `enable_joint_topology_coordination` | true | **开（只到 tuple 枚举/筛选）** |
| `enable_team_visibility_optimizer` | **false** | **关** |
| `enable_joint_pt_optimization` | **false** | **关** |
| `enable_joint_yaw_optimization` | **false** | **关** |
| `enable_elastic_tracking_region` | **false** | **关（第二轮成果已从生产路径退出）** |
| `enable_candidate_hard_corridor_scp` | **false** | **关** |
| `enable_minco_visibility_cost` | **false** | 关（legacy） |
| `prefix_progress_weight` | **0.0** | **关** |
| `encirclement_bearing_recovery_weight` | **0.08** | **开（新增）** |

生产日志 `sim_run_round2.log`（2026-09-18 18:56）计数：
`SCP_FINAL`=**0**、`scp-hard-corridor`=**0**、`EARLY_JOINT_OPT_ATTEMPT`=**0**、`team-solution-commit`=**0**、
`JOINT_FAILURE_LOCAL_NOOP reason=JOINT_TIMEOUT`≈1254、`LOCAL_DIRECT_COMMIT`=13397、`bearing-recovery-setup`=3567。

**这一条决定了很多结论的写法：Joint P/T/yaw、SCP hard row、Elastic Tracking 目前都不在生产 active path 上。**

---

# 一、合围追踪在第二轮以后新增了什么

## 新增 A【源码已确认 + 生产 active】软方位恢复（bearing recovery）——合围代价第一次获得切向权威

### 原方案
合围模式的 tracking 代价只有**径向死区带 + 高度带**：

```
q = p − o(t) − v_o·t ,  seed = R_target · q*_i
r = |q_xy|,  r* = |seed_xy|
er = band(r − r*, 0.35)          band(e,w) = copysign(max(0,|e|−w), e)
ez = band(q_z − seed_z, 0.20)
J_track = w_track (er² + ez²)
dJ/dp|_xy = 2·er·q_xy/r          ← 只有径向分量
```

### 暴露的问题
切向（方位）方向梯度**恒为 0**；且 `|Δr| ≤ 0.35 m` 时径向梯度也为 0。
`AUDIT_BLUE_RED_YELLOW_ROOT_CAUSE.md`（2026-09-18 16:33）实测目标运动段 t=0–78 s：

```
uav1  半径误差 P50=0.16 m   方位误差 P50=85.3°  P90=107.5°
uav3  半径误差 P50=0.02 m   方位误差 P50=122.3° P90=136.6°
```
并给出第一处语义失效点：
`FIRST_BLUE_BOX_SEMANTIC_FAILURE: poly_traj_optimizer.cpp::trackingGradCostP() 合围分支, dJ_dp.head<2>() = 2.0*er*q.head<2>()/radius; 切线方向梯度恒为 0`。

### 根因（设计层，不是某一行写错）
合围的几何约束是**两维的**（半径 + 方位），但代价只覆盖了半径一维。方位维没有任何恢复力，于是：
- rolling 每轮从上一版 active 轨迹取起点、P/V/A 交接连续 → 切向偏差被 "免费" 继承，**逐步累积漂移**（uav1 方位误差从 −3.5° 单调涨到 +115°）；
- 径向带内 `er=0` → 连径向也没有力 → 半径"回来了"、方位"回不来"。

### 新方案（2026-09-18，报告 16:33 定位 → 代码 18:45 落地）
在合围分支加入**风险自适应角隙 + 死区**的软方位项，参考方位取自稳定的编队种子（不是当前位置），并**对真正有执行权的 rolling 前缀加权 ×2**：

```cpp
// poly_traj_optimizer.cpp:6659-6703
φ      = atan2(q.y, q.x)
φ*     = atan2(seed.y, seed.x)          // seed = R_target · q*_i，固定编队种子
e_φ    = wrap(φ − φ*)
ρ      = clamp(elastic_risk_profile_[i_dp].risk, 0, 1)
band_φ = elasticAngularSlack(ρ, true, 0°, 45°, 20°) = 20° + 25°·ρ
e_φ,eff= max(0, |e_φ| − band_φ)
in_prefix = (prefix_execution_samples_ > 0 && i_dp < prefix_execution_samples_)
w_φ    = wei_tracking_ · encirclement_bearing_recovery_weight_ · (in_prefix ? prefix_bearing_recovery_boost_ : 1)
bearing_cost = w_φ · e_φ,eff²
```

### 公式汇总

```
J_bearing,i = w_φ(ρ, i) · [ max(0, |wrap(φ_i − φ*_i)| − band_φ(ρ)) ]²

band_φ(ρ) = band_min + ρ·(band_max − band_min),   band_min = 20°, band_max = 45°
ρ         = clamp(max(risk_state.risk), 0, 1)     // 冻结的 elastic 风险剖面，LBFGS 前算好

w_φ = w_track · w_bearing · boost(i)
      w_track = 100.0（advanced_param.xml:343）
      w_bearing = 0.08（launch 默认，manager/encirclement_bearing_recovery_weight）
      boost(i) = 2.0  当 i 落在执行前缀采样点内，否则 1.0

切向梯度：
∂φ/∂q = ( −q_y/r² , q_x/r² , 0 )
∂J_bearing/∂p|_xy = w_φ · 2·sign(e_φ)·e_φ,eff · ∂φ/∂q      ← 纯切向

总 tracking 代价与梯度：
costp  += w_track·(er² + ez²) + bearing_cost
gradp  += w_track·[ 2·er·q_xy/r + ∂J_bearing/∂p ]
gradt  += w_track·dJ_dp·(v − v_o)                    // 进 τ_i
grad_prev_t += w_track·dJ_dp·(− v_o)                 // 进 τ_{i−1}
```

**真正改变的变量：MINCO 控制点 P（`jerkOpt_.get_gdC()`）与 piece 时间 T/virtual-T（`gdT`）。yaw 不受影响。**

### 执行前缀（"只有前缀有执行权"的量化）
```
prefix_execution_span_    = local_activation_margin_ + execution_margin_
                          = 0.10 + 0.15 = 0.25 s
prefix_execution_samples_ = ceil(prefix_execution_span_ / sample_dt)
sample_dt                 = piece_length / (cps_num_prePiece · max_vel)
                          = 1.5 / (5 · 3) = 0.1 s
```
实测日志：`[bearing-recovery-setup] drone=0 w_bearing=0.080000 prefix_boost=2.000 prefix_span=0.250000 sample_dt=0.100000` ×3567。

### 当前作用 / 实现位置
- `traj_opt/src/poly_traj_optimizer.cpp:6659-6706`（`addPVAGradCost2CT` 合围分支）
- setter `:7659-7675 setEncirclementBearingRecovery`；参数 `:7663-7674`
- 调用侧 `plan_manage/src/planner_manager.cpp:377-384`（读参）、`:5728-5748`（接线 + `[bearing-recovery-setup]`）
- launch `multi_uav_formation/launch/native_egov2_rviz.launch:28, 723-725`
- header 成员 `traj_opt/include/optimizer/poly_traj_optimizer.h:313-316`

### 诚实标注（源码缺陷，只作为证据不作为成果）
`bearing_cost = w_φ·e²` 中 `w_φ` 已含 `wei_tracking_`，而写进 `dJ_dp` 的梯度在 `:6704` 又被乘了一次 `wei_tracking_`，
即**实际施加的导数是被计费代价导数的 100 倍**（径向项没有这个不一致）。这解释了实测"提交数 340→4"式的线搜索敏感性风险；作者注释 `:6656-6657` 表明他知道代价/梯度必须一致。按你的要求，这属于工程 bug，只作证据。
另有陈旧注释与日志未同步：`:6643 "no bearing prior"`、`:6728 "No bearing prior."`、启动日志 `:7429 bearing_prior=0`。

---

## 新增 B【源码已确认 + 生产 active】合围缺口的 **Local 版本**（用冻结邻机在局部代价里算 joint 的同一目标）

### 原方案
25°/170° 的"圆周缺口"（circular gap）合围判据只存在于团队层（`encirclement_geometry.h` 的 `circularGapGeometry` / `encirclementGeometryCost`），Local 层无法感知它。

### 问题
Local 与 Joint 的目标不一致：Local 只管自己的径向带，团队几何要靠 Joint 事后修；而 Joint 在生产里被关掉（见新增 D）。于是"局部看起来很好、团队几何很差"。

### 新方案
把**同一个空扇区代价**直接放进 Local 的 tracking 代价，邻机用**冻结的 peer polynomial** 在同一世界时刻采样：

```cpp
// poly_traj_optimizer.cpp:6729-6764
relative[drone_id_] = q;  velocities[drone_id_] = v;
for (peer : *swarm_trajs_) {
    state = trajectory_lifecycle::sample(peer.traj, t_now_ + t − peer.start_time);
    relative[peer.id] = state.p − o(t) − v_o·t;   velocities[peer.id] = state.v;
}
if (all ready && circularGapGeometry(relative).finite) {
    gap_cost = encirclementGeometryCost(relative, encirclement_min_gap_, encirclement_max_gap_, &gap_gradient);
    explicit_time = Σ_id gap_gradient[id] · ( −v_o )  +  Σ_{id≠self} gap_gradient[id] · velocities[id];
    costp        += encirclement_local_gap_weight_ · gap_cost;
    gradp        += encirclement_local_gap_weight_ · gap_gradient[drone_id_];
    gradt        += encirclement_local_gap_weight_ · ( explicit_time + gap_gradient[drone_id_]·v );
    grad_prev_t  += encirclement_local_gap_weight_ · explicit_time;
}
```

### 公式
```
bearings[i] = atan2(rel_i.y, rel_i.x)
按 bearing 排序得到 order[]
gaps[j] = bearings[order[(j+1)%3]] − bearings[order[j]]  (+2π 当 j=2)
J_gap = Σ_{j=0..2} [ max(0, gaps[j] − θ_max)² + max(0, θ_min − gaps[j])² ]
θ_min = 25° ,  θ_max = 170°
∂J_gap/∂rel_i = (i==a ? −2 : +2)·(hi − lo)·(−rel_i.y, rel_i.x, 0)/|rel_i,xy|²

Local 权重 encirclement_local_gap_weight_ = 100.0（/encirclement_geometry/local_gap_weight）
```
**没有等角奖励、没有 120° 先验。** 只有区间约束。

### 实现位置
`poly_traj_optimizer.cpp:6725-6764`；参数 `:7418-7430`；launch `native_egov2_rviz.launch:136-149`（`min_angular_separation_deg=25`、`max_circular_gap_deg=170`、`local_gap_weight=100`）。
置信度：【源码已确认 active】；**具体落地日期：根据调用关系与代码注释推断为 09-14 之后**（`encirclement_geometry.h` 本身 09-12，但把它接进 Local 的分支属于后加）。

---

## 新增 C【源码已确认】NOMINAL 与 team viewpoint 正式分离 + zero-progress guard + NOMINAL warm start 语义收紧

### 原方案
`teamReferenceGoal(local_target_pt_, local_target_vel_, …)` 原地覆写正常 rolling local target；hypothesis-0 可能走 accepted warm start 或 recovery guide。

### 问题（feedback_73 / feedback_74 / feedback_78）
1. cooperative viewpoint 越权替代正常 local target（`NOMINAL_TARGET_OVERWRITE`）；
2. `teamReferenceGoal()` 写出与起点几乎重合的参考 → 产生亚秒级无用轨迹 → successor 供给断档；
3. hypothesis-0 NOMINAL 被旧 SIDE/guide/team geometry 污染（`NOMINAL_WARM_START_OVERFIX_EXISTED: YES`）。

### 新方案
```
planner_manager.cpp:5421-5436   teamReferenceGoal() 只写 team_reference_target / team_reference_vel / team_relative_tracking
ego_replan_fsm.cpp:1057-1080    normal_local_target / normal_local_target_vel 独立保存
ego_replan_fsm.cpp:1217-1220    hypothesis-0 → normal_local_target；只有显式 alternative 才加 (reference − current_reference)
ego_replan_fsm.cpp:1222-1229    zero-progress guard:
                                  if (target − start).norm() ≤ visibilityMinTargetDistance()  → DROP_ALTERNATIVE
                                  visibility_min_target_distance 默认 0.20 m
planner_manager.cpp:5810-5835   nominal_warm_cache = nominal_baseline && accepted_state_cache_kind_ == NOMINAL
                                warm_start_allowed = (nominal_warm_cache || !nominal_baseline) && acceptedWarmStartEligible(...)
                                acceptedWarmStartEligible = !poly_init && !team_ref && !recovery_guide
planner_manager.cpp:5870-5872   hypothesis-0 不用 recovery guide
```
不变量（feedback_74 实测）：`NOMINAL_TARGET_OVERWRITTEN_COUNT: 0`（980 个 `[nominal-baseline-semantics] source=AUTHORITATIVE_PVA_TO_LOCAL_TARGET`）；`TEAM_VIEWPOINT_ZERO_PROGRESS_REPLAN_COUNT: 4`。

**实现位置**：`ego_replan_fsm.cpp:1057-1080, 1217-1229`；`planner_manager.cpp:495-496, 5421-5436, 5810-5835`。
【源码已确认】。标识符 `NOMINAL_WARM_START_OVERFIX` 在源码中**不存在**，只是 feedback_78 的结论标签。

---

## 新增 D【源码已确认 + 生产 active】团队评估视野 H_eval 从"固定契约"降为"实际公共已验证前缀"

### 原方案
`team_optimizer_params_.horizon` 固定 **1.5 s**（编译期缺省），selector 用 `current_available_end < evaluation_start + 1.5 s` 做准入。

### 问题
单机后继的真实验证覆盖只有 **P50 0.728 s / MAX 0.788 s**（100% < 1.5 s）。
→ 判据恒成立 → `CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE` / `NO_CONSTRUCTIBLE_TUPLE` 是**结构性必然**，不是偶发。
`AUDIT_BLUE_RED_YELLOW_ROOT_CAUSE.md` 量化：`team-reference-feedback 总反馈=1627，NO_USABLE_TEAM_REFERENCE=1627 (100%)`；
`FIRST_YELLOW_BOX_COMMON_FAILURE: topology_coordinator_core.cpp:521`。
→ **本应负责"团队方位修正"的通道整体长期不可用**（这也解释了为什么新增 A 是必须的）。

### 新方案
```cpp
// topology_coordinator_core.cpp:516-540
H_common = current_available_end − evaluation_start      // 三机取 min 之后
H_eval   = min(H_configured, H_common)
constexpr double kMinimumJointEvaluationHorizon = 0.20;  // 低于此才报 INSUFFICIENT_COVERAGE
selection.configured_evaluation_horizon  = configured_horizon;
selection.common_validated_horizon       = common_horizon;
selection.effective_evaluation_horizon   = evaluation_horizon;
selection.horizon_clamped_to_common_prefix = (H_eval < H_configured − 1e-9);
```
提案侧同步回写真实用到的视野（`multi_uav_topology_coordinator.cpp:1831-1837`）：
```
proposal.evaluation_horizon = selection.effective_evaluation_horizon > 1e-6
                              ? selection.effective_evaluation_horizon
                              : team_optimizer_params_.horizon
```
设计边界同时被钉死（注释 `:513-514`）：**Joint 只是后台 refinement —— 不延长 Local 轨迹、不降低安全验证、不阻塞 Local、也不恢复固定 1.5 s 门限。**

### 效果与当前作用
`CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE` 从 09-16 的 14–20 次/run 降到 **0**。
**但它并不能让 Joint 跑起来**——Joint 优化器本身仍是 flag off（见下）。

### 实现位置
`multi_uav_formation/src/topology_coordinator_core.cpp:506-540`；`include/multi_uav_formation/topology_coordinator_core.h:166-170`；提案回写 `multi_uav_topology_coordinator.cpp:1831-1837`。
【源码已确认】。

---

## 新增 E【源码已确认 —— 但这是"损失"而非成果】Joint `{P,τ,Ψ}` / 3-ACK / 原子提交在生产配置中被关闭

第二轮基线的核心之一"使用已有 SCP/OSQP 做时间错峰 / Joint P/T/yaw"在当前生产配置下 **不可达**：

```
multi_uav_topology_coordinator.cpp:449  requested_team_optimizer = enable_team_visibility_optimizer_ && enable_joint_pt_optimization_;
                              :471        team_optimizer_ 从不构造
                              :992-995    attemptTeamOptimization() 立即 return false
planner_manager.cpp:2452-2453             teamTrajectorySolutionCallback() 立即 return
```
启动日志：`[team-vis-opt-config] enabled=0 joint_pt=0 runtime_ready=0 fallback=legacy_selector`。
生产 run 计数：`EARLY_JOINT_OPT_ATTEMPT=0`、`TEAM_VIS_OPT_ATTEMPT/SUCCESS=0`、`team-solution-commit=0`、`TRANSACTION_FINAL_DECISION=0`；
每周期以 `JOINT_FAILURE_LOCAL_NOOP reason=JOINT_TIMEOUT`（~0.20 s pending）结束，随后 `EARLY_JOINT_FALLBACK_TO_LOCAL reason=NO_PRIMARY_JOINT_PROPOSAL_WITHIN_BUDGET`。
仍然在跑的是**廉价的 JOINT_SEED tuple 枚举/筛选**：`TEAM_TOPOLOGY_TUPLE_ENUMERATED`(78)、`TEAM_TOPOLOGY_TUPLE_SCREENED`(541)、`JOINT_TOPOLOGY_SELECTED`(543)。

➡ 第三份报告如果写"团队连续优化/时间错峰"，必须同时声明**当前生产配置下它为 off**，否则是失真。

---

# 二、可见性最大化在第二轮以后新增了什么

## 2.0 先把"只用于 ranking"和"真正进连续优化"分开

| 层 | 是否连续优化 | 改哪个变量 | 生产状态 |
|---|---|---|---|
| Local 方向性 `J_vis`（`R_static²+R_dynamic²+R_fov²`，`weight_visibility=20`） | **是** | **P + T** | **开**（52113 条 `[directional-visibility-cost]`），但**这属于第一轮基线** |
| Local `addStaticLosGradCost2CT`（legacy `minco_visibility_lambda=150`） | 是（若开） | P + T | **关**（`enable_minco_visibility_cost=false`） |
| Local **方位恢复**（新增 A） | **是** | **P + T（切向）** | **开**（`w=0.08`） |
| Team `TeamVisibilityOptimizer` 全目标（`J_K2 / J_acc / J_K2-cont / J_blackout / J_dev / J_target-dist / J_jerk / J_multiview / J_div / J_gap / J_recovery`） | **是（代码）** | **P + virtual-T + yaw knot** | **关**（两个 flag 均 false；即使开，`optimize_yaw=false` + `yaw_trust_radius=0` 把所有 yaw knot 钉死） |
| 团队排序 `topology_coordinator_core::better()` / `local_visibility_preference.h` / `evaluateCandidateVisibility` | 否（只排序/触发） | 不改变量 | **开** |
| `sharedVisibilityTopologyTrigger`（由软代价触发 SIDE 拓扑） | 否 | 不改变量 | **已被切断**（见新增 F） |

## 新增 F【源码已确认 + 生产 active】设计规则：**软可见性代价不得拥有离散 topology 权限**

### 原方案
`planner_manager.cpp` 旧 `6314-6339`：把"方向可见性代价 > 1e-9"的梯度极大值点做一次 `queryStaticLosClearance`，
就把它升级成 `static_visibility_witness_valid` → 生成 `CONFLICT_LOS_OCCLUSION` → 触发完整 LEFT/RIGHT observation plane 与 SIDE 搜索。

### 问题
实测 `directional_static=0.02836`、甚至纯 FOV 的 `directional_fov=2475.55`（而 `static_los=0 dynamic_los=0 fov=0`）都能走通这条路。
→ 一个**连续小量代价**获得了**离散拓扑权威**，产生大量无收益的大横移：
`SIDE 候选求解次数 1499`、`SIDE_DEV_GT_1M_COUNT 137`、`FINAL_NOMINAL_DEV_P90 0.906 m`。

### 根因（设计层）
**连续软代价的作用域是"轨迹形状"，不是"冲突语义"。** 把 `J_vis > ε` 当作"存在遮挡事件"，等于用代价幅值冒充几何真值。

### 新方案
```
planner_manager.cpp:6538-6566
  soft cost 分支只做诊断与计数：[soft-visibility-continuous-only] action=NO_TOPOLOGY_AUTHORITY
  新增 soft_vis_nonzero_count_
static_visibility_witness_valid 只可能来自 raw_static_los_blocked（真实遮挡真值）
spatial_visibility_trigger = (static_visibility_witness_valid && static_visibility_current) || dynamic_visibility_witness_valid
task_topology_trigger      = body_risk_triggered || static_body_support_active || spatial_visibility_trigger
```
**`J_vis` 本身完全保留，继续进入 MINCO 连续优化**（FIX4 报告原文：`J_vis 本身完全保留，继续进入 MINCO 连续优化`）。

### 效果
`SIDE 候选求解 1499 → 7`；`SIDE_DEV_GT_1M_COUNT 137 → 3`；`FINAL_NOMINAL_DEV_P50 0.100 → 0.022 m`。

### 实现位置
`plan_manage/src/planner_manager.cpp:6528-6566`（含 `:6485-6504` 的 `sharedVisibilityTopologyTrigger` 仍会把软诊断并入 `visibility_support_active`，所以是"witness 层切断、trigger 分支未完全切断"）；
`local_visibility_preference.h:32-52`。
【源码已确认】。**这是一条设计规则，不是 bug 修复**：软可见性 = 形状自由度；硬可见性（真实遮挡 witness）= 拓扑权威。

## 新增 G【源码已确认 + 生产 active】排序器窗口与真实公共前缀对齐；可见性排序真正生效

- 排序使用的评价视野改为 `H_eval`（新增 D），团队排序不再因 1.5 s 门限整体不可用。
- 单机候选排序显式使用可见性：`planner_manager.cpp:10948-10985`
  gate = `(enable_visibility_candidate_ranking_ || enable_k_of_n_team_visibility_) && los_descriptor_active_for_selection && selected_safety_rank > 0`
  reason = `LEXICOGRAPHIC_VISIBILITY_IMPROVEMENT`；并在 `:1602-1608` 无条件用于 `finalizeCapturedCandidates`。
- 比较器：`local_visibility_preference.h:54-81 betterLocalVisibilityCandidate`，字典序
  `none ↓ → atleast2 ↑ → (protect_atleast_k ? atleast_k ↑) → mean_visible_count ↑ → all3 ↑ → min_uav_visibility ↑ → max_loss_duration ↓ → diversity ↑ → min_pairwise_angle_deg ↑ → team_utility ↑`，eps 1e-9。

## 明确"未确认/不存在"（不要写进报告）
- **硬 K≥2 约束**：不存在。K2 只以软代价 + acceptance + 排序出现；`appendConstraints` 里没有 coverage 行。
- **All3 / weakest-camera 的连续目标项**：不存在。只有 metric + acceptance + 排序（`binary_weakest_camera`、`binary_all3`），目标函数（`.cpp:1339-1346`）里没有对应权重项。
- **yaw 作为 Local 决策变量**：不存在。yaw 是输入，由 `advanceTargetFacingYaw` 递推；执行侧 `traj_server.cpp` 用 `optimized_yaw_at(t)`，无有效 optimized yaw 时回退 target-facing（默认 true）。
- **团队 yaw 在生产中可动**：不存在。`optimize_yaw=false` 不写 yaw 梯度，且 `yaw_trust_radius=0.0` 把所有 yaw knot 用等式行钉在初值。
- **blackout / K2-continuity 的连续梯度**：默认权重为 0（`enable_team_k2_continuity_cost=false`、`enable_team_blackout_cost=false`），即使 Joint 被打开也不产生梯度。
- `camera_time_saturation_fraction`：dead（构造里 clamp 后从无读取）。`enable_acceptance_guard`：dead（声明无引用）。

---

# 三、三种威胁统一在第二轮以后新增了什么

## 新增 H【源码已确认 + 生产 active】统一冲突表示：一个 `ConflictDescriptor`，两路证据

```cpp
// plan_manage/include/plan_manage/planner_manager.h:40-91
enum ConflictReasonMask { CONFLICT_NONE = 0, CONFLICT_BODY_SAFETY = 1, CONFLICT_LOS_OCCLUSION = 2 };
enum class ConflictObstacleMotion { UNKNOWN = 0, STATIC = 1, DYNAMIC = 2 };

struct ConflictDescriptor {
  bool valid; unsigned int reason_mask; ConflictObstacleMotion obstacle_motion;
  int obstacle_identity; int primitive_type; std::string obstacle_name;
  unsigned long geometry_revision; double source_snapshot_time;
  double first_risk_world_time, risk_interval_start, risk_interval_end;
  // BODY 证据（独立通道）
  double body_conflict_time, body_first_risk_world_time;
  int    body_obstacle_identity;  ConflictObstacleMotion body_obstacle_motion;
  Eigen::Vector3d body_obstacle_position;
  // LOS 证据（独立通道）
  double los_conflict_time, los_first_risk_world_time;
  int    los_obstacle_identity, los_primitive_type;
  ConflictObstacleMotion los_obstacle_motion;
  Eigen::Vector3d los_obstacle_position;
  // 观测几何（LOS 用）
  Eigen::Vector3d observer_position, target_position, frame_forward, frame_right;
  Eigen::Vector3d obstacle_position, obstacle_geometry, clearance_witness;
  double clearance, radius;
};
```
头文件注释即设计契约：*"BODY and LOS evidence are intentionally kept separate."*

**统一了什么**
- 冲突检测入口：静态 LOS（`StaticLosGeometry::querySegmentClearance`）、动态 LOS（`segmentIntersectsVerticalCylinder`）、动态 BODY（`evaluateDynamicRisk`）、静态 BODY（`grid_map_->getInflateOccupancy`）全部写同一个 `nominal_result.conflict`；
- 一个 N/L/R 循环（`optimize_side(side,…)`）、一个 `optimize_side()`、一个 `classify_candidate`；
- 一个 `LocalSfcPlane` 类型，靠 `source` 区分 `STATIC_COLLISION_CORRIDOR=0` / `LOS_OBSERVATION_SIDE=1`；
- 一套 `blocker / conflict_time / world-time / frame / active interval` 词汇；
- 一套提交期平面校验（`validateRetimedLocalSfc`）；
- 一个 manager 侧世界时基（`planning_prediction_epoch` + `targetPositionAt`）。

**哪些物理语义仍然必须分开（不要写成"三类统一成同一约束"）**

| | BODY | LOS |
|---|---|---|
| 事件类型 | **点事件**（最近接近点） | **区间事件**（遮挡区间） |
| 时间窗 | `conflict_time ± kGuidanceWindowSeconds(0.45 s)` | 真实遮挡区间 `[enter, exit]` |
| 几何 | 三维中心距 / 平面法向距离 | 射线与竖直圆柱/墙求交 → 遮挡锥切线半空间 |
| 坐标系 | 路径相对：`(local_target_pt − start_pt) × ẑ` | 目标-遮挡物相对：`frame_right` |
| 选择权威 | clearance + 迟滞（`candidate_switch_margin_`） | 仅可见性排序（且不得顶掉有效 NOMINAL） |
| 分类 | `classify_candidate` 用 dynamics/static/swarm/risk | **`classify_candidate` 里没有任何 LOS 项** |
| 终检 | `validateExecutionTrajectory` | 独立的 `validateRetimedLocalSfc` |

---

## 新增 I【源码已确认 + 生产 active】LOS 观测侧拓扑：遮挡锥切线半空间 + 真实遮挡区间

### 原方案
LOS 观测面沿用 BODY 的指导窗 `risk.conflict_time ± 0.45 s`。

### 问题（`LOS_OCCLUSION_ROOT_CAUSE.md`，2026-09-18 15:13）
动态障碍 id=5 在 `t=1.8 s` 遮挡，但 `conflict_time` 已被写成 0，窗口被压成 `[0, 0.45]`：
```
[side-space] drone=0 candidate=PLUS obs=5 conflict_window=[0.000,0.450]
[observation-topology-plane] candidate=SIDE_PLUS  blocker=5 side=1  status=WINDOW_LIMITED
[observation-topology-plane] candidate=SIDE_MINUS blocker=5 side=-1 status=RECOVERY_NOT_REACHED
[los-plane-lifecycle] stage=BEFORE_SCP los_plane_count=0 action=NO_PLANE_FOR_CANDIDATE
```
→ 观察面从未覆盖真实遮挡区间 → 平面被放弃 → **提前 1.8 s 已知的遮挡按原样发生**。
根因（设计层）：**区间事件不能沿用点事件的窗口。**

### 新方案 —— blocker-target frame 与左右法向
```cpp
// planner_manager.cpp:7719-7739
e        = (blocker_xy − target_xy) / |blocker_xy − target_xy|    // 水平前向单位向量
n_right  = ( e.y , −e.x )                                          // 右法向
α        = asin( min(1, radius / distance) )                       // 遮挡锥半角
m2       = −sin(α)·e + side·cos(α)·n_right ,   side ∈ {+1,−1}      // 观测侧法向
observation_normal = ( m2.x , m2.y , 0 )
observation_point  = ( target.x , target.y , 0 )
```
（`frame_forward = (e.x,e.y,0)`、`frame_right = (e.y,−e.x,0)` 在 `:6638-6646` 静态 / `:6811-6818` 动态写入描述符。）

### observation-side inequality
实现的是**无度量余量的半空间**（`observation_plane.clearance = 0.0`，`:7812`）：
```
nᵀ (p − p0) ≥ 0
把 d = p − p0 = |d|( cosθ·e + sinθ·n_right ) 代入：
nᵀd = |d|·sin(side·θ − α) ≥ 0   ⟺   side·θ ≥ α
```
即**遮挡锥在目标处的切线半空间**；半径只通过 α 进入，不产生米制偏移。

### active interval（真实遮挡区间 ∩ 到达观察侧区间）
```cpp
// planner_manager.cpp:7337-7400 + 7772-7791
los_occlusion_window_valid = los_observation_geometry_valid && raw_los_occlusion_interval_observed_
                             && isfinite(enter) && isfinite(exit)
occlusion_start = enter ;  occlusion_end = exit           // 来自 raw LOS 扫描
los_side_reach_deadline = occlusion_start                 // 必须在遮挡开始前到达所选侧
enter_time = max(occlusion_start, reached_side ? side_reached_time : occlusion_start)
exit_time  = side_left_time >= 0 ? min(occlusion_end, side_left_time) : occlusion_end
若 reached_side == false 或无时间重叠 → WINDOW_LIMITED / RECOVERY_NOT_REACHED，放弃该平面
exit 超出扫描视野时截到 max(nominal_duration, scan_horizon)，绝不因此删面
```
raw LOS 扫描（`:6248-6446`）：
```
sample_world_time = planning_prediction_epoch + relative_time
observer          = nominal trajectory 位置
target            = target_at_prediction_epoch + object_vel·relative_time
relative_time ∈ [0, raw_los_interval_scan_horizon],  raw_los_interval_scan_horizon = max(2T/3, 2.0)
静态: clearance = radial_distance − radius,  blocked 当 clearance ≤ visibility_occlusion_margin_(0.08)
动态: radius = 0.5·max(scale.x,scale.y) + 0.08,  布尔求交
```

### 对 P / T 的作用位置与 SCP 线性化行
```cpp
// poly_traj_optimizer.cpp:2327-2372（Local SFC / SCP 行）
h        = clearance − n·(p − p0)                        // clearance = 0
grad_h   = − mincoSampleGradientWrtX(piece, τ, n, SAMPLE_POSITION, ds_dT)
row:       grad_h · Δx ≤ − h        ⟺   n·(p−p0) + (−n)·ΔP ≥ clearance
```
虚拟时间雅可比：
```
ds_dT(d) = global_α − [d < piece] ,   global_α = t / total_duration
```

### 重定时不改变语义（新增）
`active_start`/`active_end` 在优化前按**初始总时长**归一化成比例，SCP 每轮再乘当前 `total_duration`：
```cpp
// poly_traj_optimizer.cpp:1474-1484, 2336-2343
local_sfc_ratios[k] = ( clamp01(plane.active_start / initial_total_duration),
                        clamp01(plane.active_end   / initial_total_duration) )
active_start = max(0, ratio.first  · total_duration)
active_end   = min(total_duration, ratio.second · total_duration)
```
→ 时间自由度自由缩放时，LOS 观测面始终贴住真实遮挡区间（这是"时间错峰"和"LOS 语义"能共存的前提）。

### 实现位置
`plan_manage/src/planner_manager.cpp:6248-6446`（raw 扫描）、`7337-7400`（窗口语义）、`7710-7815`（观测面构造）、`7817-7885`（审计/丢失判定）；
`traj_opt/src/poly_traj_optimizer.cpp:2327-2372`（SCP 行）、`182-212 evaluateCandidateLocalSfcMaxViolation`（生产实际执行的平面检查）；
`plan_env/include/plan_env/static_los_geometry.h:188-437`（静态 LOS 几何 + witness）。
【源码已确认】。

### 诚实标注（bug，仅作证据）
- 遮挡区间对**所有 blocker 取并集**（`enter = min first_contact`、`exit = max last_contact`），两个不同 blocker 的两段不相交遮挡会被合并成一个窗口；
- `geometry_revision` 声明后从未赋值；`clearance_witness` 只在静态 LOS 分支写；**动态 BODY-only 时 primary witness 字段永不填充**（`:6760-6784` 被 `:6719-6733` 抢先置位后成为不可达分支，日志证据 `reason_mask=1 primary_motion=0 primary_id=-1`）。

---

## 新增 J【源码已确认，但**生产 flag 关闭**】dynamic BODY 的 SCP 硬行 + 物理/偏好净空分离

### 原方案
动态障碍只有软代价（`moving_obj_clearance_ = 1.1 m` 作为偏好避让触发/软修复目标），硬可行性没有优化器行。

### 问题（feedback_72）
把 1.1 m 偏好余量当物理执行门 → 过度保守；且静态 `grid_map/obstacles_inflation=0.099 m` 被错误复用为动态机体半径。

### 新方案
```
物理硬净空（execution gate）:
  d_dynamic_hard = 部署机体水平footprint + 0.5·max(live obstacle marker scale.x, scale.y)
  poly_traj_optimizer.cpp:7616-7629 getMovingObjHardClearance()
    = moving_obj_hard_body_radius_(0.384) + max_over_objects( 0.5·max(scale.x,scale.y) )
  （scale 非法时返回 0.384 + 0.5）
偏好余量（soft objective / trigger）: moving_obj_clearance_ = 1.1 m  ← 不变

SCP 硬行（poly_traj_optimizer.cpp:2424-2499，位于 runCandidateHardCorridorSCP 内）:
  dynamic_horizon = min(total_duration, moving_obj_prediction_horizon_ = 2.0)
  activation_margin = max(0, elastic_moving_risk_margin_ = 0.80)
  对每个采样 t = α·dynamic_horizon：
     obstacle(t) = evaluateConstVel(id, t_now_ + t)          // 注意是 t_now_，见下
     distance    = |p − obstacle|
     if distance > moving_obj_clearance_ + activation_margin  → skip
     normal      = (p − obstacle)/distance
     grad_distance = mincoSampleGradientWrtX(piece, τ, normal, SAMPLE_POSITION, ds_dT)     // P 雅可比
     obstacle_time_derivative = − normal·obstacle_velocity
     grad_distance[position_dim + d] += obstacle_time_derivative · ds_dT(d) · realTimeJacobianAt(virtual_t_d)   // T 雅可比
     clearance_margin = distance − getMovingObjHardClearance()
     row:  − grad_distance · Δx ≤ clearance_margin     ⟺   d + ∇d·Δx ≥ d_hard
  终检: maxDynamicBodyViolation > 2.0e-3 → SCP_FINAL_DYNAMIC_VIOLATION（:1532-1559, :3613-3620）
```

### 当前状态（必须写清）
```
dispatch: poly_traj_optimizer.cpp:1001  if (candidate_hard_corridor_scp_enabled_ && side_candidate)
flag:     optimization/enable_candidate_hard_corridor_scp 默认 false
          advanced_param.xml:111 / run_in_sim.launch:47 / native_egov2_rviz.launch:253
          run_constvel_gradient_rviz.sh:37  NATIVE_EGOV2_HARD_CORRIDOR_SCP:-false
生产 run: SCP_FINAL=0, scp-hard-corridor=0
```
➡ **行本身与 P/virtual-T 雅可比都在（feedback_70 的代码声明成立），但在当前 FULL ON 配置下 SCP 硬走廊阶段整体不运行。**
生产实际生效的动态硬门是**事后采样拒绝**：
`classify_candidate → risk.hard_collision → INVALID`（`planner_manager.cpp:6155-6156`）
与 `validateExecutionTrajectory → HARD_DYNAMIC_COLLISION_FAIL`（`:2045-2046`），
用 `dynamicHardClearanceForObject = dynamic_body_radius_(0.384) + 0.5·max(scale)`（`:4330-4343`），
按 `risk_sample_dt_ = 0.10` 在 `min(duration, 2.0)` 上采样。**这是拒绝，不是优化约束。**

---

## 新增 K【源码已确认 + 生产 active】设计规则：**future prediction 不得拥有 current execution authority**

原方案：整个多项式后缀（含几秒后的预测冲突 / 预测 LOS）都能否决当前轨迹、生成当前 authority 的 SIDE、抹掉已验证覆盖。

问题：`ROUND3_VERIFICATION.md` 记录的 t+41.3~51.4 s drone0 连续 11 次 `TERMINAL_HOLD`；
链是"覆盖到期 → 每轮无 commit → TERMINAL_HOLD"，且 `NO_CERTIFIED_SUCCESSOR` 类断供。

新方案 —— 七个执行点：

```
(A) 有界 authority horizon
    planner_manager.cpp:2073-2083  executionAuthorityHorizon(duration, touch_goal)
      = touch_goal ? duration : min(duration, moving_obj_prediction_horizon_(2.0))
    validateExecutionTrajectory(traj, activation, …)  :2033-2070
      → 动态风险在 activation epoch 上评估 (:2042)，窗口受 (A) 限制
      → 团队 swarm 检查取两机 authority horizon 的 min (:2059-2061)

(B) 预测 LOS 不得产生当前 SIDE dispatch
    :6218-6230  visibility_authority_horizon = min( touch_goal ? T : 2T/3 , 2.0 )
    :6889-6903  dynamic_visibility_witness_valid = witness_time ≤ visibility_authority_horizon + 1e-6
                否则 future_los_forecast = true
    :6979-6992  [future-los-forecast] action=NO_CURRENT_SIDE_DISPATCH
                nominal_result.risk.triggered = false（LOS-only 不得伪造 risk）

(C) 后续后缀冲突不得抹掉已认证前缀
    :1962-2008  lifecycleSuccessorDue() 每 0.15 s 重验；后缀失败时用二分搜索回退到
                仍有效的前缀，execution_safe_until_ = now + horizon
                validateActivePrefixUntil(:1022-1040) 把多项式裁剪到 [now, until]
    :4445+      LOS 观测面是"事件作用域"，只在 seed 到达所选侧后激活，不是全局可见性执行门

(D) 语义随轨迹存储，而不是每次重算
    :2227 / :3948  写入 active_execution_touch_goal_
    :1974-1975     读取它决定 rolling vs mission-terminal
    → 修掉"每条 rolling 轨迹在 lifecycle 重验时被升级成 mission-terminal 校验"的错误
      (feedback_80：由此产生的后缀级联失效与 successor 断供)

(E) pending / future successor 不得阻塞下一个 planning tick
    :1216-1220 geometryRecoveryReplanDue() 不再测 localActivationPending()
    FSM:919-944  pending 只计数 + [planning-gate] action=PLAN_ANYWAY
    trajectory_lifecycle.h:47-48  localReplanAllowed(bool){return true;}

(F) Joint（future）永远是后台质量通道
    FSM:1108-1145  reserve_for_team_transaction=false
    [committed-prefix-joint] event=LOCAL_DIRECT_COMMIT … local_gate_by_joint=0
    团队提交把它送回 EXEC_TRAJ（:354-368）

(G) Joint 只动将来尾巴，起点必须精确落在已承诺前沿
    committed_prefix_joint_contract.h:30-74 discoverCommittedFutureFrontier
      frontier   = max( now + prep_budget + joint_budget + ack_budget + activation_margin ,
                        max_d committed_start_d )
      common_end = min_d validated_end_d
      valid      = frontier < common_end − activation_margin
    :78-96   committedFrontierStillCurrent()  —— CAS：owner_revision / traj_id / committed_start 必须不变
    :98-131  reheadCommittedFutureTailSeed()  —— 把几何种子按比例缩放到 horizon，
                                                并强制 head 与 committed predecessor 的 P/V/A 完全一致
    multi_uav_topology_coordinator.cpp:1284-1296  调用点
    拒绝码: TEAM_CONTEXT_REJECT reason=FUTURE_TAIL_HEAD_NOT_ON_COMMITTED_FRONTIER
```

### 公式（时间区间）
```
硬安全验证区间   [activation, activation + H_auth],  H_auth = touch_goal ? T : min(T, 2.0)
滚动执行前缀     [now, min(start+duration, max(now, execution_safe_until_))]
                 activationEarliest() = now + 0.10
                 planningDeadline()  = validated_end − 0.10
                 canStillHandoff()   = activationEarliest() < validated_end − 1e-9
预测（规划）     query_time = planning_prediction_epoch + sample_time,
                 planning_prediction_epoch = local_activation_time_ = plan_start + 0.10
团队评估视野     H_eval = min( H_configured , current_available_end − evaluation_start ), 下限 0.20 s
```
**结论：hard-validation horizon、rolling-execution prefix、forecast horizon 现在是三个显式区分的量。**
诚实例外：静态安全仍按整条多项式采样（`:4624`）；commit 时 `execution_safe_until_` 写成整条末端（`:2199`）而证书只覆盖 `H_auth`，所以在下一次 0.15 s 重验前存在"覆盖被高估"的窗口。

---

## 新增 L【源码已确认 + 生产 active】三个小的执行层规则

1. **Swarm 互不恶化（mutual non-worsening）** — `planner_manager.cpp:5040-5135`
   ```
   绝对阈值不变：椭圆度量 d² = Δx² + Δy² + 0.25·Δz² （clearance 0.5 m）
   若某采样 d²_cand < clearance²：
       取当前 active 轨迹 vs 同一 peer、同一 global_time 的 d²_current
       放行当且仅当  d²_cand ≥ d²_current − 1e-9
   ```
   对称放宽，只对已违规状态生效；不修改阈值/度量。
   问题来源（codex_round4_prompt.md）：两机已进入彼此安全泡后（实测最小间距 0.0079 m vs 阈值 0.58 m）**所有**候选被硬拒绝，形成自锁，最终 11 次 `TERMINAL_HOLD`；根因是"必须立刻回到 clearance 之外"在动力学上不可能，唯一通过的是"瞬间跳到 0.58 m 外"。
   设计层结论：**绝对阈值不能作为"已经违规之后的唯一准入判据"；违规态需要改用"不恶化"的偏序。**

2. **每个到期窗口的重规划重试上限** — `ego_replan_fsm.cpp:486-522`
   ```
   kMaxReplanAttemptsPerWindow = 2
   window identity = (traj_id, start_time + duration)
   超限 → RETURN_TO_EXECUTION（EXEC_TRAJ），不再跨 tick 反复 REPLAN_TRAJ
   ```
   直接针对"REPLAN_TRAJ→REPLAN_TRAJ 66325 次 / 244 s"的走走停停。

3. **轨迹访问器全域化** — `traj_utils/include/traj_utils/poly_traj_utils.hpp:511-565, 745-828, 956-985`
   空轨迹哨兵 `if (N<=0) return -1;`，越界 `getPos/getVel/getAcc/getJer` 返回零；`BandedSystem`/`MinJerkOpt` 补齐 rule-of-five。
   设计规则：**轨迹访问器必须是全函数；空/非法轨迹返回零状态，绝不越界读。**（证据来源：feedback_83/84 的 SIGSEGV + `pthread_mutex_lock EINVAL`；内容 CONFIRMED，归因 INFERRED）

---

# 四、只包含新增内容的设计演进时间线

> 严格晚于两轮基线。每项标注置信度与定位。

```
[09-14 17:09]  StaticLosWitness + 带 witness 的 querySegmentClearance 重载
               （plan_env/include/plan_env/static_los_geometry.h:40-52, 188-437）
               【源码已确认】
   ↓ 暴露问题：LOS 只有"挡/没挡"，没有 blocker 身份 → 无法把 LOS 变成可追踪的拓扑意图
[09-14 +]      统一 ConflictDescriptor（BODY/LOS 双通道 reason_mask）
               （plan_manage/include/plan_manage/planner_manager.h:40-91）
               【源码已确认】
   ↓ 暴露问题：BODY+LOS 同时发生时，BODY 分支会顺手删掉 LOS 观测面
[09-16 01:48]  LOS 语义从 BODY 中解耦：BODY+LOS 与 LOS-only 都能生成 LOS_OBSERVATION_SIDE 平面；
               `optimize_side()` 不再用 `!body_hard_conflict` 关闭 observation frame
               （feedback_79；planner_manager.cpp:3097, 7797-7815）
               【源码已确认】
   ↓ 暴露问题：BODY-only 冲突仍能进入可见性/拓扑选择器（越权）
[09-17 00:43]  独立 raw LOS 真值采样并入描述符；BODY-only 不得进入可见性排序
               （feedback_81；planner_manager.cpp:6448, 6915-6918, 10948-10953）
               【源码已确认】
   ↓ 暴露问题：软 J_vis 的幅值仍被当成"存在遮挡"，触发大量无收益大横移
[09-18 12:33]  软 J_vis 与离散 topology 权威解耦（NO_TOPOLOGY_AUTHORITY）
               （FIX4_STOP_AND_GO_REPORT.md；planner_manager.cpp:6538-6566）
               SIDE 求解 1499→7，SIDE_DEV_GT_1M 137→3
               【源码已确认】
   ↓ 暴露问题：rolling 时间分配被距离下限 `max(1.0, …)` 与长度解耦（0.70 m 摊到 1.37 s）
[09-18 14:04]  时间分配改为 length × 系统参考速度
               （planner_manager.cpp:761-796；v_ref = |local_target_vel|，
                total_time = max(0.10, 1.05·d/v_ref)·scale）
               【源码已确认】
   ↓ 暴露问题：真正有执行权的只有 0.29~0.31 s 的前缀，恢复力却落在执行不到的尾段
[09-18 14:04]  rolling 前缀前向进度软代价（前缀末端单点 Huber）
               —— 实现但**缺省 weight=0.0 关闭**（A/B 显示低速尾反而变差）
               （poly_traj_optimizer.cpp:5767-5843；planner_manager.cpp:370-384）
               【源码已确认，flag-disabled】
   ↓ 暴露问题：LOS 观测面沿用 BODY 的点事件窗口 ±0.45 s，覆盖不到 1.8 s 之后的遮挡区间
[09-18 15:13]  LOS 观测面 active interval 改为真实遮挡区间
               （LOS_OCCLUSION_ROOT_CAUSE.md；planner_manager.cpp:7337-7400）
               LOS_SIDE_REACH_DEADLINE = 遮挡进入时刻
               【源码已确认】
   ↓ 暴露问题（第一根因）：合围 tracking 代价只有径向带，方位误差 P50 85°~122°
[09-18 16:33]  AUDIT_BLUE_RED_YELLOW_ROOT_CAUSE.md：
               FIRST_BLUE_BOX_SEMANTIC_FAILURE = trackingGradCostP 合围分支切向梯度恒为 0
               并指出 team reference 可用率 0/1627（H_EVAL 1.5 s vs 实际覆盖 ~0.73 s）
               【文档已确认】
   ↓ 两个并行修复
[09-18 18:26]  H_eval = min(H_configured, common validated prefix)，下限 0.20 s
               （topology_coordinator_core.cpp:506-540）
               CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE 14~20 → 0
               【源码已确认】
[09-18 18:45]  软方位恢复（新增 A）+ 执行前缀 ×2 加强
               （poly_traj_optimizer.cpp:6659-6706, 7659-7675；launch:28）
               w_bearing=0.08 生产 active
               【源码已确认 present+active；09-18 定年【根据调用关系/注释/mtime 推断】】
   ↓ 仍未解决：Joint {P,τ,Ψ} / 3-ACK / 原子提交在生产 flag 下不可达
[09-18 当晚]    Joint 通道实测 0 attempt / 0 commit，每轮 JOINT_FAILURE_LOCAL_NOOP(JOINT_TIMEOUT)
               【源码已确认 + run 日志已确认】

平行线（威胁统一 / 执行权威）：
[09-15 11:21]  动态 BODY 的 SCP 硬行 + P/virtual-T 雅可比（feedback_70）
               【源码已确认，但 enable_candidate_hard_corridor_scp=false → 生产 DEAD】
[09-15 17:47]  动态硬净空与 1.1 m 偏好余量分离：
               d_hard = 0.384 + 0.5·max(scale)（feedback_72）
               【源码已确认 active（作为事后采样拒绝门）】
[09-15 18:44]  NOMINAL 语义回到 feedback61：hypothesis-0 只用 authoritative P/V/A + normal local target
               （feedback_73）
[09-15 20:26]  team viewpoint 与 normal local target 存储分离 + zero-progress guard（feedback_74）
[09-15 21:21]  NOMINAL warm start 收紧为"仅 NOMINAL 类 ABSOLUTE_SAFE 缓存"（feedback_78）
[09-16 18:44]  active_execution_touch_goal_：rolling 轨迹不再被升级成 mission-terminal 校验（feedback_80）
[09-17 23:06]  traj_server：已排程前驱 + 有序 future queue + 同 activation 跨代替换（feedback_77）
               【源码已确认，但生产 run 中从未触发】
[09-18 00:00+] Swarm 互不恶化解锁（codex_round4_prompt.md）
               【源码已确认 active】
[09-18 00:42]  poly_traj_utils.hpp 空轨迹/UB 加固
               【内容已确认；归因【根据调用关系推断】】
[09-18 12:33]  每到期窗口重规划重试上限 kMaxReplanAttemptsPerWindow = 2
               【源码已确认 active】
```

---

# 五、给第三份报告的"该写 / 不该写"清单

## 应该写（第三份报告真正新增）
1. **合围代价补回切向权威**：软方位恢复 + 风险自适应角隙 + 执行前缀 ×2（新增 A）——公式、P/T 梯度、0.08 权重、0.25 s 前缀。
2. **合围缺口的 Local 版本**：Local 用冻结邻机算同一 `encirclementGeometryCost`（新增 B）。
3. **H_eval 从固定 1.5 s 契约改为公共已验证前缀导出，下限 0.20 s**（新增 D）——以及"团队角度修正通道长期不可用（0/1627）"这一实测根因。
4. **软可见性代价与离散 topology 权威解耦**（新增 F）——这是一条设计规则，且带来了 SIDE 1499→7 的量化效果。
5. **LOS 观测侧拓扑**：遮挡锥切线半空间 + 真实遮挡区间窗口 + 侧到达期限 + 重定时按比例保持（新增 I）。
6. **三威胁统一的实际边界**：统一了什么（描述符/循环/平面类型/词汇/时基）、以及 BODY 点事件 vs LOS 区间事件等必须分开的物理语义（新增 H）。
7. **动态 BODY 物理净空与偏好余量分离 + SCP 硬行与其 P/virtual-T 雅可比**（新增 J）——但必须注明生产 flag 关闭。
8. **执行权威设计规则**："future prediction 不得拥有 current execution authority"，以及 hard-validation / rolling-prefix / forecast 三个 horizon 的显式分离（新增 K）。
9. **Joint 只动将来尾巴、起点钉在 committed frontier**（新增 K-(G)）。
10. **Swarm 互不恶化解锁**（新增 L-1）——"绝对阈值不能作为已违规态的唯一准入判据"。

## 不应该写（属于前两轮基线，或属于第一轮）
- 动态风险触发左右绕行、LEFT/RIGHT SIDE、0.4/0.5/0.6/0.7 侧移量、A* repair、A* 后 topology 复查、A* 简化、Local SFC、repair path 时间初始化、MINCO+SCP+OSQP、SFC/v/a/j 硬约束、SAFE/IMPROVED/INVALID 分类。
- 三机 120° 参考、Elastic Tracking、静态/动态/邻机风险驱动的径向与角向弹性、三机 N/L/R 枚举（≤27）、团队评分、time-only 机间冲突修复、固定 P 只调时间、机间距离对时间的 Jacobian、用既有 SCP/OSQP 做时间错峰。
- **方向性 J_vis 进入连续优化（P/T）本身**——第一轮基线就已如此（09-10 结构审计已给公式）。
- `Q2/K2/J_acc/J_K2-cont/J_blackout/J_div/J_multiview`、3-ACK/common activation/原子提交、K-of-N 团队排序、adaptive viewpoint generator、activation scheduling、multiviewQuality/angularDiversityCost（均 ≤09-13）。

## 不要当主要成果（工程 bug，只作证据）
- 方位项代价/梯度差一个 `wei_tracking_`（×100）不一致；
- `NOMINAL_WARM_START_OVERFIX` / `NO_CERTIFIED_SUCCESSOR` / `EXECUTION_RESERVE` / `activation_earliest` 这些标识符在源码中并不存在；
- `ros_now_inside_dynamic_risk_count_` 从不自增；`[timebase-consistency]` 把同一个变量打印四次（同义反复）；
- 遮挡区间对所有 blocker 取并集；动态 BODY-only 的 primary witness 不可达分支；`geometry_revision` 永不赋值；
- `camera_time_saturation_fraction` / `enable_acceptance_guard` 为 dead；
- `TRAJECTORY_ACTIVATION_GAP_P50/P95` 采样方式使其恒为 0；
- traj_server ACTIVATION 阶段 handoff 结果被丢弃（`:538`）。

（以上若暴露了设计规则，应按"设计规则"写，而不是按"哪一行写错"写。）

---

# 六、需要你确认的一件事

`/home/bob/ALP/egov2_fc65423_` 这个路径不完整，我按 `/home/bob/ALP/egov2_fc65423_constvel/` 审计。
如果你的"前两轮"其实是指本目录 `feedback/` 编号体系里的某两轮（而不是 09-02 / 09-06~09-09 那两轮合围+可见性工作），
那么第 0 节的基线边界需要相应平移——但第一节到第三节的"新增机制清单"本身不受影响，因为它们全部带文件+行号+公式，可独立核对。
