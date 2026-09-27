# 9/8 基线之后的全部改动 + 当前版本重要公式

只读源码审计结论。基线＝**2026-09-08 及以前**（前两轮报告覆盖范围）。
代码区：`/home/bob/ALP/egov2_fc65423_constvel/`（`egov2_fc65423_` 路径不完整）。
未修改源码、未改参数、未跑仿真。

---

## 0. 9/8 基线的最硬证据

**证据 A：9/8 六个实验目录的 `run_meta.json` 记录的 31 个 launch 参数键；9/9 的并集是 32 个，唯一新增键是 `weight_visibility`。**

```
cd /home/bob/ALP
find encirclement_*_20260908 -name run_meta.json   # 9/8 的实跑参数
find encirclement_*_20260909 -name run_meta.json   # 9/9 的实跑参数
# 键并集 diff: 9/9 新增 weight_visibility (=20.0)；9/8 有而 9/9 没有的键: 无
```

因此 **9/8 已经真实打开**（全是 `true`，非默认）：
`enable_elastic_tracking_region`、`enable_relative_safety_temporal_elasticity`（time-only 机间冲突修复）、
`enable_k_of_n_team_visibility`(k=2)、`enable_visibility_candidate_ranking`、`enable_joint_topology_coordination`、
`enable_cooperative_viewpoint_reference`、`viewpoint_independent_mode`、**`enable_candidate_hard_corridor_scp`**。
`enable_encirclement_tracking` 在 9/8 是 A/B 变量（off/on 都有跑）。

**证据 B：devel 里生成的消息头时间戳（创建时点）**
```
2026-09-05 17:42  devel/include/traj_utils/TopologyCandidateBundle.h        → 9/8 前
2026-09-08 19:55  devel/include/traj_utils/EncirclementReferenceBundle.h    → 9/8 前（当天）
2026-09-09 19:04  devel/include/traj_utils/TeamTrajectoryAck.h              → 9/9 之后
2026-09-09 19:04  devel/include/traj_utils/TeamTrajectorySolution.h         → 9/9 之后
2026-09-11 07:57  devel/include/traj_utils/TeamRecoveryTarget.h             → 9/11
2026-09-11 10:06  devel/include/traj_utils/RecoveryGuide.h                  → 9/11
```

**证据 C：9/9 实验目录名本身就是时间戳**
`encirclement_visibility_elastic_20260909`(01:01) → `encirclement_directional_visibility_20260909`(02:36)
→ `encirclement_directional_visibility_final_ab_20260909`(13:24) → `encirclement_team_visibility_alignment_20260909`(14:33)
→ `encirclement_team_visibility_final_5runs_20260909`(15:11)。

**证据 D：git 完全不能用作切分依据** —— `git ls-tree -r 8cef241 -- .../planner/` 返回 **0 个文件**，整个 planner + multi_uav_formation 树是 untracked。

> 注意两个陷阱：
> 1. 根目录 `feedback_17.md` 是 `feedback/feedback_23.md` 的逐字节副本，正文日期写的是 **2026-09-09**，不是 9/4。
> 2. `team_visibility_optimizer.{h,cpp}` 的 mtime（09-16/09-17）**不是创建时间**——`feedback/ACTIVATION_LIFECYCLE_REPORT_20260910.md`（9/10）已经打印出完整目标函数。
> 3. **2026-09-10 及以后的实验目录已全部删除**，所以 9/10–9/18 的定年只能靠报告 + mtime + 幸存日志。

---

## 1. 9/8 之后做了什么（按因果顺序）

### 第 3 轮 9/9–9/10：可见性从"排序"升级为"目标 + 团队指标"

| 时间 | 改动 | 暴露的问题 | 证据 |
|---|---|---|---|
| 9/9 | **方向性 `J_vis` 进入 MINCO 连续优化**：静态 LOS、最危险动态 LOS、FOV 三个风险平方加权和，带解析梯度，`weight_visibility=20.0`（这是 9/9 **唯一新增的 launch 参数**） | 排序式 visibility 只能"选一条已存在的轨迹"，不能改变轨迹形状 | `poly_traj_optimizer.cpp:4991-5420`；键 diff |
| 9/9–9/10 | **deep-risk continuation** `κ=1.0, β=2.0`：`c≤c_in` 时不再饱和到 1，改为线性+指数逃离梯度 | smoothstep 在深度遮挡区梯度趋 0，出不来 | `tracking_visibility_geometry.h:75-105` |
| 9/9–9/10 | **`TrackingCameraContract` / `trackingCameraFovDirectionalRisk`**：FOV 也进连续代价并给位置与 yaw 梯度 | 之前 FOV 只在执行侧判 | 同上 |
| 9/9–9/10 | **Stage 2：K2 continuity + blackout**：`Q2`、`b_K2=1−Q2`、`b_0=Π(1−v_i)`、滑窗 `round(window/dt)+1` 的 `slidingWindowSquaredCost` | K2 瞬时高但会断续；黑障无惩罚 | `feedback_23`；`team_visibility_optimizer.cpp:484-545` |
| 9/9–9/10 | **Stage 3A/3B：Joint `{P,τ,Ψ}` + SCP/OSQP + 非线性 acceptance + 3-ACK 合同** | 单机各自最优 ≠ 团队最优；三机不同时切换 | `TeamTrajectoryAck/Solution.msg` 生成头 09-09 19:04；`scp_optimizer.*` |
| 9/10 | **实测 OFF/ON 3+3**：ON 的 `all3` 0.923→0.688、camera-time −18 camera·s | 可见性上升但"第三路丢失" | `feedback_24` 表 |
| 9/10 | 根因：第三路损失由 **水平 FOV + 静态 LOS** 主导，不是 blackout | — | `feedback_25` |

### 第 4 轮 9/10–9/11：把 120° 从"硬参考"降级为 seed

| 时间 | 改动 | 暴露的问题 | 证据 |
|---|---|---|---|
| 9/10 | **AdaptiveViewpointGenerator**：fixed / uniform-120° / previous-accepted 三 seed，逐机独立 bearing 搜索（24 角度 × 半径±0.35 × 高度±0.20），near 1.5 s / far 3.5 s 双尺度 | 120° 等角在遮挡场景不成立 | `adaptive_viewpoint_generator.{h,cpp}` |
| 9/10 | **`activation_schedule.h`**：`B_stage=max(20 ms, 1.25·max_recent(stage latency))`，`lead=max(min_lead, B_prep+B_joint+margin+25 ms)` | 固定 240 ms lead 遇到 310 ms 求解必然失败 | `activation_schedule.h` |
| 9/11 | **Persistent Team Recovery Target + 独立 Target/Transition Pool**：目标必须满足真实 25°/170°；过渡轨迹只受原有安全约束 | 恢复目标被 rolling 等待条件绑架 | `TeamRecoveryTarget.msg` 09-11 |
| 9/11 | **planner-backed target reachability → validated recovery guide → local 滚动 / joint 异步** | "算得出"≠"三机生产规划器可达" | `RecoveryGuide.msg` 09-11；`team_target_reachability.cpp` |
| 9/11 | **持续共同时间 TEAM_REFERENCE** | 一次性参考在 rolling 中失效 | `team_trajectory_reference.h` |

### 第 5 轮 9/12：合围判据从"等角"改成"空扇区"（**这一轮彻底删掉 120° 先验**）

| 时间 | 改动 | 暴露的问题 | 证据 |
|---|---|---|---|
| 9/12 | **`multiviewQuality`**：三个 minor 夹角，`bad=25° good=60°` smoothstep，`Q_dir=mean(q)`，`J_dir=mean((1−q)²)`；**删除 planner-level brake** | 120° 等角既非必要也非充分 | `feedback_41`；`encirclement_geometry.h:35-59` |
| 9/10–9/13 | **25°/170° 圆周空扇区判据**：`gaps` 排序后循环间隔，`min≥25°` 且 `max≤170°` | 用可执行几何定义替代等角假设 | `encirclement_geometry.h:61-145` |
| 9/12 | brake 删除 + 连续运动 | 走走停停 | `feedback_41` |

### 第 6 轮 9/12–9/18：把"执行权"从"未来预测"里剥离（最长的一条线）

| 时间 | 改动 | 暴露的问题 |
|---|---|---|
| 9/12 | **validated moving coverage 预算**：`validated_end=min(start+duration, max(now, execution_safe_until_))`、`activationEarliest=now+0.10`、`planningDeadline=validated_end−0.10`、`ExecutionPlanningBudget(1.10/0.20/2.50, ×1.35, τ½=60 s)` | 规划没有"还剩多少有效覆盖"的概念 |
| 9/13 | **两阶段供给 + `FIRST_SAFE_RESERVED`**：先占一个 ABSOLUTE_SAFE，再找更优 | 质量搜索把 successor 供给拖死 |
| 9/13 | **Early Joint**：本地候选自带的 `joint_seed` 直接喂团队优化，不再等三方握手 | Joint 总慢一拍 |
| 9/13–9/14 | **Committed Prefix + Background Joint Future Tail + 原子 adoption**：`discoverCommittedFutureFrontier` / `committedFrontierStillCurrent` / `reheadCommittedFutureTailSeed` / 3-ACK + CAS + `lead≥25 ms` | Joint 不能回头改已执行的前缀；三机必须同时切 |
| 9/14 | **`StaticLosWitness` + 带 witness 的 `querySegmentClearance` 重载** | LOS 只有"挡/没挡"，没有 blocker 身份 |
| 9/14–9/17 | **统一 `ConflictDescriptor`（BODY/LOS 双通道 reason_mask）+ `LOS_OBSERVATION_SIDE` 平面** | BODY+LOS 时 LOS 平面被 BODY 分支删掉 |
| 9/15 | **动态物理净空与偏好余量分离**：`d_hard = 0.384 + 0.5·max(scale)`，1.1 m 只作软目标 | 把偏好余量当物理门 → 过度保守 |
| 9/15 | **动态 BODY 的 SCP 硬行 + P/virtual-T 雅可比** | 优化器里动态项没有硬约束 |
| 9/15 | NOMINAL 语义回到 authoritative P/V/A + normal local target；team viewpoint 独立存储；zero-progress guard；NOMINAL warm start 收紧 | SIDE/guide/team 几何污染 NOMINAL |
| 9/15 | 轨迹发布去重 + traj_server 已排程前驱 / 有序 future queue | 乱序与重复激活 |
| 9/16 | **`active_execution_touch_goal_`**：rolling 轨迹不得被升级成 mission-terminal 校验 | 后缀冲突抹掉已认证前缀 |
| 9/17 | **Swarm 互不恶化**：已违规时改用"不恶化"偏序 | 绝对阈值在已违规态形成自锁（实测两机间距 0.0079 m vs 阈值 0.58 m 时全部候选被拒） |
| 9/18 | **软 `J_vis` 剥夺离散 topology 权威**；rolling 时间分配改为 length×参考速度；前缀进度软代价（默认关） | 软代价冒充几何真值（SIDE 求解 1499→7）；`max(1.0,…)` 把 0.3 m 摊到 1.4 s |
| 9/18 | **LOS 观测面窗口改用真实遮挡区间**（不再用 BODY 的 `±0.45 s`） | 区间事件用了点事件窗口 → 1.8 s 后的遮挡覆盖不到 |
| 9/18 | **软方位恢复 + 执行前缀 ×2**；**`H_eval=min(H_configured,H_common)`，下限 0.20 s** | 合围代价无方位项（实测方位误差 P50 85°~122°）；固定 1.5 s 与实测 0.73 s 覆盖结构性矛盾（team reference 0/1627） |

---

## 2. 当前版本的重要公式

> 🟢 = 生产 active ｜ 🔴 = flag 关闭（当前不生效）｜ ⚪ = 只排序/触发，无梯度

### 2.1 合围 tracking 代价 🟢 — `poly_traj_optimizer.cpp:6636-6706`

```
q   = p − o(t) − v_o·t                      // o(t)=目标中心
seed= R_target · q*_i                        // 每机固定编队种子
r=|q_xy|, r*=|seed_xy|, band(e,w)=copysign(max(0,|e|−w), e)

er = band(r − r*, 0.35)          // observation_radius_band_
ez = band(q_z − seed_z, 0.20)    // observation_height_band_

φ       = atan2(q.y,q.x) ;  φ* = atan2(seed.y,seed.x) ;  e_φ = wrap(φ−φ*)
ρ       = clamp(risk_state.risk, 0, 1)          // LBFGS 前冻结的风险剖面
band_φ  = elasticAngularSlack(ρ) = 20° + 25°·ρ   // 0.349 + ρ·(0.785−0.349)
e_φ,eff = max(0, |e_φ| − band_φ)

in_prefix = (i_dp < prefix_execution_samples_)   // 0.25 s 执行前缀
w_φ = w_track · w_bearing · (in_prefix ? boost : 1) = 100 · 0.08 · (2.0 | 1.0)

costp        += w_track·(er² + ez²) + w_φ·e_φ,eff²
∂φ/∂q         = ( −q_y/r² , q_x/r² , 0 )                       // 纯切向
∂J_φ/∂p       = w_φ · 2·sign(e_φ)·e_φ,eff · ∂φ/∂q
gradp        += w_track·[ (2·er·q_xy/r, 0, 2·ez) + ∂J_φ/∂p ]
gradt        += w_track·dJ_dp·(v − v_o)                        // → τ_i
grad_prev_t  += w_track·dJ_dp·(−v_o)                           // → τ_{i−1}
```
变量：**P + T**，不含 yaw。
参数：`encirclement_bearing_recovery_weight=0.08`、`encirclement_bearing_prefix_boost=2.0`、
`prefix_execution_span_ = local_activation_margin + execution_margin = 0.10+0.15 = 0.25 s`、
`sample_dt = piece_length/(cps_num_prePiece·max_vel) = 1.5/(5·3) = 0.1 s`。

### 2.2 合围缺口的 Local 版本 🟢 — `poly_traj_optimizer.cpp:6729-6764`

```
邻机用冻结 peer 多项式在同一世界时刻采样：
rel_self = q ; rel_peer = p_peer(t) − o(t) − v_o·t

bearings[i] = atan2(rel_i.y, rel_i.x)
按 bearing 排序得 order[] ；gaps[j] = bearings[order[(j+1)%3]] − bearings[order[j]]  (+2π, j=2)

J_gap = Σ_{j=0..2} [ max(0, gaps[j] − 170°)² + max(0, 25° − gaps[j])² ]
∂J_gap/∂rel_i = (i==a ? −2 : +2)·(hi − lo)·(−rel_i.y, rel_i.x, 0)/|rel_i,xy|²

costp       += w_gap · J_gap                      // w_gap = local_gap_weight = 100
explicit_time = Σ_id ∂J_gap/∂rel_id·(−v_o) + Σ_{id≠self} ∂J_gap/∂rel_id·v_id
gradt       += w_gap·( explicit_time + ∂J_gap/∂rel_self·v )
grad_prev_t += w_gap·explicit_time
```
**无 120° 先验、无等角奖励**，只有 `[25°,170°]` 区间约束。

### 2.3 局部方向性可见性代价 🟢（9/9 新增）— `poly_traj_optimizer.cpp:4991-5420`

```
J_vis(sample) = w_vis · ( R_static² + R_dynamic² + R_fov² ) ,  w_vis = 20.0

R(c) = directionalClearanceRisk(c, c_in = 0.08, c_out = 0.08+0.80, κ=1.0, β=2.0)
  c ≥ c_out        : R = 0
  c_in < c < c_out : u = (c_out−c)/w ,  R = u²(3−2u) ,  dR/dc = −6u(1−u)/w
  c ≤ c_in         : d = (c_in−c)/w  ,  R = 1 + κ[d − (1−e^{−βd})/β] ,  dR/dc = −κ(1−e^{−βd})/w

静态 LOS : c = 观测者→目标线段到静态圆柱/墙的最小 clearance
动态 LOS : 每个 moving obstacle 各算一个 R ，取 R_dynamic = max_id R_id
FOV      : R_fov = trackingCameraFovDirectionalRisk(p, predicted_yaw, target, camera).risk

写回变量（梯形积分，端点权重 0.5，intervals = ceil(duration/dt)，dt = 0.10 s）：
  jerkOpt_.get_gdC().block<6,3>(piece*6,0) += trap_w·step·(β0 · ∂J/∂p)     // → P
  gdT(piece)        += trap_w·( J/intervals + step·α·∂J/∂t )               // → T
  gdT.head(piece)   += trap_w·step·∂J/∂t_prev
  ∂J/∂t = ∂J/∂p·v + ∂J/∂target·v_o (+ ∂J/∂center·v_center) (+ ∂J/∂yaw·yaw_rate)
```
变量：**P + T**，**不含 yaw**（yaw 是输入，由 `advanceTargetFacingYaw` 递推）。
实测该 run 中 14711 次调用、`J_VIS_TOTAL_MEAN=0`（该场景该时刻未遮挡）。

### 2.4 Elastic tracking 🔴（`enable_elastic_tracking_region=false`）

```
ρ     = clamp(max(static_risk, moving_risk, swarm_risk [, visibility_risk]), 0, 1)
α(ρ)  = elasticAngularSlack(ρ, true, 0°, 45°, 20°) = 20° + 25°·ρ
s(ρ)  = ρ · elastic_radial_slack_max (0.80)
e_r   = signed distance of r outside [r*−s, r*+s]
J_region  = e_r² + r*²·[|a| − α(ρ)]₊² + h²
J_nominal = (r−r*)² + r*²·[|a| − α_base]₊² + h²
J_track   = w_track·[ (1−ρ)·J_nominal + ρ·m·J_region ] ,  m = 0.20
```

### 2.5 团队优化器目标 🔴（`enable_team_visibility_optimizer=false` 且 `enable_joint_pt_optimization=false`）

```
变量：每机 [ P 内点 | 每段 virtual-T | yaw knots ]

Q2        = v1v2 + v1v3 + v2v3 − 2·v1v2v3
b_K2      = 1 − Q2
b_0       = (1−v1)(1−v2)(1−v3)
J_K2      = mean(b_K2)
J_acc     = mean[ 1 − (v1+v2+v3)/3 ]
width     = max(1, round(window/dt)+1)
swc(s,w)  = mean_over_windows[ (mean_window s)² ]
J_K2-cont = swc(b_K2, width_K2=0.5 s)
J_blackout= swc(b_0 , width_blackout=0.5 s)

J = w_mv·J_multiview + w_K2·J_K2 + w_acc·J_acc + w_K2c·J_K2-cont + w_bo·J_blackout
  + w_dev·J_deviation + w_td·J_target-dist + w_jerk·J_jerk + w_yaw·J_yaw-prior
  + w_div·J_div + w_gap·J_encirclement-gap + w_target·J_recovery-target

权重：k2=0.20, accumulated=4.0, k2_continuity=0.0, blackout=0.0, deviation=1.0,
      target_distance=2.0(slack 0.35, near×4/far×1), jerk=1e-5,
      multiview=1.0, diversity=0.15, encirclement_gap=4.0, recovery_target=1.0, yaw_prior=0.02
虚时间：realToVirtualTime(d) = d>1 ? √(2d−1) − 1 : 1 − √(2/d − 1)
采样梯度：fixedTimeSampleGradient(optimizer, local_time, ∇_sample, order) → P 与 T
连续可见性：v = smooth(range−min) · smooth(max−range) · Π smooth(clearance_k, 0.08, 0.23)
              · exp(−max(0, FOV_risk))
约束：P trust 0.12、T trust 0.08、yaw trust 0.0（把 yaw knot 全钉死）、yaw rate 2π、
      yaw acc 5π、v/a/j = 3.0/4.0/8.0、机间 0.50、静态 0.0、动态 1.10
Solver：SCPOptimizer::solve(...)，max_iterations = 1，回溯 {1.0, 0.5, 0.25, 0.10}
无硬 K≥2 行。
```

### 2.6 合围几何（25°/170°）🔴 团队目标用；⚪ ranking 侧对应量

```
theta_i = atan2(q_i.y, q_i.x)
minor   = |wrap(theta_i − theta_j)| ,  u = clamp((minor−25°)/(60°−25°), 0, 1)
q       = u²(3−2u)
Q_dir   = Σ q/3 ,  J_dir = Σ (1−q)²/3     // 饱和区梯度为 0
J_div   = Σ_{i<j} max(0, 25° − |wrap(θ_i−θ_j)|)²
```

### 2.7 LOS 观测侧拓扑 🟢 — `planner_manager.cpp:7337-7815`；SCP 行 🔴

```
blocker-target frame
  e        = (blocker_xy − target_xy)/|…|
  n_right  = ( e.y , −e.x )
  α        = asin( min(1, radius/distance) )
  m2       = −sin(α)·e + side·cos(α)·n_right ,  side ∈ {+1,−1}
  observation_normal = ( m2.x , m2.y , 0 )
  observation_point  = ( target.x , target.y , 0 )

observation-side inequality（clearance = 0）
  nᵀ(p − p0) ≥ 0    ⟺    side·θ ≥ α        // 遮挡锥在目标处的切线半空间

active interval（真实遮挡区间 ∩ 到达观察侧区间）
  occlusion_start = raw_los_occlusion_enter_rel_        // 不再用 ±0.45 s
  occlusion_end   = raw_los_occlusion_exit_rel_
  los_side_reach_deadline = occlusion_start             // 必须在遮挡开始前到达所选侧
  enter_time = max(occlusion_start, side_reached_time)
  exit_time  = side_left_time ≥ 0 ? min(occlusion_end, side_left_time) : occlusion_end
  无重叠 → WINDOW_LIMITED / RECOVERY_NOT_REACHED，放弃该平面

SCP 线性化行（poly_traj_optimizer.cpp:2327-2372）
  h      = clearance − n·(p − p0)                       // clearance = 0
  grad_h = − mincoSampleGradientWrtX(piece, τ, n, SAMPLE_POSITION, ds_dT)
  row:     grad_h·Δx ≤ −h   ⟺   n·(p−p0) + (−n)·ΔP ≥ clearance
  ds_dT(d) = global_α − [d < piece] ,  global_α = t/total_duration    // virtual-T 雅可比

重定时保持语义
  ratio_k  = clamp01( active_k / initial_total_duration )   // 优化前归一化
  active_k = ratio_k · total_duration                       // SCP 每轮按比例还原
```

### 2.8 动态 BODY 硬约束 🔴 SCP 行；🟢 作为事后采样拒绝门

```
物理硬净空 d_hard = moving_obj_hard_body_radius_(0.384) + max_id( 0.5·max(scale_id.x, scale_id.y) )
偏好余量           = moving_obj_clearance_ = 1.1 m （只作软目标/触发）

SCP 行（runCandidateHardCorridorSCP 内，:2424-2499）
  horizon = min(total_duration, moving_obj_prediction_horizon_ = 2.0)
  activation_margin = elastic_moving_risk_margin_ = 0.80
  对采样 t = α·horizon:
    obstacle(t) = evaluateConstVel(id, t_now_ + t)
    distance = |p − obstacle|
    if distance > 1.1 + 0.80 → skip
    normal = (p − obstacle)/distance
    grad_d  = mincoSampleGradientWrtX(piece, τ, normal, SAMPLE_POSITION, ds_dT)              // P
    grad_d[position_dim+d] += (−normal·v_obstacle)·ds_dT(d)·realTimeJacobianAt(virtual_t_d)  // T
    row:  −grad_d·Δx ≤ distance − d_hard    ⟺   d + ∇d·Δx ≥ d_hard
  终检 maxDynamicBodyViolation > 2e-3 → SCP_FINAL_DYNAMIC_VIOLATION

生产实际生效（事后拒绝）
  classify_candidate → risk.hard_collision → INVALID
  validateExecutionTrajectory → HARD_DYNAMIC_COLLISION_FAIL
  采样 dt = risk_sample_dt_ = 0.10 ，窗口 = min(duration, 2.0)
```

### 2.9 执行权 / 时间区间 🟢

```
H_auth(duration, touch_goal)  = touch_goal ? duration : min(duration, 2.0)
visibility_authority_horizon  = clamp(0, min( touch_goal ? T : 2T/3 , 2.0 ))
raw_los_interval_scan_horizon = max(visibility_authority_horizon, 2.0)

planning_prediction_epoch = local_activation_time_ = plan_start + 0.10
query_time                = planning_prediction_epoch + sample_time

validated_coverage_end_ = min( active.start + duration, max(now, execution_safe_until_) )
activationEarliest()    = now + 0.10                    // local_activation_margin_
planningDeadline()      = validated_coverage_end_ − 0.10
canStillHandoff()       = activationEarliest() < validated_coverage_end_ − 1e-9

committed frontier（Joint 只能动它之后的尾巴）
  frontier   = max( now + prep_budget + joint_budget + ack_budget + activation_margin ,
                    max_d committed_start_d )
  common_end = min_d validated_end_d
  valid      = frontier < common_end − activation_margin
  且 owner_revision / traj_id / committed_start 必须不变（CAS）

团队评估视野
  H_common = current_available_end − evaluation_start
  H_eval   = min(H_configured, H_common) ,  下限 kMinimumJointEvaluationHorizon = 0.20 s

未来 LOS 不得拥有当前执行权
  dynamic_visibility_witness_valid = witness_time ≤ visibility_authority_horizon + 1e-6
  否则 future_los_forecast = true → [future-los-forecast] action=NO_CURRENT_SIDE_DISPATCH
```

### 2.10 Swarm 安全：互不恶化 🟢 — `planner_manager.cpp:5040-5135`

```
椭圆度量 d² = Δx² + Δy² + 0.25·Δz² ,  clearance = 0.5 m
若 d²_cand < clearance² :
    取当前 active 轨迹 vs 同一 peer、同一 global_time 的 d²_current
    放行 ⟺ d²_cand ≥ d²_current − 1e-9
否则维持原绝对阈值拒绝
```

### 2.11 Rolling 时间分配 🟢 — `buildFreshMovingInitializer`

```
v_ref = |local_target_vel| ，退化时 0.55·v_limit
v_ref = clamp(v_ref, 0.25·v_limit, max_vel)
total_time = max(0.10, 1.05·distance/v_ref) · requested_scale
若 max_acc > 0 : total_time = max(total_time, 2·√(distance/a_limit))   // 仅动力学下界
```

### 2.12 前缀进度软代价 🔴（`prefix_progress_weight=0.0`）

```
t_p     = min(prefix_span, Σ T_i) ,  prefix_span = 0.25 s
deficit = max(0, v_ref − v(t_p)·t̂) ,  normalized = deficit/v_ref ,  δ = 0.50
normalized ≤ δ : J = w_eff·normalized²              , ∇_v = −2·w_eff·normalized·t̂/v_ref
normalized >  δ : J = w_eff·δ·(2·normalized − δ)    , ∇_v = −2·w_eff·δ·t̂/v_ref
w_eff = w·attenuation , attenuation = 0.15 当 hard_collision 或 triggered
写回 gdC[piece] += b1·∇_vᵀ ; gdT[piece] += ∇_v·a_p （时间梯度开关 false）
```

### 2.13b 动态安全性终检的窗口与 executionAuthorityHorizon **不同**（易错点）

```
checkMovingObjSafety(traj, prediction_start_time, reason)   // poly_traj_optimizer.cpp:8240-8272
  guard: use_time_aware_moving_obj_cost_ && moving_objs_ && getObjNums()>0 && pieceNum>0
  check_end = traj.getTotalDuration()
  if (moving_obj_prediction_horizon_ > 1e-3) check_end = min(check_end, moving_obj_prediction_horizon_)
  sample_dt = 0.03
  for (t = 0; t < check_end+1e-6; t += 0.03):
      obj_p = evaluateConstVel(id, prediction_start_time + min(t, check_end))
      if (||p(t) − obj_p|| < getMovingObjHardClearance()) → false
```

**没有 `touch_goal` 旁路**（与 `executionAuthorityHorizon = touch_goal ? T : min(T,2.0)` 不同）：
即使任务终点轨迹，动态硬检查也只覆盖 `min(T, 2.0)`。`maxDynamicBodyViolation` 用同一 0.03 s 网格。
→ 报告里不要把这两个 horizon 写成同一个量。

### 2.13 Activation scheduling 🟢

```
B_stage = max(20 ms, 1.25 · max_recent(stage_wall_latency))   // 最近 32 条
required_margin = ACK_timeout + 25 ms
lead = max(configured_minimum_lead, B_prep + B_joint + required_margin + 25 ms)
activation = now_at_joint_preparation + lead
lead > 950 ms → 提前拒绝，不压缩到 planner 的 1 s 上限
```

---

## 3. 当前生产配置的真实开关状态（来自 `sim_run_round2.log` 实测打印，不是默认值推断）

```
[team-vis-opt-config]  enabled=0 joint_pt=0 runtime_ready=0 horizon=1.500 dt=0.100
                       weights_k2_acc_k2cont_blackout_dev_jerk = 0.2, 4.0, 0.0, 0.0, 1.0, 1e-5
                       trust_P=0.12 trust_T_ratio=0.08 max_iter=1
                       stage2_k2_continuity=0  stage2_blackout=0
                       stage3a_yaw_reprediction=0  stage3b_joint_yaw=0  yaw_trust=0.000
                       fallback=legacy_selector
[candidate-optimizer-config]   hard_corridor_scp=0
[elastic-tracking-config]      enabled=0
[team-visibility-config]       enabled=1  N=3  K=2  preferred_separation_deg=25
                               diversity_weight=0.150  mode=PER_UAV_CANDIDATE_SOFT_UTILITY
[topology-coordination-planner-config] enabled=1  timeout=0.200
[bearing-recovery-setup]       w_bearing=0.080000  prefix_boost=2.000  prefix_span=0.250
[prefix-progress-setup]        w_base=0.000000
[raw-los-truth] / [los-plane-audit]  在跑
```

| 机制 | 状态 | 说明 |
|---|---|---|
| 方向性 `J_vis`（P+T 梯度） | **ON** | 9/9 新增，`weight_visibility=20` |
| 软方位恢复（P+T 切向） | **ON** | 9/18 新增，`w_bearing=0.08` |
| 合围缺口 Local 项 | **ON** | `local_gap_weight=100` |
| 可见性候选排序 + K-of-N(k=2) | **ON** | 只排序/触发 |
| 团队 topology coordination | **ON** | 只做 tuple 枚举/筛选，`blocking=0` |
| LOS 观测面生成 + 事后校验 | **ON** | 生成 phase active |
| 动态硬净空事后拒绝 | **ON** | 采样拒绝，非优化约束 |
| Swarm 互不恶化 | **ON** | 9/17 新增 |
| **Joint `{P,τ,Ψ}` + K2/blackout + 3ACK + 原子提交** | **OFF** | `enabled=0 joint_pt=0`；run 里 `team-solution-commit=0` |
| **SCP 硬走廊（LOS 半空间 + 动态 BODY 行）** | **OFF** | `hard_corridor_scp=0` |
| Elastic tracking region | **OFF** | `enabled=0` |
| 前缀进度软代价 | **OFF** | `w_base=0.000000` |

**含义**：当前 FULL ON 下，可见性/合围对连续优化的作用只有 **方向性 `J_vis` + 软方位恢复** 两项；
团队连续优化（9/9–9/10 做的 Stage 2/3A/3B）与 SCP 硬走廊**都不在运行**。
报告里如果声称"团队协同优化在跑"或"LOS/动态是硬约束"，必须先打开 `enable_team_visibility_optimizer`
与 `enable_joint_pt_optimization` / `enable_candidate_hard_corridor_scp` 并重新测量。

---

## 4. 一句话总结

9/8 之后这条线的主轴：
**① 可见性从排序指标变成连续代价（9/9 方向性 `J_vis`，唯一新增 launch 参数）→ ② 变成团队指标（9/9–9/10 Stage 2 的 K2-continuity/blackout + Stage 3 joint `{P,τ,Ψ}` + 3ACK）→ ③ 合围判据从 120° 等角换成可执行的 25°/170° 空扇区（9/12）→ ④ 参考从"几何可达"升级为"planner 可达 + 持续团队参考"（9/11）→ ⑤ 执行权与未来预测分离（9/12–9/18：validated coverage / committed frontier / 原子 adoption / future-LOS 不得 dispatch）→ ⑥ 三威胁统一到一个 `ConflictDescriptor` 并把 LOS 变成观测侧半空间（9/14–9/18）→ ⑦ 补回合围的切向权威（9/18 方位恢复 + Local 缺口项）并放开被固定 1.5 s 卡死的团队评估视野。**
