# ALP Visibility-First Adaptive Encirclement 改造前结构报告

日期：2026-09-10。项目：`/home/bob/ALP/egov2_fc65423_constvel`。

本报告是 PHASE 0/1 的交付，不是 PHASE 2 完成报告。报告生成前未修改生产源码、未构建、未启动仿真。未访问或修改 RRCT。工作区原有大量已修改和未跟踪文件；不得把它们当作本轮变更覆盖或回退。

审计方法：只读检查当前源码的关键函数、消息、配置、已有 contract tests、feedback_23/24/25，以及 runs_final2 六次 visibility CSV 和 launcher 日志。下面区分源码确认、CSV 复算、既有报告结果与尚待运行验证的推断。不能据此宣称逐行阅读了仓库所有文件，或唯一定位了每个 collision sample 的轨迹来源。

## A. 当前完整数据流与 19 项结构核对

```text
target odometry / constant-velocity prediction
  → CooperativeViewpointCore 初始化固定 UAV→slot
  → φ0 + 2π slot/3 的 reference
  → 当前 φ0、φ0−15°、φ0+15° 的共同旋转 hypotheses
  → EncirclementReferenceBundle
  → FSM 将每机 reference 转成 local target / relative tracking offset
  → planner NOMINAL、风险触发 SIDE±，内部 A* / Local-SFC 修复
  → MINCO + static/dynamics/dynamic/swarm 检查和候选分类
  → TopologyCandidateBundle
  → 同 hypothesis / generation、共同世界时间的 team combinations
  → safety lattice → near-best K2 → mean-visible / All3 → hysteresis
  → joint P/T/yaw SCP、MINCO 重建、非线性安全验证
  → proposal → 3 ACK → commit → planner adoption
  → PolyTraj scheduled activation / optimized yaw
  → simulator actual odometry + camera/LOS → executed visibility CSV
```

### 1–3. 120° reference、slot 和 hypotheses

- `ros_ws/src/multi_uav_formation/include/multi_uav_formation/cooperative_viewpoint_core.h:455`：`encirclementAngles()` 明确生成 `θ_i=wrap(φ0+2π s_i/3)`；`encirclementReferencePositions()` 使用各机 offset 的平面模长与高度构造 reference，并非全队必然相同半径。
- 同文件 `initializeEncirclement()`，约 1043 行：枚举六种 slot permutation，优先 valid，然后最小化实际 bearing 到 slot 的总角位移；`estimateInitialPhi0()` 估计共同角。只初始化一次，之后不动态交换 slot。
- 同文件约 1378 行：只提出当前、−π/12、+π/12 三种共同角；通过 `selectFeasibleRotation()` 和 transition evaluation 得到 `arc_valid`。它们不是三个 UAV 可以独立偏离的角度向量。
- `ros_ws/src/multi_uav_formation/src/cooperative_viewpoint_manager.cpp:506`：outer sample-and-hold 发布最多三个 hypothesis；非 outer tick 只发布当前一个。generation、pending timeout、共同旋转状态都会限制候选。

### 4. reference 如何进入 planner

`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:814` 接收 bundle，对有效 hypothesis 取 `references[drone_id]`，形成 local target、goal、relative tracking offset 后调用 planner，保留 hypothesis id、φ0、generation。接收端还有最多三个 hypothesis 的契约（约 1549 行）。

因此不能仅改发布端角度生成：还必须处理 current hypothesis 身份、outer hold、选中后的 reference 持续状态，否则下一周期仍会回到统一 φ0。

### 5–7. tracking P/T 梯度与真实 restoring 公式

`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp:6300`：

设目标中心 `o(t)=o0+vo t`，参考相对位置 `q*`，实际相对位置 `q=p−o(t)`；`r*=||q*xy||`、`r=||qxy||`、`a=wrap(atan2(qxy)−atan2(q*xy))`、`h=qz−q*z`。

非 Elastic：`J_track=w_track ||p−o(t)−q*||²`。

Elastic 的冻结风险 `ρ∈[0,1]` 取静态、动态、swarm 与 visibility 风险中的最大激活值；不是 live deep-risk 的无界数值。设最小权重比例 `m`：

```text
α(ρ) = αbase + ρ(αmax−αbase)
s(ρ) = ρ smax
e_r  = signed distance of r outside [r*−s, r*+s]
J_region = e_r² + r*² [|a|−α(ρ)]_+² + h²
J_nominal_encirclement = (r−r*)² + r*² [|a|−αbase]_+² + h²
J_track = w_track [(1−ρ) J_nominal + ρ m J_region]
```

`elastic_tracking_contract.h` 对 αbase/αmax 还有 clamp。当前基准 angular deadband 为 20°；风险低时径向恢复仍是点半径，不是始终存在的 observation annulus。高度始终有点高度恢复。

`gradp` 为位置导数；局部时间导数含 `∂J/∂p · (v−vo)`，先前 piece 时间导数含 `∂J/∂p · (−vo)`，再经 MINCO 的系数/P/T 传播。因此这不是只用于初值的几何，而是持续改变 P/T 的优化目标。

### 8. 当前 directional J_vis 与 deep-risk

同文件约 4770–4935 行：live directional cost 是静态 LOS、最危险 dynamic LOS、FOV 风险的平方加权和，概括为 `w_vis(R_static²+R_dynamic,max²+R_fov²)`；采样与时间积分由调用层完成。它不是 team 累计 camera-time 的二值积分。

`tracking_visibility_geometry.h:75` 的 clearance continuation：设 `w=c_out−c_in`。

```text
c ≥ c_out: R=0
c_in < c < c_out: u=(c_out−c)/w, R=u²(3−2u)
c ≤ c_in: d=(c_in−c)/w,
          R=1+κ[d−(1−exp(−βd))/β]
          dR/dc=−κ(1−exp(−βd))/w
```

深度遮挡仍保留逃离梯度；这并不消除 reference 持续恢复，也不能创造已被候选结构排除的观察扇区。

### 9–11. NOMINAL / SIDE / A*、过滤与 27 组合

`planner_manager.cpp` 约 3550 行构造 NOMINAL，约 3942 行起构造 SIDE±；A* 是 SIDE/guide 修复过程的一部分，不是 team selector 中独立的第四种 candidate kind。Local-SFC 约束修复区段，MINCO 负责生成真实多项式。

候选消失发生在多个层次：

1. reference 的三共同角及 `arc_valid` 已先限定几何；
2. outer cadence / pending / current-only 限制是否实际规划 alternatives；
3. SIDE 风险触发、guide、A* 搜索、方向语义和 rejoin 条件；
4. Local-SFC、MINCO 求解、静态、动力学、动态 clearance 和分类；
5. bundle 新鲜度、epoch/start skew、generation/hypothesis 一致性；
6. common execution prefix、当前 hypothesis coverage；
7. team 优化前 source 必须 ABSOLUTE_SAFE、activation reserve 足够；
8. nonlinear safety、ACK、commit/adoption 新鲜度。

`topology_coordinator_core.cpp:466` 三重循环，但只接受相同 hypothesis 与 generation。每个 hypothesis 下若三机各有 NOMINAL/SIDE±，上界为 27；三个 hypothesis 完整存在时上界可达 81 个合法组合，不是整个系统永远只有 27。缺少 SIDE 时实际远少于上界。不得将三个 hypothesis 的所有单机候选任意混合。

### 12. selector 真实排序与额外保留策略

`topology_coordinator_core.cpp:284` 和 `team_visibility_preference.h:30`：

1. success/static/dynamics/safety_class>0 等 executable 条件；
2. safety lattice：minimum safety class，conflict pair 数，severe samples，conflict duration；若仍有 conflict，再看 separation；
3. equally-safe 集合中的 best K2，允许默认 0.01 的 near-best 差额；
4. mean visible count 最大；All3 最大；None 最小；spread 最大；
5. separation、dynamic clearance、temporal burden、tracking/native cost、hypothesis id；
6. topology hysteresis、outer φ0 sample-and-hold 可保留旧结果，并非单次排序结果必然执行。

关键修正：当前 safety 层是分级比较，不等同于“任何硬冲突都禁止提交”。若所有组合都冲突，代码仍有比较最小冲突组合的路径。

### 13–15. joint 变量、公式与 acceptance

`team_visibility_optimizer.cpp:170` 的变量为三机 MINCO 内部点 `P_i`、各 piece 虚时间 `τ_i`、yaw knots `Ψ_i`。固定边界 P/V/A；虚实时间映射保证正时长；采用 fixed-world-time 梯度与 MINCO chain rule。yaw 首 knot、trust region、rate/acceleration 约束独立保留。

`multi_uav_topology_coordinator.cpp` 的 continuous visibility 由 range smoothing、静态/动态 LOS smooth score、`exp(−FOV risk)` 组合，并给出位置和 yaw 导数。binary 使用共享 camera/range 与 LOS 判断，不是简单 threshold continuous score。

```text
Q2 = v1v2+v1v3+v2v3−2v1v2v3
b2 = 1−Q2
b0 = (1−v1)(1−v2)(1−v3)
J_K2 = mean(b2)
J_acc = mean[1−(v1+v2+v3)/3]
J_K2-cont = mean_windows[(mean_window b2)²]
J_blackout = mean_windows[(mean_window b0)²]
```

窗口宽为 `round(window/dt)+1`。Scenario A runner 明确覆盖权重：

```text
J = 2 J_K2 + 0.5 J_acc + 2 J_K2-cont + 0.5 J_blackout
    + 1 J_deviation + 1e−5 J_jerk + 0.02 J_yaw-prior
```

不能把 header 默认 continuity/blackout=0 当作 full ON 的实际权重。

`team_visibility_optimizer.cpp:333` 的 `measurableBenefit` 为 OR：K2 改善、continuity 改善、blackout 改善、mean-visible 改善、objective 改善，任一达阈值即可。默认阈值分别为 `1e−4、1e−5、1e−5、1e−3`；objective 绝对 `1e−5` 或相对 `1e−4`。另要求实际变量变化至少 `1e−5`。

约 1533 行 acceptance 默认只保护：K2 回退不超过 0.02、longest K2 loss 增加不超过 0.20 s、longest blackout 增加不超过 0.20 s。这里 K2 score 是 continuous Q2 均值，longest loss 来自 binary trace。没有独立 binary accumulated camera-time 或 All3 non-regression guard。

另有应修正的失败语义：约 608 行，`binary_valid=false` 时退回 `visibility.value>=0.5`。新 final binary acceptance 必须 fail closed，不能沿用这个替代。

### 16–17. proposal / ACK / commit / adoption / optimized yaw

消息位于 EGO Planner `traj_utils/msg`：`TeamTrajectorySolution` 携带 source ids、planning/coordination generation、activation time、三机 MINCO/yaw payload；`TeamTrajectoryAck` 回传身份与接受原因。

`team_solution_commit_contract.h` 验证 proposal 身份、每机 planning generation/source id、拒绝 ACK 与 activation lead；只有三机 ACK 齐全才 COMMIT。过期或身份不符不能算成功。

`planner_manager.cpp:960` 接收 proposal/commit。proposal 复查 source ABSOLUTE_SAFE、boundary P/V/A、dynamics/static/dynamic、Local-SFC、odom freshness 与 activation 连续性。约 1980 行 adoption 调用 `setLocalTrajFromOpt()` 并安装同 generation 的 yaw payload。

`traj_server.cpp:595` 优先执行 `optimized_yaw_at(t_cur)`；没有有效 optimized yaw 时才走 target-facing yaw。scheduled trajectory/yaw 使用同 activation time。不能仅看 optimizer 的 `optimized_yaw_valid` 就宣称执行成功，必须看到每机执行 telemetry。

### 18. executed visibility 与统计窗口

`native_egov2_rviz_scene.py:770`：实际 odometry 姿态/位置、目标位置、静态几何、当前 dynamic cylinders，计算 range AND static LOS AND dynamic LOS AND camera FOV。约 835 行的 planned measurement 是同一当前 odometry 位置配 command yaw 的对照，不是完整未来 planned trajectory 回放。

visibility CSV 保存 visible_count、各机四类判定、实际/command yaw、FOV margin、planned/executed mismatch；longest loss 按相邻真实时间差累计；ratio bucket 则按样本数统计。

本轮逐行复算六个 CSV，完全复现 feedback_25 的全 CSV loss decomposition：

| 三次合计 loss 秒数 | OFF | ON |
|---|---:|---:|
| static LOS | 11.860805 | 29.905988 |
| dynamic LOS | 8.293048 | 8.086773 |
| HFOV | 0.103589 | 33.102947 |
| VFOV | 1.964124 | 5.133160 |
| range | 0 | 0.570186 |

ON 中 UAV2 HFOV=32.466810 s；UAV1 static LOS=22.799920 s。分类优先级 range→static→dynamic→HFOV→VFOV，重叠原因不会重复累计。

重要统计口径发现：feedback_24 的逐 run mean-visible 恰好对应 `time_s>=2.0`，不是整份约 80.2 s CSV。整份 CSV 左端点积分 camera-time：OFF=228.837367/235.679159/234.963706，ON=206.066420/222.744959/215.992500 camera·s。

裁掉前 2 s 后，左端点积分分别为 OFF=222.837667/229.671302/229.170139，ON=200.163863/216.844919/210.191271，仍与报告 camera-time 有约一个采样间隔量级差异。报告给出的 227.317129→209.138979 应保留为“既有报告口径”，不能冒充本轮严格时间积分的复算结果。正式验证前必须固定 warm-up、端点权重、采样或时间加权均值，不可混用。这不改变 ON 明显下降的结论。

### 19. safety 分支覆盖与缺口

| 分支 | 已有检查 | 必须处理的问题 |
|---|---|---|
| joint P/T/yaw | nonlinear validator 全轨迹 static/dynamic/v/a/jerk；共同窗口 swarm；Local-SFC；yaw | 离散采样不是连续安全证明；跨 activation 与 yaw knot/switch 的连续性另查 |
| proposal ACK | source 身份、边界、static/dynamic/dynamics、SFC、odom/activation | commit/adoption 之后的状态变化不能只依赖之前 ACK |
| SIDE | A*/guide/SFC、MINCO、静态/动力学、dynamic 分类 | `IMPROVED_ONLY` 并不等于绝对安全 |
| post-check/retiming | 最终 polynomial 重新 risk/classify | 无 previous-safe 时 non-absolute 候选存在继续向 commit 走的路径 |
| KEEP_PREVIOUS_SAFE | remaining suffix 切片并重查 dynamics/static/dynamic/swarm | 不能仅使用 cached safe 标志；所有 retain 调用需统一使用时刻 |
| persistence | validator 确实有当前 remaining suffix 的四类检查 | validator fail 后不能再以“保持”为名执行未验证 trajectory |
| trajectory switching | freshness、未来 activation、部分 source continuity | 跨代际 swarm、position/yaw 接缝、命令跟踪误差需要 telemetry |

源码明确缺口一：`planner_manager.cpp:3648` 计算 `swarm_valid`，但 `existing_checks_passed` 仅由 dynamics/static 构成；随后 ABSOLUTE_SAFE 依据 dynamic threshold 分类。因此这个名称不是完整四类安全的证明。

源码明确缺口二：约 7350 行 post-check 后非 ABSOLUTE_SAFE 只在 previous-safe 可用时尝试 retain；没有一个在所有失败情形下必然拒绝 non-absolute commit 的统一出口。类似问题要一起审计 topology adoption fallback。

源码明确缺口三：selector executable 接受 `safety_class>0`；safety lattice 允许比较有冲突组合。这与新要求的 hard safety first 不等价。

不能据此断言 ON 16/5/11 个 collision samples 全由某一个分支造成。需要 trajectory id/generation/source 与碰撞样本同步后才可归因。现有 summary 还出现较大的 command yaw acceleration 峰值，必须检查 knot 插值与切换，不能只以 knot 离散检查通过证明实际 yaw acceleration 全程合规。

## B. 问题定位与 120°真实角色

| 位置 | 当前是否存在 120° |
|---|---|
| initialization seed | 是 |
| tracking reference | 是，共同 φ0 与固定 slot |
| continuous cost | 是，通过 slot angular deadband、径向与高度恢复 |
| selector 显式 120°距离奖励 | 未见；spread 为饱和偏好，不是等角硬目标 |
| selector 候选空间 | 间接存在，输入 hypotheses 本身只允许整体转动的等角 reference |
| hard trajectory constraint | 未见三机必须恰好 120°的硬约束 |

合理的因果机制是：共同旋转无法独立释放 UAV2 的不利 HFOV bearing 和 UAV1 的静态遮挡 bearing；reference 恢复与窄候选 family 进一步限制可达 alternatives；进入 selector 时经常只剩一个候选；K2 near-best 及 joint Q2/continuity 主导允许牺牲第三路可见；acceptance 又无 binary camera-time guard。因此 All3→exactly-two 的转换可能被优化器认可。

CSV 支持损失位置和结果；源码支持这一机制。但“某个 slot 直接导致某次具体遮挡”仍是待带轨迹来源 telemetry 验证的因果假说，不能写成已唯一证实。

## C. 累计可见时间当前是不是主目标

| 层次 | 当前角色 |
|---|---|
| reference/hypothesis | near-best K2 下的 mean-visible 偏好，受固定几何候选限制 |
| team selector | equally-safe + near-best K2 集合内的首要质量项；无显式 camera·s 字段 |
| joint objective | `0.5 J_acc` 次级 soft term，被 `2 J_K2+2 J_K2-cont` 等竞争 |
| measurableBenefit | OR 条件中的一项，不是必须改善项 |
| acceptance | 无 binary camera-time non-regression guard |
| executed metrics | 诊断项，不参与事前提交保护 |

同一 horizon、同一采样权重下，maximize mean-visible 与 maximize camera-time 数学等价。不能把“换字段名”说成 selector 的实质改进；真正改动是 reference 空间、protection baseline、统一二值评估口径、hysteresis 和 acceptance。

总体结论：当前并非端到端以累计 camera-time 为首要可见性目标。

## D. 拟修改结构（尚未实施）

```text
target / moving-obstacle prediction + shared static geometry + camera contract
  → time-varying visibility field, near/far two scales
  → {previous ACCEPTED adaptive, uniform 120° seed, existing fixed seed}
  → ONE AdaptiveViewpointGenerator
  → top-3 team hypotheses (each is three independent observation states)
  → existing NOMINAL / SIDE± / A* repair / Local-SFC / MINCO
  → hard-safe team set + K2/blackout protection
  → maximize strict binary camera-time
  → existing joint P/T/yaw with accumulated surrogate primary
  → nonlinear strict binary camera-time + safety acceptance
  → proposal / 3 ACK / commit / 3 adoption / optimized yaw
  → executed visibility + source telemetry
```

### D1. AdaptiveViewpointGenerator

输入不仅包括 φ0：统一 target/world-time snapshot、current UAV P/V/yaw、moving predictions、shared static/LOS query、camera contract、previous accepted 三机 viewpoint、fixed offset 和 uniform seed。

每个 team hypothesis 保留各机 `(θ,r,z,ψ)`。不让 downstream 根据 seed 类型分成三种 planner。旧 φ0 可作为 seed provenance/诊断，不能再成为执行状态的唯一表示。

visibility field：

```text
p(θ,r,z,t)=o(t)+(r cosθ,r sinθ,z)
V_binary = range AND static_LOS AND dynamic_LOS AND camera_FOV
v(p,ψ,t)∈[0,1] 复用同一 geometry 与 smooth score
S_field = Σ_k Δt_k Σ_i v(p_i(t_k),ψ_i(t_k),t_k)
```

yaw 使用共享 rate/acceleration model 从当前状态推进；不能一律假设瞬时 target-facing。近段 transition 从实际 UAV 状态接近观察区，远段跟随目标，避免给不可达的理想扇区虚高分。地图缺失或预测失效 fail closed，不标记高可见。

使用有界 coordinate/beam search：例如 24 bearings、3 radii、3 heights 的场缓存，各 seed 少量 coordinate sweeps，team 层保留 top-3 并去重。不要构造全部 bearing 三次笛卡尔积，不为 field samples 调完整 MINCO。

选择 K=3，因为现有 FSM/message/cadence 已围绕最多三个 hypothesis；暂不扩到 5。outer tick 缓存稳定 id/generation，inner tick 使用已接受 adaptive 状态；只在确认 selection/adoption 后更新 previous accepted，不能将 proposal 当 accepted seed。单靠更新 publisher 不足以完成此项。

### D2. 120° seed 化、diversity、annulus

```text
θ_i^seed = φ0 + 2π s_i/3
J_div = Σ_i<j [θ_min−|wrap(θ_i−θ_j)|]_+²
J_annulus = Σ_i ([r_min−r_i]_+²+[r_i−r_max]_+²)
J_height = Σ_i ([z_min−z_i]_+²+[z_i−z_max]_+²)
```

超过 θ_min 后 diversity 值与梯度为零。不再奖励接近 120°。seed penalty 默认 0；如需 tie-break，使用独立极弱项，而非原来的 `w_tracking` angular 恢复。

MINCO 仍可跟踪本轮 visibility-selected goal 以实现机动；但不能保留任何把目标角重置为固定 slot 的持续恢复路径。radial/height 采用非零宽度带，不是以 rho=0 收缩到一点。

diversity 也应进入 joint nonlinear evaluation 与解析 P 梯度/测试，不能仅靠 generator initial points 假定后续永不聚集。不可将 diversity 改为削弱 collision clearance。

### D3. 新 selector 与 binary acceptance

所有 alternatives 在同一 world-time 起点、同一 H 上评估，端点不重复算 dt：建议左端点积分 `Σ_{k=0}^{N−2}(t[k+1]−t[k]) count[k]`。不要使用 N 个点各乘 dt 得到 H+dt。

从已验证 current/local team 构造 protection reference，不再把每轮最高 K2 无条件抬升成追逐目标。若 reference 不安全，不放宽 safety；明确进入安全恢复路径，再在 hard-safe 候选中建立保护参考。

```text
hard safety
→ binary K2 >= reference−eps_K2
  longest K2 loss <= reference+eps_loss
  longest blackout <= reference+eps_blackout
→ maximum T_cam_binary
→ All3 (same-H mean-visible is redundant)
→ transition/smoothness/diversity/tracking/native tie-break
```

hysteresis 不得保留明显更低 camera-time 的旧结果；只在积分差小于量化 tie tolerance 时起作用。

joint：主可见性项为 `J_acc`，K2/continuity/blackout 作为 binary protection，soft 权重默认 0 或合计明显小于 accumulated 项，不能允许旧 runner 显式覆盖成旧权重后仍声称新模式生效。安全和必要的轨迹实现约束不是被 camera-time 替代。

acceptance 采用 **non-regression + objective tie-break** 而非每轮严格 binary improvement：0.1 s 栅格会产生 plateau，严格改善将拒绝跨边界前有用的小步。

```text
binary_valid_before AND binary_valid_after
T_after >= T_before−epsilon_regression
AND K2/longest-loss/blackout protection
AND hard safety
AND (T_after > T_before+epsilon_improve
     OR (binary tie AND surrogate/objective genuinely improves))
```

epsilon_regression 建议仅浮点级（1e−6 camera·s）；epsilon_improve 同样小于一个 sample-camera quantum。不能给整采样量级的回退额度，让 K2 收益覆盖真实第三路损失。All3 可增加独立 non-regression guard，以防同 camera-time 的结构性恶化；需要明确其与 K2 protection 的一致性。

### D4. 两尺度 horizon

当前 near joint H=1.5 s、visibility dt=0.10 s、safety dt=0.05 s；outer prediction 1.5 s、outer period=0.5 s，reference core visibility default=2 s；Scenario A target speed=0.928 m/s。

拟保留 near H=1.5 s 和现有 safety 精度；far H=3.5 s、dt=0.25 s，仅用于 viewpoint screening。目标在 near/far 分别移动约 1.39/3.25 m，far 额外提供约 1.86 m 的遮挡预警。远期动态模型不确定，不能作为延长 full MINCO 和硬安全承诺的理由。

最终 far 参数需按新 generator 的实际 p95/p99 latency 和 moving prediction validity 校准，不把 3.5 s 当实验已验证的最优值。

## E. Safety fix plan

1. 统一 source-safe 契约：ABSOLUTE_SAFE 必须包括有效的 static/dynamic/swarm/dynamics，unknown 不能自动 safe；visibility 不能降低阈值。
2. 每个提交出口在最终 polynomial、最终 activation epoch 上再验证；retiming 后不能沿用旧 risk。non-absolute 不能因 previous-safe 缺失而直接执行。
3. retain/persistence 每次用最新 world-time 切 suffix 验证；失败则进入现有安全失败/停止处理，停止轨迹自身也需检查，不能宣称悬停必定安全。
4. team 验证采用真实 scheduled bridge：旧轨迹直到 activation、新轨迹之后；把 pending/scheduled peer generations 一并考虑。维持 3 ACK 身份链，不作单机静默替换。
5. 对 Local-SFC、v/a/jerk 与 yaw 限制保持或加强现有验证；检查 piece 内极值和 yaw 插值/跨轨迹接缝，不放宽阈值来让测试通过。
6. 新增 trajectory-source telemetry：drone、traj id、source kind、hypothesis/generation、team solution id、prediction/activation/validation time、safety result、switch reason、command/odom error；collision CSV 同步 id 和 obstacle id。
7. 一次 full ON 短测先归因碰撞与验证链路。仍有 collision 或链路不完整，不进入正式 3×OFF/3×ON；不能以 visibility 改善宣布通过。

## F. 文件级修改计划

下列为拟修改集合，不表示已修改。若 Phase 2 发现新增依赖，应先记录范围与原因。

### multi_uav_formation

- `include/multi_uav_formation/cooperative_viewpoint_core.h`：统一 adaptive accepted 状态/seed 接口，终止运行时强制 uniform reference 重建。
- `src/cooperative_viewpoint_manager.cpp`：调用 generator、top-3 发布与 cache、accepted feedback、far-horizon 配置。
- `include/multi_uav_formation/team_visibility_preference.h`：camera-time/protection/tie 语义。
- `include/multi_uav_formation/topology_coordinator_core.h`：binary camera-time、longest-loss、protected reference 数据。
- `src/topology_coordinator_core.cpp`：hard-safe 集合、统一积分、protected camera-time 排序、hysteresis。
- `include/multi_uav_formation/team_visibility_optimizer.h`：binary metrics/acceptance 参数、diversity 参数。
- `src/team_visibility_optimizer.cpp`：J_acc 主目标、binary fail-closed 重评估、acceptance、diversity 梯度。
- `src/multi_uav_topology_coordinator.cpp`：共享 environment、参数、binary telemetry、selection/adoption feedback。
- `include/multi_uav_formation/tracking_visibility_geometry.h`：仅按需抽取现有 shared evaluator，不复制第二套 LOS/FOV 语义。
- `scripts/native_egov2_rviz_scene.py`：source-aware executed metrics、固定积分与窗口。
- `launch/native_egov2_rviz.launch`：新参数与完整 ON 配置贯通。
- `CMakeLists.txt`：新增模块/test targets。
- `test/cooperative_viewpoint_contract_test.cpp`、`test/topology_coordinator_contract_test.cpp`、`test/team_visibility_optimizer_contract_test.cpp`：新语义回归、有限差分与 acceptance tests。

### EGO-Planner-v2/swarm-playground/tracking_ws/src/planner

- `plan_manage/include/plan_manage/planner_manager.h`：adaptive identity/accepted state 与 source validation telemetry。
- `plan_manage/src/planner_manager.cpp`：hypothesis/accepted feedback、各 commit/retain 分支 hard-safe 出口与 activation 时刻复查。
- `plan_manage/include/plan_manage/ego_replan_fsm.h`、`plan_manage/src/ego_replan_fsm.cpp`：统一 adaptive references 的 current/outer 生命周期，失效时使用同一 generator seed/安全状态，而非恢复另一长期 family。
- `traj_opt/include/optimizer/poly_traj_optimizer.h`、`traj_opt/src/poly_traj_optimizer.cpp`：去除 slot angular restoring；radial/height bands 与 visibility-selected reference 语义。
- `traj_opt/include/optimizer/elastic_tracking_contract.h`、`traj_opt/test/elastic_visibility_contract_test.cpp`：seed-only/带宽/diversity 相关契约。
- `plan_manage/src/traj_server.cpp`：source telemetry 与 yaw/scheduled-switch 验证，保留 optimized yaw 优先级。
- `traj_utils/msg/EncirclementReferenceHypothesis.msg`、`EncirclementReferenceBundle.msg`：按需增加 adaptive hypothesis/generation/seed provenance，不建立三 family 协议。
- `traj_utils/msg/TeamTrajectorySolution.msg`：binary camera-time、binary evaluation validity 与保护结果。
- `plan_manage/launch/advanced_param.xml`、`plan_manage/launch/run_in_sim.launch`：下发 radial/height 与安全参数，不改安全阈值含义。

### 运行入口

- `run_constvel_gradient_rviz.sh`：按需贯通新模式参数。
- `visibility_effect_validation_20260910/run_effect_ab.py`：不要静默改变旧实验再复用旧标识；新 runner 单独保存，固定同场景与新权重、积分口径。

### 新增文件

- `ros_ws/src/multi_uav_formation/include/multi_uav_formation/adaptive_viewpoint_generator.h`
- `ros_ws/src/multi_uav_formation/src/adaptive_viewpoint_generator.cpp`
- `ros_ws/src/multi_uav_formation/test/adaptive_viewpoint_generator_contract_test.cpp`
- `ros_ws/src/multi_uav_formation/test/visibility_first_acceptance_contract_test.cpp`
- `visibility_first_validation_20260910/run_scenario_a_short.py`
- `visibility_first_validation_20260910/analyze_executed_visibility.py`

## G. 参数与公式变更清单

参数初案（尚未写入生产配置）：

- `adaptive_viewpoint_top_k=3`，bounded bearing/radius/height sample counts 与 search sweeps。
- `adaptive_far_horizon=3.5`、`adaptive_far_dt=0.25`；保留 near 1.5/0.10、safety 0.05。
- `adaptive_theta_min_deg`：先沿用现有 25°最低分离意图，而非任意升为 60°；经可达性测试后调整。
- `observation_radius_min/max`、`observation_height_min/max`：从当前 offsets 与 camera feasible region 设非零带宽；禁止无审计扩大到不安全区域。
- `adaptive_seed_weight=0`、transition cost/tie tolerance、search budget。
- `team_binary_camera_time_regression_tolerance=1e−6`、binary improve/tie tolerance。
- `team_binary_k2_regression_tolerance`、`team_longest_k2_loss_tolerance`、`team_longest_blackout_tolerance`：先不放宽已有保护数值，改为明确 binary 语义。
- `team_accumulated_weight` 与 K2/continuity/blackout soft weights 联合迁移；保护不依赖这些 soft weights。
- source telemetry enable 与验证 epoch/完整性字段。

公式变更：固定 slot angular restoring→无持续 seed 恢复；径向/高度点恢复→bands；spread reward→minimum diversity hinge；K2 主导 weighted sum→J_acc 主可见性项+保护；mean-only ranking→同 horizon binary camera-time；OR benefit 无回退限制→binary non-regression 前置；surrogate fallback binary→invalid 拒绝。

## H. 计算量、测试门槛与风险

cheap field 约 `3 UAV × 24 bearings × 3 radii × 3 heights × 15 far samples`，约 9720 observer-time 查询/outer tick，需缓存 LOS/预测并测量实际成本。此为上界设计估算，不是 measured latency。top-3 保持原 three-hypothesis MINCO 上界，但更有效的候选会使平均 planner 工作量上升。joint 变量维数不因 far field 扩大；binary re-evaluation 增加 near-horizon 查询；全面安全验证也会增加开销。

必须按阶段通过：build；新 generator 单元/墙遮挡/seed independence；diversity 有限差分与阈值外零梯度；candidate 到 planner 的契约；camera-time selector；binary invalid/回退 rejection；P/T/yaw 旧梯度；ACK identity；各 fallback/retime/switch 安全契约。

构建入口参考已有验证：在 `ros_ws` 的正确 ROS/catkin 环境执行 `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2`。现有 CMake 中一些 contract 是 executable 而非自动注册的 catkin test；不能仅凭 `catkin test` 无失败就宣称它们运行过，需逐个记录退出码。

短测通过标准：adaptive triggered、偏离 uniform seed 的 hypothesis 确实进入 MINCO、joint P/T/yaw 非零、proposal/3 ACK/commit/3 adoption、三机 optimized yaw executed、source-aware safety 无碰撞。任一缺失都不能写 PASS；结束后清理仅本次启动进程。

实施风险 HIGH：不仅涉及目标权重，还涉及 φ0-only 状态升级、candidate identity/accepted lifecycle、activation-time safety、shared binary 语义、yaw 执行连续性。更宽几何可能增加 crossing 和 planner reject；需用 staged contracts 和一次短测逐步解除风险。

## PHASE 1 必填结论

```text
CURRENT_ARCHITECTURE_CONFIRMED: YES

120_DEGREE_CURRENT_ROLE:
initialization seed + fixed-slot tracking reference + continuous restoring cost;
indirectly restricts selector input; not an exact 120-degree hard trajectory constraint.

GOOD_CANDIDATE_MISSING_BEFORE_SELECTION: YES

ACCUMULATED_VISIBILITY_IS_PRIMARY_OBJECTIVE_NOW: NO

PROPOSED_NEW_ARCHITECTURE:
shared future visibility field → unified adaptive generator seeded by
previous accepted / uniform 120 / existing fixed → top-3 team hypotheses →
existing SIDE/A*/Local-SFC/MINCO → hard safety and binary K2/blackout protection →
binary camera-time selection → J_acc-primary joint P/T/yaw →
binary non-regression acceptance → existing team commit/execution chain.

FILES_TO_CHANGE: See section F, per-file list.
NEW_FILES_TO_ADD: See section F, new-file list.
FORMULAS_TO_CHANGE: See sections D and G.
PARAMETERS_TO_ADD: See section G.

SAFETY_FIX_PLAN:
Close non-absolute/swarm classification and commit gaps; revalidate final
polynomial at activation, including retain/retime/bridge; add source telemetry
before attributing collision samples; preserve all safety thresholds.

EXPECTED_COMPUTE_COST_CHANGE:
Additional bounded cheap field queries and binary/safety evaluation;
no far-horizon full-planner expansion; same top-3 hypothesis cap;
average planning cost may rise; latency not yet measured.

IMPLEMENTATION_RISK: HIGH
```

## 当前交付状态（不是改造完成声明）

```text
PRODUCTION_SOURCE_CHANGED: NO
CURRENT_ARCHITECTURE_AUDITED: YES
STRUCTURE_REPORT_COMPLETED_BEFORE_CODE_CHANGE: YES
MODIFICATION_PLAN_COMPLETED_BEFORE_CODE_CHANGE: YES
PHASE_2_STARTED: NO
BUILD_RUN_THIS_PHASE: NO
UNIT_TEST_RUN_THIS_PHASE: NO
GRADIENT_TEST_RUN_THIS_PHASE: NO
CONTRACT_TEST_RUN_THIS_PHASE: NO
SCENARIO_A_RUN_THIS_PHASE: NO
SIMULATION_STARTED_THIS_PHASE: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

不得把没有运行写成 BUILD/TEST PASS，也不得把尚未实施的 generator、binary guard、two-scale horizon 写成 YES。
