# Feedback097 — Coordinator 与 Optimizer 对同一未来风险的判断分叉（只读审计）

工作目录：`/home/bob/ALP/egov2_fc65423_constvel`
审计运行：`runs/20260922_204715_37427`（ON，FULL，stress 场景）
本轮只读：未改代码、未调参数、未跑仿真、未访问 RRCT。

---

## 1. 对齐证据（contract 1）

`HANDOFF_CONTRACT_CREATED`（coordinator）：

```text
id=1 outgoing=1 incoming=2 stable=0
acquire=1790081275.722174644  preserve=1790081276.222174644
overlap=0.500 margin=0.200 m2_before=1.000000 m2_min=-1.329706
m2_time=1790081276.222174644   limiter=STATIC
```

`HANDOFF_CONTRACT_CREATED` 的 `m2_before` = `activation_margins[outgoing]`，即 **outgoing 在 activation 时刻 margin = 1.0**；
`m2_min` 出现在 **窗口末端**。

同一 contract 到 optimizer：

```text
[TEAM_VISIBILITY_ALREADY_SATISFIED] drone_id=1 contract_id=1
  window=[1790081275.722174644,1790081276.222174644] margin_min=1.000000 required=0.200000
[TEAM_REFINEMENT_SOLVE] drone=1 mode=PT margin=-inf->1.000000 dp=0 dtau=0 dT=0
```

**同一个 absolute world time `1790081276.222174644`：coordinator 得 M2 = −1.329706，optimizer 对 drone=1 得 margin = 1.000000。**

### 用实测数据判定谁对

用 `visibility_trajectory.csv` 的实测位置 + 场景 39 根圆柱自行复算（`static_los_margin_=0.08`，smoothing `0.15`，故 clearance ≥ 0.23 ⇒ risk = 0 ⇒ margin = 1.0）：

| contract | 窗口 | uav1 最小 clearance | uav2 | uav3 | 真值 margin |
|---|---|---|---|---|---|
| **1** | 1790081275.722–1790081276.222 | **0.7973 m** | 1.5697 m | 0.7104 m | **全部 1.0** |
| 2 | 1790081281.502–1790081282.002 | 0.1354 m | 0.5043 m | 0.5043 m | uav1 略受阻 |

⇒ **optimizer 的 margin = 1.0 与真值一致；coordinator 的 M2 = −1.329706 在 contract 1 上连符号都错。**

反解 `directionalClearanceRisk`（`tracking_visibility_geometry.h:199-229`）：
risk = (1 − m2)/2 = 1.165 ⇒ d ≈ 0.406 ⇒ coordinator 的 ranked[1] 机 static clearance ≈ **0.019 m**，
而同一时刻实测 0.710 m —— 偏差约 **0.69 m**。

---

## 2. FIRST_LOGICAL_DIVERGENCE

**位置：`ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp`
函数：`MultiUavTopologyCoordinator::assessPredictiveRelay()`（665–726 行），关键在 684–695 行**

```cpp
const auto &local = execution_[drone];                       // 684
...
const double tau = std::max(0.0, std::min(local.duration,
                            world_time - local.start_time)); // 689-690
samples[drone] = continuousVisibility(
    drone, local.traj.getPos(tau), target, world_time, yaw); // 694-695
```

其中 `world_time = activation + min(relay_horizon_, horizon)`，**`relay_horizon_ = 2.0 s`**（677-678 行）。

`execution_[drone]` 的来源（431-434 行）：

```cpp
execution_subscribers_[drone] = node_.subscribe<traj_utils::PolyTraj>(
    "/drone_"+std::to_string(drone)+"_planning/trajectory", 20, ...);  // planner 的 rolling Local 轨迹
```

**分叉点：coordinator 把"plan 时刻的快照多项式"外推到 2.0 s 之后当作 observer 位置；
optimizer 用的是刚 realized 的轨迹。二者必然不是同一条多项式。**

实测轨迹存活时长（`visibility_trajectory.csv` 的 `trajectory_id` 分段，drone_1：392 条，p50 = **0.266 s**，均值 0.557 s，最大 2.002 s）。

Contract 1 建立于 `1790081273.662`，此刻 coordinator 的 `execution_[]` 恰为：

| UAV | 所用 traj_id | 发布时刻 | start / duration | 实际存活 |
|---|---|---|---|---|
| drone_0 | 120 | 1790081273.470 | 1790081273.569 / 3.115 | **0.501 s** |
| drone_1 | 118 | 1790081273.559 | 1790081273.659 / 3.089 | **0.200 s** |
| drone_2 | **121** | 1790081273.661 | 1790081273.656 / 4.417 | **0.066 s** |

而 coordinator 把它们外推到 **τ = 2.563 s**。

**⇒ 外推视界 / 轨迹实际寿命 = 5×（drone_0）到 39×（drone_2）。**
`drone_2` 的 traj 121 在 CSV 中只存在 2 个采样（0.066 s），却被外推 2.56 s 用于判定 LOS 被遮挡。

---

## 3. 逐项排除

| 候选 | 判定 | 依据 |
|---|---|---|
| **A. outgoing/incoming 选错，未优化真正 limiting UAV** | **否** | `outgoing = ranked[1]`（666-726 行），即 **M2 的 limiting 机本身**；contract 1 的 SCP 确实跑在 drone=1(outgoing) 与 drone=2(incoming)；`result.limiter` 与 `outgoing` 同取 `ranked[1]`，传播正确 |
| **B. 两侧使用不同 trajectory** | **是（根因）** | 见 §2 |
| **C. world-time / relative-time 映射不同** | 否 | 两侧都是 `activation + t`；coordinator 窗口 `[acquire, preserve]` 与 optimizer `relative_start/relative_end`（`poly_traj_optimizer.cpp:2821-2830`）逐值相同 |
| **D. target / dynamic prediction revision 不一致** | 否 | 两侧都用**同一快照的常速度外推**：coordinator `p + v*(t − epoch)`（`targetAtWorldTime`, 1106-1120）；optimizer `object_p_ + object_v_*t`，其中 `object_p_ = targetPositionAt(activation)`（`planner_manager.cpp:2893-2897`）。`target_snapshot_epoch` 由 planner 填充（`planner_manager.cpp:4494`）并由 coordinator 读取（1690 行），非默认 0 |
| **E. yaw / FOV camera semantics 不一致** | 否（非本因） | 本 run 的 limiter 是 **STATIC**，yaw 只影响 component[2]；两侧 FOV 参数同源（`tracking_camera_`） |
| **F. margin 数值定义/归一化尺度不一致** | 否 | coordinator `visibility_occlusion_margin_ = 0.08`（345 行）、`visibility_los_smoothing_ = 0.15`（348 行）与 optimizer `static_los_margin_ = 0.08`、`team_visibility_los_smoothing_ = 0.15` 相同；两者都经同一 `loadScene` 加载同一场景 |
| **G. M2 与单机 margin 不是同一种 quantity** | **否** | 两者都调用 `multi_uav_formation::composeVisibilityMargin()`（`tracking_visibility_geometry.h:85-114`），4 个 component 与 `directionalClearanceRisk` primitives 完全同源 |
| **H. 其他** | — | 唯一额外发现：`RelayAssessment` **不记录 `min_world_time` 处的 limiting UAV 索引**，只记 component（720 行），contract 无法向下游指明该修哪一架 |

---

## 4. 附带确认

### 4.1 `ZERO_MODIFICATION_CERTIFICATE_BUG`：**YES**

`planner_manager.cpp` 中 `teamReferenceScheduleCallback()` 的 ACK 之后：

```cpp
if (message->handoff_contract_active) {
  team_refinement_open_ = true;                       // ← 与"是否真有修正"无关
  pending_team_trajectory_.handoff_refinement = true;
}
```

只要 SCP 走成功路径就置位，**包括 `TEAM_VISIBILITY_ALREADY_SATISFIED`（ΔP=Δτ=ΔT=0、轨迹未变）**。
随后 `processPendingTopologyCoordination()` 的 COMMITTED 分支因 `team_refinement_open_` 为真而发放
`TeamImprovementCertificate`，于是在一个**并不存在的 Team improvement** 上保护了整段认证窗口。

**这正是本 run `LOCAL_QUALITY_OVERWRITE_BLOCKED = 449` 而 3 次 refinement 全部零修改的原因**
（`TEAM_IMPROVEMENT_OVERRIDE = 0`，没有任何安全后继被误挡）。

### 4.2 `LATENCY_TELEMETRY_BUG`：**YES**

```cpp
const auto wall_start = std::chrono::steady_clock::now();   // 2879 行：函数入口
...
const double refinement_wall_begin = ros::WallTime::now().toSec();  // 3222 行：SCP 之前
...
solve_ms = now - wall_start;                 // 3295 行 → 实为"函数入口→结果"
total_ms = now - refinement_wall_begin;      // 3298 行 → 实为"freeze→结果"
```

`refinement_wall_begin > wall_start`，故 `total_ms < solve_ms` 是必然结果（实测 0.588 vs 1.838）。
两个字段都**不是** T/PT SCP 本体耗时；`solve_ms` 还包含 Team reference realization / MINCO / 候选评估。
标签与实测区间不符，且没有任何计数器度量 SCP 本体。

---

## 5. 结论字段

```text
ROOT_CAUSE:
  Coordinator 在 assessPredictiveRelay() 中用"最近一次发布的 rolling Local 轨迹"
  (订阅 /drone_N_planning/trajectory) 外推到 activation+2.0 s 作为 observer 位置来评估
  M2；而该轨迹的实际存活中位数仅 0.266 s（本 contract 涉及的 3 条分别为 0.501/0.200/0.066 s）。
  外推视界是轨迹寿命的 5–39 倍，得到的是"若永不重规划会怎样"的虚构几何，
  因此报出 −1.33…−4.50 的负 margin。optimizer 用刚 realized 的轨迹在同一世界时刻评估，
  得 margin = 1.0，与实测几何一致。

FIRST_LOGICAL_DIVERGENCE:
  file  : ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp
  func  : MultiUavTopologyCoordinator::assessPredictiveRelay()
  lines : 684-695
  var   : execution_[drone].traj.getPos(tau), tau = world_time - local.start_time
          world_time 最远达 activation + relay_horizon_(2.0s)
  src   : execution_[drone] ← executionCallback() ← /drone_N_planning/trajectory (431-434 行)
  log   : contract 1 建于 1790081273.662，使用 traj 120/118/121（存活 0.501/0.200/0.066 s），
          外推至 τ=2.563 s；同刻实测 clearance ≥0.710 m 而 coordinator 报 M2=-1.329706

COORDINATOR_LIMITING_UAV: 未记录（RelayAssessment 仅存 limiter 分量，不存 min_world_time 处的 UAV 索引）
CONTRACT_OUTGOING: 1
CONTRACT_INCOMING: 2
OPTIMIZER_TARGET_UAV: 随 schedule 到达者；contract 1 → drone 1(outgoing) 与 drone 2(incoming)；contract 2 → drone 1
SAME_WORLD_TIME: YES
SAME_TRAJECTORY: NO
SAME_VISIBILITY_SEMANTICS: YES

ZERO_MODIFICATION_CERTIFICATE_BUG: YES
LATENCY_TELEMETRY_BUG: YES

RECOMMENDED_MINIMAL_FIX:
  1)（主修）让两侧评估同一个多项式：把 M2 的 per-UAV margin/limiter 权威移到 planner 侧
     —— 在 TopologyCandidateBundle 中携带 planner 已用 teamMarginAt() 算出的
     当前候选在各 world-time sample 的 margin 与 limiter，assessPredictiveRelay() 直接
     排序这些值，不再对 execution_[drone] 做多项式外推。
     不新增第二套评估器、不改 margin 定义、不放宽任何阈值。
  2)（配套）RelayAssessment / PersistentHandoffContract 增加 M2-limiting UAV 索引并向下游传播，
     使 refinement 明确瞄准真正 limiting 的那一架。
  3)（修 4.1）仅当 SCP 真的改动了轨迹时才置 team_refinement_open_ / handoff_refinement，
     即排除 reason == "TEAM_VISIBILITY_ALREADY_SATISFIED"（或要求 ΔP+ΔT > eps 且
     margin_after > margin_before），避免为零修改结果发放 TeamImprovementCertificate。
  4)（修 4.2）在 runTeamContractSCP() 调用前单独取 scp_wall_begin 供 solve_ms 使用，
     refinement_wall_begin 继续供 total_ms，并改正标签。

AUDIT_ONLY: YES
PRODUCTION_CODE_CHANGED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
