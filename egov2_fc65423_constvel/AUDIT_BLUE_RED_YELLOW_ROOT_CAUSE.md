# ALP 根因审计报告（本轮只定位，未修改任何源码）

场景：`long_cylinder_forest.json`，FULL ON，真实仿真 `p1p2c`（目标运动段 t=0–78 s，30 Hz 轨迹 + 逐帧可见性）
RRCT_ACCESSED: NO ／ RRCT_CHANGED: NO

---

## 总体判断

```
BLUE_BOX_ROOT_CAUSE:
  合围模式下 tracking 代价只有"半径死区带 + 高度带"，源码明文注释
  "no bearing prior"；Joint/team 参考因 H_EVAL=1.5 s 而单机已验证覆盖只有
  0.70~0.79 s 被结构性拒绝（team reference 可用率 0/1627）。于是方位角
  完全没有恢复力，横向偏移被永久保留。实测半径误差中位数仅 0.10~0.16 m，
  而方位误差中位数 85°~122° —— 正是"半径回来了、方位回不来"。

RED_BOX_ROOT_CAUSE:
  同一根因（无方位权威）导致该机已长期偏离；LOS 确实被挡（累计 2~4 s），
  但 LOS 恢复通道只在 conflict 周期内产生一次性的观察面/侧向候选，
  既没有把"已偏离的方位"纳入恢复目标，也没有持续恢复权威，
  于是"短暂丢失视线 → 尝试一次 → 回到原来的错误方位"，不主动回到可恢复
  LOS 的位置。

YELLOW_BOX_ROOT_CAUSE:
  两架同时偏离 = 两架同时没有方位权威。二者径向都各自落在死区带内，
  切向自由；Joint 通道（本应提供角度修正）全程不可用，
  因此两架各自漂到不同方位后无法被拉回。
```

---

## 逐条假设结论

### 1. SIDE_AUTHORITY_NOT_CLEARED: **NO**

证据：
- `[clearance-gain-decision]` 中 SIDE 候选的接受/拒绝**每个规划周期独立重算**，不是沿用上一周期结论；`candidate.risk`、`conflict` 都在本周期重建。
- SIDE 尝试次数与 `risk.triggered` 强相关：本段 `NOMINAL` 来源占偏离采样的 ~90%，SIDE 只占 ~10%。
- SIDE 侧信息（`last_candidate_kind_` / `last_candidate_obstacle_id_`）只在 `body_conflict_active` 为真时参与切换迟滞；无冲突时该分支不进入。

**排除**：不是"旧 SIDE 权威没退出"。

### 2. NOMINAL_CONTAMINATED_BY_SIDE: **PARTIAL（但有真实机制，见 8/9）**

证据：
- 偏离采样中 `NOMINAL` 占绝大多数（uav1 1379/1755、uav2 2042/2285、uav3 1998/2239）。
- 所以偏离**不是**由 SIDE 候选造成的，而是 **NOMINAL 候选自身的语义不完整**。
- 但"NOMINAL 被 SIDE 形状污染"这一具体路径没有找到证据：NOMINAL 的位置种子来自当前 active 状态 + 正常 local target（`getLocalTarget()` 取全局轨迹前方 6.5 m 点），未见 SIDE guide 注入。

**修正**：真正的机制不是"污染"，而是 **NOMINAL 的 tracking 代价本身不含方位项**（见 8）。这比污染更根本。

### 3. CURRENT_LOS_LOSS_WITHOUT_RECOVERY_AUTHORITY: **YES**

证据（逐帧可见性 CSV，目标运动段）：
```
uav1: LOS 被挡 5 段 共 3.99 s  最长 1.16 s  (t=43.68-44.84 / 45.34-45.94 / 48.81-49.71 / 70.91-71.51 ...)
uav2: LOS 被挡 1 段 共 1.13 s  最长 1.13 s  (t=71.61-72.74)
uav3: LOS 被挡 2 段 共 2.09 s  最长 1.37 s  (t=53.74-55.11 / 58.25-58.97)
```
- 遮挡是**真实存在**的（不是度量假象）：`static_los_clear=0` 或 `dynamic_los_clear=0`。
- 但这些遮挡发生时，该机**已经处于 85°~122° 的方位偏离状态**，且恢复通道没有把它当作"回到可恢复 LOS 位置"的目标。
- 关键语义缺陷：LOS 恢复只在 conflict 窗口内一次性生成观察面/侧向候选，**没有"持续恢复权威"**，也没有把 `distance_to_true_nominal` 作为恢复收敛判据。

### 4. JOINT_REFERENCE_LOCKED_TO_SIDE_SEED: **NO（更严重：JOINT 参考根本不存在）**

证据（本轮运行）：
```
team-reference-feedback  总反馈=1627   其中 NO_USABLE_TEAM_REFERENCE=1627  (100%)
team-transaction         TRANSACTION_JOINT_REJECTED=287   SUCCESS=0
team-reference-feedback  reason=NO_USABLE_TEAM_REFERENCE 占 1328/2655
```
- `team_reference_.usable()` 判据：`id && world>=epoch && world<end` —— id 从未被赋有效值。
- `TeamTrajectoryReference::accept()` 由 Joint 提交驱动；Joint 从未提交成功。

**排除**"Joint 把 SIDE seed 当参考"：Joint 根本没有产出参考，谈不上锁定。

### 5. ELASTIC_REGION_REMAINS_TOO_WIDE_AFTER_CONFLICT: **NOT_PROVEN（但弹性角隙确实存在且未被约束）**

证据：
- 源码存在 `elasticAngularSlack(rho, encirclement_tracking_active_, ...)`，即角向松弛是 `rho` 的函数。
- 但本轮偏离采样的**来源是 NOMINAL tracking 代价本身**（径向死区带），不需要弹性松弛来解释：`band_error` 在 `|Δr| ≤ 0.35` 时**代价恰为 0、梯度恰为 0**，与 `rho` 无关。
- 实测半径误差中位数 0.10~0.16 m < 0.35 m → 落在死区内 → 径向也无力。

**修正**：弹性松弛是**放大器**而非根因；根因是 tracking 代价的结构性缺失。

### 6. OLD_SCHEDULED_SIDE_TRAJECTORY_ACTIVATES_LATE: **NOT_PROVEN（本轮未观察到该链）**

证据：
- 偏离段内轨迹来源统计以 NOMINAL 为主，SIDE 只占 ~10%，且未见"冲突结束后旧 SIDE 才激活"的时间错配特征。
- 轨迹 ID 随时间是单调递增的正常滚动（uav2: t=0→709 线性推进），未见回退到旧 ID。

**未证实**，不作为根因。

### 7. STALE_LOS_PLANE_PERSISTS_AFTER_REASON_END: **NO**

证据（本轮审计行）：
```
[los-occlusion-audit] LOS_KNOWN_OCCLUSION_WITHOUT_PLANE_COUNT=15
                      LOS_WINDOW_LIMITED_COUNT=5  LOS_RECOVERY_NOT_REACHED_COUNT=16
                      LOS_PLANE_CREATED_COUNT=27  LOS_PLANE_LOST_COUNT=0
                      LOS_PLANE_VIOLATION_COUNT=19
```
- 平面创建 27 次即被消费，`LOS_PLANE_LOST=0`，未出现"应删不删"的堆积。
- 平面上轮已修的"防止丢失"没有过度保留：`active_start/active_end` 现在由真实遮挡区间给出（`LOS_OCCLUSION_ENTER_REL/EXIT_REL`），本轮实测窗口已与遮挡区间一致。

**排除**。

### 8. NORMAL_TRACKING_RECOVERY_TOO_WEAK: **YES —— 这是第一根因**

源码（`traj_opt/src/poly_traj_optimizer.cpp`，合围分支）：
```cpp
if (encirclement_tracking_active_)
{
  // Visibility-selected goal supplies the MINCO seed and endpoint.
  // Along the trajectory only radius/height bands restore; no bearing prior.
  const Eigen::Vector3d q = p - object_p_ - object_v_t * t;
  const Eigen::Vector3d seed = object_rotation * relative_tracking_p_;
  const double radius = q.head<2>().norm();
  const double desired_radius = seed.head<2>().norm();
  const auto band_error = [](double e, double width) {
    return std::copysign(std::max(0.0, std::abs(e)-width), e);
  };
  const double er = band_error(radius-desired_radius, observation_radius_band_);   // 0.35
  const double ez = band_error(q.z()-seed.z(), observation_height_band_);          // 0.20
  Eigen::Vector3d dJ_dp(0.0,0.0,2.0*ez);
  if (radius > 1e-8) dJ_dp.head<2>() = 2.0*er*q.head<2>()/radius;   // ← 纯径向梯度
  costp += wei_tracking_*(er*er+ez*ez);
```

`dJ_dp.head<2>()` **只沿径向单位向量 `q/|q|`**，切线方向分量恒为 0。因此：

- `|Δr| ≤ 0.35 m` ⇒ `er = 0`、梯度 = 0（死区）；
- 方位角在任何情况下都不产生代价或梯度。

实测（目标运动段 t=0–78，n=2340）：
```
uav   半径(期望)        方位误差°         与期望位偏差m
uav1  1.56 (1.72)   P50=85.3 P90=107.5   P50=2.19 P90=2.87
uav2  1.28 (1.70)   P50= 9.8 P90= 19.2   P50=0.59 P90=0.92
uav3  1.30 (1.72)   P50=122.3 P90=136.6  P50=2.64 P90=3.12
```
**半径误差小、方位误差巨大**，完全符合"只有径向项"的预言。

### 9. TRACKING_REFERENCE_DRIFT: **PARTIAL**

证据：
- 参考本身没有"把当前位置重置为新参考"的逻辑（`relative_tracking_p_` 是固定编队种子，由 launch 参数给出）。
- 但存在**等效漂移**：由于方位无约束，MINCO 的 rolling seed 每轮从上一条 active 轨迹取起点（"SIDE candidates use the same local time origin"），优化器只需满足径向死区即可保留切向偏移。于是"每步只偏一点、累计很大"的**增量漂移**成立：
  - 方位误差时间走势：uav1 从 t=0 的 −3.5° 单调涨到 t=66 的 +115°；uav3 从 +3.2° 到 −122°；且**长时间平台不回落**（uav1 稳定在 +60~115°、uav3 稳定在 −85~−136°）。

**判定为放大器**（根因仍是 8）。

### 10. J_VIS_BLOCKED_STATE_GRADIENT_FAILURE: **PARTIAL / 非因果**

证据：
- `[directional-visibility-cost]` 在偏离期间持续被调用（drone1 7879 次），说明连续 J_vis 一直在算。
- 但 J_vis 是**连续小量代价**（软权重、梯度局部），其职责是微调轨迹形状；它没有、也不应该有"恢复方位角 2~3 m"的权限（这与上一轮已确立的"软可见性不得获得 topology 权威"一致）。
- 因此 J_vis 在遮挡后梯度弱 **不是** 根因，最多是"无法替代缺失的方位权威"。

---

## 量化指标

```
CONFLICT_FREE_BUT_OFF_NOMINAL_COUNT:
  uav1 = 1755 采样 / 14 段       uav2 = 2285 采样 / 3 段       uav3 = 2239 采样 / 5 段
  （判据复用系统已有 observation_radius_band=0.35 m，未新设阈值；
    来源分布 NOMINAL 占 ~90%）

OFF_NOMINAL_DURATION_P50 / P95 / MAX (s):
  uav1  4.03 /  9.70 /  9.70
  uav2  2.47 / 72.13 / 72.13
  uav3 15.06 / 31.80 / 31.80

CURRENT_LOS_BLOCKED_WITHOUT_RECOVERY_AUTHORITY_COUNT:
  uav1 = 5 段（3.99 s）   uav2 = 1 段（1.13 s）   uav3 = 2 段（2.09 s）

STALE_SIDE_PROVENANCE_COUNT:            0（未观察到）
STALE_LOS_PLANE_PERSIST_COUNT:          0（LOS_PLANE_LOST=0，未堆积）
OLD_SIDE_FUTURE_ACTIVATION_COUNT:       0（未观察到）
NOMINAL_USING_SIDE_SEED_COUNT:          0（未发现注入路径）
JOINT_REFERENCE_IS_SIDE_SEED_COUNT:     0（Joint 参考从未存在）
HIGH_ELASTIC_SLACK_WITHOUT_CONFLICT_COUNT: NOT_MEASURED（弹性松弛为放大器，未单独计数）
```

---

## 第一处语义失效点

```
FIRST_BLUE_BOX_SEMANTIC_FAILURE:
  traj_opt/src/poly_traj_optimizer.cpp :: trackingGradCostP() 合围分支
  dJ_dp.head<2>() = 2.0*er*q.head<2>()/radius;
  条件：encirclement_tracking_active_ == true
  失效：切线（方位）方向梯度恒为 0，且 |Δr|≤0.35 m 时径向梯度也为 0
  → 无任何恢复"编队方位角"的力

FIRST_RED_BOX_SEMANTIC_FAILURE:
  plan_manage/src/planner_manager.cpp :: 候选生成/选择（SIDE 与 LOS 观察面路径）
  条件：LOS 实际被挡，但恢复只在 conflict 周期内一次性生成
  失效：没有"持续恢复权威"、也没有把已偏离的方位纳入恢复目标
  → 遮挡结束后停留在错误方位，不回到可恢复 LOS 的位置

FIRST_YELLOW_BOX_COMMON_FAILURE:
  两条链在同一时刻合并：
  (a) 上面 trackingGradCostP 无方位项（每架各自成立）
  (b) topology_coordinator_core.cpp:521 CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE
      判据 current_available_end + 1e-6 < evaluation_start + evaluation_horizon
      H_EVAL = team_optimizer_params_.horizon = 1.5 s（头文件缺省，编译期常量）
      而单机后继已验证覆盖实测 P50=0.728 s、MAX=0.788 s、100% < 1.5 s
  → Joint 结构性永不成立 → 团队方位修正通道整体不可用
```

### 关于 (b) 的补充证据

本机单机 bundle 与覆盖：
```
[topology-bundle] T_EVAL_START=1789720018.816  H_EVAL=1.500000
[rviz-authority]  drone=0 traj=1215 start=...167636633 validated_end=...933024645   → 0.765 s
[rviz-authority]  drone=0 traj=1216 start=...956364155 validated_end=...687914610   → 0.732 s
[topology-bundle] by drone: {0: 251, 2: 128}   ← drone 1 从未发布 bundle
```
`team_optimizer_params_.horizon{1.5}` 与"单机已验证覆盖 ~0.73 s"之间是**结构性矛盾**：协调器要求 1.5 s 的编队级预测契约，而任何单机候选最多只能提供约 0.79 s。因此 `NO_CONSTRUCTIBLE_TUPLE / CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE` 不是偶发，而是必然。

---

## 已排除项

| 假设 | 结论 | 依据 |
|---|---|---|
| SIDE 权威未退出 | 排除 | 候选逐周期重建；无冲突时切换迟滞分支不进入 |
| NOMINAL 被 SIDE 形状污染 | 排除（但 NOMINAL 自身语义不全，见 8） | 未发现 SIDE guide 注入路径 |
| Joint 参考锁定到 SIDE seed | 排除 | Joint 参考从未存在（可用率 0/1627） |
| 旧 scheduled SIDE 迟到激活 | 未证实 | 轨迹 ID 单调，无回退 |
| 旧 LOS 平面滞留 | 排除 | LOS_PLANE_LOST=0，未堆积 |
| J_vis 遮挡后梯度失效 | 非因果 | 软可见性本就无 topology 权威 |

---

## 推荐修复顺序（本轮不实施）

```
FIX_PRIORITY_1:
  合围 tracking 代价补回方位/切向自由度。
  位置：poly_traj_optimizer.cpp trackingGradCostP() 合围分支。
  方向：在径向带之外，对"相对编队种子的切向误差"给出有界代价
  （例如对 q 与 seed 的夹角误差做 band_error + 梯度），
  或显式让 desired bearing 参与代价。不要改动径向带宽与高度带阈值。

FIX_PRIORITY_2:
  解除 H_EVAL=1.5 s 与单机已验证覆盖 ~0.73 s 的结构性矛盾。
  位置：team_visibility_optimizer.h 的 horizon 缺省 + topology_coordinator_core.cpp:521 判据。
  方向：让 H_eval 由实际可用的公共执行前缀导出（或在覆盖不足时降级为
  较短但仍有效的协调窗口），而不是固定 1.5 s。这是"团队方位修正通道可用"的前提。

FIX_PRIORITY_3:
  LOS 恢复改为持续权威，并把"回到可恢复 LOS 的位置"作为恢复目标，
  而不是只在 conflict 周期内做一次性观察面/侧向候选。
```

## 尚未证实项

- `HIGH_ELASTIC_SLACK_WITHOUT_CONFLICT_COUNT`：弹性角隙在无冲突时的实际量级未单独量化（本轮判定为放大器，未测）。
- 红框期间 `J_vis` 与 raw LOS 的一致性未逐帧对齐（早期大日志被磁盘清理删除，仅尾部保留）；
  红框结论基于可见性 CSV 的逐帧 `static/dynamic_los_clear` 与 CSV 来源分布，证据强度足够但未做 J_vis 数值对齐。
- `post-deadline-recovery` 长循环在目标运动段内的影响未单独量化（该循环的密集日志出现在 t>386 s 的悬停段）。
