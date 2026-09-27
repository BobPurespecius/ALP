# 本轮（fix4）报告：ON 状态下低速自锁 / 走走停停修复

场景：`long_cylinder_forest.json`（FULL ON，encirclement + visibility 全开）
仿真产物：`sim_final.log`、`vis_final_trajectory.csv`、`vis_final_summary.csv`

---

## 一、关键指标

```
SOFT_VISIBILITY_DIRECTLY_TRIGGERED_SIDE:
  修复前 YES
  修复后 NO（机制层面已切断：软 J_vis 不再升级为 LOS witness）

SOFT_VIS_ONLY_TRIGGERED_SIDE_COUNT: 14   （占本轮 1256 次提交的 1.1%；
                                          均为 NOMINAL 求解失败时的安全供给兜底）
SIDE_WITHOUT_DESCRIPTOR_NOMINAL_FALLBACK_COUNT: 1141

BODY_DESCRIPTOR_SIDE_TRIGGER_COUNT: 5
LOS_DESCRIPTOR_SIDE_TRIGGER_COUNT: 0

UNNECESSARY_LARGE_SIDE_MANEUVER:
  修复前 YES
  修复后 NO

FINAL_NOMINAL_DEV_P50: 0.022 m      （修复前 0.100 m）
FINAL_NOMINAL_DEV_P90: 1.026 m      （修复前 0.906 m）
SIDE_DEV_GT_1M_COUNT: 3             （修复前 137）
SIDE 候选求解调用次数: 7            （修复前 1499，下降 99.5%）

ROLLING_TIME_ALLOCATION_FIXED: YES
ROLLING_REQUIRED_SPEED_P10/P50/P90 (盘长>1 m 的松时间分配):
  0.875 / 1.237 / 1.302 m/s，比值 <0.60 的条目 0/878
  （修复前典型病灶：0.70 m / 1.37 s = 0.51 m/s）

PREFIX_PROGRESS_COST_IMPLEMENTED: YES（已实现并验证；缺省关闭，原因见第四节）
PVA_HANDOFF_CONTINUITY_PRESERVED: YES
  |v_new(0) - v_old(t_handoff)| P50 = 0.000000，P90 = 0.026 m/s，n=1182

PREFIX_SPEED_RECOVERY_COUNT: 不适用（进度代价缺省关闭；开启时的实测见第四节）
PREFIX_SPEED_DECREASE_COUNT: 不适用

UAV0_TIME_BELOW_0_30: 0.0000   UAV0_LOW_SPEED_MAX_DURATION: 0.00 s（0 段）
UAV1_TIME_BELOW_0_30: 0.0181   UAV1_LOW_SPEED_MAX_DURATION: 0.30 s
UAV2_TIME_BELOW_0_30: 0.0120   UAV2_LOW_SPEED_MAX_DURATION: 0.33 s

UAV0_CONTINUOUS_MOTION: YES
UAV1_CONTINUOUS_MOTION: YES
UAV2_CONTINUOUS_MOTION: YES

USER_VISIBLE_STOP_AND_GO: IMPROVED（低速段大幅缩短，但仍有零星 0.3 s 级瞬间减速）
```

### 运动质量对照（目标运动段，30 Hz 实测）

| 指标 | 修复前 v3 | 本轮 final | 变化 |
|---|---:|---:|---|
| uav1 P10 速度 | 0.497 | **0.875** | +76% |
| uav2 P10 速度 | 0.450 | **0.675** | +50% |
| uav3 P10 速度 | 0.424 | **0.690** | +63% |
| uav1 时间占比 <0.60 m/s | 15.2% | **0.7%** | −95% |
| uav2 时间占比 <0.60 m/s | 20.5% | **5.5%** | −73% |
| uav3 时间占比 <0.60 m/s | 16.5% | **5.4%** | −67% |
| uav2 时间占比 <0.30 m/s | 4.1% | **1.8%** | −56% |
| uav3 时间占比 <0.30 m/s | 5.9% | **1.2%** | −80% |
| uav2 最长低速段 | 2.37 s | **0.97 s** | −59% |
| uav3 最长低速段 | 2.60 s | **1.03 s** | −60% |

### 其余遗留检查

```
TERMINAL_HOLD_COUNT: 247（全部集中在目标停下的终点悬停段，按要求不计入过程口径）
END_BEFORE_NEXT_COUNT: 247（同上，与 TERMINAL_HOLD 一一对应）
JOINT_SUCCESS_COUNT / JOINT_TIMEOUT_COUNT: 本场景未触发 Joint 事务
LOS_PLANE_CREATED_COUNT / LOST / VIOLATION: 0 / 0 / 0
MIXED_TARGET_TIMEBASE_COUNT: 0
MIXED_DYNAMIC_RISK_TIMEBASE_COUNT: 0
COLLISION_COUNT: 0（472 处 "COLLISION" 均为 SIDE 初始化的
                 SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION 拒绝原因，非真实碰撞）
SWARM_VIOLATION_COUNT: 0
进程崩溃: 0

MEAN_VISIBLE: uav1 1.000 / uav2 0.983 / uav3 0.990
K2_RATIO: 0.9729（无 K2 丢失，executed_longest_k2_loss_s = 0.000）
K2_LONGEST_LOSS: 0.000 s
BLACKOUT_RATIO: 0.0000
ALL3_RATIO: 0.9729
```

---

## 二、修改内容（production code）

### 修复 1：软 visibility 代价不再获得 SIDE topology 权限

`plan_manage/src/planner_manager.cpp`，原 6314-6339 行。

原实现把"方向可见性代价 > 1e-9"的梯度极大值点做一次 `queryStaticLosClearance`，
就把查询结果升级成 `static_visibility_witness_valid`，从而生成
`CONFLICT_LOS_OCCLUSION` 描述符并触发完整 LEFT/RIGHT observation plane 与 SIDE
搜索。日志实测 `directional_static=0.02836`、甚至 FOV-only 的
`directional_fov=2475.55`（而 `static_los=0 dynamic_los=0 fov=0`）都能走通这条路。

现在该分支只做诊断与计数（`[soft-visibility-continuous-only]`），
`static_visibility_witness_valid` 只可能来自 `raw_static_los_blocked` 这一真实
遮挡真值；`spatial_visibility_trigger` 因此只反映真实 BODY 或 LOS 描述符。
`J_vis` 本身完全保留，继续进入 MINCO 连续优化。

改动要点（新增代码，未删除任何代价项）：
- 软代价分支只记 `soft_vis_nonzero_count_`，不再写 witness；
- 删除由此产生的 `CONFLICT_LOS_OCCLUSION` 与 `visibility_support_active` 路径依赖；
- 新增审计计数器 `soft_vis_nonzero_count_` / `soft_vis_only_triggered_side_count_`
  / `body_descriptor_side_trigger_count_` / `los_descriptor_side_trigger_count_`
  / `side_without_descriptor_nominal_fallback_count_`，随
  `[visibility-topology-audit]` 每 5 s 与结束时输出。

效果：`identity` 层面，SIDE 候选求解次数 1499 → 7，`SIDE_DEV_GT_1M_COUNT`
137 → 3。

### 修复 2：大偏移 SIDE 必须对应真实 topology 收益

候选选择逻辑本身（`consider_candidate` / LOS-only 分支 / `preferred_side` 平局
分支）不做重写：它已经要求"LOS-only 候选不得顶掉有效 NOMINAL"，且
`preferred_side` 必须来自真实 blocker 的横向逃逸方向。本轮的实际修法是让这些
判据**只对真实描述符生效**（修复 1），于是"全局 visibility 略好 → 大横移"的
路径自然消失：修复后候选求解只剩 7 次，其中 >1 m 偏移 3 次，且均在同轮
NOMINAL 失败的安全供给路径上。

未有新增 hard gate、dwell、confirm、switch lock。

### 修复 3：rolling trajectory 时间分配收紧

`plan_manage/src/planner_manager.cpp`，`buildFreshMovingInitializer()`。

原实现：
```cpp
const double v_ref = std::max(0.25, 0.55 * v_limit);
double total_time = std::max(1.0, 1.20 * distance / v_ref) * requested_scale;
if (pp_.max_acc_ > 0.0)
  total_time = std::max(total_time, 2.0*sqrt(std::max(distance,1.0e-3)/a_limit));
```
`max(1.0, ...)` 的距离下限把时间分配与真实长度解耦：0.70 m 的局部目标被强行摊到
1.37 s（≈0.51 m/s），initializer 天然生成慢速前段，随后被 rolling 的 P/V/A 继承
链锁死。

改为复用系统既有速度标尺：
```cpp
const double nominal_tracking_speed = local_target_vel.norm();
double v_ref = (isfinite(nominal_tracking_speed) && nominal_tracking_speed > 1e-3)
                   ? nominal_tracking_speed : 0.55 * v_limit;
v_ref = std::max(0.25 * v_limit, std::min(v_ref, max_vel));
double total_time = std::max(0.10, 1.05 * distance / v_ref) * requested_scale;
```
加速度只保留 `2*sqrt(distance/a_limit)` 作为"动力学不可行再拉伸"的下界。
新增 `[fresh-time-allocation]` 遥测（distance / v_ref / total_time /
length_over_duration）。

效果（盘长 > 1 m）：`LENGTH_OVER_DURATION` P10/P50/P90 =
0.875 / 1.237 / 1.302 m/s，**比值 <0.60 的松时间分配条目 0/878**。
未引入任何硬速度下限，未触碰 handoff 起点头状态。

### 修复 4：rolling 前缀前向进度软目标（已实现，缺省关闭）

`traj_opt/src/poly_traj_optimizer.cpp` + `poly_traj_optimizer.h`：
新增 `setPrefixProgressCost(enabled, span, v_ref, weight, risk_attenuation,
forward_direction)` 与 `[prefix-progress]` 遥测。代价形式为前缀末端单点 Huber：

```
deficit  = max(0, v_ref - v(t_prefix)·t_hat)
J_prefix = w_eff · h(deficit / v_ref)
grad_v   = -2·w_eff·min(deficit/v_ref, δ)·t_hat / v_ref
```

- 前缀长度 `span = local_activation_margin + execution_margin = 0.10 + 0.15
  = 0.25 s`（系统既有 lifecycle 量，非拍脑袋常量）；
- 参考速度取本批次 `local_target_vel`，退化时用目标状态，再退化用
  `0.55*max_vel`，始终以 `max_vel` 为上界；
- 风险衰减：`prefixProgressRiskAttenuation()` 复用系统已有 BODY 风险证据
  （`hard_collision` 或 `triggered` → 0.15），真实避障照常减速；
- 不改变 handoff P/V/A，不引入任何硬速度约束。

**为什么缺省关闭（诚实记录）**：

1. 逐样本铺满前缀 + 时间自由度梯度：NOMINAL 求解全线崩溃
   （`solve_success=0` 28539 次，仅 drone0 起飞，`return_code=-1005`）；
2. 只留位置自由度、改用前缀末端单点后求解恢复正常
   （1795 commits，三架都飞），但**低速尾反而变差**：
   | | prefix ON | prefix OFF |
   |---|---:|---:|
   | uav1 P10 | 0.556 | **0.684** |
   | uav2 P10 | 0.634 | **0.722** |
   | uav3 P10 | 0.665 | **0.728** |
   | uav3 <0.30 占比 | 1.45% | **0.00%** |
   原因是参考速度被解析成 1.65 m/s（车辆速度上限标尺）而目标只有 0.928 m/s，
   代价把中位速度推到 1.02~1.05 m/s（高于目标），并未消除低速瞬间。
   把参考速度改用 `local_target_vel` 后三架仍正常飞行，但低速尾仍未改善，
   故保持缺省 `prefix_progress_weight=0.0`。
   （消融口径：同图同目标，`prefix_progress_weight:=0` 与 `:=30` 各一次真实仿真。）

未引入 brake / stop / hold / velocity floor。

### 未改动项（确认）

`p_new(0)=p_old(t_handoff)`、`v_new(0)=v_old(t_handoff)`、
`a_new(0)=a_old(t_handoff)` 完全保留（实测 |dv| P50 = 0.000000）；
未改物理安全阈值、机体尺寸、动态障碍硬净空、N/L/R 语义、LOS topology、
Joint 架构、TeamVisibilityOptimizer 架构、控制器。

---

## 三、四个问题的回答

1. **小 visibility signal 是否还会错误触发大 SIDE？**
   不会。软 `J_vis` 已与 topology 权限解耦，SIDE 候选求解 1499 → 7 次。

2. **SIDE 是否仍产生大量无收益的大横移？**
   不再。`SIDE_DEV_GT_1M_COUNT` 137 → 3，且剩余 3 次都在 NOMINAL 求解失败的
   安全供给路径上。

3. **rolling duration 是否仍把短距离轨迹拉得过慢？**
   不再。盘长 > 1 m 的时间分配比 P10 0.875 m/s，<0.60 m/s 的条目 0/878；
   原来的 0.70 m / 1.37 s 病灶消失。

4. **低速 handoff 后，新轨迹是否会在实际执行前缀内开始恢复速度？**
   P/V/A 交接连续性完好（|dv| P50 = 0），起点头状态不被抬高；由于本轮把
   慢速来源（无谓 SIDE + 松时间分配）从源头去掉，实际执行段已不再出现
   "指令速度持续下滑"的链条：全程最低速段降到 0.30~0.33 s。

5. **三架 UAV 是否真正从走走停停恢复成持续运动？**
   是。三架全程持续规划（615/342/298 次提交，0 崩溃），uav1 完全无 <0.30 m/s
   样本，uav2/uav3 的 <0.30 占比从 4.1%/5.9% 降到 1.8%/1.2%，最长低速段
   从 2.37/2.60 s 降到 0.97/1.03 s。

---

## 四、遗留

`SOFT_VIS_ONLY_TRIGGERED_SIDE_COUNT = 14`：这 14 次不是可见性触发的，而是
`execution_reserve_limited_` 下"NOMINAL 求解失败（lbfgs -1005）→ SIDE 安全供给
兜底"的既有语义（同轮 `SIDE_WITHOUT_DESCRIPTOR_NOMINAL_FALLBACK_COUNT=1141`）。
二者共用 `static_visibility_support` 这一软信号作为标记，因此该计数器会在
NOMINAL 失败且同时存在软可见性代价时一并计数。若要彻底分离，需要把
"安全供给兜底"与"可见性授权"拆成两个独立入口——属于独立模块改动，本轮不改。

**FIRST_REMAINING_LOW_SPEED_ROOT_CAUSE**：剩余最低速事件（uav2 0.30 s、
uav3 0.33 s）起点均为 `lbfgs-error return_code=-1005` 导致的 NOMINAL 求解失败
→ 同轮改用 SIDE 安全供给候选 → 该候选前段速度低于 NOMINAL。即根因已从
"无谓 SIDE 大机动 + 松时间分配"转移到"NOMINAL SCP 求解偶发不收敛"，属其他独立
模块，按要求只记录、不扩改。
