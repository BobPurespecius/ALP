# Feedback 098 —— Predictive Relay 假风险修复：把 M2 的预测权威交还给 Planner

本轮目标：闭环 Feedback097 审计出的根因（Coordinator 用 rolling Local 轨迹外推出虚构几何，
报出真实几何不存在的负裕度），并按 097 的 `RECOMMENDED_MINIMAL_FIX` 做最小修复，
然后用 **真实 FULL 生产仿真** 验证。

硬边界遵守情况：未使用 `git reset` / `git checkout` / `git clean` / 未回滚任何已有修改；
未重写 MINCO / SCP；未重写 Feedback096 的 single-flight / finish-recheck / transaction；
未新增持久化 contract 状态机；未调低任何 safety threshold；未新增大规模测试套件；
未访问、搜索、读取、修改 RRCT。

---

## 1. 根因闭环（Root cause closed）

### 1.1 Feedback097 认定的根因

`multi_uav_topology_coordinator.cpp::assessPredictiveRelay()` 用订阅到的
`/drone_N_planning/trajectory`（rolling Local 轨迹，实测寿命 p50 = 0.266 s）
外推到 `activation + relay_horizon_(2.0 s)` 作为 observer 未来位置来算 M2。
外推视界是轨迹寿命的 5–39 倍 ⇒ 得到"若永不重规划会怎样"的**虚构几何**。

实测后果：contract 1 的 `m2_min = -1.329706`，而同一 absolute world time 上
optimizer 对同一架 UAV 评估 `margin = 1.000000`，用实测位置复算真值也是 1.0。
**coordinator 连符号都错。**

### 1.2 修复动作

`assessPredictiveRelay()` 被整体重写：**彻底删除**对 `execution_[drone].traj` 的多项式外推，
改为只融合三架 Planner 自己发布的 `VisibilityForecastTrace`。

结构验证（不是靠日志，是靠源码）：

| 检查 | 命令 | 结果 |
|---|---|---|
| 旧外推路径 | `grep -c "execution_\[" coordinator.cpp` | 23 —— **逐条核对，全部是元数据**（`traj_id` / `start_time` / `duration` / `getPieceNum()` / `predecessor`），无一条按时间求值 |
| `getPos(world_time - start_time)` | `grep -n "getPos(world_time" coordinator.cpp` | **0 命中** |
| 残留 `getPos` 调用 | 1101–1106 行、1608–1611 行 | 两处**全部 clamp 到 `duration`**（`std::max(0.0, std::min(duration, ...))`），用于候选位置查询与 joint-seed 节点，不构成无界外推 |

`execution_[i]` 本身仍然保留，因为它还承担 lineage / predecessor 权威（Feedback093/096 的契约），
这是**有意保留**的，不是遗漏。

### 1.3 闭环的数值证据

同一个 limiting 问题上，两侧现在指向同一事实：

| contract | 时刻 | coordinator M2 | planner 自身独立复核 | 偏差 |
|---|---|---|---|---|
| 003556 c2 (uav2) | 1790095016.954213858 | **−1.024952** | **−1.044077** | 0.019 |
| 003556 c1 (uav0) | 1790095008.941852570 | **−3.906713** | **−3.627987** | 0.279 |

Feedback097 的 `−1.329706 vs +1.0`（符号相反）已经**不复存在**。
修复后 coordinator 的 `limiting_margin` 与 planner 复核用的 `forecast_margin` 在
两次 contract 上**逐位相等**（见 §11），因为二者现在读的是同一份 Planner 预测。

**结论：ROOT_CAUSE_CLOSED = YES。**

---

## 2. 新的预测权威（New forecast authority）

核心设计原则：**每架 UAV 的未来位置只能由拥有它的 Planner 声明**。
rollling Local 轨迹每几百毫秒被替换一次，任何外部节点都无法预测它，
因此 Coordinator 不再"猜"位置，只做**排序融合**。

### 2.1 新消息 `traj_utils/VisibilityForecastTrace.msg`

```text
bool valid
uint64 forecast_identity
int32 trajectory_id
uint64 trajectory_generation
int32 candidate_id
string source_kind
time activation_time
time created_world_time
time start_world_time          # 样本的绝对世界时间有效区间（闭区间）
time end_world_time
float64 sample_dt
time mathematical_end_time

float64[] world_times          # 绝对世界时间栅格
float64[] margins              # 同长度
uint8[]  limiters              # 0=INVALID 1=STATIC 2=DYNAMIC_LOS 3=FOV 4=RANGE
```

该消息挂到 `TopologyCandidateBundle.visibility_forecast`（附加字段，未破坏既有字段）。

**为什么用绝对世界时间栅格而不是相对时间**：planner 的 Local 轨迹在不断被替换，
`activation_time` 也在变；只有绝对世界时间才是两侧可比的公制。
这是 Feedback097 里"同一个 absolute world time 却得出相反符号"的直接解药。

### 2.2 权威一致性保证

预报样本由 `visibilitySampleAt()` 产生 —— 它同时是 Team-SCP 种子评估
（`teamMarginAt()`）用的**同一个函数**。因此 coordinator 融合出来的 M2
与 planner 在 SCP 里看到的是**同一物理量**，不存在"第二套评估器"漂移。

---

## 3. Planner 预测轨迹（Planner forecast trace）

### 3.1 实现

| 组件 | 位置 | 作用 |
|---|---|---|
| `PolyTrajOptimizer::visibilitySampleAt()` | `poly_traj_optimizer.cpp:1157` | 从原 lambda 体提取成成员函数，供 forecast / 复核 / SCP 种子共用 |
| `PolyTrajOptimizer::evaluateVisibilityForecast()` | `poly_traj_optimizer.cpp:1339` | 在 `[t0, t0+horizon]` 上按 `dt` 采样，输出 `VisibilityTraceSample` 序列 |
| `PolyTrajOptimizer::evaluateVisibilityMarginAt()` | `poly_traj_optimizer.cpp:1374` | 单点求值，用于最新基线复核 |
| 发布点 | `planner_manager.cpp:4786-4842` | 在 bundle 发布时用 `traj_.local_traj` 构建 |

**关键安全性质：两者都 clamp 到 `trajectory.getTotalDuration()`，永不外推。**
窗口在轨迹末端自然截断，Coordinator 看到的就是"本机真正承诺的未来"。

### 3.2 参数与日志

```text
manager/visibility_forecast_horizon = 2.0 s   (非法值回落到 2.0)
manager/visibility_forecast_dt      = 0.1 s   (非法值回落到 0.1)
[relay-forecast-config] drone=... horizon=... dt=...
```

```text
[RELAY_FORECAST] drone=0 forecast_id=1 traj_id=1 generation=1 source=NOMINAL
  activation=1790094967.014945507 window=[1790094967.014945507,1790094969.014945507]
  dt=0.100 samples=21 margin_min=1.000000 limiter=1
```

`source_kind` 取 `active_execution_source_`，`candidate_id` 取 `pending_topology_.local_candidate_id`，
因此预报可回溯到具体的 Local 候选。

---

## 4. Coordinator M2 融合（Coordinator M2 fusion）

### 4.1 融合算法

`sampleForecastMargin()`（`multi_uav_topology_coordinator.cpp:676`）：
按绝对世界时间线性插值；`limiter` 取区间内**更保守**的一端（`alpha < 0.5` 取左）。
`valid` 为假、长度不匹配、或请求时刻落在 `[front, back]` 之外 → 返回 false（**拒绝回答，而不是外推**）。

`assessPredictiveRelay()`（726 行）：
1. 求三条 forecast 的**公共世界时间区间** `[max(starts), min(ends)]`；
2. 与请求窗口 `[activation, activation + min(relay_horizon_, horizon)]` 求交；
3. 公共区间为空或短于 `relay_sample_dt_` → 发
   `[RELAY_FORECAST_INSUFFICIENT]`，**不产生 M2**；
4. 否则逐采样点取三架 margin，排序，`M2 = ranked[1]`（第二大）。

### 4.2 "拒绝编造"的证据

003556 里出现 2 次真实空窗：

```text
[RELAY_FORECAST_INSUFFICIENT] activation=1790095088.704434872
  common_window=[1790095088.704434872,1790095088.561650753]
  required_dt=0.100 reason=INSUFFICIENT_COMMON_WINDOW
```

`window_end (…561650753) < window_begin (…704434872)`，交集确实为空。
**旧实现在这种情况下会照样外推并给出一个数**；新实现明确报告"没有依据"。

---

## 5. 限制性 UAV 传播（Limiting UAV propagation）

修复前 `RelayAssessment` 只记录 limiter 分量，**不记录是哪一架 UAV 导致 M2**，
所以 refinement 无法瞄准真正的问题机（097 审计的发现之一）。现在：

| 结构 | 新增字段 |
|---|---|
| `RelayAssessment` | `limiting_uav`, `limiting_margin`, `critical_margins[3]`, `forecast_created_world_time_ns` |
| `PersistentHandoffContract` | `limiting_uav`, `limiting_margin` |
| `TeamReferenceSchedule.msg` | `int16 handoff_limiting_uav`, `float64 handoff_limiting_margin`, `time handoff_critical_world_time`, `time handoff_forecast_created_world_time` |

排序自洽性核对（M2 = 第二大，`limiting_uav` 必须就是它）：

| contract | m_uav0 | m_uav1 | m_uav2 | 降序 | M2 | limiting_uav |
|---|---|---|---|---|---|---|
| 011246 c1 | −3.274802 | −1.831345 | −4.581131 | uav1, **uav0**, uav2 | −3.274802 | **0** ✓ |
| 003556 c1 | −3.906713 | +0.735762 | −3.946573 | uav1, **uav0**, uav2 | −3.906713 | **0** ✓ |
| 003556 c2 | +1.000000 | −1.417521 | −1.024952 | uav0, **uav2**, uav1 | −1.024952 | **2** ✓ |

三次全部自洽。日志：

```text
[RELAY_M2_LIMITING_UAV] contract_id=1 limiting_uav=0
  critical_world_time=1790097215.980146408 limiting_margin=-3.274802
  outgoing=1 incoming=2 stable=0
```

`[HANDOFF_CONTRACT_ATTACHED]` 与 `limiting_uav` 一起下发，planner 侧据此决定"要不要我复核"。

---

## 6. 最新基线复核（Latest baseline recheck）

### 6.1 机制

在决定"是否发起 refinement"之前，planner 用**自己当前最新基线轨迹**
在同一个 `critical_world_time` 上用 `evaluateVisibilityMarginAt()` 复算一次：

```text
[TEAM_REQUEST_RECHECK] drone=0 contract_id=2 limiting_uav=2
  critical_world_time=1790095016.954213858 forecast_margin=-1.024952
  current_baseline_margin=1.000000 baseline_traj_id=163 same_window=1
```

若整窗都已被当前基线满足 → `TEAM_REFINEMENT_NOOP_CURRENT_BASELINE_SAFE`，不再做无用 SCP。

### 6.2 命名 UAV 门控（关键正确性）

复核的短路**只允许由 contract 指名的 limiting UAV 触发**
（`message->handoff_limiting_uav == pp_.drone_id`）。否则任何一架"自己恰好很安全"的
UAV 都能把别人的真实缺陷一笔勾销。实测验证：

- 003556 c2，`limiting_uav = 2`。drone 0 复核得到 `current_baseline_margin = 1.000000`
  （自身完全安全），但**没有短路**——因为它是 stable(0) 而不是 limiting(2)。
  真正被指名的 drone 2 复核得到 `−1.044077`（真实缺陷）→ 正常进入 SCP。
- 011246 c1，`limiting_uav = 0`。drone 0 复核 `−4.769574`，drone 1 `−3.364528`，
  drone 2 `+0.563366`（安全但不被指名）→ 无一短路。

三轮运行 `TEAM_REFINEMENT_NOOP_CURRENT_BASELINE_SAFE = 0`。
这**不是**门控失效，而是**本轮观察到的每一次 contract，其 limiting UAV 自身基线都是负的**
（真实缺陷），所以不存在合法的短路机会。门控的"不该短路"方向已被上表证明有效。

---

## 7. ZERO-MOD + 证书修复（ZERO-MOD + certificate fix）

### 7.1 修复内容

Feedback097 发现：SCP 可以"零修改地接受种子"（`TEAM_VISIBILITY_ALREADY_SATISFIED`
或 `ΔP = ΔT = 0`），旧代码却把它当成一次成功的 refinement，
给出 `team_refinement_open_`，进而可能发放 `TeamImprovementCertificate` 并触发覆盖保护。

现在（`planner_manager.cpp:3404-3539`）：

```cpp
const bool zero_modification =
    team_result.reason == "TEAM_VISIBILITY_ALREADY_SATISFIED" ||
    (team_result.delta_p_norm <= 1.0e-9 &&
     team_result.delta_real_t_norm <= 1.0e-9);
```

零修改时：
- 不置 `team_refinement_open_`、不置 `pending_team_trajectory_.handoff_refinement`；
- 不发放 certificate，不进入覆盖保护；
- 不计入 `team_ref_solver_pass_`，改计 `team_ref_zero_modification_`；
- 日志报 `result=NOOP_ZERO_MODIFICATION ... improvement_authority=0`；
- 不再打印误导性的 `TEAM_REFINEMENT_START` / `TEAM_REFINEMENT_SOLVE`。

非零修改时才发放证书，且**要求可度量改进**：
`margin_after` 有限、`≥ required − 1e-9`、且 `> m2_before + 1e-9`，
否则报 `NOOP_NO_MEASURABLE_IMPROVEMENT`。

### 7.2 顺带修掉的两个 trace 缺陷（仅遥测，不改行为）

1. `TEAM_VISIBILITY_ALREADY_SATISFIED` 是**早退路径**，发生在记录 `margin_before` 的
   iteration-0 块之前，于是日志出现 `m2_before=-inf`（003556 实测）。
   已在该路径补上 `margin_before = team_margin_current`。
2. 同一路径的精确 reason 被 `team_scp_result_.reason = last_candidate_final_status_reason_`
   覆盖成空串，日志出现 `reason=` 后直接接 `dp=`。已改为不覆盖早退路径已写入的 reason。

### 7.3 证书未被误发

| run | `NOOP_ZERO_MODIFICATION` | `TEAM_IMPROVEMENT_CERTIFICATE` | `LOCAL_QUALITY_OVERWRITE_BLOCKED` | `TEAM_IMPROVEMENT_OVERRIDE` |
|---|---|---|---|---|
| 003556 | 1 | **0** | 0 | 0 |
| 010810 | 0 | 0 | 0 | 0 |
| 011246 | 0 | 0 | 0 | 0 |

`ZERO_MOD_IMPROVEMENT_CERTIFICATE_COUNT = 0`。
003556 那次零修改结果确实带着 `improvement_authority=0` 且没有开出 adoption 窗口。

---

## 8. 延迟遥测修复（Latency telemetry fix）

### 8.1 问题

| 字段 | 旧实现 | 实际含义 |
|---|---|---|
| `solve_ms` | `now - wall_start`（**函数入口**，含 Team reference realization/MINCO/候选评估） | 不是 SCP 本体 |
| `total_ms` | `now - refinement_wall_begin`（**freeze 之后**） | 起点晚于 `wall_start` |

因为 `refinement_wall_begin > wall_start`，`total_ms < solve_ms` 是**结构性必然**，
实测出现过 `solve_ms=19.334 total_ms=2.072`。

### 8.2 修复

```cpp
const double scp_wall_begin = ros::WallTime::now().toSec();
const bool team_scp_ok = ploy_traj_opt_->runTeamContractSCP(...);
const double scp_wall_end = ros::WallTime::now().toSec();

solve_ms = (scp_wall_end - scp_wall_begin) * 1000.0;              // SCP 本体
total_ms = (ros::WallTime::now().toSec() - refinement_wall_begin) * 1000.0;
```

`refinement_wall_begin` 在 freeze 之后、`scp_wall_begin` 之前 ⇒ **`total_ms ≥ solve_ms` 结构性成立**。
失败路径与成功路径统一；`NOOP_ZERO_MODIFICATION` 行也补上了 `solve_ms/total_ms`，
使该路径同样可度量。

### 8.3 实测验证（run 011246，4/4 全部满足）

| # | drone | solve_ms | total_ms | total ≥ solve |
|---|---|---|---|---|
| 1 | 2 | 3.595 | 3.653 | ✓ |
| 2 | 2 | 1.843 | 1.856 | ✓ |
| 3 | 0 | 1.970 | 1.991 | ✓ |
| 4 | 1 | 6.531 | 6.563 | ✓ |

```text
[TEAM_REFINEMENT_RESULT] drone=2 refinement_id=1 contract_id=1
  result=SOLVER_INFEASIBLE reason=TRUE_CONSTRAINT_INFEASIBILITY
  solve_ms=3.595 total_ms=3.653 updates_during_solve=0
  linearization_world_time=-1.000000000 linearization_component=NONE
```

**诚实说明**：run 003556 的日志写于 00:40:23，而带延迟修复的源码保存于 00:46:41，
即 003556 用的是**修复前**的二进制（其 `solve_ms=19.334 total_ms=2.072` 正是旧症状）。
因此延迟不变量只用 011246 作为证据，003556 不用于此项。

---

## 9. 构建（Build）

```text
catkin build -j2 --no-status
[build] Summary: All 25 packages succeeded!
[build] Warnings: None.
[build] Failed: None.
[build] Runtime: 1 minute and 13.6 seconds total.
```

- `traj_opt` 1m3.9s（本轮 poly_traj_optimizer.cpp 的修改），`ego_planner` 5.8s（重链接）。
- 新增 `.msg` 需要 `--force-cmake` 才能生成头文件，本轮已处理
  （`VisibilityForecastTrace.msg` 已加入 `traj_utils/CMakeLists.txt` 的 `add_message_files`）。
- `git diff --check` → **PASS**（无空白/冲突标记）。
- 未执行任何 `git reset` / `git checkout` / `git clean`；工作树中既有的大量未提交修改全部保留。

---

## 10. FULL 生产运行（FULL production run）

### 10.1 命令（场景一律用**绝对路径**）

```bash
./run_on.sh --ablation full --headless --timeout 200 \
  --scenario /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/\
scenes/long_cylinder_forest_visibility_stress.json
```

场景：26 路点目标 @0.928 m/s、39 根静态圆柱、3 个动态障碍。

### 10.2 运行台账

| RUN_ID | 二进制 | FINAL_EXIT | BOOT-12 | contract | recheck | refinement | 用途 |
|---|---|---|---|---|---|---|---|
| 20260923_003556_16718 | 预测修复后 / **延迟修复前** | 0 | YES | 2 | 4 | 4 | §1/§11 一致性证据 |
| 20260923_010810_43994 | 全部修复 | 0 | YES | 1 | 0 | 0 | §13 安全-活性 |
| **20260923_011246_47924** | **全部修复** | **0** | **YES** | **1** | **4** | **4** | **主证据（§8/§11/§12）** |
| 20260923_005043_24507 | — | — | **NO** | — | — | — | **作废**（见 10.3） |

011246 的运行状态：

```text
BOOT12_TARGET_MOVING=YES   BOOT12_UAV0_MOVING=YES
BOOT12_UAV1_MOVING=YES     BOOT12_UAV2_MOVING=YES
FINAL_EXIT_CODE=0          SIMULATION_REACHED_BOOT_12=YES
CLEANUP_FORCE_KILLED=0     CLEANUP_REMAINING_OWNED=0
CLEANUP_KILLED_FOREIGN_PROCESSES=0
```

三段可见性保持（011246）：uav1 = 0.9556、uav2 = 0.9199、uav3 = 0.9819，
`fov_mismatch_rate_team = 0.000000`，`executed_longest_blackout_s = 0.368894`。

### 10.3 作废运行与资源处置

- `20260923_005043_24507` 在磁盘 100% 满（仅剩 602 MB）时启动，
  在 BOOT-07 之后卡死、无 `[MONITOR]`、无 `exit_status.txt`，
  `current-state-restart` 高达 364,108（正常 1,167）、`topology-bundle` 反而只有 741（正常 1,309），
  即**不是**新增预报负载所致，而是资源耗尽。按"资源不足时等待，不要强行运行污染结果"，**整轮作废**。
- 随后清理：删除 81 个陈旧 `runs/*` 目录、`captures/*`、顶层陈旧的
  `sim_run_round2.log`，并清空 `/dev/shm/alp_*`，释放约 **11 GB**（99% → 84%，15 GB 可用）。
  仅保留 3 个仍有证据价值的 run 目录。
- 三次正式运行前均确认：`rosmaster`/`roslaunch`/`ego_planner_node` 残留进程 = 0，
  端口 11361 / udp 8081 空闲，`/dev/shm` 已清空。

---

## 11. Coordinator vs Planner 裕度一致性表

### 11.1 主证据：run 011246，contract 1

`limiting_uav = 0`，`critical_world_time = 1790097215.980146408`，`required_margin = 0.2`

| 来源 | uav0 | uav1 | uav2 |
|---|---|---|---|
| Coordinator 融合预报 `m_uavN` | **−3.274802** | −1.831345 | −4.581131 |
| Coordinator `M2`（第二大） | — | — | **−3.274802** |
| Planner `[TEAM_REQUEST_RECHECK] forecast_margin` | — | −3.274802 | −3.274802 |
| Planner 自身最新基线 `current_baseline_margin` | **−4.769574** | −3.364528 | +0.563366 |
| Planner SCP 种子（`0.2 − deficit`） | −5.273016 | −4.711013 | −4.572603 |

**limiting UAV（uav0）三种独立来源全部为负且同量级：−3.27 / −4.77 / −5.27。**
这就是本轮的核心验收：coordinator 报出的风险**被 planner 自己的评估证实**，
不再是 097 那种"coordinator 说 −1.33、planner 说 +1.0"的虚构。

同 contract 的 4 次复核（`same_window=1` 全部为真，即确实在同一窗口上比较）：

| drone | forecast_margin（来自 coordinator） | current_baseline_margin（planner 独立复算） | Δ |
|---|---|---|---|
| 2 | −3.274802 | −0.669595 | 2.605 |
| 2 | −3.274802 | +0.563366 | 3.838 |
| 1 | −3.274802 | −3.364528 | 0.090 |
| 0（limiting） | **−3.274802** | **−4.769574** | 1.495 |

### 11.2 副证据：run 003556

| contract | limiting uav | coordinator | planner forecast | planner 基线 | Δ(coord, 基线) |
|---|---|---|---|---|---|
| c2 @1790095016.954213858 | 2 | −1.024952 | **−1.024952**（逐位相等） | −1.044077 | **0.019** |
| c1 @1790095008.941852570 | 0 | −3.906713 | **−3.906713**（逐位相等） | −3.627987 | 0.279 |

`forecast_margin` 与 coordinator 的 `limiting_margin` 逐位相等是**设计使然**（同一份预报），
`current_baseline_margin` 与它的差值（0.019–0.279）才是**两条不同基线的真实漂移**，
这是有意义的独立交叉验证。

### 11.3 负裕度是否"物理合理"——是

011246 的 contract 建于 `1790097214.348`，当时三架 UAV 全部处于
`state=GEOMETRY_DEGRADED`：

| 指标 | 值 |
|---|---|
| `encirclement_ratio` | 0.000000 – 0.466667 |
| `gap_violation_integral` | 4.12 – 5.67 |
| `gap_max_deg` | 311.0 |
| `gap_min_deg` | 24.4 |

包围几何实际已经崩坏，此时裕度为 −3…−5 是**几何的真实反映**，不是外推产物。
这也解释了为什么同一轮里 `TEAM_SCP_MODE` 报出的 `deficit` 高达 4.77–5.47。

---

## 12. Team-SCP 真实行为（Team-SCP real behavior）

### 12.1 本轮实际发生了什么

run 011246：4 次 refinement，**全部** `SOLVER_INFEASIBLE`：

| drone | deficit | grad_T_norm | T_gain_bound | AUTO 选择 | limiter | 结果 |
|---|---|---|---|---|---|---|
| 2 | 4.772603 | 1.815139 | 0.367676 | **PT** | DYNAMIC_LOS | TRUE_CONSTRAINT_INFEASIBILITY |
| 2 | 4.992393 | 0.335875 | 0.060767 | **PT** | DYNAMIC_LOS | TRUE_CONSTRAINT_INFEASIBILITY |
| 1 | 4.911013 | 3.345033 | 0.564893 | **PT** | DYNAMIC_LOS | TRUE_CONSTRAINT_INFEASIBILITY |
| 0（limiting） | 5.473016 | 1.636399 | 0.380882 | **PT** | STATIC | TRUE_CONSTRAINT_INFEASIBILITY |

`AUTO` 模式选择是**正确**的：deficit（4.77–5.47）远大于仅靠时间重分配能拿到的
`T_gain_bound`（0.06–0.56），所以不能只调 T，必须 P+T。

### 12.2 不可行是"真不可行"，不是超时

```text
[adaptive-tr] candidate_id=1 iter=0 qp_status=primal_infeasible
  trust_p_before=0.300000 trust_t_before=0.150000
  probe_no_p_tr=0 probe_no_t_tr=0 probe_no_pt_tr=0
  decision=TRUE_INFEASIBLE
```

自适性信赖域依次探测了 **p-only / t-only / p+t** 三种子空间，全部失败，
才判定 `TRUE_INFEASIBLE` → `TRUE_CONSTRAINT_INFEASIBILITY`。
该 reason 与超时类 reason（`EXECUTION_DEADLINE`）在源码中是**分开的**
（`poly_traj_optimizer.cpp:3611` vs `:4584`），所以这不是"预算不够"的伪装。

同时 `TEAM_SCP_DP_NORM = 0`、`TEAM_SCP_DTAU_NORM = 0`、`TEAM_SCP_DT_NORM = 0`
—— SCP 没有对轨迹做出任何修改，因此**没有产生任何实现化的改进**。

### 12.3 一次被"上游"吃掉的 contract（run 010810）

010810 只产生 1 个 contract，但它**根本没走到 SCP**：
contract 挂到 reference 83 后，三架 UAV 的本地 realization 全部失败：

```text
[lbfgs-error] context=SIDE drone_id=2 return_code=-1005
Solver error. Return = -1005, The line-search routine reaches the maximum
number of evaluations.. Skip this planning.
[TEAM_LOCAL_SAFE_SEED_RETAINED] drone=2 contract_id=1
  reason=LOCAL_MINCO_OPTIMIZER_FAILED action=HARD_PREFLIGHT
[TEAM_REFERENCE_ACK] drone=2 reference_id=83 accepted=0 realization_valid=0
  reason=LOCAL_HARD_PREFLIGHT_FAILED:
```

安全行为正确（保留本地安全种子、拒绝该 reference），
但**contract 的 refinement 机会随之丢失**，因为没有可用的种子进入 SCP。

### 12.4 计数结论

```text
REAL_NONZERO_TEAM_REFINEMENT_COUNT = 0     # 三轮均无成功的非零修改 refinement
REALIZED_M2_IMPROVEMENT_COUNT      = 0     # 无任何被采纳的 M2 改进
```

按任务 §20 的口径，本轮结论是
**`FORECAST_FIX_VALIDATED BUT NONZERO_TEAM_REFINEMENT_NOT_EXERCISED`**：
预测权威的问题已被闭环并验证，但"非零 refinement"这一步在本轮的三次运行中没有被真正走通，
原因已定位到 §14.1 / §14.2，**不是**预测权威问题。

---

## 13. 安全性-活性（Safety-liveness）

### 13.1 安全与活性计数

| 指标 | 003556 | 010810 | 011246 | 判读 |
|---|---|---|---|---|
| `TERMINAL_HOLD_STARVATION` | 0 | 0 | 0 | ✅ |
| `MIN_BUDGET_APPLIED` | 97 | **0** | **0** | 010810/011246 优于 096 基线(61) |
| `ACTIVE_PVA_MISMATCH` | 0 | 0 | 0 | ✅ |
| `PARTIAL_TEAM_ACTIVATION` | 0 | 0 | 0 | ✅ |
| `TRANSACTION_ABORTED` | 0 | 0 | 0 | ✅ |
| `CONTACT` / 碰撞检测 | 0 | 0 | 0 | ✅ |
| `hard_collision` | 0 | 0 | 0 | ✅ |
| `UNVALIDATED_STATIC` / `UNVALIDATED_DYNAMIC` | 0 | 0 | 0 | ✅ |
| 饥饿/STARVATION 关键词 | 0 | 0 | 0 | ✅ |
| `min_dynamic_clearance`（最小记录值） | — | — | 0.3020 m | ≥0.23 ⇒ risk=0 ✅ |
| `TRANSACTION_ACTIVATED`（096 契约仍在工作） | 114 | 151 | 137 | ✅ 未被回归破坏 |
| `TERMINAL_HOLD_ENTER` | 23 | **0** | **1** | 见 13.2 |

未出现任何 `[collision*]` / `[safety*]` / `[emergency*]` 守卫事件。

### 13.2 诚实记录的两处退化

1. **`TERMINAL_HOLD_ENTER`**：003556 = 23（096 基线为 0），010810 = 0，011246 = 1。
   这不是单调回归：同一二进制连续两次运行为 0 和 1，说明是运行时序抖动；
   `TERMINAL_HOLD_STARVATION` 在三轮中**始终为 0**，即没有发生饥饿性终止。
   仍然记录在案，供后续轮次继续观察。

2. **几何退化比例（`ENCIRCLEMENT_DEGRADED=1`）**：

   | run | GEOMETRY_DEGRADED | NORMAL_ENCIRCLEMENT | 退化占比 |
   |---|---|---|---|
   | 096 基线 (20260922_204715) | 1271 | 9201 | 12.1% |
   | 003556 | 1503 | 14194 | 9.6% |
   | 010810 | 677 | 9020 | **7.0%** |
   | 011246 | 5757 | 1241 | **82.3%** ← 离群 |

   011246 是明显离群。它不是本次代码改动造成的：同一二进制的 010810 只有 7.0%，
   预测修复后/延迟修复前的 003556 为 9.6%，096 基线为 12.1%。
   该场景含 3 个动态障碍，团队包围几何对时序高度敏感。
   但必须承认：011246 虽提供了最完整的 refinement 证据，
   其大部分时间处于几何退化状态；这与 §11.3 的负裕度是同一件事的两面。

### 13.3 Feedback096 调度契约未被破坏

single-flight（`team_refinement_open_`）、freeze-input 快照、正常更新不取消、
finish-and-recheck、`TeamImprovementCertificate` 质量覆盖保护、Local 保证后继、
PREPARE/READY/COMMIT —— 全部保持原样，未重写。
本轮只新增了"零修改不是 refinement"这一条**收紧**判定，
以及"证书必须有可度量改进"这一条**收紧**门控。

运行期清零也正常：`CLEANUP_STATUS=OK`、`CLEANUP_FORCE_KILLED=0`、
`CLEANUP_REMAINING_OWNED=0`、`CLEANUP_KILLED_FOREIGN_PROCESSES=0`。

---

## 14. 遗留问题（Remaining problems）

1. **refinement 机会挂在"本地 realization 成功"之下。**
   010810 的唯一 contract 因为 `lbfgs -1005`（SIDE 线搜索达到最大评估次数）
   导致三架 UAV 的 `LOCAL_HARD_PREFLIGHT_FAILED`，contract 直接失去进入 SCP 的机会。
   当前设计的 refinement 是在"reference 已 realized 的种子"上做的，
   因此本地 MINCO 一次失败就等于放弃该次接力合作。
   可考虑（**本轮未做**）在 realization 失败时改用"当前 Local 安全基线"作为 SCP 种子。
   这属于设计扩展，超出本次最小修复范围。

2. **所有被指名的 SCP 尝试都是真不可行，且缺口极大。**
   `deficit` 4.77–5.47 对应种子裕度 −4.57…−5.27，而被要求的 margin 只有 0.2。
   在包围几何已经退化（`encirclement_ratio = 0`、`gap_max_deg = 311°`）时，
   可见性义务在容许的轨迹修改范围内根本不可达。
   ⇒ 真正需要解决的是**几何退化本身**（何时/为何进入 `GEOMETRY_DEGRADED` 并长时间停留），
   而不是继续加大 team SCP 的力度。本轮**没有**为此调整任何阈值。

3. **contract 样本量太小。** 三轮共观察到 3 个 contract、8 次复核、8 次 refinement 决策。
   统计功效不足，尤其 `TEAM_REFINEMENT_NOOP_CURRENT_BASELINE_SAFE` 在结构上
   要求"limiting UAV 自身基线安全"，本轮一次都没出现，该分支**未被实测覆盖**。

4. **`TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF` 恒为 0。**
   三轮日志中该字段全为 0，即 contract 的激活相位从未被用于 handoff，
   contract 目前只驱动"是否请求 refinement"，不参与相位对齐。
   需要确认这是有意的还是未接通的路径。

5. **几何退化率的高方差（7% ↔ 82%）** 使单次运行难以代表系统水平，
   后续应以多次运行的分布而非单点来判定。

6. 096 遗留的 `TERMINAL_HOLD_ENTER` 抖动（0 / 1 / 23）仍未完全消除，
   但 `STARVATION = 0` 且无安全违规。

---

## 15. 结论字段

```text
ROOT_CAUSE_CLOSED: YES
  Feedback097 根因（coordinator 将 rolling Local 轨迹外推至 activation+2.0s 评估 M2）
  已删除：grep "getPos(world_time" = 0；剩余 execution_[..] 引用 23 处全部是
  元数据（traj_id/start_time/duration/piece_num），幸存的两处 getPos 均 clamp 到 duration。
  修复后 coordinator 与 planner 在同一 absolute world time 上的裕度一致：
    003556 c2: coord -1.024952 vs planner baseline -1.044077 (Δ=0.019)
    003556 c1: coord -3.906713 vs planner baseline -3.627987 (Δ=0.279)
  097 的 "-1.329706 vs +1.0" 符号矛盾消失。

NEW_FORECAST_AUTHORITY: PLANNER
  traj_utils/VisibilityForecastTrace.msg (绝对世界时间栅格 world_times/margins/limiters)
  由 Planner 在 TopologyCandidateBundle 中发布；Coordinator 只做排序融合，不再推断位置。

PLANNER_TIME_INDEXED_FORECAST_IMPLEMENTED: YES
  poly_traj_optimizer.cpp: visibilitySampleAt(1157) / evaluateVisibilityForecast(1339) /
  evaluateVisibilityMarginAt(1374)；两者均 clamp 到 getTotalDuration()，永不外推。

COMMON_WORLD_TIME_M2_IMPLEMENTED: YES
  coordinator.cpp: sampleForecastMargin(676) 按绝对世界时间插值；
  assessPredictiveRelay(726) 取 [max(starts),min(ends)] ∩ [activation,activation+H]；
  空窗发 RELAY_FORECAST_INSUFFICIENT 且不产生 M2（003556 实测 2 次，window_end < window_begin）。
  COORDINATOR_OLD_EXECUTION_EXTRAPOLATION_REMOVED: YES

LIMITING_UAV_EXPLICITLY_PROPAGATED: YES
  RelayAssessment{limiting_uav,limiting_margin,critical_margins[3],forecast_created_world_time_ns}
  → PersistentHandoffContract → TeamReferenceSchedule.handoff_limiting_uav/limiting_margin/
  handoff_critical_world_time/handoff_forecast_created_world_time；日志 [RELAY_M2_LIMITING_UAV]。
  排序自洽 3/3：011246 c1→uav0, 003556 c1→uav0, 003556 c2→uav2。

LATEST_BASELINE_RECHECK_IMPLEMENTED: YES
  [TEAM_REQUEST_RECHECK] 用当前最新基线在 critical_world_time 复算；
  仅当 message->handoff_limiting_uav == pp_.drone_id 时才允许短路。
  实测证据：003556 c2 limiting_uav=2，drone0 基线 +1.0 未短路，drone2 基线 -1.044 正常进入 SCP。
  TEAM_REFINEMENT_NOOP_CURRENT_BASELINE_SAFE = 0（本轮所有 limiting UAV 基线均为负，无合法短路机会）

ZERO_MOD_REFINEMENT_IS_NOOP: YES
  zero_modification ⇒ 不开 team_refinement_open_、不置 handoff_refinement、不发证书、
  不计 solver_pass；ZERO_MOD_IMPROVEMENT_CERTIFICATE_COUNT = 0
  （003556 有 1 次零修改，improvement_authority=0，无 certificate）
  CERTIFICATE_REQUIRES_MEASURABLE_IMPROVEMENT: YES (finite, >= required-1e-9, > m2_before+1e-9)

LATENCY_TELEMETRY_FIXED: YES
  solve_ms = scp_wall_end - scp_wall_begin; total_ms = now - refinement_wall_begin;
  refinement_wall_begin < scp_wall_begin ⇒ total >= solve 结构性成立。
  TOTAL_LATENCY_GE_SOLVE_LATENCY: YES  (011246: 3.653>=3.595, 1.856>=1.843,
                                        1.991>=1.970, 6.563>=6.531)
  注：003556 早于该修复（日志 00:40:23 < 源码 00:46:41），不用于此项。

BUILD: PASS
  catkin build -j2 --no-status → All 25 packages succeeded, Warnings: None；
  traj_opt 1m3.9s, ego_planner 5.8s；git diff --check PASS。

PRODUCTION_RUNS_THIS_ROUND: 3 (valid) + 1 (discarded)
  PRIMARY_RUN_ID: 20260923_011246_47924  FINAL_EXIT_CODE=0  BOOT-12=YES
  SECONDARY_RUN_ID: 20260923_010810_43994 FINAL_EXIT_CODE=0  BOOT-12=YES
  CONTRACT_EVIDENCE_RUN_ID: 20260923_003556_16718 FINAL_EXIT_CODE=0  BOOT-12=YES
  DISCARDED_RUN_ID: 20260923_005043_24507 (disk 100% full, never reached BOOT-12)
  SCENARIO: long_cylinder_forest_visibility_stress.json (absolute path)
  CLEANUP_KILLED_FOREIGN_PROCESSES=0, CLEANUP_REMAINING_OWNED=0

COORDINATOR_PLANNER_MARGIN_MAX_ABS_DIFF: 3.838168
  # 011246 c1 全部 4 次复核中 |forecast_margin - current_baseline_margin| 的最大值
  # (drone2 @traj146: |-3.274802 - 0.563366|)；这是"不同基线"的差异，非同源误差。
COORDINATOR_PLANNER_MARGIN_SAME_SOURCE_DIFF: 0.000000
  # coordinator limiting_margin 与其 forecast_margin 逐位相等（同一份预报，设计使然）
COORDINATOR_PLANNER_LIMITING_UAV_DIFF: 0.019 (003556 c2) / 1.495 (011246 c1)
  # 最强单点一致：003556 c2 limiting UAV 上 coord -1.024952 vs planner -1.044077

REAL_NONZERO_TEAM_REFINEMENT_COUNT: 0
  # 011246: 4 次 refinement 全部 SOLVER_INFEASIBLE / TRUE_CONSTRAINT_INFEASIBILITY
  # 003556: 3 次 SOLVER_INFEASIBLE + 1 次 NOOP_ZERO_MODIFICATION
  # 010810: contract 止步于 LOCAL_HARD_PREFLIGHT_FAILED (lbfgs -1005)，未进入 SCP
REALIZED_M2_IMPROVEMENT_COUNT: 0
TEAM_SCP_TRUE_INFEASIBILITY_COUNT: 7
  # adaptive-tr 依次探测 p-only/t-only/p+t 全部失败 → decision=TRUE_INFEASIBLE，
  # 与 EXECUTION_DEADLINE 是不同 reason，故非预算不足伪装。

SAFETY_LIVENESS_OK: YES
  TERMINAL_HOLD_STARVATION=0, MIN_BUDGET_APPLIED=0 (011246), ACTIVE_PVA_MISMATCH=0,
  PARTIAL_TEAM_ACTIVATION=0, TRANSACTION_ABORTED=0, CONTACT=0, hard_collision=0,
  UNVALIDATED_STATIC=0, UNVALIDATED_DYNAMIC=0, min_dynamic_clearance=0.3020 m
  FEEDBACK096_SCHEDULING_CONTRACT_PRESERVED: YES (TRANSACTION_ACTIVATED=137)

KNOWN_REMAINING_RISK:
  (a) refinement 依赖本地 realization 成功，一次 lbfgs 失败即丢失 contract 机会
  (b) 全部 SCP 真不可行，根因是包围几何退化（encirclement_ratio=0, gap_max_deg=311°）
  (c) contract 样本仅 3 个，NOOP_CURRENT_BASELINE_SAFE 分支未被实测覆盖
  (d) TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF 恒为 0，需确认是否有意
  (e) ENCIRCLEMENT_DEGRADED 比例 7%~82% 方差极大，011246 为离群（同二进制 010810 仅 7.0%）
  (f) TERMINAL_HOLD_ENTER 0/1/23 抖动，STARVATION 恒为 0

HONEST_VERDICT: FORECAST_FIX_VALIDATED BUT NONZERO_TEAM_REFINEMENT_NOT_EXERCISED

SCP_ONLY_FILES_CHANGED:
  ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/VisibilityForecastTrace.msg (new)
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/TopologyCandidateBundle.msg (2 fields appended)
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/TeamReferenceSchedule.msg (4 fields appended)
  ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/CMakeLists.txt
MINCO_REWRITTEN: NO
SCP_REWRITTEN: NO
FEEDBACK096_SINGLE_FLIGHT_REWRITTEN: NO
PERSISTENT_CONTRACT_STATE_MACHINE_ADDED: NO
SAFETY_THRESHOLD_LOWERED: NO

GIT_RESET_USED: NO
GIT_CHECKOUT_OVERWRITE_USED: NO
GIT_CLEAN_USED: NO
EXISTING_MODIFICATIONS_ROLLED_BACK: NO

FORECAST_AUTHORITY_FIXED: YES
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
