# ALPv2 基准状态报告（以最后一次仿真为基准）

**基准 run**：`runs/20260921_170305_331181`
**性质**：FULL 消融 + RViz，`FINAL_EXIT_CODE=0`，`SIMULATION_REACHED_BOOT_12=YES`
**二进制**：`traj_server` 构建于 16:07:43、`ego_planner_node` 构建于 16:24:09（含 Feedback093 全部生命周期修改）
**时长**：wall 239.8 s；场景 235.5 s（目标运动 0–80 s，之后静止于 (36.00, 0.00)）

> **基准有效性前提**：本次无 terminal hold 属**非确定性**结果——同一二进制、同一配置的上一次 RViz run（16:48）出现一次永不恢复的 planner 进程级挂起。因此本报告描述的是"当前代码在一次良性运行中的表现"，不是稳定保证。

---

## 1. 任务完成度（全 235.5 s，7062 样本/机）

| 机 | 末距目标 | 均距 | 中位距 | 最大距 | ≤2.5 m 占比 | 速度中位 |
|---|---|---|---|---|---|---|
| uav1 | 2.46 m | 1.82 m | 1.76 m | 3.39 m | 99.1% | 0.84 m/s |
| uav2 | 1.69 m | 1.63 m | 1.69 m | 2.52 m | 99.9% | 0.84 m/s |
| uav3 | 1.54 m | 1.82 m | 1.76 m | 3.18 m | 99.3% | 0.86 m/s |

三机全程保持对目标的合围跟踪，无掉队、无停滞。

---

## 2. 可见性（记录窗口仅前 76.5 s / 2293 样本 —— 占任务时长 32%）

| 指标 | 值 |
|---|---|
| K2（≥2 机同时可见） | **100.00%** |
| ALL3（3 机同时可见） | **93.68%** |
| BLACKOUT（0 机可见） | **0.00%** |
| 可见数分布 | 3 机 93.7% / 2 机 6.3% / 1 机 0% / 0 机 0% |
| `executed_longest_k2_loss_s` | **0.000** |
| `executed_longest_blackout_s` | **0.000** |

逐机（权威值取自 `visibility_summary.csv`，已用样本级重算交叉核对一致）：

| 机 | visibility_ratio | static_los 失败 | dynamic_los 失败 | fov 失效 | range 失效 | 朝向失配率 | 最长连续不可见 |
|---|---|---|---|---|---|---|---|
| uav1 | 98.08% | **0** | 23 (1.00%) | 21 (0.92%) | **0** | 0.92% | 0.764 s @70.2–71.0 |
| uav2 | **100.00%** | **0** | **0** | **0** | **0** | **0.00%** | 0 |
| uav3 | 95.60% | **0** | 28 (1.22%) | 73 (3.18%) | **0** | 1.31% | 1.834 s @17.9–19.7 |

团队朝向失配率 `fov_mismatch_rate_team = 2.22%`。

摄像头配置：水平 FOV 85.0°、垂直 54.54°、最大距离 8.0 m、遮挡余量 0.08 m。

---

## 3. 安全（全 7062 样本 × 3 机）

| 指标 | 值 |
|---|---|
| `static_clearance_m ≤ 0`（静态接触） | **0** |
| `moving_clearance_m ≤ 0`（动态接触） | **0** |
| `safety_validated = True` | **7062/7062（100%）每机** |

---

## 4. 生命周期契约（Feedback093 改动验证）

| 计数器 | 值 | 判定 |
|---|---|---|
| `ACTIVE_PVA_MISMATCH` | **0** | ✅（Feedback091 为 229） |
| `EXPECTED_PREDECESSOR_INVALIDATED` | 0 | ✅ 路径就位未触发 |
| `EXPECTED_PREDECESSOR_NOT_AVAILABLE` | 0 | ✅ |
| `ACTIVATION_HANDOFF_DISCONTINUITY` | 0 | ✅ |
| `DUPLICATE_ACTIVE_IDENTITY` / `EXPIRED_PAYLOAD` | 0 / 0 | ✅ |
| `EXECUTOR_CONFIRMED_ACTIVATION_BUT_LOCAL_COPY_MISSING` | **0** | ✅ |
| `TERMINAL_HOLD_ENTER` / `EXIT` | **0 / 0** | ✅ |
| `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` | **0** | ✅ |
| `MOVING_SUCCESSOR_STARVATION` | **0** | ✅ |
| `traj-server-heartbeat-stale` | **0** | ✅ |
| `LOCAL_UPDATE_DEFERRED` | **0** | ✅ 阻塞路径已删除 |
| `PARTIAL_TEAM_ACTIVATION` | **0** | ✅ 原子性保持 |
| `planner-authority-switch` | 1200（全部 `EXECUTOR_CONFIRMED_ACTIVATION`） | ✅ 权威切换只在 activation |
| `execution-activated` | 1193 | ✅ |
| `team-speculative-abort` | 9 | ✅ 全部带 `local_authoritative_predecessor_unchanged=1` |
| `TEAM_COMMIT_REVOKE_REQUEST` | **0** | ✅ 无提交后撤销 |

Team 事务：`TRANSACTION_COMMIT 140` / `TRANSACTION_ACTIVATED 137` / `traj-server-team-cancel 9`；
Team speculative 状态分布：`COMMITTED_FUTURE 140` / `ACTIVE 134`。
Team 求解：`REALIZED_TEAM_VALIDATED 251`、`NO_LEXICOGRAPHIC_TEAM_IMPROVEMENT 134`、`TEAM_REFERENCE_NOOP 483`。

### ⚠ 未被行使的修复

`MIN_BUDGET_APPLIED = 0`、`current-state-restart event=ENTER = 0` → **本次运行从未进入 current-state-restart**，因此 Feedback093 §26 的"restart 最低预算"修复在本 run 中**未被执行、也未获验证**。

---

## 5. 性能遥测

| 指标 | drone_0 | drone_1 | drone_2 |
|---|---|---|---|
| 重规划总次数 | 1797 | 1735 | 1802 |
| 间隔 P50 | 12.2 ms | 12.2 ms | 12.3 ms |
| 间隔 P95 | 630.2 ms | 650.0 ms | 632.7 ms |
| 间隔 MAX | **2022.7 ms** | **2024.3 ms** | **2023.0 ms** |

规划时延（`[execution-planning-budget]`，n=5410）：P50 **1.4 ms**、P95 8.1 ms、mean 2.8 ms、max 173.6 ms。
`traj-server-scheduled` 共 1112 次。

> P50 = 12 ms 说明多数 tick 是快速重试；MAX ≈ 2.02 s（三机几乎相同）说明存在**单次超过 2 秒的规划停顿**，值得单独审查。

---

## 6. 残留缺陷（本轮未修，本次运行仍然活跃）

### 6.1 侧向候选判据（"有 LOS 却选错方向"的根因）

`candidate_is_accepted` 只看 `clearance_gain ≥ 0`（硬间隙），不含可见性；且 `planner_manager.cpp:9988` 在 NOMINAL 为 `ABSOLUTE_SAFE` 时**无条件短路选择 NOMINAL**。

| 指标 | 值 |
|---|---|
| SIDE_PLUS 被拒 / 接受 | **272** / 196 |
| SIDE_MINUS 被拒 / 接受 | **295** / 38 |
| **被拒但 MINCO 实际成功（`final_candidate_clearance > 0`）** | **379** |
| `side-both-failed-fallback` | **338** |
| `SIDE_BOTH_FAILED` | 626 |
| `NO_STATIC_FEASIBLE_SIDE` | 87 |
| `candidate-preinit-failure` | 151 |
| `LOS_PLANE_SCP_FAILED` | 36 |

### 6.2 求解器与候选失败

| 指标 | 值 |
|---|---|
| `[lbfgs-error]` / `Solver error` | **448** / 448 |
| `[nominal-candidate] solve_success=0` | 332（记录到的全部为失败） |
| `dynamics_valid=0` | **400** |
| `candidate-final-status final_success=0` | 188（占记录到候选的 23%） |
| `EXECUTION_DEADLINE` | 474 |
| `OBJECTIVE_EVALUATION_CANCELLED` | 309 |

### 6.3 执行源质量

全 235.5 s 的 `trajectory_source` 分布显示，约 **47% 的执行时间落在 `FEASIBLE_FALLBACK`**（回退轨迹）而非 nominal 优化解：

| 机 | NOMINAL | FEASIBLE_FALLBACK | TEAM_REALIZED_LOCAL | SIDE_PLUS/MINUS |
|---|---|---|---|---|
| uav1 | 3413 (48.3%) | 3342 (47.3%) | 163 (2.3%) | 144 (2.0%) |
| uav2 | 3462 (49.0%) | 3234 (45.8%) | 156 (2.2%) | 206 (2.9%) |
| uav3 | 3390 (48.0%) | 3342 (47.3%) | 162 (2.3%) | 168 (2.4%) |

回退原因计数：`NO_FEASIBLE_INITIALIZER 188`、`EXECUTION_DEADLINE 158`。

### 6.4 偏航控制

`visibility_summary.csv` 报告的最大指令偏航**角加速度**异常高：

| 机 | max 偏航率 | max 偏航角加速度 |
|---|---|---|
| uav1 | 3.607 rad/s | **15438 rad/s²** |
| uav2 | 2.992 rad/s | **7067 rad/s²** |
| uav3 | 3.607 rad/s | **14622 rad/s²** |

历史 run（`20260920_005724_484367`）亦出现 17743 / 22566 / 14376 rad/s²，属**长期存在、尚未审查**的问题。

### 6.5 未修复的挂起机制

planner 进程级挂起（全部 ROS 定时器停止、进程存活、无 core dump）的代码路径**未做任何改动**。本次未触发，但机制仍在同一二进制中。

---

## 7. 与上一次（冻结）run 的对比

| 指标 | 冻结 run `164802_321591` | 基准 run `170305_331181` |
|---|---|---|
| `TERMINAL_HOLD_ENTER` / `EXIT` | 1 / **0（永不退出）** | **0 / 0** |
| `traj-server-heartbeat-stale` | 1 | 0 |
| uav3 末距目标 | **18.0 m**（卡死 174 s） | **1.54 m** |
| `ACTIVE_PVA_MISMATCH` | 0 | 0 |
| `side-both-failed-fallback` | 536 | 338 |
| `NO_STATIC_FEASIBLE_SIDE` | 191 | 87 |
| SIDE 被拒 / 接受 | 849 / 191 | 567 / 234 |
| 静态接触 | 0 | 0 |

**两次使用同一二进制、同一 `roslaunch_argv` / `resolved_params` / `ablation_manifest`（diff 全空）。差异来自非确定性，不来自任何修改。**

---

## 8. 数据覆盖与可信度边界

1. **可见性数据只覆盖前 76.5 s**（2293 样本，占 235.5 s 任务的 32%）。K2=100%、ALL3=93.7%、BLACKOUT=0% 仅对这 76.5 s 成立；后 159 s（含目标静止后的合围段）无可见性记录。
2. **restart 预算修复未被行使**（`MIN_BUDGET_APPLIED=0`）。
3. **本次无 hold 不构成稳定基线**：同代码上一次运行出现永久卡死。
4. `[nominal-candidate]` 只观察到 `solve_success=0` 的记录（332 条），无法据此断言 nominal 求解成功率——需确认成功路径是否另有日志点。
5. 本报告全部数字均直接来自该 run 的 `roslaunch_stdout.log`、`visibility.csv`、`visibility_trajectory.csv`、`visibility_summary.csv`、`ablation_counters.txt`、`exit_status.txt`；未修改任何源码与配置。
