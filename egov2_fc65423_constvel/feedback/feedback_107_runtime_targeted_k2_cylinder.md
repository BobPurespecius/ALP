# Feedback 107 — 实际射线命中的静态圆柱与 K2-loss

## 场景调整

上一版 v5 的实际轨迹证明 id36 挡住了 UAV1 的 21 个目标射线样本；旧 id37 没有命中任何实际可见射线。本轮将 id37 放到晚段 UAV3—目标视线上，保持静态圆柱总数为 38、移动障碍为 11，没有修改 planner。

新场景：[natural_team_stress_dense_38_targeted_k2_v6.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json)。新 id37 的中心为 `(32.841659, 1.067125) m`，半径 `0.28 m`，高度 `3.6 m`。target-route 校验为 26 个 waypoint、0 个相交，最小全局静态目标净空 `0.097619 m`。

基于 v5 实际轨迹的固定轨迹预筛预测 id37 会在 `73.751–74.285 s` 挡住 17 个 UAV3 射线样本，使可见数从 2 降到 1。完整预筛记录：[v6 screen](../runs/natural_team_stress_dense_38_targeted_k2_v6_screen.json)。

## v6 正式运行

Run `20260924_163326_1175833` 使用 FULL、headless；`FINAL_EXIT_CODE=0`、BOOT-12 成功。ROS 日志目录为 `runs/20260924_163326_1175833/ros_log`，runner manifest 标记 `persistent_run_dir(tmpfs_disabled)`。29 个 runner 自有进程均退出，强杀数为 0，也没有杀其他进程。

最终可见性统计为 2348 个样本、78.248 s：UAV1/2/3 可见率 `94.63% / 98.55% / 97.19%`。K2-loss 最长 `0.527694 s`，blackout 为 0。单机可见样本 16 个，发生在 `73.887839–74.382185 s`。

按实际 v6 轨迹重算几何：

| 圆柱 | 实际命中的射线 | 样本数 | 时窗 |
|---|---|---:|---|
| id36 | UAV1—目标 | 21 | `8.148599–8.815716 s` |
| 新 id37 | UAV3—目标 | 23 | `73.648688–74.382185 s` |

id37 命中全部 16 个 K2-loss 样本中的 UAV3 射线；这些样本中 UAV1 可见、UAV2 被 moving gate id10（`targeted_k2_gate_10_v4`）遮挡、UAV3 的 static LOS 被 id37 挡住，因此实际可见数为 1。它不是只在图上靠近路线的圆柱，而是在运行轨迹上直接截中了 LOS。id37 对实际 target 轨迹的最小表面净空为 `0.408475 m`，对 UAV 轨迹为 `0.316780 m`；全轨迹最小静态/移动净空为 `0.316780 / 0.302913 m`，非正净空样本为 0。

## 场景图

[全局布局与 K2-loss 瞬间的实测射线](../runs/natural_team_stress_dense_38_targeted_k2_v6_layout.png)。

## Team 结果与因果边界

Run 中识别到 3 个 Team contract，`TEAM_PT_ATTEMPT_COUNT=128`，有 1 次 PT PASS 和 1 次 ADOPTED；T PASS 为 0。PT PASS/ADOPTED 属于较早的 contract 1，不能归因于晚段 id37 遮挡。晚段 contract 3 与 K2-loss 时间重合，包含 STATIC 与 DYNAMIC_LOS PT 候选，但两者都以 `TRUE_CONSTRAINT_INFEASIBILITY` 失败。因此本轮证明了圆柱直接造成 LOS 遮挡并触发真实 K2-loss，也观察到 Team 优化活动；没有证明 Team 修复成功挽回了这次 K2-loss。

安全与活性计数：`MIN_BUDGET_APPLIED=0`、terminal hold `0`、successor starvation `0`、PVA mismatch `0`、partial activation `0`、unvalidated execution `0`。24,555 条轨迹记录均标记 `safety_validated=true`。

运行证据：[exit status](../runs/20260924_163326_1175833/exit_status.txt)、[process cleanup](../runs/20260924_163326_1175833/process_status.txt)、[visibility summary](../runs/20260924_163326_1175833/visibility_summary.csv)、[manifest](../runs/20260924_163326_1175833/run_manifest.txt)。

```text
TARGETED_ID36_ACTUAL_LOS_HITS: 21
TARGETED_ID37_ACTUAL_LOS_HITS: 23
ID37_HITS_OVERLAP_K2_LOSS_SAMPLES: 16
LONGEST_K2_LOSS_S: 0.527694
BLACKOUT_S: 0
TEAM_PT_PASS / T_PASS / ADOPTED: 1 / 0 / 1
CONTRACT_3_REPAIR_FOR_LATE_K2_EVENT: SOLVER_INFEASIBLE
MIN_STATIC / MOVING_CLEARANCE_M: 0.316780 / 0.302913
FINAL_EXIT_CODE / CLEANUP_STATUS: 0 / OK
