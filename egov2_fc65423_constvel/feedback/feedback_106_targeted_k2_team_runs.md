# Feedback 106 — 实测 LOS gate、K2-loss 与 Team 结果

## 目标

在不改 planner 的情况下，验证实测视线附近的动态障碍能否产生 K2-loss，以及 Team PT-SCP 是否能通过并激活修复。所有运行均使用 `scripts/run_alp_full_on.sh --headless`；ROS 日志写入本次 `runs/<RUN_ID>/ros_log`，没有使用 RViz。

## 两轮场景与结果

| 场景 / run | K2 结果 | Team 结果 | 实际结论 |
|---|---|---|---|
| `natural_team_stress_dense_38_targeted_k2_v3.json` / `20260924_015749_761376` | K2=100%；2349 点中 count=2 为 142、count=3 为 2207；最长 K2-loss=0 | contract 4 出现 1 次 `TEAM_SCP_PT_PASS`；`team_solution_id=146` 为 `ADOPTED`，三机均进入同一 Team 激活 | Team 求解、提交和联合激活成功；但 adoption 的 `INTERVENTION_SLACK=-2.174 s`，比预测 crossing 晚，不能声称它挽回了 K2-loss。该轮 gate10 在预设的 63.5–65.5 s LOS 窗口内没有实际射线相交。 |
| `natural_team_stress_dense_38_targeted_k2_v4.json` / `20260924_021117_775967` | 2349 点中 count=1 为 7；最长 K2-loss `0.230039 s`，无 blackout | 3 次 PT-SCP 均 `TRUE_CONSTRAINT_INFEASIBILITY`；PT PASS、T PASS、ADOPTED 均为 0 | 产生了真实但短暂的 K2-loss，没有得到 Team 修复通过。 |

v3 的 PT 通过记录为 `contract_id=4, drone=2`：`TEAM_SCP_NONLINEAR_MARGIN=0.783136`，模型一致性误差 `0.056391`。随后 `team_solution_id=146` 在 drone 0/1/2 上均有 `EARLY_JOINT_ACTIVATED`。不过从 crossing 到 outcome 的 `L_forecast_to_outcome=3418.723 ms`，adoption 发生在 `t_cross` 之后约 2.17 s，因此这是 solver/提交成功证据，不是及时恢复该窗口可见性的证据。

## K2-loss 的实际遮挡来源

v4 的 7 个 count=1 样本位于 `71.768297–71.965646 s`，可见掩码均为 `100`（仅 UAV1 可见）。该时段 UAV3 的 static LOS 已被静态树遮挡，UAV2 的 dynamic LOS 被已有的 `long_forest_moving_9` 遮挡。按实际轨迹和 `0.08 m` LOS margin 重算，v4 新增的 gate10 在这 7 个样本没有与任何 LOS 射线相交。

因此可以确认 v4 run 发生了 K2-loss，但不能把它表述成“gate10 直接挡住目标”。新增 gate 与 LOS 几何有偏差；本轮记录只能确认直接 blocker 是 mover9，gate 是否通过改变局部轨迹间接促成该相交尚未证明。

## 安全与活性

| 指标 | v3 run | v4 run |
|---|---:|---:|
| 最小静态净空 | 0.336738 m | 0.099154 m |
| 最小移动净空 | 0.403623 m | 0.023112 m |
| MIN_BUDGET_APPLIED | 0 | 83 |
| TERMINAL_HOLD_ENTER | 0 | 1 |
| MOVING_SUCCESSOR_STARVATION | 0 | 3 |
| blackout / 碰撞净空非正 | 0 / 0 | 0 / 0 |

v4 的两个净空仍为正，但移动净空仅约 2.3 cm，并出现 terminal hold 与 successor starvation；因此 v4 只适合作为 K2-loss/失败路径复现，不作为安全和活性通过场景。34 条 `safety_validated=False` 轨迹记录来自 `TERMINAL_HOLD` 段（约 53.67–54.77 s）。

## 进程与内存清理

两次 runner 终态均为 `FINAL_EXIT_CODE=0`、`SIMULATION_REACHED_BOOT_12=YES`、`CLEANUP_STATUS=OK`。每轮登记的 29 个 PID 全部已退出，`CLEANUP_FORCE_KILLED=0`、`CLEANUP_KILLED_FOREIGN_PROCESSES=0`；端口 8081/11361 已释放，没有残留 ALP ROS/Gazebo/RViz 进程。`ROS_LOG_DIR` 位于每次 run 的持久磁盘目录；`/dev/shm` 没有 ALP 日志增长。

运行期间可用内存最低约 13 GiB，结束后约 15–18 GiB；swap 使用量保持约 0.93 GiB，没有出现持续增长或 OOM。v4 结束时磁盘仍有约 8.3 GiB 可用。

## 文件与结论

- v3 场景：[natural_team_stress_dense_38_targeted_k2_v3.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v3.json)，几何筛选：[v3 screen](../runs/natural_team_stress_dense_38_targeted_k2_v3_screen.json)
- v4 场景：[natural_team_stress_dense_38_targeted_k2_v4.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v4.json)，几何筛选：[v4 screen](../runs/natural_team_stress_dense_38_targeted_k2_v4_screen.json)
- v3 完整记录：[run 015749](../runs/20260924_015749_761376/exit_status.txt)、[visibility summary](../runs/20260924_015749_761376/visibility_summary.csv)
- v4 完整记录：[run 021117](../runs/20260924_021117_775967/exit_status.txt)、[visibility summary](../runs/20260924_021117_775967/visibility_summary.csv)

本轮分别得到了一次 Team PT PASS/ADOPTED 和一次真实短时 K2-loss，但两者出现在不同 run；K2-loss run 的 Team SCP 未通过。v4 直接遮挡来自既有 mover9，gate10 的直接 LOS 命中尚未闭环，因此不应把 v4 gate 当作已验证的精准挡线方案。
