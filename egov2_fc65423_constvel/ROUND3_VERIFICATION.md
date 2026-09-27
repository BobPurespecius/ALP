# 第三轮修复与验收记录（pending 阻塞 + 运行连续性）

场景：`long_cylinder_forest.json`，FULL ON（risk-triggered candidates / 10 moving obstacles /
encirclement tracking / cooperative viewpoint / visibility ranking / K-of-N(k=2) /
Early Joint+topology / native RViz），启动前已打印 resolved 参数。
构建：`catkin build traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`
→ All 6 packages succeeded。

## 一、本轮代码修改

### 1. pending 不再是 planning 入口 gate
- `ego_replan_fsm.cpp` `callReboundReplan()`：删除 `if (localActivationPending()) return PENDING;`
  整段；改为只做审计计数（`planning_tick_while_local_pending_count_`）后继续正常规划。
- `planner_manager.cpp` `geometryRecoveryReplanDue()`：`if(!local_geometry_active_ || localActivationPending()) return false;`
  → `if(!local_geometry_active_) return false;`
- 结果：`localActivationPending()` 现在只用于状态/遥测（planning-gate 审计计数），
  不再阻止任何正常 planning tick。

### 2. busy-loop 只在同一个 tick 内解决
- `planning_attempted_this_tick_` 在 `execFSMCallback()` 入口（早退分支之前）复位；
- `REPLAN_TRAJ` / `GEN_NEW_TRAJ` / `SEQUENTIAL_START` 三个会真正调用 planner 的分支：
  本 tick 已尝试过则 `++same_tick_duplicate_replan_suppressed_count_` 并直接结束本 tick；
  否则置位后才调用 planner；
- `REPLAN_TRAJ` 失败时回到 `REPLAN_TRAJ` 但受同 tick 保护，重试留给下一个 timer tick。

### 3. 新增逐机遥测
- `[replan-cadence-per-uav]`：REPLAN_TOTAL_COUNT / P50 / P95 / MAX / LT_10MS / LT_5MS
- `[planning-gate-audit]`：PLANNING_TICK_WHILE_LOCAL_PENDING_COUNT /
  PLANNING_TICK_WHILE_JOINT_PENDING_COUNT / SAME_TICK_DUPLICATE_REPLAN_SUPPRESSED_COUNT
- `traj_server.cpp` `[motion-continuity]`：MOVING_DURATION / LOW_SPEED_EPISODE_* /
  TRAJECTORY_ACTIVATION_GAP_* / MAX_SUCCESSOR_GAP（低速阈值 0.10 m/s，
  代码库中没有可复用的 stall 阈值，已写成具名常量并注明只用于遥测）
- `[first-stop-and-go]`：第一条低速事件的快照

## 二、验收数据（sim_run_round4.log，单次真实 FULL-ON，239.7 s）

| 指标 | drone0 | drone1 | drone2 |
|---|---:|---:|---:|
| REPLAN_TOTAL_COUNT | 1465 | 2471 | 698 |
| REPLAN_INTERVAL_P50 (ms) | 16.1 | 17.0 | 18.2 |
| REPLAN_INTERVAL_P95 (ms) | 114.9 | 119.3 | 122.3 |
| REPLAN_INTERVAL_MAX (ms) | 914.8 | 881.0 | 899.7 |
| REPLAN_INTERVAL_LT_10MS_COUNT | 0 | 1 | 0 |
| PLANNING_TICK_WHILE_LOCAL_PENDING_COUNT | 9 | 40 | 6 |
| PLANNING_TICK_WHILE_JOINT_PENDING_COUNT | 10 | 4 | 11 |
| SAME_TICK_DUPLICATE_REPLAN_SUPPRESSED_COUNT | 0 | 0 | 0 |

- 未再出现 <10 ms 的连续 planning burst（修复前 drone0：330 次中 206 次 <10 ms）。
- pending 期间仍在规划 36 次（`[planning-gate] ... action=PLAN_ANYWAY`），证明 gate 已消除。
- MAX 间隔出现在起飞阶段 t+5.2~6.1 s，窗口内日志显示这是"等下一个 replan 触发条件"，
  不是被打断（该窗口内 trajectory 覆盖 1.22~1.50 s 仍在有效期内）。
- 三架 UAV 全程都在产生新 candidate/successor，没有出现单机长期停摆。

其他：`MIXED_TARGET_TIMEBASE_COUNT=0`、`MIXED_DYNAMIC_RISK_TIMEBASE_COUNT=0`、
`ROS_NOW_INSIDE_DYNAMIC_RISK_COUNT=0`、`SIDE_BOTH_FAILED_COUNT=1116` =
`SIDE_BOTH_FAILED_FALLBACK_NOMINAL_COUNT=1116`、`LOS_PLANE_CREATED_COUNT=273`、
`LOS_PLANE_LOST_COUNT=0`、`LOS_PLANE_VIOLATION_COUNT=204`、A* repair 未触发、
`EXECUTED_COLLISION=0`、`SWARM_VIOLATION=0`、无进程崩溃。

Joint：`TEAM_PENDING_START=98`、`JOINT_TIMEOUT=150`、`NO_ADMISSIBLE_JOINT_TAIL=22`、
`TRANSACTION_ACTIVATED=0`（本轮只统计，不修改）。

## 三、残留的第一条真实问题：t+41.3 ~ t+51.4 s drone0 连续 11 次 TERMINAL_HOLD

15 次 `TERMINAL_HOLD_ENTER` 中 11 次集中在 drone0、t+41.35~51.39 s（约每 1.7 s 一次），
其余 4 次在 t+22.6~28.3 s（drone0/drone2），t+83.7 s 有 1 次 drone1。此后 190 s 无 hold。

该窗口内的真实链（每个 tick 重复出现，间隔约 15~20 ms）：

```
VALIDATED_MOVING_COVERAGE_REMAINING drone=0 trajectory_id=95 t_validated_end=...+0.337
execution-reserve action=SOLVE_ALLOWED
future-activation activation_earliest=now+0.100
recovery-local-batch ... guide_available=0
risk-candidate drone=0 triggered=1 obs=6 nominal=4.679 plus=FAILED minus=4.680
team-visibility drone_id=0 ... atleast_k=0.000000 all3=0.000000 none=0.000000 current_uav_visibility=0.000000
（无 planner-traj-commit / LOCAL_DIRECT_COMMIT）
→ 覆盖到期 → TERMINAL_HOLD_ENTER
```

关键事实：
1. planner 一直在跑（不是 PLANNING_NOT_RUNNING）；
2. `pending=0`，joint 不是原因（不是 JOINT_INTERFERENCE）；
3. 每个 tick 都有 NOMINAL/SIDE 候选，但没有产生任何 commit；
4. 同一时刻 `team-visibility` 的可见性全为 0（观测者不可用），选边退化为盲选；
5. validated end 在循环中始终只领先 now 约 0.34 s 且不再增长。

分类：`SUCCESSOR_NOT_INSTALLED`（候选生成正常，但每轮都没有安装 successor），
可疑放大因素是 `team-visibility` 全零导致的选择退化。按本轮约束，
**没有继续修改**，留作下一轮第一优先级。

## 四、环境问题（非代码）

- `~/.ros` 在本环境是只读挂载，rosmaster 无法写 `~/.ros/log`；runner 已改为
  工作区内 `ROS_HOME`/`ROS_LOG_DIR`。
- rosmaster 必须与仿真在同一个后台作业内启动，否则会被沙箱 `--die-with-parent` 连带杀死。
- 日志文件不会再跨 run 追加污染（每轮独立文件名）。
