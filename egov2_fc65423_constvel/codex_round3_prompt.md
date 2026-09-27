# 任务：修复 pending successor 阻塞 rolling replanning，并补齐验收遥测

工作区（所有修改都在这里）：`/home/bob/ALP/egov2_fc65423_constvel`

规划器源码根目录：
`/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/`

| 文件 | 相对路径 |
|---|---|
| FSM 实现 | `plan_manage/src/ego_replan_fsm.cpp` |
| FSM 头 | `plan_manage/include/plan_manage/ego_replan_fsm.h` |
| Manager 实现 | `plan_manage/src/planner_manager.cpp` |
| Manager 头 | `plan_manage/include/plan_manage/planner_manager.h` |

**禁止访问 `/home/bob/RRCT`。**
**所有代码注释与提交说明用中文。**

---

## 一、本轮唯一要改的语义：pending 不能阻塞下一轮 planning

### 现状（上一轮引入，已确认是缺陷）

`plan_manage/src/ego_replan_fsm.cpp` 约 845-865 行，`callReboundReplan()` 开头：

```cpp
    if (planner_manager_->localActivationPending())
    {
      ++pending_replan_suppressed_count_;
      last_replan_outcome_ = ReplanOutcome::PENDING;
      ROS_INFO_THROTTLE(1.0, "[replan-pending] ... action=WAIT_FOR_SUCCESSOR_ACTIVATION ...");
      return ReplanOutcome::PENDING;
    }
```

`planner_manager.h` 约 323 行：

```cpp
    bool localActivationPending() const {
      return traj_.local_traj.traj_id > 0 &&
          (executed_traj_id_ != traj_.local_traj.traj_id || executed_generation_ != active_traj_generation_);
    }
```

`executed_traj_id_/executed_generation_` 只在执行器 adoption 回调
（`planner_manager.cpp` 约 350 行，订阅 `/trajectory_execution/activated`）里更新。
于是只要执行器反馈晚到（或丢失），后续每一个 timer tick 都会在入口被跳过，
rolling planning 直接停摆 —— 这就是"有 pending 就停止 rolling replanning"，
与上一轮要修的 busy-loop 一样是错误的。

### 正确原则（只有一句）

**限制同一个 planning tick 内的重复求解，绝不能限制下一个正常 planning tick 继续规划。**

### 必须做的修改

1. **删除 pending 作为 planning 入口 gate。**
   `callReboundReplan()` 里那段 `localActivationPending()` 提前 return `PENDING` 的代码必须删除；
   `localActivationPending()` 只允许用于：future trajectory 状态、predecessor 关系、
   replacement/supersede 判定、遥测。**不得**再作为正常 planner 的入口条件。
   同时检查 `planner_manager.cpp` 约 1185 行
   `if(!local_geometry_active_ || localActivationPending()) return false;`
   （`geometryRecoveryReplanDue()`）：pending 不得阻止下一轮 rolling planning，
   该判断里 `localActivationPending()` 也必须去掉（`!local_geometry_active_` 保留）。

2. **busy-loop 改为"同一 tick 最多一次真实 planning attempt"。**
   在 `execFSMCallback()` 内实现一个 same-tick guard，语义必须是：

```text
onPlanningTick():
    if (已经在本 tick 内执行过一次真实 planning attempt)
        return;                     // 结束本 tick
    标记本 tick 已尝试
    outcome = callReboundReplan(...)   // 一次
    处理 outcome（状态迁移、发布等）
    return;                         // 立即结束当前 callback
下一个 timer tick：标记自动复位，允许再次 planning
```

   具体做法（可自由选择等价实现，但语义必须一致）：
   - 在 `execFSMCallback()` 进入 `switch` 之前把 `planning_attempted_this_tick_ = false;`
     （每个 timer tick 复位一次）；
   - 在 `GEN_NEW_TRAJ` / `REPLAN_TRAJ` / `SEQUENTIAL_START` 这些会真正调用
     `planFromGlobalTraj()` / `planFromLocalTraj()` / `callReboundReplan()` 的分支里，
     先判断 `planning_attempted_this_tick_`，若已为 true 则不再进入 planner；
   - 真正调用 planner 之前把它置为 true；
   - **`REPLAN_TRAJ` 在 attempt 失败后不得在同一次 callback 里再次进入 planner**：
     attempt 后直接 `break;`（回到 `force_return`），把重试留给下一个 timer tick。
     不允许出现"同一 tick 内 REPLAN_TRAJ → 再进 planner → REPLAN_TRAJ"的循环。

3. **preserve/test：pending 存在时仍必须允许 planning。**
   明确保证下面四条语义（在代码注释里写清楚，并在日志里可观测）：

```text
1) 有 pending + 本轮 SUCCESS → 新 successor 可以更新/替换旧 pending（走现有 supersede 路径）
2) 有 pending + 本轮 FAILED  → 保留旧 pending，下一 tick 继续正常 planning
3) 无 pending + SUCCESS      → 正常安装 successor
4) 无 pending + FAILED       → 本 tick 结束，下一 tick 继续 planning
```

   **禁止新增**：pending lock、dwell、confirm、最小替换间隔、generation freeze、
   "已有 future 就不规划"、"已有 future 就拒绝新 candidate"。若新 candidate 通过
   现有的 predecessor P/V/A、identity/generation、current revision、hard preflight、
   activation timing 检查，就允许它替换 pending future。

4. **Joint 必须是真正后台任务。**
   检查 `finalizeCapturedCandidates()`（`planner_manager.cpp`）返回
   `TopologyProcessStatus::PENDING` 时 `ego_replan_fsm.cpp` 的 `finish_captured`
   处理（约 1023-1046 行，上一轮被我改成返回 `ReplanOutcome::PENDING`）：
   joint pending / joint timeout / joint failure **都不得阻止本 tick 之外的 local planning**。
   `JOINT_FAILURE_LOCAL_NOOP` 的 NOOP 只能表示"Joint 失败不覆盖 Local"，
   不能表示"Local 停止规划"。
   如果去掉 pending gate 后 `ReplanOutcome::PENDING` 已无任何"阻止下一 tick"的语义，
   可以让 `finish_captured` 在该状态下直接返回 `ReplanOutcome::FAILED`/`SUCCESS`
   让 FSM 正常推进；**不要**重构 Joint 架构。

   注意：**不要**修改 Joint 的算法、超时值、窗口、协调协议。

5. **删除不再使用的成员/计数器**（避免留下误导性遥测）：
   `pending_replan_suppressed_count_` 若不再有语义，改成新名字与语义
   `PLANNING_TICK_SKIPPED_SAME_TICK_COUNT`（同 tick 内第二次 attempt 被抑制的次数），
   或在保留名字的同时改为统计"同 tick 抑制"，并在报告中说明。

---

## 二、必须新增的逐机遥测（第一验收标准）

### 2.1 每架 UAV 的 planning cadence

在 `EGOReplanFSM` 内按 `planner_manager_->pp_.drone_id` 分别统计
（**必须是 per-UAV，不能只统计本进程自己**；每个 drone 的 planner 是独立进程，
因此"本进程的 drone_id"就是该机的编号，直接用即可）。

每次真实 planning attempt（进入 planner 之前打点）记录 wall time，统计：

- `REPLAN_TOTAL_COUNT`
- `REPLAN_INTERVAL_P50` / `REPLAN_INTERVAL_P95` / `REPLAN_INTERVAL_MAX`（单位 ms）
- `REPLAN_INTERVAL_LT_10MS_COUNT`

输出行（每 5 秒一次 + 结束时一次），前缀固定，必须可 grep：

```
[replan-cadence-per-uav] drone=%d REPLAN_TOTAL_COUNT=%lu REPLAN_INTERVAL_P50_MS=%.3f REPLAN_INTERVAL_P95_MS=%.3f REPLAN_INTERVAL_MAX_MS=%.3f REPLAN_INTERVAL_LT_10MS_COUNT=%lu
```

### 2.2 pending 期间是否仍在规划

- `PLANNING_TICK_WHILE_LOCAL_PENDING_COUNT`：真实 planning attempt 发生的那一刻
  `localActivationPending()` 为 true 的次数（即"有 pending 仍在规划"）。
- `PLANNING_TICK_WHILE_JOINT_PENDING_COUNT`：attempt 发生时 joint/topology pending 为 true 的次数
  （用 `planner_manager_` 现有的 topology pending 查询接口，若没有 public 查询接口，
  用现有 `TopologyProcessStatus` / `pending_topology_` 可观测状态的最小只读封装，
  不要改 Joint 架构）。
- `SAME_TICK_DUPLICATE_REPLAN_SUPPRESSED_COUNT`：同一 tick 内第二次 attempt 被抑制的次数。

输出行：

```
[planning-gate-audit] drone=%d PLANNING_TICK_WHILE_LOCAL_PENDING_COUNT=%lu PLANNING_TICK_WHILE_JOINT_PENDING_COUNT=%lu SAME_TICK_DUPLICATE_REPLAN_SUPPRESSED_COUNT=%lu
```

### 2.3 运动连续性（第二验收标准，用户可见）

**复用现有 stall / terminal hold 遥测里的低速定义，不要新造控制阈值。**
先找到现有定义（例如 `traj_server.cpp` / `trajectory_lifecycle::sample` /
现有 `minimum_speed_*_reference`、`speed_before_activation`、
`terminal_hold` 相关代码里已经在用的低速阈值），把它作为 low-speed 判据；
如果代码里确实没有可直接复用的阈值，就用 `0.10 m/s` 并在报告里说明来源。

在 `traj_server.cpp` 的 `cmdCallback()`（已经每 10 ms 发布 PositionCommand）
里按 10 ms 采样每架 UAV 的实际指令速度，统计：

- `MOVING_DURATION`（该机处于速度 > 阈值的累计时长）
- `LOW_SPEED_EPISODE_COUNT`
- `LOW_SPEED_EPISODE_P50/P95/MAX`
- `TRAJECTORY_ACTIVATION_GAP_P50/P95/MAX`（相邻两次 trajectory 激活的时间间隔）
- `MAX_SUCCESSOR_GAP`（相邻两次 successor 激活的最大间隔）

输出行（每 5 秒 + 结束）：

```
[motion-continuity] drone=%d MOVING_DURATION=%.3f LOW_SPEED_EPISODE_COUNT=%lu LOW_SPEED_EPISODE_P50=%.4f LOW_SPEED_EPISODE_P95=%.4f LOW_SPEED_EPISODE_MAX=%.4f TRAJECTORY_ACTIVATION_GAP_P50=%.4f TRAJECTORY_ACTIVATION_GAP_P95=%.4f TRAJECTORY_ACTIVATION_GAP_MAX=%.4f MAX_SUCCESSOR_GAP=%.4f
```

低速事件判据说明（务必写进代码注释）：速度从 > 阈值跌到 <= 阈值开始记一次 episode，
回到 > 阈值结束；episode 长度单位秒。

### 2.4 第一条走走停停事件的完整因果链（重要）

当 **第一次** 出现 low-speed episode 时，打印一条完整快照（只打第一条，用一次性标志）：

```
[first-stop-and-go] drone=%d timestamp=%.9f speed_before=%.6f min_speed=%.6f
  active_trajectory_id=%d active_generation=%lu remaining_duration=%.6f
  last_successful_planning_time=%.9f next_planning_time=%.9f
  local_pending=%d joint_pending=%d candidate_generation=%lu
  nominal_status=%s plus_status=%s minus_status=%s
  future_successor_id=%d scheduled_activation=%.9f validated_end=%.9f actual_activation=%.9f
```

只允许使用现有可观测状态，不要为了这条日志新增跨模块接口。

---

## 三、禁止修改（除非第一根因明确落在 pending 调度语义）

不要修改：N/L/R geometry、BODY/LOS semantics、raw LOS detector、Local-SFC、
LOS observation plane 数学、MINCO/SCP、P/T/yaw、visibility objective、
target-distance objective、physical safety threshold、FREE TIME、
Feedback80 lifecycle semantics、target route、controller、
traj_server hold 策略、SIGSEGV、future queue 整体架构、Joint 算法与超时值。

不要顺手修上一轮遗留的 `TERMINAL_HOLD`、`JOINT_TIMEOUT`、LOS plane、
timebase 计数器定义等问题，除非第一根因就是 pending 调度。

---

## 四、编译

只允许：

```bash
source /opt/ros/noetic/setup.bash
source /home/bob/ALP/guidance/ros_ws/devel/setup.bash
cd /home/bob/ALP/egov2_fc65423_constvel
catkin build traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

不要跑 contract tests、smoke suite、hash tests、OFF/A-B、无关测试矩阵。
**不要启动仿真**（仿真由我来跑）。

---

## 五、完成后用中文回复

- 逐条列出修改的文件与行号范围；
- 编译结果；
- 明确说明 `localActivationPending()` 现在还有哪些调用点、各自语义；
- 明确回答：修复后"有 pending 时下一个正常 planning tick 是否仍会 planning"，以及代码依据；
- 列出新增的全部日志前缀与计数器名称，便于 grep；
- `CODE_MODIFIED_THIS_TURN: YES`
- `RRCT_ACCESSED: NO`
- `RRCT_CHANGED: NO`
