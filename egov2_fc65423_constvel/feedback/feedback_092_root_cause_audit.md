# Feedback092 根本原因审计（只读轮）

**审计对象**：`runs/20260921_131436_177465`（Feedback091，FULL 消融）
**对照对象**：`runs/20260920_160211_684140`、`runs/20260920_190112_693595`（Feedback090）
**核心问题**：为什么 Feedback090 没有 terminal hold，而 Feedback091 出现？为什么 drone_0 恢复、drone_2 不恢复、drone_1 不受影响？

**本轮约束遵守声明**：未修改任何源码 / 配置 / 参数 / launch / 脚本 / 日志分析程序；未 commit / reset / checkout / clean；未生成补丁；未"顺手修复"；未访问 RRCT；未运行新的大型仿真。全部结论仅由既有 `roslaunch_stdout.log`、`visibility_trajectory.csv` 与 `ros_ws/src` 只读检视得出。

---

## 1. 结论摘要（一句话）

> **第一分叉不是一个控制/优化/可行性问题，而是一次"先提交、后撤销"的团队事务：**
> `1789967703.0027` 规划器针对 `drone=2` 发出全 run **唯一一次** `TEAM_COMMIT_REVOKE_REQUEST team_solution_id=43 reason=FINAL_PREFLIGHT_FAILED`，并在 2–5 ms 后向**三台** traj_server 广播 `CANCEL_ALL_FUTURE`。该撤销按 `team_solution_id` **擦除了执行器侧尚未激活的后继轨迹**，而本地规划器仍把它当作链接前驱。执行器与本地规划器对"前驱是谁"的认知自此永久分叉，`executionHandoffGate` 静默回退到陈旧 active 轨迹做 PVA 比对，产生 `ACTIVE_PVA_MISMATCH` 硬拒绝；该拒绝在入队逻辑之前 `return`，后继被销毁而非暂存，形成正反馈自锁。

---

## 2. 证据基线

| 项 | 值 |
|---|---|
| Feedback091 run | `runs/20260921_131436_177465` |
| 退出状态 | `CORE_EXIT_CODE=0` / `FINAL_EXIT_CODE=0` / `LAST_BOOT_STAGE=BOOT-12` / `SIMULATION_REACHED_BOOT_12=YES` |
| 消融模式 | `ABLATION_MODE=full`；`team_pt=ON`、`los_topology=ON`、`dynamic_body_topology=ON`、`local_visibility=ON`、`local_gap=ON`；`joint_yaw=OFF [LOCKED]` |
| 日志跨度 | `1789967681.55 … 1789967913.82`（≈232 s） |
| 关键窗口 | `1789967702.5 – 1789967750.0` |

三个时间戳基准说明：日志内 `[<epoch>]` 为 wall 时间；`visibility_trajectory.csv` 的 `timestamp` 列同为 wall epoch，可直接与日志对齐（本轮已验证，无需偏移补偿）。

---

## 3. 事件时间线（T0 = 1789967703.0027）

### 3.1 T0 之前：正常的团队共同激活闭环

```
1789967702.9710  [TEAM_EXECUTION_PREPARE] reference_id=43 activation=1789967703.575293541 source=THREE_LOCAL_REALIZATIONS authority=LOCAL_ONLY
1789967702.9885  [TEAM_EXECUTION_COMMIT]  reference_id=43 activation=1789967703.575293541 executor_ready=1,1,1
1789967702.9888  [TEAM_EXECUTION_COMMIT]  drone=0 team_solution_id=43 activation=1789967703.575293541
1789967702.9888  [TEAM_EXECUTION_COMMIT]  drone=2 team_solution_id=43 activation=1789967703.575293541
1789967702.9889  [TEAM_EXECUTION_COMMIT]  drone=1 team_solution_id=43 activation=1789967703.575293541
```

三机执行器均已 `PREPARE → READY` 并接受提交。此时各机 traj_server 把该团队实现放入未来调度槽：

```
drone_2: [traj-server-scheduled]       id=77 activation_time=1789967703.5753 lead=0.5866 duration=5.9284 cur=76
drone_0: [traj-server-scheduled-queue] id=78 activation_time=1789967703.5753 lead=0.5866 duration=4.7786
```

**共同激活契约（`same_activation_contract=1`）为整队保留了 lead≈0.59 s 的未来激活时刻 703.5753**。这是后续致命性的前提。

### 3.2 T0：撤销

```
1789967703.0027  [planner-commit-atomic] event=TEAM_COMMIT_REVOKE_REQUEST drone=2 team_solution_id=43 reason=FINAL_PREFLIGHT_FAILED
1789967703.0029  [traj-server-team-cancel] node=/drone_0_traj_server team_solution_id=43
                 reason=EXECUTION_REVOKE:TEAM_COMMIT_REVOKE:FINAL_PREFLIGHT_FAILED action=CANCEL_ALL_FUTURE
1789967703.0030  [traj-server-team-cancel] node=/drone_1_traj_server … action=CANCEL_ALL_FUTURE
1789967703.0032  [traj-server-team-cancel] node=/drone_2_traj_server … action=CANCEL_ALL_FUTURE
```

**提交到撤销间隔 = 14 ms。** 撤销仅由 drone=2 一台的本地终检失败触发，却对三台执行器全部生效。全 run 中 `TEAM_COMMIT_REVOKE_REQUEST` **仅此一次**（已全量核验）。

### 3.3 T0 之后：三机命运分岔（唯一判据 = 撤销到达瞬间该机所处状态）

| 无人机 | 撤销瞬间状态 | 撤销的后果 | `ACTIVE_PVA_MISMATCH` | 后续 |
|---|---|---|---|---|
| **drone_1** | **已 ACTIVE**<br>`[traj-server-scheduled-activate] id=77 previous=76 activation_time=1789967703.001597881 actual_time=1789967703.0108`（早于撤销广播 **1.4 ms**） | `traj_server.cpp:336-345` 的 scheduled 擦除分支不适用；前驱身份未断裂 | **1 次**（id=79 @703.5809） | 下一笔本地后继 id=80 @703.7239 被接受、@703.8306 激活 → **链 0.25 s 内接回** |
| **drone_0** | **在 QUEUE 中**（id=78） | 队列项被 `erase`（`:346-352`） | **84 次**（703.5782 → 715.4816，跨 11.9 s） | 706.5455 `TERMINAL_HOLD_ENTER` traj=77 → 720.2253 `TERMINAL_HOLD_EXIT` duration=**13.679677963** |
| **drone_2** | **在 scheduled_ 中**（id=77） | 调度槽被清（`:336-345`） | **144 次**（703.1210 → 741.0978，跨 38.0 s） | 707.2722 `TERMINAL_HOLD_ENTER` traj=76 → **无 EXIT**（至日志末 913.82，仍 hold） |

### 3.4 首次拒绝的量值（关键：候选在物理上是好的）

drone_2 第一次拒绝 `1789967703.1209 [execution-handoff] stage=RECEIVE drone=2 OLD_ID=76 NEW_ID=78`：

| 轴 | old_p | new_p | old_v | new_v | old_a | new_a |
|---|---|---|---|---|---|---|
| x | −11.634302 | −11.635102 | 0.779101 | 0.761108 | 0.323084 | 0.062486 |
| **y** | 0.338074 | **0.341366** | 0.329249 | **0.403304** | **−1.108481** | **−0.035236** |
| z | 1.497316 | 1.497316 | 0.000705 | 0.000706 | 0.000122 | 0.000134 |

`HANDOFF_DP=0.0033883`（3.4 **毫米**）、`HANDOFF_DV=0.0762101`、`HANDOFF_DA=1.1044304`、`old_remaining=4.0434`（前驱远未到期）。

对照：同一前驱 76、13 ms 之前的 `1789967702.9888` 那条（id=77，团队实现）为 `DP=2.4370829e-07 / DV=1.1758803e-08 / DA=5.2826075e-08` —— 完全吻合；健康期一路为 `1e-7 … 1e-6` 量级。**即：残差在 0.13 s 内跃升 4 个数量级，而绝对量级仍只有 3.4 mm。**

### 3.5 正反馈发散（drone_2，DP 单调增长，`old_remaining` 转负）

| wall | old→new | HANDOFF_DP | accepted | old_remaining |
|---|---|---|---|---|
| 702.9714 | 76→77（团队） | 2.44e-07 | **1** | 3.688 |
| 702.9888 | 76→77（团队） | 2.44e-07 | **1** | 3.688 |
| **703.1209** | 76→78 | **3.3883e-03** | **0** | 4.043 |
| 703.2386 | 76→79 | 2.0978e-02 | 0 | 3.926 |
| 703.3812 | 76→80 | 7.0428e-02 | 0 | 3.783 |
| 703.6552 | 76→81 | 2.4879e-01 | 0 | 3.509 |
| 703.9305 | 76→82 | 4.7503e-01 | 0 | 3.234 |
| 704.3211 | 76→85 | 7.3393e-01 | 0 | 2.845 |
| 706.3108 | 76→100 | 8.4427e-01 | 0 | 0.853 |
| 707.1245 | 76→107 | 8.5935e-01 | 0 | 0.000 |
| 708.6043 | 76→120 | 1.2952e+00 | 0 | **−1.440** |
| 709.4050 | 76→127 | 2.0466e+00 | 0 | **−2.240** |

`old_remaining` 转负说明**执行器的"活动轨迹"76 早已过期却仍被当作 active 前驱**，而规划器仍在以 ~0.107–0.12 s 的周期（健康期 0.26–0.30 s）产出后继。

### 3.6 前驱身份分叉的直接证据（drone_2 @ 1789967707.12）

同一时刻两条日志并列：

```
[handoff-gate]      drone=2 OLD_ID=106 NEW_ID=107 HANDOFF_DP=0.000000000 HANDOFF_DV=0.000000000 HANDOFF_DA=0.000000000
[execution-handoff] stage=RECEIVE drone=2 OLD_ID=76 NEW_ID=107 HANDOFF_DP=0.85935230425454701
```

**规划器自检残差 = 0（对 OLD_ID=106），执行器实测残差 = 0.859（对 OLD_ID=76）。** 两侧对同一份后继使用不同的前驱权威，且互不校验。

### 3.7 恢复事件

```
drone_0
1789967718.9944 [current-state-restart] event=PREDECESSOR_HANDOFF_DEADLINE_EXPIRED drone=0 expired_predecessor_id=162
                t_validated_end=1789967719.092279673
1789967718.9989 [current-state-restart] event=ENTER drone=0 expired_predecessor_id=162 authority=ODOMETRY_SAME_LOCAL_PLANNER
1789967718.9990 [current-state-restart] event=START_AUTHORITY drone=0 source=ODOMETRY_PV_ZERO_ACCELERATION
                p=(-8.450388,-1.900087,1.500000) v=(-0.000000,-0.000000,0.000000) a=(0.000000,0.000000,0.000000)
1789967719.0751 … 1789967719.6181  多轮 ATTEMPT / CANDIDATE_REANCHORED / NO_SAFE_CANDIDATE
1789967720.1158 [traj-server-scheduled] id=163 activation_time=1789967720.2150 duration=6.8764 current_trajectory_id=77
1789967720.2253 [execution-lifecycle] event=TERMINAL_HOLD_EXIT node=/drone_0_traj_server trajectory_id=77 duration=13.679677963

drone_2
1789967749.2532 [current-state-restart] event=ENTER drone=2 expired_predecessor_id=221 authority=ODOMETRY_SAME_LOCAL_PLANNER
                ← 此后至日志末（1789967913.82，约 164 s）：planner-traj-commit = 0
```

全 run `current-state-restart event=ENTER` 共 4 次：drone_0 ×3（718.9989 / 729.4714 / 729.5185）、drone_2 ×1（749.2532）；**drone_1 一次也没有**（12 条相关日志，0 条 ENTER）。

---

## 4. 定位到源码的因果链

### 4.1 撤销擦除了执行器侧的前驱

`ros_ws/src/EGO-Planner-v2/.../plan_manage/src/traj_server.cpp:336-352`

```cpp
if (scheduled_traj_valid_ &&
    scheduled_source_.team_solution_id == message->team_solution_id &&
    now < scheduled_start_time_)                    // 336-338
{ scheduled_traj_valid_ = false; scheduled_traj_.reset(); ... }   // 340-344
scheduled_traj_queue_.erase(                          // 346-352
    std::remove_if(..., [&](const ScheduledTrajectoryEntry &entry) {
        return entry.source.team_solution_id == message->team_solution_id; }),
    scheduled_traj_queue_.end());
```

按 `team_solution_id` 无条件擦除，**不检查该条目是否已被本地规划器采纳为链接前驱**。

### 4.2 门禁静默回退到陈旧前驱（第一处缺陷）

`traj_server.cpp:429-455`

```cpp
const bool future_admission = stage=="RECEIVE" || stage=="PREPARE";          // 429-430
const bool use_queued_predecessor =
    future_admission && !scheduled_traj_queue_.empty() &&
    scheduled_traj_queue_.back().source.generation < source.generation &&
    activation > scheduled_traj_queue_.back().start_time.toSec() + 1.0e-6;   // 431-434  ← 要求激活时刻严格递增
const bool use_scheduled_predecessor =
    future_admission && !use_queued_predecessor &&
    scheduled_traj_valid_ && scheduled_traj_ &&
    scheduled_source_.generation < source.generation &&
    activation > scheduled_start_time_.toSec() + 1.0e-6;                     // 435-439  ← 同上
const poly_traj::Trajectory *predecessor =
    use_queued_predecessor ? scheduled_traj_queue_.back().traj.get() :
    (use_scheduled_predecessor ? scheduled_traj_.get() : traj_.get());       // 440-442  ← 静默回退
...
const auto old_state = trajectory_lifecycle::sample(*predecessor, activation - predecessor_start);  // 453
const auto new_state = trajectory_lifecycle::sample(next, 0.0);                                     // 454
const auto residual  = trajectory_lifecycle::compare(old_state, new_state);                          // 455
```

两条"使用计划前驱"的分支都带 `activation > <reserved_start> + 1e-6` 前置条件。当后继的激活时刻**早于**已保留的团队激活时刻时（本地 `lead≈0.099` 对团队 `lead≈0.5866`），两个分支同时为假 → **`predecessor` 静默退化为 `traj_`（陈旧 active）**，比对的就不是规划器实际链接的那条轨迹。**门禁没有任何"前驱身份不一致"的检测，也不会因此拒绝并报告——它直接换了一条前驱继续算残差。**

### 4.3 拒绝路径早于入队路径（自锁的关键）

`traj_server.cpp:661-665` 与 `687-731`

```cpp
if (!executionHandoffGate(*new_traj,*msg,"RECEIVE")) {
  ROS_ERROR("[traj-server-handoff-reject] … reason=ACTIVE_PVA_MISMATCH", …);
  return;                                            // 664 ← 直接返回
}
...
if (scheduled_traj_valid_ && scheduled_traj_) {      // 708
  ScheduledTrajectoryEntry queued; ...
  scheduled_traj_queue_.push_back(queued);           // 719 ← 本可容纳乱序后继的路径
  return;
}
```

**被拒的后继在到达 719 行之前就 `return` 了。** 队列本是为"后继先于前驱激活到达"设计的缓冲区，但恰恰在需要它的场景下不可达 → 后继被**销毁**而非暂存 → 链永不收敛。

### 4.4 唯一的逃生门

`traj_server.cpp:456-460`

```cpp
const bool current_state_restart = trajectory_lifecycle::currentStateRestartHandoff(
    source.lifecycle_state=="CURRENT_STATE_RESTART_VALIDATED",
    source.safety_validated, predecessor_source.generation, source.generation, activation);
const bool valid = current_state_restart || residual.accepted(handoff_tolerances_);   // 460
```

`current_state_restart` 为真时**完全绕过 PVA 残差容差**。这是 drone_0 唯一的出路，也是 drone_1 用不到、drone_2 走不通的门。

---

## 5. 为什么 Feedback090 没有 terminal hold

全量计数对照（`roslaunch_stdout.log` 逐行统计）：

| 指标 | 090 `…160211_684140` | 090 `…190112_693595` | **091 `…131436_177465`** |
|---|---|---|---|
| `TEAM_EXECUTION_PREPARE]` | **0** | **0** | 29 |
| `TEAM_EXECUTION_COMMIT]` | **0** | **0** | 51 |
| `traj-server-team-cancel` | **0** | **0** | **51** |
| `TEAM_COMMIT_REVOKE_REQUEST` | 1 | 3 | 1 |
| `traj-server-handoff-reject` | **0** | **0** | **229** |
| `ACTIVE_PVA_MISMATCH` | **0** | **0** | **229** |
| `traj-server-scheduled-queue` | 178 | 103 | 38 |

**决定性事实**：Feedback090 中 `TEAM_COMMIT_REVOKE_REQUEST` 确实触发过（1 次、3 次），但 `traj-server-team-cancel` **为 0** —— 撤销消息从未真正作用到 traj_server。原因：090 中 Team 执行器流水线（`EXECUTOR_PREPARE → 3×EXECUTOR_READY → COMMIT → 共同激活`）**尚不存在**，发布的 `PolyTraj.team_solution_id` 不是被撤销的那个 id，`traj_server.cpp:336-352` 的匹配条件永不成立。

**因此 090 中不存在任何"从执行器侧移除一条未激活后继"的路径**；执行器与本地规划器对前驱的认知始终一致，`traj_server.cpp:440-442` 的静默回退分支**从未被触发**，229 次 `ACTIVE_PVA_MISMATCH` 一次都没发生。090 的 revoke 是**空操作**。

> 091 新增的 common-activation 执行器流水线，第一次让"先提交、后撤销"具备真实的执行器副作用 —— 这是 Feedback091 引入 terminal hold 的**充分且必要**的新增条件（回归类型见 §10）。

---

## 6. 为什么 drone_0 在 13.679678 s 后恢复

恢复**不是自愈**，而是**另一条旁路门禁**：

1. 规划器自身的前驱 162 到期：`1789967718.9944 PREDECESSOR_HANDOFF_DEADLINE_EXPIRED … t_validated_end=1789967719.092279673`；
2. 进入里程计重锚定：`1789967718.9989 ENTER authority=ODOMETRY_SAME_LOCAL_PLANNER`；`1789967718.9990 START_AUTHORITY source=ODOMETRY_PV_ZERO_ACCELERATION p=(-8.450388,-1.900087,1.500000) v=(0,0,0) a=(0,0,0)` —— 以**实测静止状态**为新起点，不再与任何历史轨迹比 PVA；
3. 该状态置 `lifecycle_state=CURRENT_STATE_RESTART_VALIDATED`，执行器在 `traj_server.cpp:460` 走 `current_state_restart ||` 短路，**跳过残差容差**；
4. `1789967720.1158 [traj-server-scheduled] id=163 activation_time=1789967720.2150 current_trajectory_id=77` 被接受；
5. `1789967720.2253 TERMINAL_HOLD_EXIT duration=13.679677963`。

13.6797 s 全部消耗在"等待自身前驱的 `t_validated_end` 到期"上，而非任何计算或恢复尝试。

---

## 7. 为什么 drone_2 没有恢复

drone_2 **走进了**同一恢复路径，但**走不出去**：

```
1789967749.2424 [MOVING_SUCCESSOR_STARVATION] drone=2 trajectory_id=221 remaining=0.099266 reason=NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE
1789967749.2532 [current-state-restart] event=ENTER drone=2 expired_predecessor_id=221 authority=ODOMETRY_SAME_LOCAL_PLANNER
```

首次进入比 drone_0 晚 **30.25 s**、比自身 hold 晚 **41.98 s**。进入后至日志末（1789967913.8229，约 164 s）：

| 事件 | 次数 |
|---|---|
| `current-state-restart` 相关行 | 8181 |
| `[execution-reserve] … OBJECTIVE_EVALUATION_CANCELLED` | 多轮，持续 |
| `… reason=EXECUTION_DEADLINE` | 多轮，持续 |
| `NO_EXECUTABLE` | 2 |
| **`planner-traj-commit`** | **0** |

每轮 `planning_budget` 塌缩到 **0.013–0.038 s**，候选在截止前被取消（`initializer_restored=1 partial_cost_accepted=0`）。**即：drone_2 卡死不是"没走恢复路径"，而是恢复路径上拿不到任何可执行候选 —— 预算饥饿。**

---

## 8. 为什么 drone_1 不受影响

三机唯一差别是**撤销到达瞬间该机所处状态**（同一份 `team_solution_id=43`、同一时刻 703.5753 保留激活）：

```
1789967703.0016  ← drone_1 的 id=77 保留激活时刻
1789967703.0030  ← 撤销广播到达 drone_1
1789967703.0108  [traj-server-scheduled-activate] node=/drone_1_traj_server trajectory_id=77 previous_trajectory_id=76
                 activation_time=1789967703.001597881 actual_time=1789967703.0108
```

**激活比撤销早 1.4 ms。** 撤销到达时该轨迹已是 `ACTIVE`，`traj_server.cpp:336-345` 的 `now < scheduled_start_time_` 条件不成立，擦除分支不适用（且 `already_active` 分支不改变执行状态）。执行器的前驱与规划器的前驱保持同一，故只有 1 次瞬时残差拒绝（id=79 @703.5809），下一笔本地后继 id=80 @703.7239 即被接受、@703.8306 激活。

**drone_0（QUEUE 中的 id=78）与 drone_2（`scheduled_` 中的 id=77）都在"未激活"状态被擦除** → 前驱身份永久分叉。判据是**状态机位置**，不是机型、参数或任务分配差异。

---

## 9. 分级判定

| 分级 | 结论 | 证据 |
|---|---|---|
| **ROOT CAUSE** | **按 `team_solution_id` 撤销整队已提交解时，无保护地擦除执行器侧未激活后继，且不通知/不校正本地规划器所链接的前驱** —— `traj_server.cpp:336-352`；配合 `executionHandoffGate` 在"激活时刻非递增"时**静默替换前驱**（`:440-442`），使前后两侧对同一后继使用不同前驱权威且互不校验 | 唯一一次 revoke @703.0027；三机 2–5 ms 内收到 `CANCEL_ALL_FUTURE`；drone_0/2 前驱分叉；`handoff-gate OLD_ID=106` vs `execution-handoff OLD_ID=76` |
| **DIRECT TRIGGER** | 该次撤销本身（drone=2 单机 `FINAL_PREFLIGHT_FAILED` 却对三机生效），发生在**提交后 14 ms**、且在保留激活时刻 703.5753 **之前 0.572 s** | `TEAM_EXECUTION_COMMIT` @702.9885 → revoke @703.0027；首次拒绝 @703.1210 |
| **AMPLIFIER** | ① 拒绝路径 `:661-665` 早于入队路径 `:687-731` → 后继被销毁而非暂存；<br>② 拒绝后本地重规划周期由 0.26–0.30 s 收紧到 ~0.107–0.12 s，规划器以自己被拒的后继为基准继续链接 → 残差单调放大（DP 3.4e-3 → 2.05，`old_remaining` 4.04 → −2.24）；<br>③ 队列 `push_back`（`:719`）无激活时刻单调性校验；<br>④ 共同激活契约把整队激活保留到 lead≈0.59 s 的未来，使"本地后继更早"成为常态；<br>⑤ Team P/T 未能产出联合解，团队退化为 `source=THREE_LOCAL_REALIZATIONS authority=LOCAL_ONLY`（本 run：`THREE_LOCAL_REALIZATIONS`=29、`authority=LOCAL_ONLY`=115、`JOINT_CAS_STALE_AFTER_3ACK`=44、`NO_TOPK_OBJECTIVE_IMPROVEMENT`=7），把执行权交给"每 ~0.1 s 提交一次"的本地流 | §3.5 表；§4.3/§4.2 |
| **RECOVERY BUG** | ① 恢复只能靠 `current_state_restart` **旁路门禁**，而非修复前驱身份，属"绕开"而非"恢复"；<br>② 触发条件 `PREDECESSOR_HANDOFF_DEADLINE_EXPIRED` 依赖自身前驱自然到期，等待时长不可控（drone_0 13.68 s，drone_2 41.98 s）；<br>③ 恢复路径无预算保障：drone_2 进入后 `planning_budget` 塌缩至 0.013–0.038 s，164 s 内 0 次 commit；<br>④ 规划器侧 `[handoff-gate]` 与执行器侧 `[execution-handoff]` 是两套独立实现，规划器报 `DP=0.000000000` 而执行器报 `DP=0.859`，无任何交叉校验或告警 | §3.6、§3.7、§7 |
| **CONSEQUENCE** | `TERMINAL_HOLD_ENTER/EXIT`、`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`、`MOVING_SUCCESSOR_STARVATION`、`UNVALIDATED_EXECUTED_SAMPLES`、`STATIC_CONTACT_SAMPLES`、目标窗口内的位置/速度偏离（tracking error 最大 13.876 m） | §11、§12 |

---

## 10. 回归类型判定：**C（既是回归，也暴露既有潜在缺陷）**

| 维度 | 判定 | 依据 |
|---|---|---|
| **A：Feedback091 引入的新行为** | **成立（触发侧）** | 091 首次出现 `TEAM_EXECUTION_PREPARE=29` / `TEAM_EXECUTION_COMMIT=51` / `traj-server-team-cancel=51`；090 同三项全为 **0**。撤销第一次具备"从执行器移除未激活后继"的真实副作用 |
| **B：暴露既有潜在缺陷** | **成立（机制侧）** | `traj_server.cpp:429-442`（前驱静默回退 + 激活时刻严格递增假设）、`:661-665`（拒绝早于入队）、`:719`（队列无单调性校验）在 090 中已存在，仅因无擦除路径而从未被触发 |
| **D：证据不足** | 不成立 | 两侧（源码 + 运行时日志）均已定位到具体行与具体时刻 |

**结论：C。** 触发条件是 091 新增的（A），被触发的是 090 就已存在的结构缺陷（B）。二者缺一不可：
- 只有 A 无 B → 撤销不会致命（090 即为此情形）；
- 只有 B 无 A → 无擦除事件，回退分支永不进入。

---

## 11. 非根本原因清单（逐项反驳）

| 候选"原因" | 为什么不是 | 证据 |
|---|---|---|
| **TERMINAL_HOLD 本身** | 末链后果。hold 由 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` 产生，而后者由"从无后继被接受"产生 | drone_0 hold @706.5455 晚于首次拒绝 @703.5782 共 2.97 s |
| **`MOVING_SUCCESSOR_STARVATION`** | hold **之后**才出现 | drone_0 @729.4713（hold 后 22.93 s）；drone_2 @749.2424（hold 后 41.97 s） |
| **`TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`** | 是"active 过期 → hold"的机制描述，不是原因 | 上游为 229 次拒绝 |
| **候选不可行 / MINCO 失败 / preflight 失败** | 被拒候选在物理上**良好**：3.4 **毫米**位置偏差、`dynamics_valid=1`、`LOCAL_MINCO_HARD_PREFLIGHT_PASS`、`TEAM_REFERENCE_ACK accepted=1 realization_valid=1` | §3.4；拒绝原因字符串是 `ACTIVE_PVA_MISMATCH`（交接连续性门禁），不是任何可行性判据 |
| **未验证执行（unvalidated samples）** | **100% 落在 hold 区间内**，是 hold 的度量而非成因 | drone_0：410 样本 `[706.5774, 720.2183]`，traj=77，hold=`[706.5454, 720.2251]`；drone_2：6198 样本 `[707.2761, 913.8427]`，traj=76，hold 起点 707.2720；drone_1：**0** |
| **静态接触（17 样本）** | 全部在 drone_2 **已通过验证**的 traj=76 上，时刻 1789967706.110–1789967706.644，**早于 hold 0.63–1.16 s**，但**晚于首次拒绝 3.0 s** → 是发散期的后果 | `safety_validated=True` 与 `static_clearance_m=0.0` 同现 |
| **激活陈旧（stale activation）** | `scheduled_traj_valid_` 残留是**状态**，不是原因；使它致残的是撤销删除了规划器所链接的前驱 | §4.1 |
| **队列为空（`use_queued_predecessor=false`）** | 是 `erase` 的**结果** | drone_0 队列项 id=78 被擦除 |
| **J_vis（局部可见性代价）** | 完全不在拒绝路径上：`executionHandoffGate` 只读 PVA 残差、`generation`、`lifecycle_state`；且失败候选的 `J_VIS_TOTAL_MEAN` 仅 0.45–0.75 | `directional-visibility-cost` 日志与拒绝日志无因果关系；J_vis 最多贡献时延 |
| **Team P/T 未产出联合解** | 上游"条件化因素"而非根本原因：撤销理由是 **per-drone 本地终检** `FINAL_PREFLIGHT_FAILED`，而该解此前已 `executor_ready=1,1,1` 提交成功；团队的 `authority=LOCAL_ONLY`（115 次）与 `source=THREE_LOCAL_REALIZATIONS`（29 次）只是把执行权交给本地流，并未直接产生拒绝 | `[TEAM_REFERENCE_NOOP]` 原因分布（全 run）：`REALIZED_TEAM_VALIDATION_NO_LEXICOGRAPHIC_TEAM_IMPROVEMENT`=48、`JOINT_CAS_STALE_AFTER_3ACK`=44、`LOCAL_REALIZATION_FAILED:LOCAL_MINCO_REALIZATION_FAILED`=29、**`EXECUTOR_PREPARE_FAILED:EXECUTOR_PREPARE_HANDOFF_REJECTED`=27**、`STALE_PLANNING_GENERATION`=19、`STALE_TASK_REFERENCE`=12、`NO_TOPK_OBJECTIVE_IMPROVEMENT`=7 —— 注意其中 27 次团队路径**同样死在同一个门禁上**（下游症状，非上游原因）；本 run 日志中 `INITIAL_METRICS_INVALID`、`EARLY_JOINT_OPT_ATTEMPT`、`EARLY_JOINT_OPT_FAIL` 均为 **0 次**（该三项属其它 run 的统计，本报告不引用） |

---

## 12. 逐条回答任务书的五项归因要求

### 12.1 未验证执行样本归因 → **Case C（纯后果）**

| uav_id | 无人机 | 未验证样本 | 区间(wall) | traj_id | 对应 hold |
|---|---|---|---|---|---|
| 1 | drone_0 | **410** | 1789967706.5774 → 1789967720.2183 | 77 | 706.5454 → 720.2251 |
| 3 | drone_2 | **6198** | 1789967707.2761 → 1789967913.8427 | 76 | 707.2720 → 无（至日志末） |
| 2 | drone_1 | **0** | — | — | 无 |

判定规则：`visibility_trajectory.csv` 的 `safety_validated` 字段为 `False`。**未验证窗口的起止与 hold 窗口逐点重合，且涉及的 `trajectory_id` 恰为 hold 所指向的那条**（drone_0=77、drone_2=76）。因此不存在"未验证执行导致故障"的可能 —— 反向因果成立：**hold 导致未验证执行**。也没有"Case A（独立执行器缺陷）"或"Case B（独立验证器缺陷）"的证据。

### 12.2 静态接触时序归因 → **CONSEQUENCE**

17 个静态接触样本全部属于 `uav_id=3`（drone_2），时刻 1789967706.110–1789967706.644，全部落在 `trajectory_id=76`（`FEASIBLE_FALLBACK`，`source=local_first_safe_direct`）上，且**同一样本 `safety_validated=True`**。

时序关系（同一机）：
```
703.1210  首次 ACTIVE_PVA_MISMATCH（发散开始）
…         执行器仍执行 76，而规划器已按另一条链重新规划 →
706.110–706.644  17 个静态接触样本（static_clearance=0.0）
707.2720  TERMINAL_HOLD_ENTER
```
接触**晚于**首次拒绝 2.99 s、**早于** hold 1.16–0.63 s。归因：**前驱分叉导致"执行中轨迹的几何"与"规划器重规划所用参考"脱节，使一条曾通过静态校验的轨迹在真实执行中擦碰静态障碍** —— 是发散的后果，不是原因。`DYNAMIC_CONTACT_SAMPLES=0`。

### 12.3 J_vis 因果性 → **无因果，仅潜在时延**

- `J_vis` 的任何计数器都不出现在 `executionHandoffGate`（`traj_server.cpp:418-507`）的判定表达式中；
- 被拒候选的 `directional-visibility-cost` 遥测显示 `J_VIS_TOTAL_MEAN` 仅 0.45–0.75（量级正常）；
- 被拒原因是 `ACTIVE_PVA_MISMATCH`（PVA 残差超容差），与可见性代价无通路；
- 结论：J_vis **不是**本轮 terminal hold 的原因，也不构成必要条件。

### 12.4 共同激活因果性 → **DIRECT TRIGGER 的前提条件（AMPLIFIER）**

共同激活契约（`same_activation_contract=1`）为整队解保留 `activation=1789967703.575293541`，`lead=0.5866`。而本地流以 `lead≈0.099` 每 ~0.107–0.12 s 产出一笔后继。这使得"本地后继的激活时刻早于已保留的团队激活时刻"成为**结构性常态**（本例 703.2198 < 703.5753，早 0.355 s），从而让 `traj_server.cpp:434/439` 的两个分支同时为假。

**但共同激活本身不构成根本原因**：090 也有共同激活语义（`team_solution_id` 已存在于消息中），却因无擦除路径而未致命。因此分级为 **AMPLIFIER / 触发前提**。

### 12.5 快照身份与 Team 队列因果性

- **快照身份（snapshot identity）→ RECOVERY BUG**：`[handoff-gate] OLD_ID=106 HANDOFF_DP=0.000000000`（规划器）与 `[execution-handoff] OLD_ID=76 HANDOFF_DP=0.85935230425454701`（执行器）在同一时刻对同一后继给出**完全不同**的残差。两套门禁实现无交叉校验、无一致性告警，规划器据此继续以 10 Hz 提交"自认为完美"的后继。
- **Team 队列 → 非本轮故障点，但为潜在缺陷（AMPLIFIER）**：drone_2 的条目进入 `scheduled_traj_`（非队列）后被擦除；drone_0 的条目在 `scheduled_traj_queue_` 中被擦除。`push_back`（`:719`）**无激活时刻单调性校验**，而 `executionHandoffGate` 的 `use_queued_predecessor` 却假定 `back()` 是"最新的、激活最晚的"前驱 —— 该假设在乱序入队时会被破坏。本轮未由此触发，但属同一族的潜在缺陷。

### 12.6 FSM / 恢复因果性 → **RECOVERY BUG（见 §9）**

---

## 13. 机器可读摘要

```yaml
audit: feedback_092_root_cause_audit
run_audited: runs/20260921_131436_177465
run_reference: [runs/20260920_160211_684140, runs/20260920_190112_693595]
read_only: true
source_modified: NO
config_modified: NO
simulation_rerun: NO
rrct_accessed: NO
rrct_changed: NO

first_divergence:
  wall: 1789967703.0027
  event: TEAM_COMMIT_REVOKE_REQUEST
  drone: 2
  team_solution_id: 43
  reason: FINAL_PREFLIGHT_FAILED
  broadcast_to_drones: [0, 1, 2]
  broadcast_action: CANCEL_ALL_FUTURE
  ms_after_team_execution_commit: 14
  occurrences_in_run: 1
  first_rejection_after: {wall: 1789967703.1210, drone: 2, trajectory_id: 78,
                          dp: 3.388268131551745e-03, dv: 7.6210121833448427e-02,
                          da: 1.1044304173062593, old_remaining: 4.0433933734893799,
                          healthy_dp_before: 2.4370829284119006e-07}

root_cause:
  class: ROOT_CAUSE
  summary: >
    按 team_solution_id 整队撤销已提交解时，无保护地擦除执行器侧未激活后继，
    且不通知本地规划器；executionHandoffGate 在激活时刻非递增时静默把前驱
    替换为陈旧 active 轨迹，导致规划器/执行器对同一后继使用不同前驱权威。
  source:
    - file: ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp
      function: teamCancelCallback
      lines: 336-352
      defect: 按 team_solution_id 无条件 erase scheduled/_queue，不校验是否为规划器链接前驱
    - file: ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp
      function: executionHandoffGate
      lines: 429-442
      defect: 两个 use_*_predecessor 分支均要求 activation 严格大于保留激活；
              否则 predecessor 静默回退为 traj_（陈旧 active），无身份不一致检测
    - file: ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp
      function: polyTrajCallback
      lines: 661-665
      defect: 拒绝路径 return 早于入队路径 687-731，乱序后继被销毁而非暂存
    - file: ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp
      function: polyTrajCallback
      lines: 687-731
      defect: scheduled_traj_queue_.push_back 无激活时刻单调性校验（潜在，本轮未触发）

direct_trigger:
  class: DIRECT_TRIGGER
  summary: drone=2 单机 FINAL_PREFLIGHT_FAILED 触发整队撤销，且发生在保留激活时刻前 0.572 s

amplifiers:
  - class: AMPLIFIER
    id: reject_before_queue
    detail: 拒绝于 :661-665 return，永远无法到达 :719 的队列缓冲
  - class: AMPLIFIER
    id: replan_period_collapse
    detail: 周期由 0.26-0.30 s 收紧到 0.107-0.12 s，以被拒后继为基准继续链接
    dp_growth: [3.388268131551745e-03, 2.0465994383447361]
    old_remaining_growth: [4.0433933734893799, -2.2400338649749756]
  - class: AMPLIFIER
    id: common_activation_far_future_lead
    detail: 团队 lead≈0.5866 s vs 本地 lead≈0.099 s，使本地后继激活时刻结构性更早
  - class: AMPLIFIER
    id: team_pt_degraded_to_local_only
    detail: >
      Team P/T 未产出联合解，团队退化为 source=THREE_LOCAL_REALIZATIONS authority=LOCAL_ONLY
      （THREE_LOCAL_REALIZATIONS=29, authority=LOCAL_ONLY=115,
       JOINT_CAS_STALE_AFTER_3ACK=44, NO_TOPK_OBJECTIVE_IMPROVEMENT=7），
      执行权交由每 ~0.1 s 提交一次的本地流;
      INITIAL_METRICS_INVALID / EARLY_JOINT_OPT_* 在本 run 日志中均为 0 次

recovery_bugs:
  - class: RECOVERY_BUG
    id: bypass_not_repair
    detail: 恢复依赖 current_state_restart 旁路（:456-460），不修复前驱身份
  - class: RECOVERY_BUG
    id: unbounded_wait_for_predecessor_expiry
    detail: 触发依赖 PREDECESSOR_HANDOFF_DEADLINE_EXPIRED，等待时长不可控
  - class: RECOVERY_BUG
    id: no_budget_guarantee_in_recovery
    detail: drone_2 进入恢复后 planning_budget 塌缩至 0.013-0.038 s，164 s 内 0 次 commit
  - class: RECOVERY_BUG
    id: dual_gate_no_cross_validation
    detail: handoff-gate DP=0.000000000 vs execution-handoff DP=0.85935230425454701 同刻并存

consequences:
  - TERMINAL_HOLD_ENTER
  - TRAJECTORY_END_BEFORE_NEXT_ACTIVATION
  - MOVING_SUCCESSOR_STARVATION
  - UNVALIDATED_EXECUTED_SAMPLES
  - STATIC_CONTACT_SAMPLES
  - tracking_error_max_m: 13.876

answers:
  q1_why_090_no_hold: >
    Feedback090 中 Team 执行器流水线不存在（TEAM_EXECUTION_PREPARE=0、TEAM_EXECUTION_COMMIT=0、
    traj-server-team-cancel=0），TEAM_COMMIT_REVOKE_REQUEST（1 次 / 3 次）在 traj_server 侧匹配不到
    team_solution_id，属空操作，从未移除任何执行器侧后继；执行器与规划器前驱认知始终一致，
    executionHandoffGate 的前驱回退分支从未进入，ACTIVE_PVA_MISMATCH = 0。
  q2_why_drone0_recovered: >
    1789967718.9944 命中 PREDECESSOR_HANDOFF_DEADLINE_EXPIRED（predecessor 162），
    进入 ODOMETRY_SAME_LOCAL_PLANNER 并以 ODOMETRY_PV_ZERO_ACCELERATION 重锚定，
    lifecycle_state=CURRENT_STATE_RESTART_VALIDATED 使执行器走 :460 的 current_state_restart 短路，
    绕过 PVA 残差容差；1789967720.1158 接受 id=163，1789967720.2253 TERMINAL_HOLD_EXIT，
    duration=13.679677963 s（几乎全部为等待自身前驱到期） 。
  q3_why_drone2_not_recovered: >
    直到 1789967749.2532 才首次 ENTER 恢复路径（比 drone_0 晚 30.25 s，比自身 hold 晚 41.98 s）；
    其后 164 s 内 planner-traj-commit = 0，每轮 OBJECTIVE_EVALUATION_CANCELLED / EXECUTION_DEADLINE，
    planning_budget 塌缩至 0.013-0.038 s，恢复路径无预算保障。
  q4_why_drone1_unaffected: >
    其 id=77 在撤销广播前 1.4 ms（703.0016 vs 703.0030）已 ACTIVE，
    traj_server.cpp:336-345 的擦除分支不适用，前驱身份未断裂；
    仅 1 次瞬时拒绝（id=79 @703.5809），下一笔 id=80 @703.7239 即被接受。

regression_class: C
regression_class_detail: >
  A 成立：091 首次引入 Team 执行器 common-activation 流水线（PREPARE/COMMIT/cancel 计数 29/51/51，090 全为 0），
  使"先提交后撤销"首次具备执行器副作用。
  B 成立：traj_server.cpp:429-442 前驱静默回退、:661-665 拒绝早于入队、:719 队列无单调性校验
  在 090 已存在，仅因无擦除路径而未触发。
  二者缺一不可，故判定 C。

candidate_set_note: >
  本轮结论不依赖任何"候选集/威胁语义/安全阈值/物理参数"变更；
  未修改 body radius、N/L/R 语义、PVA handoff、控制器、traj_server hold 逻辑或场景几何。

deferred_fixes_not_applied:
  - 修复 INITIAL_METRICS_INVALID 阻断 Team P/T
  - 修复 team_visibility_optimizer 目标函数中 deviation_weight*deviation_cost 重复计项
  - 任何活性/恢复修复
```

---

## 14. 供后续轮次的修复切入点（本轮未实施）

仅作为定位结论的自然延伸记录，**本轮未做任何修改**：

1. **撤销的事务性**（对应 ROOT CAUSE）：撤销整队解时，若某执行器已把该后继登记为前驱候选，应把撤销升级为"带 PVA 重锚定的替换"，而不是按 id 静默擦除。
2. **门禁的前驱身份一致性**（对应 ROOT CAUSE）：`executionHandoffGate` 应检测"规划器所链接的前驱 ≠ 执行器所见前驱"，并在此情形下**显式拒绝并上报**，而不是静默换前驱算残差。
3. **拒绝不得早于入队**（对应 AMPLIFIER ①）：把 `:661-665` 的硬拒绝改为"先入队 + 标记待复核"，使乱序后继获得缓冲机会。
4. **恢复预算保障**（对应 RECOVERY BUG ③）：`current-state-restart` 路径需独立的预算下限，避免 `planning_budget` 塌缩导致 0 commit。
5. **双门禁交叉校验**（对应 RECOVERY BUG ④）：规划器侧 `[handoff-gate]` 与执行器侧 `[execution-handoff]` 残差不利时应产生告警。

---

## 15. 声明

```
SOURCE_MODIFIED:      NO
CONFIG_MODIFIED:      NO
SIMULATION_RERUN:     NO
RRCT_ACCESSED:        NO
RRCT_CHANGED:         NO
```

本轮仅执行只读检视：读取 `runs/20260921_131436_177465/roslaunch_stdout.log`、`visibility_trajectory.csv`、`ablation_*.txt`、`exit_status.txt` 与 `runs/20260920_*` 对照日志；读取 `ros_ws/src` 下 `traj_server.cpp` 等源码。未修改任何源码、配置、参数、launch、脚本或日志分析程序；未 commit / reset / checkout / clean；未生成补丁；未访问 RRCT；未运行新的大型仿真。
