# Feedback 135 — run9 双机静默停摆只读归因（未修复）

日期：2026-09-27。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。目标：只查找 run9（20260927_060327_1111508）中 drone1/drone2 planner 静默停摆（K2 loss 13.57s、ALL3 69.2%）的原因。**未修改任何 production 代码，未运行仿真，未访问 RRCT。**（feedback_134 已被并行的 topology 因果调查占用，故本报告顺延为 135。）

## 0. 一句话结论

**两台 planner 进程在 gen-192 提交完成后、下一个 FSM tick 开始前（sim 38.535/38.562，相差 27ms）整体冻结：所有线程最终阻塞在 stdout 写上（exec 线程阻塞在 tick 末尾 cout 冲刷、100Hz 心跳线程的审计日志在 ~0.4s 内填满自己的 4KB libc 缓冲后阻塞在 write(2)）——因为它们各自的 roslaunch 输出管道停止了排空。进程活着但停算：心跳停止 → executor 判 heartbeat-stale → 保险性地飞完已承诺的 traj 192 到多项式末端 → TERMINAL_HOLD → uav2/uav3 停车 → 几何上 2/3 机出视场 63.0-76.5s → K2 洞 13.57s。**

## 1. 证据链（每条都有日志/CSV 工件）

1. **冻结点在批间而非批内**：drone1/drone2 的最后一批完整走完（`[candidate-batch-activation]`→`[FROZEN_CANDIDATE]`→`[COMMIT_SAME_REVISION]`→`[topology-bundle]`→`[execution-planning-budget]` 全在场，268.5349/268.5619），FSM 迁移 cout `[MOVING_ROLLING]: from REPLAN_TRAJ to EXEC_TRAJ` 亦已打印；此后 planner 侧任何日志再未出现。
2. **不是"卡在等待激活分支"**：`EGO drone N waits for coordinated trajectory activation` 节流日志的末次出现为 38.29/38.32（traj 191 激活前），冻结后没有新的 waits——若是活着的 FSM 卡在该分支，0.5s 节流日志会持续出现。
3. **心跳停止由独立证据钉死**：`[traj-server-heartbeat-stale] node=/drone_1_traj_server / drone_2_traj_server`（269.144，stale 阈值 0.5s）→ 最后心跳 ≈268.6，即冻结后 ~0.1-0.6s。心跳回调（heartbeatTimerCallback）每次调用都执行多个 `reportXxx` 审计日志 + 周期性 `[planner-heartbeat]` ROS_INFO——专用 spinner 线程自己也写 stdout，其 100Hz 日志 ≈10KB/s，4KB libc 缓冲 ≈0.4s 填满后阻塞——与 stale 时刻精确吻合。
4. **executor 完全无辜且行为正确**：`remaining_duration=1.555` 与 traj 192 证书末端（270.699−269.144）**精确吻合** → traj_server 确实激活了 192 并飞到多项式末端；CSV 显示 uav2/uav3 在 planner 静默后仍以 ~1m/s 平滑飞行至 CSV 57-58 减速停车（=traj 192 末端，start 268.669+4.10s≈272.77），随后 `TERMINAL_HOLD`。
5. **coordinator 无辜**：它接收了两机 gen-192 bundle（`[topology-bundle-recv] drone=1 generation=192` 268.5351）并在 268.57-268.58 正常评估了含 252/300（两机 gen-192 候选）的元组（`[TEAM_TARGET_CENTERED_OPT] candidate_ids=400,252,300 / 402,252,300`）；是 planner 冻结导致其后续 ACK/pend 超时处理永不发生。
6. **team pend 永不闭合是冻结的结果而非原因**：gen-192 的 `TEAM_PENDING_START`（mode=EARLY_JOINT_PRIMARY，全 run 903 次 pend 全是该正常模式）的 0.65s JOINT_TIMEOUT 应在 269.18 由 planner tick 执行——269.14 时日志管线已被 traj_server 的 WARN 证明通畅，planner 却无输出 → planner 已冻结。
7. **进程活着（非崩溃）**：无 `process has died` 记录；收尾 `CLEANUP_OWNED_PROCESS_COUNT=27 / FORCE_KILLED=0 / 剩余 0`——全部进程在收尾时存在并对 SIGINT 优雅退出。阻塞在 write(2) 的线程被 SIGINT 以 EINTR 打断后可优雅退出，与观察一致。
8. **stdout 管道异常的直接痕迹**：同一窗口出现行撕裂与乱序——`eration=191 ...`（行首丢失）、`planning_epoch=1790460268.5 predicted_delta...`（数值中途截断）、且**文件尾部本身**就存在 drone0 traj_server 的迟冲刷残行（ts=.227 的残行出现在 .96x 行之后）——stdout 链路的滞后/撕裂在 run9 全程系统性存在，268.53 只是对两条 planner 管道成为永久事件。
9. **节点名行的最后出现时刻**：`/drone_1_ego_planner_node` 最后带节点名的日志 35.69（`[planner-heartbeat]` 诊断 ~25s 周期，下一次 ~60.7 永未出现）；`/drone_1_traj_server` 的同类行持续到 220.38（运行结束）——planner 死、executor 活，分属两个进程、两条独立 stdout 管道。

## 2. 因果链（完整）

stdout 管道停排空（roslaunch per-node 泵停转/死亡，触发点见 §4）
→ planner 所有线程逐一阻塞在 stdout 写（每个动作都先写日志；专用心跳线程的审计日志 ~0.4s 填满缓冲）
→ planner 整体停算：不再 replan、不再导出 bundle、pend 超时永不执行、心跳停止
→ traj_server 判 stale → `CONTINUE_ACTIVE_TRAJECTORY` → 飞完 traj 192 → `TERMINAL_HOLD`
→ uav2/uav3 定格（CSV ~58 起），目标与 uav1 继续前进
→ 63.0s 起几何上 2/3 机不可见（K2 loss 13.57s、ALL3 69.2%）
→ 直至收尾。

## 3. 排除清单（逐项有证据）

- **非 coordinator**：正常接收并评估 gen-192 元组（证据 5）。
- **非 executor/traj_server**：激活并飞完 192，行为完全正确（证据 4）。
- **非场景/预测器/时钟/磁盘**：它们的日志连续、时间戳全程单调；df 计账显示 run9 全程剩余 ≥1.5G（无 ENOSPC）。
- **非崩溃/段错误**：证据 7。
- **非本轮（feedback_131-133）任何代码修改**：冻结点在批间 tick，批内代码（含全部新增路径）完整执行并收尾；批间 tick 不执行任何本轮新增代码；解析 jerk 路径为有界闭式解（quartic→solveQuart，Sturm 无界循环只对次数≥5 可达，jerk 的 |j|² 导数是三次）且带 NaN 防护。
- **非 K3/LOS/selector**：冻结与 LOS 无关；纯粹是进程级停算。

## 4. 未决点（诚实声明）

**stdout 泵停转/死亡的直接触发不可从现存证据恢复**：能钉死它的每节点 ROS 日志文件（rosconsole 独立于 stdout 的文件后端）与 roslaunch 内部状态，已在本会话磁盘紧张时的清理中删除；roslaunch 自身不记录泵线程异常。已知的候选触发（按可能性）：roslaunch（Python）泵线程异常死亡（其日志写/屏幕写遇一次性 OSError）、共享下游（roslaunch→runner 的 stdout 路径）的一次未恢复的停滞恰逢两条 planner 管道处于同步 epoch 的日志突发峰值（~25KB/6ms，管道最满时刻）。"为什么恰好这两条、恰好同时"由机队同步 epoch 解释：冻结激活契约使两机在同一 27ms 窗口做同样的事、处于同样的满管状态。

## 5. 后续取证与修复建议（未实施，等待授权）

1. **取证类**：完整保留每节点 ROS 日志（ros_log/ 不删）；在 runner 中记录 roslaunch 泵线程存活状态；复现实验——人为让一条节点的 stdout 停排空（模拟泵死亡），验证 planner 是否呈现完全相同的停摆指纹（心跳 0.4-0.6s 内停、executor stale、飞完承诺轨迹、terminal hold）。
2. **修复类（若授权）**：节点侧——stdout 阻塞看门狗（写超时后降级丢弃日志而非阻塞）、降低日志体积（突发峰值 25KB/6ms 是主因）、rosconsole 输出改非阻塞 fd；调度侧——心跳线程与日志解耦（心跳回调不写 stdout）；runner 侧——健壮的 roslaunch stdout 读取与背压隔离。
3. 上述任何一项实施前，本轮保持只读。

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

INVESTIGATION_TARGET: run9 (20260927_060327_1111508) dual-planner silent stall
FREEZE_INSTANT: sim 38.535 (drone1) / 38.562 (drone2), between gen-192 commit and next FSM tick
MECHANISM: stdout pipe stall -> all planner threads block on stdout writes -> whole-process halt -> heartbeat stops (~0.5s) -> executor heartbeat-stale -> fly committed traj 192 to end -> TERMINAL_HOLD -> K2 hole 63.0-76.5s
EVIDENCE_KEYS: batch-complete+budget-line; waits-absent; heartbeat-stale WARN remaining=1.555 exact cert match; CSV freeze profile = traj 192 polynomial end; coordinator evaluated gen-192 tuples; pend timeout never ran though pipeline proven alive at 269.14; no crash records; graceful cleanup; line-tearing + late-flush fragments in same window and at file tail
EXCLUDED: coordinator, executor, scene/predictor/clock/disk, crashes, this-session code changes (freeze is between batches; exact-jerk paths bounded closed-form)
UNRESOLVED: direct trigger of the roslaunch per-node pump stall/death (per-node logs deleted; roslaunch internal state unlogged)
NEXT_STEP: 取证复现实验 + 修复方案已列出，等待授权

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[run9 stdout](../runs/20260927_060327_1111508/roslaunch_stdout.log.gz)、[run9 退出状态](../runs/20260927_060327_1111508/process_status.txt)、[run9 visibility CSV](../runs/20260927_060327_1111508/visibility.csv)、[run9 轨迹 CSV](../runs/20260927_060327_1111508/visibility_trajectory.csv)。
