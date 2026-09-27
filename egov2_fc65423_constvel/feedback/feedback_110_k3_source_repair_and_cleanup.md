# Feedback 110 — K3 源码修复、单次 FULL ON 与磁盘清理

## 执行顺序与边界

先核对 K3 forecast → contract → schedule → Planner realization → T/PT-SCP → RealizedTeamValidator → COMMIT 的相关源码及 Feedback 109，再集中修改、执行 `catkin build -j2 --no-status` 和 `git diff --check`，最后仅运行一次 FULL / K3 ON。没有为本轮再运行 OFF，也没有修改 RRCT、M2 门槛、SCP trust、required margin、validator 非退化条件或 TeamTargetCenteredOptimizer 主目标。

源码核对纠正了两项先前推断：K3 由未来 forecast 网格选窗，本 run 的 33 个 event 从检测到 `t_cross` 的提前量为 0.573–1.954 s，中位数 0.801 s；`TRUE_CONSTRAINT_INFEASIBILITY` 只证明当前线性化 QP 的硬行在移除 P/T trust 后仍不可行，不证明精确非线性几何或整个 topology 永久不可行。

## 集中修复

1. Team SIDE 重建原先把候选的局部引导窗扩成归一化全程 `1.0`，可以在固定 MINCO 端点建立无梯度且上界为负的硬行。K3 现在沿用 Local SIDE 的实际冲突进度、方向、偏移、归一化窗口宽度；这些值在 Local 生成候选时记录，由同一 `CandidateSideScope` 和最终 SIDE certificate 使用。M2 和普通 Team realization 的 SIDE 行为未改。保留了 K3 硬行来源日志，未增加生产 QP 或完整 visibility evaluator 到 Local L-BFGS。
2. K3 合同只有在当前滚动 activation 仍被旧窗口覆盖、且最新同目标风险窗仍包含于旧合同的容差范围内才复用；旧窗口越界会清除并由本轮预测建立新合同。此判断只作用于 K3，不改 M2 合同。
3. K3 schedule 的 `handoff_critical_world_time` 改为 K3 `t_star`；M2 继续使用 `m2_min_time`。本 run 的 14 次 K3 Planner recheck 全部与对应合同 `t_star` 一致。
4. Local hard preflight 的短路检查补全明确失败原因，原有动力学、静态、动态、swarm、Local-SFC 和最终执行检查保持原序与原阈值。

## 编译与正式运行

`catkin build -j2 --no-status`：25/25 成功；`git diff --check`：通过。运行命令：

```bash
./run_on.sh --ablation full --headless --timeout 240 --boot-timeout 180 \
  --scenario /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json \
  --k3-repair on
```

RUN_ID `20260925_021921_128945`：BOOT-12 成功、`FINAL_EXIT_CODE=0`；runner 登记的 29 个自有进程全部退出，强杀和误杀其他进程均为 0。参数证明 K3 enabled、FULL 模式，日志位于持久磁盘。证据：[exit](../runs/20260925_021921_128945/exit_status.txt)、[params](../runs/20260925_021921_128945/resolved_params.txt)、[cleanup](../runs/20260925_021921_128945/process_status.txt)、[K3 事件摘录](../runs/20260925_021921_128945/k3_event_evidence.log)、[完整执行指标](../runs/20260925_021921_128945/native_metrics_full.json)。完整 ROS 原始日志已压缩保留在同一 run 的 `ros_log/`。

| 指标 | 本次 FULL ON |
|---|---:|
| K3 event / K3 target SCP start | 33 / 15 |
| K3 PT solver PASS / validator 调用 | 1 / 1 |
| K3 validator PASS / adopted / activated | 0 / 0 / 0 |
| K3 合同刷新：窗口变化 / 风险或目标变化 / M2 抢占 | 20 / 10 / 3 |
| K3 首轮硬行诊断中无梯度负上界行 | 0（12 次诊断） |
| executed K2 / All3 | 0.989353 / 0.916951 |
| 最长 K2 loss / All3 loss | 0.833795 s / 1.294287 s |
| 最小静态 / 动态净空 | 0.235611 / 0.310994 m |
| 静态接触 / 动态接触 / swarm violation / 未验证执行样本 | 0 / 0 / 0 / 0 |
| MIN_BUDGET_APPLIED / terminal hold / successor starvation | 0 / 0 / 0 |

唯一的 K3 PT PASS 对 contract 5、UAV0 的精确非线性 margin 达到 `0.217022`，P 与 T 均有非零修改。最终 validator 在同一 common horizon 计算得到：临界 K3 `0→1`、全时域 K3 `41→50`、逐样本 K2 回退 `0`；这三项 K3 专用条件都通过。原有连续 Q2 却从 `0.872842` 降为 `0.872723`，最终以 `PRIMARY_VISIBILITY_WORSENED` 拒绝。该 proposal 没有 PREPARE/COMMIT/activation。故本次**没有可归因于 K3 repair 的执行增益**，不能把运行级 All3 数值归功于 K3。

Feedback 109 的主配对 ON 为 executed K2 `0.988931`、All3 `0.918689`，本次 K2 略高而 All3 略低；两次都没有 K3 adoption，场景运行本身有波动，这不是因果 OFF/ON 对照。本次 hard safety 与 Local 活性事件为 0，但单次运行不能证明统计意义上的改善。

## 清理与剩余阻塞

停止旧仿真后，删除了本轮前期产生的 6 个未归档诊断 run、一个未被反馈引用的 K3 诊断 run，以及 11 个未被报告或场景引用的历史重复 run；保留 Feedback 100–109 引用的原始证据和场景元数据引用的 run。清空 `.ros_home/log`、旧 catkin 日志和 `__pycache__`。依工作区 `CLEANUP_POLICY.md` 保留有效的 `ros_ws/build`、`ros_ws/devel` 以避免全量重编译。本次 run 的重复控制台日志已删除，完整 ROS 日志已压缩到约 30 MB；run 目录约 37 MB。可用空间从清理前约 0.6 GB 回升至约 8.2 GB。

当前主要阻塞已不是 K3 接线：虽然一条候选满足 K3 binary 增益与 K2 底座，现有连续 Q2 非退化仍可拒绝它；其余多数 trial 的联合硬行仍不可行。保持原 validator 与 SCP 边界时，本次源码修复没有实现 K3 adopted/activated。下一步若继续追求 K3 采纳，需要先从 exact joint Q2 与各硬行来源设计一个符合原约束的候选生成办法，不能以放宽 Q2、trust 或 required margin 伪造成功。
