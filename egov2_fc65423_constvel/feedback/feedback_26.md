# Visibility-First Adaptive Encirclement 运行闭环验证 — 2026-09-10

本轮完成新的 adaptive hypothesis 实际执行闭环。共运行两次 Scenario A ON：第一次用于诊断，针对明确 latency estimator 小问题修复并通过 build/tests 后运行第二次；第二次监测到完整闭环即停止，没有第三次。未改 visibility-first 主算法、120° seed 语义、梯度、binary 定义或 proposal 协议，未降低任何安全阈值。

**安全结论需分开看：** 成功的第二轮没有记录到 clearance 违规、collision、emergency stop、heartbeat loss 或 unvalidated execution；第一轮末尾有5个机间距离低于0.5 m的pair samples，因此本轮总体 SAFETY_REGRESSION_OBSERVED=YES。该既有安全问题未被本轮时序修复解决，不能宣称所有执行分支均已安全。

证据目录：[adaptive_runtime_validation_20260910](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910)。旧 feedback 最大序号25，本报告以独占创建方式写入 feedback_26.md，没有覆盖旧报告。

## 1. 构建与真实运行来源

首先执行 `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status`，相关6包通过。修复后的最终构建同样6包通过、无warning，耗时11.0 s。

manager/coordinator executable，以及 adaptive generator、team optimizer、topology core shared library均使用当前 workspace 的 `-O2` 构建，未使用fast-math。generator是manager链接的库，不是另一个独立ROS节点。

[optimized_manifest_fix.json](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/optimized_manifest_fix.json) 保存第二轮运行前hash和flags；[launch_provenance.json](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/launch_provenance.json) 交叉核对roslaunch实际启动路径、运行前后hash一致以及ldd库路径。实际路径均在本项目 `ros_ws/devel/.private/multi_uav_formation/`，没有旧workspace executable。手工/proc快照取得时manager/coordinator已因成功后停止而退出，不能用该不完整快照充当它们的运行证明，运行证据采用roslaunch自身启动日志与hash。

真实ROS generator latency（mean / P95 / max）：

| 运行 | 延迟 ms |
|---|---|
| 首次诊断 | 5.945 / 6.654 / 6.733 |
| 修复后 | 6.657 / 7.284 / 7.394 |

此前861 ms mean的未优化运行情况未复现。第二轮near horizon=1.5 s、far=3.5 s、far_dt=0.25 s，完整ON参数见两个运行目录的command.json。

## 2. Activation 的诊断与小修复

85 ms来源仍为ACK timeout 60 ms + minimum commit lead 25 ms；此外调度保留25 ms pipeline slack。此前固定240 ms无法覆盖167.348 ms joint加消息准备而仅剩70.135 ms，属于固定耗时假设失效。

本轮开始时源码已经包含动态调度。第一轮两次seed proposal余量568.102 / 564.371 ms，均完成执行，没有ACTIVATION_RESERVE_EXHAUSTED。一个adaptive事件完成joint但camera-time 3.0→3.0、未饱和，正确被ACCEPTANCE_BINARY_CAMERA_TIME拒绝，不能通过降低验收标准绕过。

另一个明确问题：snapshot8/hypothesis2选中后，UAV1 bundle年龄496.552 ms，因PREDICTED_PLANNER_WINDOW_EXPIRED提前拒绝。旧estimator等待8个任意观测，即使已有三次完整joint评估（4.188、8.688、3.329 ms），仍保留437.5 ms冷启动预算。估算年龄+prepare25+joint437.5+ACK60≈1019 ms，超过未改变的planner接受窗口。快速INITIAL_METRICS_INVALID返回却也计入冷启动观测数，不能代表实际求解耗时。

修复只涉及估计器校准与telemetry：

- 三次完整binary before/after评估后，使用最近32次耗时最大值×1.25；20 ms stage预算下限保留。
- 快速invalid返回不作为完整校准观测，但耗时仍进入最大值，因此慢失败仍能抬高预算。无需joint最终accepted才计入完整评估，避免accepted稀少导致无法校准。
- 窗口内完整观测不足3次时恢复冷启动保护。
- 增加complete_observations、observed_joint_ms、next_joint_budget_ms日志。

共同seed/adaptive计算仍为：

```text
B_joint = max(20 ms, 1.25 * recent_max_joint)
          # 不足3个完整观测时同时保留timeout冷启动下限
B_prepare = max(20 ms, 1.25 * recent_max_prepare)
lead = max(240 ms, B_prepare + B_joint + 85 ms + 25 ms)
activation_time = now_before_source_slicing + lead
```

这是基于真实耗时的估计，有冷启动与近期峰值保护；不是保证永不超时。超过950 ms提前拒绝，planner的1 s未来激活上限、stale/identity/generation及所有发布前85 ms检查均保留。不会在优化后平移时间原点。候选生成已经完成，不把上游耗时重复加进remaining pipeline预算。

第二轮真实校准：第1/2/3次完整观测约4.729/6.250/3.349 ms，预算从437.5降至20 ms，计划lead由实际预算公式落到既有240 ms下限。新adaptive proposal仍剩236.258 ms。估计耗时变小不等于安全margin降低；85 ms、额外25 ms slack以及clearance/dynamics/yaw/Local-SFC约束均未改。

修改文件（原始快照与diff在证据目录baseline/diffs）：

1. `ros_ws/src/multi_uav_formation/include/multi_uav_formation/activation_schedule.h`：完整观测计数、三次校准、失败耗时峰值保留。
2. `ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp`：`attemptTeamOptimization()`标记完整评估，增加估计日志。
3. `ros_ws/src/multi_uav_formation/test/team_solution_commit_contract_test.cpp`：实测序列回放、冷启动、慢失败抬高预算及原有生命周期测试。
4. `ros_ws/src/multi_uav_formation/test/adaptive_execution_contract_test.py`：更新observe调用的生产接线断言。

验证脚本另外修正实时rosout去重、记录停止请求时刻、增加机间距离违规停止条件。未修改生产visibility公式或安全执行入口。

## 3. 真正执行的新adaptive事件

身份：encirclement_generation=11，hypothesis_id=1，coordination_generation=15，team_solution_id=8。三机planning_generation=(16,17,16)，source_candidate_id=(171,36,34)，执行trajectory_id=(14,16,14)。CSV uav_id采用1/2/3，以下UAV编号采用ROS 0/1/2。

| 项目 | 值 |
|---|---|
| FINAL_VIEWPOINT_ANGLES | (-75°,45°,75°) |
| DEVIATION_FROM_120_DEG | (-15°,-135°,15°) |
| VIEWPOINT_HYPOTHESIS_COUNT | 3 |
| T_CAM_BINARY_BEFORE / AFTER | 3.600000 / 3.700000 camera·s |
| MEAN_VISIBLE_BEFORE / AFTER | 2.400000 / 2.466667 |
| K2_BEFORE / AFTER | 1.000000 / 1.000000 |
| ALL3_BEFORE / AFTER | 0.400000 / 0.466667 |
| LONGEST_K2_LOSS_BEFORE / AFTER | 0 / 0 s |
| LONGEST_BLACKOUT_BEFORE / AFTER | 0 / 0 s |
| P_CHANGE_NORM | 0.094795842 |
| T_CHANGE_NORM | 0.084779142 |
| YAW_CHANGE_NORM | 0.000762287 |
| generator报告耗时 / generation核心阶段耗时 | 7.152 / 7.089 ms |
| 选中hypothesis candidate规划 UAV0/1/2 | 2.091 / 2.211 / 1.566 ms |
| hypothesis batch规划 UAV0/1/2 | 23.625 / 6.347 / 5.600 ms |
| joint wall latency | 3.603 ms |
| input prepare / message prepare | 0.058 / 0.071 ms |
| coordinator至proposal-ready | 3.732 ms |
| 总pipeline：viewpoint start→proposal send | 207.114 ms，含异步等待 |
| 计划activation lead | 240.000 ms |
| proposal实际remaining lead / required | 236.258 / 85.000 ms |

这些visibility before/after是共同未来1.5 s上的候选/优化轨迹binary预测，不是两条实际执行轨迹的反事实对照。该事件未饱和，预测增加0.1 camera·s（约2.78%）；不能据此声称全场景执行camera-time提高2.78%。generator far评价9.6 camera·s也不能替代joint near评价。

绝对ROS时间证据：

| 阶段 | timestamp |
|---|---:|
| viewpoint start / end | 1789039846.466045618 / 1789039846.473135233 |
| team selection | 1789039846.669412613 |
| joint start / end，binary acceptance返回 | 1789039846.669475555 / 1789039846.673078775 |
| proposal ready / sent | 1789039846.673149824 / 1789039846.673159599 |
| ACK UAV0 / UAV1 / UAV2 | 1789039846.674123445 / 1789039846.674015651 / 1789039846.674100030 |
| commit | 1789039846.674255684 |
| planner adoption UAV0 / UAV1 / UAV2 | 1789039846.681479730 / 1789039846.684517150 / 1789039846.679057815 |
| common activation | 1789039846.909417629 |
| traj_server source执行激活 UAV0 / UAV1 / UAV2 | 1789039846.9153304 / 1789039846.9163568 / 1789039846.911669 |
| 完整三机实际执行CSV例 | 1789039847.0149424 |

planner adoption表示接纳scheduled payload；后续traj_server source激活和OPTIMIZED_YAW_EXECUTED日志证明实际切换，二者不能混淆。三机执行记录均source=JOINT_TEAM_SOLUTION、team_solution_id=8、hypothesis=1、encirclement_generation=11、safety_validated=True。完整CSV中该event有73个单机执行samples。

独立selector CPU耗时、binary acceptance内部单独耗时未单独计时，N/A；已记录selector时刻，binary nonlinear重评估包含在joint 3.603 ms内，不能把它们写成0。详见[event_details.json](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/event_details.json)、[adaptive_execution_event.log](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/scenario_a_on_fix/adaptive_execution_event.log)、[event_executed_rows.json](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/scenario_a_on_fix/event_executed_rows.json)。

## 4. 安全与执行可见性

两个运行完整CSV左端点积分，包含停止期间记录，不删除warm-up；不同长度，不能作OFF/ON或性能提升对照。

| 指标 | 第一轮诊断 | 第二轮成功短测 |
|---|---:|---:|
| 记录积分时长 s | 31.071076 | 8.267242 |
| executed camera-time camera·s | 79.011289 | 23.935486 |
| mean_visible | 2.542921 | 2.895220 |
| K2 / All3 | 0.974306 / 0.568615 | 1.000000 / 0.895220 |
| longest K2 loss / blackout s | 0.798340 / 0 | 0 / 0 |
| min swarm欧氏 m | 0.485185 | 0.873648 |
| min swarm椭球 m（阈值0.5） | 0.482227 | 0.873647 |
| min static表面clearance m | 0.300909 | 0.385164 |
| min dynamic表面clearance m | 8.238591 | 27.922290 |
| collision / swarm违规pair samples | 0 / 5 | 0 / 0 |
| emergency stops / heartbeat-stale events | 0 / 0 | 0 / 0 |
| unvalidated executed samples | 0 | 0 |

第二轮heartbeat最大记录周期11.346 ms。其source samples为NOMINAL484、JOINT_TEAM_SOLUTION199、PERSISTENCE_FALLBACK67。

成功event的完整nonlinear预测验证：min swarm 0.813561 m，static clearance0.455995 m，dynamic clearance24.492554 m；max v/a/jerk为2.031731/3.475838/13.137608，低于3/6/22限制。yaw/Local-SFC/generation等原验证契约保留并通过该solution验收；未另做所有实际轨迹连续时间v/a/jerk/yaw峰值重建，不能用CSV或safety_validated字段替代这种证明。

第一轮违规pair为UAV0 NOMINAL trajectory18与UAV1 PERSISTENCE_FALLBACK trajectory18，team_solution_id均0、generation18、encirclement_generation22、hypothesis0。时间30.584379–30.751713 s，最低椭球0.482227 m。首样本timestamp1789039593.554103，比roslaunch日志ProcessMonitor.shutdown（19:26:33.558）早约3.9 ms。因此不能把全部违规归因于关闭；后几帧存在重复位置，也不能把它们全视为持续更新的独立状态。保留原始证据，不宣称该问题已修复或必然由joint/adaptive造成。详见[swarm_violation_evidence.json](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/swarm_violation_evidence.json)。

源代码安全契约检查显示normal/joint/fallback仍经过原nonlinear gateway，失败候选不能借retiming/fallback重新提交；但这不保证异步多机实际执行永远满足距离。第一轮记录说明需要后续专门核对同时执行轨迹、相互预测版本及停止顺序，当前日志不足以唯一定位到可安全修复的一处生产入口。

## 5. Tests、停止与剩余事项

构建PASS；6/6 CTests PASS（含joint P/tau/yaw、visibility/diversity有限差分、slow activation、stale identity/late ACK契约）。另有Elastic visibility、time-only feasibility、time-only swarm temporal、Local-SFC boundary、adaptive execution Python10项及tracking visibility测试通过，共12套。日志见build_fix.log、ctest_fix.log、ctest_fix_details.log、ros_test_results.json及各test日志。

新增校准fixture回放第一轮3个完整耗时和8个快速失败，保证快速失败不解除冷启动、3个完整评估后能在原窗口安排activation，600 ms慢失败重新抬高预算；原310 ms慢pipeline、超窗拒绝和85 ms检查继续通过。接线测试首次因observe函数签名增加参数而失败，更新为检查完整观测条件后通过，未删除检查。

第二轮约7.602 s mission已取得闭环，监测器随即请求停止；完整CSV约8.267 s含退出阶段，wall28.706 s含清理。launcher退出120来自SIGINT/tee关闭链；原始roslaunch退出日志保留。最终进程核查无ROS/仿真残留，见[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/adaptive_runtime_validation_20260910/final_verification.json)。不再运行第三次。

剩余问题：第一轮机间距离不足尚未唯一定位；候选覆盖不足、INITIAL_METRICS_INVALID，以及第二轮3个TRAJECTORY_BOUNDARY_MISMATCH和1个STALE_PLANNING_GENERATION拒绝仍有记录，均未通过放宽验证绕过。成功event8身份匹配并完整执行；不能把其他reject全部当成协议缺陷，也不宣称长期稳定性已验证。本轮未改上述主算法/安全问题以追求更多成功样本。

```text
CORRECT_OPTIMIZED_RUNTIME_CONFIRMED: YES
ADAPTIVE_GENERATOR_RUNTIME_LATENCY_OK: YES
ACTIVATION_RESERVE_PROBLEM_FIXED: YES
ACTIVATION_MARGIN_REDUCED: NO
ADAPTIVE_VIEWPOINT_GENERATED: YES
ADAPTIVE_CANDIDATE_SELECTED: YES
JOINT_PT_YAW_SUCCESS: YES
BINARY_CAMERA_TIME_ACCEPTED: YES
ADAPTIVE_PROPOSAL_SENT: YES
THREE_ACK_COMMIT: YES
THREE_UAV_ADOPTION: YES
OPTIMIZED_YAW_EXECUTED: YES
NEW_ADAPTIVE_JOINT_EXECUTION_CHAIN_VERIFIED: YES
SAFETY_REGRESSION_OBSERVED: YES
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES
SMALL_FIXES_APPLIED:
- 三次完整joint评估校准近期最大耗时；快速失败不计完整观测，慢失败仍增加预算
- activation estimator观测telemetry及回归契约
- 验证监测rosout去重、机间距离停止条件、停止时刻记录
REMAINING_ROOT_CAUSE:
- 第一轮NOMINAL/PERSISTENCE机间距离低于0.5 m；接近shutdown但不能完全归因于shutdown，生产根因未唯一定位
- 零星候选覆盖/metrics、boundary及stale拒绝保留
REPORT_FILE:
/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_26.md
ON_RUNS_THIS_AGENT: 2
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
