本轮已实现动态 activation scheduling 并通过构建/测试；唯一一次 Scenario A ON 短测未进入 joint optimizer，因此未验证 adaptive proposal→三机 ACK→commit→adoption→optimized yaw execution。没有第二次仿真；全部 ROS/仿真进程已停止。

本轮仅修改时序预算、阶段 telemetry、同快照计算复用与测试。没有修改 adaptive 主公式、J_acc/K2/blackout、selector 排序、120°语义或安全阈值；原有工作树修改保留，未提交 Git commit。

**85 ms 的真实来源与根因。** 原 `attemptTeamOptimization()` 在候选生成和 selector 之后，使用 `now + team_activation_lead`，默认 lead=240 ms。joint 返回后要求剩余 `team_ack_timeout(60 ms) + minimum_commit_lead(25 ms) = 85 ms`。上次 adaptive solve=167.348 ms，剩余=70.135 ms，故本阶段总消耗169.865 ms，其中约2.517 ms为求解外开销；最多允许的155 ms已经不足。问题是240 ms没有随计算延迟调整，不能通过降低85 ms解决。viewpoint/candidate 的上游耗时影响候选年龄和是否仍可执行，但并不直接从旧代码这个在 joint 前新建的240 ms里扣除。

其他独立约束保持不变：planner 收 proposal 的最小 lead=15 ms、最大未来激活=1 s；commit 仍要求25 ms；traj_server 10 ms命令周期按共同 activation 切换；`topology_min_active_remaining=0.35 s` 是本地候选 staging 时已有轨迹的剩余时长要求；team solution max age=0.65 s；planner 原有等待预算在此次参数下为1.0 s。它们均不是85 ms的来源，未放宽这些 age/generation/identity/执行限制。

**新的 activation 计算。** 所有 seed/adaptive 使用同一函数。在候选生成、三机 bundle 汇集和 selector 已完成后、source trajectory 切片与 nonlinear evaluation 之前确定唯一共同时间原点：

```text
B_stage = max(20 ms, 1.25 * max_recent(stage_wall_latency))
required_margin = ACK_timeout + 25 ms                 # 仍为85 ms
lead = max(configured_minimum_lead,
           B_prepare + B_joint + required_margin + 25 ms)
activation = now_at_joint_preparation + lead
```

每阶段保留最近32条有效观测，前8条不足时保留冷启动预算；joint 冷启动取已配置 optimizer timeout，本次350 ms，prepare 冷启动20 ms。对应初始计划 lead=572.5 ms；这是配置推导值，本次运行未进入 SCHEDULE，并非一次实测 proposal lead。适应后仍保留240 ms作为下限，但不依赖它足够覆盖慢计算。

上游 viewpoint/candidate/排队时间在设置 activation 时已经过去，不重复加进剩余预算；它们由阶段日志记录，且 candidate 源 suffix 与 bundle freshness 继续验证。准备和求解使用墙钟耗时，activation 使用ROS世界时间，与已有目标/障碍预测一致。

如果估计 lead 超过950 ms，提前拒绝，不把它压缩到 planner 的1 s上限。预测 bundle 会过期则在昂贵工作前拒绝。joint 前再次检查预算；joint 后、payload构造后、实际 publish 前分别检查原85 ms余量。不足时 fail closed，下一次新快照依据新测量安排。不会在优化后平移 activation 来复用已经失效的 P/T/yaw、LOS/FOV 或 safety validation。team_solution_id、coordination/planning/encirclement generation、hypothesis、三机共同 activation 和 ACK payload 检查保持原链。

**低风险计算复用与文件。** 新增 `multi_uav_formation/include/multi_uav_formation/activation_schedule.h`；coordinator 的 `attemptTeamOptimization()` 接入估计、前置/发布前检查和时间戳。`adaptive_viewpoint_generator.h/.cpp` 增加可选 current evaluation 输出，在同一次 generate 的精确输入缓存中复用当前 viewpoint，避免 manager 再算一次相同 LOS。manager 记录 viewpoint start/end；FSM 记录每 hypothesis 和 batch 的 start/end、candidate IDs，并以 move 收集已完成候选集。coordinator 仅缓存同一次同步优化中相同 world-time 的 target prediction；不缓存 observer、yaw、FOV、LOS 或任何随 P/T/yaw 变化的结果。

本轮修改8个现有文件、新增1个头文件。完整路径见 `changed_files.json`，改造前快照在 `baseline/`，差异在 `diffs/`。测试修改为 `team_solution_commit_contract_test.cpp`、`adaptive_viewpoint_generator_contract_test.cpp`、`adaptive_execution_contract_test.py`。

阶段日志用 generation/hypothesis/candidate IDs 关联，包含 viewpoint、candidate、TEAM_SELECTION、SCHEDULE、joint start/end、binary acceptance、PROPOSAL_READY、PROPOSAL_SENT。`t_binary_acceptance` 是 optimizer 返回并完成其内部最终验收时的记录点。joint前准备和消息准备分别统计。完整端到端耗时由同一 hypothesis 的时间戳计算，不能拿仅 coordinator 耗时冒充 T_total。

**构建与测试。** `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status` 相关6包成功，无编译警告，最终构建12.6 s。6/6 CTest通过，另有 Elastic、time-only feasibility、time-only swarm temporal、Local-SFC boundary、adaptive execution Python（10项）、tracking visibility，共12套通过。P/tau/yaw、piece boundary、visibility/diversity有限差分及原安全契约继续通过。

新增慢流水线测试：joint=310 ms、prepare=35 ms，已超过旧240 ms；估计安排lead=591.25 ms，proposal剩余246.25 ms≥85 ms。验证负载尖峰增加预算、超出激活窗口拒绝、seed/adaptive共用契约、过近activation拒绝、旧ACK身份/代数/迟到拒绝，以及生产调用顺序。缓存结果与独立评估相等。测试 master 已停止。

测试阶段曾因沙箱禁止socket而中断；用户开放权限后，隔离ROS master测试通过。曾使用本机旧CTest不支持的 `--test-dir`，该次没有发现测试，不计为PASS；随后在正确build目录执行6/6 PASS。详见 `ctest.log`、`ctest_details.log`、`ros_test_results.json` 及其余测试日志。

**唯一一次 Scenario A ON 及失败根因。** 场景 `long_cylinder_forest_visibility_stress.json`，SHA256 `430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`，保持上次FULL_ON算法参数。到30 s短测mission期限停止；CSV完整积分窗口31.13334 s（含关闭期间继续记录），wall总时长51.925 s含清理。launcher SIGINT/tee清理退出120；主要C++节点日志确认SIGINT后返回0，最终无残留。

23次adaptive触发，81次安全alternative candidate生成日志；这81次不等于相对当前camera-time严格提升。selector成功0、joint attempts=0、SCHEDULE=0、proposal/ACK/commit/adoption/optimized yaw execution均0。未再次出现activation reserve exhaustion，但原因是未进入该路径，不能当作修复已在仿真通过。

记录到3次 `CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE`：selector在枚举组合前要求 current/local candidate 的有效世界时间末端覆盖共同 `evaluation_start+1.5 s`。这是一项预测时间区间覆盖检查，不是K2 visibility阈值。比如已有27个潜在可执行组合的planning generation=(4,4,4)仍被这个前置条件拒绝，adaptive组合没有得到排名。上游已有1 s coordinator timeout后local fallback，使部分planning-to-commit median约507 ms；共同评估起点异步前移。早期还出现INNER bundle因 `PRESERVE_PENDING_OUTER_SNAPSHOT` 未被收纳。日志没有保存所有candidate完整payload，不能声称已从日志精确重建每一个不足的末端值或把所有上游现象归因到某一行。

随后15次 `NO_EXECUTABLE_COMBINATION`：至少一机没有满足原hard safety/有效候选契约的输入。首个明确例子是UAV1 planning generation6、candidate17：`dynamics_valid=0, static_valid=1, safety_class=INVALID`，bundle `executable_candidate_count=0`；实际转入经过重新验证的previous-safe persistence。没有通过放宽dynamics或selector绕过它。这个拒绝发生在新activation调度之前；本轮按限定范围记录根因，不扩展修改selector或候选安全逻辑。

选取encirclement generation5、hypothesis1作为一次可关联的非seed案例：

| 事件 | ROS世界时间/值 |
|---|---|
| viewpoint start/end | 1789035750.276697397 / 1789035750.282314539 |
| T_VIEWPOINT_MS | 5.617 |
| FINAL_VIEWPOINT_ANGLES | (-150.461°,180°,150.461°) |
| DEVIATION_FROM_120_DEG | (-90.461°,0°,90.461°) |
| generator far T_cam | 10.5 camera·s，不能替代candidate/joint验收 |
| UAV0 candidate12 start/end | 1789035751.036629915 / 1789035751.039702892；3.073 ms |
| UAV1 candidate12 start/end | 1789035750.875307798 / 1789035750.878082037；2.774 ms |
| UAV2 candidate11 start/end | 1789035750.984743357 / 1789035750.985993624；1.250 ms |
| 三机batch规划耗时 UAV0/1/2 | 11.067 / 8.269 / 3.996 ms |
| selector拒绝记录时间 | 1789035751.0465546；CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE |
| viewpoint开始至selector拒绝 | 769.857 ms，包含上游等待，不是CPU求解时间 |
| joint start/end、binary acceptance、proposal-ready、activation | N/A，未到达 |
| T_JOINT / T_MESSAGE_PREPARE / T_TOTAL_BEFORE_PROPOSAL | N/A，不能把未发生的阶段写成实测0 ms |
| PLANNED/ACTUAL_PROPOSAL_REMAINING_LEAD | N/A；REQUIRED_ACTIVATION_MARGIN配置=85 ms |
| T_cam、mean_visible、K2、All3 BEFORE/AFTER | N/A，无joint验收样本 |
| P_CHANGE_NORM / T_CHANGE_NORM / YAW_CHANGE_NORM | N/A |

**实际执行与性能。** 全记录左端点积分；没有OFF对照，也没有删除warm-up。

| 指标 | 结果 |
|---|---:|
| accumulated camera-time / mean_visible | 50.770027 camera·s / 1.630729 |
| K2 / All3 | 0.392940 / 0.248396 |
| longest K2 loss / blackout | 18.899810 / 0.229617 s |
| min swarm欧氏/椭球距离 | 0.822602 / 0.821075 m；椭球阈值仍0.5 m |
| min static/dynamic表面clearance | 0.314581 / 8.622459 m |
| collision / swarm违规pair samples | 0 / 0 |
| emergency stops / heartbeat-stale事件 | 0 / 0 |
| heartbeat最大记录周期 | 12.064 ms |
| 未验证执行samples | 0 |
| sources NOMINAL / PERSISTENCE_FALLBACK / SIDE_MINUS | 1748 / 1056 / 1 samples |
| generator latency mean/p95/max | 5.575 / 5.943 / 6.784 ms |
| 每次hypothesis batch planning mean/p95/max | 9.179 / 15.967 / 42.664 ms |
| joint latency | N/A |

短测没有观察到记录范围内的安全异常，但可见性和continuity很差，不宣称整体行为改善。SAFETY_REGRESSION_OBSERVED:NO仅指本次安全事件与余量观测，不是和OFF统计比较，也不证明所有fallback分支安全问题已经消除。

**确认正确优化编译。** `optimized_build_manifest.json` 保存启动前两个运行二进制SHA256及manager/coordinator/三个数值库的 `-O2` flags。唯一运行的roslaunch日志记录的实际启动路径与这些二进制一致，运行前后hash一致；没有fast-math，源码hash运行前后一致。由于启动行没有出现在合并stdout里，monitor的 `/proc` 快照字典为空；最终使用roslaunch自身启动命令记录交叉验证，见 `scenario_a_on/launch_build_evidence.json`。没有把这个空字典当作运行证据。

分析采用去重后的唯一rosout日志，避免 `ros_logs/latest` 链接与launcher重复计数；monitor即时计数因此可能较高，最终以 `scenario_a_on/analysis.json` 为准。最终检查见 `final_verification.json`。本轮没有访问或修改禁止目录。

```text
ACTIVATION_LIFECYCLE_ROOT_CAUSE_CONFIRMED: YES
DYNAMIC_ACTIVATION_SCHEDULING_IMPLEMENTED: YES
ACTIVATION_MARGIN_REDUCED: NO
ADAPTIVE_PROPOSAL_SENT: NO
THREE_ACK_COMMIT: NO
THREE_UAV_ADOPTION: NO
OPTIMIZED_YAW_EXECUTED: NO
NEW_ADAPTIVE_JOINT_EXECUTION_CHAIN_VERIFIED: NO

CORRECT_OPTIMIZED_BUILD_USED_IN_SIMULATION: YES

SAFETY_REGRESSION_OBSERVED: NO
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES

ON_RUNS_THIS_AGENT: 1
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

证据目录：[activation_latency_validation_20260910](/home/bob/ALP/egov2_fc65423_constvel/activation_latency_validation_20260910)。本报告中未展开的相对证据路径均相对于该目录。
