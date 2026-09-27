ALP trajectory lifecycle 只读审计 — 2026-09-10

**结论：用户看到的停顿具有真实 execution 证据，主因是候选到接管之间的生命周期阻塞。** 候选生成很快，但 coordinator pending 将普通重规划阻塞约 1 秒；原始 head 随后过时，被拒绝后再次等待。失败期间保留的有限轨迹逐渐跑到末端，traj_server 真正进入零速度/零加速度 terminal hold，之后新轨迹才接管。本轮未修改算法、参数、生产代码，未 build、未运行仿真。

**1. 数据和计数口径**

数据来自 `/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910` 的 canonical_rosout.log、launcher.log、trajectory.csv、visibility.csv、run_meta.json 和上一审计附件。统一有效区间为 mission **0.051156–80.204541 s**，长度 **80.153385 s**；绝对时钟起点为 1789041219.420578957。UAV0/1/2 对应 CSV uav_id=1/2/3。保留区间前已经激活的第一条轨迹，供首个间隔计算；频率只统计区间内的新事件。

“实际 activation”以 traj-server-receive 或 traj-server-scheduled-activate 为准，对相邻相同 `(UAV, trajectory_id, start_time)` 去重；scheduled 使用 actual_time。source 标签变化不算新 polynomial。日志中的 PUBLISHED、ACTIVATED 也不能直接按行数当成新轨迹。没有完整 polynomial payload/PositionCommand P/V/A 时序，不能恢复完整未来曲线；本报告用源码、明确的 head_error 日志及事件时间补充证据，不以 odom 差分冒充 polynomial 导数。

区间内 **105** 次真正新 activation，对应 **105** 次 planner commit；另有 **234** 次同 id/start 的重复接收。source 标签转入 persistence 的 **19** 次均不是新 polynomial。整条链的已激活轨迹（含区间前 3 条初始化）共 108 条，见 [activated_trajectory_lifecycle.csv](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_audit_20260910/activated_trajectory_lifecycle.csv)。
所有可观测阶段和原日志行号见 [lifecycle_timeline.csv](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_audit_20260910/lifecycle_timeline.csv)，另有 uav0/1/2_timeline.csv；实际相邻接管完整数据见 [activation_intervals.json](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_audit_20260910/activation_intervals.json)，pending generation 对应关系见 [pending_generations.json](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_audit_20260910/pending_generations.json)。

replan trigger 的 stdout 状态跳转没有 UAV 标识和可靠时间戳，且多进程输出交错，因此精确的逐 UAV FSM trigger 次数为 **N/A**。下表 PLANNER_REPLAN_RATE 使用可定位的 candidate batch 入口作为 **重规划尝试代理**：一个 batch 可包含多个 hypothesis，也可能是一次 FSM 调用中的 retry。CANDIDATE_GENERATION_RATE 为有 stage=CANDIDATE 日志的 hypothesis solve 调用；candidate 对象数包含无效结果，不能解释为成功安全轨迹数。

| UAV | batch 尝试数 / 秒 | hypothesis solve 数 / 秒 | candidate 对象数 / 秒 | commit 数 / 秒 | 实际 activation 数 / 秒 |
| --- | --- | --- | --- | --- | --- |
| 0 | 125 / 1.560 | 229 / 2.857 | 240 / 2.994 | 39 / 0.487 | 39 / 0.487 |
| 1 | 301 / 3.755 | 403 / 5.028 | 412 / 5.140 | 34 / 0.424 | 34 / 0.424 |
| 2 | 200 / 2.495 | 302 / 3.768 | 306 / 3.818 | 32 / 0.399 | 32 / 0.399 |

| UAV | activation Δt P50 / P95 / max (s) | 无新 activation >0.5 / >1 / >2 s | 仅有限 polynomial 执行前缀 >0.5 / >1 / >2 s |
| --- | --- | --- | --- |
| 0 | 1.255 / 4.708 / 5.090 | 32 / 31 / 16 | 32 / 31 / 16 |
| 1 | 2.420 / 4.069 / 4.274 | 31 / 31 / 23 | 31 / 31 / 23 |
| 2 | 2.147 / 4.440 / 5.674 | 29 / 28 / 24 | 29 / 28 / 24 |

“无新 activation”包含末端 hold；右列截去 hold。最后一条在 mission 结束截断，不人为补一次 activation。首个间隔可从区间前的已激活轨迹起算。多数 commit 都迅速送达并执行，**主要损失发生在 candidate→coordination/finalization→commit，而非 commit→traj_server transport。**

![80 秒接管时间轴](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_audit_20260910/lifecycle_timeline.png)

图中蓝色为普通轨迹、绿色为 joint，数字为 trajectory_id；红色为真实末端 hold，灰色为 pending 生命周期，紫色叉为 stale head rejection。

**2. 直接证实的“跑完→等待→下一条”事件**

polynomial 的真实截止时刻是 `message.start_time + duration`。receive/实际 scheduled callback 一般略晚于 start_time，不能借此延长 polynomial。下表使用真实截止时刻；附件同时给出用户公式 `actual_receive_time + duration` 的差值，分类不变。按 ±10 ms 区分：99 次结束前接管、0 次近似刚好接管、6 次结束后接管（105 次中的 5.7%）。这是反复发生的事件，集中于 UAV1，并非所有切换都会跑完。

| UAV / old→new id | old_end (mission s) | 新 activation (s) | gap (s) | 末端前 source | 末端 odom speed (m/s) | gap 内 collision / swarm samples |
| --- | --- | --- | --- | --- | --- | --- |
| 1 / 7→8 | 14.964348 | 15.396659 | 0.432311 | PERSISTENCE_FALLBACK | 0.067 | 0 / 0 |
| 1 / 12→13 | 25.155428 | 26.347718 | 1.192289 | PERSISTENCE_FALLBACK | 1.184 | 12 / 0 |
| 1 / 16→17 | 33.705971 | 33.950934 | 0.244962 | PERSISTENCE_FALLBACK | 0.036 | 0 / 0 |
| 1 / 24→25 | 56.852643 | 57.430456 | 0.577812 | PERSISTENCE_FALLBACK | 0.266 | 0 / 15 |
| 1 / 30→31 | 68.927964 | 69.491128 | 0.563164 | NOMINAL | 0.085 | 0 / 0 |
| 2 / 24→25 | 53.121283 | 53.144664 | 0.023381 | PERSISTENCE_FALLBACK | 0.054 | 0 / 0 |

UAV1 的 5 段 hold 累计 **3.010539 s**，最长 **1.192289 s**；UAV2 一段 **0.023381 s**；UAV0 无结束后接管。6 段均有独立 traj-server-terminal-hold 日志，与 start+duration 相符。全部原始行号见 [gap_events.json](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_audit_20260910/gap_events.json)。

[traj_server.cpp:585](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp:585)：t_cur≥duration 后，持续 publish_cmd(terminal_pos, Zero velocity, Zero acceleration, Zero jerk, last_yaw, 0)。这不是停止发消息后 controller 自行保持上一 desired state，而是 traj_server 每 10 ms 主动发送末端位置命令。是否在进入 hold 瞬间存在 v/a 指令阶跃，仍取决于末端 polynomial 导数，旧数据不足以数值还原；但 hold 行为及 UAV 的低速停顿均已证实。
UAV1 id7、16、30 的 hold 内平均 odom speed 分别约 0.066、0.031、0.041 m/s；id12 到期时仍约 1.184 m/s，hold 中发生静态接触，不能把它视为已经安全停稳。

**3. 等待发生在哪里、为什么候选没有接管**

[planner_manager.cpp:433](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:433)：effective team timeout = 0.35 + team_solution_max_age 0.65 = **1.00 s**，topology_min_active_remaining 同时被提升为 **1.05 s**。runtime_params.yaml 中原始 0.35 是加载参数值，实际运行值以 team-coordination-budget 和 topology-coordination-planner-config 为准。它与 85 ms ACK/commit activation margin 不是同一个阈值。
[ego_replan_fsm.cpp:235](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:235)：processPendingTopologyCoordination()==PENDING 时直接 return，普通 EXEC/REPLAN 状态机不继续运行；这里虽没有阻塞线程 sleep，却形成了**规划状态机层面的等待**。[planner_manager.cpp:2215](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:2215)：没有匹配结果就一直等 deadline，随后本地 fallback；[multi_uav_topology_coordinator.cpp:1524](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp:1524)：NO_EXECUTABLE_COMBINATION 或 core selection unavailable 时 consume snapshot 后返回，不发可让 planner 立即结束 pending 的否定结果。outer snapshot 重置/等待三机有效 bundle 的入口见 [multi_uav_topology_coordinator.cpp:595](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp:595)。

| UAV | pending 生命周期覆盖 (s) | coordinator timeout 次数 / 累计 wait (s) | stale head 次数 | batch compute mean/P95/max (ms) |
| --- | --- | --- | --- | --- |
| 0 | 75.825 | 74 / 74.387 | 40 | 7.055 / 13.488 / 154.448 |
| 1 | 70.591 | 68 / 68.400 | 37 | 13.059 / 85.717 / 238.727 |
| 2 | 74.413 | 73 / 73.397 | 43 | 5.435 / 20.616 / 86.827 |

全队累计 pending 生命周期 **220.829 UAV·s**，约占三机有效时长的 **91.84%**。这包括被后续 bundle 取代的 4 段很短窗口和结束时尚未解决的 3 段，故是 pending 生命周期覆盖统计，并非采样器测出的精确 CPU 阻塞占比。215 次 timeout 的等待 P50/P95/max 为 **1.006 / 1.010 / 1.011 s**。

215 次 timeout 后的可定位结果：119 次 STALE_HEAD_REPLAN；60 次 NOT_ABSOLUTE_SAFE 后保留旧轨迹；16 次 NO_SOLVED_CANDIDATE 后保留旧轨迹；17 次成功 commit；另 3 次候选复查失败且 previous validation 也失败。另有一次非 timeout 的 stale head，总计 120 次。**119 次是在 safety reclassification 已通过后，单纯因等待使 head 过时而拒绝**，因此可以明确回答“安全候选确实频繁算出但没有执行”，不需要把所有 generated=1 都误算为安全成功。

另一个明确缺陷位于 [planner_manager.cpp:1665](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1665)–[planner_manager.cpp:1788](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1788)：stageCapturedTopologyCoordination 先设置 pending_topology_.active=true，填充/发布 bundle，最后 `return executable_count >= 1`；当 executable_count=0 时没有清理 active。调用者按 false 去走 finalizeCapturedLocal，但下一次 FSM callback 仍会发现 pending。日志记录 **18** 个零 executable bundle，其中 **14** 个完整等到 timeout，总生命周期约 **14.098 s**；另 4 个被 retry 很快替换。coordinator 还以 `NO_RESULT_PLANNERS_ALREADY_BYPASSED` 描述这种情况，与 planner 实际状态不一致。

下表不是仅列字符串次数：每个原因都关联到该 UAV 的实际接管区间。等待代理 = “原因发生后到下一次实际 activation”的区间并集（每 UAV 去重，跨 UAV 求和），右侧给最长一段。不同原因会重叠，**不能相加或声称这些秒数是已分离的因果贡献**；唯一可直接测量的协议等待贡献是上面的 pending/timeout 区间。coordinator 的团队原因保守关联三机；静态错误等重复阶段日志保留原行数。

| 原因 | 日志事件数 | 关联 >1 s 接管间隔数 | 事件后等待并集 (UAV·s) | 最长事件后等待 (s) |
| --- | --- | --- | --- | --- |
| NO_VALID_VISIBILITY_ALTERNATIVE | 345 | 34 | 62.989 | 4.844 |
| candidate_count=1 | 897 | 90 | 230.548 | 5.663 |
| candidate_count=0 | 0 | 0 | 0.000 | 0.000 |
| STATIC_COLLISION | 266 | 22 | 38.779 | 4.588 |
| DYNAMIC_RISK_INVALID | 0 | 0 | 0.000 | 0.000 |
| SOURCE_CANDIDATE_NOT_ABSOLUTE_SAFE | 0 | 0 | 0.000 | 0.000 |
| ACTIVATION_RESERVE_EXHAUSTED | 0 | 0 | 0.000 | 0.000 |
| TRAJECTORY_BOUNDARY_MISMATCH | 0 | 0 | 0.000 | 0.000 |
| STALE_PLANNING_GENERATION | 3 | 3 | 6.213 | 3.643 |
| NO_PENDING_TOPOLOGY_GENERATION | 1 | 1 | 3.645 | 3.645 |
| KEEP_PREVIOUS_SAFE | 595 | 53 | 94.005 | 4.843 |
| PERSISTENCE_FALLBACK | 19 | 19 | 23.367 | 4.253 |
| JOINT_REJECTION | 6 | 5 | 9.985 | 3.654 |
| CAMERA_TIME_REJECTION | 1 | 1 | 0.031 | 0.012 |
| STALE_HEAD_REPLAN | 120 | 62 | 98.344 | 4.657 |
| COORDINATOR_TIMEOUT | 215 | 90 | 140.785 | 4.657 |
| NO_SOLVED_CANDIDATE | 297 | 19 | 25.350 | 3.514 |
| NOT_ABSOLUTE_SAFE_RECHECK | 68 | 38 | 57.391 | 4.255 |
| STALE_BUNDLE | 12 | 32 | 58.957 | 5.662 |
| CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE | 8 | 16 | 31.350 | 3.049 |
| EXECUTION_SAFETY_REJECT | 7 | 1 | 0.435 | 0.435 |

`candidate_count=0` 为 0 不代表没有空的有效集合：capture 可保留失败的 MINCO/init 对象，`candidate_count=1` 也不代表有效。934 条 hypothesis solve 日志中 generated=1 为 927、generated=0 为 7；零 executable bundle 为 18。STATIC_COLLISION 行包括重复诊断阶段，不是 266 次真实碰撞。NO_PENDING 和 STALE_PLANNING_GENERATION 只计 proposal-reject，不重复计 ACK-send。

本次 ACTVATION_RESERVE_EXHAUSTED 实际日志为 0；另有 1 次 PREDICTED_PLANNER_WINDOW_EXPIRED。有效 binary camera-time rejection 为 1 次，事件到下一激活的最大等待约 12 ms，不能解释全程数秒停顿。没有必要降低 85 ms margin 或改 visibility objective 来解释本次问题。

最长 10 个无新 activation 区间如下；每段所有 rejection/fallback 关联见附件 JSON，不能把同段多原因重复累加。

| UAV / id→id | 起止 mission s | 无新 activation (s) | timeout / stale head / NO_SOLVED 日志数 |
| --- | --- | --- | --- |
| 2 / 25→26 | 53.145–58.819 | 5.674 | 5 / 2 / 46 |
| 0 / 9→10 | 10.765–15.856 | 5.090 | 5 / 3 / 0 |
| 0 / 32→33 | 54.246–59.314 | 5.068 | 5 / 5 / 0 |
| 2 / 24→25 | 48.282–53.145 | 4.862 | 4 / 3 / 29 |
| 0 / 24→25 | 34.259–38.927 | 4.668 | 4 / 2 / 23 |
| 1 / 19→20 | 38.020–42.294 | 4.274 | 4 / 0 / 2 |
| 0 / 34→35 | 63.368–67.509 | 4.142 | 4 / 1 / 0 |
| 0 / 35→36 | 67.509–71.632 | 4.123 | 4 / 1 / 0 |
| 0 / 22→23 | 29.142–33.242 | 4.100 | 4 / 1 / 2 |
| 0 / 27→28 | 44.010–48.108 | 4.098 | 4 / 3 / 0 |

**4. 起点的时间语义与最终 P/V/A gate**

| 执行路径 | head 来自何时/何处 | 最终 active→new position | 最终 active→new velocity / acceleration |
| --- | --- | --- | --- |
| NOMINAL | FSM 规划调用时的 active polynomial P/V/A；tracking error/expired 则当前 odom p/v、a=0；commit 时重设 start=now | 仅候选 p(0) 对当前 odom ≤0.5 m，不是严格同一时刻 active p | 无 / 无 |
| SIDE_PLUS / SIDE_MINUS | 沿用同一调用的 start_pt/start_vel/start_acc；之后 A*/MINCO/SCP/retiming | 同普通 finalize gate | 无 / 无 |
| FEASIBLE_FALLBACK | SIDE 保存的 feasible initializer，经同一最终检查入口 | 同普通 finalize gate | 无 / 无；本次无实际执行样本 |
| PERSISTENCE_FALLBACK | 继续原 id/start/poly 和原世界时间相位，未重置起点 | 无新轨迹可比较；对剩余轨迹做 safety revalidation | 原 polynomial 延续；退出时走 successor 的弱 gate |
| JOINT_TEAM_SOLUTION | source candidate 在 common activation 时刻的 suffix P/V/A；不是实际 active 的 future P/V/A | proposal 对 actual active future position 只要求 ≤0.5 m | source suffix P/V/A 检查 1e-5；actual active→new 无 v/a gate |
| traj_server activation | 直接采用消息 coefficients/start_time | 无独立接管 P gate | 无 / 无 |

普通 head 取值：[ego_replan_fsm.cpp:1002](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:1002)、[ego_replan_fsm.cpp:1056](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:1056)；安全投影（若触发）可修改位置、移除部分速度并清零加速度：[ego_replan_fsm.cpp:1724](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:1724)。本次没有该投影 warning，不把它列为已证实主因。常规 topology finalize 的 position-only gate 在 [planner_manager.cpp:1936](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1936)，legacy direct finalize 的同类 gate 在 [planner_manager.cpp:7459](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:7459)；真正设置 start_time 的入口为 [planner_manager.cpp:917](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:917)。因此确有“用规划时刻状态生成的 head，约 1 秒以后才从 t=0 开始执行”的路径。

这不是单纯 200 ms 的假设：120 次 stale head 的 planning→final gate 延迟 P50 **1.009458 s**。还存在 **17** 次等待超过 0.5 s 后仍通过 position freshness 的普通 commit，head-to-current-odom 差值 P50/P95/max **0.358 / 0.492 / 0.496 m**。慢下来后旧 head 更容易重新落入 0.5 m 以内，形成“快飞时拒绝，临近末端减速后又能接受”的机制。后一句是源码和统计支持的机制解释，并不等于每次停顿都由这个 gate 单独造成。

JOINT：[multi_uav_topology_coordinator.cpp:925](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp:925) 对 source candidate 做 suffix，[planner_manager.cpp:1222](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1222) 校验 proposed 与 source suffix 的两端 P/V/A；[planner_manager.cpp:1309](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1309) 才比较实际 active 在 future activation 的位置，并没有比较其 v/a。stage bundle 在 [planner_manager.cpp:1670](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1670) 把轨迹 origin 声明为 evaluation_start_time（now 或预测 commit median），并未同步重建 head；因此 source suffix 的时间一致性不能证明实际 active 接管一致性。

已执行的 **team solution 6 / UAV0 / trajectory40**：[runtime head evidence](/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910/run/canonical_rosout.log:32915) 明确记录 activation_head_error **0.286041 m**，active_tracking_error 仅 **0.013738 m**，随后三机 commit/adoption/执行已留证。这里 head_error 的计算是 proposed p(0) 与 actual active polynomial 在共同 activation 时刻的距离，已足以证实一个被允许的**参考位置边界偏差**；不能再用上一份“缺完整系数”的结论断言没有切换问题。所有已 commit team 的 12 条 head 校验日志见 extra.json。v/a 的真实边界 residual 仍未记录，不能凭空补数。

full safety validation 不等于跨代 C2 validation。上述各新轨迹仍调用 validateExecutionTrajectory（dynamics/static/dynamic/swarm，joint 使用团队 payload），但该函数没有 actual active→new P/V/A head compatibility。traj_server 的 safety_validated/generation/尺寸/有限数值检查也不补这个缺口。

**5. Persistence 完整 episode 与 safety 生命周期**

以下 19 段按 traj_server source telemetry 切入 persistence 计数；它们继续旧 polynomial，并非 19 次新 activation。planner 内部 KEEP_PREVIOUS 日志更多：它可能尚未重新发布 source，不能把每条日志都当作一次执行切换。等待以接收新 id/start 的时刻计，source 标签退出可能晚约一个 10 ms command tick。

| UAV/id | 进入 mission s | 剩余时长 s | 进入 speed | 下一 activation 等待 s | 标签持续 s | 退出 source | 退出 Δv / Δheading 代理 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0/5 | 6.625 | 1.670 | 2.600 | 1.012 | 1.020 | NOMINAL | 0.025 / 0.718 |
| 0/22 | 32.225 | 1.301 | 0.953 | 1.017 | 1.020 | NOMINAL | 0.015 / N/A |
| 0/24 | 38.635 | 0.724 | 0.634 | 0.292 | 0.300 | NOMINAL | 0.001 / 0.019 |
| 0/34 | 64.405 | 4.101 | 0.734 | 3.104 | 3.110 | NOMINAL | 0.014 / 0.006 |
| 1/6 | 11.582 | 0.654 | 0.825 | 0.424 | 0.430 | NOMINAL | 0.059 / N/A |
| 1/7 | 12.032 | 2.933 | 0.101 | 3.365 | 3.370 | NOMINAL | 0.078 / N/A |
| 1/10 | 19.741 | 1.364 | 1.900 | 1.127 | 1.130 | NOMINAL | 0.006 / N/A |
| 1/12 | 24.072 | 1.084 | 1.133 | 2.276 | 2.280 | NOMINAL | 0.050 / 0.006 |
| 1/14 | 26.842 | 2.836 | 1.045 | 1.280 | 1.280 | JOINT_TEAM_SOLUTION | 0.063 / 1.596 |
| 1/15 | 29.311 | 1.910 | 2.048 | 1.015 | 1.020 | NOMINAL | 0.021 / 0.032 |
| 1/16 | 33.522 | 0.184 | 0.016 | 0.429 | 0.430 | NOMINAL | 0.007 / N/A |
| 1/19 | 38.042 | 5.168 | 0.994 | 4.253 | 4.260 | SIDE_MINUS | 0.033 / 0.048 |
| 1/20 | 44.352 | 1.865 | 2.017 | 1.020 | 1.020 | NOMINAL | 0.082 / 1.141 |
| 1/24 | 56.602 | 0.251 | 0.069 | 0.829 | 0.830 | NOMINAL | 0.086 / 0.277 |
| 1/32 | 76.642 | 0.630 | 0.431 | 0.390 | 0.390 | NOMINAL | 0.024 / N/A |
| 2/16 | 30.376 | 0.303 | 0.174 | 0.197 | 0.200 | NOMINAL | 0.019 / N/A |
| 2/24 | 52.396 | 0.726 | 0.741 | 0.749 | 0.750 | NOMINAL | 0.004 / N/A |
| 2/25 | 58.266 | 0.772 | 1.058 | 0.553 | 0.560 | NOMINAL | 0.039 / N/A |
| 2/29 | 68.976 | 0.746 | 0.694 | 0.035 | 0.040 | SIDE_PLUS | 0.056 / 0.188 |

9/19 次进入时剩余 <1 s，持续时间 P50/P95/max **1.020 / 3.459 / 4.260 s**。表中 Δv/Δheading 是上一审计的邻近 odom 外推代理，不是精确 command/poly 跳变；完整估计方法见 feedback_28，全部数据在 fallback_episodes.json。

验证确实存在：[planner_manager.cpp:2384](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:2384) 每次使用前 slice remaining，再检查 dynamics/static/dynamic/swarm/完整 execution safety。不能声称 persistence 没有验证。但 validation 的有效范围是剩余 polynomial，**不能自动涵盖到期后的无限 terminal hold**。[traj_server.cpp:585](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp:585) 进入 hold 的分支还早于 source telemetry 发布分支，因此 CSV 的 PERSISTENCE_FALLBACK/safety_validated=true 会继续沿用旧值，并没有显式显示 TERMINAL_HOLD。

安全事件现在可进一步归位：

- UAV1 id12：old_end=25.155428，terminal hold 日志 [canonical_rosout.log:10492](/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910/run/canonical_rosout.log:10492)，直到 26.347718 新 id13 接管。此前报告的 12 个 static collision/contact samples 全在 25.417288–25.784160，**全部发生在这个 hold 期间**。
- UAV1 id24：old_end=56.852643，terminal hold 日志 [canonical_rosout.log:23985](/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910/run/canonical_rosout.log:23985)，直到 57.430456 新 id25 接管。56.884599–57.350720 的 15 个 swarm violation pair samples 全落在此段，另一架为 UAV2 正在执行的 NOMINAL/id25。
- UAV2 id25 在 56.215652 已收到 previous validator 的 **swarm invalid**，预测冲突在约 0.72 s 后，见 [canonical_rosout.log:23147](/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910/run/canonical_rosout.log:23147)；实际仍执行到 58.818869 左右的下一 activation。57.221735 又在 t=0 报 swarm invalid，见 [canonical_rosout.log:24427](/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910/run/canonical_rosout.log:24427)。`event=INVALIDATED active=0` 是 planner 诊断字段，**不是发给 traj_server 的取消/安全制动命令**。全区间 28 条 INVALIDATED（27 swarm、1 static）进一步说明 safety reject 与当前执行状态没有形成充分闭环。

另一个源码语义差异：[planner_manager.cpp:3154](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:3154) 对过期队友轨迹采用 `end_pos + elapsed * end_vel` 线性外推，traj_server 实际执行 terminal position hold。若 end_vel 非零，两边模型不同；本次未存末端 coefficients，不能认定该差异就是这段 swarm violation 的唯一原因。

**6. 相邻未来轨迹方向变化：已知与未知**

旧数据没有完整 coefficients、MINCOTraj/PolyTraj payload 或完整未来采样。因此 actual trajectories 在共同未来世界时段的 maximum spatial separation、initial polynomial tangent angle、heading profile difference 均为 **N/A**。本轮没有再次用 MINCO 内部 C2 公式替代跨轨迹检查，也没有运行新仿真补数据。

可以恢复的是“被选中 candidate 的 fresh-init 规划起点→目标方向”代理：依据 candidate_id 串到最终 selected/adopt 与 actual activation；它是该次规划请求的目标，**不是最终 polynomial 的完整曲线**，joint 更不能把 source 原始 head 当成优化后 suffix head。105 对可匹配事件的目标方向变化 P50/P95/max = 8.868° / 73.529° / 130.355°；目标位置变化 P50/P95/max = 2.725 / 6.194 / 7.549 m，包含目标自身运动与滚动 horizon，不能直接当作同世界时间曲线距离。

| UAV/id→id | mission s | 初始 source→新 source | encirclement generation | 规划目标方向变化° | 规划目标位置变化 m |
| --- | --- | --- | --- | --- | --- |
| 2/18→19 | 33.934 | NOMINAL→NOMINAL | 24→26 | 130.355 | 6.255 |
| 1/16→17 | 33.951 | NOMINAL→NOMINAL | 24→26 | 123.245 | 5.231 |
| 2/29→30 | 69.011 | NOMINAL→SIDE_PLUS | 48→49 | 101.120 | 5.660 |
| 1/30→31 | 69.491 | NOMINAL→NOMINAL | 48→50 | 96.062 | 6.965 |
| 1/25→26 | 60.494 | NOMINAL→NOMINAL | 42→44 | 86.175 | 4.452 |
| 2/14→15 | 26.823 | NOMINAL→NOMINAL | 20→21 | 74.134 | 2.781 |

105 次实际相邻新轨迹的 hypothesis_id 全为 0→0，但其中 80 次 encirclement_generation 改变。**id=0 不意味着几何不变或回到固定120°**；新 generation 的 current/adaptive reference 可以不同。上表大转向多数发生在 NOMINAL→NOMINAL，不能把 SIDE 符号翻转当作唯一主因。已验证 adaptive geometry 曾作为 hypothesis0 被实际执行，但缺全未来轨迹，无法确认某次大幅曲线变化仅由 hypothesis/topology 切换造成。

源码允许这些变化：[ego_replan_fsm.cpp:862](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:862) 根据新的 reference 平移 local target；[ego_replan_fsm.cpp:1002](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:1002) 失败后从 odom/a=0 重建；[planner_manager.cpp:3493](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:3493) 在 flag_polyInit=true 时不使用已接受轨迹 warm start。656 条 NO_ACCEPTED_STATE 中包括“因 flag_polyInit 而跳过”的情况，不能误解为 cache 真不存在；实际 USED_PREVIOUS_SAFE 229 条，HORIZON_TOO_SHORT 56 条。连续失败、末端剩余耗尽和重新初始化会增大下一条轨迹相对旧计划的变化。
局部 SIDE hysteresis ([planner_manager.cpp:6808](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:6808)) 只约束同障碍、两侧均合格时 PLUS/MINUS 的切换；team tie hysteresis ([topology_coordinator_core.cpp:536](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/topology_coordinator_core.cpp:536)) 只比较 hypothesis label/kinds 和 camera-time tie，并不限制两代完整曲线的几何差异。现有滞回没有承诺跨代轨迹形状相似，不能称其实现“失效”；没有 transition/head contract 才是该层明确缺口。

**7. 按影响排序的根因**

| 排序 / 级别 | 源码与运行证据 | 对现象的解释 |
| --- | --- | --- |
| ROOT_CAUSE_1 — CONFIRMED | FSM:235；planner_manager:433、2215；215 次约1 s timeout，119 次超时后 stale head；实际接管0.40–0.49 Hz | 候选算完却等 coordination；迟迟不接管，旧轨迹继续走，导致低有效更新频率和临近末端减速。 |
| ROOT_CAUSE_2 — CONFIRMED | FSM:1002/1056；planner_manager:1670/1936/1309；120 stale head、17 个延迟 freshness 通过、已执行 joint6 的 head 差0.286041 m | head 的时间基准没有和真实 future activation 统一；position freshness代替C2 gate，允许参考位置偏差，v/a和方向突变风险未受接管gate约束。真实Δv/Δa数值仍待payload。 |
| ROOT_CAUSE_3 — CONFIRMED | planner_manager:1665–1788；18 个零executable pending，14次完整timeout | 不可能进入团队执行的bundle仍占pending生命周期，制造无意义等待和重复fallback。 |
| ROOT_CAUSE_4 — CONFIRMED | planner_manager:2384；traj_server:585；19 persistence段、6 hold、最长1.192 s | 保留有限旧轨迹不延长寿命；持续失败后真到末端，发零速hold；随后新generation成功才重启运动。 |
| ROOT_CAUSE_5 — CONFIRMED（具体碰撞全部因果贡献未唯一分离） | planner_manager:1814/2384；日志23147、24427；28 invalidation未等于取消执行；hold覆盖12 static和15 swarm samples | 当前剩余轨迹已被判不安全也可能继续执行；到期hold没有独立状态/安全契约，source标签掩盖这一阶段。 |
| ROOT_CAUSE_6 — LIKELY | FSM:862/1002；planner_manager:3493；目标方向代理最大130°，大量同NOMINAL代际更新 | 几秒后新target/reference、odom重初始化、短horizon或不可用warm start，导致下一条未来计划显著不同；精确共同未来曲线差异仍缺证据。 |
| ROOT_CAUSE_7 — POSSIBLE | planner_manager:3154 对队友末端线性外推，traj_server:585 实际hold | end_vel非零时，预测队友与实际停住的队友不同；需保留末端导数才能归因到这次冲突。 |

这次全程的主证据不支持“generator/joint 太慢”或“RViz 红 marker 稀疏”作为主要解释；毫秒计算与秒级 pending/有限轨迹末端 hold 是不同问题。普通 periodic replan 阈值本来就为 1 s，不能把 100 Hz FSM/heartbeat timer 当作应有 100 Hz 轨迹接管率。

**8. 最小针对性修复方向（仅建议，本轮未实施）**

1. 修复零 executable 的 pending 副作用：在设置/发布 pending 前确认可执行集合；失败清理 pending/team identity。对明确 unavailable 的协调结果给带 generation 的终态，允许立即走已验证的本地 successor，避免固定等满 1 s。
2. 将“候选等待协调”与“普通轨迹继续滚动更新”分开管理。以实际剩余执行寿命设 deadline；一旦决定换用本地 successor，原 bundle/proposal 必须按 generation 正确失效，不能放宽 stale/identity 检查。不要通过降低 85 ms ACK/commit margin 解决 1 s 等待问题。
3. 所有新轨迹采用同一 future activation 状态：从真正 active polynomial 在 activation 时刻取 P/V/A，再规划/切片/验收。在不可直接兼容时需要短 C2 transition，并重新做完整 nonlinear safety/dynamics/yaw/Local-SFC 检查；不应只扩大 freshness tolerance 或盲目 blend。优先实现 actual active→new P/V/A gate，保留现有 source-suffix gate。
4. persistence 每次使用前的重验证保留，并把“已 invalid / 即将到期 / 正在 terminal hold”作为 execution 生命周期状态。持续失败时，在剩余安全前缀用尽前产生可验证的制动/保持轨迹；不能只写 INVALIDATED 日志后继续旧命令。terminal hold 与队友预测使用同一模型，执行 telemetry 显式标记 HOLD 及验证时刻/有效期。
5. generation 只做必要的 stale/identity 和接管兼容限制，不添加无证据的低频 rate limit，也不冻结 adaptive reference。为后续验证保留 active/new polynomial、PositionCommand P/V/A、实际 activation/head residual；再评估是否需要额外的几何 transition 约束。visibility-first、J_vis、J_acc、K2/blackout 和安全阈值保持原语义。

**9. 最终判定**

以下 NO 对“是否由 topology/hypothesis 导致大几何变化”表示未得到该因果链的充分证据，不表示几何稳定。YES 对“frequently reach end”表示反复出现了可见事件（6/105，集中 UAV1），不表示多数轨迹都过期。

```text
IS_EFFECTIVE_TRAJECTORY_ACTIVATION_RATE_TOO_LOW: YES
DO_TRAJECTORIES_FREQUENTLY_REACH_END_BEFORE_NEXT_ACTIVATION: YES
DOES_PERSISTENCE_FALLBACK_CAUSE_LONG_HOLDS_OR_NEAR-END_EXECUTION: YES
IS_NEW_TRAJECTORY_HEAD_BASED_ON_WRONG_TIME_STATE: YES
IS_ACTIVE_TO_NEW_PVA_COMPATIBILITY_FULLY_CHECKED: NO
ARE_VALID_CANDIDATES_FREQUENTLY_COMPUTED_BUT_NOT_EXECUTED: YES
IS_TOPOLOGY/HYPOTHESIS_SWITCHING_CAUSING_LARGE_GEOMETRIC_CHANGES: NO
IS_RVIZ_SPARSE_RENDERING_THE_PRIMARY_PROBLEM: NO

PRODUCTION_SOURCE_CHANGED: NO
SIMULATION_RUN: NO
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

根因一句话：**秒级 coordination pending 消耗了候选 head 的有效时间和旧轨迹寿命，而跨代接管只做宽松位置 freshness，失败后继续旧轨迹直到 terminal hold，形成“飞一段—减速/停住—新方向接管”。**

离线交叉核验：105 commit↔105新activation、6 terminal-hold日志↔6正gap、19 persistence标签段、12 static与15 swarm样本的hold归属一致；读取前后 1466 个生产普通文件 SHA256 无变化。当前没有 ROS/仿真/RViz 残留进程。本轮仅新增审计脚本、数据附件和本报告。
