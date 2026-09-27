# ALP trajectory lifecycle 修复与完整运行验证 — 2026-09-10

**滚动更新与跨代 P/V/A 接管问题已修复并实测；整体安全验收仍未通过。** 最终生产版本完整跑完 Scenario A：三机真实接管率 2.981/3.106/3.318 Hz，754 次接管、零轨迹耗尽等待、零 terminal hold。所有已激活新 polynomial 的边界 P/V/A 误差均在新 gate 内。另一方面，实测有 6 个机间距离违规样本、4 个遥测标记为未验证的执行样本；在已失效且没有可行制动的情况下，旧执行仍延续约 0.14–0.16 s。因此不能把本轮描述为完整安全闭环已解决。

本轮 2 次 FULL ON 完整运行。第一次发现具体 scheduled-local→team 时序错误以及 20 ms 制动末端等待后，进行了针对性小修复和重新 build/tests，再运行最终版本。没有 OFF、A/B，没有调整 visibility 权重或安全阈值寻找更好数值。两次都跑到 target 正常结束，均已清理。

最终证据目录：[trajectory_lifecycle_fix_verified_20260910](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_fix_verified_20260910)；第一次证据：[trajectory_lifecycle_fix_20260910](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_fix_20260910)。完整数据、运行命令、执行器系数、命令样本、日志、diff、测试及 hash 均保留。没有修改 visibility-first / AdaptiveViewpointGenerator / J_vis / J_acc / K2 / blackout / 120° seed 主公式。

## 1. 改动范围与真实执行结构

| 源码 | 主要修改 |
|---|---|
| [ego_replan_fsm.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp:233) | pending 不再提前返回冻结 FSM；候选先 local finalize / publish，再将同代精确 local polynomial 送入 team bundle；每次规划使用 future activation head；监测寿命并尝试 successor / validated brake。 |
| [planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:867) | 新增 `prepareFutureActivation`、`prepareLocalHandoff`、`checkActiveHandoff`、`validateActivePrefixUntil`、`lifecycleSuccessorDue`、`validatedBrakeSuccessor`、`invalidatePendingTopology`；统一直接/捕获候选的最终入口；保留 nonlinear safety、Local-SFC、source suffix 和团队 payload 检查。 |
| [planner_manager.h](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h) | active/scheduled predecessor、真实 adoption identity、planning latency budget、P/V/A tolerance、validation state/expiry 等成员和接口。 |
| [traj_server.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp:118) | 接收及 scheduled activation 两次 actual predecessor→new P/V/A gate；拒绝旧 generation；同 id/start 重发只更新状态，不重置 polynomial；精确 activation/PVA/source 日志；terminal hold 显式 ENTER/EXIT 且不冒充已验证。 |
| [multi_uav_topology_coordinator.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp:1490) | `publishTerminalNegative` 对空可执行组合、明确 selection unavailable、已过接纳窗口的 snapshot 返回带三机 planning generation 的终态。 |
| [trajectory_lifecycle.h](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/include/traj_utils/trajectory_lifecycle.h)（新增） | 共享 finite polynomial→terminal position hold 的 P/V/A 采样及 handoff residual / generation / reserve contract。 |
| [PolyTraj.msg](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/PolyTraj.msg) | 同一个执行 payload 携带 candidate-ready、commit、pending duration、validation time/expiry、lifecycle state。 |
| [poly_traj_optimizer.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) | 仅统一四处过期 peer 的预测：末端位置保持，v/a=0；去掉 `end_pos + elapsed * end_vel`。不修改 visibility cost 公式。 |
| [CMakeLists.txt](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/CMakeLists.txt)、新增两个 lifecycle test、现有 `adaptive_execution_contract_test.py` | 加入 numerical/wiring contracts；现有 emergency/finalize 测试适配统一入口。 |

新主链：**local safe successor 先提交并排程 → team pending 后台增强 → generation、source、实际前驱和激活时间全部有效时接管**。期间新 local commit 清理旧 pending；旧 team result 不能覆盖较新 local generation。不是将原来的 1 s timeout 简单改短。

受保护源码 hash 对照：cooperative_viewpoint_manager、adaptive_viewpoint_generator、team_visibility_optimizer、topology_coordinator_core 均与本轮开始前一致。构建及运行后的 hash 也一致。

## 2. pending、zero executable 与协议时序

原问题是 FSM 顶部对 `PENDING` 无条件 return；逻辑等待 1 s 虽不阻塞 ROS 线程，却停止普通滚动 replan。现在 pending 只处理已经到达的结果，不拥有本地规划的进度许可；本地候选不再放到 timeout 后从原 t=0 执行。最终运行记录到 **411 次 pending=1 时的真实 planning start**。

`stageCapturedTopologyCoordination` 先构造 inactive 状态并计算 executable_count，只有至少一个 ABSOLUTE_SAFE 可执行候选才设置 active 并 publish。零集合清除本轮 identity；coordinator 的 negative result 只有 planning generation 匹配时才清理该 planner pending。`ZERO_EXECUTABLE_DOES_NOT_ENTER_PENDING=PASS`，最终运行零 ghost pending。

保留 proposal→ACK→commit 协议。已经发出接受 ACK 时，前驱只保留到其 common activation；这是短暂的已接受接管预约，不是普通 pending 的 1 s 阻塞。安全撤销通过既有 negative ACK 通知 coordinator。generation/source/yaw payload 一致性仍检查。

第一次运行的新问题：本地 successor 已排程但尚未执行，旧 joint ACK 逻辑要求 `active_now>=0`，误把有效 future predecessor 判为 stale；traj_server 又只有一个 scheduled slot，提前发布 team 会覆盖其尚未执行的 local 前驱。修复后：当前跟踪检查使用保存的正在执行 predecessor；future head 使用将先执行的 local trajectory；team commit 的发布等待 `/trajectory_execution/activated` 确认该 local id/generation 已实际接管。没有用固定 15 ms 猜测 adoption，也没有降低 ACK/commit 余量。

## 3. future activation 与 P/V/A gate

规划开始先安排：

`activation_seed = plan_start + max_recent_decay_planning_budget + 0.10 s`

预算是 `max(0.04, previous_budget*0.98, measured_planning_time*1.25)`。候选已经 ready 后，不再空等未用完的规划预算：

`activation = candidate_ready_now + 0.10 s`

随后以实际前驱在这个世界时刻的 P/V/A 重建 MINCO head，保留候选内部位置和 durations，并重新做完整 safety / Local-SFC 检查。最终检查如果已耗尽接管窗口会拒绝，不把同一系数改一个 start_time 硬发出去。已有 team 的 60 ms ACK + 25 ms commit = **85 ms** 要求未降低；原 stale/identity 检查保留。

Gate：`dp=||p_new(0)-p_active(t_activation)||`，dv、da 同理。默认全局参数 `/trajectory_lifecycle/handoff_position_tolerance=0.02 m`、`handoff_velocity_tolerance=0.05 m/s`、`handoff_acceleration_tolerance=0.10 m/s²`，planner 与 traj_server 共用。joint 同时保留 proposed→source suffix P/V/A 契约和新增 actual predecessor→proposed 契约。运行中的误差来自 float32 PolyTraj 系数传输/重建量级，没有放大 tolerance 接受不连续。

所有新 polynomial 走 `setLocalTrajFromOpt`；新增 old prefix 到 activation 的安全检查。没有给所有曲线加 blending，没有通过新轨迹头瞬间置零速度“修复”接管。

## 4. persistence、制动与 terminal-end model

剩余寿命阈值为 planning budget + 0.10 s activation margin + 0.65 s braking reserve。EXEC 阶段提前请求 local successor；remaining-suffix 重验证仍在。状态包含 CURRENTLY_VALID / INVALIDATED / EXPIRING / TERMINAL_HOLD，记录 last validation、expiry；完整 suffix 失效时，还独立验证覆盖重新规划/激活的短前缀。

候选不能及时接管时，`validatedBrakeSuccessor` 从真实 future P/V/A 构造有限最小 jerk 制动至零 v/a；每个试探都检查 dynamics/static/dynamic/swarm/Local-SFC，并另外验证有限后续 hold。没有把旧的瞬时 zero-v/a emergency 轨迹当作合法切换。最终有 5 次实际采用 validated brake，没有 polynomial 耗尽。

**这一部分仍不完整：** 如果完整 suffix 和短前缀都已经 invalid，而所有 brake 候选都不可行，当前实现只发出 BRAKE_UNAVAILABLE 并继续 urgent replan，traj_server 没有拿到合法替代前仍执行原曲线。validation expiry 目前也没有独立执行层强制取消/安全替代机制。最终运行确实触发此分支，不能声称“不安全 active 已完全取消”。这也是最终 `PERSISTENCE_EXPIRY_MANAGED` 保守记 NO 的原因：寿命预警已生效，完整失效状态闭环未达标。

共享 expiry model 在 planner/optimizer 使用末端位置保持，不再线性外推；traj_server 到期异常分支仍以此保持并显式标为 TERMINAL_HOLD、safety_validated=false。保留该异常分支不代表它是经过环境验证的安全策略。本次最终有效任务区间没有进入它。

## 5. 构建、测试与运行来源

最后构建：`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status`，6/6 packages 成功，无 warning。formation runtime 保持 -O2，ego_planner_node / traj_server / traj_opt 保持 -O3；没有 fast-math。运行中 executable、加载库路径和 hash 与当前 workspace 匹配，未使用旧 binary。

14 组检查全部通过：新增 C++ lifecycle 数值、6 个 formation contract executable、3 个 traj_opt contract、已有 tracking visibility / adaptive execution Python 检查、新增 lifecycle wiring、独立 ROS master 下真实 traj_server 的 scheduled handoff contract。后者实际发送错误 v/a、旧 generation、重发和连续 local→team polynomial，验证错误被拒、合法排程执行、重发不产生新 activation。

新增覆盖：pending 下继续 local replan；zero executable；generation-aware negative；future P/V/A；dv/da 拒绝；expiry advance reserve + brake gateway；peer terminal prediction；stale team/local generation；scheduled predecessor 的真实 adoption 顺序；predecessor prefix 验证。joint P/T/yaw、J_acc、K2/blackout 和 directional visibility 的 finite-difference/gradient contracts 通过。测试通过不等于本次物理执行安全通过。

证据：[tests.json](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_fix_20260910/tests.json)、[最终 build](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_fix_20260910/build_final_runtime.log)、[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_fix_verified_20260910/final_verification.json)。

## 6. 两次运行与统一统计口径

第一次完整运行主要结果：接管率 2.033/3.119/2.295 Hz，597 次接管，零静态/动态 collision、零机间违规；一次 VALIDATED_BRAKE→SIDE_PLUS 在旧轨迹结束后 20.275 ms 排程，terminal hold 20.038 ms；joint 因上述 scheduled predecessor 问题未 commit。修复这两个明确问题后才做第二次完整验证，不将第一次的有利安全数字移到最终版本。

**以下全部数据属于第二次、最终生产版本。** Target mission `80.204540745 s`，正常到达路线终点；统计绝对时间 `[1789047250.376031876, 1789047330.554010630]`，相对 mission `[0.026562, 80.204541] s`，有效时长 **80.177978754 s**。统一裁掉缺少完整首帧的数据和关闭阶段，不裁掉任何违规区间。

真实 activation 以 traj_server 的执行 adoption / ACTIVATION 日志交叉核对；同 id/generation/start 的重发、persistence source 改标均不是新 activation。保存了全部执行 polynomial 系数和 24,054 个 PositionCommand 样本。replan rate 是实际 `callReboundReplan` 进入次数，candidate rate 是 hypothesis solve 次数，不声称每次都成功或绝对安全。速度/clearance 使用实际 odom；source 与 id/generation/team_solution_id 关联。

## 7. 有效更新频率与等待

| UAV | Replan count / Hz | Candidate count / Hz | Commit count / Hz | Actual activation count / Hz | Activation interval P50/P95/max s |
| --- | --- | --- | --- | --- | --- |
| 0 | 335 / 4.178205 | 561 / 6.996934 | 239 / 2.980868 | 239 / 2.980868 | 0.279744/1.094022/1.459642 |
| 1 | 302 / 3.766620 | 527 / 6.572877 | 249 / 3.105591 | 249 / 3.105591 | 0.279707/0.814215/1.659809 |
| 2 | 260 / 3.242786 | 486 / 6.061515 | 266 / 3.317619 | 266 / 3.317619 | 0.279948/0.560276/1.260657 |

| 指标 | 最终运行值 |
| --- | --- |
| COORDINATION_PENDING_COUNT | 611 |
| COORDINATION_PENDING_DURATION P50/P95/max s | 0.244832/0.569587/1.010135 |
| COORDINATOR_TIMEOUT_COUNT | 20 |
| STALE_HEAD_REPLAN_COUNT | 0 |
| ZERO_EXECUTABLE_PENDING_COUNT | 0 |
| TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT | 0 |
| TERMINAL_HOLD_COUNT / TOTAL / MAX | 0 / 0 s / 0 s |

仍有 20 次后台 timeout，最大 pending 约 1.01 s；没有使普通 replan 停止，也没有再拿等满 timeout 的旧 head 直接执行。没有要求将 timeout 计数本身变成零。

| UAV | 无新 activation >0.5 / >1 / >2 s 次数 |
| --- | --- |
| 0 | 31 / 14 / 0 |
| 1 | 25 / 8 / 0 |
| 2 | 22 / 6 / 0 |

## 8. 精确 handoff、source 与几何变化

754 次真实切换，均通过 configured tolerance；traj_server handoff reject=0。以下根据实际执行 payload 直接重建，非 odom 邻近差分估计。

| Residual | P50 | P95 | max |
| --- | --- | --- | --- |
| dp | 3.643082023e-07 | 1.570930144e-06 | 3.235342584e-06 |
| dv | 2.819645711e-08 | 1.708889869e-07 | 7.694649006e-07 |
| da | 1.714786516e-08 | 1.204202311e-06 | 6.385413775e-06 |

按真实切换当时 OLD_SOURCE→NEW_SOURCE 分类；每格为 P50/P95/max。PERSISTENCE 是原 polynomial 的执行状态，不能把一次改标计为新轨迹。

| Transition | count | Δp m | Δv m/s | Δa m/s² |
| --- | --- | --- | --- | --- |
| NOMINAL->NOMINAL | 384 | 4.131e-07/1.726e-06/3.235e-06 | 2.571e-08/7.965e-08/1.517e-07 | 1.218e-08/7.300e-08/2.258e-07 |
| NOMINAL->JOINT_TEAM_SOLUTION | 136 | 3.870e-07/1.453e-06/1.895e-06 | 6.095e-08/5.575e-07/7.695e-07 | 4.964e-07/4.453e-06/6.385e-06 |
| JOINT_TEAM_SOLUTION->NOMINAL | 121 | 3.616e-07/1.198e-06/1.873e-06 | 2.035e-08/5.329e-08/9.972e-08 | 7.163e-09/3.964e-08/6.044e-08 |
| PERSISTENCE_FALLBACK->NOMINAL | 69 | 2.321e-07/1.353e-06/1.906e-06 | 3.697e-08/1.622e-07/3.384e-07 | 3.470e-08/4.737e-07/6.030e-07 |
| PERSISTENCE_FALLBACK->VALIDATED_BRAKE | 5 | 1.801e-07/5.454e-07/5.909e-07 | 1.240e-08/2.504e-08/2.531e-08 | 3.534e-08/1.333e-07/1.412e-07 |
| VALIDATED_BRAKE->NOMINAL | 2 | 2.674e-07/4.186e-07/4.354e-07 | 6.728e-09/7.891e-09/8.020e-09 | 7.716e-08/1.409e-07/1.479e-07 |
| PERSISTENCE_FALLBACK->SIDE_MINUS | 2 | 5.113e-08/5.121e-08/5.122e-08 | 3.932e-08/5.588e-08/5.772e-08 | 6.098e-08/1.058e-07/1.108e-07 |
| SIDE_MINUS->SIDE_MINUS | 5 | 6.053e-08/2.141e-07/2.459e-07 | 2.024e-08/6.216e-08/6.563e-08 | 6.564e-08/1.625e-07/1.778e-07 |
| SIDE_MINUS->JOINT_TEAM_SOLUTION | 1 | 5.827e-08/5.827e-08/5.827e-08 | 2.876e-08/2.876e-08/2.876e-08 | 1.398e-07/1.398e-07/1.398e-07 |
| PERSISTENCE_FALLBACK->FEASIBLE_FALLBACK | 2 | 1.778e-07/2.862e-07/2.982e-07 | 3.770e-07/7.016e-07/7.376e-07 | 7.232e-07/1.343e-06/1.412e-06 |
| PERSISTENCE_FALLBACK->SIDE_PLUS | 3 | 2.959e-07/3.205e-07/3.232e-07 | 4.995e-08/9.175e-08/9.639e-08 | 5.560e-08/3.020e-07/3.294e-07 |
| SIDE_PLUS->NOMINAL | 4 | 1.426e-07/4.574e-07/5.104e-07 | 3.527e-08/4.440e-08/4.467e-08 | 3.094e-08/1.420e-07/1.597e-07 |
| NOMINAL->SIDE_PLUS | 7 | 1.738e-07/1.444e-06/1.534e-06 | 1.840e-08/3.706e-08/4.334e-08 | 2.450e-08/1.239e-07/1.559e-07 |
| SIDE_PLUS->SIDE_PLUS | 3 | 4.438e-07/4.609e-07/4.628e-07 | 3.260e-08/5.313e-08/5.541e-08 | 6.114e-08/7.192e-08/7.311e-08 |
| SIDE_PLUS->JOINT_TEAM_SOLUTION | 1 | 3.386e-07/3.386e-07/3.386e-07 | 6.428e-08/6.428e-08/6.428e-08 | 3.272e-07/3.272e-07/3.272e-07 |
| NOMINAL->SIDE_MINUS | 4 | 6.437e-08/1.175e-07/1.247e-07 | 5.312e-08/8.025e-08/8.193e-08 | 1.487e-08/8.695e-08/9.878e-08 |
| SIDE_MINUS->NOMINAL | 4 | 1.060e-07/2.986e-07/3.300e-07 | 4.510e-08/9.216e-08/9.530e-08 | 2.935e-08/4.615e-08/4.734e-08 |
| FEASIBLE_FALLBACK->NOMINAL | 1 | 5.200e-08/5.200e-08/5.200e-08 | 3.420e-08/3.420e-08/3.420e-08 | 1.920e-07/1.920e-07/1.920e-07 |

共同未来最多 1 s 的曲线最大空间偏离：P50=0.039488 m，P95=0.461028 m，max=1.323575 m。最大项为 mission 40.341379 s，UAV0，PERSISTENCE_FALLBACK（原 FEASIBLE_FALLBACK id131）→NOMINAL id132；接管头仍满足 C2。**未来曲线仍允许明显改变，未承诺两代未来形状相同**；这一项不能说已经消除，也不能把它等同于执行头硬切。

PositionCommand 与对应 polynomial 速度差 mean=0.000006857、P95=0.000009030、max=0.001663650 m/s。最大值含 command header 时间与该 tick 求值时刻的微小差别，未发现足以解释严重速度突变的执行器/系数不一致。

## 9. 团队链、RViz 与“飞一段—停—再飞”

Proposal=50，commit=46，planner adoption=138；实际执行 payload 逐 id 证实 **46 个 team solution 均三机实际激活**，optimized yaw actual execution=138。第一次未 commit 的问题没有通过关闭 team optimizer 回避。

两次均启动 native RViz；配置路径为 `ros_ws/src/multi_uav_formation/config/native_egov2.rviz`。最终窗口包含 UAV、target、obstacle、planned trajectory、odom path / FOV 显示项。XWD 留存了窗口/配置，但未取得 OpenGL 子窗口画面，不能声称该截图证明肉眼观察的飞行连续性。本轮关于运动的判定以 logs + commands + odom 为准：没有 terminal hold；除 mission 起步约 0.067–0.360 s 外，没有 speed<0.15 m/s 持续≥0.2 s 的段。**未再观察到任务中“旧曲线耗尽→停住等待→重启”行为**。

[全程实际 activation 与速度图](/home/bob/ALP/egov2_fc65423_constvel/trajectory_lifecycle_fix_verified_20260910/analysis/lifecycle_execution.png)；没有把 RViz 稀疏 marker 作为根因解释。

## 10. 安全结果及未完成项

| 指标 | 值 |
| --- | --- |
| MAX_ODOM_SPEED m/s | 3.196349 |
| MAX_COMMAND_SPEED m/s | 3.063448 |
| MAX_PLANNER_TRAJECTORY_SPEED m/s | 3.063454 |
| MIN_SWARM_DISTANCE 欧氏 m | 0.649390 |
| MIN_SWARM_DISTANCE 椭球 m | 0.474611 |
| MIN_STATIC_CLEARANCE m | 0.172766 |
| MIN_DYNAMIC_CLEARANCE m | 0.912044 |
| COLLISION_SAMPLES | 0 |
| SWARM_CLEARANCE_VIOLATION_SAMPLES | 6 |
| EMERGENCY_STOPS | 0 |
| HEARTBEAT_LOSSES | 0 |
| UNVALIDATED_EXECUTED_SAMPLES | 4 |
| UNVALIDATED_NEW_POLYNOMIAL_ADOPTIONS | 0 |
| ACTIVE_TRAJECTORY_INVALIDATED | 5 |
| VALIDATED_BRAKE adopted / BRAKE_UNAVAILABLE | 5 / 3 |

clearance 口径与上一完整审计一致：点到原始 static/dynamic 表面的距离；collision 为该距离≤0。机间使用 `sqrt(dx²+dy²+0.25*dz²)<0.5 m`，每 pair/采样记一次，不以欧氏距离掩盖椭球违规。planner 最大速度是已采用完整 polynomial 的解析速度极值，可能包含未执行的远端；command/odom 最大值来自同一完整有效运行窗。

**REMAINING_ROOT_CAUSE_1 — CONFIRMED：失效且无可行 brake 时的执行替代闭环仍有缺口。**

- UAV1 id140 在 `1789047298.250772` 发现 swarm invalid，短前缀也 invalid；两次 BRAKE_UNAVAILABLE；到 `1789047298.411980` 才由 id141 真正接管，约 0.161209 s。
- UAV0 id144 在 `1789047298.337192` 发现 swarm invalid at t=0；BRAKE_UNAVAILABLE；到 `1789047298.481553` 才由 id145 接管，约 0.144361 s。
- 4 个 safety_validated=False 样本是 UAV0 id144/PERSISTENCE_FALLBACK，`1789047298.377308–1789047298.476990`。该计数不能证明其他 invalidated 区间不存在延迟改标。所有新 polynomial 的初始 validation=true，并不等于此前已失效 active 会立即停止执行。
- 源码入口：[planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1000) 的 brake 全部失败后返回 false；FSM urgent replan；traj_server validation expiry 是遥测而非独立安全替代触发器。不能通过直接置零命令“取消”来绕过 P/V/A 和制动安全验证。

**REMAINING_ROOT_CAUSE_2 — CONFIRMED / 尚未唯一分离全部贡献：实际跟踪偏差与异步 peer 更新仍能导致近距离违规。**

6 个违规样本位于 mission 47.759860–47.926830 s，UAV0 id143→144 与 UAV1 id140；无 terminal hold。第一违规时刻参考 polynomial 的椭球距离约 0.570693 m，实际为 0.494491 m，说明仅验证参考曲线不足以保证实际 clearances。随后 active prefix 对更新后的 peer 也判 invalid，说明不能只归为渲染或单条 MINCO 内部连续性。peer 的 future-scheduled 预测/更新顺序仍值得独立闭环核查；本轮只统一了 expiry model，没有声称已经为并发 local successors 建立完整安全预约。

**REMAINING_ROOT_CAUSE_3 — CONFIRMED：速度约束现有 5% feasibility 容差 + odom overshoot。**

原配置 max_vel=3、feasibility_tolerance=0.05 未改，生产 dynamics gate 因此允许参考速度到 3.15 m/s。最终 polynomial/command 约 3.06345 m/s，低于既有验收上限，但高于名义 3 m/s。最高 odom 出现在 mission 38.693564 s，UAV1、PERSISTENCE_FALLBACK id125；相邻 command≈3.049758 m/s，odom≈3.196349 m/s。跨代 P/V/A 误差极小，不能再把这全部解释为轨迹头硬切。没有降低速度、clearance、yaw、Local-SFC 或 activation margin 来消除告警。

最小后续修复方向是：在 active 仍有可验证安全前缀时准备并保留可执行的制动 successor；把 queued successor 的最新 peer 检查、撤销与实际 adoption 对齐；只有存在已验证替代时才切换执行许可；把实际跟踪误差纳入 safety 预测/约束。名义 3 m/s 与现有 5% 容差也需要明确执行验收语义。这些不能用冻结 generation、重引入120°或改 visibility 目标代替。**本轮保留失败证据，不继续重跑寻找无违规的一次。**

## 11. 其他全程统计

| Trajectory source | 实际单机样本数 | 占比 |
| --- | --- | --- |
| NOMINAL | 4489 | 62.218% |
| JOINT_TEAM_SOLUTION | 995 | 13.791% |
| PERSISTENCE_FALLBACK | 1505 | 20.859% |
| VALIDATED_BRAKE | 9 | 0.125% |
| SIDE_PLUS | 93 | 1.289% |
| SIDE_MINUS | 102 | 1.414% |
| FEASIBLE_FALLBACK | 22 | 0.305% |

| 性能 ms | count | mean | P95 | max |
| --- | --- | --- | --- | --- |
| generator | 115 | 9.975443 | 12.907700 | 14.706000 |
| joint | 98 | 4.038408 | 5.665600 | 8.955000 |
| hypothesis_planning | 896 | 10.488170 | 20.677250 | 828.110000 |
| candidate_planning | 1574 | 5.943817 | 11.546050 | 828.095000 |

Viewpoint generator 仍为毫秒级；规划批次有一次 828 ms 长尾，本轮的 reserve 与 final reanchor 处理其真实耗时，没有把这一尾部当作 1 s FSM pending 的理由。

Executed visibility 使用共同有效区间、实际 odom/yaw、range AND static LOS AND dynamic LOS AND camera FOV，左端保持按真实时间积分并裁剪末端。camera-time=205.720919 camera·s；mean_visible=2.565803；K2=0.953014；All3=0.620767；None=0.007977；longest K2 loss=1.400414 s；longest blackout=0.639611 s。没有由本轮两次时序修复运行声称 visibility 提升。

## 12. 最终验收标记

PERSISTENCE_EXPIRY_MANAGED=NO 表示完整 persistence 失效状态闭环未通过；到期预警、正常接续和零 hold 已实测生效。LARGE_SUCCESSIVE_TRAJECTORY_JUMP_OBSERVED=YES 指共同未来曲线仍有明显几何变化，不代表头部 P/V/A 硬切。

```text
PRODUCTION_SOURCE_CHANGED: YES
PENDING_NO_LONGER_BLOCKS_LOCAL_REPLAN: YES
ZERO_EXECUTABLE_PENDING_FIXED: YES
FUTURE_ACTIVATION_STATE_UNIFIED: YES
ACTIVE_TO_NEW_PVA_GATE_ACTIVE: YES
PERSISTENCE_EXPIRY_MANAGED: NO
PLANNER_TRAJ_SERVER_END_MODEL_UNIFIED: YES
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 0
TERMINAL_HOLD_COUNT: 0
TERMINAL_HOLD_TOTAL_DURATION: 0 s
ACTUAL_ACTIVATION_RATE_UAV0: 2.980868 Hz
ACTUAL_ACTIVATION_RATE_UAV1: 3.105591 Hz
ACTUAL_ACTIVATION_RATE_UAV2: 3.317619 Hz
HANDOFF_DP_MAX: 3.235342584e-06 m
HANDOFF_DV_MAX: 7.694649006e-07 m/s
HANDOFF_DA_MAX: 6.385413775e-06 m/s^2
VISIBLE_STOP_AND_RESTART_BEHAVIOR_OBSERVED: NO
LARGE_SUCCESSIVE_TRAJECTORY_JUMP_OBSERVED: YES
SAFETY_REGRESSION_OBSERVED: YES
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES
SCENARIO_A_FULL_RUN_COMPLETED: YES
RVIZ_ENABLED: YES
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

本轮没有覆盖旧 feedback、没有提交 Git commit。两次仿真、测试 ROS master、RViz 均已结束，最终进程核验无残留。

REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_30.md
