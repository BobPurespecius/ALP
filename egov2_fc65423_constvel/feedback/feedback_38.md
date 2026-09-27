# 持续共同时间轨迹参考：实现完成，完整场景验证未达标

**不能将本轮交付称为已验证成功的协同版本。** 最终源码已实现持续 TEAM_REFERENCE 接线，指定构建及20项测试入口通过；但两次完整 Scenario A 都没有形成三机 commit。必要复验还出现严重退化：合围16.90%、静态接触采样1934、terminal hold 4.58秒。最后发现的消息到达顺序问题已经修复并通过行为测试，**该最终修复没有再进行完整 Scenario A 验证**，因为本轮完整运行上限已用完。两次失败数据全部保留，没有第三次仿真、OFF或调参。

## 1. 最终源码中的唯一团队参考

沿用 coordinator 的 `/topology_coordination/team_solution` 和现有 TeamTrajectorySolution PROPOSAL→3 ACK→COMMIT。COMMIT 的真实 team_solution_id 是参考版本，activation_time 是原共同 world-time epoch，三机 MINCO position polynomial、原 optimized yaw trace 组成只读软参考。没有新增 topic、审批协议、参考发布者或恢复状态机。

planner 实际接受 team 接管后保留该参考；后续 local generation 仍使 pending proposal 失效，但不删除参考。位置参考在原三条 polynomial 的共同有效尾端到期，不平移 epoch、不续期。原 yaw trace 只在其本来覆盖的时间内使用，超过 yaw 尾端回到原 target-facing yaw；不把1.5秒 yaw/评价窗口解释为合围完成期限。

Local 优先用本机参考剩余段初始化 MINCO，头部重新取本次 activation 的真实前驱 P/V/A。原 Guide 跟踪代价位置只使用一个来源：有 team reference 时为绝对世界位置的平方跟踪误差（沿用原 local_target_weight=4），否则为原 timed target-relative Guide。J_enc、J_vis、J_acc、K2/blackout、radius/height bands 及既有安全偏好保留。

## 2. Local 独立站位如何收敛

此前 `callReboundReplan` 每轮重新读取 encirclement hypotheses，重新选择观察方位；team 只短时接管。现在有效参考期间以 team polynomial 终端及剩余路径初始化，不再进入独立站位假设循环。NOMINAL/SIDE/A*/Local-SFC 原候选链、失败 fallback 和持续 rolling 仍保留。

抽取同一个 captured-candidate finalize→绑定实际提交曲线→发布候选束入口，供普通假设和 team follower 共用。这样跟随 team 时仍向 joint 提供实际采用的 local 候选，不会断掉下一轮团队输入。

PolyTraj 仅增加 `team_reference_id`、`team_reference_reason`。local 修改后的轨迹仍标记 NOMINAL/SIDE/FEASIBLE_FALLBACK/VALIDATED_BRAKE 等真实 source，不冒充 JOINT_TEAM_SOLUTION。原位置、yaw、generation、activation 与安全标记仍原子传输。偏离诊断区分滚动修正、障碍绕行、可行初始化 fallback 和 brake；这些是事件元数据，不是执行资格。

## 3. Team 更新、执行快照与两次缺陷修复

继续使用原 joint P/T/yaw 优化器。旧 team 的剩余内部路径和时间用作可选 warm start；保留当前候选尾部契约，重置头部到实际已提交 local 的共同 activation P/V/A。warm start 不改变收益比较基线。候选优化仍经过原安全/收益检查，再使用同一 camera/geometry/benefit 工具与实际 local 组合比较。

首次完整运行发现：新 team head 已从实际前驱重建，但旧 ACK 仍要求 head 等于未必执行的 source candidate suffix。69个窗口内 proposal 没有形成 commit。修复为同样的1e-5边界容差检查实际前驱 P/V/A，候选尾部 P/V/A 检查不变；新增实际头部/旧候选头部/尾部变化的生产回归测试。

第二次完整运行仍有9个 proposal、0 commit。逐载荷核对发现，候选束在对应执行广播 callback 之前到达时，coordinator 使用了前一代 execution buffer。最终修复改用候选束中已绑定 post-check 实际 local 提交的 selected candidate，保留 pending.active_generation 身份契约；较新的执行广播仍能立即否决旧候选束，较旧广播不能覆盖当前束的头部。新增了 callback 先后顺序、更新排程否决和不可提交候选拒绝测试。

**最后这项修复在第二次完整运行之后完成，仅完成构建与行为验证。** [运行二进制 hash](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/run_build_manifest.json) 与 [最终二进制 hash](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/build_manifest.json) 分别留存；[版本区别](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/run_final_separation.json) 明确记录不同，不把运行结果冒充最终源码结果。

## 4. 保留的连续性与安全机制

已 ACK 的短期接管预约和 scheduled predecessor 顺序保持；没有新增 pending 等待 return、无几何进展停飞、无限 hold 或 local 等团队批准的分支。peer buffer 保存排程 successor 前的实际前驱，按 world-time activation 边界采样安全和几何。软 team reference 不写入 peer 执行安全 buffer。

原 dynamics/static/dynamic/swarm、Local-SFC、前驱安全前缀、actual-active→new P/V/A、generation/identity/activation、coverage/deadline 检查均保留。[八个生产安全函数逐体一致检查](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/unchanged_safety_functions.json) 全 true，包括 brake、coverage、deadline 和实际接管验证。安全距离、速度/加速度/jerk/yaw 限制、P/V/A 容差、activation margin、brake策略未调。25°/170°仍是真实合围标准，120°仍仅seed。

这些源码契约不等于完整场景行为安全已经通过；复验的实际失败如下，不能用“检查保留”掩盖。

## 5. 构建与测试

`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws` 最终6/6包成功，保持原O2/O3，无fast-math。仅有原 `base_delta_phi` unused-variable warning。

全部20项测试入口最终通过，原 unit/gradient/contract/executor suites 保留。新增生产 local 连续3轮×3机候选生成、参考身份/epoch持久、fresh P/V/A seed、无参考候选供给、ACK预约保护、陈旧pending、排程peer冲突与消息乱序快照测试。原 joint 全目标有限差分及 yaw/commit 契约通过。

新增生产 follow component 位置/世界时间有限差分最大误差1.28832e-10；完整 `trackingGradCostP` 新接线的当前时间/前段时间导数误差4.03511e-10 / 9.75798e-11。joint warm-start 测试确认 fresh heads 和原 local 基线不被替换。

过程记录：第二次启动前，生产行为测试已通过，但一个旧 source-text wiring 断言仍要求旧候选head表达式；运行脚本未以失败码中止而启动了复验。这是不符合“全测试通过后再运行”的流程遗漏。该断言在复验开始后改为检查真实前驱P/V/A、源尾部及六项原1e-5容差，保留等价生产行为覆盖；原失败日志为 [旧wiring失败日志](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/lifecycle_wiring_before_update.log)。最终完整20项suite在最后源码上重新全部通过，见 [最终测试结果](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/tests.json) 和 [最终测试运行日志](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/test_snapshot_fix.log)。没有将旧失败删除或称为提前全部通过。

## 6. 两次完整 Scenario A

均为 long_cylinder_forest_visibility_stress.json、FULL ON、native RViz，target 路线80.204541秒正常结束。仿真进程/库来自本 workspace，当次运行hash已核对；最后源码与当次运行的区别见上文。

第一次窗口80.174940秒：合围62.98%、同半平面20.37%，camera-time231.856634 camera·s、mean_visible2.891884；collision/terminal hold/未验证执行采样均0，但窗口内69 proposal、0 commit，未建立共同参考。完整日志与数据在 [第一次失败数据](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/failed_attempt_1/run)。首次新 recorder 对 ROS uint8[] 的 bytes 序列化失败，因此原始 TeamTrajectorySolution 文件为空；其proposal/ACK/abort可由完整ROS日志审计。复验前已修复 bytes→整数列表，并实际测试消息序列化；没有伪造第一次缺失载荷。

第二次采用统一 recording交集起点→target route结束窗口，时长80.130934秒：

| 指标 | 完整复验结果 |
|---|---:|
| 共同参考实际覆盖 | 0秒 / 0% |
| 各机参考覆盖及非JOINT沿参考覆盖 | 均0秒 |
| team proposal / commit / 三机实际采用 | 9 / 0 / 0 |
| 实际合围 / 同半平面 | 16.90% / 64.98% |
| g_min P50 / P95 / 最小 | 29.581° / 93.425° / 0.060° |
| g_max P50 / P95 / 最大 | 198.395° / 233.716° / 315.132° |
| camera-time / mean_visible | 165.347247 camera·s / 2.063463 |
| K2 / All3 / None | 88.80% / 17.85% / 0.29% |
| 最长K2 loss / blackout | 5.634491s / 0.199605s |
| local实际activation UAV0/1/2 | 3.257 / 4.393 / 4.493 Hz |
| 排除brake后的activation | 0.624 / 3.669 / 4.143 Hz |
| terminal hold | 1次，4.580423s，UAV1 |
| 前轨迹提前耗尽 | 1次，gap约4.581s |
| P/V/A residual最大值 | 3.59e-06m / 3.23e-07m/s / 1.01e-06m/s² |
| collision / swarm violation采样 | 1934 / 0 |
| 未验证执行遥测采样 / 未验证polynomial adoption | 137 / 0 |
| planner / command / odom速度峰值 | 3.146967 / 3.146935 / 3.392047 m/s |
| brake实际activation / 执行时间占比 | 297 / 34.41% |
| 最长低速段（odom<0.2m/s） | 62.429690s |

1934为按原 static_clearance_m<=0 口径的采样数，均在UAV0；不是1934次独立碰撞事件。首次接触采样在任务15.743808秒，静态clearance为0。137个未验证执行遥测样本与terminal hold对应；没有未验证polynomial被adopt。名义3m/s超速仍存在，没有改阈值消除告警。Brake策略未改，不能把次数或退化归因给某一个改动；但该完整运行明显未满足任务的连续性、安全和协同成功标准。

## 7. “两前一后”在哪一层

[按world-time对齐的完整CSV](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/analysis/team_local_actual_world_time.csv) 包含每机reference id和team/local/actual的gap时间序列。**没有已接受team版本，因此team曲线缺失、team-reference跟踪误差及偏离原因时长均为N/A，不能用JOINT source占比替代覆盖率，也不能编造三层共同版本比较。** [对齐统计](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/analysis/team_reference_alignment.json) 和 [时间序列图](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/analysis/team_local_actual.png) 保留了这个缺失。

以目标速度方向投影正负定义“两前一后”（仅分析，不进入生产目标）：actual出现57.148233秒，local提交曲线出现57.739022秒。进一步以原25°界限标记前方两机bearing靠近，actual有20.372222秒，这些采样同时已出现在local曲线上；没有“只在odom出现、local未出现”的对应采样。

因此这次失败复验里的该布局主要已经存在于local规划/持续brake状态中，不能只归咎command/odom没跟上。UAV0存在长达62.43秒低速段。因为共同计划从未commit，**无法据此判断已接受的团队目标函数是否长期允许同样布局**，也没有增加舒适区、120°奖励或信息矩阵掩盖问题。

## 8. 留档、清理和剩余工作

修改前真实工作树源文件备份、diff/hash与baseline测试在 [baseline](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/baseline)；当前17个修改/新增文件见 [源码差异](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/changes.diff)、[文件清单](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/changed_files.json)。当前说明同步到 `ros_ws/src/multi_uav_formation/RECOVERY_THREE_LAYERS.md`。没有reset或覆盖用户历史修改；只使用一个agent，禁止目录未访问/修改。

复验完整PolyTraj、PositionCommand、odom、Target/Guide、TeamTrajectorySolution、activation/source/reference id、ROS日志和native RViz窗口截图在 [复验原始数据](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/run)；[两次原始数据hash](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/run_hashes.json) 已保存。[清理核对](/home/bob/ALP/egov2_fc65423_constvel/team_reference_20260911/cleanup.json) 显示本轮ROS/RViz/recorder/test进程均已退出，12583/12584端口关闭，未处理其他用户进程。

剩余项明确：最终callback顺序修复尚无完整运行验证；共同参考持续兑现未被运行证实；第二次场景中的静态接触、耗尽/terminal hold、brake占比和超速仍未解决。本轮不能宣称已达成“团队共同决定、local连续兑现”的目标，也没有扩大到重设计brake或几何目标。
