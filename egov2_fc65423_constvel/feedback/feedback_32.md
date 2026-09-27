本轮已把 local 几何保持/恢复判定接入最终执行选择，并修复有限制动覆盖不足及 objective 内循环无法及时取消的问题。最终完整 Scenario A：严格实际合围 **57.68%**，同半平面 **28.05%**，camera-time **230.493691 camera·s**，**0 terminal hold / 0 collision / 0 swarm violation / 0 unvalidated samples**。三机实际接管约 3.280200 / 3.629423 / 3.554589 Hz。

相对 feedback_31，严格合围提高15.03个百分点，同半平面减少5.06个百分点，camera-time增加约3.39%。但仍有42.32%的时间未满足严格25°/170°条件，最长退化段9.60s；K2降低约1.14个百分点，名义3m/s超速仍存在。**不能把本轮结果解释成全程严格合围、全项安全验收通过或统计显著性结论。**

**源码与执行契约**

真实生产链保持 rolling local planner → asynchronous team enhancement → 原 proposal/ACK/commit/adoption → traj_server。没有重新引入固定120°恢复项，没有修改 AdaptiveViewpointGenerator、J_vis/J_acc/K2/blackout 的目标定义及权重，没有改变场景、安全距离、v/a/jerk、yaw、Local-SFC、generation 或 activation margin。

主要改动：

| 文件/函数 | 最终行为 |
| --- | --- |
| plan_manage/include/plan_manage/local_execution_contract.h（新增） | 共享的 local 三态判定、保持/进展比较、时间衰减最大耗时预算 |
| planner_manager.cpp::evaluateLocalGeometry | 复用 encirclement_geometry.h，在同一世界时刻预测实际 active、各候选和 peer；输出 ratio、双 gap violation、max/min |
| planner_manager.cpp::finalizeCapturedCandidates / finalizeCapturedLocal | 汇合当前及其他 hypothesis 的所有安全 NOMINAL/SIDE/FEASIBLE 候选；统一未来 head，完整安全预检后比较；兼容 wrapper 也委托此入口 |
| planner_manager.cpp::geometryRecoveryReplanDue / logExecutionGeometry | 对 remaining suffix 计算 geometry；退化时提前触发恢复，不取消原 persistence 安全复验；发布 source/id/generation/state/remaining 等遥测 |
| planner_manager.cpp::ensureExecutionCoverage / finishPlanningBatch | 成功和失败均计入预算；solve 前确认真实已验证覆盖，必要时先提交安全制动；有限预算的 nominal 恢复不启动 SIDE/多 hypothesis refinement |
| planner_manager.cpp::validatedBrakeSuccessor | 真正执行的制动 polynomial 时长覆盖预算；不再把另行验证、没有发布的1s hold算成覆盖；全段动态/静态/swarm/Local-SFC/actual-active PVA检查 |
| ego_replan_fsm.cpp::callReboundReplan / execFSMCallback | 覆盖检查先于昂贵规划；RAII记录完整 planning batch；local 多 hypothesis 选择先于 team bundle；pending不冻结local |
| encirclement_geometry.h::encirclementGeometryCost | 在原 g_max helper 上增加真实 g_min 违反项及解析梯度；合法非120°分布仍为零代价 |
| poly_traj_optimizer.cpp::trackingGradCostP | local 恢复方向同时包含 min/max gap，保持原weight=100和时间链式梯度 |
| poly_traj_optimizer.cpp::optimizeTrajectory / costFunctionCallback / quadrature loops | deadline覆盖LBFGS、SCP、单次objective积分；取消整次未完成evaluation，恢复有限initializer，保留已执行safe successor |
| scp_optimizer.cpp::solve / PolyTrajOptimizer::solveExecutionQP | 进入QP前检查reserve，给支持PROFILING的OSQP设置time_limit；数值精度与安全阈值不变 |
| tracking_visibility_geometry.h::advanceTargetFacingYaw | 修复达到yaw rate上限后遗漏基础积分项的问题；预测与执行共用的同一修复 |
| native_egov2_rviz.launch | 暴露local geometry eps、horizon、urgent period和execution budget参数 |
| tests / CMakeLists.txt | 新增local状态、预算、取消、yaw饱和、双gap梯度契约；旧接口检查跟随真实入口调整 |

完整文件列表与差异：[changed_files.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/changed_files.json)、[production_changes.diff](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/production_changes.diff)。修改前生产快照保留在 local_recovery_reserve_20260911/baseline；未重置或覆盖工作区其他历史改动。

**Local geometry 语义**

仍由共享 helper 按实际 XY bearing 排序，满足 `g_min>=25° && g_max<=170°` 才算合围。每个候选和当前active suffix在同一 `activation + t` 比较，near horizon=1.5s、dt=0.10s，使用区间中点等时权重；peer到期沿用既有terminal-position模型，target使用带时间戳的状态外推。不存在固定UAV编号顺序或120°偏差恢复项。

`J_geometry = mean([g_max-170°]_+² + [25°-g_min]_+²)`，角度计算单位rad。Geometry是最终选择的明确条件，soft cost仅提供优化方向。

默认ROS参数 `/encirclement_geometry/`：

| 参数 | 值 |
| --- | --- |
| min_angular_separation_deg / max_circular_gap_deg | 25 / 170 |
| ratio_min | 0.8，正常team原判据不变 |
| local_eps_ratio / local_eps_progress | 0.02 / 0.02 |
| local_eps_violation / local_eps_geometry | 0.0025 / 0.0025 rad² |
| local_horizon / local_dt | 1.5 / 0.10 s |
| urgent_replan_period | 0.15 s |
| local_gap_weight | 原100，未调大 |

`NORMAL_ENCIRCLEMENT`：候选达到normal ratio，并相对已正常baseline保持ratio及violation；当safe previous仍有充分寿命且新候选均破坏正常几何，优先保留safe previous继续规划。

`RECOVERING_ENCIRCLEMENT`：baseline已退化，候选ratio增加超过eps_progress，或双gap violation下降超过eps_geometry；达到正常ratio则标NORMAL。退化候选不必瞬间达到80%。同一进展等级内，显著ratio/violation改善优先，再比较binary camera-time和native cost。

`SAFETY_DEGRADED`：没有当前可用的安全保持/恢复候选时，允许已通过完整安全检查的执行方案继续；明确记录ENCIRCLEMENT_DEGRADED，并保持urgent recovery。它是枚举候选集合的降级结果，不声称已证明环境中所有可能恢复轨迹都不存在。该状态不能被统计为合围成功。

NOMINAL / SIDE_PLUS / SIDE_MINUS / FEASIBLE_FALLBACK都经过共同preflight和选择；PERSISTENCE保留原suffix复验并增加geometry/lifetime复查；正常JOINT仍保留原ratio>=0.8非线性guard。actual-active→new的0.02m / 0.05m/s / 0.10m/s²门限原样保留。

候选可见性排名复用从原fillTopologyCandidateMessage抽取的 `fillCandidateBinaryVisibility`，没有沿用旧local诊断中缺FOV的LOS-only分数。定义仍是 range AND static LOS AND dynamic LOS AND camera FOV；预测yaw同源。其他两机在一次本机候选比较中的预测不变，因此最大化本机binary camera-time等价于最大化该snapshot的team camera-time。没有另建visibility定义。

**Execution reserve 与取消**

`B(now)=clip(factor * decayed_recent_max, B_min, B_max)`，最近慢solve包括失败；peak按墙钟60s半衰期衰减，快速solve不会直接抹掉长尾。默认 `/trajectory_lifecycle/`：initial=1.10s、min=0.20s、max=2.50s、factor=1.35、half_life=60s、execution_margin=0.15s、uninterruptible_budget=0.05s。

启动完整batch前必须有 `verified_coverage >= B + 0.10s activation + 0.15s execution`。预期head用的短lead与执行覆盖的worst-case预算分离；最终仍按真实future activation重新构造head并做P/V/A gate。

若不够，先验证并发布覆盖预算的制动；制动基础时长至少 `max(0.6, B+activation+execution+0.25)`，有限scale/stop-distance备选全部经过原安全验证。已提交的未来local successor先等待真实adoption，随后才开始新一轮solve。新制动不绕过MINCO的正常生成/广播模型，不靠额外未发布hold延长寿命。

长制动不可用时，仅在还有0.20s bounded nominal求解预算及完整余量时允许一次受deadline约束的nominal恢复，禁用该batch的SIDE/多hypothesis refinement；覆盖再不足则不启动昂贵solve，继续urgent safe-successor尝试。保持安全优先，不执行不安全几何恢复。

LBFGS iteration、SCP/QP entry之外，两个可变长度visibility积分也每64个样本检查deadline；固定PVA积分按piece检查。取消后返回“整个objective无效”，不会接受部分积分。LBFGS按原返回路径释放手工分配的内存，外层随后恢复原有限initializer，避免把巨大trial trajectory继续送到后续risk循环。这些检查不改变成功完成evaluation时的目标或梯度。

最终运行真实触发一次取消：UAV2在mission18.711s开始batch，coverage=3.392824s，budget=0.884591s，deadline=1789058742.289638。取消完成于1789058742.281003，总batch876.417ms，partial_cost_accepted=0；预算提高到1.183163s且后续快速solve没有清零。旧id72持续执行，随后id73在1789058742.654632接管，远早于旧轨迹末端。证据：[deadline_execution_proof.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/analysis/deadline_execution_proof.json)。

**有明确原因的三次验证**

没有OFF、A/B、场景调参，也没有为挑选漂亮结果重复相同版本。

| FULL ON运行 | 合围ratio | same-semicircle | camera-time | terminal hold | 后续动作 |
| --- | --- | --- | --- | --- | --- |
| local_recovery_reserve_20260911 | 0.744607 | 0.128009 | 205.324747 | 0 | 定位UAV1 yaw rate饱和后heading冻结，修复积分缺项 |
| local_recovery_reserve_yawfix_20260911 | 0.707109 | 0.177052 | 218.880420 | 1次/8.506048s | 定位单次objective约660万样本绕过iteration deadline，补齐内循环取消 |
| local_recovery_reserve_deadline_20260911（最终生产版本） | 0.576815 | 0.280477 | 230.493691 | 0 | 验证结束后停止，未做第四次运行 |

第一轮UAV1 command yaw连续23.9505s不变，却一直报告yaw_dot=2π。共享yaw函数的饱和分支只积分了rate增量；当input rate已经等于limit，原式给max_step=0。补回 `input.yaw_rate*dt` 后，原rate/acceleration limits不变，正常target-facing yaw持续转动。最终三个UAV HFOV/VFOV loss均为0，饱和冻结事件为0。这是执行/预测积分bug修复，不是重新设计yaw objective。

第二轮失败发生在mission58.397–66.903s，单batch9779.757ms、terminal hold8.506048s、256个未验证hold样本。单次visibility积分耗时约8.704s，迭代外deadline来不及运行。失败证据保留在 [deadline_root_cause.log](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_yawfix_20260911/analysis/deadline_root_cause.log)。

第一轮测试runner因rosout轮转丢失startup marker而延迟清理约35.23s；通过末端target位置/零速度验证正常结束，统计未包含额外停留。之后runner保存mission start并读取轮转日志，最终将昂贵chain重建移到离线，最终target结束后约0.452s即发起正常清理。三轮数据分别保留，报告下列主统计只使用最终生产版本。

**最终完整运行口径与结果**

场景 `long_cylinder_forest_visibility_stress.json`，FULL ON + native RViz；route正常完成，用时80.204540745s。统一有效统计区间 `[1789058722.720362902, 1789058802.898406744]`，RUN_DURATION=80.178043842s；从三机完整odom/visibility数据的首个有效时刻到target路线结束，未剔除退化、遮挡、低速或异常片段。按实际dt左端保持积分，角gap分位数按时间加权。实际合围来自三机odom+target重算，不使用predicted ratio替代。

| Geometry | 值 |
| --- | --- |
| EXECUTED_ENCIRCLEMENT_RATIO | 0.576815 |
| SAME_SEMICIRCLE_RATIO (`g_max>=π`) | 0.280477 |
| GAP_MIN P50/P95/min | 55.057242 / 96.811409 / 18.653792 deg |
| GAP_MAX P50/P95/max | 166.940182 / 215.889171 / 316.039275 deg |
| NORMAL_ENCIRCLEMENT_RATIO | 0.567877 |
| RECOVERING_ENCIRCLEMENT_RATIO | 0.002109 |
| SAFETY_DEGRADED_RATIO | 0.430014 |
| DEGRADED_EPISODE_COUNT | 20 |
| DEGRADED_EPISODE P50/P95/max | 0.071246 / 4.850409 / 9.599991 s |
| RECOVERY_SUCCESS_COUNT | 19 |
| RECOVERY_TIME P50/P95/max | 0.067651 / 5.100387 / 9.599991 s |

三态时间占比按真实正在执行的trajectory id关联该轨迹最新geometry事件：三机全NORMAL才是团队NORMAL；任一SAFETY_DEGRADED则团队SAFETY_DEGRADED；其余存在RECOVERING则团队RECOVERING。无UNKNOWN缺口。该保守聚合不同于实际odom严格geometry比例；RECOVERING说明该轨迹被采纳时通过进展比较，不是声称当前三机已合围。逐UAV三态占比和完整事件序列见 [recovery.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/analysis/recovery.json)。退化episode按实际geometry逐帧连段，无人为去抖；19次恢复中包含边界附近短过渡，最后一次到mission结束仍未恢复。

| Executed visibility | 值 |
| --- | --- |
| ACCUMULATED_CAMERA_VISIBLE_TIME | 230.493691 camera·s |
| MEAN_VISIBLE | 2.874773 |
| K2 | 0.983155 |
| ALL3 | 0.891989 |
| NONE | 0.000371 |
| LONGEST_K2_LOSS | 0.898946 s |
| LONGEST_BLACKOUT | 0.029785 s |

| UAV | visible ratio | static LOS loss | dynamic LOS loss | HFOV loss | VFOV loss | range loss |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 0.926972 | 4.695702 | 1.159576 | 0.000000 | 0.000000 | 0.000000 |
| 1 | 0.983023 | 0.560015 | 0.801171 | 0.000000 | 0.000000 | 0.000000 |
| 2 | 0.964779 | 0.891031 | 1.932946 | 0.000000 | 0.000000 | 0.000000 |

损失时间单位s，可重叠，不应相加当作互斥原因。camera-time和All3提高，但K2及其最长loss逊于feedback_31；不能说所有coverage指标均改善。

| UAV | replan Hz | candidate Hz | commit Hz | actual activation Hz | activation间隔P50/P95/max s |
| --- | --- | --- | --- | --- | --- |
| 0 | 4.614730 | 7.408512 | 3.292672 | 3.280200 | 0.269865 / 0.651060 / 1.310129 |
| 1 | 3.554589 | 6.348371 | 3.616950 | 3.629423 | 0.269982 / 0.355320 / 1.129748 |
| 2 | 3.891340 | 6.672650 | 3.542117 | 3.554589 | 0.270013 / 0.336848 / 2.300034 |

统计窗口边缘的commit和activation可能分处两侧，不能强行把次数改成一致。后台coordinator timeout=3，stale head=0，zero-executable pending=0；约3Hz滚动执行没有退回1秒pending冻结。

| Lifecycle / performance / safety | 值 |
| --- | --- |
| TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT | 0 |
| TERMINAL_HOLD_COUNT / TOTAL | 0 / 0.000000 s |
| VALIDATED_BRAKE_COUNT / BRAKE_UNAVAILABLE_COUNT | 14 / 0 |
| Brake duration P50/P95/max | 1.337191 / 1.593646 / 1.595558 s |
| PLANNING_LATENCY P50/P95/max | 4.189000 / 16.698100 / 876.417000 ms |
| PLANNING_BUDGET P50/P95/max | 788.664000 / 1089.812100 / 1183.163000 ms |
| HANDOFF dp P50/P95/max | 0.000000 / 0.000002 / 0.000863 m |
| HANDOFF dv P50/P95/max | 0.000000 / 0.000000 / 0.010771 m/s |
| HANDOFF da P50/P95/max | 0.000000 / 0.000000 / 0.093658 m/s² |
| MIN_STATIC / DYNAMIC_CLEARANCE | 0.182074 / 0.821146 m |
| MIN_SWARM_DISTANCE elliptical | 0.795416 m |
| COLLISION / SWARM_CLEARANCE_VIOLATION_SAMPLES | 0 / 0 |
| UNVALIDATED_EXECUTED_SAMPLES | 0 |
| UNVALIDATED_NEW_POLYNOMIAL_ADOPTIONS | 0 |
| MAX_COMMAND / ODOM_SPEED | 3.069171 / 3.291272 m/s |
| MAX_PLANNER_TRAJECTORY_SPEED | 3.137336 m/s |
| EMERGENCY_STOPS / HEARTBEAT_LOSSES | 0 / 0 |

共839次handoff，全部通过原P/V/A tolerance；P95保持微小残差，max不是全部微小，不能只报平均值。完整按source transition数据在 [summary.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/analysis/summary.json)。实际低于0.15m/s并持续至少0.2s的片段只有开场UAV0约0.200s、UAV1约0.434s；未再出现任务中因polynomial到期而停住。最长activation间隔仍有2.300s，但旧轨迹有覆盖，没有到期hold。

Static/dynamic clearance为到表面的距离，collision<=0；swarm按原elliptical距离小于0.5m判违规。保留原5% trajectory feasibility tolerance（名义3m/s对应3.15m/s数值gate），没有以它掩盖odom超速。SAFETY_REGRESSION_OBSERVED=YES沿用feedback_31的保守口径：名义速度超限仍属于执行验收异常；并非宣称本轮进行了统计A/B。当前硬安全门、碰撞、未验证执行指标均无新增失败，但未解决控制器/odom名义超速。

**Build / tests / runtime / RViz**

`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`：6/6成功；保留一条既有base_delta_phi unused-variable warning。16组unit/gradient/contract测试全部PASS。新增包括normal保持、退化进展排序、恢复候选都不安全时安全降级及urgent replan、persistence提前恢复、0.6s覆盖不能启动0.9s batch、失败长尾保持、refinement deadline、双gap XYZ梯度（最大误差1.171e-10）、yaw饱和持续转向与时间导数、巨大积分取消及有限initializer恢复。旧MINCO P/tau/visibility/yaw梯度、generation/identity/PVA、真实隔离ROS traj_server契约均PASS。

formation manager/coordinator/generator/joint维持-O2，planner/traj_opt/traj_server维持既有-O3。运行executable hash、加载的.so、源码hash与最终build一致；仿真后未修改生产源码。证据：[tests.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/tests.json)、[build_final.log](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/build_final.log)、[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/final_verification.json)。

FULL ON中119次generator trigger，52次team joint尝试，38次成功proposal，26次commit且26组三机真实adoption/optimized yaw execution，共78次executor optimized-yaw日志。原team正常geometry acceptance仍满足ratio>=0.8。完整身份闭环：[verified_chains.json](/home/bob/ALP/egov2_fc65423_constvel/local_recovery_reserve_deadline_20260911/analysis/verified_chains.json)。

三轮都启动native RViz，保留UAV/target/obstacles/trajectory/path/FOV配置。第一轮有可用native窗口截图；最终轮已按PID确认RViz和native配置正常加载，但直接X窗口截图的OpenGL区域为黑色，不能拿该截图作最终全程视觉证明，也不把黑色抓图判断为polynomial问题。最终“多数时间真正环绕”和“没有任务中到期停顿”的结论来自全程odom/activation/command数据。没有为补图重开仿真。

正常停止本轮roslaunch/RViz/recorders，全部本轮test master与仿真master端口12551/12552/12553/12556/12557/12558已关闭，无本轮残留。其他不属于本轮的工作未处理。所有正式报告只写feedback，未覆盖旧报告，未访问或修改禁止目录。

**剩余限制**

1. Local恢复是“有进展则优先、无可用安全进展则明确降级”的执行契约，不能从一次运行推导全时域几何保证。实际严格合围57.68%，仍有9.60s退化段和一个截至任务结束尚未恢复的episode；继续改善需提高安全恢复候选的可用性和跨机恢复一致性，不能通过放宽25°/170°、恢复120°或只加权重掩盖。
2. 最终camera-time提升，但K2和longest K2 loss未同步改善；现有team coverage protection保留，local safety-only段仍可能丢coverage。
3. 名义速度上限问题仍需单独处理controller/odom与已有feasibility容差。无碰撞不等于所有速度指标满足名义3m/s。
4. 预算和内循环取消已在本次真实长尾事件验证；不能把单次零hold当作任意场景/任意系统负载下永不超时的证明。未通过延长无验证hold或放宽安全条件实现本次结果。

```text
PRODUCTION_SOURCE_CHANGED: YES
LOCAL_ENCIRCLEMENT_RECOVERY_CONTRACT_ACTIVE: YES
LOCAL_GEOMETRY_PRESERVATION_ACTIVE: YES
DEGRADED_GEOMETRY_PROGRESS_REQUIRED: YES
SAFETY_DEGRADED_FALLBACK_SUPPORTED: YES
PERSISTENCE_GEOMETRY_RECOVERY_ACTIVE: YES
120_DEGREE_RESTORING_REINTRODUCED: NO
EXECUTED_ENCIRCLEMENT_RATIO: 0.576815
SAME_SEMICIRCLE_RATIO: 0.280477
NORMAL_ENCIRCLEMENT_RATIO: 0.567877
RECOVERING_ENCIRCLEMENT_RATIO: 0.002109
SAFETY_DEGRADED_RATIO: 0.430014
BRAKE_COVERAGE_AWARE_PLANNING_ACTIVE: YES
EXPENSIVE_SOLVE_EXECUTION_DEADLINE_ACTIVE: YES
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 0
TERMINAL_HOLD_COUNT: 0
TERMINAL_HOLD_TOTAL_DURATION: 0.000000 s
ACCUMULATED_CAMERA_VISIBLE_TIME: 230.493691 camera·s
MEAN_VISIBLE: 2.874773
K2: 0.983155
ALL3: 0.891989
ACTUAL_ACTIVATION_RATE_UAV0/UAV1/UAV2: 3.280200 / 3.629423 / 3.554589 Hz
SAFETY_REGRESSION_OBSERVED: YES
COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES
SCENARIO_A_FULL_RUN_COMPLETED: YES
RVIZ_ENABLED: YES
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_32.md
```
