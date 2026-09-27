本轮实现了 **Persistent Team Recovery Target + 独立 Target/Transition Pool**。最终目标必须满足真实25°/170°圆周gap定义；过渡轨迹仍只受原有安全、动力学、P/V/A、身份/activation及execution reserve约束。目标、意图和peer数据均不构成rolling planner等待条件。

唯一一次完整 Scenario A FULL ON + native RViz 运行中，实际合围从feedback_33的40.65%变为 **55.44%**，同半平面从28.87%降到 **21.73%**；actual activation为 **3.903937 / 4.228226 / 3.953828 Hz**，**0 terminal hold、0碰撞、0 swarm violation、0未验证执行样本**。实际合围仍非全程成立，不能声称全局收敛。camera-time下降约1.01%，K2和All3也有回退；没有为改善数字调参或重复运行。

制动生产策略完全未改。本轮审计发现的是候选供给和最终可执行性问题，不能把`SCP_FINAL_OK`或早期`ABSOLUTE_SAFE`当成已经通过最终全轨迹安全预检。新运行仍有2.299906s低速段；零terminal hold不等于没有低速/制动。名义3m/s仍被超过，安全验收不能全部标为通过。

**实现与数据通路**

| 层 | 本轮实现 | 源码 |
| --- | --- | --- |
| 团队长期目标 | `TeamRecoveryTarget`记录唯一plan_id、创建/确认时刻、source generation/hypothesis、target-relative offsets/bearings/gaps、camera/K2、粗略到达代价；`PersistentRecoveryTarget`管理保持/失效/显著收益/无进展换目标 | [team_recovery_target.h](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/include/multi_uav_formation/team_recovery_target.h:47) |
| 唯一发布者 | cooperative viewpoint manager在同一target/三机odom快照下选择目标，发布latched `/encirclement/recovery_target`；三机planner和coordinator独立读取同一个目标，**没有ACK、barrier或等待** | [cooperative_viewpoint_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/cooperative_viewpoint_manager.cpp:202) |
| Target Pool | 仅终态布局通过25°/170°、finite及廉价障碍检查；评价最终布局的LOS/FOV/binary camera-time、当前到目标位移和旧目标连续性，保留fixed/uniform/previous/actual/独立bearing搜索seed | [adaptive_viewpoint_generator.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/adaptive_viewpoint_generator.cpp:153) |
| Transition Pool | 与Target Pool不同类型/容器；保留当前/原始搜索/短期意图及向持久目标移动的0.25/0.50rad参考seed。过渡仍可230°→205°或230°→235°；没有目标几何资格门 | [adaptive_viewpoint_generator.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/adaptive_viewpoint_generator.cpp:242) |
| 短期意图 | `deriveIntent`从共享target和实际bearing差导出方向、最大gap两端及每机角位移；不再在已有可用目标时各自重新决定长期去向；没有target时保留原gap方向恢复 | [team_recovery_target.h](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/include/multi_uav_formation/team_recovery_target.h:67) |
| Team selector | 全部hard-safe组合仍可参与；恢复中先比较共享目标距离，然后E/ratio、camera、K2等；旧hysteresis不能覆盖明确target收益 | [topology_coordinator_core.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/topology_coordinator_core.cpp:316) |
| Joint P/T/yaw | 同一world-time样本增加目标bearing软代价及MINCO P/virtual-T解析梯度；reference horizon已经NORMAL时目标代价关闭。保留相对E/ratio接受，新增向目标推进的相对收益，不要求ratio≥0.8 | [team_visibility_optimizer.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/team_visibility_optimizer.cpp:770) |
| Local objective | 原radius/height、J_vis、gap项保留；加入小权重target bearing牵引，包含moving-target和previous-piece显式时间导数。新鲜度在solve起点判断，避免样本时间跨expiry造成额外导数跳变 | [poly_traj_optimizer.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp:6372) |
| Local selection | actual-active future P/V/A重建及完整安全检查之后，恢复时比较target progress→原几何/visibility排序。无target进展也选择现有安全successor，未新增execution return/gate | [planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1071) |
| 通信 | 独立`traj_utils/TeamRecoveryTarget`消息，仅携带软参考。旧target id拒收仅丢弃旧参考，不影响已有轨迹或下一次local replan | [TeamRecoveryTarget.msg](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/TeamRecoveryTarget.msg) |

20个修改文件（含header/msg/CMake/launch/tests）见[changed_files.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/changed_files.json)和[production_changes.diff](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/production_changes.diff)。原有`validatedBrakeSuccessor`、`ensureExecutionCoverage`、`optionalRefinementAllowed`、`finishPlanningBatch`、`lifecycleSuccessorDue`、`reboundReplan`、`validateExecutionTrajectory`、`checkActiveHandoff`、remaining-suffix revalidation及整个FSM逐段/逐文件相同，见[brake_source_unchanged.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/brake_source_unchanged.json)。SIDE/A*/fallback生成、brake duration/trigger/coverage及所有安全阈值均未修改。

**目标保持、代价及参数**

共享几何定义仍为 `g_min≥25° && g_max≤170°`；`E_enc=Σ([g_j−170°]₊²+[25°−g_j]₊²)`未改。目标代价采用 `Σ 2(1−cos(theta_i−theta_i*))`，其位置梯度为 `2sin(error)·(-y,x,0)/r²`；只指向所选择的合法非等角target，没有120°参考项或额外奖励。三gap均满足区间的120°与90°/110°/160°布局，在其它quality相同时没有额外优先级。NORMAL时关闭目标牵引；已有J_gap继续提供合围区间目标。

目标评分使用廉价最终布局检查和视线预测：从同一target位置/速度外推，检查当前及未来3.5s的参考点，评价最终站位的static/dynamic LOS、FOV与binary camera-time。粗略reach为三机当前位置到目标参考点的平方距离，continuity为新旧offset平方差。**这不是对通往目标的全路径安全证明**；最终运动仍由原planner完整验证。3.5s只是目标质量筛查，1.5s仍是原近期轨迹/visibility/geometry评价窗，都不构成恢复完成期限。radius/height搜索仍使用已有固定半径与band，没有提高`local_gap_weight=100`。

| 新ROS参数（`/encirclement_geometry/`） | 默认值/语义 |
| --- | --- |
| target_camera_gain | 0.5 camera·s，显著更优camera门槛，仅用于换目标偏好 |
| target_reach_gain | 0.25；到达代价下降至少25%并额外超过1m²，且camera不回退时可换目标 |
| target_minimum_dwell | 2s；显著收益换目标的最短保留时间，失效可立即替换 |
| target_no_progress_time | 8s；实际bearing目标cost未取得累计进展时搜索不同布局，绝不等待 |
| target_progress_epsilon | 0.02，实际/预测目标cost进展判据 |
| target_max_age | 3s，旧参考失去吸引力，local照常执行；不放松peer碰撞判断 |
| local_target_weight / joint_target_weight | 4 / 1；本轮只设置一次，没有依实测反复调参 |
| target_ranking_epsilon | 0.02，安全候选间target cost差异偏好 |

显著变化、无进展和当前旧target的重新筛查共同防止“每周期重选”及“永久锁死”。旧目标失去可行性/可见性时仅使软目标失效；空target pool仍返回，不停止local。实际pointwise正常合围可使target inactive，保留布局供下一次seed/恢复使用；因此`TARGET_REACHED`表示达到真实NORMAL区间，**不表示精确到达三个期望bearing**。

三机target由一个发布者生成，排序依实际circular order。例plan1的期望bearing约(-30.503°,-180°,135.503°)，gap约149.497°/166.005°/44.497°；在本轮有效区间起点，最大gap端点为UAV0/2，三机期望角变化约(+125.053°,0°,-20.054°)。plan2的bearing为(-45°,180°,75°)，gap为120°/105°/135°。这些均非固定slot。各目标的动态gap端点/每机角位移见[team_target_allocation.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/team_target_allocation.json)，runtime短期意图记录保留每机contribution；yaw仍只处理FOV。

**统计口径与完整运行**

场景`long_cylinder_forest_visibility_stress.json`，FULL ON，native RViz，target正常完成80.204541s路线。统一有效区间为`[1789085098.094614983, 1789085178.270087719]`，RUN_DURATION=80.175472736s。按三机trajectory/visibility有效起点到route终点，全部纳入退化、低速、风险及启动片段；没有剪掉不利区间。

geometry来自实际odom+target，每帧重排真实圆周gap，按dt积分，gap分位数按时间加权；visibility沿用原binary range∧static LOS∧dynamic LOS∧FOV，camera-time单位camera·s。NORMAL/RECOVERING/SAFETY_DEGRADED为实际执行identity关联的预测contract状态：三机全部NORMAL才算团队NORMAL，任一degraded则降级，因此与点态实际合围ratio不等价。degraded episode逐帧连段、不去抖；许多短episode只是边界穿越，不能将恢复次数当成大型任务成功数。

| Geometry | 本轮结果 |
| --- | --- |
| EXECUTED_ENCIRCLEMENT_RATIO | 0.554449 |
| SAME_SEMICIRCLE_RATIO | 0.217326 |
| GAP_MIN P50/P95/min | 53.359943 / 96.329404 / 0.803078 deg |
| GAP_MAX P50/P95/max | 168.576626 / 202.647916 / 314.921220 deg |
| NORMAL_ENCIRCLEMENT_RATIO | 0.441606 |
| RECOVERING_ENCIRCLEMENT_RATIO | 0.062274 |
| SAFETY_DEGRADED_RATIO | 0.496119 |
| DEGRADED_EPISODE_COUNT / P50/P95/max | 34 / 0.095281 / 3.184783 / 5.066983 s |
| RECOVERY_SUCCESS_COUNT / P50/P95/max | 33 / 0.066556 / 3.229575 / 5.066983 s |

| 持久目标/恢复牵引 | 本轮结果 |
| --- | --- |
| TARGET_CREATED / REUSED / REPLACED / INVALIDATED / REACHED | 9 / 30 / 12 / 9 / 14 |
| 包含有效区间前初始化的created / reused | 10 / 32；最终汇总使用有效区间内9 / 30 |
| TARGET_LIFETIME P50/P95/max | 2.599359 / 8.247166 / 12.999573 s；同一ID的保留期，包含inactive NORMAL时段 |
| ACTIVE目标段 P50/P95/max | 1.299301 / 2.513687 / 2.800505 s；24段，实际启用累计31.376390s |
| TARGET_CHANGE_RATE_HZ | 0.261925，21次ID更换/完整有效时长 |
| Replacement reason | 11次NEW_TARGET_SIGNIFICANTLY_BETTER，1次NO_PROGRESS |
| Target message真实25°/170°检查 | 802个有效消息全部通过 |
| TRANSITION_CANDIDATE_COUNT | 336；generator实际输出到reference bundle的hypothesis，非内部搜索试探数 |
| TRANSITION_RATIO_LT_0P8_EXECUTED_COUNT | 53组三机joint实际执行；其中30组携带active persistent target |
| bearing error to active target P50/P95 | 11.470741 / 111.284810 deg；三机误差按dt加权 |
| 实际target-progress rate | 8/24=0.333333；同一active连续段首尾target cost下降>0.02 |
| Local target progress | 531次安全候选对比中0.141243超过0.02；候选评估比例，非实际执行成功率 |
| Joint target progress | 70次active-target尝试中0.271429超过0.02 |
| E_enc实际active段首/尾mean | 0.401744831 / 0.112134147 rad² |
| E_enc实际active段首/尾P50 | 0.014754123 / 0.000110182 rad² |
| E_enc实际active段首/尾P95 | 1.500488902 / 0.626116371 rad² |
| Local同一epoch的E_before/E_after mean | 0.324354969 / 0.310806872 rad² |

日志有一个诊断限制：同次update先invalidate再create时，`TARGET_INVALIDATED`一行打印的是update后的新ID；且旧screen失败返回camera=0时reason也会写成`VISIBILITY_DEGRADED`，不能据9行日志断言全部是纯LOS/FOV退化。报告的目标寿命/ID更换来自实际target消息时序；9次invalid事件数量可靠，细分障碍/可见性原因未作过度推断。此问题不改变target失效决策或轨迹执行；未借此修改brake或重复场景。

具体真实执行链：team_solution_id=3、coordination_generation=5、encirclement_generation=7、hypothesis=0、persistent plan_id=1。joint目标cost **1.616664534→1.513212600**，E **0.032161608→0.016016870**，ratio仍 **0→0**，仍然proposal→3 ACK→commit→3 adoption→optimized yaw。proposal=1789085100.604462147，activation=1789085100.839889050，lead=235.427ms≥原85ms。实际UAV0/1/2 activation分别1789085100.844108 /1789085100.842116 /1789085100.848982，trajectory id14/14/12，各5个yaw knots。P/T/yaw change norm为0.115454650 /0.032795242 /0.000049936。camera-time before/after均4.5，K2/All3均1；这是饱和事件的恢复闭环证据，不是该事件可见性提升证据。

全程173次joint尝试、154次joint接受、82组三机实际执行；其中53组after ratio<0.8。证据按coordination_generation关联，避免把重复encirclement generation/hypothesis误当同一事件。SAFETY_DEGRADED期间actual activation UAV0/1/2为4.198448 /4.701257 /4.072746Hz，absolute recovery-ratio拒绝数0。后台2次timeout、PVA/旧generation拒绝只淘汰增强解，local继续。

| Executed visibility | 本轮结果 |
| --- | --- |
| ACCUMULATED_CAMERA_VISIBLE_TIME | 228.632941 camera·s |
| MEAN_VISIBLE | 2.851657 |
| K2 / ALL3 / NONE | 0.977270 / 0.876085 / 0.001698 |
| LONGEST_K2_LOSS / LONGEST_BLACKOUT | 0.899673 / 0.136153 s |

| UAV | replan Hz | candidate Hz | commit Hz | actual activation Hz | activation interval P50/P95/max s |
| --- | --- | --- | --- | --- | --- |
| 0 | 3.429977 | 6.223849 | 3.916410 | 3.903937 | 0.260208 / 0.369888 / 1.049650 |
| 1 | 3.878992 | 6.672864 | 4.215753 | 4.228226 | 0.259495 / 0.350170 / 1.140481 |
| 2 | 3.504813 | 6.298684 | 3.953828 | 3.953828 | 0.269405 / 0.360045 / 1.130039 |

Commit与activation分别按事件发生时刻落入有效区间统计，区间边界可能相差1，不能强行匹配计数。candidate rate按原`stage=CANDIDATE`的hypothesis planning输出事件统计，非每个SIDE内部试探数。

| Lifecycle / safety / performance | 结果 |
| --- | --- |
| TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT | 0；基于记录的完整poly系数/activation/duration |
| TERMINAL_HOLD_COUNT / TOTAL / MAX | 0 / 0 / 0 s |
| HANDOFF dp/dv/da max | 0.002177239m / 0.018472184m/s / 0.089126469m/s² |
| 所有真实handoff | 969次，均在原0.02 /0.05 /0.10 tolerance内 |
| ZERO_EXECUTABLE_PENDING / stale head / coordinator timeout | 0 / 0 / 2 |
| PLANNING_LATENCY P50/P95/max | 4.666000 / 18.365200 / 430.432000 ms |
| PLANNING_BUDGET P50/P95/max | 707.950000 / 1073.352400 / 1097.037000 ms；原策略未改 |
| VIEWPOINT_GENERATOR mean/P95/max | 20.524804 / 24.534100 / 40.458000 ms |
| JOINT mean/P95/max | 3.809503 / 5.229000 / 23.710000 ms |
| COLLISION / SWARM_CLEARANCE_VIOLATION / UNVALIDATED_EXECUTED samples | 0 / 0 / 0 |
| MIN_STATIC / MIN_DYNAMIC_CLEARANCE | 0.205824 / 0.995642 m |
| MIN_SWARM elliptical distance | 0.729049 m |
| MAX_COMMAND / MAX_ODOM / MAX_PLANNER_TRAJECTORY_SPEED | 3.145910 / 3.258740 / 3.145952 m/s |
| EMERGENCY_STOPS / HEARTBEAT_LOSSES | 0 / 0 |

速度峰值由实际command/odom与完整polynomial速度极值分别计算。原名义max_vel=3及原5%可行性容差未变，planner峰值仍小于原3.15 gate，但odom峰值3.258740高于feedback_33的3.161125。故**SAFETY_REGRESSION_OBSERVED=YES**，不能用0碰撞覆盖速度问题。实际表面clearance与planner中心/椭圆安全距离的模型不同，不能直接用0.995642和1.10相减解释为碰撞。

**制动审计：只诊断**

BRAKE_AUDIT_ONLY=YES；BRAKE_PRODUCTION_LOGIC_CHANGED=NO。

| 指标 | feedback_33数据重算 | 本轮 |
| --- | --- | --- |
| VALIDATED_BRAKE实际activation数 | 45 | 52 |
| 有效区间内brake commit日志数 | 46 | 51 |
| LOW_SPEED_EPISODE_COUNT | 5 | 5 |
| LOW_SPEED_DURATION P50/P95/max | 0.574263 / 4.860404 / 5.467084 s | 0.366273 / 1.913382 / 2.299906 s |

低速定义为实际odom speed<0.15m/s持续≥0.2s，启动片段纳入，不剔除。brake统计主值为实际activation。当前有两条区间前commit/区间内activate的id2（UAV1/2），一条UAV0 id315在终点前commit、终点后activate，造成51 commit/52 activation；baseline另有一条UAV1 id178已commit但在接管前被替代。因此不能把feedback_33的46直接当作45次实际执行的矛盾。

| 数据 | UAV | 低速mission区间 s | duration s | 区间内planning starts | actual brakes | 含重叠首轮batch |
| --- | --- | --- | --- | --- | --- | --- |
| feedback_33 | 1 | 47.918203–53.385287 | 5.467084 | 11 | 9 | 12 |
| feedback_33 | 1 | 39.751372–42.185054 | 2.433682 | 12 | 11 | 13 |
| feedback_33 | 1 | 0.051275–0.625538 | 0.574263 | 4 | 0 | 5 |
| 本轮 | 1 | 35.229472–37.529377 | 2.299906 | 16 | 13 | 17 |
| 本轮 | 1 | 0.029068–0.396356 | 0.367288 | 2 | 2 | 3 |
| 本轮 | 2 | 12.462870–12.829143 | 0.366273 | 2 | 0 | 3 |

本轮最长35.229472–37.529377s段有16个planning starts、13次实际brake。第二长0.029068–0.396356s是启动/coverage刷新段；第三长UAV2的12.462870–12.829143s没有brake activation，是连续轨迹低速/跟踪瞬态，之后joint接管。不能把所有低速样本都算成制动。

逐轮timeline以每机`rolling-replan PLANNING_START`至下一次start为batch窗口；保留跨低速起点的首轮，以解释当时active轨迹来源。开始数与feedback_33先前按完成日志得到的11可能不同，本次39.751s段有12次start。下表N/S±为对应batch全部hypothesis的安全分类/最终状态集合；`无记录`不是宣称数学上无路径。详尽timestamp、active id/generation、candidate generation/id、N/S±完整结果、feasible initializer/fallback、A*/repair、MINCO/SCP、preflight、最终brake reason及原始行号全部保留在CSV/JSON中。

[feedback33_brake_batches.csv](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/feedback33_brake_batches.csv)、[feedback33_brake_audit.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/feedback33_brake_audit.json)；[current_brake_batches.csv](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/current_brake_batches.csv)、[current_brake_audit.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/current_brake_audit.json)。

**feedback_33 UAV1，低速起点47.918203s：逐batch明细**

| batch start s / active id | candidate generations | NOMINAL | SIDE_PLUS | SIDE_MINUS | A*/fallback/最终预检 | brake decision |
| --- | --- | --- | --- | --- | --- | --- |
| 47.786105 / 196 | 67:h0 | INVALID | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | INVALID / QP_MAX_ITER_EXHAUSTED | A*=1 | PERSISTENCE_EXPIRING |
| 48.176878 / 197 | 67:h0 | INVALID | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | INVALID / QP_MAX_ITER_EXHAUSTED | A*=1 | PERSISTENCE_EXPIRING |
| 48.553928 / 198 | 68:h0,68:h1,68:h2 | INVALID | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | INVALID,ABSOLUTE_SAFE / QP_MAX_ITER_EXHAUSTED,P_TRUST_TOO_SMALL,FEASIBLE_INITIALIZER_FALLBACK:EXECUTION_DEADLINE | A*=3; fallback=EXECUTION_DEADLINE; preflight=static; deadline截断 | PERSISTENCE_EXPIRING |
| 49.445600 / 199 | 68:h0 | INVALID | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | INVALID / QP_MAX_ITER_EXHAUSTED | A*=1 | PERSISTENCE_EXPIRING |
| 49.803582 / 200 | 68:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / QP_MAX_ITER | A*=1 | PERSISTENCE_EXPIRING |
| 50.297733 / 201 | 69:h0,69:h1,69:h2 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / QP_MAX_ITER_EXHAUSTED,P_TRUST_TOO_SMALL | A*=2 | PERSISTENCE_EXPIRING |
| 51.045114 / 202 | 69:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / QP_MAX_ITER_EXHAUSTED | A*=1 | PERSISTENCE_EXPIRING |
| 51.311224 / 203 | 70:h0,70:h1,70:h2 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / P_TRUST_TOO_SMALL,QP_MAX_ITER_EXHAUSTED | A*=3 | PERSISTENCE_EXPIRING |
| 52.262266 / 204 | 70:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / P_TRUST_TOO_SMALL | A*=1 | PERSISTENCE_EXPIRING |
| 53.020662 / 205 | 71:h0,71:h1,71:h2 | INVALID | INVALID,ABSOLUTE_SAFE / FEASIBLE_INITIALIZER_FALLBACK:VELOCITY_LIMIT | INVALID / STATIC_INFEASIBLE,NA,ASTAR_REPAIR_ACCEPT,STATIC_INFEASIBLE_AFTER_RETIMING,DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=1; fallback=VELOCITY_LIMIT | 无brake提交；继续/采用safe轨迹 |
| 53.190865 / 206 | 71:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA,ASTAR_REPAIR_ACCEPT,STATIC_INFEASIBLE_AFTER_RETIMING | INVALID / DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | A*=2 | 无brake提交；继续/采用safe轨迹 |
| 53.334179 / 206 | 71:h0 | INVALID | ABSOLUTE_SAFE / SCP_FINAL_OK | INVALID / QP_MAX_ITER_EXHAUSTED | A*=1 | 无brake提交；继续/采用safe轨迹 |

**feedback_33 UAV1，低速起点39.751372s：逐batch明细**

| batch start s / active id | candidate generations | NOMINAL | SIDE_PLUS | SIDE_MINUS | A*/fallback/最终预检 | brake decision |
| --- | --- | --- | --- | --- | --- | --- |
| 39.655356 / 170 | 61:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | IMPROVED_ONLY / SCP_FINAL_OK | A*=0 | PERSISTENCE_EXPIRING |
| 39.784120 / 171 | 61:h0 | INVALID | INVALID / SCP_FINAL_OK | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=0 | PERSISTENCE_EXPIRING |
| 39.905853 / 172 | 62:h0,62:h1,62:h2 | INVALID | INVALID / STATIC_INFEASIBLE,NA,ASTAR_REPAIR_ACCEPT,STATIC_INFEASIBLE_AFTER_RETIMING | IMPROVED_ONLY,INVALID / SCP_FINAL_OK | A*=2 | PERSISTENCE_EXPIRING |
| 40.048366 / 173 | 62:h0 | INVALID | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=0 | PERSISTENCE_EXPIRING |
| 40.167739 / 174 | 62:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / SCP_FINAL_OK | A*=0 | PERSISTENCE_EXPIRING |
| 40.293145 / 175 | 62:h0 | INVALID | INVALID / SCP_FINAL_OK | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=0 | PERSISTENCE_EXPIRING |
| 40.414155 / 176 | 62:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / SCP_FINAL_OK | A*=0 | PERSISTENCE_EXPIRING |
| 40.537772 / 177 | 62:h0 | INVALID | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=0 | PERSISTENCE_EXPIRING |
| 40.761151 / 179 | 62:h0 | INVALID | INVALID / TRUE_CONSTRAINT_INFEASIBILITY | INVALID / SCP_FINAL_OK | A*=1 | PERSISTENCE_EXPIRING |
| 41.125410 / 180 | 62:h0 | INVALID | INVALID / QP_MAX_ITER_EXHAUSTED | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=1 | PERSISTENCE_EXPIRING |
| 41.361550 / 181 | 62:h0 | INVALID | INVALID / P_TRUST_TOO_SMALL | INVALID / SCP_FINAL_OK | A*=1 | PERSISTENCE_EXPIRING |
| 41.829640 / 182 | 63:h0,63:h1,63:h2 | INVALID,ABSOLUTE_SAFE | IMPROVED_ONLY,ABSOLUTE_SAFE,INVALID / SCP_FINAL_OK | INVALID / DYNAMIC_INVALID_AFTER_BACKOFF,INSUFFICIENT_DYNAMIC_IMPROVEMENT | A*=0 | 无brake提交；继续/采用safe轨迹 |
| 41.974385 / 183 | 63:h0 | INVALID | INVALID / P_TRUST_TOO_SMALL | IMPROVED_ONLY / SCP_FINAL_OK | A*=1 | 无brake提交；继续/采用safe轨迹 |

**本轮 UAV1，低速起点35.229472s：逐batch明细**

| batch start s / active id | candidate generations | NOMINAL | SIDE_PLUS | SIDE_MINUS | A*/fallback/最终预检 | brake decision |
| --- | --- | --- | --- | --- | --- | --- |
| 35.146573 / 148 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / STATIC_INFEASIBLE,NA | A*=0 | PERSISTENCE_EXPIRING |
| 35.264215 / 149 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / STATIC_INFEASIBLE,NA | A*=0 | PERSISTENCE_EXPIRING |
| 35.384398 / 150 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / STATIC_INFEASIBLE,NA | A*=0 | PERSISTENCE_EXPIRING |
| 35.504005 / 151 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / STATIC_INFEASIBLE,NA | A*=0 | PERSISTENCE_EXPIRING |
| 35.624559 / 152 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / STATIC_INFEASIBLE,NA | A*=0 | PERSISTENCE_EXPIRING |
| 35.743693 / 153 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | INVALID / STATIC_INFEASIBLE,NA | A*=0 | PERSISTENCE_EXPIRING |
| 35.865761 / 154 | 51:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA,ASTAR_REPAIR_ACCEPT,STATIC_INFEASIBLE_AFTER_RETIMING | INVALID / DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | A*=2 | PERSISTENCE_EXPIRING |
| 36.129548 / 155 | 51:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0; preflight=static | PERSISTENCE_EXPIRING |
| 36.267601 / 156 | 51:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0; preflight=static | PERSISTENCE_EXPIRING |
| 36.385855 / 157 | 52:h0,52:h1,52:h2 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0; preflight=DYNAMIC_FAIL | PERSISTENCE_EXPIRING |
| 36.509866 / 158 | 52:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0; preflight=DYNAMIC_FAIL | PERSISTENCE_EXPIRING |
| 36.628671 / 159 | 52:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0; preflight=DYNAMIC_FAIL | PERSISTENCE_EXPIRING |
| 36.748415 / 160 | 52:h0 | INVALID | ABSOLUTE_SAFE / SCP_FINAL_OK | ABSOLUTE_SAFE / SCP_FINAL_OK | A*=1; preflight=DYNAMIC_FAIL,swarm | PERSISTENCE_EXPIRING |
| 36.909871 / 161 | 52:h0 | INVALID | INVALID / STATIC_INFEASIBLE,NA | ABSOLUTE_SAFE / SCP_FINAL_OK | A*=0 | 无brake提交；继续/采用safe轨迹 |
| 37.073935 / 162 | 53:h0,53:h1,53:h2 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0; preflight=DYNAMIC_FAIL,swarm | 无brake提交；继续/采用safe轨迹 |
| 37.186686 / 162 | 53:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | 无brake提交；继续/采用safe轨迹 |
| 37.355792 / 163 | 53:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | PRE_SOLVE_EXECUTION_COVERAGE |

**本轮 UAV1，低速起点0.029068s：逐batch明细**

| batch start s / active id | candidate generations | NOMINAL | SIDE_PLUS | SIDE_MINUS | A*/fallback/最终预检 | brake decision |
| --- | --- | --- | --- | --- | --- | --- |
| -0.172452 / 1 | 3:h0,3:h1,3:h2 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | PRE_SOLVE_EXECUTION_COVERAGE |
| 0.060730 / 2 | 3:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | PRE_SOLVE_EXECUTION_COVERAGE |
| 0.295093 / 4 | 4:h0,4:h1,4:h2 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | 无brake提交；继续/采用safe轨迹 |

**本轮 UAV2，低速起点12.462870s：逐batch明细**

| batch start s / active id | candidate generations | NOMINAL | SIDE_PLUS | SIDE_MINUS | A*/fallback/最终预检 | brake decision |
| --- | --- | --- | --- | --- | --- | --- |
| 12.230595 / 50 | 20:h0 | INVALID | 无记录 | 无记录 | A*=0 | 无brake提交；继续/采用safe轨迹 |
| 12.516495 / 50 | 20:h0 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | 无brake提交；继续/采用safe轨迹 |
| 12.774809 / 52 | 21:h0,21:h1,21:h2 | ABSOLUTE_SAFE | 无记录 | 无记录 | A*=0 | 无brake提交；继续/采用safe轨迹 |

上述reason集合只压缩显示；同一batch的不同hypothesis、A*前失败和A*后成功不能相互覆盖，原始JSON保留全部顺序。`SCP_FINAL_OK`仍需后续风险分类和最终preflight；`ASTAR_REPAIR_ACCEPT`是guide可供MINCO/SCP继续使用，不等于最终轨迹安全。

**Brake根因排序及八项回答**

1. **CONFIRMED：完整验证通过的moving successor供给不足。** baseline 39.784s的SIDE_PLUS虽`SCP_FINAL_OK`，dynamic clearance=0.303184<1.10；39.906s的SIDE_MINUS相对改善到0.971534，仍为`IMPROVED_ONLY`。它们都不是可以执行的绝对安全轨迹。两侧frontend还反复`STATIC_INFEASIBLE`或`DYNAMIC_INVALID_AFTER_BACKOFF`。首要原因不是geometry排序，而是已有候选没有通过最终许可。
2. **CONFIRMED：候选早期分类与最终全轨迹预检不同。** 本轮36.130s NOMINAL candidate372报告动态距离4.816513、`ABSOLUTE_SAFE`，但finalize用统一future activation重建后，完整static checker在local t=4.9s发现占据点，继而brake id156。36.268s candidate373在5.86667s被static拒；36.386s后多条NOMINAL被`DYNAMIC_FAIL`拒。同期两个swarm拒绝也保留在原始timeline。源码[planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1320)明确检查dynamics→full static→full dynamic→swarm；最终拒绝不是target/ratio gate。
3. **CONFIRMED：A* guide到可执行MINCO/SCP的转换失败。** baseline 47.786s SIDE_MINUS A* raw path有30点、static_free=1、side_valid=1，但最终SCP `QP_MAX_ITER_EXHAUSTED`，duration约21.25s。长低速段反复出现QP迭代耗尽/P_TRUST_TOO_SMALL，另有dynamics model trust耗尽；不能把它写成“A*没找到路”，也不能把可行guide直接当成动态可执行轨迹。
4. **CONFIRMED：有限frontend搜索与条件性A*入口。** 当前SIDE offset只在请求值到请求值−0.3之间按map_step回退（默认0.7/0.6/0.5/0.4）；仅保留有足够动态改善的base进入local A*，最多3个rejoin尝试。因此确有candidate在optimization前被过滤，参见[planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:4550)和[planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:5029)。但没有穷尽更广搜索或给出一条被遗漏且完整安全的路径，**不能确认搜索空间不足就是根因**。
5. **CONFIRMED截断事件，但不能确认deadline独立导致brake。** baseline48.554s batch的SIDE_MINUS refinement达到EXECUTION_DEADLINE，原逻辑仍保留feasible initializer；candidate686随后在final preflight local t=9.4s发现static collision，才未执行。deadline并未丢弃已经最终验证通过的moving candidate。本轮最长低速段没有deadline截断。coverage/deadline的保护仍工作，未修改。
6. **POSSIBLE而未证实：dynamic prediction保守性。** 源码预测来自`/dynamic/prediction_*`路径并插值，超出末端时间使用最后位置；长duration候选可能依赖远期预测。但这批证据没有保存每个被拒候选的完整polynomial与当时障碍prediction快照，无法对每次拒绝重放“预测vs实际”并证明误拒。动态拒绝很多不等于预测过保守，也不能降低1.10阈值。

逐项结论：没有发现**通过完整最终验证**的moving candidate仅因排序而未选；有candidate在frontend就没有形成polynomial；有A*成功但MINCO/SCP失败；有early ABSOLUTE_SAFE在full preflight失败；deadline发生过但不能归为已证实独立brake原因；连续brake失败理由并非完全相同，会从dynamic/front-end转为QP/static/full-preflight；确实存在**planner持续运行但当前可执行运动候选供给不足**的结构性问题。“只能brake”仅限该时刻当前枚举且通过全部最终检查的集合，绝不是全空间不可避障证明。

BRAKE_ROOT_CAUSE_PRIMARY: 当前枚举moving candidate未通过完整最终安全/动力学验证，安全successor供给不足。

BRAKE_ROOT_CAUSE_SECONDARY: SIDE/A* guide到MINCO/SCP的转换失败，以及早期/prefix分类与final全轨迹校验差异；有限frontend空间可能进一步限制供给。

SAFE_MOVING_CANDIDATE_SUPPLY_INSUFFICIENT: YES（限定当前实现/枚举集合）

DYNAMIC_PREDICTION_OVERCONSERVATIVE_EVIDENCE: NO（未获得误拒的对照证据）

SEARCH_SPACE_INSUFFICIENT_EVIDENCE: NO（确认搜索有界，不等于证明扩大搜索就存在安全moving解）

DEADLINE_CAUSED_BRAKE_EVIDENCE: NO（有截断，但保留的fallback随后独立失败于static预检）

**与feedback_33比较及剩余问题**

| 指标 | feedback_33 | 本轮 | 单次观测差异 |
| --- | --- | --- | --- |
| EXECUTED_ENCIRCLEMENT_RATIO | 0.406460 | 0.554449 | +14.80个百分点 |
| SAME_SEMICIRCLE_RATIO | 0.288722 | 0.217326 | −7.14个百分点 |
| SAFETY_DEGRADED_RATIO | 0.686628 | 0.496119 | −19.05个百分点 |
| ACCUMULATED_CAMERA_VISIBLE_TIME | 230.964219 | 228.632941 | −1.01% |
| MEAN_VISIBLE | 2.881520 | 2.851657 | −0.029863 |
| K2 | 0.985032 | 0.977270 | −0.776个百分点 |
| ALL3 | 0.896488 | 0.876085 | −2.040个百分点 |
| TERMINAL_HOLD_COUNT | 0 | 0 | 保持 |

目标ID变化0.262Hz，明显低于3.4–3.9Hz local replans；同一ID跨周期保留，真实恢复链有目标牵引，随机重新选布局的机制已被替代。本次几何结果改善，与目标/过渡分离的预期一致；这是一次场景观测，**不是统计显著A/B，也不能只凭一次运行将全部改善归因于单个模块**。

剩余：实际target bearing error P95仍达111.28°，只有8/24 active段取得>0.02的净target-cost下降；严格合围55.44%、仍有同侧段；visibility少量回退及0.136153s blackout；名义速度超限仍存在；安全制动低速仍存在。没有为了弥补这些结果恢复ratio门、固定120°或提高惩罚权重。下一步应基于已保存的target/候选/执行时序分析预测与执行差异；制动问题如需处理，需另一个明确任务，本轮只审计。

图与核心原始数据：[persistent_target_tracking.png](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/persistent_target_tracking.png)、[encirclement_execution.png](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/encirclement_execution.png)、[persistent_targets.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/persistent_targets.json)、[verified_chains.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/verified_chains.json)、[summary.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/summary.json)、[encirclement.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/encirclement.json)、[recovery.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/recovery.json)、[supplement.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/analysis/supplement.json)。完整polynomial、PositionCommand、trajectory/visibility CSV、target消息和ROS日志在[本轮原始证据目录](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/run)。

**Build、测试、RViz与清理**

`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`：6/6 PASS；17组unit/gradient/contract全PASS。新测试覆盖target保持/显著换目标/障碍失效/空pool、终态严格而过渡不设几何门、230→205及230→235、NORMAL关闭代价、120无额外奖励、共享target消息identity、实际joint ratio<0.8→三ACK commit。原lifecycle/pending/ghost generation/PVA/coverage/gradient/ROS executor测试保持通过。

新增目标代价通过**真实production joint完整objective**的MINCO waypoint/virtual-T有限差分，最大误差1.67615e−10；local production tracking target组件的位置及moving-target时间梯度通过。运行后仅强化测试，使NO_PROGRESS fixture没有同时存在camera显著改善，并覆盖真实generator空pool返回；针对该test重新build/执行通过，所有生产源码、runtime executable/library哈希仍与完整运行一致，没有第二次仿真。见[tests.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/tests.json)、[test_summary.log](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/test_summary.log)、[persistent_recovery_target_contract_test.log](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/persistent_recovery_target_contract_test.log)、[post_run_test_strengthening.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/post_run_test_strengthening.json)。

formation manager/coordinator/generator/joint为-O2，planner/traj_opt/traj_server为既有-O3；运行时/proc executable及加载.so均为当前workspace，hash与build manifest一致。见[build_manifest.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/build_manifest.json)、[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/final_verification.json)、[cleanup_and_source_check.json](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/cleanup_and_source_check.json)。

native RViz确已启用，owned window/native配置和UAV/target/obstacle/trajectory/path显示项已核对；OpenGL截图区域仍为黑色，**没有声称做了完整人工视觉确认**。运动/合围结论来自odom、command和完整polynomial数据，不将RViz marker折线当作真实polynomial断裂。截图[native_rviz_owned_window.png](/home/bob/ALP/egov2_fc65423_constvel/persistent_team_recovery_20260911/run/native_rviz_owned_window.png)。

target正常结束后停止本轮roslaunch/RViz/recorder/test master；12573/12574端口已关闭，无本轮进程残留。未访问或修改禁止目录。报告实际检查最大feedback序号后以exclusive create创建，不覆盖历史报告。

```text
PRODUCTION_SOURCE_CHANGED: YES

PERSISTENT_TEAM_RECOVERY_TARGET_IMPLEMENTED: YES
RECOVERY_TARGET_IS_HARD_EXECUTION_GATE: NO
RECOVERY_TARGET_CAN_BLOCK_LOCAL_REPLAN: NO

RECOVERY_TARGET_POOL_IMPLEMENTED: YES
RECOVERY_TRANSITION_POOL_IMPLEMENTED: YES
TARGET_AND_TRANSITION_SEMANTICS_SEPARATED: YES

TARGET_REQUIRES_TRUE_25_170_GEOMETRY: YES
TRANSITION_REQUIRES_TRUE_25_170_GEOMETRY: NO

ABSOLUTE_RECOVERY_RATIO_GATE_REINTRODUCED: NO
NEAR_1P5S_BLOCKS_RECOVERY: NO
LOCAL_GEOMETRY_IS_EXECUTION_GATE: NO
PENDING_BLOCKS_LOCAL_REPLAN: NO

120_DEGREE_RESTORING_REINTRODUCED: NO
120_DEGREE_HAS_EXTRA_TARGET_REWARD: NO

RECOVERY_TARGET_CREATED_COUNT: 9
RECOVERY_TARGET_REUSED_COUNT: 30
RECOVERY_TARGET_REPLACED_COUNT: 12
RECOVERY_TARGET_REACHED_COUNT: 14

EXECUTED_ENCIRCLEMENT_RATIO: 0.554449
SAME_SEMICIRCLE_RATIO: 0.217326
NORMAL_ENCIRCLEMENT_RATIO: 0.441606
RECOVERING_ENCIRCLEMENT_RATIO: 0.062274
SAFETY_DEGRADED_RATIO: 0.496119

ACCUMULATED_CAMERA_VISIBLE_TIME: 228.632941 camera·s
MEAN_VISIBLE: 2.851657
K2: 0.977270
ALL3: 0.876085

TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 0
TERMINAL_HOLD_COUNT: 0
ACTUAL_ACTIVATION_RATE_UAV0/UAV1/UAV2: 3.903937 / 4.228226 / 3.953828 Hz

BRAKE_AUDIT_ONLY: YES
BRAKE_PRODUCTION_LOGIC_CHANGED: NO
VALIDATED_BRAKE_COUNT: 52 actual activations (51 commits within interval)
BRAKE_ROOT_CAUSE_PRIMARY: ENUMERATED_MOVING_SUCCESSORS_FAIL_FINAL_SAFETY_OR_DYNAMICS
SAFE_MOVING_CANDIDATE_SUPPLY_INSUFFICIENT: YES

COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
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

REPORT_FILE:
/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_34.md
```
