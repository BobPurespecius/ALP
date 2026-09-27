本轮已把恢复改成非阻塞的持续优化：generator/selector 不再以正常合围作为恢复候选资格，joint 使用相对改善，local 的几何条件只用于安全候选间排序。最终完整 Scenario A 验证 **58组 ratio<0.8 的恢复解完成三机实际执行**；SAFETY_DEGRADED期间仍持续滚动接管，**0 terminal hold / 0 collision / 0 swarm violation / 0 unvalidated samples**。

但实际严格合围 **40.65%**，低于feedback_32的57.68%；同半平面28.87%略高于此前28.05%。机制闭环通过，不能宣称合围表现改善或“全程不停车”。仍有最长5.467084s的安全制动低速段，期间planner和activation并未冻结。没有为得到更漂亮数字调参重跑。

**改动与真实数据流**

rolling local planner持续维护已验证successor；team optimizer仍为异步增强。只有安全、动力学、P/V/A handoff、有效身份与执行时间契约能够拒绝实际提交。几何/visibility/K2决定相对偏好：joint没有收益时保留local执行，不让team增强失败变成本机等待条件。没有改动场景、25°/170°、collision/dynamics/yaw阈值、Local-SFC、generation、85ms activation margin、execution coverage预算或deadline。

| 入口 | 最终语义 |
| --- | --- |
| [encirclement_geometry.h::inline double encirclementGeometryCost](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/include/multi_uav_formation/encirclement_geometry.h:81) | 共用真实sorted circular gaps，改为所有gap的上下界违反平方和；各gap两端都有梯度，没有120°恢复项 |
| [encirclement_recovery.h::struct RecoveryIntent](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/include/multi_uav_formation/encirclement_recovery.h:25) | 新增soft intent、相对改善和peer confidence helper；无wait、ACK或同步锁 |
| [adaptive_viewpoint_generator.cpp::AdaptiveViewpointGenerator::evaluateCached](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/adaptive_viewpoint_generator.cpp:112) | 删除170°/25°输出硬过滤；finite geometry及既有参考点检查后计算E、camera-time和intent；normal、恢复和过渡seed都可评价/排序 |
| [adaptive_viewpoint_generator.cpp::AdaptiveViewpointGenerator::generate](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/adaptive_viewpoint_generator.cpp:153) | fixed/uniform/previous accepted seed保留，增加实际位置seed及soft intent seed；坐标搜索及原始过渡seed均参与排序，有限top-k仍有容量限制 |
| [topology_coordinator_core.cpp::bool TopologyCoordinatorCore::better](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/topology_coordinator_core.cpp:307) | 所有hard-safe组合参与；先E/ratio，再binary camera-time，随后K2/blackout、intent及其他tie-break。取消coverage保护集合的绝对排除；历史保持只能在geometry和camera均近似持平时生效 |
| [team_visibility_optimizer.h::static bool relativeEncirclementAcceptance](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/include/multi_uav_formation/team_visibility_optimizer.h:241) | baseline已退化时比较相对E/ratio或一致短暂过渡，不要求ratio达到0.8；baseline正常时保留相对不退化偏好 |
| [team_visibility_optimizer.cpp::TeamVisibilityOptimizationResult TeamVisibilityOptimizer::optimize](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/team_visibility_optimizer.cpp:1411) | 每个非线性trial重算sorted gaps；双gap梯度进入P/T。删除绝对ratio及K2/blackout硬acceptance；binary plateau允许几何/连续objective改善，不必先camera-time饱和；yaw继续只影响FOV |
| [planner_manager.cpp::EGOPlannerManager::finalizeCapturedCandidates](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1063) | 删除因破坏normal geometry而直接RETAINED_PREVIOUS的额外分支；所有安全且handoff通过的候选按geometry/visibility/intent排序。有safe successor即提交，没有几何进展也不阻塞 |
| [planner_manager.cpp::EGOPlannerManager::evaluateLocalGeometry](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:932) | 同一future activation下比较active及candidate，返回ratio/E/gaps/confidence/intent cost；peer旧时采用已知polynomial及terminal hold，或限时odom外推后保守保持，不等peer刷新 |
| [planner_manager.cpp::EGOPlannerManager::logExecutionGeometry](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:988) | local、persistence、brake、joint均保留三态记录与urgent recovery；joint记录也关联提交前baseline |
| [poly_traj_optimizer.cpp::bool PolyTrajOptimizer::samplePointsToCheck](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp:3606) | 修复本次实测发现的采样范围bug，详见下文；全部调用显式传入本次轨迹的full/prefix范围 |

完整21个修改文件（含tests/header/launch）及相对本轮起点的diff：[changed_files.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/changed_files.json)、[production_changes.diff](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/production_changes.diff)。原数学visibility/J_acc/K2/blackout定义与权重未重写；按用户本轮要求改变了它们在selector/acceptance中的资格门语义。没有恢复固定UAV→120°slot。

**几何目标与恢复意图**

`E_enc = Σ_j([g_j−170°]_+² + [25°−g_j]_+²)`，实际计算使用rad，优化梯度沿当前sample的排序分支，trial重新排序。三gap总和仍为2π。正常点态定义始终是`g_min>=25° && g_max<=170°`；near-horizon ratio>=0.8只定义NORMAL状态，不是退化恢复的绝对资格门。near horizon仍为1.5s，local采用0.1s中点平均，team采用统一世界时间样本平均；ratio按有效时长积分，binary camera-time沿用原range AND static LOS AND dynamic LOS AND FOV定义。

ROS参数均在`/encirclement_geometry/`：`recovery_eps_geometry=.0025 rad²`、`recovery_eps_ratio=.02`、`recovery_transition_slack=.20 rad²`、`recovery_intent_weight=.02`、`peer_geometry_fresh_age=.5s`。原local eps及urgent_replan_period=.15s不变。transition_slack是**退化解间的短期E变化容忍量**，不改变170°/25°的合围分类，不放宽任何安全阈值。它允许测试中的230°→235°短期绕障；之后继续优化，不能把该时段计作正常合围。

相对joint接受：退化baseline下`E_after<E_before−eps_geometry`或`ratio_after>ratio_before+eps_ratio`可直接构成恢复收益；短期plateau/绕障需E变化在slack内且intent cost保持近似一致或后续终点geometry改善，同时通过全部hard safety、binary camera-time不回退及相对收益判断。正常baseline继续比较ratio/E的不退化。未选中的joint增强不会暂停local。K2/blackout保留cost和排序影响，已按本轮原则取消其绝对覆盖门。

RecoveryIntent记录plan_id、创建/确认时刻、最大gap两个端点UAV、每机角向descent贡献和期望bearing。两端共同缩小大gap，压缩小gap的梯度同时作用于对应邻机；不是让单机独自修复，也不以yaw充当位置合围。意图角参考来自当前几何梯度，不来自120°。相邻周期梯度方向一致时保留，出现相反梯度/失效时更新；明显更优或唯一安全候选可以绕过该弱偏好。Generator通过既有reference offsets把共同意图交给三机；各组件的soft plan_id是诊断/偏好状态，不取代原generation/hypothesis/team_solution_id协议，不要求它们同步等待。previous accepted normal seed仍须真实三机activation及正常geometry；它与未完成恢复的soft intent分开。

Peer confidence仅降低geometry比较可信度，**不会降低peer碰撞检查**。本次runtime peer_confidence均为1，过旧分支由contract覆盖；不能声称本次实测发生了peer stale压力场景。

**两次完整运行及唯一追加修复**

| FULL ON版本 | 严格合围 | camera-time | terminal hold | 说明 |
| --- | --- | --- | --- | --- |
| nonblocking_recovery_20260911 | 0.306431 | 207.165113 | 1次 / 28.242178s | 首次完整验证暴露制动采样范围bug；847个terminal-hold未验证样本，保留原始数据 |
| nonblocking_recovery_brake_sampling_20260911 | 0.406460 | 230.964219 | 0 / 0.000000s | 修复明确bug并build/test后必要验证；没有第三次运行 |

首轮UAV0的id204为已验证制动，activation=1789061945.579010487，duration=1.230477s。后续制动通过安全和handoff，但`setLocalTrajFromOpt(... touch_goal=true)`调用的`computePointsToCheck`读取的是上一次rolling solve遗留的成员`touch_goal_=false`。采样到duration边界后返回false，日志明确`touch_goal_=0, pts_check.size()=5`，继而`POINTS_TO_CHECK_FAILED`和`BRAKE_UNAVAILABLE`。这使execution reserve刷新失败，最终id204在1789061946.812950进入terminal hold。

修复把full_trajectory作为显式参数穿过local commit、team ACK预检及optimizer自身调用，采样本身提取为可测试函数。**安全检查没有跳过**，改的是检查点元数据的采样范围；不使用未发布hold增加coverage。新增数值复现：同一1.230477s单piece制动在错误prefix模式失败，在正确full模式返回有效检查点。证据：[brake_sampling_root_cause.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/brake_sampling_root_cause.json)。最终46次制动提交、0次BRAKE_UNAVAILABLE、0次制动metadata拒绝；该bug未再复现。

**最终完整区间统计**

场景`long_cylinder_forest_visibility_stress.json`，与首轮/feedback_32相同FULL ON参数，native RViz。Target正常完成route，80.204541s；统一有效区间`[1789062418.950864077, 1789062499.104460716]`，RUN_DURATION=80.153597s。取三机实际trajectory/visibility均有效的起点至target mission结束，不剔除退化、低速、遮挡或风险片段。按实际dt积分；geometry由实际UAV odom和target odom重算，gap分位数时间加权，不能以generator/joint predicted值代替。

| Geometry / recovery | 全程结果 |
| --- | --- |
| EXECUTED_ENCIRCLEMENT_RATIO | 0.406460 |
| SAME_SEMICIRCLE_RATIO (`g_max>=π`) | 0.288722 |
| GAP_MIN P50/P95/min | 46.296545 / 74.790063 / 0.576766 deg |
| GAP_MAX P50/P95/max | 172.292570 / 213.296315 / 315.308254 deg |
| NORMAL_ENCIRCLEMENT_RATIO | 0.274528 |
| RECOVERING_ENCIRCLEMENT_RATIO | 0.038844 |
| SAFETY_DEGRADED_RATIO | 0.686628 |
| DEGRADED_EPISODE_COUNT | 48 |
| DEGRADED_EPISODE_DURATION P50/P95/max | 0.066276 / 4.466349 / 5.699815 s |
| RECOVERY_SUCCESS_COUNT | 47 |
| RECOVERY_TIME P50/P95/max | 0.066263 / 4.499683 / 5.699815 s |

三态比例来自实际执行identity关联的预测几何状态：三机全NORMAL才计团队NORMAL，任一SAFETY_DEGRADED则团队降级，其余有RECOVERING则计恢复；无UNKNOWN缺口。该保守团队聚合与odom点态严格合围ratio不同。episode是实际geometry逐帧连段、不做去抖；短至约0.066s的边界往返计入47次恢复，不能把47理解为47次大型恢复规划成功，任务末仍有1段未恢复。

| Executed visibility | 全程结果 |
| --- | --- |
| ACCUMULATED_CAMERA_VISIBLE_TIME | 230.964219 camera·s |
| MEAN_VISIBLE | 2.881520 |
| K2 | 0.985032 |
| ALL3 | 0.896488 |
| NONE | 0.000000 |
| LONGEST_K2_LOSS | 0.600057 s |
| LONGEST_BLACKOUT | 0.000000 s |

相对feedback_32：严格合围下降17.04个百分点，同半平面增加约0.82个百分点；camera-time增加约0.20%，K2从0.983155变为0.985032，All3从0.891989变为0.896488。可见性没有明显牺牲，但不能掩盖几何结果退化。两次不同代码版本的必要验证不是统计A/B。

| UAV | replan Hz | candidate Hz | commit Hz | actual activation Hz | activation间隔 P50/P95/max s |
| --- | --- | --- | --- | --- | --- |
| 0 | 3.655482 | 6.400212 | 4.104619 | 4.104619 | 0.260113 / 0.355649 / 1.369936 |
| 1 | 4.940514 | 7.685245 | 3.867574 | 3.855098 | 0.259979 / 0.496083 / 1.409830 |
| 2 | 4.478901 | 7.223631 | 4.017287 | 4.017287 | 0.260101 / 0.379234 / 2.510843 |

SAFETY_DEGRADED期间，按该状态的55.035729s总时长计算实际activation rate，UAV0/1/2分别为4.269953 / 3.833873 / 4.124593 Hz。geometry绝对ratio拒绝计数=0；后台pending不冻结local，stale head=0、zero executable pending=0。4次coordinator timeout只影响后台增强；PVA/旧generation拒绝保持有效。

| Lifecycle / safety / performance | 最终结果 |
| --- | --- |
| TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT | 0 |
| TERMINAL_HOLD_COUNT / TOTAL | 0 / 0.000000 s |
| VALIDATED_BRAKE / BRAKE_UNAVAILABLE | 46 / 0 |
| HANDOFF dp/dv/da max | 0.004106m / 0.021842m/s / 0.096358m/s² |
| PLANNING_LATENCY P50/P95/max | 4.146500 / 34.492650 / 835.616000 ms |
| PLANNING_BUDGET P50/P95/max | 821.455500 / 1093.111450 / 1128.081000 ms |
| GENERATOR latency mean/P95/max | 15.680373 / 20.042900 / 23.414000 ms |
| JOINT latency mean/P95/max | 3.409709 / 4.603000 / 7.456000 ms |
| COLLISION / SWARM_VIOLATION / UNVALIDATED_SAMPLES | 0 / 0 / 0 |
| MIN_STATIC / MIN_DYNAMIC_CLEARANCE | 0.253873 / 0.738391 m |
| MIN_SWARM elliptical distance | 0.753592 m |
| MAX_COMMAND / MAX_ODOM_SPEED | 3.141344 / 3.161125 m/s |
| MAX_PLANNER_TRAJECTORY_SPEED | 3.148015 m/s |
| EMERGENCY_STOPS / HEARTBEAT_LOSSES | 0 / 0 |

960次真实handoff全部在原0.02m / 0.05m/s / 0.10m/s²范围内；da最大值0.096358接近原阈值，不能仅用P50微小残差概括。最终触发3次objective evaluation取消、2次optional refinement停止，既有deadline/coverage仍工作。Static/dynamic clearance是运行CSV中的实际表面距离，swarm为原elliptical距离，不能将不同模型的clearance数字直接等同为同一约束。

SAFETY_REGRESSION_OBSERVED=YES沿用前两份报告的保守验收口径：**名义3m/s仍被超过**；不是宣称本次相对baseline发生统计显著的安全恶化。原5%可行性数值容差未变，planner最大3.148015低于其3.15 gate；odom最大3.161125仍超名义值。碰撞/未验证执行为0并不意味着所有名义速度验收项通过。

**恢复链实际执行证据**

110次generator调用，330个hypothesis中45个尚未满足点态25°/170°仍被输出评价；111个ratio<0.8组合被selector选出。141次joint尝试、131次成功proposal、77次commit及77组三机实际执行；其中98次joint尝试的after ratio<0.8，93次被接受，**58组完成真实三机接管/optimized yaw执行**。证据按coordination_generation关联，避免重复encirclement_generation/hypothesis误配。

具体team_solution_id=69、coordination_generation=76、encirclement_generation=53、hypothesis=0：E从0.514321248降到0.418543366，ratio从0.066667升到0.200000，仍远低于0.8，照常proposal→3 ACK→commit→adoption。proposal=1789062450.967065573，activation=1789062451.202414274，remaining lead=235.349ms≥原85ms。三机真实activation分别1789062451.206077 / 1789062451.206533 / 1789062451.205518，对应trajectory id146/137/145、各5个yaw knots，执行日志确认optimized yaw。P/T/yaw change norm分别0.176508112 / 0.147181037 / 0.000352592。该事件camera-time before/after均4.5 camera·s、K2/All3均1，**只证明恢复和执行闭环，不声称这一个饱和事件提高可见性**。

详见[analysis/relative_recovery_execution_proof.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/analysis/relative_recovery_execution_proof.json)、[analysis/verified_chains.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/analysis/verified_chains.json)、[analysis/nonblocking.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/analysis/nonblocking.json)。

**剩余低速与几何限制**

UAV1在mission39.751–42.185s低于0.15m/s持续2.433682s，期间11个planning batches、12次activation（11个VALIDATED_BRAKE及1个SIDE_PLUS）；在47.918–53.385s低速5.467084s，期间11个batches、10次activation（9个制动及1个FEASIBLE_FALLBACK）。NOMINAL反复因预测dynamic距离0.53–0.79m小于原1.10阈值被判INVALID，另有static拒绝；safe替代出现后继续接管。这里没有geometry等待或terminal hold，但**确实有安全制动导致的可见低速/暂停**。证据[analysis/low_speed_nonblocking_audit.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/analysis/low_speed_nonblocking_audit.json)。它只说明当前枚举候选的失败原因，不证明环境中所有可能绕障路径都无解。

跨周期soft intent及相对恢复不提供全局收敛保证。最终normal占比仍低，存在很多边界往返；允许短期过渡之后，未来peer预测、local安全候选可用性和实际三机联合动作仍可能使E反复变化。下一步若继续优化，应针对这些退化episode的候选供给及预测/实际差异，不应恢复绝对ratio门、强锁恢复方向、放宽25°/170°或调高固定120°项。本轮在完成有明确原因的两次验证后停止，没有为改善40.65%数字扩大修改。

**Build、tests、runtime及清理**

最终`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`为6/6 PASS。16组unit/gradient/contract全部PASS：原visibility/MINCO P/tau/yaw/时序梯度、实际ROS executor PVA/generation契约保留；新增230→200相对恢复、230→235短期一致绕障、无进展safe successor、normal相对保护、双周期intent/明显收益解锁、peer confidence无等待、真实ratio<0.8 joint→三ACK commit，以及制动显式采样范围复现。多小gap同时违反的解析梯度有限差分误差2.74e-9；原局部积分deadline取消测试仍通过。

formation manager/coordinator/generator/joint为-O2，planner/traj_opt/traj_server为既有-O3；没有重用旧devel产物。运行`/proc`可执行文件hash和加载.so与build manifest匹配，最终运行后生产源码hash未变。[build_final.log](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/build_final.log)、[tests.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/tests.json)、[build_manifest.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/build_manifest.json)、[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/final_verification.json)。

两次均启动native RViz，进程、配置和显示项已核对；owned-window抓图的OpenGL画面仍为黑色，不能声称通过该图做了全程人工视觉确认。结论来自实际odom/command/polynomial身份数据；没有为补图再跑仿真。最终绘图[analysis/encirclement_execution.png](/home/bob/ALP/egov2_fc65423_constvel/nonblocking_recovery_brake_sampling_20260911/analysis/encirclement_execution.png)用于查看真实gap和activation时序，不替代RViz实况。

两次任务均正常到target结束；本轮roslaunch/RViz/recorders/test masters全部停止，12563/12564/12568/12569端口关闭，无本轮进程残留。不处理其他用户进程。未访问/修改禁止目录。报告以目录最大序号32加1创建，不覆盖旧feedback。

```text
PRODUCTION_SOURCE_CHANGED: YES
ABSOLUTE_RECOVERY_RATIO_GATE_REMOVED: YES
NEAR_1P5S_STILL_BLOCKS_RECOVERY: NO
GENERATOR_RECOVERY_HYPOTHESES_ALLOWED: YES
SELECTOR_RELATIVE_GEOMETRY_RECOVERY_ACTIVE: YES
JOINT_RELATIVE_GEOMETRY_ACCEPTANCE_ACTIVE: YES
LOCAL_GEOMETRY_IS_EXECUTION_GATE: NO
RECOVERY_INTENT_IMPLEMENTED: YES
RECOVERY_INTENT_CAN_BLOCK_LOCAL_REPLAN: NO
PEER_STALE_CAN_BLOCK_LOCAL_REPLAN: NO
120_DEGREE_RESTORING_REINTRODUCED: NO
PENDING_BLOCKS_LOCAL_REPLAN: NO
TERMINAL_HOLD_COUNT: 0
ACTUAL_ACTIVATION_RATE_UAV0/UAV1/UAV2: 4.104619 / 3.855098 / 4.017287 Hz
EXECUTED_ENCIRCLEMENT_RATIO: 0.406460
SAME_SEMICIRCLE_RATIO: 0.288722
SAFETY_DEGRADED_RATIO: 0.686628
ACCUMULATED_CAMERA_VISIBLE_TIME: 230.964219 camera·s
MEAN_VISIBLE: 2.881520
K2: 0.985032
ALL3: 0.896488
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
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_33.md
```
