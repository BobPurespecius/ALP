已实现最大空扇区约束、筛选、联合优化、非线性 acceptance 与执行遥测，并补齐本机滚动轨迹前缀的几何恢复入口。**本轮仍未通过完整生产验收。** 最终运行 SAME_SEMICIRCLE_RATIO=33.10%，严格满足 25°/170°条件的 EXECUTED_ENCIRCLEMENT_RATIO=42.65%；38 个正常合围 team solution 完成三机实际执行，但出现一次任务中 terminal hold（约1.94 s）、58个未验证 hold 样本和超过名义3 m/s的速度。不能把“几何约束链已实现”写成“全程严格合围、安全与连续性全部通过”。

本报告对应最新生产源码，最终验证在 true_encirclement_gap_recovery_20260910。本轮共两次完整 FULL ON + native RViz：第一次暴露滚动前缀未接入 gap 的缺口，针对性补齐后第二次验证；无 OFF、无 A/B、无场景调整，没有第三次重跑寻找有利结果。第一次数据单独保留在 [true_encirclement_gap_20260910](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_20260910)，以下主统计全部属于第二次。

**真实结构与修改范围**

feedback_30 的 rolling-local + asynchronous-team 结构与源码一致。AdaptiveViewpointGenerator 以多个 seed 做廉价 visibility 搜索；manager 发 EncirclementReferenceBundle；FSM 生成 NOMINAL/SIDE 候选并立即维持 local successor；TopologyCoordinatorCore 使用候选轨迹做组合筛选；TeamVisibilityOptimizer 求解 P/tau/yaw，随后经过原 proposal/ACK/commit、planner adoption、traj_server scheduled activation。此前局部 MINCO 的 angular restoring 已移除，tracking prefix 只保留 radius/height band，这也是新增 selector 条件后必须补上几何恢复入口的原因。

| 文件 / 主要函数 | 修改 |
| --- | --- |
| [encirclement_geometry.h](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/include/multi_uav_formation/encirclement_geometry.h) | 共享 sorted circular gaps、合法性、最大 gap 梯度、参数检查和分位数 |
| [adaptive_viewpoint_generator.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/adaptive_viewpoint_generator.cpp) / evaluateCached、generate | 搜索前廉价几何筛选；fixed/uniform/previous/current 均不能绕过最终有效性 |
| [topology_coordinator_core.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/topology_coordinator_core.cpp) / evaluate、select | 同世界时间 trajectory 几何积分；先 eligibility，再 coverage/camera-time |
| [team_visibility_optimizer.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/team_visibility_optimizer.cpp) / evaluate、optimize | gap 代价与 MINCO P/tau 梯度；trial/final nonlinear ratio guard |
| [multi_uav_topology_coordinator.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp) | 统一 ROS 参数，selected/joint geometry telemetry |
| [cooperative_viewpoint_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/cooperative_viewpoint_manager.cpp) / adaptiveTick | 实际三机 adoption + odom geometry 才传播 previous seed；共享 C++ geometry 执行 CSV |
| [poly_traj_optimizer.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) / trackingGradCostP、setParam | 把同一 gap 恢复代价接入 rolling prefix；包含 peer/target 世界时间梯度 |
| [native_egov2_rviz.launch](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch) | 集中暴露 min/max/ratio/joint/local weights 和 CSV 路径 |
| [CMakeLists.txt](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/CMakeLists.txt) 与 test/ | 新数值/梯度/契约测试；历史任意直线 fixtures 显式隔离 geometry；新测试使用正常默认 geometry |


头文件中的 params/metrics 同步扩展。完整本轮源码差异：[production_changes.diff](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/production_changes.diff)；文件列表：[changed_files.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/changed_files.json)。未修改 FSM、planner_manager、traj_server 的生命周期、安全阈值或接管协议源码；未修改 J_vis、J_acc、K2/blackout 的数学定义。

**圆周几何与参数**

对目标相对 XY bearing 按真实角度排序，计算三段 circular gap，总和2π。正常 geometry 要求 `g_min>=25° AND g_max<=170°`。ROS 全局参数位于 `/encirclement_geometry/`：

| 参数 | 默认值 | 语义 |
| --- | --- | --- |
| min_angular_separation_deg | 25 | 最小相邻圆周分离 |
| max_circular_gap_deg | 170 | 最大无人空扇区；必须小于180° |
| ratio_min | 0.8 | 正常近端 team 解至少80%时长满足双条件 |
| joint_gap_weight | 4 | joint mean squared gap violation |
| local_gap_weight | 100 | rolling prefix 几何恢复权重 |
| executed_csv | visibility.csv.encirclement.csv | 共享 C++ helper 的 actual-odom 几何数据 |


170°给半平面边界保留10°余量；0.8在1.5 s near horizon允许最多约0.3 s过渡，不放宽静态/动态/机间安全。极角在目标位置不可定义时判 geometry invalid；参数拒绝最大gap≥180°、不可行角度范围、NaN。

`J_gap=mean_t([g_max(t)-theta_gap_max]_+²)`。若最大 gap 由顺时针相邻 a→b 定义，局部分支上 `dJ/dtheta_a=-2e`、`dJ/dtheta_b=+2e`，再乘 `dtheta/d(x,y)=(-y,x)/r²`。一次 QP 的梯度使用该 evaluation 的 cyclic branch，trial 和下一次线性化重新排序；不是按 UAV 编号排列。相等最大值取确定的合法 subgradient。满足界限后代价及梯度均为零。joint 使用已有固定世界时间 MINCO P/tau 链式求导；yaw 不直接改变位置几何。

本机 rolling recovery 使用同一 helper，以本机 sample 和另外两机世界时间预测构成三机 bearing。peer 到期遵循现有 terminal-hold model，尚未到 start 的 peer 不参与该项，完整三机预测可用才计算。`gradt` 包含本机速度、peer速度和target速度；`grad_prev_t` 包含显式世界时间移动，避免把 peer/target 当常量漏掉时长梯度。local weight=100 与当前 local radius/height tracking weight=100 同一量级，用于补齐几何恢复而非向某一固定 bearing 拉回；没有根据第二次结果再调大权重。它是软恢复项，不能声称所有 local fallback 满足硬几何。

120°仍只有 initialization/uniform seed 和候选 tie 的作用；没有新增 `(theta-theta_120)^2`，没有把 delta_i 压向零。0/100/220°与0/120/240°在满足界限时 gap 代价都为零。visibility 继续决定可行域内的具体角度。

**selector、acceptance、previous seed**

Generator 在 expensive LOS evaluation 前进行廉价几何过滤；fixed/uniform/previous/current 一视同仁，seed 可作为搜索起点，不能以 seed 身份直接输出非法 geometry。visibility 使用原 tracking_visibility_geometry.h 中的定义和 snapshot cache。

Selector 顺序为 hard safety → near-horizon encirclement ratio eligibility → K2/blackout protection → binary accumulated camera-time → 原 All3/transition/tracking 等 tie。gaps 从实际候选 trajectory 的同一世界时间样本计算，非 reference proxy。没有正常组合时返回带 generation 的 terminal negative：`ENCIRCLEMENT_DEGRADED_NO_ELIGIBLE_SAFE_COMBINATION`，本机滚动规划继续，不能把这种 fallback 计为合围成功。

Joint 对 trial 重新排序：若当前解已满足 ratio，trial 不允许跌破；最终 acceptance 无条件要求 ratio>=0.8，即使 camera-time 或 objective 改善也不能豁免。原 binary camera-time non-regression、非饱和改善、饱和 tie、K2/blackout保护完整保留。J_acc仍是主 visibility 项；新增几何定义是 eligibility，不是把camera-time换回120°恢复。

Previous accepted offset 仅在3个真实 execution-adoption identity一致、已到activation、payload safety_validated，且当前实际odom三机geometry和该hypothesis的geometry都满足条件后保存。初始fixed offsets不会因此自动成为accepted previous seed。

**第一轮暴露的接入缺口**

第一轮 generator 输出合法geometry，但 local prefix没有几何梯度。selector在执行前筛掉不满足ratio的组合后，joint gap根本没有机会修复本机前缀。全程严格合围只有0.4568%、同半平面99.0953%，正常joint尝试/commit均0；local实际接管仍约3 Hz且零hold。该结果明确说明“只在generator和team入口过滤”不够。

因此补齐上述 rolling prefix cost 后才做第二次验证。两次都是同一Scenario A、相同安全和visibility参数；第二次新增的是缺失的恢复路径，不是筛掉第一轮不利时间段。第一轮统计不能替代最终版本的安全数据。

**构建、测试、运行来源**

`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`：6/6 packages成功。最终编译有一条原有 `base_delta_phi` unused-variable warning；不是新增安全或数学错误。formation manager/coordinator/generator/joint保持-O2；traj_opt、planner、traj_server保持现有-O3，无fast-math。实际运行executable hash和loaded .so与本工作区构建一致，RViz使用系统ROS executable及native配置。详见 [build_recovery.log](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_20260910/build_recovery.log)、[build_manifest.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/build_manifest.json)、[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/final_verification.json)。

15组检查全部通过：原生命周期数值/真实traj_server契约、6个formation测试、新gap测试、3个traj_opt测试、Python visibility/adaptive/lifecycle契约。新增测试包括[-40,20,80]判非合围、[0,100,220]合法、120无特殊奖励、同侧camera-time高者被过滤、合法generator output进入selector、真实joint nonlinear同侧解拒绝。gap XYZ有限差分、MINCO P/tau有限差分、local sample/previous-time梯度均通过；P/tau最大误差约1.43e-9。历史使用任意直线的visibility fixtures显式令ratio=0、gap_weight=0以隔离原数学目标；新增默认配置测试负责严格geometry，不影响生产参数。证据：[tests.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/tests.json)、[encirclement_geometry_contract_test.log](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/encirclement_geometry_contract_test.log)。

**最终完整运行的统计口径**

Target正常完成路线，mission duration=80.204540745 s。所有主统计统一世界时间 `[ 1789054861.056388855, 1789054941.200317621 ]`，有效 RUN_DURATION=80.143928766 s；裁掉首帧前的数据缺口及关闭阶段，不剔除hold、geometry退化或安全异常。实际odom+target每帧重算bearing/gap，左端保持按真实dt积分；共2406个geometry时间段。gap P50/P95为时间加权分位数，角度单位degree；same-semicircle严格按g_max>=π。

在线C++共享helper CSV保留850条记录（含启动/结束缓冲），与正式统计用的全程actual trajectory CSV并存。主几何结果来自后者三机同帧数据，便于和visibility/clearance统一区间。未用predicted比例替代executed比例。

| Executed geometry | 值 |
| --- | --- |
| EXECUTED_ENCIRCLEMENT_RATIO | 0.426541287 |
| SAME_SEMICIRCLE_RATIO | 0.331045747 |
| GAP_MIN_P50/P95/MIN | 49.404152/103.858452/0.097029 |
| GAP_MAX_P50/P95/MAX | 172.984927/253.381496/315.360919 |


约66.90%时间三机不能被同一个半平面覆盖，说明已改变长期同侧跟踪；但严格170°/25°合围只有42.65%，**尚不能说严格合围占多数或达到0.8全程比例**。g_max>170°占56.31%，其中170°–180°边界带23.21%；g_min<25°占19.88%（可与g_max超限重叠）。这些时间明确是ENCIRCLEMENT_DEGRADED。

| Executed visibility | 值 |
| --- | --- |
| ACCUMULATED_CAMERA_VISIBLE_TIME | 222.934220 camera·s |
| MEAN_VISIBLE | 2.781673 |
| K2 | 0.994593 |
| ALL3 | 0.787080 |
| NONE | 0.000000 |
| LONGEST_K2_LOSS | 0.433352 s |
| LONGEST_BLACKOUT | 0.000000 s |


binary visibility仍为range AND static LOS AND dynamic LOS AND camera FOV，实际odom/yaw求值。未运行OFF，不由本轮开发运行声称统计意义上的visibility提升。

| UAV | VISIBLE_RATIO | static LOS loss s | dynamic LOS loss s | HFOV loss s | VFOV loss s | range loss s |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 0.943423 | 3.900450 | 0.633854 | 0.000000 | 0.000000 | 0.000000 |
| 1 | 0.873134 | 0.531945 | 0.633525 | 8.969017 | 0.000000 | 0.134886 |
| 2 | 0.965116 | 1.996325 | 0.799419 | 0.000000 | 0.000000 | 0.000000 |


**pipeline 与实际执行证据**

有效任务窗 generator trigger=124，生成并记录367个hypothesis，全部g_min/g_max合法；selector正常eligibility通过51次、joint尝试51次、joint success/proposal40次、commit38次、planner adoption114次、真实三机team executions38组、traj_server optimized yaw execution114次。最终normal joint的ratio检查有效；10次camera-time acceptance拒绝和1次initial swarm separation拒绝未被绕过。

示例 team_solution_id=1、encirclement_generation=5、hypothesis=0：FINAL_VIEWPOINT_ANGLES=(-105,-165,60)°，DEVIATION_FROM_120=(-45,15,0)°，reference gaps为60/165/135°。编号0是当前adaptive geometry，不等于120seed；旧runner仅把hypothesis1/2认成adaptive，故其`complete_chains=0`不是实际无团队链，本报告按payload身份重建。

Selector近端ratio=1.0、g_max P50/P95/max=147.614/156.334/159.340°；joint after ratio=1.0、153.848/156.288/156.301°。三机实际activation后最近完整odom帧g_max=135.654899°，g_min=103.059208°，geometry有效。

该event camera-time before/after=4.5/4.5 camera·s，mean_visible=3/3、K2=1/1、All3=1/1、longest K2 loss/blackout均0。P_CHANGE_NORM=0.000193555、T_CHANGE_NORM=0.000040051、YAW_CHANGE_NORM=0.000016086。这是饱和event，**只证明真实链路成功，不声称visibility提高**。

Proposal=1789054863.068138599，activation=1789054863.636510849，proposal余量568.372 ms，要求85 ms未降低；UAV0/1/2 ACK齐全，commit acked=1,1,1。实际traj_server adoption UAV0=1789054863.6457539(id7/gen7)，UAV1=1789054863.6397760(id9/gen9)，UAV2=1789054863.6388204(id7/gen7)，各自5个yaw knots；实际optimized yaw日志逐id交叉验证。完整38组：[verified_chains.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/analysis/verified_chains.json)。

角度随visibility变化，没有等角奖励。全程wrapped viewpoint角度范围：UAV0[-165.461,180]°、UAV1[-165,180]°、UAV2[-134.539,150.461]°；relative-120 deviation范围分别[-120,179.539]、[-165,150]、[-179.539,165.461]°。这是带wrap的范围，不能将普通线性均值当圆周均值。

**滚动接管、handoff 与安全**

| UAV | replan Hz | candidate Hz | local+team commit Hz | actual activation Hz | activation interval P50/P95/max s |
| --- | --- | --- | --- | --- | --- |
| 0 | 4.117592 | 7.162115 | 3.244163 | 3.244163 | 0.270674467/0.594078076/2.670189142 |
| 1 | 3.668400 | 6.712923 | 3.206731 | 3.194253 | 0.279988766/0.572406530/1.920774937 |
| 2 | 4.279800 | 7.324323 | 3.456282 | 3.443804 | 0.279841900/0.507838726/2.770036459 |


Pending=657，后台timeout=7，stale head=0，zero-executable pending=0。pending P50/P95/max=0.178000000/0.331526400/1.007693000 s。约3Hz rolling activation保留，未重新引入1秒pending冻结。任务窗边界存在已commit但尚未activation的消息，因此commit不强行等同activation。

| actual active→new | P50 / P95 / max |
| --- | --- |
| dp | 0.000000365/0.000001489/0.000003487 |
| dv | 0.000000026/0.000000104/0.000002148 |
| da | 0.000000016/0.000000720/0.000004722 |


共792次实际handoff，全部在既有0.02m/0.05m·s^-1/0.10m·s^-2 tolerance内，未放宽。按source transition的完整残差：[summary.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/analysis/summary.json) 的handoff_by_source，及 [switches.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/analysis/switches.json)。未来1s两代曲线最大空间差P50/P95/max=0.054802588/0.435095248/1.352840445 m；头部C2通过不代表远端形状固定。

| 安全/执行 | 值 |
| --- | --- |
| MIN_SWARM_DISTANCE (elliptical) | 0.681132 m |
| MIN_SWARM_DISTANCE (Euclidean) | 0.681302 m |
| MIN_STATIC_CLEARANCE | 0.148928 m |
| MIN_DYNAMIC_CLEARANCE | 0.895355 m |
| COLLISION_SAMPLES | 0 |
| SWARM_CLEARANCE_VIOLATION_SAMPLES | 0 |
| MAX_COMMAND_SPEED | 3.144895 m/s |
| MAX_ODOM_SPEED | 3.265231 m/s |
| MAX_PLANNER_TRAJECTORY_SPEED | 3.144907 m/s |
| EMERGENCY_STOPS | 0 |
| HEARTBEAT_LOSSES | 0 |
| UNVALIDATED_EXECUTED_SAMPLES | 58 |
| UNVALIDATED_NEW_POLYNOMIAL_ADOPTIONS | 0 |
| TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT | 1 |
| TERMINAL_HOLD_COUNT | 1 |
| TERMINAL_HOLD_TOTAL/MAX | 1.940499 s |


Static/dynamic clearance为点到原始表面；collision≤0；机间使用既有elliptical sqrt(dx²+dy²+0.25dz²)<0.5m标准，没有用欧氏距离掩盖违规。0collision/0swarm violation不等于全项安全通过：58个terminal hold样本明确safety_validated=false，且速度超过名义3m/s。既有5% feasibility tolerance未改，所以command3.144895仍在原3.15 gate内；odom还存在跟踪超调。SAFETY_REGRESSION_OBSERVED=YES指本次出现执行验收异常，不代表进行了统计对照试验。

| Trajectory source | 单机执行样本数 |
| --- | --- |
| NOMINAL | 4954 |
| SIDE_PLUS | 123 |
| SIDE_MINUS | 124 |
| FEASIBLE_FALLBACK | 7 |
| PERSISTENCE_FALLBACK | 1104 |
| JOINT_TEAM_SOLUTION | 789 |
| VALIDATED_BRAKE | 56 |
| TERMINAL_HOLD | 58 |
| OTHER | 0 |


**尚未解决的根因与最小后续方向**

1. CONFIRMED：局部恢复仍是有限权重soft cost，safe local successor不要求normal-team的0.8 ratio；在障碍/候选不可行时，正常team guard只能拒绝，不能保证fallback维持合围。84次selector因无eligible safe combination降级，实际g_max常贴近/越过170°，g_min也会不足25°。下一步应明确local recovery的几何可行性/进展与短时退化边界，让双gap要求进入local约束或带验证的恢复步骤；不能靠重新引入固定120°，也不能只调大权重或降低170°标准。

2. CONFIRMED：UAV0 id136/gen136 VALIDATED_BRAKE仅0.6s，old_end=1789054899.148931。新制动id137/gen137 activation=1789054901.089632；真实terminal hold从1789054899.154786到1789054901.095273，约1.940486s，位于mission38.159–40.100s，非正常mission结束。期间三次planning batch耗时799.074/756.058/869.680ms，均pending=0。此前候选因NOT_ABSOLUTE_SAFE保留旧轨迹，末端以后直到batch返回才重新制动。这是有限brake寿命与不可抢占昂贵solve不匹配；不能写成coordinator阻塞，也不是RViz折线。证据：[hold_evidence.log](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/analysis/hold_evidence.log)、[positive_gaps.json](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/analysis/positive_gaps.json)；已有入口是planner_manager.cpp的validatedBrakeSuccessor/remaining-lifetime检查。最小修复方向是进入昂贵solve前确保已验证successor/hold覆盖其预算，并在budget耗尽前让执行维护有机会运行；所有延长hold或替代轨迹仍需完整安全验证。本轮未继续修改生命周期并做第三次仿真。

3. CONFIRMED：严格全程合围与零terminal hold验收未过。实际速度<0.15m/s的UAV0 mission37.860s起一段持续3.667s，含制动和上述terminal hold；还有mission20.827s约0.8s低速段。不能报告“肉眼/日志已无停顿”。源代码接口与所有测试通过，也不能替代完整任务的物理执行验收。

**性能与RViz**

| 性能ms | count | mean | P95 | max |
| --- | --- | --- | --- | --- |
| generator | 124 | 4.143903 | 5.677550 | 6.126000 |
| hypothesis_planning | 967 | 8.893169 | 14.088200 | 869.680000 |
| candidate_planning | 1699 | 5.041190 | 7.366700 | 799.018000 |
| joint | 51 | 3.048039 | 3.795000 | 4.595000 |


Generator仍毫秒级，问题不在generator。多数常规solve很快不能掩盖接近0.9s的失败长尾。RViz使用native配置，两帧窗口截图实际包含UAV、target、static/dynamic cylinders、planned trajectories、odom path和camera FOV，渲染并非旧XWD的黑OpenGL画面：[rviz_1789054887.png](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/run/rviz_1789054887.png)、[rviz_1789054926.png](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/run/rviz_1789054926.png)。视图支持已出现环绕几何，但全程比例以odom数据为准；没有把稀疏红线误判为polynomial内部不连续。

全程几何/接管图：[encirclement_execution.png](/home/bob/ALP/egov2_fc65423_constvel/true_encirclement_gap_recovery_20260910/analysis/encirclement_execution.png)。normal shutdown完成；本轮roslaunch、RViz、recorders、独立test master无残留。其他不属于本轮的进程未处理。没有访问或修改禁止目录，没有覆盖任何旧feedback。

**最终标记**

这些YES表示相关生产代码已实现并启用；不代表EXECUTED_ENCIRCLEMENT_RATIO达到0.8或零hold验收通过。

```text
PRODUCTION_SOURCE_CHANGED: YES
TRUE_ENCIRCLEMENT_GAP_CONSTRAINT_IMPLEMENTED: YES
MAX_CIRCULAR_GAP_LIMIT_DEG: 170
MIN_ANGULAR_SEPARATION_DEG: 25
120_DEGREE_USED_AS_SEED_ONLY: YES
120_DEGREE_RESTORING_REINTRODUCED: NO
GENERATOR_ENCIRCLEMENT_FILTER_ACTIVE: YES
SELECTOR_ENCIRCLEMENT_PROTECTION_ACTIVE: YES
JOINT_ENCIRCLEMENT_COST_ACTIVE: YES
NONLINEAR_ENCIRCLEMENT_ACCEPTANCE_ACTIVE: YES
EXECUTED_ENCIRCLEMENT_RATIO: 0.426541287
SAME_SEMICIRCLE_RATIO: 0.331045747
GAP_MAX_P50: 172.984927 deg
GAP_MAX_P95: 253.381496 deg
GAP_MAX_MAX: 315.360919 deg
ACCUMULATED_CAMERA_VISIBLE_TIME: 222.934220 camera·s
MEAN_VISIBLE: 2.781673
K2: 0.994593
ALL3: 0.787080
ACTUAL_ACTIVATION_RATE_UAV0/UAV1/UAV2: 3.244163 / 3.194253 / 3.443804 Hz
TERMINAL_HOLD_COUNT: 1
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
FULL_PRODUCTION_ACCEPTANCE_PASS: NO
ON_RUNS_THIS_AGENT: 2
```

REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_31.md
