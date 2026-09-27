# Scenario A 全程 FULL ON + native RViz — 2026-09-10

本轮仅运行一次，target 完整走到终点后停止；没有 OFF、A/B 或重跑，没有修改生产源码、主算法、参数权重或安全阈值。构建成功、实际 adaptive 几何与 joint P/T/yaw 执行已确认，但全程安全不通过：12个静态接触/穿入样本、15个机间距离违规 pair samples，且实际速度采样存在超限。

证据目录：[scenario_a_full_rviz_20260910](/home/bob/ALP/egov2_fc65423_constvel/scenario_a_full_rviz_20260910)。本报告独占创建，没有覆盖旧反馈。

## 1. 运行、构建及配置

执行 `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status`：6/6包成功、无warning，1.5 s。未修改生产代码，因此本轮未重新运行上一轮已通过的unit/gradient/contract suites。

manager、coordinator、adaptive generator、team visibility optimizer、topology core 使用已修正的 `-O2`；ego_planner_node、traj_server、traj_opt 保持原工程 `-O3`。不把整个工程笼统标为全都-O2。运行中 /proc/exe、maps 验证 executable 与实际装载库来自当前 constvel workspace；运行前后构建hash一致。证据：build_manifest.json、run/proc_snapshot.json、final_verification.json。

FULL ON：adaptive manager、encirclement、directional J_vis（weight_visibility=20）、camera-time主目标（team_accumulated_weight=4）、K2/continuity/blackout protection、binary selector/acceptance、joint P/T、joint yaw、predicted attitude/FOV 和 optimized yaw execution 启用。near=1.5 s，far=3.5 s，far_dt=0.25 s。完整实参和ROS参数分别为 run/command.json、runtime_params.yaml。

`enable_minco_visibility_cost=false` 是旧独立MINCO LOS cost开关，不是 directional J_vis 开关。源码 directionalVisibilityCostEnabled 使用 encirclement_configured、guidance_valid、weight_visibility>0，不能把旧开关当成 directional OFF。120°仍仅seed/tie-break，未恢复固定angular restoring。

RViz使用 `ros_ws/src/multi_uav_formation/config/native_egov2.rviz`，在桌面 :0 先启动场景/RViz、预热2 s，再启动实验。已确认target、三机marker、柱状障碍、planned trajectory marker、actual odometry path、executed-yaw FOV均显示，订阅连接见rviz_node.txt，截图见rviz_during.png与rviz_late.png。截图全局状态有黄色提示，但各请求显示有数据；不能把该提示描述为整个RViz全绿。红色轨迹为marker采样显示，不据视觉折线推断polynomial不连续。

## 2. 正常完成与统一统计口径

target route长度/速度确定回放时长 80.204540745 s；mission start=1789041219.420579，mission end=1789041299.625119686。末尾CSV确认target=(36,0,1.5)、速度为0，stop_reason=target_replay_normally_completed。target节点正常到达后持续发布终点，不会自动退出，因此监测到终点后主动正常请求SIGINT停止roslaunch/RViz。

RUN_DURATION（完整mission）=80.204540745 s。统一有效统计区间为相对mission [0.051156, 80.204540745] s，积分时长=80.153384745 s。启动后的首 0.051156 s 缺少完整visibility采样，不虚构为可见或不可见；没有删除任何后续不利区间。关闭阶段、终点静止阶段均排除。

visibility按实际odom/yaw的range AND static LOS AND dynamic LOS AND camera FOV二值判定；逐行复核与CSV visible字段零差异。约30 Hz样本采用左端保持、按真实相邻时间差积分，最后一段裁剪到精确route end。统计末端是积分边界，不是额外实际sample。K2为至少2机、All3为3机、None为0机的时间比例；mean_visible是0–3平均相机数。camera-time为三机可见时长之和。

安全和source分布使用同一时间窗内 7215 个单机实际采样（2405组三机样本）；log事件和性能数据按同一绝对时间窗过滤。原始CSV保留，统计窗CSV在valid_interval。LOS/FOV各分项损失可重叠，不能相加当总不可见时间。

## 3. 全程 executed visibility

| 指标 | 值 |
|---|---:|
| ACCUMULATED_CAMERA_VISIBLE_TIME (camera·s) | 210.222517 |
| MEAN_VISIBLE | 2.622753 |
| K2 | 0.953407 |
| ALL3 | 0.681310 |
| NONE | 0.011965 |
| LONGEST_K2_LOSS (s) | 1.829841 |
| LONGEST_BLACKOUT (s) | 0.462356 |



UAV采用ROS编号0/1/2，对应CSV uav_id=1/2/3。

| UAV | Visible ratio | Static LOS loss s | Dynamic LOS loss s | HFOV loss s | VFOV loss s | Range loss s |
|---|---:|---:|---:|---:|---:|---:|
| UAV0 | 0.902860 | 3.521389 | 0.666355 | 2.632373 | 0.932746 | 0.098672 |
| UAV1 | 0.801388 | 1.933132 | 1.399617 | 11.938839 | 1.155492 | 0.300068 |
| UAV2 | 0.918504 | 2.464272 | 0.700007 | 3.134018 | 0.133734 | 0.167271 |



这是一次完整ON实测，不与此前不同执行区间的OFF/短测做效果对照，也不声称总体可见性提升。

## 4. Adaptive / joint 执行与观察角

ADAPTIVE_VIEWPOINT_TRIGGER_COUNT=54；高可见安全candidate生成日志=248；team selector结果11次；TEAM_VIS_OPT_ATTEMPTS=10；TEAM_VIS_OPT_SUCCESS=6（proposal阶段成功）；PROPOSALS=6；COMMITS=4；THREE_UAV_ADOPTIONS=4。

OPTIMIZED_YAW_EXECUTION_COUNT=32（traj_server的OPTIMIZED_YAW_EXECUTED=1事件）；其中4个已commit team solution均有三机CSV执行和匹配yaw证据。不要把planner接纳scheduled payload当成已经激活。

ADAPTIVE_HYPOTHESIS_SELECTED_COUNT=1（team选择中可明确匹配到不同于初始fixed seed几何的joint事件：solution6）。非零hypothesis_id选择次数=0。二者不矛盾：源码在current无效时可把generated最优项放入index0；后续current还可以保存此前accepted adaptive几何。编号0不是永久120°或fixed-slot模式。监测脚本沿用旧“hypothesis=1/2”启发式会输出complete_chains=0，最终审计已用 generation+hypothesis→真实角度+实际source纠正；没有修改生产协议或日志。

初始current/fixed角约(-150.461°,180°,150.461°)。实际执行还匹配到(15°,150°,120°)、(-75.461°,60°,30.461°)、(-45°,135°,105°)等新几何。最终joint solution6对应encirclement_generation56、coordination_generation11、hypothesis0，角度(-45°,135°,105°)，相对uniform120°seed偏移(15°,-45°,45°)，三机最小角差30°，显然不是严格等角。

生成的158个hypothesis角度日志（包含current、未选中项）典型值：(-150.461°,180°,150.461°)出现28次；(-45°,135°,105°)出现25次。以下范围是wrap后数值范围，跨±180°不代表旋转一整圈：

| 字段 | UAV0 min/max | UAV1 min/max | UAV2 min/max |
|---|---|---|---|
| FINAL_VIEWPOINT_ANGLES | -165.461 / 165.000 | -165.000 / 180.000 | -179.539 / 180.000 |
| DEVIATION_FROM_120_DEG | -135.461 / 90.000 | -165.000 / 180.000 | -165.000 / 120.461 |



solution6证据：proposal=1789041298.687760115；剩余activation lead=235.143 ms≥85 ms；三ACK的planning_generation UAV0/1/2=(84,76,86)，source_candidate=(321,833,517)；commit common activation=1789041298.922902822；三机adoption和实际traj_server yaw对应trajectory_id=(40,35,33)。mission 79.517218–80.185372 s有JOINT_TEAM_SOLUTION、team_solution_id6、encirclement_generation56、hypothesis0的实际CSV。完整链证据见valid_interval/chain_evidence.log、adaptive_geometry_execution_evidence.json。

该事件joint binary预测：camera-time 3.1→3.4 camera·s，mean_visible 2.066667→2.266667，K2 1→1，All3 .066667→.266667，longest K2 loss/blackout均0→0；P/T/yaw change norm=.123922467/.056762762/.173725410。这是未来1.5 s预测；事件接近终点，实际只执行约0.7 s便正常结束回放，不能声称实际执行获得完整预测增益。

## 5. 安全与执行来源

| 指标 | 值 |
|---|---:|
| MIN_SWARM_DISTANCE 欧氏 m | 0.1563992087863613 |
| MIN_SWARM_DISTANCE 椭球 m | 0.15537470251701155 |
| MIN_STATIC_CLEARANCE m | 0.0 |
| MIN_DYNAMIC_CLEARANCE m | 0.962069 |
| COLLISION_SAMPLES | 12 |
| SWARM_CLEARANCE_VIOLATION_SAMPLES | 15 |
| EMERGENCY_STOPS | 0 |
| HEARTBEAT_LOSSES | 0 |
| UNVALIDATED_EXECUTED_SAMPLES | 0 |



clearance是actual UAV位置点到场景原始柱体/墙体的表面距离；实现将内部距离裁为0。因此COLLISION_SAMPLES=静态或动态clearance≤0的单机采样，无法仅凭这个字段区分接触和穿入深度，不额外推测机体物理碰撞半径。机间违规按sqrt(dx²+dy²+0.25 dz²)<0.5 m统计每一对/每个时刻。

12个static collision samples均为UAV1，mission25.417288–25.784160 s，PERSISTENCE_FALLBACK、trajectory12。15个swarm违规样本在56.884599–57.350720 s：UAV1 PERSISTENCE_FALLBACK trajectory24 与 UAV2 NOMINAL trajectory25，最低椭球距离0.155375 m。全在正常mission中，不能归因于停止过程。原始身份见collision_samples.json和swarm_violations.json。

进一步检查实际速度：最高 4.578530 m/s，发生在UAV1、mission80.185372 s、JOINT_TEAM_SOLUTION6 trajectory35；高于配置max_vel=3。该solution预测日志max_velocity=3.143593；本轮未修改既有验收容差或控制器，不能把safety_validated=True解释为真实执行必然安全。未重建全程连续a/jerk/yaw峰值，本报告不证明这些连续约束全通过。

heartbeat loss按真正的[traj-server-heartbeat-stale]事件计数=0，记录的planner heartbeat最大周期12.087 ms；不在包含heartbeat topic名的整行日志上宽泛匹配stale，否则会把generation stale误计为heartbeat loss。EMERGENCY_STOP日志=0。unvalidated source标记样本=0仅表示遥测为True，与实测clearance违规并存。

根因没有扩大到生产修复：fallback静态碰撞、异步多机实际距离以及joint末段速度偏差需要后续专门调查。本轮目的为当前最终版本全程测量，未通过修改算法/阈值掩盖结果。

| trajectory_source | 单机sample数 | 比例 |
|---|---:|---:|
| NOMINAL | 5788 | 80.222% |
| SIDE_PLUS | 186 | 2.578% |
| SIDE_MINUS | 152 | 2.107% |
| FEASIBLE_FALLBACK | 0 | 0.000% |
| PERSISTENCE_FALLBACK | 703 | 9.744% |
| JOINT_TEAM_SOLUTION | 386 | 5.350% |
| OTHER | 0 | 0.000% |



## 6. 性能（同一有效任务区间）

| 项目 | count | mean ms | P95 ms | max ms |
|---|---:|---:|---:|---:|
| VIEWPOINT_GENERATOR_LATENCY | 54 | 7.391537 | 9.555700 | 10.794000 |
| HYPOTHESIS_PLANNING_LATENCY（整批） | 626 | 9.434990 | 37.144250 | 238.732000 |
| JOINT_OPTIMIZER_LATENCY（wall，含失败） | 10 | 2.205500 | 4.269550 | 4.531000 |
| PLANNING_LATENCY（单hypothesis candidate） | 934 | 6.295106 | 17.969350 | 238.715000 |



planning latency以现有stage=CANDIDATE wall计时定义；整批hypothesis规划另列，不能将两者相加或当成端到端异步pipeline时间。无单独全程端到端规划时延序列，未以未带时间戳的stdout total time替代统一时间窗测量。P95使用线性插值。

## 7. 结束与范围

仅1次完整ON。回放正常完成后已结束roslaunch/RViz，最终进程核查无ROS/仿真残留。清理wall时间包含SIGINT等待；launcher退出码0不作为target mission成功证据，成功依据路线时间、终点位置和零速度。

本轮新增的仅为运行/分析脚本、证据与报告；无生产源码修改、无算法修复、无安全margin降低。未访问或修改RRCT。

```text

SCENARIO_A_FULL_RUN_COMPLETED: YES

RVIZ_ENABLED: YES

RUN_DURATION: 80.204541 s

VALID_STATISTICS_DURATION: 80.153385 s

ACCUMULATED_CAMERA_VISIBLE_TIME: 210.222517 camera·s

MEAN_VISIBLE: 2.622753

K2: 0.953407

ALL3: 0.681310

NONE: 0.011965

LONGEST_K2_LOSS: 1.829841 s

LONGEST_BLACKOUT: 0.462356 s

ADAPTIVE_VIEWPOINT_ACTUALLY_EXECUTED: YES

JOINT_PT_YAW_ACTUALLY_EXECUTED: YES

OPTIMIZED_YAW_EXECUTED: YES

SAFETY_REGRESSION_OBSERVED: YES

COLLISION_SAMPLES: 12

EMERGENCY_STOPS: 0

HEARTBEAT_LOSSES: 0

BUILD/RUNTIME_PROVENANCE_CONFIRMED: YES

ON_RUNS_THIS_AGENT: 1

SIMULATION_LEFT_RUNNING: NO

RRCT_ACCESSED: NO

RRCT_CHANGED: NO

REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_27.md

```
