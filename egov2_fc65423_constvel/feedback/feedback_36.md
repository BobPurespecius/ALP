本轮完成 Target → Guide → Trajectory 缩减式重构、指定构建、20 项测试入口，以及唯一一次完整 Scenario A FULL ON + native RViz 运行。**结构和非阻塞执行验证通过，但不能宣称全部性能成功标准已经满足。** 真实合围从54.37%变为62.11%，同半平面从26.64%变为19.77%；camera-time、Target bearing 进展、local 延迟和 brake 指标存在回退。没有调参、OFF/A-B或为结果重复运行。

最终流程：**选 Target，规划 Guide，local 滚动执行；Guide 失效先换 Guide，同 Target 反复无法生成新 Guide 再找替代 Target，找到可规划替代后切换，任何恢复失败都不阻塞飞行。**

## 1. 三层怎样连接

cooperative_viewpoint_manager / adaptive generator 用共享 circularGapGeometry 生成25°/170°合法终态 proposal。按既有 worker score 排序后，用原 adaptive_top_k 限制发布数量（本次至多3个）。终点筛选没有激活权限。

原 team_target_reachability 独立进程现在承担按需 Guide 规划：三个隔离生产 planner 的 NOMINAL/SIDE/A*/MINCO capture → 共同 future activation 的 fresh path 对齐 → 原团队 preflight → 一次性发布 Target + Guide。成功路径就是该快照的路径证据，不存在另一个证书求解器。local 读取同一个 timed target-relative Guide 作为 MINCO seed / 软目标，joint 异步优化 P/T/yaw 后仍走三机 ACK/commit/adoption。Guide 从不直接提交执行。

Guide 监测保留原0.5s周期，但只检查原 epoch 下的新鲜度、耗尽、地图膨胀占据和动态障碍参考可用性。失效清除 Guide，保留 Target；下一次 fresh planning 优先用相同 Target。三次原规则下、位移快照不同的一致 fresh failure 才启动替代搜索，预算/源快照失败不作不可达证明。替代 Guide 真正成功前保留原 Target 身份；无 Guide 时软牵引关闭、local 继续。几何已合围仍按原25°/170°判断释放恢复 Target。

## 2. 实际删除、合并的内容

- 删除旧 Guide 的周期性 trimRecoveryProbe → reanchor → MINCO 重建 → full team preflight；移除 trimRecoveryProbe 接口/实现、worker 的候选 evidence_ 缓存和 finalCheck 的 reuse 分支。保留的 finalCheck 仅服务新生产路径的首次验证。
- 删除 RecoveryIntent struct 的 plan id、created/confirmed、refresh/失效/方向保持状态；删除 generator、planner manager、topology core、joint optimizer 四处成员。短期偏好从可用 Guide lookahead 临时计算。没有 Guide 时保留无历史状态的 geometry descent seed。
- 删除 transition_pool_ 及 transitionPool()，不再复制/维护第二份候选容器。原返回候选、未完成合围的绕障 seed、SIDE/A* 与 fallback 仍存在。
- 删除 generator 直接朝最终 bearing 的 transition seed、无世界时间的 Target 吸引排序和未使用的 hypothesis target_cost。**当前工作树的 local 连续目标本来已是 Guide + radius/height bands，没有虚构并删除另一个同时生效的连续 bearing 项。** 本轮主要去掉生成/排序中的竞争引导，J_enc/J_vis/J_acc/K2/blackout 数学公式和权重不重设计。
- 删除 generator 因 Guide 不可用而修改 Target.active 的第二失效来源；Target 生命周期只由 worker 发布。Guide 监测不再覆盖旧证据的源 trajectory identity。
- 删除 joint 中以 encirclement_ratio 独立关闭 Guide 参考的条件；ratio 继续作为派生统计、相对收益及原滚动调度信息。未改变原评价窗口、调度节奏或新增瞬时开关/计时器。planner 的 geometry 日志状态改为当次派生值，不维护跨事件可变 state。
- 共享 BinaryCameraInterval（generator/topology/joint）、costImproves、coveragePreserved 以及原 shared geometry/Guide cost；local 选择和 joint 收益替换仍为不同入口，未关掉收益检查。
- launch 参数 recovery_intent_weight 改名 guide_direction_weight，默认0.02不变；已扫描生产调用者并同步接线。worker period 改为监测/失败重试语义，不是每周期三机求解。

相对本轮保存的真实工作树，20个已有文件变化，生产文件合计净减少31行；测试净增，另加三层说明文档。缩减主要是删除可变状态、第二候选容器及旧 Guide 重复求解，未靠压缩排版或删测试计数。

完整差异：[changes.diff](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/changes.diff)；[文件清单](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/changed_files.json)；[三层文档](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/RECOVERY_THREE_LAYERS.md)。

## 3. 看似重复但保留的内容

生产 worker/进程名和 probeRecoveryTarget/reanchorRecoveryProbe 接口保留，避免把昂贵求解搬入 local。reanchor 仍用于**新生成路径**经过求解耗时后的共同 future activation 对齐；这是首次团队验证，不能和旧软参考的反复证明混为一谈。原工作树已经把 fresh planning 与 Guide 生成放在同一入口，本轮没有声称删掉三个实际不存在的串行求解器。

TeamRecoveryTarget.msg 的 planner_reachable、reachability_generation、validation/activation/source identity、MINCOTraj evidence 保留为单一发布链的兼容/审计字段；reachability_generation 是 Guide 版本，plan_id 是 Target 身份。没有第二个可达性发布者、证书 worker 或 Target 管理器。部分 intent_cost 字段名保留兼容日志和已有比较接口，其值来自当次 Guide，无 intent 生命周期。

保留实际 Trajectory 全部 dynamics/static/dynamic/swarm/Local-SFC、actual-active→new P/V/A、前驱安全前缀、identity/generation/activation、coverage/deadline 检查。保留 joint P/T/yaw 和 optimized yaw execution。brake生产函数、FSM、traj_server、advanced_param.xml 和 run_in_sim.launch 与本轮备份一致：[逐函数/文件核对](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/unchanged_safety_contracts.json)。安全距离、速度/加速度/jerk/yaw限制、可行性和P/V/A容差、activation margin、brake及SIDE/A*策略均未改。

## 4. 构建及行为测试

指定命令 `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws` 通过，6/6包成功。最终20项入口全exit 0（19组unit/gradient/contract及参数装载）；原suite保留。旧 Intent 身份测试替换为同快照确定性、几何变化立即更新的等价 seed 行为测试，未以删除检查规避失败。

| 要求 | 验证 |
|---|---|
| 旧 Guide 失效保留 Target，同 Target 新 generation | 实际 worker + production capture/preflight 测试通过 |
| 无 Guide、空池、worker/joint 失败不阻塞 | 原local/commit/executor契约、空池和原seed接线测试；运行期无Target/Guide仍持续activation |
| 合法终点但路径失败不发布新成功 | 生产障碍墙/native规划失败测试 |
| 单机分别可行、三机组合冲突拒绝 | 原生产team preflight/swarm契约 |
| Guide可先远离最终bearing | detour seed与真实local Guide代价测试 |
| 无额外最终bearing竞争牵引 | 改动final bearing不改变同Guide代价/梯度测试，generator无直接bearing transition |
| 有效Guide不重复昂贵规划 | 实际worker probe_count不增，源身份/epoch/validation/generation不变；源码接线测试 |
| 陈旧Guide不是安全凭证 | freshness/exhaustion、无Guide订阅及原执行安全验证测试 |
| 无Intent/Transition管理层仍有候选 | stateless geometry seed、原过渡seed、generator和NOMINAL/SIDE/fallback生产测试 |
| pending/PVA/deadline/brake/coverage/stale身份 | 原suite全部通过；增加同Target旧Guide generation拒绝测试 |

production local Guide位置/移动目标时间导数有限差分最大误差 **1.3789e-11**；完整 production joint MINCO P/virtual-T objective最大误差 **1.36189e-11**。新增时间差分首轮因Unix epoch的1e-6名义步长舍入而失败；改用实际可表示时间增量，误差门限仍1e-6，生产公式未改，[首轮失败日志](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/finite_difference_first_attempt.log)保留。

-O2/-O3维持，未启用fast-math。运行时exe和.so hash与本次build一致，运行后源码/二进制未变：[构建日志](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/build_test_fix.log)、[测试结果](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/tests.json)、[优化flags](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/optimization_flags.json)、[运行版本核验](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/final_verification.json)。

## 5. 完整 Scenario A：同口径描述性比较

场景 `long_cylinder_forest_visibility_stress.json` SHA256 `430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`。FULL ON + native RViz唯一一次，路线80.204541s，到终点(36,0)、target速度归零后结束；stop_reason=target_replay_normally_completed。没有OFF/A-B，没有第二次完整复验。

baseline取 **target_guide_lifecycle_20260911 最近真实运行**，不是feedback_35；原始数据存在，其最近解耦6个留档文件hash均与本轮修改前快照一致。两次统计均从首个共同有效记录到各自route结束：baseline [1789099205.103293896, 1789099285.295543671]，本轮 [1789103362.473322868, 1789103442.617415667]。分母使用各自完整有效窗口。Guide可用期另外截断到原epoch+duration及原validation+3s；旧数据按相同可用性口径离线复核，未写入旧运行目录。

Target active寿命按发布消息边界积分；Target/Guide进展来自实际odom，不用优化器预测值替代：Target cost下降>0.02，Guide arc进展>0.01；bearing error用实际目标相对位置。Guide generation数量包括原版本复验产生的版本：旧43个中39个为fresh生成，其余4个是旧Guide重锚复验；本轮窗口17个均fresh生成。初始跨窗口Target/Guide已计入。

| 指标 | 最近真实运行 | 本轮 |
|---|---:|---:|
| 统计时长 s | 80.192250 | 80.144093 |
| 真实合围 25°/170° | 54.37% | 62.11% |
| 同半平面比例 | 26.64% | 19.77% |
| g_min P50/P95/min/max ° | 55.171036 / 93.395421 / 0.111402 / 113.806042 | 60.973379 / 105.722973 / 0.489295 / 118.092051 |
| g_max P50/P95/min/max ° | 168.445036 / 224.976751 / 126.523416 / 314.718304 | 166.527621 / 207.920277 / 121.584636 / 314.452417 |
| camera-time camera·s | 230.496724 | 228.563230 |
| mean_visible | 2.874302 | 2.851904 |
| K2 | 0.995846 | 0.988451 |
| All3 | 0.878456 | 0.863453 |
| None | 0.000000 | 0.000000 |
| 最长 K2 loss / blackout s | 0.199396 / 0.000000 | 0.488839 / 0.000000 |
| 实际 active Target 数（含跨窗口初始 Target） | 15 | 14 |
| Target active 寿命 P50/P95/max s | 1.556824 / 3.343883 / 5.130625 | 1.545176 / 3.435134 / 3.511468 |
| 每 Target 的 Guide generation P50/P95/max/mean | 3.000000 / 5.000000 / 5.000000 / 2.866667 | 1.000000 / 2.000000 / 2.000000 / 1.214286 |
| Guide generation 数 | 43 | 17 |
| Guide 可用寿命 P50/P95/max s | 0.516036 / 0.518934 / 0.521468 | 1.041157 / 2.471823 / 2.888230 |
| 同 Target fresh Guide 成功 / 尝试 | 24 / 37 (64.86%) | 3 / 7 (42.86%) |
| 旧 Guide 失效次数 | 27 | 4 |
| 实际 Target bearing cost 进展率 | 7/15 (46.67%) | 4/14 (28.57%) |
| 实际 Guide arc 进展率 | 81.40% | 76.47% |
| 实际 bearing error P50/P95/max ° | 8.120048 / 85.144744 / 152.866557 | 11.068043 / 97.449388 / 168.779974 |
| 实际 activation UAV0/1/2 Hz | 4.140051 / 4.202401 / 4.264751 | 4.179971 / 4.254836 / 4.067673 |
| 排除 brake 的 activation UAV0/1/2 Hz | 3.903120 / 3.666190 / 3.928060 | 3.755735 / 3.805645 / 3.755735 |
| terminal hold / 轨迹提前耗尽 | 0 / 0 | 0 / 0 |
| collision / swarm violation / 未验证执行样本 | 0 / 0 / 0 | 0 / 0 / 0 |
| planner / command / odom 速度峰值 m/s | 3.110336 / 3.110318 / 3.296802 | 3.139405 / 3.093981 / 3.337094 |
| brake 实际 activation 次数 | 89 | 95 |
| brake 执行时长占三机总执行时间 | 4.64% | 5.04% |
| 连续 brake 段 P50/P95/max s | 0.120353 / 0.480171 / 0.960161 | 0.129984 / 0.509538 / 0.971251 |
| 低速段 P50/P95/max s（odom <0.2 m/s） | 0.260161 / 0.526615 / 0.566726 | 0.152890 / 0.451621 / 0.706663 |
| worker fresh planning 次数 | 74 | 33 |
| worker fresh planning wall P50/P95/max ms | 4.090428 / 24.274540 / 117.814779 | 10.029078 / 154.658937 / 374.141932 |
| worker fresh planning mean wall / CPU ms | 8.291032 / 9.847986 | 33.160000 / 18.049152 |
| worker 进程平均 CPU %（单核口径） | 5.532601 | 5.407794 |
| local 规划延迟 P50/P95/max ms | 4.082000 / 23.028050 / 515.923000 | 7.393000 / 54.047700 / 832.085000 |
| REFRESH_BEFORE_SOLVE | 33 | 29 |
| OBJECTIVE_EVALUATION_CANCELLED | 1 | 2 |
| OPTIONAL_REFINEMENT_STOP | 2 | 4 |

本轮fresh Guide planning 33次，单次wall更长；合计wall约1.094s，对比旧约0.614s，不能把调用数减少等同于所有计算耗时改善。原Guide重复重锚求解已从生产链移除；28次GUIDE_REUSED只作轻量监测和重发。全原始记录中18个Guide版本（含统计窗口外尾部版本）的28次重复发布，epoch/validation/source/path证据均无同版本篡改。

窗口内4次Guide失效：2次耗尽/过期、2次地图阻断，全部保留Target；不再发生旧reference重锚动力学失败。fresh同Target失败仍有2次DYNAMICS_FAIL、2次MINCO_SCP_FAILURE，由真实新路径校验拒绝；不能误读为动力学检查被删。3次成功重建对应Target 6、11、14，均沿用原plan_id。没有达到持续失败换Target门槛的运行样本，该分支由生产测试覆盖。

54组joint三机proposal/ACK/commit/adoption/optimized-yaw执行链核验成立，其中30组接受时ratio<0.8，恢复失败未成为local等待条件。native RViz进程和配置确实启用；截图OpenGL区仍黑色，因此不声称完成完整视觉确认，数值结论来自完整轨迹/command/odom。

主要明细：[同口径比较JSON](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/analysis/comparison.json)、[Target/Guide/Brake审计](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/analysis/decoupling_brake_audit.json)、[实际执行曲线](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/analysis/encirclement_execution.png)、[版本与brake时间线](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/analysis/target_guide_brake_timeline.png)、[三机执行身份链](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/analysis/verified_chains.json)。

## 6. 非阻塞、候选和安全

本轮可用Guide约22.181935s，Target存在但Guide不可用约3.196466s，无Target约54.765692s。无Guide阶段实际activation为2.503/2.503/3.128Hz；无Target阶段4.017/3.871/3.743Hz。没有观察到terminal hold或提前耗尽。保留原NOMINAL/SIDE±、可行fallback和persistence候选；恢复引用不参与实际commit安全许可，joint无收益/失败只保留local。

collision/swarm violation/未验证执行均为0样本；这不等同于名义3m/s限制已达标。速度峰值如表，仍超过名义值，且本轮planner/odom峰值高于baseline，未通过修改限制或容差消除告警。

## 7. Brake变化、限制与留档

Brake实际activation **89→95**，执行时长占比 **4.638%→5.038%**，最长低速段 **0.567→0.707s**。生产brake策略未改，不能声称本轮减少brake或给出单一因果解释。合围改善伴随camera-time约下降0.84%、K2/All3下降、最长K2-loss增加、Target bearing进展率下降和local延迟升高；结构目标与原安全执行契约保持，但“整体行为无明显退化/全面性能改善”不能据此宣告成立。本轮未扩大到brake、可见性或名义超速重设计。

修改前diff/hash、完整源码快照与baseline测试在 `/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/baseline/`。完整PolyTraj、PositionCommand、Odometry、Target/Guide、activation/source、ROS日志和资源采样位于 `/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/run/`，运行数据约339MB；[原始产物hash](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/run_hashes.json)可复核。分析脚本、修正后的分析入口和全部结果均留在证据目录，空candidate_ids作为空候选解析，未丢掉失败事件。

本轮只使用一个agent；未reset或覆盖用户历史修改；未访问/修改禁止目录。本轮ROS/RViz/recorder/test master全部结束，12583/12584端口关闭，未处理其他用户进程：[清理及源码核对](/home/bob/ALP/egov2_fc65423_constvel/recovery_three_layers_20260911/cleanup_and_source_check.json)。
