**ALP visibility-first adaptive encirclement 实现与验证报告，2026-09-10**

生产源码、构建和测试已完成；唯一一次 Scenario A ON 短测未通过“新的 adaptive viewpoint 经 joint commit 后实际执行”的完整验收。新 adaptive hypothesis 已经过候选生成、安全过滤、team selector、joint P/T/yaw 优化和 binary acceptance，但因 activation reserve 不足停在 proposal 之前。已有保留 seed 的 joint solution 完成了 proposal、三机 ACK、commit、三机 adoption 和 optimized yaw 实际执行。两者不能合并算成新链成功。

本轮没有追加仿真。短测中没有记录到碰撞或急停，但累计执行可见性仍不理想，不能宣称已解决 Scenario A 的可见性下降，也不能把一次短测当作安全问题全部关闭的证据。

**真实结构与背景核对。** 项目为 `/home/bob/ALP/egov2_fc65423_constvel`。保留工作区原有修改，以本轮开始时的文件快照计算差异。没有访问或修改禁止目录。核对了已有改造前源码审计及相关生产调用链，没有重新实现已有 MINCO/SCP/OSQP、SIDE/A*、Local-SFC、joint P/T/yaw 或提交协议。

改造后实际数据流为：

```text
target prediction + UAV odometry/yaw + scene static/dynamic geometry
  → CooperativeViewpointManager::adaptiveTick
  → AdaptiveViewpointGenerator（fixed / uniform-120° / previous-accepted seeds）
  → EncirclementReferenceBundle（每机独立 reference；最多三个 hypothesis）
  → EgoReplanFSM：各 hypothesis 的 NOMINAL / SIDE± 候选
  → 现有 A* / Local-SFC / MINCO / SCP + 安全分类
  → TopologyCandidateBundle（同 generation/hypothesis，真实 binary trace）
  → TopologyCoordinatorCore：hard safety → coverage protection → camera-time
  → TeamVisibilityOptimizer：joint {P_i, tau_i, Psi_i}
  → nonlinear safety + binary camera-time acceptance
  → proposal → 3 ACK → commit → 3 adoption
  → PolyTraj scheduled activation → traj_server position / optimized yaw
  → actual-execution source + activation feedback + trajectory/visibility CSV
```

与用户背景需要区分的源码细节：原 selector 的 safety 层实际允许 `safety_class>0`，并存在“全部组合有冲突时择较小冲突”的路径；原 candidate 分类没有把已计算的 swarm validity 纳入全部已有检查；原 binary 无效时有 continuous≥0.5 的替代；原 nonlinear validator 的返回值存在没有完整作为最终成功门槛的路径。这些不是已闭合的 hard safety，本轮一并收紧。旧共同 phi0/hypothesis helper 和诊断字段仍在源码中，但启用 reference 的新运行路径经过 `adaptiveTick()`，不再由旧共同旋转逻辑驱动。

既有 Scenario A 审计还指出统计窗口差异：历史报告的 mean_visible 使用了去掉前 2 秒的口径，其 camera-time 与严格左端点积分并非完全相同。本轮固定使用全量实际记录、不删除 warm-up、相邻时间差左端点积分；不与历史三次均值直接作增减比较。更完整的改造前事实见 [既有源码审计](/home/bob/ALP/egov2_fc65423_constvel/ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md)。

**修改文件与接口。** 本轮修改 20 个现有文件、新增 4 个文件；完整相对路径在 [changed_files.json](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/changed_files.json)，逐文件补丁在 `diffs/`，改造前快照在 `baseline/`。下表中的 formation 路径均属于 `ros_ws/src/multi_uav_formation`，planner 路径属于 EGO-Planner-v2 的 tracking_ws/src/planner。

| 文件/接口 | 实际修改 |
|---|---|
| 新增 `adaptive_viewpoint_generator.h/.cpp` | `generate()`、`profile()`、`evaluateCached()`；统一 seeds、两尺度 binary 筛选、独立角度；`angularDiversityCost()` 及梯度 |
| `cooperative_viewpoint_manager.cpp` | `adaptiveTick()`、timer 接入；发布 references、记录角度；根据三机实际 activation 提升 previous accepted viewpoint |
| `cooperative_viewpoint_core.h` | 向 generator 暴露现有 scene geometry、dynamic obstacles 和 camera contract |
| `topology_coordinator_core.h/.cpp` | binary camera-time 和 loss 指标；`evaluate()` / `select()` / `better()` 的安全、coverage、camera-time 排序 |
| `team_visibility_optimizer.h/.cpp` | binary metrics、`cameraTimeAcceptance()`、非线性重评估、P/tau diversity 梯度、最终安全门槛 |
| `multi_uav_topology_coordinator.cpp` | 读取/限制 visibility 权重、严格 binary callback、coverage 参数、before/after 日志 |
| `native_egov2_rviz.launch` | adaptive horizon/top-K/bearing、camera-time eps/saturation、diversity 参数接线 |
| `poly_traj_optimizer.h/.cpp` | encirclement tracking 改为 observation radius/height bands，移除启用分支的固定 slot angular restoring |
| `ego_replan_fsm.cpp` | current hypothesis 失败时仍保留安全 alternative 候选；执行消息加 source；修正 emergency failure 返回/发布 |
| `planner_manager.h/.cpp` | `validateExecutionTrajectory()`、`validateRetimedLocalSfc()`、`setLocalTrajFromOpt()`、`annotateExecutionSource()`；候选、retiming、fallback、ACK/commit 检查 |
| `TopologyCandidate.msg` | 新增 `binary_visibility_valid`，与 trace 一起传递 |
| `PolyTraj.msg` | 执行 source、generation、team solution、viewpoint identity、safety validation 元数据 |
| `traj_server.cpp` | 验证消息、随 scheduled trajectory 切换 source；在实际激活时发布 source 和 activation feedback |
| `native_egov2_rviz_scene.py` | 订阅实际执行来源，将身份/时间/validation 写入 trajectory CSV |
| 新增 `adaptive_viewpoint_generator_contract_test.cpp` | 墙场景、自由角度、diversity 梯度、候选可达、selector/acceptance 和记录状态回放 |
| 新增 `adaptive_execution_contract_test.py` | 9 项安全入口、fallback、source、generation 和 launch 结构契约 |
| `topology_coordinator_contract_test.cpp`、`team_visibility_optimizer_contract_test.cpp` | 更新 binary/safety 输入契约并扩展验证 |
| formation `CMakeLists.txt` | 新库和测试目标、CTest 注册；短测后修正运行目标未优化编译问题 |

**Adaptive viewpoint 逻辑与公式。** 每机 viewpoint 为目标相对偏移：

```text
q_i = (r_i cos(theta_i), r_i sin(theta_i), h_i)
p_ref_i(t) = target(t) + q_i
```

generator 从 fixed offsets、一次性初始化的 uniform-120°、previous actually accepted solution 出发，分别进行两轮坐标搜索。每轮依次优化三机 bearing；默认 24 个角度采样，每角度评价基准、半径±0.35 m、高度±0.20 m 五种选择。半径搜索有 0.3 m 下限。按预测 camera-time、near camera-time、diversity、transition cost 排序，去重后最多输出三个 team hypotheses。缓存只在同一次状态快照内复用各 UAV 的 visibility profile，避免复用过期目标/LOS/yaw。

筛选使用当前位置、速度、yaw/rate 和恒速目标预测；以五次 blend 估计从当前运动状态到 viewpoint 的过渡，并用现有 target-facing yaw predictor 估计 FOV。该过渡只用于便宜的 viewpoint 评分，不充当可执行轨迹或安全证明，实际轨迹仍由现有 planner 生成和验证。

所有 binary visibility 使用共享的 `tracking_visibility_geometry.h` camera/range 与动态 cylinder LOS，以及现有 `StaticLosGeometry` 静态 LOS：

```text
V_i(t_k) = range_ok AND static_LOS_ok AND dynamic_LOS_ok AND camera_FOV_ok
T_cam = sum_k dt_k * sum_i V_i(t_k)
mean_visible = T_cam / H
```

固定 dt 时就是用户要求的 ΔtΣΣV；尾段按真实剩余 dt 积分，不多算 horizon 终点。invalid geometry/prediction/trace 不再转为 continuous threshold 的“binary”。实际 candidate trace 和 joint nonlinear re-evaluation 均使用自己的可执行 position/yaw，不能拿 generator 的便宜预测代替最终验收。

采用两尺度：near horizon=1.5 s、dt=0.10 s；far horizon=3.5 s、dt=0.25 s，far 仅用于 generator 筛选，不扩大 joint optimization 或 collision 验证窗口。此次 target speed=0.928 m/s，near/far 对应约 1.39/3.25 m 的目标前进距离，far 提供额外约 1.86 m 的遮挡预警。它是当前场景的初始参数，并未通过多轮仿真调优。

生成的新 trial 要满足最低 bearing separation；joint 中加入同一 lower-bound 代价：

```text
J_div = sum_{i<j} max(0, theta_min - abs(wrap(theta_i-theta_j)))^2
theta_min = 25°
```

超过下限时该 pair 的代价和梯度均为零，没有继续向 120°拉近的奖励。generator 对输出搜索结果筛选下限；joint 中是 soft penalty，不宣称实际运动全过程都严格满足 25°。

**120°现在的实际作用。** uniform 120°只生成初始化 seed，并作为角度偏离日志的基准；fixed offsets 和 previous accepted offsets 也只是 generator seeds/保留候选。同分时允许保留当前 hypothesis，因而在无遮挡环境可能继续采用 120°，但没有固定 slot 的持续 angular restoring。没有增加 fixed/120/adaptive 三套长期 planner。

启用 encirclement 的 local tracking 沿轨迹仅使用：

```text
e_r = sign(r-r_ref) * max(0, abs(r-r_ref)-radius_band)
e_h = sign(h-h_ref) * max(0, abs(h-h_ref)-height_band)
J_track = w_track * (e_r^2 + e_h^2)
```

MINCO 的初始化和 terminal goal 仍指向 visibility-selected viewpoint；若最后选中的候选恰好来自 120° seed，endpoint 也可能在该位置。这不构成显式的 120°角度恢复代价。旧通用/非 adaptive helper 保留兼容性，不把“源码仍有 120°字符串”误报为新路径仍施加恢复力。本轮没有加入 `J_seed=lambda*delta²`。

**累计可见性、coverage、selector 和 acceptance。** continuous objective 保留可导形式：

```text
J_acc = mean(1 - (v0+v1+v2)/3)
q2 = v0*v1 + v0*v2 + v1*v2 - 2*v0*v1*v2
b_k2 = 1-q2
b_blackout = (1-v0)*(1-v1)*(1-v2)
```

此次 ON 的权重为 acc=4.0、K2=0.20、K2-cont=0.20、blackout=0.05、diversity=0.15、deviation=1、jerk=1e-5、yaw-prior=0.02。运行节点将 acc 至少设为 4，并把三项竞争 visibility 权重之和限制在 acc 的 25% 以内，避免旧 launcher 权重重新让 coverage soft terms 主导。加权本身不证明任意状态下梯度绝对主导；真正保证 camera-time 优先的是最终二值排序和不可绕过的 acceptance gate。

selector 的新语义是：

1. 仅允许完整 executable、ABSOLUTE_SAFE（class=2）且团队没有 swarm 冲突的组合。
2. 以当前 hard-safe local combination 为 coverage 参考；它不存在时，以 hard-safe best-K2 recovery combination 为参考。相对参考要求 binary K2 降幅≤0.02、最长 K2 loss 增加≤0.20 s、最长 blackout 增加≤0.20 s。这是相对保护，不是绝对保证 K2 接近 1。
3. 在通过保护的集合内最大化同 horizon 的 binary T_cam；随后 All3、diversity、余量、temporal burden、tracking/native cost 等 tie-break。固定 horizon 下 mean_visible 与 T_cam 数学等价，不另设矛盾的排序。
4. history hysteresis 仅能在 camera-time 差≤1e-6 camera·s 且 All3 同分时保留，不能盖过更高 camera-time。

candidate 缺失入口同步调整：current seed 规划失败不会提前丢弃安全 adaptive alternatives；原 hypothesis id 0/1/2 继续作为 bundle 内身份，phi0 兼容字段置零。仍强制匹配真实 hypothesis/generation，不混合不同 team hypothesis 的单机候选，不绕过安全、freshness、activation 或 version 检查。

joint 使用 continuous v_i 的 P/tau/yaw 梯度；每个 nonlinear evaluation 重算真实 binary trace。mandatory `cameraTimeAcceptance()` 不受旧可选 guard 开关绕过：

```text
binary invalid / horizon inconsistent / nonlinear safety failed -> reject
coverage protection failed -> reject
T_after < T_before - eps_regression -> reject
T_after > T_before + eps_improve -> pass camera-time gate
否则仅当 T_before >= saturation_fraction * 3H
  且 measurableBenefit 成立，才允许饱和平台上的 tie-break
```

默认 `eps_improve=eps_regression=1e-6 camera·s`、saturation_fraction=0.98；均参数化。非饱和 binary 平台上只改善 objective 会拒绝。饱和时允许 eps 范围内的数值误差，再由连续指标/目标改进决定；不允许明显 camera-time 回退被 K2-cont、tracking 或 total objective 改善覆盖。SCP 中间更新也检查相对于优化前 baseline 的 camera-time/coverage，不仅在最后检查。

**安全 telemetry 与已明确修复的入口。** 每次 `setLocalTrajFromOpt()` 提交统一经过 dynamics、完整 static、完整 dynamic、swarm 验证。joint 使用同一 activation 的团队 payload 做 peer 检查；retiming 之后重查 Local-SFC。candidate 的 ABSOLUTE_SAFE 分类纳入 swarm validity，并采用 candidate 对应的预测 epoch。

明确封堵了两条 non-absolute candidate 在 previous-safe 不可用时继续落入 commit 的路径。STATIC_COLLISION、dynamic invalid 和 retiming/dynamics fail 不能借 fallback 再被提交。persistence 检查当前实际 remaining suffix；missing dynamic prediction 失效，无动态障碍则作为有效无障碍处理。emergency trajectory 生成/验证失败不再返回成功或重发旧 trajectory。COMMIT 的 activation、generation、source IDs、position/MINCO、yaw payload 必须与 ACK 时一致。

未降低 static/dynamic/swarm clearance、v/a/jerk、yaw limits、Local-SFC 或 generation 校验。这里“完整”指对整条相关 polynomial 使用已有 nonlinear/sampling 检查，不是形式化的连续时间碰撞证明。

`PolyTraj` source 分类为 NOMINAL、SIDE_PLUS、SIDE_MINUS、FEASIBLE_FALLBACK、PERSISTENCE_FALLBACK、JOINT_TEAM_SOLUTION、OTHER；携带 trajectory_id、generation、team_solution_id、encirclement generation、hypothesis、start/activation timestamp 和 safety_validated。traj_server 拒绝未验证 payload，元数据随 scheduled trajectory 一起切换，实际激活时发布 `/trajectory_execution/source` 和 `/trajectory_execution/activated`。CSV 保存身份与实际时刻，previous accepted viewpoint 只由三机 matching actual activation 更新。

短测覆盖了 NOMINAL、JOINT_TEAM_SOLUTION、PERSISTENCE_FALLBACK 实际执行；SIDE±/FEASIBLE_FALLBACK 的入口由源码契约和已有安全测试覆盖，此次没有这些 source 的实际执行样本。没有把过去所有 ON collision samples 唯一归因到本次修复的分支。

**构建与测试。** ROS Noetic 下执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/bob/ALP/egov2_fc65423_constvel/ros_ws
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status
```

相关 6 个包全部构建成功；最终构建 27.2 s。记录见 [最终 build](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/build_post_short.log)。12 项测试套件最终退出码均为零，见 [test_results.json](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/test_results.json)。其中 ROS 依赖测试先前因无 master 超时，在隔离的测试 master 上重跑通过，测试 master 已停止。

| 验证 | 结果 |
|---|---|
| 墙阻挡原 120° slot；允许显著非零 delta | PASS；bearing 偏离 45°，预测 camera-time 7→8.5 camera·s |
| diversity 到达下限后零代价/零梯度 | PASS |
| diversity 位置及 MINCO P/tau 有限差分 | PASS；MINCO 最大误差约 1.18e-11 |
| 当前 seed 无效时 adaptive 安全候选仍进入 selector | PASS |
| coverage 均满足时 camera-time 更高者胜出 | PASS |
| objective 改善但 camera-time 明显回退拒绝 | PASS |
| 非饱和平台拒绝、真实改善接受、饱和 tie、binary invalid、blackout protection | PASS |
| 现有 P/tau、piece boundary、K2、J_acc、continuity、blackout、yaw 优化 | PASS |
| joint 全 polynomial static safety 拒绝及 commit 身份契约 | PASS |
| adaptive/joint/fallback 安全入口、persistence suffix、source 原子切换 | 9 项 Python 结构契约 PASS；不等价于所有分支实飞/物理安全证明 |
| Static LOS wall、Elastic visibility、Local-SFC boundary、time-only feasibility/swarm temporal、tracking visibility | PASS |

短测后重新编译受影响目标并运行 CTest：[6/6 PASS](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/ctest_post_short.log)。详细梯度结果见 [joint tests](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/team_visibility_optimizer_contract_test.log)。

**唯一一次 Scenario A ON 短测。** 场景为 `long_cylinder_forest_visibility_stress.json`，SHA256=`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`。FULL_ON、target speed=0.928 m/s，完整参数在 [command.json](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/scenario_a_on/command.json)。执行到短测 mission deadline 停止；最后 trajectory 时刻 31.008315 s，可见性积分区间 30.96658 s，含关闭过程的 wall time 51.765 s。

launcher 在 SIGINT/tee 清理时退出码为 120；不能把它写成正常零退出，也不能据此否认已记录的实际运行。自动 monitor 的 `complete_chains=[]` 未识别被并发 stdout 截断的日志前缀，报告的已有链验证来自后处理日志加实际执行 CSV，见 [analysis.json](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/scenario_a_on/analysis.json) 和 [chain_evidence.log](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/scenario_a_on/chain_evidence.log)。

| 实际执行指标：全量记录，不裁 warm-up | 值 |
|---|---:|
| accumulated camera-time | 54.865593 camera·s |
| mean_visible | 1.771768 |
| K2 | 0.566160 |
| All3 | 0.205608 |
| None | 0 |
| longest K2 loss | 8.566866 s |
| longest blackout | 0 s |
| min swarm Euclidean / elliptical | 0.634934 / 0.634922 m |
| min static surface clearance | 0.204236 m |
| min dynamic surface clearance | 7.173274 m |
| collision samples / swarm clearance violating pair samples | 0 / 0 |
| emergency stops / unvalidated executed samples | 0 / 0 |
| trajectory / visibility samples | 2793 / 930 |
| source 样本：NOMINAL / JOINT_TEAM_SOLUTION / PERSISTENCE_FALLBACK | 1898 / 207 / 688 |

swarm 使用既有 vertical squared weight=0.25 的 ellipsoidal metric，当前下限 0.5 m。static/dynamic 表中为几何表面余量，不能直接当成 planner 的动态中心距离阈值。`SAFETY_REGRESSION_OBSERVED: NO` 只表示本次短测未观察到所记录的碰撞/clearance/未验证执行异常；本轮没有 OFF 对照。

共记录 `ADAPTIVE_VIEWPOINT_TRIGGERED` 19 次、安全非当前 viewpoint candidate 生成事件 85 次、非当前 adaptive hypothesis team selection 1 次、joint attempts 9 次、proposal 2 次、commit 1 次。`HIGH_VISIBILITY_CANDIDATE_GENERATED` 表示安全 alternative 到达候选生成入口，不表示全部 85 次都已相对当前 binary camera-time 获得严格增益。

| 新 adaptive 事件 | 记录 |
|---|---|
| viewpoint generation / hypothesis | 9 / 1 |
| VIEWPOINT_HYPOTHESIS_COUNT / generator far T_cam | 3 / 10.300000 camera·s |
| `FINAL_VIEWPOINT_ANGLES` | (14.539°, 45.000°, 135.461°) |
| `DEVIATION_FROM_120_DEG`（各机相对初始化 seed 的 wrapped 差） | (74.539°, −135.000°, 75.461°) |
| planner candidate IDs：UAV0/1/2 | 73 / 35 / 30 |
| selector / joint / nonlinear binary acceptance | 到达 / 到达 / 通过 |
| T_CAM_BINARY_BEFORE→AFTER | 4.500000→4.500000 camera·s |
| MEAN_VISIBLE_BEFORE→AFTER | 3→3 |
| K2、All3 BEFORE→AFTER | 均为 1→1 |
| LONGEST_K2_LOSS、LONGEST_BLACKOUT BEFORE→AFTER | 均为 0→0 s |
| P_CHANGE_NORM / T_CHANGE_NORM / YAW_CHANGE_NORM | 0.000101793 / 0.000095639 / 0.000795206 |
| proposal / 3 ACK / commit / adoption / actual joint execution | 未达到 |
| 拒绝原因 | ACTIVATION_RESERVE_EXHAUSTED |
| solve latency / 剩余 lead / 所需 lead | 167.348 ms / 0.070135 s / 0.085000 s |

这次 near horizon 原本已饱和，camera-time 不变按饱和 tie 接受，不能宣称该 joint 更新提升了 camera-time。日志中的 `FINAL_VIEWPOINT_ANGLES` 是生成器输出 hypothesis 的角度，也不能当成已执行角度。

成功执行的是保留 seed 的 hypothesis 0、team_solution_id=1、coordination generation=7：proposal→ACK0/1/2→commit→adoption0/1/2→actual execution0/1/2→optimized yaw0/1/2 全部有证据，各机 trajectory_id=5。该解 T_cam=4.5→4.5、mean_visible=3→3、K2/All3=1→1、最长 loss=0；P/T/yaw change 分别为 0.002411868、0.000848374、0.000479684。它验证既有 joint P/T/yaw 工程链保留，不验证新 adaptive 几何的完整提交执行链。

| 短测时 latency（ms） | mean | p95 | max |
|---|---:|---:|---:|
| viewpoint generator，18 条完整日志 | 861.370 | 1110.444 | 1129.436 |
| 每 hypothesis planner | 5.079 | 8.349 | 69.862 |
| joint solve | 174.099 | 197.225 | 197.653 |

短测共出现 7 次 activation reserve exhaustion，另一次 proposal 被 UAV2 以 `NO_PENDING_TOPOLOGY_GENERATION` 拒绝。这些 lifecycle/latency 问题仍限制新链的运行验证。

**短测后的修复与版本边界。** 检查发现 formation 的 CMAKE_BUILD_TYPE 为空，manager/coordinator 及其 Eigen inline 路径没有启用优化编译。最终 CMake 为运行目标和相关数值测试显式启用 `-O2`，没有 fast-math，也没有放宽 safety 或 activation 阈值。

使用同一 Scenario A 几何与本次记录 t≈14.9 s 的状态做一次确定性函数回放：generator=5.99735 ms，产生 2 个 hypotheses，测试全部通过，日志显式 `SIMULATION=0`。这只是脱离 ROS 调度负载的函数回放，不能与上表 wall latency 直接作同条件性能比较，也不能证明 activation reserve 和 generation 问题已经修复。没有第二次仿真。

最终源代码与短测源代码只在 formation `CMakeLists.txt` 和新增 generator contract test 的 recorded-snapshot 入口这两处不同。短测哈希在 `scenario_a_on/source_sha256.json`，最终哈希在 `final_source_sha256.json`；[final_verification.json](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/final_verification.json) 确认最终 hash 全匹配、run_count=1、剩余 ROS/仿真进程为空。回放证据见 [recorded_snapshot_test.log](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910/recorded_snapshot_test.log)。

尚未通过的关键验收项为新 adaptive hypothesis 的 joint commit 实际执行，以及由此带来的 executed camera-time 改善。当前交付应视为已实现并通过构建/测试、但运行验收不完整的版本。

以下标志的 YES 表示对应代码机制或列出的测试证据成立；不将其扩展为新链运行验收成功。camera-time regression 的 NO 指超过配置 eps 的明显回退；seed-only 指启用的新 adaptive 路径。

```text
PRODUCTION_SOURCE_CHANGED: YES

ADAPTIVE_VIEWPOINT_IMPLEMENTED: YES
120_DEGREE_USED_AS_SEED_ONLY: YES
120_DEGREE_STILL_USED_AS_RESTORING_TARGET: NO
ANGULAR_DIVERSITY_LOWER_BOUND_ACTIVE: YES

ACCUMULATED_VISIBILITY_PRIMARY_OBJECTIVE: YES
K2_BLACKOUT_PROTECTION_ACTIVE: YES
BINARY_CAMERA_TIME_SELECTOR_ACTIVE: YES
BINARY_CAMERA_TIME_ACCEPTANCE_ACTIVE: YES
CAMERA_TIME_REGRESSION_CAN_BE_ACCEPTED_BY_OTHER_OBJECTIVES: NO

GOOD_VISIBILITY_CANDIDATE_REACHES_TEAM_SELECTION: YES

JOINT_PT_YAW_PRESERVED: YES
TEAM_COMMIT_CHAIN_PRESERVED: YES
OPTIMIZED_YAW_EXECUTION_PRESERVED: YES
NEW_ADAPTIVE_JOINT_EXECUTION_CHAIN_VERIFIED: NO

SAFETY_EXECUTION_SOURCE_TRACEABLE: YES
SAFETY_REGRESSION_OBSERVED: NO

BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES

SCENARIO_A_ON_SHORT_RUN: YES
SIMULATION_LEFT_RUNNING: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

证据目录：[visibility_first_validation_20260910](/home/bob/ALP/egov2_fc65423_constvel/visibility_first_validation_20260910)。本报告中未展开的相对证据路径均相对于该目录。
