# 协同链路、重规划失败与“两前一后”：停止修改后的分析报告

已按用户“立刻停止修改，完成分析，书写报告”冻结源码；该指令后仅运行离线分析、版本核对并写本报告。最后一次 FULL ON + native RViz 仿真已经正常走完80.204541秒目标路线，没有追加仿真。

**结论：共同参考链路已从不能实际建立恢复为可持续使用，但问题没有全部解决。** 最终窗口共同参考覆盖53.20%，窗口内32次完整三机实际接管；仍有大量brake，合围比例52.12%，没有优于修改前的53.47%。不能宣称完成了“更稳定合围且不退化”的全部目标。

## 1. 哪个思想没有真正落实

### 团队持续规划与local兑现：此前协议输入生命周期不一致

修改前完整运行仅1个proposal、0个commit。唯一proposal的两机拒绝原因为 `NO_PENDING_TOPOLOGY_GENERATION`。coordinator在发出上一轮终结结果后仍保留三机bundle；一架更新就可能把另两架已消费的pending身份再次拿去求解，而planner已正确释放这些身份。

现在在已有 `consumeCurrentSnapshot()` 中消费对应bundle，延迟重复消息不能覆盖新快照。没有新增协议、状态机或参考发布者，也没有让local等待三机。新增ROS行为测试调用真实coordinator：保存的修改前源码重新编译后必现“旧peer身份被复用”，当前版本通过同一测试。证据：`identity_red_green.json`、`baseline_identity_repro.log`、`fixed_identity_repro.log`。最终运行的 `NO_PENDING_TOPOLOGY_GENERATION` proposal拒绝为0；仍保留真实陈旧generation和P/V/A不匹配拒绝。

### 有deadline，但没有覆盖完整joint求解

原deadline仅在SCP外循环检查；初始objective、约束采样、底层QP和最终全路径验证未完整受同一预算约束。现在按一次调用的steady-clock预算取消初始评估、约束和全路径检查，向已有OSQP调用传入剩余时间；取消后的部分验证不能被视为成功。沿用原timeout参数，没有改变动力学/安全阈值。

修改前coordinator在任务约7秒的第4次joint attempt后不再返回日志。三个短时诊断未复现该长停滞，抓到的是正常等待callback的堆栈；关闭阶段boost mutex异常不能当作运行期根因。**该次长停滞究竟卡在哪个底层调用仍未证实；不能把deadline补全称为已证明的唯一根因。** 两次完整修复后运行均持续处理输入，最终记录64次joint尝试，耗时均值4.03ms、P95 8.68ms、最大17.82ms。超时单元行为由生产求解测试验证，而非靠本次未发生超时来证明。

### 共同时间参考的初始化曾自行改变轨迹

原 `TeamTrajectoryReference::seed()` 将原曲线按等长时间重新采位置，再生成MINCO。即使head与参考完全相同，非均匀分段轨迹的剩余曲线仍发生变化；生产seed复现最大位置误差0.0336086m。这不是local主动避险，而是初始化误差。

现保留原junction时间、原世界epoch和有效尾端，仅用真实active/scheduled predecessor的P/V/A替换head。必要时只细分原段，不移动原junction；若joint固定维数容不下原junction，则放弃这个可选warm start，使用原fresh local初始化。生产回归的整个剩余区间最大P/V/A误差7.69088e-13；不同fresh head也正确保留。没有新增follow目标、修改权重、延长旧参考有效期或把参考作为执行许可。

这证明修掉了可复现的初始化误差，**不证明它解释了全部动力学失败或brake增加**。

## 2. 最终运行与前两次完整运行

最终统计窗口为 `1789143467.103907108 → 1789143547.245131731`，时长80.141225秒，使用共同记录交集起点到目标路线结束；之前两次采用同一规则，分别80.154617秒、80.170229秒。是单次运行的描述性比较，非A/B或因果估计。

| 指标 | 修改前，feedback_39 | 身份/deadline修复后 | 最终，再保留原分段 |
|---|---:|---:|---:|
| 共同参考覆盖 | 0% | 59.45% | 53.20% |
| 窗口内 proposal / commit | 1 / 0 | 35 / 26 | 40 / 33 |
| 窗口内完整三机实际接管 | 0 | 26 | 32 |
| 实际合围 / 同半平面 | 53.47% / 25.27% | 51.14% / 38.15% | 52.12% / 29.54% |
| camera-time（camera·s） | 230.397 | 226.839 | 226.974 |
| mean_visible | 2.8744 | 2.8295 | 2.8322 |
| K2 / All3 / None | 99.67% / 87.77% / 0% | 97.34% / 85.61% / 0% | 99.33% / 83.89% / 0% |
| brake实际激活次数 | 114 | 468 | 466 |
| brake占三机总执行时间 | 5.99% | 23.83% | 24.27% |
| terminal hold / 提前耗尽 | 0 / 0 | 0 / 0 | 0 / 0 |
| 碰撞 / swarm violation / 未验证执行采样 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| planner / command / odom速度峰值（m/s） | 3.1481 / 3.1481 / 3.3037 | 3.1495 / 3.1495 / 3.4403 | 3.1433 / 3.1432 / 3.4432 |

最终窗口33次commit、99条planner adoption日志，与32次完整三机**实际激活**不是同一指标。team 40在窗口结束附近跨边界激活：窗口内只记录一架，完整记录随后三架齐全；因此整个录制有33次三机激活，窗口内只有32次。路线结束后新发的team 41部分激活也未算入主窗口。原始消息总共记录34个commit版本，不应直接作为窗口成功次数。见 `analysis/team_activation_window_audit.json`。

其他最终指标：

- g_min：P50 65.079°、P95 98.336°、最小 0.339°；g_max：P50 168.760°、P95 313.760°、最大 326.510°。
- 最长K2 loss 0.267708s，blackout 0s。
- 三机实际activation率：5.403, 5.141, 5.266 Hz；排除brake：3.681, 3.494, 2.820 Hz。
- 接管P/V/A残差最大值：0.000095727m / 0.002705291m/s / 0.049626934m/s²；保留原容差。
- 采样最小static / dynamic clearance：0.125892 / 0.775763m；最小swarm椭球距离 0.822536m。零违规采样不等于对采样之间的连续运动另作数学证明。
- 最长低速段（原分析口径，odom速度<0.2m/s）：3.999640s；brake策略未改。名义3m/s超速仍在。

## 3. 共同参考是否真正被兑现

最终三机同时使用同一未过期reference id共42.633018s，占53.20%，按原world epoch采样，不用JOINT source占比替代。

但这只是共同参考可用并被实际轨迹关联的时间，并非全部严格跟随：其中至少一架在brake上的时间21.122334s；三机都非brake的共同参考时间21.510684s（全窗口约26.84%）。所以不能把53.20%说成三机持续无偏离兑现。

| UAV | 参考有效且关联执行(s) | 原JOINT执行(s) | local rolling修正(s) | 绕障偏离(s) | brake偏离(s) | odom-reference均值/P95/最大(m) |
|---|---:|---:|---:|---:|---:|---|
| 0 | 42.789 | 7.127 | 23.346 | 0.125 | 12.191 | 0.133 / 0.440 / 0.849 |
| 1 | 42.796 | 8.808 | 22.635 | 0.000 | 11.353 | 0.109 / 0.370 / 0.728 |
| 2 | 42.823 | 10.730 | 17.554 | 0.442 | 14.098 | 0.114 / 0.344 / 1.329 |

三机local-reference均值误差为0.114, 0.080, 0.084m；odom-command均值为0.059, 0.058, 0.052m。MINCO离线重建与实际JOINT polynomial的最大位置差2.75844e-7m，支持三层曲线对齐的时间/表示一致性。

## 4. “两前一后”发生在哪一层

分析定义：以目标速度方向上的投影正负区分前后；“前方两机靠近”使用已有bearing差<25°，这是诊断定义，不是新生产阈值。“两前一后”本身不等于违反25°/170°合围。

全窗口实际“两前一后”54.171s，local曲线55.032s；其中“前方两机靠近且一机在后”，actual 1.304s、local 1.340s。actual的1.272s已存在于local，仅0.032s首次出现在odom层。

在共同参考有效且版本相同的窗口内：

| 布局 | team计划 | local提交 | actual odom |
|---|---:|---:|---:|
| 两前一后 | 32.406s | 33.254s | 32.520s |
| 前方两机bearing<25°且一机在后 | 0.870s | 0.940s | 0.938s |

其中actual靠近布局0.836s已被team计划，0.071s首次在local出现，0.032s首次在odom出现。因此最终运行的剩余布局主要在team规划层已存在，不能归结为local覆盖或控制器跟不上。当前J_enc/visibility/收益机制允许恢复过渡中的这种布局；共同执行一致性不会自动使允许的布局消失。本轮未增加120°奖励、舒适区或改写几何公式来掩盖。

共同时间序列与图：

- [team_liveness_fix_20260911_235337/analysis/team_local_actual_world_time.csv](/home/bob/ALP/egov2_fc65423_constvel/team_liveness_fix_20260911_235337/analysis/team_local_actual_world_time.csv)
- [team/local/actual gaps与跟踪误差图](/home/bob/ALP/egov2_fc65423_constvel/team_liveness_fix_20260911_235337/analysis/team_local_actual.png)
- [版本对齐统计](/home/bob/ALP/egov2_fc65423_constvel/team_liveness_fix_20260911_235337/analysis/team_reference_alignment.json)

## 5. “重规划经常失败”具体是什么

最终1,351个最终候选preflight记录：906个安全，386个 `DYNAMICS_FAIL`、32个static、26个dynamic、1个swarm拒绝。安全候选率67.06%，修改前1,726个中1,426个安全（82.62%）。不同候选数量/分支不能直接当成功飞行概率；但动力学失败仍明显存在，不能说已修好。

joint方面最终记录：46个 `CURRENT_HYPOTHESIS_INSUFFICIENT_COVERAGE` 终结结果、2个stale bundle、2个无可执行组合。proposal ACK拒绝包括7条P/V/A/source-tail边界不匹配、2条真正的stale generation；`NO_PENDING_TOPOLOGY_GENERATION` 为0。这些真实边界检查未放宽。详见 `analysis/failure_reasons.json`。

旧分析器的 `unmatched` 分类不是team-follow的权威标记，不能把其中所有失败直接称为共同参考导致。最初长停滞未锁定底层调用；brake增加的全部原因也尚未完成因果诊断。用户要求停止修改后，没有继续实现新的seed退让、fallback或brake改动。

## 6. 构建、测试及保留能力

最终指定 `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws` 6/6包成功，原O2/O3保留，无fast-math；仅原 `base_delta_phi` unused-variable warning。全部21个测试入口通过，包括原unit/gradient/contract/executor suites、生产full-objective有限差分，以及新ROS身份消费、初始/全路径deadline、精确timed suffix与fresh head测试。没有新增连续目标或修改其导数。

本轮生产改动集中于coordinator、team optimizer的deadline接线和team reference seed。原planner_manager.cpp、ego_replan_fsm.cpp、poly_traj_optimizer.cpp、traj_server.cpp、encirclement_recovery.h源码hash均与修改前一致，见 `unchanged_execution_sources.json`。local非阻塞、原NOMINAL/SIDE/A*候选、scheduled predecessor、P/V/A、generation、execution coverage、joint yaw及三机ACK/commit安全链保留。没有删除消息、新增协议、改brake策略或改安全/可行性阈值。

过程失败也保留：第一次测试复制漏了参数fixture，参数加载和随后的production fixture测试失败，随后补齐fixture并让runner遇失败中止，最终21项全通过；最后重建曾磁盘空间不足，保留失败构建日志。只对本轮产生的47个大日志/原始数据文件做gzip无损压缩，并逐文件验证解压SHA256一致，未删除历史实验数据。最终构建/测试通过后才启动最后一次完整复验。

## 7. 数据、版本与停止状态

- 修改前完整A：`scenario_a_20260911_234722/`，已独占记录feedback_39。
- 3次未修改源码的短时诊断：`team_liveness_fix_20260911_235337/diagnostic_1/`、`diagnostic_2/`、`diagnostic_3/`，分别约26/46/56秒目标运行；用途是调查停滞及消息身份，没有称为完整场景结果。部分诊断及两次完整修复后运行加载本轮 `allow_debug.so`，其构造函数仅设置本进程的调试attach许可；未改变优化参数。最终完整运行未触发停滞抓栈。
- 第一次完整修复后A（身份/deadline修复，尚未修seed）：`team_liveness_fix_20260911_235337/attempt_1/`，完整失败/退化数据保留。
- 最终完整A：`team_liveness_fix_20260911_235337/run/`；最终统计 `analysis/`，原始运行哈希 `final_run_hashes.json`，运行源码/二进制哈希 `run_source_manifest.json`、`run_build_manifest.json`。
- 修改前diff/hash在 `baseline/`；最终相对本轮修改前的7个文件diff在 `changes.diff`、`changed_files.json`。其中包含新增行为测试和文档。
- 停止后核对源码与最终运行版本完全一致：`source_frozen_verification.json`；运行实际装载当前workspace二进制/库且hash一致：`final_verification.json`。
- 旧原始日志的gzip路径及解压后原始SHA256：`lossless_compression_manifest.json`。分析无需伪造或覆盖旧baseline。
- 本轮ROS/RViz/测试进程已退出，端口12585/12586/12587/12588均关闭。没有处理其他用户进程，RRCT未访问。

当前应定性为：**协议身份复用与参考初始化错误已修，团队参考能够实际延续；完整行为仍有明显brake负担和合围不足，整体问题未全部解决。已遵从停止修改指令，不再继续试验或改生产代码。**
