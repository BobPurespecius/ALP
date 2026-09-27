# ALP brake 删除验收与多方向协同质量：实现完成，整体验收失败

最终源码对应 build9。**不能交付为“连续运动已恢复”或“高质量多方向协同已经在完整 A 中成立”。** planner-level brake 已删除，30秒开放回归和22项测试通过；但两次完整 FULL ON + native RViz Scenario A 均失败。最终复验出现 3 次 terminal hold、11 个 swarm 间距违规采样，HIGH_QUALITY_MULTI3 仅 4.6979%。未恢复 brake、未调权重刷结果、未再跑第三次完整 A。

## 实际实现

25°/170°仍只判断几何合围。共享 `encirclement_geometry.h::multiviewQuality` 使用三个 minor pair angles，25°以下 q=0，25°至60°为 cubic smoothstep，60°以上 q=1；Q_dir=mean(q)，J_dir=mean((1-q)^2)。解析 XYZ 导数、wrap、ID permutation 与饱和区零梯度均有测试。没有固定120°slot或奖励，也没有60°硬执行门。低于25°的平坦区按用户定义没有恢复梯度，仍依赖原候选与Target/Guide提供可行离开路径。

新增位置协同项仅进入 Team Joint P/T/yaw 的生产 objective：共同 world-time sample 计算三机位置相对目标的 J_dir，通过 `fixedTimeSampleGradient` 同时传到三机 MINCO P/virtual-T；yaw继续负责FOV。local没有新J_dir或第二套Target bearing attraction。

Target generator与cooperative manager序列化 proposal统一使用原 `camera_time - .05*reach - .01*continuity + Q_dir` 排序，路径证据仍由原异步生产Guide规划/团队验证提供。Joint acceptance增加Q改善收益，保留camera、K2/blackout、geometry相对收益和全部hard safety守卫。相机已饱和时可以接受Q改善，不能以Q改善豁免可见性回退。

只增加共享参数 `/encirclement_geometry/multiview_bad_sep_deg=25`、`multiview_good_sep_deg=60`、`multiview_weight=1`，没有新增manager、协议、消息枚举或恢复状态机。受控短测所需init/relative X被改为launch显式arg，默认值不变；展开默认参数比较只新增这三个multiview参数。Team Reference身份、共同epoch、optimized yaw、proposal/3 ACK/commit/adoption保持原链。

## 一次scale audit与测试

[multiview_scale_audit.json](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/multiview_scale_audit.json) 使用真实production MINCO/FOV objective的典型2m距离、0.928m/s、30/165/165布局，包括FOV边缘情况；它是生产函数fixture，不是假称真实障碍场景回放。

| 项 | cost | P梯度范数 | tau梯度范数 |
|---|---:|---:|---:|
| J_dir，w=1 | 0.299139 | 0.274306 | 0.089832 |
| FOV边缘J_acc | 2.267394 | 0.673855 | 0.193596 |
| K2/blackout | 0.237174 | 0.126512 | 0.033853 |

权重1从audit起一直未改。安全仍是硬约束。新增完整objective有限差分还暴露原target-facing yaw prior漏传P/tau导数，修复了导数接线，未改该项公式或权重。

构建命令：`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`。build9六个构建包成功，保留-O2/-O3，无fast-math。[build9.log](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/build9.log)；[22项测试记录](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/tests.json)；[梯度/质量测试](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/multiview_contract_test.log)。

新增XYZ finite difference最大误差8.80602e-11，完整MINCO P/virtual-T/yaw误差5.8222e-11。真实优化fixture Q_dir从0.684227改善至0.696636，安全/相机守卫通过。另含非uniform饱和等价、ID交换、±179° wrap、原25°/170°、yaw独立、camera平台接受/回退拒绝、高Q共同参考下多次local生成和无参考不等待测试。原unit/gradient/ROS/contract保留。测试通过**没有覆盖完整场景所有可行候选供给情况**，运行失败即揭示此覆盖缺口。

## Brake删除与连续运动验收

生产源码审计166个相关文件，关键brake符号可达路径0：[源码审计](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/planner_brake_source_audit.json)。`validatedBrakeSuccessor`、VALIDATED_BRAKE生成/commit/adoption、BRAKE_UNAVAILABLE、专用reserve参数和planner emergency polynomial路径均移除；无改名stop fallback。保留有限轨迹结束后的原executor terminal模型来真实暴露失败，不能将其视为成功fallback或coverage。

保留pending不冻结local、异步team、scheduled predecessor、future activation/PVA、deadline/cancellation、Local-SFC、SIDE/A*/MINCO/SCP和原安全检查。正常rolling终端速度取target/reference连续状态；正常任务不默认v(T)=0。最终运行中低于0.2m/s的三个terminal tail均发生在启动0.143/0.194/0.267秒，之后terminal speed中位数约0.928m/s。

本轮已处理：target twist与FOV模式解耦；future activation warm切片；动态repair在candidate分类之前执行；完整static repair接线；短stationary假设不覆盖moving执行，同时保留短moving候选；有限Team Reference之后生成新moving continuation而不延长旧参考；偏离head临近旧junction时放弃不可用seed但保留共同时间cost；短moving候选可生成并完整验证新MINCO continuation，失败则保留原候选；远期冲突时验证真实有限安全前缀，消除固定约0.19秒prefix与coverage预算必然不兼容的问题。

之前brake阶段两次完整A也失败，数据保留于 `continuous_motion_20260912_004327/full_attempt1_failed` 和 `continuous_motion_20260912_004327/run`，分别记录881/527次starvation和各3次hold，不能追认通过。本轮multiview两次完整A亦均失败。

最终二进制30秒开放回归：通过=True；brake/starvation/hold/提前耗尽/emergency均0；低速episode=0；activation=4.237240 / 5.505075 / 3.303045 Hz。collision/swarm/unvalidated=0/0/0。它证明简单场景连续性，不证明森林场景通过。[开放回归](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/open_analysis/motion_summary.json)。

## 短生产链与完整运行的区别

受控20秒生产场景用0°/30°/195°低差异初态及远处真实障碍，保持FULL ON、原生产planner/协议/阈值。它是验证fixture，不是调整A场景或优化权重。

| 运行 | Joint尝试 | Q改善成功 | Q改善commit | Q改善实际三机adoption |
|---|---:|---:|---:|---:|
| 受控短测 | 26 | 21 | 16 | 16 |
| 完整A第一次 | 46 | 17 | 15 | 15 |
| 完整A最终复验 | 5 | 0 | 0 | 0 |

短测有13条改善链在同world time比较时三机曲线均发生位置变化。选取其中一条完整identity证据（对应短测，不能冒充最终A）：

```json
{
  "team_solution_id": 1,
  "coordination_generation": 3,
  "reference_id": 1,
  "activation": 1789150612.19732,
  "trajectory_ids": {
    "2": 7,
    "1": 7,
    "0": 6
  },
  "proposal_timestamp": 1789150611.6293578,
  "pre_proposal_trajectory_ids": {
    "0": 5,
    "1": 6,
    "2": 6
  },
  "Q_dir_before": 0.772592551,
  "Q_dir_after": 0.843404923,
  "camera_time_before": 4.5,
  "camera_time_after": 4.5,
  "K2_before": 1.0,
  "K2_after": 1.0,
  "three_position_curve_changes_m": [
    0.14319729876572582,
    0.1764906831188443,
    0.0001250278578384923
  ]
}
```

该链3 ACK齐全，原JOINT source actual activation齐全，前后pairwise separation在[完整短测identity记录](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/low_diversity_analysis/multiview_identity_chains.json)，采用proposal发布前的已提交曲线作比较，未用commit后的local更新冒充before。相机未牺牲，但改善幅度有限；短测仍不能证明持续达到60°诊断标准。

## 最终完整Scenario A

场景 `long_cylinder_forest_visibility_stress.json`，FULL ON + native RViz，目标路线80.2045407446秒，stop_reason=`target_replay_normally_completed`。首轮+一次明确coverage接线修复后的必要复验；没有OFF/A-B、没有为了指标改权重或第三次完整复跑。首次失败原始数据在 `full_attempt1_failed`，最终在 `run`。最终统计窗口 [1789151019.607393026, 1789151099.784297705]，共同odom/visibility起点至route结束，长 80.176904678s；所有核心几何/质量/可见性/activation均按该窗。仅低速连续运动判据另排除启动3秒。

| 指标 | 最终A |
|---|---:|
| strict25°/170°合围 | 11.9247% |
| 同半平面 | 78.4000% |
| g_min P50 / P95 / min | 11.840051 / 70.711622 / 0.005870° |
| g_max P50 / P95 / max | 205.629498 / 260.024945 / 314.442404° |
| actual Q mean / P50 / P95 | 0.740349 / 0.666667 / 1.000000 |
| min pair separation P50 / P95 / min | 11.840051 / 70.711622 / 0.005870° |
| HIGH_QUALITY_DIRECTIONAL | 11.2087% |
| HIGH_QUALITY_MULTI3 | 4.6979% |
| HIGH_QUALITY_MULTI2 | 14.0876% |
| camera-time / mean_visible | 124.247133 camera·s / 1.549662 |
| K2 / All3 / None | 49.5181% / 9.4355% / 3.9873% |
| 最长K2 loss / blackout | 40.111670 / 1.366888 s |
| 实际activation 0/1/2 | 0.286866 / 1.796028 / 3.641947 Hz |
| hold / 提前耗尽 / emergency | 3 / 3 / 0 |
| starvation日志事件 | 508 |
| collision / swarm violation / 未验证执行采样 | 0 / 11 / 3534 |
| planner / command / odom峰值速度 | 3.014627 / 3.010741 / 3.232924 m/s |

二进制source为moving的activation率与总率一致，因为brake为0；不以此声称无人低速/停止。已提交poly的planner峰值仍超过名义3m/s，未改现有可行性容差消除告警。未验证执行3534采样主要来自有限轨迹耗尽后的hold；invalid polynomial adoption标记数为0，两者不是同一口径。P/V/A接管残差最大 2.76899851e-06m / 9.97624343e-07m/s / 1.88473258e-06m/s²；不能以接管连续掩盖随后无successor。

## Team / Local / Actual与“两前一后”归因

共同reference实际覆盖 2.164617秒，2.6998%，按有效reference ID+epoch+end对齐，不按JOINT source样本占比。三机local修正但仍引用共同计划的时长分别 1.899014 / 1.999806 / 1.866484秒。

| 层 | 全部本层可用窗Q mean | 共同reference窗Q mean | 两前一后 | 前两机<25° | 前两机<60° |
| team | 0.999897 | 0.999897 | 0.0000% | 0.0000% | 0.0000% |
| local | 0.738489 | 0.999897 | 37.9229% | 11.2251% | 33.1383% |
| actual | 0.740349 | 0.998804 | 37.9601% | 10.8115% | 32.6163% |

Team仅有2.16秒有效数据，local/actual是80.18秒；**不能把不同分母的Team≈1与actual≈0.74直接相减并归咎follower**。相同reference窗的TEAM_TO_LOCAL_Q_DIR_LOSS=4.55008053e-07，LOCAL_TO_ACTUAL=0.00109270179。三机actual参考位置误差mean/P95/max（米）：[{"mean": 0.05303618609497174, "p95": 0.10378403985624417, "max": 0.1204916450801938}, {"mean": 0.07214367287275451, "p95": 0.13121542062557517, "max": 0.15297423940354785}, {"mean": 0.13783862144026504, "p95": 0.3672552751243423, "max": 0.41949673472682925}]。

最终A中低差异两前一后主要在共同reference缺失及轨迹耗尽后出现：local的<60°比例33.1383%，actual 32.6163%，不是controller单独把长期高Q Team轨迹破坏。最终5次joint attempt只有一次OK且Q已饱和，其余为{'INITIAL_METRICS_INVALID': 3, 'INITIAL_DYNAMICS_VIOLATION': 1, 'OK': 1}，没有Q改善的实际三机接管。首次A虽有15条Q改善三机接管，但Team可用窗Q均值只有0.795036、全程实际质量仍差。当前团队目标/候选可行域允许低Q布局，没有证据证明它已持续共同实现高质量观测。

## 未完成的关键修复与“0.3秒停一下”

1. **普通fresh接续语义仍有缺口。** 最终UAV0约7.03s时Guide不可用、warm cache返回HORIZON_TOO_SHORT，`computeInitState`旧optimal初始化仍把旧有限remaining duration配上新的数米外local_target。0.354382s初值要求39.600291m/s、42469.700957m/s³ jerk；随后越缩越短，到0.301644s要求46.359206m/s。UAV1约38.66s有同类现象。新的已可行候选continuation位于后面的finalization，救不了此前已经不可行的初值。**这一根因仍未修复，不把反复调用replan等同于持续moving trajectory供给。** 证据：[剩余初始化根因](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/remaining_initializer_root_cause.json)。
2. UAV2约76.08s主要为static地图拒绝（例如预测位置34.44/3.10m，occupancy=1），原候选/repair没有形成可执行绕行。未改SIDE/A*策略或放松static/swarm安全检查。
3. local仍在调用：三机replan约 17.174522 / 17.274301 / 10.264801 Hz；但actual activation降至上表，说明非阻塞接口还在，**连续可行successor供应没有成立**。Team缺少持续可行输入，后续三机增强也不能持续发生。
4. 本轮已耗尽允许的两次完整A。未再调权重、未重引入stop，也未在最终运行后追加未经完整复验的生产修复。因此最终代码与复验二进制一致，剩余问题明确保留为FAIL。

最终低速episode（速度<0.2m/s持续>0.2s，排除启动3秒）共9，包含未重新启动的长期停止。P50/P95/max=0.333454 / 59.027447 / 71.667323秒。三次早期NOMINAL短低速及六个hold阶段低速分开见[运动summary](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/motion_summary.json)；0.3秒低速并没有随brake删除全部消失。长时间停止由solver/candidate持续失败→有限trajectory耗尽→原terminal模型导致，普通成功接管本身没有速度跳变。早期短低速属于rolling位置轨迹的减速形状，不能只因没有brake就宣称根因全部修完；controller对瞬态的贡献未单独隔离。

| 实际source transition | 样本数 | before speed P50 | after speed P50 | 前后0.3s odom最低速度 |
| NOMINAL->NOMINAL | 446 | 1.102692 | 1.102692 | 0.000879 |
| NOMINAL->JOINT_TEAM_SOLUTION | 3 | 0.709348 | 0.709349 | 0.201994 |
| JOINT_TEAM_SOLUTION->NOMINAL | 3 | 1.331730 | 1.331730 | 0.663014 |
| NOMINAL->SIDE_MINUS | 2 | 0.530783 | 0.530783 | 0.243799 |
| SIDE_MINUS->SIDE_PLUS | 1 | 0.192786 | 0.192786 | 0.197645 |
| SIDE_PLUS->NOMINAL | 2 | 0.908354 | 0.908354 | 0.197645 |
| SIDE_MINUS->NOMINAL | 1 | 1.749357 | 1.749357 | 1.250845 |
| NOMINAL->SIDE_PLUS | 1 | 1.234237 | 1.234237 | 1.162893 |

Persistence重新验证不产生新的actual activation，未凭空添加PERSISTENCE→NORMAL activation样本。

STOP_GO_CAUSED_BY_PLANNER_BRAKE: NO  
STOP_GO_CAUSED_BY_TERMINAL_ZERO_VELOCITY: NO（启动阶段三个零尾速除外；正常任务尾速非默认零）  
STOP_GO_CAUSED_BY_TRAJECTORY_GAP: YES  
STOP_GO_CAUSED_BY_SOLVER_FAILURE: YES  
STOP_GO_CAUSED_BY_CONTROLLER_TRACKING: INCONCLUSIVE（不足以解释规划供给消失，但瞬态贡献未隔离）

[speed/source/id/activation同轴图](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/continuous_motion_timeline.png)；[Team/local/actual Q及pair separation同轴图](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/multiview_team_local_actual.png)；[三层g_min/g_max同轴图](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/team_local_actual.png)；[统一world-time逐行数据](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/team_local_actual_world_time.csv)；[逐activation运动诊断](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/analysis/activation_motion.csv)。

## 与最近真实完整运行比较

保留的 `team_liveness_fix_20260911_235337/run` raw可用，采用同一multiview离线算法重新计算，[baseline重算](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/baseline_analysis/multiview_summary.json)。旧运行含大量brake，故仅作描述性比较，不能把差异单独归因于J_dir或brake删除。

| 指标 | 最近历史完整运行 | 本轮首次A | 最终A |
|---|---:|---:|---:|
| actual Q mean | 0.904616 | 0.492195 | 0.740349 |
| HIGH_QUALITY_DIRECTIONAL | 58.5978% | 11.0151% | 11.2087% |
| HIGH_QUALITY_MULTI3 | 48.9448% | 7.0796% | 4.6979% |
| 前两机<60°且两前一后 | 23.7590% | 30.6428% | 32.6163% |
| 共同参考覆盖 | 53.1974% | 21.4577% | 2.6998% |

最近历史报告feedback_40的strict合围52.12%、camera-time226.974s、K2约99.33%；最终A分别只有11.92%、124.247s、49.52%。明显退化，不能宣称多方向协同目标达成。

## 清理、版本与留档

清理前按metadata/正常route结束标志/timestamp识别59个完整A记录，清理时最新3次FULL A的整个相关目录保留。删除130个旧大raw文件，涉及61个历史目录，释放5947824795字节；“61”不是删除61个完整目录，摘要/manifest/源码/backup/build/test/feedback保留。[清理manifest](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/../cleanup_before_multiview_20260912_010408.json)。未跟随symlink删除项目外内容，未删除ros_ws/feedback。

源码/历史diff与初始测试在 `continuous_motion_20260912_004327/baseline`、`multiview_20260912_013929/baseline`。[当前修改差异](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/changes_since_multiview.diff)；[source hashes](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/run_source_manifest.json)；[build hashes](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/run_build_manifest.json)；[runtime /proc路径与maps](/home/bob/ALP/egov2_fc65423_constvel/multiview_20260912_013929/run/proc_snapshot.json)。运行中的planner/coordinator/worker及本轮记录的关键库hash匹配预运行manifest。plan_env/path_searching加载路径也均为本workspace；它们未进入预运行hash清单，补充当前hash列于failure_diagnostics，不能声称拥有未采集的预运行hash。

每个完整run保留PolyTraj、PositionCommand、odom、candidate bundles、Target/Guide/team messages、ROS日志、scene/command、native RViz截图和identity分析。本轮所有相关ROS/RViz/recorder进程已退出，隔离端口均关闭；没有处理其他用户进程。RRCT未访问、未修改。

## 最终状态

以下运行数值均来自最终build9完整A；开放回归另列，测试通过不代表整体验收通过。

```text
TASK_ACCEPTANCE_PASS: NO
BRAKE_REMOVAL_IMPLEMENTATION_COMPLETE: YES
BRAKE_REMOVAL_FULL_SCENARIO_ACCEPTANCE_PASS: NO
OLD_SIMULATION_DATA_CLEANED: YES
LATEST_FULL_RUNS_RETAINED: 3
CLEANUP_OLD_RAW_RUNS_REMOVED: 61 affected directories / 130 raw files
CLEANUP_BYTES_FREED: 5947824795
MULTIDIRECTIONAL_QUALITY_IMPLEMENTED: YES
MULTIDIRECTIONAL_QUALITY_IS_TEAM_LEVEL: YES
MULTIDIRECTIONAL_ACTUAL_FULL_RUN_ACCEPTANCE_PASS: NO
FIXED_120_RESTORING_REINTRODUCED: NO
MULTIVIEW_IS_HARD_EXECUTION_GATE: NO
ORIGINAL_25_170_ENCIRCLEMENT_DEFINITION_CHANGED: NO
MULTIVIEW_GOOD_SEPARATION_DEG: 60
ACTUAL_Q_DIR_MEAN: 0.7403492506682082
ACTUAL_Q_DIR_P50/P95: 0.6666666666666666 / 1.0
HIGH_QUALITY_DIRECTIONAL_RATIO: 0.11208713127758825
HIGH_QUALITY_MULTI3_RATIO: 0.04697920027146392
HIGH_QUALITY_MULTI2_RATIO: 0.14087563873250958
TWO_FRONT_ONE_BACK_RATIO: 0.3796014759577888
TWO_FRONT_CLOSE_ONE_BACK_RATIO_25: 0.10811459702860161
TWO_FRONT_LOW_DIVERSITY_ONE_BACK_RATIO_60: 0.32616285710184617
TEAM_Q_DIR_MEAN: 0.9998974544340244
LOCAL_Q_DIR_MEAN: 0.7384894622238479
TEAM_TO_LOCAL_Q_DIR_LOSS: 4.550080527315789e-07
LOCAL_TO_ACTUAL_Q_DIR_LOSS: 0.0010927017886305792
MULTIVIEW_JOINT_ATTEMPT_COUNT: 5
MULTIVIEW_JOINT_IMPROVEMENT_COUNT: 0
MULTIVIEW_COMMIT_COUNT: 0
MULTIVIEW_ACTUAL_3UAV_ADOPTION_COUNT: 0
EXECUTED_ENCIRCLEMENT_RATIO: 0.11924677743053663
SAME_SEMICIRCLE_RATIO: 0.7840004947926634
ACCUMULATED_CAMERA_VISIBLE_TIME: 124.24713277816772
MEAN_VISIBLE: 1.5496623781701826
K2: 0.495180802958165
ALL3: 0.09435492575148287
NONE: 0.039873350539465315
LONGEST_K2_LOSS: 40.11166977882385
LONGEST_BLACKOUT: 1.3668878078460693
TEAM_REFERENCE_COVERAGE_RATIO: 0.026998009362926435
ACTUAL_ACTIVATION_RATE_UAV0/UAV1/UAV2: 0.286866 / 1.796028 / 3.641947
MOVING_ACTIVATION_RATE_UAV0/UAV1/UAV2: 0.286866 / 1.796028 / 3.641947
MOVING_SUCCESSOR_STARVATION_COUNT: 508
TERMINAL_HOLD_COUNT: 3
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 3
EMERGENCY_STOP_COUNT: 0
STOP_GO_EPISODE_COUNT: 9
STOP_GO_ROOT_CAUSE: moving candidate generation fails; stale shrinking initialization; finite trajectory exhaustion; separate short NOMINAL decelerations remain
MIN_MOVING_SPEED_EPISODE_COUNT: 9
LOW_SPEED_EPISODE_P50/P95/MAX: 0.333454 / 59.027447 / 71.667323
COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 11
UNVALIDATED_EXECUTED_SAMPLES: 3534
MAX_PLANNER_SPEED: 3.0146272804102088
MAX_COMMAND_SPEED: 3.010741171995602
MAX_ODOM_SPEED: 3.23292409145003
PLANNER_LEVEL_BRAKE_REMOVED: YES
VALIDATED_BRAKE_PRODUCTION_PATH_EXISTS: NO
BRAKE_USED_AS_EXECUTION_COVERAGE: NO
PLANNER_BRAKE_SYMBOLS_REMAINING: NONE in production; historical reports and negative contract assertions only
PLANNER_BRAKE_REACHABLE_CODE_PATHS: 0
PLANNER_LEVEL_BRAKE_REINTRODUCED: NO
NORMAL_TRAJECTORY_DEFAULTS_TO_ZERO_TERMINAL_SPEED: NO
CONTINUOUS_MOVING_EXECUTION_CONTRACT_ACTIVE: YES
NONBLOCKING_LOCAL_REPLAN_PRESERVED: YES
TEAM_REFERENCE_PRESERVED: YES
TARGET_GUIDE_TRAJECTORY_STRUCTURE_PRESERVED: YES
NO_NEW_STOP_FALLBACK_PATCH_INTRODUCED: YES
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES
CONTINUOUS_MOTION_TEST_PASS: YES (30s open regression)
SCENARIO_A_CONTINUOUS_MOTION_CONTRACT_PASS: NO
SCENARIO_A_FULL_RUN_COMPLETED: YES
SCENARIO_A_ACCEPTANCE_PASS: NO
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_41.md
```

最终结构仍是：25°/170°判“有没有围住”，Q_dir判“方向是否互补”，Team Joint决定空间与共同时间轨迹，Local负责连续安全兑现。**当前最后一句在复杂场景尚未实现，不能用新增quality项或停车掩盖。**
