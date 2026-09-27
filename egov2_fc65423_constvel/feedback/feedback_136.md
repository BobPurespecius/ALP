# Feedback 136 — Local SIDE constrained SCP 接线与 FULL 结果

日期：2026-09-27。唯一工作区：`/home/bob/ALP/egov2_fc65423_constvel`。原任务指定 `feedback_135.md`，但该文件在本轮开始前已存在，内容是 run9 静默停摆的只读报告；为保留已有工作，本报告顺延为 `feedback_136.md`。未访问 RRCT，未运行 OFF、单元或合同测试。

## 修改

在 [poly_traj_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) 中，Local SIDE topology 或 LOS observation plane 本身现在触发 `runCandidateHardCorridorSCP()`，不再等 BODY、PVAJ、动态或 swarm 硬风险。SIDE hard band 的生成条件改为 SIDE topology 且 P 可优化：Local P/T 和 Team PT 使用原有 `sin(pi*u)` band；显式 Team T-only 不生成空间行，Team AUTO 带 SIDE 时选 PT。trial 和 final 均检查同一 region violation。

LOS observation 行以非负 slack 加入**同一次** OSQP QP，和 PVAJ、static、dynamic、swarm、Local-SFC、SIDE band、trust 行共用决策变量。沿用已有 LOS slack 权重；LOS QP 失败时仅重试原硬约束 QP，不因 LOS 不足拒绝候选。`enforceCandidateLosPlanesSCP()` 的旧函数定义尚在源码，但 production 调用点已全部移除；最终没有第二次 LOS 多项式改写。修正 slack 非负行的系数，使其为 1，二次权重只进入 Hessian。没有修改安全阈值、SIDE band 形状、SCP trust、Team 选择或 K3 comparator。

第一次 FULL 暴露另一个现有出口：SCP 失败后的 `FEASIBLE_INITIALIZER_FALLBACK` 可以把 band 外的 seed 以 SIDE 身份交给后续选择。于是仅在 [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp) 的该 fallback 处复用现有 `evaluateCandidateSideRegion()`，要求最大 violation ≤0.002；LOS 仍不作 hard gate。最终源码在第二次 FULL 前构建完成，之后未修改 production。

`git diff --check` 通过；两次 `catkin build -j2 --no-status` 均为 **25/25 packages succeeded，package warnings 0**。catkin 有缓存 `CMAKE_PREFIX_PATH` 与当前环境不同的提示，但无 package 编译警告。

## 两次 FULL ON 与数学结构

两轮都使用原 v6 场景、FULL、K3 repair/escalation ON，Team、动态障碍和安全模块照常开启。第一次 run `20260927_122252_1256991` 用于发现并关闭 fallback 漏口；最终代码 run 为 **`20260927_123509_1279906`**。两次均 BOOT-12、`FINAL_EXIT_CODE=0`；最终 run 的 runner 29/29 自有进程退出，强杀 0、残留 0、杀外部进程 0。每轮结束后仅压缩本轮 stdout、console 和 ROS `.log*`，保留 CSV、manifest、状态文件与可解压日志；最终 run 约 209 MB。

最终 run 的可解析遥测如下。ROS stdout 有少量线程交织行，因此这些是**完整可解析行**的计数，不把损坏行臆补为成功：

| 指标 | 最终 run |
|---|---:|
| Local SIDE dispatched to constrained SCP | 1,165 条可解析 SIDE SCP 结果；对应源码派发路径已接通 |
| SIDE hard-region rows active | 1,165/1,165；累计 11,998 行（跨迭代重复计数） |
| SCP `SCP_FINAL_OK` | 216/1,165 |
| seed region violation | P50 0.143398，P95 1.090664，max 2.890938；仅 162/1,165 在 0.002 内 |
| 成功 SCP 的 final region violation | max 0.001908；`>0.002` 为 **0/216** |
| fallback region 审核 | 839 次；834 次 band 外拒绝，5 次接纳且 max violation 0 |
| 成功 SIDE 的非零 LOS slack | 27 次；max 1.485849 |
| 独立 post-LOS polynomial mutation | **0 次**，production 无调用点 |

在 SCP 成功的 216 条中，151 条 seed 原本已在 band 内，65 条从 band 外进入 band；938 条 band 外 seed 未能成功 SCP。失败码主要为 `T_TRUST_TOO_SMALL` 413、`P_TRUST_TOO_SMALL` 211、`P_T_TRUST_TOO_SMALL` 85、`TRUST_RETRY_EXHAUSTED` 109、`SIDE_REGION_FINAL_VIOLATION` 68。失败码仅说明当前行集和数值求解未收敛，**不能**据此认定扩大 trust 安全或必然可行。Manager 记录 210 条 `SIDE` final success；K3 comparator 有 91 批，50 条 SIDE 被最终选择，48 条匹配到安全标记为 true 的实际 SIDE 轨迹。没有为本 run 保存独立的 blocker-relative 正确侧 oracle，因此“正确 SIDE 被选/执行”的精确数**不可判定**。

Feedback134 的旧实现有 10/11 个严格正确 seed witness 在主优化后先丢失恢复收益，且 93/138 对 post-LOS 前后轨迹发生 PVA break。新 run 的 `SIDE_TOPOLOGY_COLLAPSE=0/216` 是**band violation 口径**，不是对旧 11 个 witness 的同对象重放；不能宣称旧 10/11 全被修好。`POST_LOS_PVA_BREAK=0` 是因为后置变异阶段已消失；不代表整条执行链没有其他 PVA 或覆盖问题。最终 run 的 51 条可读 SIDE `topology-post-check` 均记录 `dynamics_valid=1`、`static_valid=1`、`safety_class=ABSOLUTE_SAFE`；日志交织使其不能充当全部提交的逐条形式证明。新增 `physical_max_violation` 是 native ray/dynamics 等诊断最大值，成功记录中 8 条仍大于 0.002；它与最终 static map preflight 的判定口径不同，不能把该单值冒充 SafetyKernel 证书。执行 CSV 与 preflight 是下节安全判定依据。

## 执行可见性、安全与失败边界

| 指标 | 既存 Feedback134 run11 | 首轮接线 run | 最终 run |
|---|---:|---:|---:|
| K3 / ALL3 | 0.891351 | 0.879421 | **0.884157** |
| K2 | 0.993183 | 1.000000 | **0.994037** |
| K3 loss 总时长 | 8.498273 s | 9.441720 s | **9.074259 s** |
| 最长 K3 loss | 未在本表重算 | 1.566509 s | **1.702291 s** |
| K3 loss episodes / LOS 涉及 episodes | 未在本表重算 | 9 / 9 | **8 / 8** |
| blackout samples / mean visible | 未在本表重算 | 0 / 2.879421 | **0 / 2.878194** |

最终 run 的 2,348 个 visibility 样本覆盖 78.247993 s。执行轨迹 CSV 最小 static/dynamic clearance 为 **0.275479/0.198878 m**，非正接触样本均为 0，最小机间距约 **1.241943 m**。没有在可读日志中发现 PVA mismatch、partial Team activation 或 PVAJ-violation commit 标记；不能把标记缺席提升为对所有提交的独立连续证明。

**系统安全/活性验收未通过。** 最终 run 在目标路线期间有 13 条 `TERMINAL_HOLD`、`safety_validated=False` 执行样本，首条为 UAV2、轨迹 259、`t=73.918942 s`；完整 273.8 s 轨迹文件有 11,602 条未验证 hold 样本，并记录 24 条 `MOVING_SUCCESSOR_STARVATION` 和 156 条 `TERMINAL_HOLD_ENTER` 日志。首轮分别为目标窗口 13 条、完整 11,737 条。既存 run11 的目标窗口也有 11 条 hold，因此仅凭这些运行不能把 hold 归因于本轮 SIDE 修改；但新代码同样**没有**满足“无 unvalidated execution / handoff regression”的验收要求。最终 run 没有永久 planner 静默停摆，runner 完成并清理；局部 successor coverage 仍会断档。`FINAL_EXIT_CODE=0` 只证明 runner 完成。

## 结论与剩余根因

**回答原问题：是。** 旧 Local LOS-only SIDE 可绕开 constrained SCP，且 SIDE hard rows 被 `team_scp` 门禁屏蔽；Feedback134 的严格 witness 证明主优化中存在恢复几何退化。本轮接通后，**成功通过 SCP 的 SIDE** 保持原 band（216 条中 0 条超出 0.002），fallback 也不再绕过 band；LOS 仍是有非零 slack 的软 visibility authority，没有后置 QP 破坏 physical polynomial 的机会。这个结论限于 band 与已记录 hard checks；本轮没有证明 K3 恢复收益提升，最终 K3 反而低于旧 run11。

当前首要剩余优化瓶颈是**SIDE initializer 与既有 hard band 不一致，加上当前 SCP 对 band 外 seed 的可达率低**：1,003 条可解析 seed 在 band 外，只有 65 条经 SCP 进入 band，834/839 次 fallback 因同一不变量被正确拒绝。这比继续调 visibility/side 权重或修改 C3/D3 排序更靠前。下一步应对同一候选记录 seed 到 band 的最短位移、active time、P/T Jacobian 和冲突行，核对 band/seed 的相位与几何定义；不能仅凭 `*_TRUST_TOO_SMALL` 扩 trust。独立的执行瓶颈是末段 successor coverage/terminal hold，应按轨迹 259 的有效前缀、deadline 和下一候选时间链审计，不能由 K3 总指标推断原因。本轮不再修改或追加仿真。

## 收尾字段

```text
LOCAL_SIDE_SCP_DISPATCH_FIXED: YES
LOCAL_SIDE_HARD_REGION_ROWS_FIXED: YES
LOS_SOFT_SLACK_AUTHORITY_PRESERVED: YES

SIDE_TOPOLOGY_COLLAPSE_BEFORE: 10/11 strict recovery witnesses lost benefit in Feedback134; different metric from band violation
SIDE_TOPOLOGY_COLLAPSE_AFTER: 0/216 successful SCP finals above 0.002; fallback 5/5 accepted within band
POST_LOS_PVA_BREAK_BEFORE: 93/138 paired post-LOS mutations in Feedback134
POST_LOS_PVA_BREAK_AFTER: 0 post-LOS mutation stages (call sites removed)

LOCAL_SIDE_DISPATCHED_TO_CONSTRAINED_SCP: >=1165 parseable attempts
LOCAL_SIDE_HARD_REGION_ROWS_ACTIVE: 1165/1165 parseable attempts
LOCAL_SIDE_HARD_REGION_ROW_COUNT: 11998 cumulative across SCP iterations
SIDE_SEED_REGION_VIOLATION: P50 0.143398; P95 1.090664; max 2.890938
SIDE_FINAL_REGION_VIOLATION: max 0.001908 among 216 SCP successes
LOS_SLACK_NONZERO: 27 successful SIDE SCP records
LOS_SLACK_MAX: 1.485849
SIDE_MINCO_SUCCESS: 210 manager SIDE final-success logs; 216 SCP_FINAL_OK logs (different stages)
SIDE_HARD_SAFE: 51 readable SIDE topology-post-check logs; 50 selected, 48 observed executed
CORRECT_SIDE_SELECTED: UNKNOWN; no independent final-run blocker-relative oracle
CORRECT_SIDE_EXECUTED: UNKNOWN; no independent final-run blocker-relative oracle

K3: 0.884157
K2: 0.994037
K3_LOSS_TOTAL: 9.074259 s
LONGEST_K3_LOSS: 1.702291 s
K3_LOSS_EPISODES: 8
LOS_CAUSED_EPISODES: 8
BLACKOUT_SAMPLES: 0
MEAN_VISIBLE: 2.878194

CONTACTS: 0 static / 0 dynamic sampled contacts
UNVALIDATED_COMMITS: 0 observed in readable commit/post-check logs; exhaustive identity-level proof unavailable
PVAJ_VIOLATION_COMMITS: 0 observed; exhaustive identity-level proof unavailable
UNVALIDATED_EXECUTION: 13 target-window samples / 11602 full-file TERMINAL_HOLD samples
SUCCESSOR_STARVATION: 24 logged events
TERMINAL_HOLD_ENTER: 156 logged entries
PLANNER_DEADLOCK: NO permanent silent stall; YES successor coverage failures
SYSTEM_SAFETY_ACCEPTANCE: NO

PRIMARY_REMAINING_ROOT_CAUSE: SIDE seed/band mismatch and low SCP reachability for band-outside seeds; separate successor coverage failure remains
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[最终指标 JSON](artifacts/feedback136_metrics.json)、[最终 run manifest](../runs/20260927_123509_1279906/run_manifest.txt)、[退出状态](../runs/20260927_123509_1279906/exit_status.txt)、[进程清理](../runs/20260927_123509_1279906/process_status.txt)、[压缩 stdout](../runs/20260927_123509_1279906/roslaunch_stdout.log.gz)、[visibility](../runs/20260927_123509_1279906/visibility.csv)、[执行轨迹](../runs/20260927_123509_1279906/visibility_trajectory.csv)；首轮 run 证据在 `runs/20260927_122252_1256991/`。
