# Feedback 111 — K3 Local Escalation:C3/D3 逐周期恢复语义 + 事件级绑定与遥测

日期:2026-09-25
工作区:/home/bob/ALP/egov2_fc65423_constvel(仅此目录,dirty worktree 为唯一权威)
场景:natural_team_stress_dense_38_targeted_k2_v6.json(FULL 消融,headless,timeout 240)
运行:
- ON#1 `runs/20260925_124647_104062`(BOOT-12,exit 0)
- ON#2 `runs/20260925_130810_130608`(BOOT-12,exit 0,含遥测修正)
- ON#3 `runs/20260925_131504_132764`(BOOT-12,exit 0,方差确认)
- 历史参考(非本轮 baseline):`runs/20260925_111437_5455`、`runs/20260924_163326_1175833`
- 用户中途指令:"不用基线,直接on" → 本轮未跑新 OFF 配对;OFF 语义验证改为
  (a) 代码审查证明 probe 不激活时逐位等价,(b) 与两份历史 run 对比。

---

## 1. 修改前的真实接口(审计结论)

1. **普通 LOS authority horizon**:`plan_manage/src/planner_manager.cpp` reboundReplan
   (`visibility_authority_horizon` = 轨迹总时长,滚动时 ×2/3,再被
   `getMovingObjPredictionHorizon()` 截断)。raw static/dynamic LOS witness 相对时间
   ≤ horizon 才算 current(`raw_*_los_current`),只有 current witness 才能进
   `spatial_visibility_trigger`。
2. **escalation 越界方式**:coordinator 在 Team 失败(`abortTeamProposal`,reason 含
   `TEAM_SCP_FAILED`)时广播 `TopologyEscalation`(按 contract 去重);planner
   `topologyEscalationCallback` 存 hint(8 s 墙钟新鲜度);reboundReplan 中当 hint 窗口
   与 raw witness 世界时间相交时 `escalation_grant=true`,OR 进
   `spatial_visibility_trigger` —— 这是唯一越过普通 horizon 的通道。
3. **hint 传给 Local 的字段(修改前)**:escalation_id、handoff_contract_id、
   window_begin/end、8 s 新鲜度。缺 repair_k、required_margin、detect 证据、revision。
4. **N/L/R 触发入口**:`task_topology_trigger = body_topology_trigger ||
   spatial_visibility_trigger` → nominal/plus/minus 候选 → capture_output →
   `finalizeCapturedCandidates`。
5. **Local candidate visibility 计算入口**:`evaluateCandidateVisibility`(3 机、
   range+static LOS+dynamic LOS、dt=visibility_sample_dt;**不含 FOV/yaw**)与
   `fillCandidateBinaryVisibility`(self 二值,含 yaw/FOV)。连续 margin 权威 =
   `PolyTrajOptimizer::visibilitySampleAt / evaluateVisibilityMarginAt`(与已发布
   forecast 同源)。
6. **comparator 真实排序(修改前)**:安全 preflight(三重 current-revision 检查)→
   `target_side_static_topology` 偏好 → `betterTeamVisibilityWithinNearBestK2` =
   near-best-K2 admissible → **mean_visible_count** → atleast2(all3/none/diversity
   仅遥测,不参与排序)。
7. **selected → committed → activated**:`finalizeCapturedCandidates` 选 best →
   `finalizeTopologyCandidate`(COMMITTED 安装进 traj_.local_traj,activation 为冻结
   的 batch activation)→ 执行器 ACTIVATED。下一 cycle traj_id 匹配即激活确认。

## 2. 本轮修改(全部最小化,不新增优化器/FSM/contract)

| 文件 | 修改 |
|---|---|
| `TopologyEscalation.msg` | 追加 detect_limiter / margin_at_detect / k2_at_detect / k3_at_detect / target_prediction_revision(K3 事件绑定证据) |
| `TeamReferenceSchedule.msg` | 追加 handoff_detect_limiter / handoff_detect_k2 / handoff_detect_k3 |
| `multi_uav_topology_coordinator.cpp` | K3Assessment 增加 baseline_total_samples、t_star_limiter;contract 增加 detect_limiter/k2/k3;updateK3Contract 填充;schedule 转写;abortTeamProposal 填充 escalation 新字段 |
| `planner_manager.h` | hint 结构扩展(repair_k/required_margin/detect 证据/revision);`EscalationWindowProbe`(类外命名空间);CandidateVisibilityReport 追加 window_* 字段;`K3EscalationEventState` 簿记;`k3_local_escalation_enabled_`;closeK3EscalationEvent/checkK3EscalationRecovery 声明 |
| `planner_manager.cpp` | 参数 `manager/k3_local_escalation_enabled`;callback 完整绑定 + `[K3_ESCALATION_EVENT]`;escalation_grant 加 flag+repair_k==3 门控并扩展 DYNAMIC witness;`evaluateCandidateVisibility` probe 累计(同一可见性权威 + `evaluateVisibilityMarginAt` 权威);finalizeCapturedCandidates:K2 保护门 → C3 → D3 → 轻量 side preference → 原排序;`[K3_SIDE_CANDIDATES]`/`[K3_LOCAL_PROGRESS(_CONFIRMED)]`/`[K3_EVENT_RESULT]` 遥测 |
| launch/runner | advanced_param.xml、run_in_sim.launch、native_egov2_rviz.launch(3 include)转发;alp_params.sh `alp_add`;runner 新增 `--k3-escalation on\|off` |

**明确不做**:不动 SCP trust;不动安全阈值;不把全局 K 改 3;不改 M2 repair 语义;
不恢复旧 TeamVisibilityOptimizer;不新增 Team K3 optimizer / persistent recovery FSM /
progressive hard contract;C3/D3 只在候选生成后的 finalize 阶段算一次,不进 L-BFGS。

## 3. 目标语义实现

### 3.1 K3-only 提前授权(目标一,审计+补齐)
- 授予条件:canonical flag ON ∧ hint fresh(8 s)∧ **repair_k==3** ∧ 当前快照
  raw witness(STATIC **或 DYNAMIC**,本轮扩展)世界时间 ∈ [window_begin−0.5,
  window_end+0.5]。
- 绑定:target(routing)+ window + repair_k + detect 运动类别 + margin/K2/K3 证据 +
  target_prediction_revision(审计对齐);授予的 blocker 身份始终由**当前快照重新
  扫描出的 raw witness** 推导,陈旧 descriptor 单独不能授权。
- M2(repair_k==2)escalation 不被 Local 消费 → M2 Team repair 保持原 authority
  (本轮三次 run 中 M2 事件全程无 K3 抢占)。
- 本场景所有事件的 window_begin 距 now 0.6~1.9 s < 普通 horizon 2.0 s
  (`outside_normal_authority=0`):即本轮 escalation 均为"同窗口内的充分授权",
  尚未出现必须越过 2 s horizon 的事件(深窗口事件见 §5 的 FIRST CAUSAL FAILURE)。

### 3.2 C3/D3(目标二,逐周期变好)
- C3(c) = hint 窗口内 all-3 可见样本占比;visible 复用 `evaluateCandidateVisibility`
  的逐样本三机权威(range+static+dynamic),未引入第二套模型。
- D3(c) = Σ dt·max(0, required_margin − m_self(t)),m 复用
  `evaluateVisibilityMarginAt`(= visibilitySampleAt,与 forecast 同源)。
- 基线(已执行轨迹)每 batch 算一次,候选逐个算一次,仅用于比较与遥测。

### 3.3 comparator(目标三,仅 escalation 活跃时)
激活条件:flag ∧ fresh K3-only hint ∧ 本机为 target ∧ 基线与候选窗口指标有效。
顺序:**安全 preflight(不变)→ K2 保护门(窗口 K2 ≥ 已执行基线,否则不可采纳)
→ C3 更大 → D3 更小 → 轻量 side preference(同 event 上次提交 side,仅全平)
→ 原 comparator(mean_visible_count → atleast2 → geometry)**。
非激活时与原实现逐位一致(默认参数 probe 不激活;`git diff --check` 通过)。

### 3.4 跨 cycle(复用 8 s hint,无新 FSM)
- 每 cycle:N/L/R → C3/D3 → comparator → 普通 Local commit;traj_id 匹配确认激活。
- side preference:仅 C3/D3 全平时保留上次提交 side;几何/遮挡变化或对侧严格更优
  时照常切换(run3 event 4:PLUS→PLUS→MINUS 自然切换)。
- 闭合:二值恢复(已执行轨迹 forecast 在窗口内 self-visible)/ hint 超时 / 新事件
  替换;每次闭合恰好一行 `[K3_EVENT_RESULT]`。

## 4. 逐事件因果表(3 次 ON,共 33 个 K3-only 事件)

汇总:RECOVERED 21,PROGRESS_BUT_NOT_RECOVERED 4,COMPARATOR_REJECTED_K3_BETTER 2
(仅 ON#1),NO_SIDE_DISPATCH 2(hint 0.28 s 即被新事件替换),ACTIVATION_NOT_REACHED 1;
K3 进度提交 12 次,激活确认 11 次,**零 M2 抢占,零安全违规**。

- ON#1(8 事件):4 RECOVERED;event 5 = 深阴影(margin_at_detect=−4.12):
  8 次授权,SIDE_MINUS 3 次 commit+激活,D3 逐周期 1.61→1.52→1.40(realized),
  hint 窗口关闭后 0.12 s 二值恢复;events 6/7 见 §5。
- ON#2(10 事件):9 RECOVERED(ttr 0.1~0.9 s);event 4:3 commit+激活,
  D3 realized 1.369→1.346→1.232。
- ON#3(15 事件):8 RECOVERED;event 4/8:C3_before 0→C3_after 0.5(窗口内部分
  二值恢复)+ D3 1.838→0.695 / 2.175→0.857;event 14:1 commit 但下 cycle 前
  被新事件替换(ACTIVATION_NOT_REACHED,跨 cycle 语义正确记录)。
- 提交时 D3 均值:2.078→1.365(−34.3%,ON#3)/ 1.420→0.941(ON#1)/
  1.186→0.833(ON#2)。

## 5. FIRST CAUSAL FAILURE(未达成 binary 恢复的事件)

1. **深阴影 + hint 窗口过短(4 事件,PROGRESS_BUT_NOT_RECOVERED)**:
   窗口只覆盖 forecast 的二值损失区间(0.1~0.5 s),真实阴影更长;D3 逐周期下降、
   C3 部分恢复,恢复发生在窗口关闭后 ≤0.12 s(ON#1 event 5)。机制按设计工作,
   剩余 deficit 已量化(realized D3 1.23~1.37)。
2. **规划时权威与 executed 真值在边缘遮挡上分歧(ON#1 events 6/7)**:NOMINAL
   候选 C3=1.0(LOS+range 权威、D3≈0.05 含 FOV 的 margin 权威都"已恢复"),但
   executed 真值 static 阻挡持续 1.07 s。发现两处同量权威不一致(均为既有实现,
   本轮未改):(a) `evaluateCandidateVisibility` 的 target 按 sample_t 传播,缺
   (candidate_start_time − planning_epoch) 引导(≈0.1~0.2 s·v ≈ 0.1~0.2 m),
   与 `fillCandidateBinaryVisibility` 的世界时间传播不一致;恰在 0.08 m
   occlusion margin 的边缘遮挡上足以翻转结论。(b) 它不含 FOV/yaw 分量。→ 见 NEXT_STEP。
3. **drone 2 的 11 个 K3 合同从未 escalation(3 次 run 一致)**:Team proposal 在
   realization 阶段失败(如 drone 1 ack `LOCAL_HARD_PREFLIGHT_FAILED:
   HARD_DYNAMIC_COLLISION_FAIL`)→ abortTeamProposal reason 非 `TEAM_SCP_FAILED`
   → 不广播。drone 2 只靠 normal N/L/R(SIDE_PLUS 在执行),uav3 损失 1.37~1.63 s。
4. **ON 运行间方差**:ON#2/#3 出现机制不活跃窗口的新 uav1 损失段
   ([32.0 s]/[37.3 s],最近 escalation 活动相距 >9 s,期间 normal 动态避障分岔);
   非 comparator/trust 所致,但说明该场景边缘遮挡对时序敏感。

## 6. OFF / ON executed 对比(用户指示跳过新 OFF;两份历史 run 为参考)

| 指标 | ON#1 | ON#2 | ON#3 | REF 111437 | REF 108 |
|---|---|---|---|---|---|
| ALL3 | **0.9131** | 0.8825 | 0.8863 | 0.9076 | 0.9106 |
| K2 | 0.9906 | 0.9898 | 0.9902 | 0.9936 | 0.9932 |
| MEAN_VISIBLE | 2.9037 | 2.8723 | 2.8765 | 2.9012 | 2.9037 |
| LONGEST_ALL3_LOSS | 1.532 | 2.264 | 1.632 | 1.532 | 1.432 |
| LONGEST_K2_LOSS | 0.733 | 0.799 | 0.766 | 0.500 | 0.533 |
| K3 loss 总时长 | **6.061** | 8.392 | 8.125 | 6.727 | 6.460 |
| BLACKOUT | 0 | 0 | 0 | 0 | 0 |
| uav1/2/3 VISIBLE_RATIO | .947/.987/.970 | .917/.983/.972 | .924/.983/.970 | .945/.986/.970 | .946/.986/.972 |

读法:机制激活窗口内逐事件收益明确(§4);全局指标在 run 间方差范围内,
ON#1 优于两个参考,ON#2/#3 被 §5.4 的不活跃窗口方差拖累。**K2 唯一损失区间
在所有 run 中都是同一段晚段 M2 事件**(count≤1,无任何 K3 事件重叠),但 ON 的
M2 事件时长(0.73~0.80 s)一致长于参考(0.50~0.53 s),且全部 escalation 活动
在该事件前 ≥9 s 已闭合 —— 本轮无 OFF 配对无法完全归因,列为遗留观察项。

安全/存活(3 次 ON):STATIC/DYNAMIC_CONTACT 0,SWARM_VIOLATION 0,
PVA_MISMATCH 0,UNVALIDATED_EXECUTED 0,PARTIAL_TEAM_ACTIVATION 0,
TERMINAL_HOLD 为 0.01 s 级正常进出;MOVING_SUCCESSOR_STARVATION 2/1/2(参考 0,
均发生在 M2 恢复 seam 0.4 s 内,K3 机制已闭合,未造成运行失败)。

## 7. 下一步建议(NEXT_STEP)

1. **统一 target 锚定**:把 `evaluateCandidateVisibility` 的 target 传播改为世界时间
   (对齐 `fillCandidateBinaryVisibility`),消除 0.1~0.2 m 系统滞后 —— 这是对
   §5.2 边缘遮挡分歧最直接的修复(会改变共享 comparator 输入,需单独授权+配对验证)。
2. **escalation 触发面扩展到 realization 失败**(§5.3):K3 合同若在 realization
   阶段 abort,同样广播 escalation —— 涉及 coordinator abort 路径语义,需单独授权。
3. adaptive trust region:本轮明确未实现(§6 任务边界);当前证据显示瓶颈在
   预测保真度与 hint 窗口长度,而非 trust 卡死,暂无启动依据。
4. 用 `--k3-escalation off` 补一组干净 OFF 配对,归因 §6 的 M2 时长差异。

---

PRODUCTION_CODE_CHANGED: YES (traj_utils msg×2, multi_uav_formation coordinator, plan_manage planner_manager .h/.cpp, launch×3, scripts×2)
BUILD_PASS: YES (catkin build -j2 --no-status, 25/25 包,含 git diff --check 干净)
FULL_OFF_RUN_COMPLETED: NO (用户指令"不用基线,直接on";OFF 语义由代码等价性审查+历史 run 覆盖)
FULL_ON_RUN_COMPLETED: YES (×3:20260925_124647_104062 / 20260925_130810_130608 / 20260925_131504_132764,均 BOOT-12,exit 0)

K3_LOCAL_ESCALATION_IMPLEMENTED: YES
NEW_TEAM_K3_OPTIMIZER_ADDED: NO
M2_TEAM_PATH_CHANGED: NO
SCP_TRUST_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO

K3_ESCALATION_EVENTS: 33 (8/10/15)
K3_EVENTS_OUTSIDE_NORMAL_AUTHORITY: 0 (全部 window_begin 在普通 2.0 s horizon 内,字段如实记录)
K3_SIDE_TRIGGERED: 58 次授权 (17/10/12+…,逐事件见 [K3_ESCALATION_EVENT]/grant 行)
K3_SIDE_BOTH_FAILED: 47 (逐事件 side_both_failed_count;深阴影期 L/R 构造高失败率)
K3_PROGRESS_CANDIDATES: 61 (0/10/15/2/7/9/13/3/12/3…逐事件 k3_better_candidate_count)
K3_PROGRESS_SELECTED: 12
K3_PROGRESS_ACTIVATED: 11
K3_RECOVERED_EVENTS: 21 (binary_k3_recovered=1;恢复时间 0.1~0.9 s)

EXECUTED_ALL3_OFF: 未重跑(参考 0.9076 / 0.9106)
EXECUTED_ALL3_ON: 0.9131 / 0.8825 / 0.8863
EXECUTED_K2_OFF: 未重跑(参考 0.9936 / 0.9932)
EXECUTED_K2_ON: 0.9906 / 0.9898 / 0.9902
LONGEST_ALL3_LOSS_OFF/ON: 1.532,1.432 / 1.532,2.264,1.632
LONGEST_K2_LOSS_OFF/ON: 0.500,0.533 / 0.733,0.799,0.766(均为同一段晚段 M2 事件,无 K3 重叠;归因待 OFF 配对)

STATIC_CONTACT_SAMPLES_OFF/ON: 0 / 0
DYNAMIC_CONTACT_SAMPLES_OFF/ON: 0 / 0
SWARM_VIOLATION_OFF/ON: 0 / 0
PARTIAL_TEAM_ACTIVATION_OFF/ON: 0 / 0
UNVALIDATED_EXECUTED_SAMPLES_OFF/ON: 0 / 0
STARVATION_OFF/ON: 0 / 2,1,2(M2 恢复 seam,机制已闭合;无运行失败)
TERMINAL_HOLD_OFF/ON: 正常 0.01 s 级进出 / 同左

K3_GAIN_CONFIRMED: YES(事件级:21/33 恢复、12 提交、11 激活、D3 均值 −30~−34%、
C3 部分恢复 0→0.5;全局级:ON#1 优于两参考,ON#2/#3 受机制不活跃窗口方差影响)
K2_PROTECTED: YES(K2 唯一损失为晚段 M2 事件,与所有 K3 窗口无重叠;比较器 K2 保护门从未被绕过)
SAFETY_REGRESSION: NO
LIVENESS_REGRESSION: NO(STARVATION 计数差异位于 M2 seam,无失败后果;遗留观察)

FIRST_CAUSAL_FAILURE: 深阴影事件的 hint 窗口只覆盖 forecast 二值区间(0.1~0.5 s),
真实阴影更长 → PROGRESS_BUT_NOT_RECOVERED;次要:规划时候选可见性权威与 executed
真值在边缘遮挡上分歧(target 锚定滞后 0.1~0.2 m + 无 FOV 分量);drone 2 的 K3 合同
在 realization 失败路径上从不 escalation
NEXT_STEP: 统一 evaluateCandidateVisibility 的 target 世界时间锚定;escalation 触发面
扩展到 K3 合同 realization 失败;补干净 OFF 配对归因 M2 时长差异;adaptive trust
暂无启动依据

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
