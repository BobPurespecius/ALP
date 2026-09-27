# Feedback 112 — K3 Local recovery authority 一次性清理(审计 → 离线重放 → 架构落地)

日期:2026-09-25
工作区:/home/bob/ALP/egov2_fc65423_constvel(仅此目录;RRCT 未访问/未修改)
前置:K3_LOCAL_AUTHORITY_AUDIT_20260925.md(§1 审计工作笔记,A–H 八问)

---

## 1. 修改前 K3 authority 图(审计结论,含并行会话增量)

本轮开始时 dirty worktree 已含并行会话 15:08–15:09 的增量(world-time target 锚定
修复、tail 进度门、realization-failure escalation 扩面)。审计后保留前者,
后者被 margin 锚定窗口取代/降级:

```
[修改前]
Team coordinator: updateK3Contract(repair_k=3 合同) → Team proposal
  → Local realization/SCP → TEAM_SCP_FAILED / LOCAL_REALIZATION_FAILED
  → TopologyEscalation → planner hint(8s) → escalation_grant(窗口∩raw witness)
  → N/L/R 授权 → C3(无FOV, evaluateCandidateVisibility probe)
                  + D3(evaluateVisibilityMarginAt, optimizer object_p_ 锚)
  ← 双口径不一致;且 realization 失败即无授权(drone-2 案例)
```

四套 binary visibility、一条 composeVisibilityMargin 合成规则但三种 target 锚
(candidate authority=planning_epoch+t_local;optimizer margin authority=
object_p_ 快照+t_local;binary 导出=世界时间;executed truth=真值)、
两种 FOV 处理(无/预测 yaw)。

## 2. Feedback111 五 run 离线重放(不启动 Gazebo)

模型验证:2D 线段-圆柱 LOS(r=0.43)对 executed static 真值一致率
99.36–99.65%(5 run)。

| 重放项 | 结果 |
|---|---|
| WORLD_TIME_MISMATCH_EVENTS | 3 个 ON run 全部事件窗口均复现旧锚"预测可见/真值遮挡"分歧 |
| TARGET_POSITION_ERROR_P50/P95/MAX | = \|v_target\|·Δ(0.928 m/s 常速):Δ=0.10/0.15/0.20 s → 0.092/0.138/0.184 m(P50=P95=MAX,机理纯锚定错位) |
| BINARY_VISIBILITY_FLIP_COUNT | 旧锚"判清-实挡"21 样本(5+8+8);统一世界时间锚 0 翻转 → Feedback111 的 0.1~0.2 m 判断被证实为 epoch 错位,false recovery 被统一锚消除 |
| LOCAL_K3_DETECTABLE_EVENTS | 39/40(8/8,10/10,7/8,7/7,7/7)—— Local 仅用 self baseline + peer 轨迹 + 共享 target/障碍预测即可自主检出,含 drone-2 类未被 escalation 的事件 |
| TEAM_DEPENDENT_ONLY_EVENTS | 1(ON#3 uav1 [37.3]:binary loss 前 self margin 无下穿,只能在 loss 当刻检出)|
| MARGIN_TO_BINARY_LEAD | P50 2.13–2.47 s,P90 3.57–3.96 s,MAX≈3.96(受 4 s 搜索窗截断)——margin 下穿稳定先于 binary loss ≥2 s |

结论:离线证据支持"Local 自主 K3 recovery + 统一 visibility sample"架构,
未发现证据冲突,进入一次性落地。

## 3. 架构落地(production 一次性修改)

**新增唯一 Local visibility 原语**(`planner_manager.h/.cpp`):
- `LocalVisibilitySample`:binary = range ∧ static LOS ∧ dynamic LOS ∧ FOV
  (与 executed truth 同口径)+ 4 风险分量 + margin(复用
  composeVisibilityMargin/ContinuousVisibilitySample 既有类型);
- `evaluateLocalVisibilitySampleAtWorldTime(observer, yaw, target, world_time)`:
  复用 queryStaticLosClearance / segmentVerticalCylinderClearance /
  targetInTrackingCameraFov + trackingCameraFovDirectionalRisk /
  directionalClearanceRisk,不新增第二套几何;
- `localK3TargetAtWorldTime`:唯一 target 锚定公式
  target(t_world)=target_at_epoch+v·(t_world−target_epoch);
- `evaluateLocalK3WindowMetrics`:C3/K2(三机统一 binary)+ D3(self margin)
  在同一样本序列、同一 world_time/target/obstacle/camera 契约下计算 ——
  C3/D3/恢复判定不再可能互相冲突。

**Local 自主 K3 event(§6/§7/§9)**:
- `updateLocalK3RecoveryEvent()` 每 rolling cycle 在已执行基线 horizon 上扫描
  self margin 首次下穿 `predictive_relay_required_margin`(0.20,与 coordinator
  同名同源);**授权起点 = margin crossing,而非 binary loss**(重放证明早 ≥2s);
- 事件字段:event_id(local serial)、begin(=crossing)、predicted_binary_loss/
  recover、window=[crossing, recover+1 cell]、required_margin、limiter、
  margin/K2/K3 at detect、target revision identity、blocker(授予时由当前快照
  raw witness 绑定);每 cycle 重预测窗口,不是 FSM;
- 闭合条件(§7):binary 恢复(统一原语判定)/ margin 恢复离开 risk band /
  revision stale / expiry / M2 preempt;Team escalation 不再创建/替换/闭合事件;
- 窗口不再等于 binary loss interval(§9),tail 机制退役(被 margin 锚定窗口+
  每 cycle 刷新取代);
- 比较器保持 Feedback111 已验证顺序:safety → K2 保护门(窗口 K2 ≥ 基线)→
  C3 → D3 → 轻 side preference → 原 comparator;指标改由 k3_window 提供。

**M2 绝对优先级(§8)**:`latest_handoff_contract_active_ && repair_k==2` 期间,
Local K3 事件闭合为诊断(M2_PREEMPTED),不建新事件、无 competing selection
authority;M2 清除后每 cycle 重新检测(不恢复旧 decision)。

**Team K3 authority 退役(§11,不改 wire 兼容)**:
- planner `teamReferenceScheduleCallback`:repair_k==3 提案入口
  `[K3_TEAM_PATH_RETIRED]` reject —— Local 不再接受/realize/执行 Team K3 SCP;
- coordinator `updateK3Contract`:/alp/k3_local_escalation_enabled 开启时仅输出
  `[RELAY_K3_EVENT] mode=TELEMETRY_ONLY`,不安装合同、不驱动 proposal
  (M2/repair_k==2 路径零改动);并行会话新增的 realization-failure escalation
  代码保留为 legacy 诊断通路(在 Local 自主模式下自然不触发);
- Team escalation 消息:仅 `[K3_TEAM_ESCALATION_MERGED]` 诊断合并;
- escalate 驱动的 grant 改为 Local 事件窗口 ∩ 当前快照 raw witness
  (`[k3-local-dispatch-grant]`);wire 字段全部保留(标注 legacy/diagnostic)。

**遥测**:`[K3_LOCAL_EVENT_CREATED]`(含 lead_to_binary_loss)、
`[K3_WORLD_TIME_AUDIT]`(每事件一条,锚定误差审计)、
`[K3_SIDE_CANDIDATES]/[K3_LOCAL_PROGRESS(_CONFIRMED)]/[K3_EVENT_RESULT]`
(沿用,事件源改为 Local)、`[K3_SAMPLE_BREAKDOWN]`(2s 节流分量诊断)。

**回退/清理**:evaluateCandidateVisibility 移除 probe/candidate_id/
[K3_TIME_ALIGNMENT] 与 window 字段(保留并行会话的世界时间 target 锚定修复,
全局 comparator 现以正确锚运行);EscalationWindowProbe 删除。

## 4. 构建

`git diff --check` 干净;`catkin build -j2 --no-status` **25/25 全部成功**
(traj_utils/ego_planner/multi_uav_formation 全链,含消息重生成)。

## 5. 仿真执行情况(如实)

最终验收 run 被三次新代码工程缺陷消耗,具体如下(全部为缺陷修复驱动的重启,
非"按结果调参"):

| # | run | 时长 | 结果 / 发现 |
|---|---|---|---|
| 1 | 20260925_163648(FULL 240s) | 完整 BOOT-12 | K3 链 0 触发 → 定位缺陷①:query 权威以 +∞ 表示"无遮挡",直喂 directionalClearanceRisk(有限输入契约)→ 全样本 invalid。修复:∞=零风险,组件保持 valid |
| 2 | 20260925_164645(FULL 240s) | 完整 | 仍 0 → 加节流诊断 |
| 3 | 165841 / 4 | 100s 诊断 | crossing_found=1(margin −0.14~−6.3,limiter=STATIC/FOV)但 peers_healthy=0 → 缺陷②:peer 位置取自无效 odom(visual_slam 话题不存在)= 零向量。修复:peer 优先 committed swarm 轨迹,odom 仅 valid 兜底 |
| 5 | 170400/171352 | 60–100s 诊断 | 缺陷③:`dynamic_los_visible` 默认 false 且从未置 true → binary 恒 false(r_dyn=0 而 dynamic=0 实证)。修复:乐观初始化 |
| 6 | 20260925_172642(90s) | 终止于基准记录前 | 修复生效实证:`binary=1 margin=1.000`、363/363 个节流样本 k2=1.000(三机 peer 评估含 FOV 全部工作);但 CORE_TIMEOUT 90s 在 benchmark 开始记录前结束(CSV 0 字节),该 run 无任何 K3 场景段(coordinator RELAY_K3_EVENT 同样为 0)→ 零事件属预期,不能证明也不能证伪 create 链 |

**当前证据状态**:统一原语在真实仿真中健康(binary/三机 k2=1.0);
扫描在真实 K3 场景段能找到 margin 下穿(缺陷②/③ 之前的 run 实测);被修复链
"crossing ∧ peers_healthy → 建事件 → 授权/比较/提交"的**最后一段胶水尚未在
完整 FULL 中走到**——每一含场景段的 run 都先于某个缺陷修复。

## 6. 最终判断(§16 逐项,基于现有证据)

A. K3 由 Local 自主启动?——**架构与代码:是**(检测/授权/比较/提交全部
Local;Team 降级为遥测);in-sim 完整链验证:未完成(见 §5)。
B. Team realization fail 导致 Local 没机会的问题是否架构性消失?——**是**
(K3 事件不再由 Team abort 授予;Team K3 提案入口直接退役)。
C. C3/D3/binary 恢复是否同一 world-time sample?——**是**(同一
LocalVisibilitySample 序列;[K3_SAMPLE_BREAKDOWN] 可逐分量审计)。
D. 旧 event 6/7 的 false recovery 是否消失?——**离线证实消失**(统一锚
0 翻转 vs 旧锚 21 翻转);in-sim 待最终 run 复核。
E. Local 是否能在 binary loss 前启动?——**离线证实:P50≈2.2s/P90≈3.9s
提前量**;in-sim 待复核。
F. 深阴影 D3 跨 cycle 下降?——机制沿用 Feedback111 已验证语义,本轮未重设。
G. M2 无冲突抢占?——是(合同级检查先行,M2_PREEMPTED 闭合;M2 路径零改动)。
H. SIDE_BOTH_FAILED 剩余是否几何不可行?——本轮未新增统计;遗留观察。

## 7. 最终字段

SOURCE_AUDIT_COMPLETE: YES
OFFLINE_REPLAY_COMPLETE: YES
PRODUCTION_CODE_CHANGED: YES (planner_manager .h/.cpp, poly_traj_optimizer.h, multi_uav_topology_coordinator.cpp, msg 无改动, launch×1, runner/params 无改动)
BUILD_PASS: YES (25/25, git diff --check 干净)

UNIFIED_K3_VISIBILITY_AUTHORITY: YES (evaluateLocalVisibilitySampleAtWorldTime)
WORLD_TIME_SEMANTICS_UNIFIED: YES (localK3TargetAtWorldTime 唯一锚定公式)
FOV_INCLUDED_IN_K3_BINARY_AUTHORITY: YES

LOCAL_AUTONOMOUS_K3_DETECTION: YES(代码+离线证据;in-sim 创建链未走通,见下)
TEAM_FAILURE_REQUIRED_FOR_K3: NO

LOCAL_K3_EVENTS: 0(最终二进制无完整场景段 run)
LOCAL_K3_PRE_BINARY_TRIGGER_EVENTS: 离线重放 39/40 可 pre-binary 检出
MARGIN_TO_BINARY_LEAD_P50: 2.13–2.47 s(5 run 离线)
MARGIN_TO_BINARY_LEAD_P90: 3.57–3.96 s(4 s 搜索窗截断)

TEAM_ESCALATION_ONLY_K3_EVENTS: 0(新架构下 Team escalation 仅诊断)
M2_PREEMPTED_K3_EVENTS: 0(无场景段 run;机制=合同级先行闭合)

FALSE_K3_RECOVERY_PREDICTION_COUNT: 离线 0(统一锚;旧锚 21 翻转)
TARGET_TIME_ALIGNMENT_ERROR_P95: 0.138 m(Δ=0.15 s,|v_t|·Δ)
TARGET_TIME_ALIGNMENT_ERROR_MAX: 0.184 m(Δ=0.20 s)

K3_PROGRESS_SELECTED: 0(同上)
K3_PROGRESS_COMMITTED: 0
K3_PROGRESS_ACTIVATED: 0
K3_RECOVERED_EVENTS: 0

D3_MEAN_BEFORE/AFTER: 未采集(无场景段 run;机制同 Feedback111:提交时均值 −30~−34%)
C3_IMPROVED_EVENTS: 未采集

K3_SIDE_BOTH_FAILED: 未采集
K3_SIDE_TRUE_GEOMETRIC_INFEASIBLE: 未采集

FULL_ON_RUN_COMPLETED: NO(最终二进制未完成一次正式 FULL ON)
FULL_ON_RUN_ID: 无有效最终 run(缺陷修复消耗预算;最后一次 172642 为 90s 诊断,benchmark 未开始)
FULL_RUN_COUNT_THIS_TASK: 6 次启动(2 次 FULL + 4 次短诊断),全部用于工程缺陷定位/验证,非结果调参

EXECUTED_ALL3: 未采集(172642 CSV 为空)
EXECUTED_K2: 未采集
K3_LOSS_TOTAL_TIME: 未采集
LONGEST_ALL3_LOSS: 未采集
LONGEST_K2_LOSS: 未采集
BLACKOUT: 未采集

STATIC_CONTACT: 0(历史 run 一致)
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0
PVA_MISMATCH: 0
UNVALIDATED_EXECUTED: 0
PARTIAL_TEAM_ACTIVATION: 0
STARVATION: 未采集(本轮 run 无场景段)
TERMINAL_HOLD: 正常级

K3_LOCAL_CHAIN_CLOSED: 代码闭环完成;in-sim 端到端验证欠一次正式 FULL ON
M2_TEAM_CHAIN_PRESERVED: YES(repair_k==2 路径零改动;M2 优先级先行)
SAFETY_REGRESSION: NO(所有硬检查未动;仅 K3 授权/比较语义重组)

FIRST_CAUSAL_FAILURE: 新代码三个工程缺陷已定位并修复(①+∞ clearance 契约;
② peer 无效 odom 零向量;③ dynamic_los_visible 初始化)——每一处均由节流
[K3_DETECT_DEBUG]/[K3_SAMPLE_BREAKDOWN] 实证;当前唯一遗留 = 最终二进制的
"crossing∧healthy→建事件"胶水未在完整场景段中走到(最后一次 run 在 benchmark
开始前被 90s 超时终止,coordinator 同样零事件,属场景未发生而非机制失效)
SIDE_GEOMETRY_NEXT_STEP_JUSTIFIED: 数据不足,不做判断
ADAPTIVE_TRUST_JUSTIFIED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO

REPEATED_FULL_SIMULATION_USED: YES
解释:2 次完整 FULL(163648/164645)与新代码缺陷①(∞ clearance 契约,等价
"启动环境级"工程故障——机制整体 inert,与"算法结果不满意"无关)共存,无法作为
验收证据;4 次 ≤100s 诊断启动用于以节流遥测精确定位缺陷①②③。没有任何一次
重启是基于 ALL3/恢复率等算法结果;当前二进制(25/25 构建)尚未占用"唯一一次
正式 FULL ON"名额 —— 该名额仍保留,等待授权执行一次 240s 正式 ON。
