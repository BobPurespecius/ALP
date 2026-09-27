# K3 Local Authority 审计工作笔记(2026-09-25,只读,feedback_112 前置)

## A. binary visibility 定义:4 套

| # | 实现 | range | static LOS | dynamic LOS | FOV/yaw | target 时间基准 |
|---|---|---|---|---|---|---|
| 1 | `evaluateCandidateVisibility`(planner,全局 comparator 输入) | visibility_min/max_target_distance | queryStaticLosClearance ≥ visibility_occlusion_margin_ | segmentIntersectsVerticalCylinder | **无** | planning_epoch + t_local(缺 candidate_start 引导) |
| 2 | `fillCandidateBinaryVisibility`(planner,导出用) | 同上(经 FOV 检查内的 range) | 同上 | 同上 | targetInTrackingCameraFov + advanceTargetFacingYaw | target_state_time,世界时间正确 |
| 3 | `visibilitySampleAt`(optimizer,margin/binary 权威) | camera.min/max_range | static_los_geometry_ + directionalClearanceRisk | segmentVerticalCylinderClearanceGradient | trackingCameraFovDirectionalRisk,visibility_yaw_ 状态推进 | **object_p_ 快照 epoch + t_local(未世界时间校正)**;动态障碍用 activation+t(正确) |
| 4 | executed truth(benchmark visibility.csv) | 真值 | 真值 | 真值 | 真值(actual yaw) | 真值 |

## B. continuous margin 定义:1 条合成规则,≥3 个 target 锚

- 合成规则唯一:`composeVisibilityMargin`(tracking_visibility_geometry.h),margin=1−2·max(4 分量 risk)。
- 但喂进它的 target 时间基准不同:optimizer `visibilitySampleAt` 用 object_p_ 快照+t_local;
  coordinator `continuousVisibility(...world_time)` 世界时间正确;Feedback111 的 K3 probe
  D3 走 evaluateVisibilityMarginAt(=1)而 C3 走 evaluateCandidateVisibility(锚定不同、无
  FOV)→ **同一 K3 事件内 C3/D3 口径不一致(Feedback111 §5.2 的根因)**。

## C. 见 A 表;关键错位:三套 planning 权威的 target 锚分别差
(candidate_start − planning_epoch)、(activation − object_snapshot_epoch)、0。

## D. 统一 Local K3 visibility authority = planner 侧新原语
`evaluateLocalVisibilitySampleAtWorldTime`:复用 queryStaticLosClearance /
obj_predictor_->evaluateConstVel + segmentVerticalCylinderClearance(Gradient) /
targetInTrackingCameraFov + trackingCameraFovDirectionalRisk / directionalClearanceRisk /
composeVisibilityMargin / advanceTargetFacingYaw;target = target_position +
v·(t_world − planning_epoch);动态障碍 evaluateConstVel(id, t_world);peer =
executionAt(t_world);self yaw 实际状态按同一推进模型。binary = range ∧ static ∧
dynamic ∧ FOV;risks+margin 同一样本内合成。K3 检测/C3/D3/恢复判定全部走它。

## E. K3 event 为何仍依赖 Team escalation
历史路径:K3 relay 生于 Team(updateK3Contract→proposal→runTeamContractSCP→
TEAM_SCP_FAILED→TopologyEscalation→planner escalation_grant)。Feedback111 保留该依赖
(grant 需 hint)。drone-2 案例证明依赖不可靠:realization 失败(HARD_DYNAMIC_
COLLISION_FAIL ack)在 SCP 之前 abort → 永不 escalation → Local 无授权。

## F. Local 自主判断"我是唯一弱通道"所需信息 Local 全部已有
swarm_traj.executionAt(t_world)(peer 未来)+ 自身 candidate/baseline + target 预测
(planning_epoch)+ obj_predictor → evaluateCandidateVisibility 已在本地算三机可见性。
Team 不提供任何 Local 拿不到的几何信息(K3Assessment 只是把三方 forecast 融合)。

## G. M2 Team repair 真正入口 & K3 让位
入口:coordinator updateRelayContract(repair_k==2)→ TeamReferenceSchedule →
planner teamReferenceScheduleCallback(已有 latest_handoff_contract_active_ /
latest_handoff_repair_k_)。K3 让位:M2 active 时 Local K3 event 降级为诊断,
comparator 不激活,不产生 K3-specific selection authority;M2 清除后重新检测
(每 cycle 重跑,不恢复旧 decision)。

## H. 8s hint / K3EscalationEventState 取舍
保留(新语义):window、required_margin、恢复判定簿记、cycle/进度计数、side
preference、pending activation check。
退役为 legacy/diagnostic(Team 路径遗留):escalation_id/contract_id 作为**授权依据**
(降级为标签)、8s received_wall 新鲜度(被 Local event expiry 取代)、
target_prediction_revision 作为门(降级为审计字段)。

## 团队 K3 路径的 planner 侧清单(退役范围)
- teamReferenceScheduleCallback 中 repair_k==3 的接受/SCP 尝试(3492/3573/3694/3701/3717)
  → flag ON 且 Local 自主时在入口 reject,M2(repair_k==2)路径零改动。
- escalation_grant(8101 区域)→ 由 Local event 窗口取代;Team escalation 仅 merge/诊断。
- evaluateCandidateVisibility 的 Feedback111 probe → 回退原签名,K3 窗口指标改走统一原语
  (历史全局 comparator 逐位还原)。
