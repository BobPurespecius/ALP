# 阶段 G(2026-09-25 授权):topology escalation。
# K3/M2 repair 的 PT-SCP 判 TRUE_CONSTRAINT_INFEASIBLE 后,coordinator 把
# (target, window) 广播回 planner:同一窗口存在 raw static LOS witness 时
# 授予该窗口的 SIDE dispatch —— same-topology 已证明不可行,由既有 N/L/R
# 解。这是"forecast raw witness + Team 失败证据"的显式 authority 授予,
# 不改变 soft-visibility ≠ topology 的其余边界。
Header header
uint64 escalation_id
uint64 handoff_contract_id
uint64 task_reference_generation
int16 target_drone
uint8 repair_k
time window_begin_world
time window_end_world
float64 required_margin
string reason
# K3 Local escalation binding(2026-09-25 第二轮):事件级绑定字段。
# target/window 之外,额外绑定 detect 时刻的阻塞运动类别(ContinuousVisibilitySample::Limiter),
# target 的 margin 与二值 K2/K3 占比,以及产生该评估的 target prediction revision。
# Planner 侧授予仍要求"当前快照重新扫描出的 raw witness 与窗口相交":
# revision 只用于审计对齐,不能用陈旧 descriptor 直接授权。
uint8 detect_limiter
float64 margin_at_detect
float64 k2_at_detect
float64 k3_at_detect
uint64 target_prediction_revision
