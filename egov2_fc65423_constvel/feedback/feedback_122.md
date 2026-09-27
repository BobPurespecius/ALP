# Feedback 122 — Local 三接口收敛阶段结果

日期：2026-09-26。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。

## 结论

**本轮没有完成要求的整体收敛。** 已修正连续静态 corridor 检查的两个数学错误，并把 A* 路径、细分后 SFC 和 MINCO piece 的部分接口绑定起来；25/25 包构建通过，最终 package warning 为 0。要求的离线合同 A–F 尚未全部通过，因此按“所有离线检查通过后才允许 FULL”的门槛，**未启动 FULL，也没有新的运行指标**。以下源码处于未完成集成验证状态，不应当成已验收的生产版本。

## 已改动的接口

- [poly_traj_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp#L252)：连续 checker 使用正确 piece 起点和完整五项 quartic 导数；按导数临界点隔离区间实根，核查端点、降阶、重根和根残差。同一 evaluator 供 SCP 与 [retime 检查](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L2303)使用。STATIC 时间由 `piece_id` 和局部 `u∈[0,1]` 映射；LOS 世界时间未改。OSQP slack wrapper 先检查 success、维度和 finite，再切取 step。
- [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L11021)：静态 A* base 不再由动态 seed 的 hard collision/有效性决定；raw route 检查 anchor 和相邻边，并把真实 anchor 纳入最终路线。N A* 的 target-facing 半空间硬约束关闭，仍可作为搜索偏好。
- 同一文件的 [SFC builder](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L11540)：明确为定高 2.5D extrusion，发出与 voxel 碰撞审计相同的 6-plane 体；失败 leaf 导致整段失败并清空 planes。成功细分点按时序插入最终 guide，静态 plane 绑定对应 MINCO piece。未覆盖 fast path 仍依赖原静态轨迹检查。新增闭合遥测 `ASTAR_STATIC_SAFE = SFC_BUILD_SUCCESS + SFC_BUILD_FAILED + SFC_NOT_REQUIRED`，但尚无新 run 核查计数。
- [SCP 时间行](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp#L3486)对 STATIC plane 使用 piece-local `u`；动态障碍 world-time 导数不再误用 `ds_local/dT`；tracking angular-band 的显式时间导数已补。LOS-only sampled residual 不再作为静态 corridor violation。
- [CandidateResult](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h#L204)增加 `checked_until`；已预检候选提交时不再再次 reanchor/增 revision；需要安全 successor 时，K3 质量 veto 不能排空首个 hard-safe 候选。普通 swarm hard checker 删除隐式 `NON_WORSENING` 放行，使用原物理 clearance。

## 确定性验证

`git diff --check` 通过。`catkin build -j2 --no-status` 最终 **25/25 packages succeeded，package warnings 0**。第一次完整编译曾有一个新增 unused-variable warning，已去除并重新编译通过；更早一次 catkin 会话以 143 中断、没有编译器结果。仅保留当前的有效最终构建结论。

[连续 checker 用例](artifacts/feedback122_continuous_contract.cpp)直接调用当前 `traj_opt` 库，覆盖 `f(s)=s`、`0.5-s`、`0.4-s+s^5`、非均匀多 piece、裁剪窗口、降阶和重根；[结果](artifacts/feedback122_continuous_result.txt)为 `PASS`。该用例不能证明 SFC 地图扫描、SCP 整体 Jacobian、PVA timing 或执行 identity。临时测试二进制已删除；没有新增仿真数据。

## 离线门槛仍不通过的具体原因

| 门槛 | 当前缺口 | 判定 |
|---|---|---|
| A 连续 checker | 列出的解析反例通过；未做对生产 SCP 所有 active-set 与最终 validator 调用的独立性质测试 | 局部通过 |
| B SFC | 没有实际地图下失败 leaf、斜段、voxel corner、相邻 overlap、anchor/grid edge 的完整 production 几何测试；新定高 extrusion 会拒绝斜段，实际供给率未知 | **未通过** |
| C timing/Jacobian | [A* seed timing](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L12076)仍是一次 λ 缩放，固定 PVA 后的真实 residual 可不合格；fresh/N 的多次 timing 分支仍在。尚无非零 head V/A、非均匀 T、动态/target/bearing 全链 finite difference | **未通过** |
| D authority | [seed tube](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp#L3395)仍生成硬行并参加 final gate；旧 Team 外部 polynomial callback 仍存在；可选 polish 的 incumbent 保留尚未验证 | **未通过** |
| E execution identity | batch 后二次 reanchor 已对 prepared candidate 禁用，但没有同 ID/revision/polynomial/activation 从 preflight、quality、commit 到 executor ACK 的端到端证据 | **未通过** |
| F checked_until | 已将本机 coverage 截到预测 authority horizon；缺 map/peer/prediction 数据的 UNKNOWN 语义与各 checker 精确末端尚未统一 | **未通过** |

另外，[旧 `TRUE_CONSTRAINT_INFEASIBILITY` 状态](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp#L4418)仍未重命名和分解；同一 SCP 内的 seed tube、status、numerical restoration 也未完成。不能把 Feedback120 的 100/117 旧 SFC 构造失败说成已修复或仅换了 label；本轮没有新 run 可量化其变化。

## 对要求的十个问题

1. route→SFC→piece：**部分**。静态 plane 已显式 piece 绑定；完整三维/相邻覆盖实证缺失。
2. 旧 100/117 build failure：**未证明**分类闭合后的真实原因分布；新计数等待运行。
3. continuous checker：列出的一组解析用例通过；整体数学正确性仍须扩大离线覆盖。
4. fixed-PVA timing/Jacobian：**否**；initializer 与全链 finite difference 未闭合。
5. seed tube/topology/LOS/K3 quality 退出 safety：**否**；仅完成部分清理。
6. hard-safe incumbent 抵抗 optional quality failure：K3 必需 successor 有保留路径；LOS polish 和 Team 入口未验证，**未证明全域成立**。
7. final preflight 与 commit payload：prepared Local 路径不再二次 reanchor；未做 payload identity 端到端证明。
8. checked_until：已修明显的整条尾部过度认证赋值；UNKNOWN 和精确 predicate 时域未闭合。
9. fallback/retry：**未实质合并完成**。
10. 失败类型：仅部分区分；没有新 run，不能给 GEOMETRY/NUMERICAL/BUDGET/PHYSICAL_SAFETY/TASK_QUALITY 新分布。

## 状态字段

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES  # final 25/25, package warnings 0
OFFLINE_CONTRACT_CHECKS_PASS: NO

ROUTE_SFC_PIECE_CONTRACT: PARTIAL
CONTINUOUS_CHECKER_CORRECT: PARTIAL  # specified analytic cases pass; full system proof absent
PVA_AWARE_TIMING_ACTIVE: NO
TIME_JACOBIANS_AUDITED: PARTIAL
SEED_TUBE_HARD_AUTHORITY_REMOVED: NO

LOS_HARD_AUTHORITY_PRESENT: UNRESOLVED
TOPOLOGY_HARD_AUTHORITY_PRESENT: YES  # remaining seed tube / Team side rows
QUALITY_CAN_KILL_SAFE_INCUMBENT: UNRESOLVED

FINAL_PREFLIGHT_AUTHORITATIVE: PARTIAL
COMMIT_PAYLOAD_EQUALS_VALIDATED_PAYLOAD: NOT_PROVEN
CHECKED_UNTIL_TRUTHFUL: PARTIAL

LEGACY_FALLBACKS_REMOVED_OR_MERGED: NO
LEGACY_TEAM_EXTERNAL_POLYNOMIAL_AUTHORITY: PRESENT

ASTAR_ROUTE_VALID: N/A  # no new production run
SFC_BUILD_SUCCESS: N/A
SFC_BUILD_FAILED: N/A
SFC_NOT_REQUIRED: N/A
PIECE_BINDING_VALID: N/A
SCP_ATTEMPT: N/A
SCP_SUCCESS: N/A
QP_INFEASIBLE: N/A
NUMERICAL_FAILURE: N/A
MODEL_MISMATCH: N/A
BUDGET_STOP: N/A

EXECUTED_ALL3: N/A
EXECUTED_K2: N/A
ALL3_LOSS_TOTAL: N/A
K2_LOSS_TOTAL: N/A
LONGEST_ALL3_LOSS: N/A
LONGEST_K2_LOSS: N/A
BLACKOUT: N/A
STATIC_CONTACT: N/A
DYNAMIC_CONTACT: N/A
SWARM_VIOLATION: N/A
PVA_MISMATCH: N/A
UNVALIDATED_EXECUTED: N/A
PARTIAL_TEAM_ACTIVATION: N/A
STARVATION: N/A
TERMINAL_HOLD: N/A

ARCHITECTURE_HEALTH: NOT_YET_VERIFIED
FIRST_REMAINING_CAUSAL_FAILURE: full route/SFC/timing contract has no offline proof; fixed-PVA initializer and hard seed tube remain
NEXT_STEP: complete remaining interfaces and offline A-F checks before the single FULL ON

FULL_RUN_COUNT_THIS_TASK: 0  # required offline gate did not pass
REPEATED_FULL_SIMULATION_USED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
