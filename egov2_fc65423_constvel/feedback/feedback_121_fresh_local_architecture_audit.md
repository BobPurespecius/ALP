# Feedback 121 — Local planner 独立只读架构审计

审计日期：2026-09-26。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。

**结论：目标架构基本合理，但当前 Local 还不能称为健康、职责闭合的实现。问题主要是静态几何到 polynomial 的合同断裂、时间/验证器实现错误，以及初值与 SCP 可行性恢复不匹配；现有证据不支持“主要是正常物理无路”或“主要是 OSQP 不行”。** 应重构现有接口，保留 A*、Local-SFC、MINCO、SCP、OSQP 的基本分工，不增加 planner、状态机或 fallback 链。

最重要的新核查结果：

1. 历史 run 的完整分节点 ROS 日志中，117 次 A* handoff 有 **100 次 corridor 构造失败、16 次有静态平面、1 次不需要平面**。累计 `CORRIDOR_BUILD_FAILED=0` 是计数分支错误，不能据它认定前端已打通。第一处可定位的 A* 链断点在 corridor build，早于第一例 SCP failure。
2. 连续 checker 同时存在 **piece 时间原点提前更新**和**五次函数导数缺四次项**两个确定错误；既可能假阳性，也可能假阴性。修正时间原点一个点不够。
3. 当前 SFC 不是已证明的三维 inflated-static-free 子集；还有细分部分成功被当作整体成功、缺 piece 绑定、三套时间映射不一致的问题。需要重整**现有** Local-SFC 合同。
4. 固定真实 head/tail PVA 后重新生成 MINCO，不满足简单统一时间缩放规律。完整 117 条 timing 中，97 条 `j_after>20`；这证明初始化质量问题，不能证明无可行轨迹。
5. 197 条 SIDE `EXECUTION_DEADLINE` **全部在目标停止之后**；195 条优化器耗时不足 1 ms。它们不能解释运动阶段 corridor SCP 的 0/16。
6. LOS 和 topology authority 尚未彻底清理：硬 seed tube、N 的 A* 半空间、条件性 LOS residual gate、质量 polish 覆盖 hard-safe 结果、K3 的 K2 veto，以及旧 Team proposal 接口仍需处理。

## 0. 方法、边界与证据索引

### 0.1 审计口径

只读设计、源码、dirty diff、已有日志/CSV/报告；用内存中的 Python 做文本统计与数学核算。没有编译、运行生产程序、运行仿真、执行历史分析器、修改配置或参数。唯一交付写入是本报告。没有访问 `/home/bob/RRCT`，没有执行 git reset/checkout/restore/clean。

HEAD 为 `8cef24143c8a8a23bae77dd0e6cf5395c7dc6d2c`，但**审计对象是当前工作树，而非 HEAD**。工作区内 tracked diff 为 21 个文件、24,932 insertions、977 deletions；A*、SCP adapter、FSM/executor、visibility/Team 等重要文件还有 untracked 内容。故仅审阅 `git diff` 不足以审计当前系统。审计前后对 `ros_ws/src` 下非符号链接普通文件及启动脚本共 905 个文件做 SHA-256 比较，未发生变化；这不是历史 run 二进制与 dirty source 的逐字节对应证明。

证据等级：

- **SOURCE / HIGH**：当前源码可直接证明的控制流或数学结论。
- **RUN / HIGH**：指定旧 run 的原始记录，保留节点、时间与阶段；日志中的“safe”仅代表该 checker 当时输出。
- **INFERENCE / MEDIUM**：机制与运行现象一致，但缺精确 candidate/PVA/QP replay。
- **CANNOT_CONCLUDE**：不能由现有材料推出物理无解、碰撞消失、某 bug 贡献百分比或当前 binary 行为。

### 0.2 文件缩写

以下 `文件缩写:行号` 均指当前工作树，表中的一行可以覆盖该函数内列出的同职责子判据。源码行号不是历史报告的旧行号。

`P = ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner`。

| 缩写 | 文件 |
|---|---|
| PM | [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp) |
| PMH | [planner_manager.h](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h) |
| PO | [poly_traj_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) |
| POH | [poly_traj_optimizer.h](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h) |
| QP | [scp_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/scp_optimizer.cpp) |
| MINCO | [poly_traj_utils.hpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/include/optimizer/poly_traj_utils.hpp) |
| AS | [dyn_a_star.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/path_searching/src/dyn_a_star.cpp) |
| GM | [grid_map.h](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_env/include/plan_env/grid_map.h) |
| BS | [local_sfc_boundary_scan.h](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/local_sfc_boundary_scan.h) |
| EC | [local_execution_contract.h](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/local_execution_contract.h) |
| FSM | [ego_replan_fsm.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp) |
| TS | [traj_server.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp) |
| VG | [tracking_visibility_geometry.h](../ros_ws/src/multi_uav_formation/include/multi_uav_formation/tracking_visibility_geometry.h) |
| TC | [multi_uav_topology_coordinator.cpp](../ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp) |
| TV | [realized_team_validator.cpp](../ros_ws/src/multi_uav_formation/src/realized_team_validator.cpp) |

### 0.3 设计与历史材料如何使用

已阅读/定位原始 N/L/R、安全类别、弧长 timing、SIDE/A*/SFC、Projected MINCO、弹性跟踪、visibility-first、K3/Team relay，以及 Feedback115–120。重点材料：

- [THREE_CLASS_AND_ARCLENGTH_TIME_INIT_REPORT](../THREE_CLASS_AND_ARCLENGTH_TIME_INIT_REPORT.md)、[ALP_SFC_MINCO_SCP_SOURCE_CHAIN_AUDIT](../ALP_SFC_MINCO_SCP_SOURCE_CHAIN_AUDIT_20260902.txt)、[Projected_MINCO_Implementation_Report](../Projected_MINCO_Implementation_Report.md)。当前生产主链不是这些文档中每种算法的叠加；PO:5481/5485 的 LBFGS hook 为 NULL，不能把旧 projected MINCO 描述当作正在生效的投影器。
- [ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT](../ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md)、[AUDIT_PREDICTIVE_VISIBILITY_RELAY](../AUDIT_PREDICTIVE_VISIBILITY_RELAY_20260922.md)、[K3_LOCAL_AUTHORITY_AUDIT](../K3_LOCAL_AUTHORITY_AUDIT_20260925.md)。历史 frozen elastic risk profile 已不再是当前生产恢复机制；当前应读 PO:7779 的 band/bearing tracking。
- Feedback115 的固定 PVA/观察平面冲突，仅作旧运行事实；116/117 的“LOS 全域 soft”必须逐出口复核；118 的架构判断不直接继承；119 的“bounded corridor 保证子集、一次链”必须检查几何与调用次数；120 的 checker/timing 结论用独立代数和完整日志重新核查。

特别纠正：旧报告的 `LOS_HARD_REJECT_COUNT=0` 不是结构性无 LOS gate 的证明；`RAW_PATH_STATIC_SAFE` 不是边级连续自由证书；`TRUE_CONSTRAINT_INFEASIBILITY` 不是原始飞行问题无解证书；`safety_validated=True` 不是独立执行真值。

## 1. 十项目标架构的独立评价

| # | DESIGN_VERDICT | 理由与最小修正 |
|---|---|---|
| 1 N/L/R 只表达局部 topology intent | SOUND_BUT_INCOMPLETE | 低维离散选择适合滚动规划；左右应绑定一个 blocker/frame/window，只指导 seed/优先级。它不证明全局 homotopy，也不能用 seed 距离 tube/整路 side sign 把偏好升级为飞行安全。 |
| 2 A* 只负责静态几何 repair | SOUND | 静态图不具备动态时间/观察模型。输入资格也须是静态条件；动态碰撞 seed 仍可进入静态修复，之后在 polynomial 层求时空可行性。 |
| 3 static-safe route 后构造 static-free Local-SFC | SOUND_BUT_INCOMPLETE | 必须证明 route 边自由、corridor 子集自由、衔接、覆盖范围与 piece 归属。仅点自由、平面数或采样 rectangle 不能兑现该承诺。 |
| 4 corridor 内 MINCO-SCP-OSQP 生成可执行轨迹 | SOUND_BUT_INCOMPLETE | MINCO 是参数化，SCP 是局部非凸近似，OSQP 解该轮凸问题；三者不自动保证原问题可行。需与固定 PVA 一致的初值、可诊断的内部可行性恢复及最终非线性验证。 |
| 5 static/dynamic/swarm/PVA/collision corridor 为 hard safety | SOUND_BUT_INCOMPLETE | 物理碰撞与 PVA 连续性应硬。**选定的内近似 corridor 不是整个自由空间**：违反它只能说明该表示下未获证，不能叫“物理碰撞/全局无路”。所有 hard 阈值、几何与验证时域要统一。 |
| 6 LOS/FOV/range/C3/D3/J_vis 为观察质量 | SOUND | 与机体碰撞分离。任务可判“本次观察升级无收益”，但不能撤销当前安全轨迹或阻止必需安全 successor；质量失败不得改写 safety_class。 |
| 7 observation-side 作 soft/semi-hard，不 hard reject | SOUND_BUT_INCOMPLETE | 非负无上界 slack 可以表达渐进恢复。还必须保证可选质量求解失败、超时、或扰坏安全时保留同一已验证 revision；仅把 return false 改 true 不足以隔离 authority。 |
| 8 Local K3 / Team M2 | SOUND_BUT_INCOMPLETE | 单机恢复可在其他两机健康时局部做；M2 是至少两机可见的协同任务合同，不是物理安全。应明确抢占、冻结共同世界时刻与真值归属，且合同不可保证任何遮挡布局都能恢复。 |
| 9 Local 唯一 executable polynomial producer | SOUND | Team 发布任务参考、选择和协同时序；Local 实现、最终预检、返回精确 payload。Team 可重建 payload 做评价，不应另造可执行多项式并绕过 Local 生产链。 |
| 10 SIDE 可渐进恢复 | SOUND | 单周期完全二值恢复不是一般可达条件。以 hard-safe 集中的 C3/D3 等进度排序，跨周期以 executed binary truth 判恢复；不能让质量基线缺失关闭唯一安全后继。 |

当前问题不是必须推翻这十点，而是补足第 3/4/5/7 点的合同，并把不符合它们的旧出口删除或降回正确职责。

### 1.1 弹性跟踪、合围和 deep-risk

PO:7820–7938 的当前结构是半径/高度带宽恢复、弱方位恢复、冲突时间窗放宽角带、可选前缀增强和合围 gap 软代价。它不再维护历史文档中的整套 frozen elastic risk profile。PO:6647–6862 的 live directional visibility 继续包含 static LOS、dynamic LOS、FOV 风险；VG:199 的 deep continuation 避免进入遮挡深处后梯度完全饱和。这些思想可以保留。

需要修正而不是调权重：PO:7866–7910 的 angular_band 显式随 t 变化，代价 `w*max(0,|e_phi|-band(t))²` 在激活区含时间导数 `-2*w*e_eff*band'(t)`；PO:7907 后的 gradt/grad_prev_t 只累加空间梯度与相对速度，未补这一项。固定 p/target、但 t 落在威胁窗斜坡时，代价变化而这部分时间梯度为零。前缀 boost 的分段边界也要有一致的积分/导数语义。该项是 **SOURCE / HIGH 的公式不一致，具体导致多少 LBFGS/SCP 失败 CANNOT_CONCLUDE**。不要用调 bearing weight 掩盖导数问题。

合围角度/间隙与连续 visibility 只应影响候选排序和软目标。二值 visibility 由 range、static LOS、dynamic LOS、camera FOV 联合决定；VG:45–113 的连续 margin `1-2*max(risk)` 不能当作二值可见真值。

## 2. 从源码重建真实 pipeline

**当前是混合链，所有候选并不统一经过 SFC→SCP。** N 和无静态平面的 SIDE 主要走 LBFGS；带 STATIC_COLLISION_CORRIDOR 的 SIDE 强制走 hard SCP；Team 的 Local realization 又可调用同一 SCP。见 PO:5394–5428、PM:4202、PO:1913。

| 层/源码 | INPUT | OUTPUT | AUTHORITY | CAN_REJECT? / WHY | WHO_RECHECKS_IT? |
|---|---|---|---|---|---|
| coverage / head，PM:1302、2036 | active schedule、odom PVA、时钟 | batch deadline、activation seed、起点 PVA | 生命周期/计算预算 | 是；时间耗尽、无合法 handoff | prepareLocalHandoff、publish、TS |
| risk / conflict，PM:8934、8998、9755 | N/已有轨迹、静态/动态/swarm/LOS、目标快照 | ConflictDescriptor、body/LOS/frame/window | 探索触发和任务意图 | 能压制 SIDE 生成；不应凭质量否定 safety | 候选时空 checker、K3 |
| N/LEFT/RIGHT，PM:9850、10135 | descriptor、N seed、预算 | 0–3 个候选及 id | 有界局部探索 | dispatch/预算可不给某侧机会；不是“该侧物理无路” | 后续逐候选 |
| SIDE guide，PM:10357–10845 | head PVA、目标、offset、frame | 有限偏移 seed、A* repair base | 初值/意图 | 当前因数值、窗口、动态资格死亡 | A*、optimizer、preflight |
| static feasibility，PM:10493、12383 | seed、inflated map | OBS_FREE/FINISH/ERR、native rays | 静态检查及 native 约束生成 | ERR 当前会杀；ERR 包含 A* 搜索失败，不仅状态损坏 | SFC/SCP、PM static checker |
| local A*，PM:11105；AS:174–449 | grid、静态 anchors | raw grid path | 静态几何搜索 | 搜索失败/范围/timeout；N 还有偏好半空间 | 点检查、简化边采样、SFC |
| simplification，PM:11278–11425 | raw path | 简化点、长度/曲率、T | 几何表示/数值初始化 | 点/长度/维度不合法可失败 | SFC、native checker、preflight |
| Local-SFC，PM:11548–11825 | 简化 segment、map、seed T | 4-plane XY boxes、active seconds | 静态内近似，当前未证明 | 构造失败会杀；不能当全空间无路 | SCP 连续/采样、retime SFC、static map |
| MINCO seed，PM:11845–11990 | guide P、长度/曲率 T、固定 head/tail PVA | 分段 quintic 初值 | 数值初值 | 当前若 native ERR/维度/某些初值门会杀 | SCP/LBFGS、最终 hard checker |
| SCP/OSQP 或 LBFGS，PO:2045/5363 | P/T、seed tube、native rays、SFC、动态、软质量 | 优化后的 MINCO / 失败状态 | 局部数值求解 | 能杀；数值失败≠物理无解 | 内部非线性检查、PM classify、preflight |
| LOS soft-QP，PO:444、5427、5571 | 优化后 P、固定 T、观察侧平面 | 改动 P 的新 polynomial | 观察引导 | 函数显式总返回 true；但改坏 hard safety/预算异常可间接杀 | PM 分类/preflight；原安全 revision 未保留 |
| dynamic/swarm/PVA 分类，PM:8934 | 最终 polynomial、epoch、peer/prediction | safety_class、静态/动态标志 | 多份 feasibility authority | 是；且 swarm trigger margin 混入 hard class | batch preflight、commit、publish |
| batch hard preflight，PM:1595–1640 | 重新生成的 head、冻结 activation | prepared revision、hard-safe candidate | 执行前安全 | 是；SFC、PVA、物理检查；另有短静止门 | commit 又 reanchor、publish |
| C3/D3/visibility，PM:1658–1830 | hard-safe prepared revisions、共同窗口 | 排序/选择、K3 进度 | 观察任务质量 | 排序合理；当前 baseline/K2 可把所有候选排空 | commit 后仅部分 metadata 更新，未重排 K3 |
| commit，PM:6462、2637 | selected candidate | 再 reanchor 的新 revision、trajectory message | 最终执行授权 | 是；前缀、PVA、动态/static/swarm、Team certificate | TS 验 payload/lineage/PVA；不重做几何 |
| activation，TS:795、1018、1530 | 消息、prepared transaction、执行 lineage、odom | 真正 active polynomial / reject | 执行连续性与身份 | 是；身份、先后依赖、PVA/live mismatch | 下一 rolling 的生命周期验证 |

正常 physical hard reject 必须发生在**具体 polynomial、具体 activation、具体快照、明确验证时域**上。搜索空集、单轮 QP primal infeasible、某 seed 数值坏，只能描述该层结果。

## 3. A–K 设计实现一致性

| 项目 | 当前结论 | 源码及解释 |
|---|---|---|
| A topology 是否 hard reject | 仍存在 | SIDE A* 整路 sign gate 已退出（PM:11217、12105）；但 N A* 强半空间、SCP seed tube、非 capture 的 SAME_BLOCKER invalid、Team side rows 仍在。 |
| B static repair 被动态/quality 阻塞 | YES | PM:10478 的 `trial_dynamic_valid` 要求 !hard_collision 才保留 A* base；PM:10845 依赖 base。换 path-frame 的支路 PM:10533 却只要求 risk.valid，入口语义不一致。动态可由后续 P/T 修复，不能决定静态搜索资格。 |
| C LOS 隐藏 reject | NOT CLEAN | retime/main连续checker已 skip LOS；但 PO:2462 的 sampled lambda 未 skip，LOS-only SCP 可从 PO:5110 hard fail。postsolve LOS 改 P/耗预算还能间接丢掉安全结果。 |
| D static corridor 只表示 static safety | 设计意图是，实际合同混杂 | source enum 区分良好；但与 LOS 共用容器、与 seed tube 共称 corridor、XY box 无三维证明，且 Team side 行一起影响可行性。 |
| E path→corridor→MINCO 清晰合同 | NO | SFC 细分点未变为 polynomial piece；window 映射不同；native checker 仍重跑 A*、造 rays；N 又是另一条 A*→MINCO→check 链。 |
| F seed 当 executable | 部分修正，仍混乱 | A* cut-corner seed 不再要求立即 static-safe，可进 SCP；最终执行仍需预检。独立验证过的 initializer 被执行本身合理，不能称“未验证 seed 直接飞”。问题在 preopt 数值门/ERR 仍杀初值，且 frontend telemetry 把 dynamic_valid 强置 true（PM:12220）。 |
| G active interval / 三种时间 | 不一致 | PO:2383 ratio、3168 absolute active samples、3398 ratio window、continuous raw seconds；PM reanchor/SFC 检查另有时标，见第 5 节。 |
| H quintic 连续验证 | WRONG | 原点更新错误、导数少一阶；系数反转正确但根函数名与实现不符。 |
| I 固定 PVA 的 P/T timing | 不符合 scaling 假设 | PM:11949–11978 不改真实端点导数却用纯 dilation 比例推 seed 时间；完整 run 的 jerk 仍超限。 |
| J preflight 重复/阈值 | YES | classify→batch→commit→publish→suffix 重复；部分重检因 revision 改变必要，部分完全相同；SFC、dynamic、swarm、PVAJ 用不同阈值/几何。 |
| K 身份/revision/world anchor | 不统一 | PM:10143 生成 SIDE id，但 PM:10731、12526 记录 nominal id，MINCO_INPUT 甚至可写 NOMINAL；SCP 自有 id，A* 有 NA。batch 后 commit 再改 polynomial；不能按 kind/最近日志拼“同一 candidate”。 |

还有两个可直接定位的同候选数据不一致：PM:10413 的 side_init_candidate_traj 在 duration/frame fallback 前复制，PM:10588/10595/10689 仍拿该旧 polynomial 求观察侧到达时刻/guide；PM:10533 虽重算 fallback_risk，PM:10785 却继续用之前的 trial_risk 评价新 seed。它们不会自动证明产生了不安全执行，但证明了一个 candidate 的几何与风险证据未必属于同一 revision。应在现有候选值对象替换时原子刷新其派生量，不新增又一份缓存。

PM:11860–11879 还将实际 anchor_start 与栅格 simplified_path 混入 guide；append_guide 只按严格递增时间追加，最后 anchor_goal 与最后 path vertex 同时刻时被忽略。如果 raw grid endpoint 与实际 anchor 不同，这条连接边/端点对应就不是声明的原样修复。N initializer 会显式替换 raw.front/back（PM:8338），SIDE 应采用同一个明确的连接合同；该分支的实际端点误差未在本 run 保存，影响量 CANNOT_CONCLUDE。

### 3.1 A* 角色的具体边界

SIDE A* 当前 clearVisibilityCostContext、使用普通静态三参数搜索（PM:11105）；这是应保留的方向。N 的 static initializer（PM:8312）却把 target_side_preference 传入 `enforce_secondary_halfspace`；AS:338–344 丢弃另一侧节点，AS:378 后还有 outward 代价，并非纯 tie-break。搜索失败才跑 unbiased A*，若有偏路径搜索成功、后续 MINCO 失败，不会再去另一侧。最小修正是 topology 只作搜索次序/软偏好，不能切掉静态自由空间。

AS:356 检查相邻节点的占据，而非整条 26 邻接边的 voxel supercover；PM:11079 的 raw admission 也是点检查。简化默认保留 current+1，较远 shortcut 才另作采样验证；所以“117 raw safe”最多证明这些点检查通过，不能证明 117 条连续折线全自由。端点被栅格化/调整（AS:174、449）也需要显式连接边合同。AS:423 修改 priority_queue 内节点 fScore 而无 decrease-key/reinsert，最短路最优性也不应直接宣称；它不是本次首要运行证据。

A* 的 pool/range/搜索时限失败必须独立报 `SEARCH_LIMIT`/`BUDGET`，不能均落到 `NO_STATIC_FEASIBLE_SIDE`。不需要 space-time A*。

## 4. continuousLocalSfcMaxViolation 独立数学核查

源码：PO:233–379；Piece 系数约定见 MINCO 的 getCoeffMat/getPos。

设一段 quintic 为 `p_i(s)=Σ(k=0..5) a_k s^k`，段始时 `S_i=Σ(j<i) T_j`；平面 margin 为 `f(s)=n·(p_i(s)-q)-d`。真实检查域是：

`lo=max(active_start,S_i), hi=min(active_end,S_i+T_i)`，若 hi≥lo，则 `s∈[lo-S_i,hi-S_i]`。

| 检查项 | 判定 |
|---|---|
| coefficient order | getCoeffMat 为 descending；`coeff.col(5-k)` 转 ascending 正确，Horner 方向正确。 |
| 常数项/normal | 对 n 正规化后，常数减 n·q 和 clearance 的表达正确，前提是平面已有同一距离单位。 |
| piece 起止/累积 | **错误**：PO:333 先 `t_piece_start=t_piece_end`，PO:353 才减 t_piece_start；局部域被平移成通常的 [-T,0]。外层累计能走到下一 piece，不代表本 piece 评估原点正确。 |
| trajectory/world/local | static plane 是 trajectory seconds，不应直接混 world time；本函数的主要原点 bug 是 piece end 与 piece start 混淆。world-anchored LOS 在此 skip 正确。另有 P/T 改变后 plane seconds 没更新的问题。 |
| derivative coefficients | **错误**：传入 Vector4d(c1,2c2,3c3,4c4)，漏 `5c5`。五次函数的一阶导应有五个系数，最高四次。 |
| root solver | **错误**：名为 quarticRealRoots，实收 4 系数，3 个复根迭代，求的是 cubic/降阶；固定 64 次迭代，没有可用的残差/收敛证书。 |
| interval clipping | lo/hi 与 active interval 的交集形式正确；用错误起点转 local 后失效。时长修改后的 interval 本身还可能陈旧。 |
| endpoints | 有端点检查，但检查的是错误 local 域的端点，不能补救；真实内部极值也会遗漏。 |

三个无需构建/仿真的解析反例（单段 T=1，合法 quintic 的低阶特例）：

1. `f(s)=s`：真实 [0,1] 最小值 0；现算法在 [-1,0] 最小值 -1，制造 violation=1。
2. `f(s)=0.5-s`：真实最小值 -0.5；现算法最小值 0.5，漏掉 violation=0.5。
3. 单独修正时间原点后，`f(s)=0.4-s+s^5` 的端点均为 0.4；真导数 `-1+5s^4=0` 在 s≈0.668740 有极小值约 -0.134992。现四系数导数退化为常数 -1，仍漏 violation。

**对 active-set/QP 的影响必须准确表述：** 错 checker 会加入错误的“最坏时刻”或漏加真实极值，改变 active-set 及终止判断。返回时刻仍可能落在合法 trajectory interval，后续行又用真实 getPos/Jacobian 生成；不能把 checker 报出的 34 m 假 violation 直接说成被原样塞进了 QP RHS。它足以污染 SCP，但不能单独证明每个 QP primal infeasible 都由此产生。

最小修正合同：一个可信的五次平面极值 evaluator；先保存 piece_start、正确形成 quartic 导数、在 clipped local interval 求全部实根并检端点，处理降阶/近重根/残差，最后再更新累计时间。可复用现有 root_finder 工具，不需要第二套 corridor checker。修正后再审计 active-set，不用错误数值调 trust/OSQP。

## 5. Local-SFC：几何、覆盖和时间合同

**SFC_DESIGN_STATUS = NEEDS_REDESIGN。限定为重整现有 Local-SFC 的表示与接口，不增加第二套 SFC，不换 FIRI/IRIS。** “沿静态路径分段构造凸内近似”保留；当前的“4-plane bounded segment + 时间重叠即证安全”不足以兑现设计。

### 5.1 几何构造不是子集证明

PM:11548 将 tangent.z 清零；三个扫描站和 whole-box audit 均位于 pa.z。左右墙和前后 caps 的法向量 z 均为 0，故区域沿 z 无界。GM:375–386 的地图是三维、出界/顶棚可 occupied。即使当前多为定高圆柱森林，也没有源码合同把所有 polynomial z 限定到同一已验证高度层。因此一般地不能证明 `corridor⊂inflated static free space`。

BS:56 用最后 free/第一次 occupied 之间的中点作为边界；它可能仍落在 occupied voxel。PM:11580 的 whole-box audit 又向内缩 0.25 resolution、以 0.5 resolution 间距点采样，实际发出的墙和 cap 没同步缩入审核区域；边条、旋转 box 与 voxel 的相交区域不具完整覆盖证明。即使把采样步长变小，也不等于严格 voxel 子集证明。

最小几何合同应使用**相同 inflated map 的 voxel/边界表示**，对实际发出的凸体与 occupied voxel 的交集作保守认证，所有 caps、墙和 z 域都在同一证明范围。若工程明确限定平面飞行，可采用有明确高度范围的 extrusion 合同；否则在现有多-plane 结构内加入必要的 z 界。4-plane 足够表达二维 rectangle，但不足以一般表达三维安全域；不需要为了“更高级”换生成器。

### 5.2 split、overlap、path 对齐

- 只对扫到墙的 segment 及邻段建 corridor，open-field 段无平面（PM:11640）。这是局部 corridor，可保留，但“未覆盖段由谁验证”必须显式声明，不能声称整轨迹在完整 SFC 中。
- split 触发是 try_build_segment 失败、depth<2、长度>2r（PM:11770）。它有界，但没有区分窄度、cap 碰撞、无有效 z、窗口问题等子因；“达到细分上限”不是无路证明。
- **确定实现 bug**：PM:11747 的 segment_built 初值 false，任一 leaf 成功即 true；另一个 leaf 失败且不可再 split 时没有把整体置 false。最后只检查是否至少成功一次，可能接受缺覆盖的 segment。应要求全部需要的叶子成功，并原子提交该 segment 的平面。
- cap 扩 r，在理想同平面、正宽度、公共端点情况下可让相邻 rectangle 包含公共点；这不是足够宽的交集、PVA 通行性或全 active-time 交集非空的证明。PM:11823 的 `adjacent_overlap=1` 和 `corridor_subset_of_static_free=1` 是打印常数。
- 新 midpoint 写进 corridor_points，MINCO guide 仍由 simplified_path 构造（PM:11870）。POH:51–55 明确 source_segment/source_guide_index 仅诊断。故平面不拥有可靠的 polynomial piece 归属。
- caps/小短段/弯折的多个时间窗同时激活，可能形成过窄甚至错误组合的内近似；另一方面 z 无界、扫描中点和未审核边条又可能过宽。问题不是统一“扩大/缩小 corridor”能解决。

### 5.3 时间合同目前至少有三种映射

1. 前端给 seed absolute trajectory seconds，按几何长度/曲率 T 加 overlap（PM:11731）；一次统一 dilation 同比缩放静态窗（PM:11964）。这一特定统一 dilation 的 interval 同比变换本身合理，但新 polynomial 形状因固定 PVA 改变，仍需验证。
2. SCP 存 `plane.active_start/end / initial_total_duration`，硬行及 sampled violation 用 ratio×当前总时长（PO:2375、2480、3398）。非均匀优化各 piece T 时，全局总时长比例不能保持某个 segment/piece 归属。
3. `sfc_active_times` 是绝对秒样本（PO:3168），T 改变后不按 piece 映射，只在新 ratio window 内过滤；continuous evaluator 又使用原 plane.active_start/end。PO:9086 之外没有把优化后的时间写回 candidate_local_sfc_planes_。PM:2291 的 retime checker 再用 source 当前轨迹总长作分母，原 seed-plane seconds 可能已不匹配 source。

PO:3433 的 `ds/dT_j=global_alpha-I(j<i)` 适用于**固定全局 normalized progress**样本；当前 active samples 却是缓存 absolute t。数值在固定 t 评估、导数按 progress 移动，线性化不是所检查函数的导数。

最小修正：静态 plane 绑定 piece/segment 及 piece-local normalized interval，统一映射到累计 T 后的 trajectory time；split 同步更新绑定；active-set 保存这样的坐标，不能保存无归属秒数。world-anchored LOS 独立保存 world interval，只在质量采样时用 activation 换算。所有 checker、SCP、handoff 使用同一映射接口。

### 5.4 continuous verification 与 active set

需要连续 polynomial containment：仅 25 点、0.03 s 或 voxel 点采样无法排除段中间越界。quintic 对固定平面有低阶极值问题，适合精确做，不应删除此层。

PO:3168 的端点/中点+最坏点增量约束方向合理；当前 checker 错、time coordinate 错、每 plane 最多 12 点且新增点下轮才入行。PO:5060 的提前结束主要看 seed tube/dynamics，没有覆盖全部 Local-SFC/dynamic/native-static 可行性，可能在新约束生效前退出。应以共同非线性 hard residual 和可信连续 checker 定义终止；达到上限报 NUMERICAL_LIMIT，不能宣称物理无解。

## 6. A*-repair MINCO timing 与 SCP/OSQP

### 6.1 初始 T 与固定边界

PM:11425–11494 用 segment 长度、相邻 vertex speed、转角曲率给 T；speed cap 为 0.8*vmax，曲率速度约 `sqrt(0.8*amax/kappa)`，另有 0.1 speed floor 和 1e-3 piece Tmin。路径简化限制 shortcut 跨 raw 点数量、保留转角点，可能产生许多短 piece。这个几何分配没有同时满足真实 head V/A、tail V/A、jerk 和短段间动态过渡。

PM:11922 把原 seed head/tail PVA 交给 MINCO，先 generate，再用 `λ=max(1,v/vlim,sqrt(a/alim),cbrt(j/jlim))` 统一乘 durations，再**保持这些 PVA** generate 一次。真实边界不能随意缩放，这一要求是正确的；把纯时间缩放当成该再生成过程的精确定律是错误的。

若 `q(t)=p(t/λ)`，则 q 的边界 V/A 分别为原来的 1/λ、1/λ²；当前 q 要保留真实 V/A，所以通常 q≠p(t/λ)。以单段 quintic 为例（MINCO:1239 的系数）：

`j(0) = (3*aT-9*a0)/T - (36*v0+24*vT)/T² + 60*(pT-p0)/T³`。

固定边界下，jerk 混合 T^-1/T^-2/T^-3 项；不能只按 λ^-3 估算，更不能保证任意放大 λ 后空间仍在原 corridor。多 piece 的耦合不消除该问题。

### 6.2 运行证据与归因

完整 rosout 中 N=117：λ 均>1，P50≈2.858，max≈5.352；j_before P50≈466.991/max3065.865，j_after P50≈21.119/max166.280；**97/117 高于标称 20**。duration P50≈4.687→12.413 s。PM dynamics checker 有 feasibility tolerance，且 jerk 是离散采样，故“j_after>20”既不等于 97 次最终 hard reject，也不是连续峰值上界。

第一例 corridor SCP 的 seed T 是 `[2.91061,0.308177,0.102726,0.102726,0.102726,0.308177,2.91061]`，最大/最小约 28.3。中间短段与很长前后延续、固定边界、局部 caps 一起造成尺度与可行性恢复困难。该事件 j≈299.711→20.965，SCP 仍失败。

| 分类 | 判断 |
|---|---|
| timing initialization | 主要问题；未满足固定 PVA/jerk 的完整边界条件，纯 dilation 假设不成立。 |
| boundary condition | 真正 PVA 必须固定；问题是时标/guide 没尊重边界，而不是应把真实 head V/A 改小。 |
| path segmentation | 重要共同因素；过密 waypoint、短 T、corridor 与 MINCO piece 不同步。 |
| MINCO representation | 未见“quintic MINCO 本质不适合”的证据；它准确生成给定插值与 PVA 的平滑多项式，不自动保持折线同侧或 corridor。 |
| SCP formulation | 共同因素；一步 hard 线性化、seed tube、试探点须立即 static-free、错误时间 Jacobian 会妨碍不可行 seed 的恢复。 |
| OSQP | 无证据应换 solver；它解的是上游构造的问题。max_iter 或线性化 primal infeasible 不等于真实运动不可行。 |

最小修改方向：一个共享的 PVA-aware timing initializer，依据实际固定边界生成的 polynomial 残差来分配 T，并限制无意义的短 piece/过度插值；几何点和 corridor piece 一起决定分段。失败输出 NUMERICAL_INITIALIZATION_FAILED，允许同一 SCP 的有界内部可行性恢复。不能再叠“更大 λ”“第二轮放大”“再加 initializer fallback”。本轮不确定新 T 的参数值，也不调参数。

### 6.3 SCP 还有四项独立问题

- **seed-preservation tube 是额外硬拓扑约束**：PO:2414、3319–3368、5110 限制离参考 seed 的距离；零偏移处以 radius/sqrt(3) 的 box 行替代 ball，约束更保守。这个 tube 既不是 static SFC，也不是迭代信任域，可能把绕障空间排除。第一例 QP 的 337 行里 seed tube 144 行、Local-SFC 65、dynamic 24、dynamics 104。应删其终局 hard safety 身份，保留必要软偏好；迭代 trust region 继续有独立数值职责。
- **不可行 seed 的恢复合同矛盾**：前端允许 seed cut corner，PO:4660 的 trial_static_free 却要求接受的 trial 一步已 free；加上线性化 hard 行和有限 trust，很容易无第一步。可以在同一个 SCP 内使用可诊断、单调降低 hard residual 的内部恢复；只有最终满足零物理违反的轨迹能执行。内部恢复不等于放宽最终安全。
- **dynamic world-time Jacobian 错用 piece-local 导数**：PO:3780 设 `ds/dT=alpha-I(previous pieces)`，PO:3800 给障碍运动项也乘 ds/dT。障碍查询的时间是 `activation+t`，其导数应为 dt/dT，不含减去 piece 累计时间的项；若 t 固定于 prediction horizon 采样，dt/dT 甚至为 0。不能把自身 polynomial 的 local time 与障碍 world time 当同一变量。这是 SOURCE/HIGH；运行影响量尚无逐 Jacobian replay。
- **“TRUE”诊断越权**：PO:4198–4333 在主 QP primal_infeasible 后做 no-P/no-T/no-PT probes，三个 `!success` 就报 TRUE_CONSTRAINT_INFEASIBILITY；probe timeout、max_iter、数值失败也能进入 !success。即使 probes 都给出可信 infeasible，也只证明本轮线性化且含人工 tube/side 行的 QP，不证明原非凸问题或自由空间无解。

QP adapter（QP:55–201）基本职责合理：输入检查、CSC、正对角 Hessian、OSQP 状态与 residual 输出；solved_inaccurate 必须继续做 nonlinear validation。max_iter=2000、eps=1e-3、scaling=10 不能据此次审计直接调大/调小。每次 setup/cleanup 新 workspace，即使 warm_start=1 也没有跨调用复用解；time_limit 受编译条件 PROFILING 控制，当前二进制实际保证 CANNOT_CONCLUDE。

**另一个确定实现 bug 在 wrapper，而非 OSQP**：PO:439 无条件 `result.step=qp2.step.head(n)`；QP:192 只在 solved/solved_inaccurate 时填 step。失败/预算直接返回时 step 可为空，head(n) 越界，debug 可断言、release 行为未定义。现有 run 的 LOS QP 记录未证明触发过该崩溃。应先验证 success/维度再取切片。

## 7. LOS、K3/Team 与最终 preflight 的职责边界

### 7.1 已清理的 LOS 路径与仍存在的路径

应认可已经完成的修正：PO:209/310 的 initializer/continuous SFC checker、PO:3147/3389 的 static plane 校验/硬行、PM:2299 的 retime checker 都跳过 LOS source；enforceCandidateLosPlanesSCP 的正常显式出口均为 true。PM:12978 的 LOS provenance loss 是 warning。SIDE A* 的整路 side-sign gate 也已退出当前 admission。

但以下仍构成真实源码路径：

- **条件性直接 hard reject**：PO:2462 的 maxLocalSfcViolation 遍历全部 plane，未排除 LOS；PO:4102/5075 在无 static plane 时使用它，PO:5110 可据它 reject。当前 run 全 SIDE hard-SCP flag=false，带 static plane 的强制 SCP 用 continuous 分支，所以不能把这个条件分支说成此次 16 次失败的已证首因；flag-enabled LOS-only SIDE 或适用 Team 调用仍暴露该路径。
- **间接质量 veto**：PO:444 的 LOS-QP 只有 LOS slack 行与 position trust box，没有 static/dynamic/swarm/PVA 硬行。PO:685 修改 inner_points，结束后直接生成新 polynomial。PO:5427 在 hard SCP 成功后执行它，却返回旧 scp_success；PO:5571 同理。最后 preflight 拒绝被改坏的 polynomial 是正确的，但原本已安全的优化结果被覆盖，质量机制由此能消灭 candidate。源码注释“last checked iterate”不成立：上一次 LOS iterate 未做完整 hard recheck。
- **预算也可变成质量 veto**：LOS 内 executionCheckpoint 抛出异常后，PO:5345 的 wrapper 恢复初始 seed 并把整次求解判 EXECUTION_DEADLINE，可能丢掉已完成的安全优化结果。应保留同一候选的最后 hard-safe revision，而不是再构造新 fallback。
- `LOS_HARD_REJECT_COUNT` 在 PM:12656 仅针对旧字符串 LOS_PLANE_SCP_FAILED，自然漏掉 residual 走廊出口、下游 safety reject 和预算出口。零计数不能验证“LOS 不会杀候选”。

最少机制方案：将 observation-side 的 slack/软项合入已有求解流程，或者使现有 polish 成为一个不能替换已验证 revision 的可选提案；一次普通 hard validation 决定是否接受修改，失败保留同一已有结果。二者择一，不保留两套质量修复链。

### 7.2 K3 / M2

PM:3266 的 binary recovery 使用带 stamp 的实际 odom position/yaw、同 world time 的目标/障碍评估，并绑定 executor-confirmed trajectory/generation；不再以未来计划的 C3 改善直接报恢复。应保留这个 executed witness。但 odom 不自带 traj id，当前归属来自已确认的 executor 状态；目标/动态预测也不是实测障碍真值。该结构优于 plan-label，但运行验收仍须保留 identity 与 sensor/scene truth 的区别。

PM:3350 明确 M2 抢占 Local K3；PM:3714 和 TC:1301 在 Local K3 enabled 时退役 Team K3 realization。当前主链的职责划分基本正确。

缺口是 PM:1713–1730：K2 baseline 缺失时 SIDE 不可成为 best；window invalid/K2 低于 baseline 时，即使 candidate hard-safe 也被排除，最后只能 retain previous。代码没有在该 comparator 内为“仅剩安全 successor”绕开质量 veto 的明确分支。PM:2688 的 safety-required bypass 只处理另一个 Team certificate gate，不能代替这里的隔离。K2 保护可拒绝**可选 K3 升级**，不能关闭安全供给。

Team M2 可以有任务成功门：make-before-break、至少两机可见、realized benefit。PO:3510–3574 的 visibility hard rows、PO:5120 的 final margin/binary reject应记为 TASK_CONTRACT_REJECT；不能记 physical infeasibility。Team SIDE sign（PO:3450、5123；TV:233）也是拓扑/任务门，不能把整个 Local safety class 改坏。

TC:2202 发布 target-centered schedule，PM:3697 在 Local 生成 polynomial，PM:5179 对 ACK 中每个 PVA/inner/duration 字段做精确 payload 对比。这是当前应保留的唯一 producer 合同。

但是 PM:5309–5510 仍接受旧 TeamTrajectorySolution::STATE_PROPOSAL，重建外部 MINCO，检边界与安全后进入 pending 接受链。当前 coordinator 主路径搜索到的是 TeamReferenceSchedule proposal，而不是该旧外部 polynomial proposal，因此将它归为**兼容/条件入口**，不声称旧 producer 在此 run 实际工作。为严格满足 Local 唯一 producer，应删/禁该旧 proposal authority；同一精确 Local payload 的重建和 ACK 对比可留。

### 7.3 final execution preflight：需要保留，但现在不统一

1. **重复与变更混杂**。PM:8934 classify、1631 batch preflight、6514 commit reanchor、2761 publish preflight、7200 suffix revalidation 多次做同类工作。变更 head/activation 后重检是必要的；问题是变更多次且先选质量后改轨迹。PM:6522/6524 对完全相同 committed trajectory 调 swarm conflict 两遍；PM:4227 的 Team local_hard_preflight 先逐项验，最后又 validateExecutionTrajectory；PM:7286 的 suffix 同样重复。
2. **阈值/几何不一致**。SCP sampled residual 用 2e-3、全局 dynamics 约 limit×1.0001；initializer/fallback lattice 用 1e-6；PM:7923 用 feasibility_tolerance；SFC fallback 1e-3、retime 1e-6。SCP dynamic hard radius（PO:8507）取所有障碍最大半径加 body radius，PM:7372 取每个障碍实际半径。两者都是 3D center-distance，但并非同一 physical clearance 判据。其球形中心距离模型也不自动等价于任意高度处的有限圆柱碰撞；若依赖定高近似，需要显式约束飞行高度适用域。
3. **swarm trigger 被当 hard**。PM:8007 的 Euclidean trigger=clearance+temporal_swarm_trigger_margin，在 PM:8965 用 !triggered 作为 swarm_valid；最终 PM:8097 却用 `dx²+dy²+0.25*dz²`。另外 PO:5503 的 LBFGS 成功出口要求 `min_ellip_dist2_>(1.25*swarm_clearance_)²`，否则进入碰撞重启；PO:7599 的该量来自目标函数采样。这又把1.25倍soft buffer变成硬验收，且采样域是前2/3控制点。早期偏好/触发范围和最终物理判据不同。
4. **最终 swarm 还含安全例外**。PM:8164–8200 当 active reference 在同一未来时刻也违反 clearance 时，只要候选不比它更差就 ACCEPT_NON_WORSENING。这不是绝对 hard clearance；甚至判定依据是未来 active reference，不能自动等同“当前已不可避免地身处违规”。普通 candidate 的更早 classify 可能挡住一部分，但 Team realization、suffix validation 等直接使用本函数。Team payload 的 PM:2366 检查又无此例外。应停止把该结果称 absolute hard-safe；已有违规需要明确不可认证的前提，不能靠隐藏例外证明安全，更不能以“双机都会对称避开”的文字代替闭环保证。此 run 的触发与执行影响未做逐身份重放，CANNOT_CONCLUDE。
5. **validated coverage 被高估**。PM:2388 对 rolling 的 static/dynamic/swarm horizon=min(duration,prediction horizon)，却在 PM:2780、2256 写 execution_safe_until=activation/end of whole trajectory。PM:1318 随后把它用于真实后继 deadline。周期 0.15 s 重验可以降低实际风险，但不能把尚未验证的尾部提前认证为可用覆盖。证书必须携带真正 checked_until，非全 duration；预测尾部和已验证前缀分开即可，不需要新状态机。
6. **checker 还有采样边界**。jerk 用 0.02–0.05 s 样本；static 用有限时间样本；swarm 0.03 s，某些循环不精确补 interval end。static map 缺失时 PM:7955 返回 true；dynamic 缺某个 prediction 时 PM:7301 跳过（零 prediction 又 invalid），异常障碍尺寸不能形成 clearance；这些是“未完整评估”的条件，不能静默升格安全。应按同一模型声明缺数据、区间和保守性，不能因同一 boolean 为 true 就声称连续 physical safety。
7. **revision 未冻结到 commit**。PM:1466 冻结 batch activation、1595 prepare、1639 revision++ 后比较 C3/D3；PM:6514 又用 now+margin regenerate，revision++。普通 visibility 重算不等于 K3 window 重新评估/重新排序。再加 PM:2090 的 LOS world-plane rebase 只在传入 activation 发生变化时触发，batch caller 已传新 epoch 时可漏更新旧 plane anchor。最小修正是一次定稿的 `(candidate id, revision, polynomial, activation, world snapshot, checked_until)`；变化就使相关证书失效，不能沿用旧排名/安全元数据。

## 8. Candidate rejection authority 全表

**计数规则**：按“阶段×独立判据职责族”计数，不按每个 return/每条日志计数。同函数的维度/finite 子检查合并；同一物理判据在不同阶段具有独立拒绝权的，分别列出，并标明重复。覆盖主 N/L/R Local、Local 实现的 Team 接口和 executor；不把全局任务生成器、未调用的测试 helper、纯 telemetry 纳入。R01–R40 为 Local，R41–R49 为 Team 接口，R50–R52 为 executor，**TOTAL_REJECT_AUTHORITIES=52**。这不是“一条 SIDE 必经 52 个串联门”。开关、是否 A*、是否 Team 会选不同分支。

类别：H=HARD，S=SOFT/TASK，T=TOPOLOGY，N=NUMERICAL，B=BUDGET。CAN_REJECT 包括不给候选生成机会、candidate invalid、拒绝发布/激活；正常质量排序落选另行说明，不计作安全失败。

| ID / STAGE | FUNCTION | FILE:LINE | CONDITION | PHYSICAL_MEANING | 类别 | CAN_REJECT? | SHOULD_REJECT? | DUPLICATED_ELSEWHERE? |
|---|---|---|---|---|---|---|---|---|
| R01 batch | reboundReplan / side_work_allowed | PM:8534、10139 | optional 和 mandatory 均不允许/预算不足 | 计算机会耗尽，不是无路 | B | 是，未生成 | 应停止过期工作，保留已认证候选 | R25、R39 |
| R02 moving initializer | buildFreshMovingInitializer | PM:778、872 | 非有限 PVA、生成异常、7 次 timing 仍不合格 | 初值失败/边界条件冲突未区分 | N/H | 是 | 坏状态应拒；数值 seed 失败不得称无路 | R03、R05、R20 |
| R03 N static initializer | buildStaticFeasibleInitializer | PM:8249、8324、8405、8470 | A* fail、path<3/>25、timing 耗尽 | 有限搜索/表示/初值限制 | N/T/B | 是 | 分因；不能全叫静态不可行 | R08、R09、R02 |
| R04 topology dispatch | reboundReplan / attempt_side | PM:8998、9850、9930 | risk/descriptor/witness/K3/M2 授权未满足 | 有界探索策略 | T/S/B | 是，压制 SIDE | 可管探索；不能否定已有安全轨迹 | K3/Team 任务资格 |
| R05 SIDE initializer/window | reboundReplan | PM:10352–10465 | offset 候选生成失败、duration 无效/不覆盖 conflict window | 表示/时域不适用 | N/S | 是，最终可全失败 | 坏初值可丢；质量窗口不足不能冒充无路 | R02、R07 |
| R06 repair eligibility | trial_dynamic_valid / astar_base_valid | PM:10478、10845 | seed risk invalid 或 hard_collision | 动态 seed 条件阻塞静态 repair | H/N 混用 | 是 | 不应作 static search 门 | R21、R29、R40 |
| R07 repair anchors | SIDE static interval/rejoin | PM:10970–11059、12256 | 无 collision interval、无 free anchors/rejoin | 本次 repair domain 无法构造 | N/H | 是 | 应报 DOMAIN_UNAVAILABLE，不证明全局无路 | native checker anchor |
| R08 local search | AstarSearch | AS:174–449；PM:11139 | out of pool、occupied endpoints、timeout、search exhausted | 静态有限图搜索 | H/B/N | 是 | 可拒该搜索；各因分开 | R03、native A* |
| R09 route/guide | raw_path_is_static_free / simplify | PM:11079、11278、11342 | raw 点 occupied/不合法、guide 维度/连接不满足 | 部分静态证据/表示失败 | H/N | 是 | 物理点碰撞应拒；须补边级合同 | R10、R28 |
| R10 corridor build | try_build_segment / tasks | PM:11548、11795 | 扫描/rectangle/窗口失败、leaf 构造失败 | 选定内近似构造失败 | H/N/B | 是 | 不能叫物理无路；先修全叶覆盖 | R19、R34 |
| R11 pre-MINCO checker | finelyCheckAndSetConstraintPoints | PM:12383；PO:5715、5813 | CHK_RET::ERR | 可含采样建表/内部 A* 搜索失败 | N/H/B | 是 | 区分坏数据与 constraint-generation fail | R14、R22 |
| R12 optimizer input | optimizeTrajectoryWithinBudget | PO:5363 | inner/T 维度不符、不可进入求解 | 数值输入合同 | N | 是 | 应拒坏输入 | R02、R15 |
| R13 native MINCO | LBFGS loop | PO:5480–5578 | solver error、min_ellip_dist2未超过1.25倍clearance、静态碰撞重启/反弹次数耗尽 | 局部数值失败/物理与buffer混用 | N/B/H/S | 是 | 不可认证输出应拒；多余swarm buffer不应hard；不等于无路 | R22、R28、R30、R40 |
| R14 SCP static rows | initial_static_check | PO:2796 | native rays/内部搜索构造 ERR | 静态约束生成失败 | N/H/B | 是 | 不应仅命名 STATIC_QP_FAILURE | R11、R22 |
| R15 SCP plane/state | runCandidateHardCorridorSCP | PO:2045、3147、3253 | plane/state/T 无效 | 数据合同坏 | N | 是 | 应拒 static 数据损坏；LOS 不入此门 | R12、R34 |
| R16 QP/trust | runCandidateHardCorridorSCP / solve | PO:4170–4348、4940；QP:55 | max_iter、infeasible、数值/信任域/步数耗尽 | 本轮凸近似失败 | N/B | 是 | 可结束该 solve；禁止推导全局物理无解 | R25 |
| R17 SCP trial | nonlinear trial acceptance | PO:4530–4956 | trial static/dynamic/dynamics 或 model agreement 不满足 | 中间迭代/最终安全混用 | N/H | 是 | 最终应 hard；内部需一致可行性恢复 | R19–R22 |
| R18 seed tube | maxViolation / final | PO:2414、3319、5110 | 离 reference seed 超 radius | 人工拓扑保留，不是碰撞 | T | 是 | 不应作终局 safety gate | side soft cost/iterative trust |
| R19 continuous SFC | continuousLocalSfcMaxViolation / final | PO:303、5075、5110 | static corridor violation>2e-3 | 当前 checker 错；正确时是选定 corridor 未获证 | H/N | 是 | 修数学/时间后作 containment；不证全局无路 | R10、R34、R40 |
| R20 SCP PVAJ | appendDynamicsConstraints / final | PO:5136、5159 | lattice residual>2e-3 或 global rate 超限 | polynomial 动力学 | H/N | 是 | 应拒真实超限，统一 checker/tol | R02、R27、R40 |
| R21 SCP dynamic | maxDynamicBodyViolation | PO:2436、5149 | 最大物体半径模型的 residual>2e-3 | 动态机体安全的另一模型 | H | 是 | 应统一 per-object 判据 | R06、R29、R40 |
| R22 SCP static final | finelyCheckAndSetConstraintPoints | PO:5180 | 非 OBS_FREE | native static 检查/生成混合 | H/N | 是 | unsafe 拒；checker error 独立 | R11、R14、R28 |
| R23 LOS-only residual | maxLocalSfcViolation / final | PO:2462、5075、5110 | 没 static plane 时 LOS residual 被纳入 | 观察质量 | S 误作 H | 是，条件分支 | 不应 | LOS soft-QP / retime 已 skip |
| R24 LOS polish | enforceCandidateLosPlanesSCP | PO:444、685、5427、5571 | 改坏已安全 P，原 revision 丢失；失败 step 切片越界 | 可选质量损害可用结果 | S/N | 间接是 | 不应剥夺已有安全结果 | R25、R27–R40 |
| R25 optimizer deadline | executionCheckpoint / wrapper | POH:887–900；PO:5345 | now+call_reserve≥deadline | 预算/预留量不足 | B | 是，整次恢复 initializer | 停止未完成工作；不应抹掉已认证结果 | R01、R16、R24 |
| R26 initializer execution admission | feasible_initializer / fallback_contract | PM:12570、12707 | static/lattice/SFC/side flags/class 未满足 | 决定 seed 能否被执行 | H/N | 是，不能兜底 | 应经同一 final authority；不额外要求拓扑/不同 tol | R27–R30、R40 |
| R27 classify dynamics | classify_candidate | PM:8949 | manager PVAJ invalid | 动力学证书 | H | 是 | 是，同口径 | R20、R40 |
| R28 classify static | classify_candidate | PM:8957 | sampled map/ceiling fail | 机体静态 | H | 是 | 是，同几何/时域 | R11、R22、R40 |
| R29 classify dynamic | classify_candidate | PM:8980 | risk invalid/hard_collision/negative clearance | 动态证据/物理安全 | H | 是 | 是，但缺数据≠碰撞 | R06、R21、R40 |
| R30 classify swarm | evaluateTrajectorySwarmConflict | PM:8961、8007 | Euclidean distance<trigger+margin | trigger buffer 被升格 safety | H/S | 是 | 仅统一物理 predicate 可 hard | R40 |
| R31 existing SIDE persistence | preserve_active_observation_side | PM:9876–9885 | 同 blocker 旧 SIDE 可留 | topology preference | T | 是，N class=INVALID | 不应改写 safety | 普通 comparator/persistence |
| R32 ready/head regeneration | prepareLocalHandoff | PM:2083 | 无效 state、已无 predecessor coverage、重生成失败 | 生命周期/PVA/数值 | H/N/B | 是 | 必要，但只定稿一次 | R35、R37、R52 |
| R33 short stationary | finalizeCapturedCandidates | PM:1620 | duration<rolling_coverage 且 max speed<0.2 | 连续运动/供给策略，不是碰撞 | S/B | 是 | 不能覆盖合法 terminal 语义或冒充安全 | FSM terminal、R01 |
| R34 retime SFC | validateRetimedLocalSfc | PM:2291–2327 | malformed static plane、采样 residual>1e-6 | containment/当前时间映射 | H/N | 是 | 应统一可信连续 checker | R19、R40 |
| R35 candidate handoff | checkActiveHandoff | PM:2200–2235、1632 | activation 超 validated end 或 PVA 不连续 | 执行拼接 | H/B | 是 | 是，但 coverage 必须真实 | R32、R39、R52 |
| R36 K3 eligibility | finalizeCapturedCandidates | PM:1713–1746 | 无 baseline、window invalid、K2 regression | 质量合同 | S | 是，可能无 best | 只拒可选升级，不拒唯一安全供给 | Team quality |
| R37 commit revision | finalizeTopologyCandidate | PM:6514–6569 | 再 reanchor、再 SFC/classify 不通过 | 修改后的新 polynomial | H/N | 是 | 变了须验；应消除多次修改 | R32、R34、R40 |
| R38 Local-vs-Team certificate | setLocalTrajFromOpt | PM:2688–2719 | 活跃 Team quality certificate，且非 safety-required successor | 任务时间承诺 | S | 是 | 仅可选升级适用；现有 safety bypass 应保留 | Team adoption |
| R39 publish preparation | setLocalTrajFromOpt | PM:2749–2768 | lead<0.015、prefix/handoff fail、检查点生成 fail | 激活/PVA/数据 | H/B/N | 是 | 必要；区分数据与安全 | R32、R35、R52 |
| R40 final physical preflight | validateExecutionTrajectory | PM:2330–2378 | dynamics/static/dynamic/swarm/Team exact peers 不通过 | 最终飞行安全 | H | 是 | 应为唯一执行安全裁决；修 coverage、模型、swarm 例外 | R20–R22、R27–R30、R37 |
| R41 Team reference input | teamReferenceScheduleCallback | PM:3801–3924 | stale generation/CAS/context/roles/time/budget | 协调身份/时域 | H/N/B | 是，拒 Team 提案 | 是；Local 已安装轨迹继续 | R48、R49 |
| R42 Team SCP input | runTeamContractSCP / contract rows | PO:1913、3510–3574 | disabled/inactive、窗越界、margin gradient 无效 | 任务/数值输入不完整 | S/N | 是 | 拒该任务求解，不能说飞行无路 | R41 |
| R43 Team SIDE sign | side rows/maxTeamSideViolation | PO:3450、5123 | signed side/window 不满足 | 拓扑保留 | T | 是 | 不应飞行安全化 | R18、TV:233 |
| R44 Team visibility | teamMarginAt / final | PO:3574、5120–5134 | margin/binary visibility 未达合同 | 观察任务 | S | 是 | 可拒 Team upgrade；不能杀 Local safety | R46 |
| R45 Team Local realization safety | local_hard_preflight / trial admission | PM:4227–4282、4700–5027 | realized 安全、repair资格/收益、预算未满足 | 安全/任务/数值多个条件 | H/S/N/B | 是 | 应分开返回，重用 R40 | R26、R40、R44 |
| R46 realized Team validation | validate realized triple | TV:175–487 | identity/cert/side/interaction/overlap/baseline/收益 | Team task contract，含物理交互 | H/S/T | 是 | 物理用 R40；任务拒绝仅影响升级 | R43–R45 |
| R47 legacy external polynomial | teamTrajectorySolutionCallback | PM:5309–5522 | 外部 payload 格式/上下文/边界/安全不符 | 旧 producer 兼容入口 | H/N/S | 是 | 旧 proposal authority 应退出 | R41、R45、R48 |
| R48 Team PREPARE/COMMIT | teamTrajectorySolutionCallback | PM:5141–5308 | ACK exact payload changed、无 prepare | transaction/身份 | H | 是 | 必要，保留精确 Local payload | executor prepared gate |
| R49 Team adoption | Team refinement/adoption & pending commit | PM:2882、4479–5027、setLocalTrajFromOpt | 过期上下文/不再必要/activation 不可达 | 可选任务升级/CAS | S/H/B | 是 | 拒升级，不能阻塞基础安全供给 | R38、R41、R48 |
| R50 receive payload | trajCallback | TS:1018–1175 | 无 safety_validated、无效 order/duration/PVA/payload | 数据/生产者证书 | H/N | 是 | 必要，但 flag 不是独立物理验证 | R12、R40 |
| R51 scheduled transaction | prepare/commit/normalizeFutureTimeline | TS:443–572、695–793 | 无对应 PREPARE、ABORT、依赖/同 activation revision 冲突 | 执行 lineage | H | 是 | 必要，不能让 speculative Team 覆盖 guaranteed Local | R48、R49 |
| R52 activation handoff | executionHandoffGate | TS:795–980、1530 | late/缺 predecessor、id/generation 不符、PVA/live-odom mismatch | 实际激活连续性 | H/B | 是 | 必须保留，与 planner 同容差/身份 | R35、R39 |

C3/D3/普通 geometry/cost comparator 的**正常落选**不计 reject authority：它只在同安全集合里选一个，不使未选候选物理无效。R36 是额外资格 veto，性质不同。

### 8.1 隐藏或越权 hard gate 汇总

**HIDDEN_HARD_REJECTS=11 个源码机制族，不是 11 次运行事件：**

| H | 机制 | 关联 authority | 运行归因边界 |
|---|---|---|---|
| H01 | 动态 seed 才可 static repair | R06 | source 确定；31 个 NO_VALID_SIDE_SEED 不能全部归它 |
| H02 | seed tube 作为 final corridor | R18 | 首例 QP 144 行已见；每次失败贡献未知 |
| H03 | N target-facing A* 硬半空间 | R03/R08 | 条件入口；不是 SIDE path-wide sign 的旧门 |
| H04 | LOS-only sampled residual | R23 | 条件 SCP 路径，未证本 run 触发 |
| H05 | LOS polish 覆盖安全结果/耗预算 | R24/R25 | 源码可达；日志缺 pre/post safe identity，次数未知 |
| H06 | 同 blocker 把 N 置 INVALID | R31 | 非 capture/回落入口，不能冒称主 captured 链必走 |
| H07 | K3 baseline/K2 质量排空 safe set | R36 | 当前源存在；没有本报告逐事件死亡计数 |
| H08 | Team SIDE sign 约束/validator | R43/R46 | 任务/拓扑条件，不能叫 physical safety |
| H09 | pre-MINCO ERR 含搜索失败，却按 corruption 终止 | R11/R14 | native A* 失败有真实 return 路径；非单纯 malformed seed |
| H10 | short-stationary/liveness 门 | R33 | 与终点任务语义需分开，不能混作碰撞失败 |
| H11 | LBFGS 以1.25倍swarm buffer硬验收 | R13 | PO:5503；条件路径活跃，具体失败数缺统一id/residual记录 |

另有 **swarm non-worsening 隐藏放行**，性质是 hard safety 例外，不计进上述 rejection 数量。

## 9. Fallback / retry / repair / reuse 全表

计数按一个有唯一意图的机制族，多个调用点共享者合并。F01–F32 当前有 production/条件调用路径；F33–F34 仅发现声明、实现和 test 调用，列为 dormant。**TOTAL_FALLBACK_REPAIR_PATHS=34（32 个可达或条件机制族 + 2 个 dormant helper）**。不把 Team 上层 reference optimizer 的多目标阶段回退计入 Local；它不直接生产 Local polynomial。旧 Projected MINCO 文档也不充作一个活跃 fallback。

| ID | 机制 / SOURCE | 判定 | 唯一职责与处置 |
|---|---|---|---|
| F01 | accepted P/T warm start；PM:7089、10362 | NECESSARY | 使用旧解改善初值；必须重新绑定 head/时间，不携带旧安全证书。 |
| F02 | remaining suffix eligibility / geometry reuse；PM:921 | NECESSARY | 判断旧 suffix 是否仍是相同任务初值；与 F01 合成一入口，不能拿短旧 T 强配新目标。 |
| F03 | fresh direct 最多 7 次时间放大；PM:872–915 | PATCH_ACCUMULATION | 补 PVA-aware timing 缺口；合入唯一 initializer 的有界数值过程。 |
| F04 | N target-halfspace A* 失败再 unbiased；PM:8318 | REDUNDANT | 为硬偏好剪枝补救；去掉 hard halfspace 后无需这条回退。 |
| F05 | N A* seed 最多 7 次时标重试；PM:8445 | PATCH_ACCUMULATION | 与 F03/F14 修同一 timing 问题，应合并。 |
| F06 | NOMINAL optimizer 失败保留安全 initializer；PM:8858 | NECESSARY | 一份已独立验证的 incumbent 可保留；与 F20/F31 合为共同“保留已认证 revision”。 |
| F07 | SIDE init failed 改 flag_polyInit 再 computeInitState；PM:10401；PM:985 | REDUNDANT | computeInitState 忽略 poly/random/ts；path-frame 相同时重复同输入。若省略 observation override 导致换 frame，职责已与 F09 重叠。 |
| F08 | SIDE offset 0.7→约0.4 backoff；PM:10352 | PATCH_ACCUMULATION | 有界 seed 几何枚举，可缩入同一 guide 构造规则；不应每个 offset 都反复做完整 native 搜索/验证。 |
| F09 | observation frame→path frame；PM:10508 | NECESSARY（需简化） | 观察方向不适用时保持静态候选供给；作为 seed 策略选择一次完成，移除混用 trial_risk。 |
| F10 | SIDE duration/window fallback；PM:10423 | PATCH_ACCUMULATION | 补初值时域；统一 timing/window 合同后删除独立重建。 |
| F11 | 多个 rejoin anchor / first search success 保留；PM:11112–11208 | NECESSARY（有界） | 单次局部 repair 的 anchor 选择，不是新 planner。存在多次 A*，所以源码注释“一 SIDE 最多一次 A*”不是普遍事实。 |
| F12 | 简化后保留拐点/近端点；PM:11278–11425 | NECESSARY（需统一） | 几何表示保真；不能以无限恢复 raw 点来修 MINCO，把分段与 corridor 合同一起决定。 |
| F13 | corridor depth≤2 midpoint split；PM:11770 | NECESSARY（实现错） | 凸体生成的有界细分；必须全叶成功、覆盖一致，不能部分成功即接受。 |
| F14 | A* MINCO 一次 λ dilation；PM:11949 | PATCH_ACCUMULATION | 纯 scaling 假设不成立；被正确固定 PVA timing 替代，禁止再叠 λ。 |
| F15 | native fine checker 内 A*/段合并/rays；PO:5792–5813 | LEGACY（仍活跃） | 与显式 A*+SFC 重复修 static geometry。SFC 候选应消费统一几何合同；无 SFC fast path 保留必要 native 检查，避免双重建图。 |
| F16 | LBFGS collision restart≤3；PO:5495–5550 | LEGACY（仍活跃） | native 无 corridor fast path 的障碍约束更新。不能再套在 SFC-SCP 链外补相同缺陷。 |
| F17 | LBFGS rebound≤20；PO:5530–5550 | LEGACY（仍活跃） | 与 F16 同属 native 求解生命周期，合并迭代与返回语义，不另成 fallback authority。 |
| F18 | SCP trust shrink/grow、no-P/no-T/no-PT probes；PO:4198–4348 | NECESSARY（过度混合） | 信任域是数值必要机制；诊断 probe 不能当物理 oracle，每种失败必须保留 solver status，避免重复无效求解。 |
| F19 | LOS QP失败保留 last iterate；PO:655 | PATCH_ACCUMULATION | 该 iterate 不保证 hard-safe；用共同已认证 incumbent 替代，失败切片越界先修。 |
| F20 | SIDE feasible-initializer/deadline fallback；PM:12707 | NECESSARY（需合并） | 允许安全未优化轨迹执行是合理的；不应依赖244次独立兜底维持架构，统一 F06/F31。 |
| F21 | KEEP_PREVIOUS_SAFE / PERSISTENCE_FALLBACK 多出口；PM:2005、6462、7200 | NECESSARY | 保持仍有效的执行前缀；统一一次相同快照下的安全验证和来源，不改变物理阈值。 |
| F22 | suffix invalid 后 horizon 连续减半找 safe prefix；PM:2263 | NECESSARY（需合同化） | 获取可用 coverage；应输出真正 checked_until，不能再扩成全 duration。 |
| F23 | predecessor 过期后 current-state restart；PM:1330、2050 | NECESSARY | 旧 handoff 不存在时从真实 odom PVA 进入同一 Local；已发生 coverage failure 不能靠重启计为健康。 |
| F24 | captured path 不成功后普通 try_replan；FSM:1115–1138 | PATCH_ACCUMULATION | 两种候选生产/选择 authority 回落。合并 captured 与普通生产合同，失败不应重复整批算法。 |
| F25 | FOV soft retry / 改相对距离；FSM:1140 | PATCH_ACCUMULATION | 为可选质量扰坏求解再跑整批；质量应不夺 incumbent，移除重复入口。 |
| F26 | FSM local→poly→random→odom/global 多次重试；FSM:1168–1274 | PATCH_ACCUMULATION | poly/random 标志部分已无效；仅保留必要真实 head 失效后的 odom 重建，合并 F23。 |
| F27 | executor future timeline/Team abort 保留 guaranteed Local；TS:443–793 | NECESSARY | 事务失效不得覆盖已承诺轨迹；是执行调度一致性，不是几何修复。 |
| F28 | terminal hold；TS:1668–1690 | NECESSARY | 有限轨迹终点的命令定义；rolling 耗尽后 hold 是供给失败，合法任务终点须单独识别。 |
| F29 | heartbeat stale 继续有效 active trajectory；TS:1634–1650 | NECESSARY | 通信新鲜度下降时不打断尚有效的轨迹；不得把 expired trajectory 延长。 |
| F30 | optimized yaw 无效时 target-facing yaw；TS:1040–1085 | NECESSARY | 同一位置轨迹的相机/航向命令回落，与平移 feasibility 分开，不认证 visibility 已恢复。 |
| F31 | Team Local optimization/预检失败保留 initializer；PM:4220–4282 | NECESSARY（需合并） | 与 F06/F20 同一个 incumbent 原则，不再有第三份“安全 seed”逻辑。 |
| F32 | Team AUTO T-only 失败从原 seed 再做 P/T；PO:1942–1986 | NECESSARY（有条件） | 自由度升级有独立职责，可保留为同一求解模式选择；不能把 T 子空间失败叫 physical infeasible，也不新增下一层模式。 |
| F33 | runCandidateTimeOnlyFeasibilityCorrection；PO:708 | LEGACY / DORMANT | 当前 production 未发现调用，仅 test；不要重新接入作为第四套 timing repair。 |
| F34 | runCandidateTimeOnlySwarmDeconfliction；PO:917 | LEGACY / DORMANT | 当前 production 未发现调用，仅 test；不要为下一失败标签重新启用。 |

F32 的必要性依赖保留 AUTO 子空间策略；若默认直接采用所需的 P/T 自由度，可删除该子空间升级。保留时不得扩展模式链。

最明确的合并组：**F03/F05/F10/F14→一个 timing initializer；F06/F19/F20/F31→一个已认证 incumbent；F01/F02→一个 reuse 入口；F21/F22→一个 prefix certificate；F23/F26→同一 odom/head 修复；F15/F16/F17 只留给无 corridor 的 native 路径并统一状态，不与已认证 SFC 双重修复。**

## 10. 计算量审计：哪些工作贵，哪些只是标签多

### 10.1 一次 N/L/R rolling 的实际计算链

`coverage/head → N initializer（可能 N A*、7 次 timing）→ native constraint points → N LBFGS（每步 MINCO+积分+LOS/动态/swarm，可能 restart/rebound）→ N 分类/风险/raw LOS → PLUS（offset/frame/window/可选 A*）→ MINUS（取决于是否仍有预算/安全供给）→ candidate prepare/preflight → visibility/K3/geometry → 再 commit reanchor/验证 → publish/activation`。

每个 A* SIDE 内部又是：`多个 seed precheck（其内部可再 A*）→ 至多多个 rejoin 搜索 → simplify/curvature timing → SFC 扫描/box audit/细分 → MINCO seed + dilation +多种检查 → native rays → hard SCP → 可选 LOS-QP → 分类`。不是简单的“三次独立 MINCO”。

| 环节 | 成本来源 / 证据 | 判定 |
|---|---|---|
| N、PLUS、MINUS 数量 | 常规至多三个 intent，但 offset、frame、rejoin、FSM 整批重试让实际计算次数远大于三；MINUS 可因预算不执行 | 有放大作用；不能只凭三候选就说应删一侧 |
| A* | 本 run 显式 SIDE rejoin 搜索 118 条，P50≈0.030 ms、P90≈0.156 ms、max≈0.512 ms；不含 native checker 内搜索 | **不是已观测主瓶颈**；无需先缓存/加 budget |
| corridor build | 3 站扫描 + rectangle 网格审核，量级约 segment_length×width/r²，加细分；100/117 在此失败 | 主要供给断点；缺独立 timer，不能给 CPU 占比 |
| MINCO generate | banded solve 随 piece 数增长；多次 regenerate、精确 V/A 极值与 jerk 扫描叠加 | 数值初值造成的重复比 MINCO 表示本身更可疑；缺单独耗时拆分 |
| SCP/OSQP | 每轮 MINCO Jacobian、多个 hard row family、native objective/visibility、最多多次 trust QP及 probes | 昂贵尾部路径；错误约束/坏初值造成低收益求解 |
| LOS soft-QP | 另建含每行 slack 的 QP、重复 MINCO Jacobian、改 P 后又重新 hard check | 与主优化分离产生重复工作和结果覆盖；应合入/严格可选 |
| final/preflight | manager 分类、batch、commit、publish、suffix、Team wrapper 多次同函数扫描；部分还重新生成 | 存在确定重复，尤其同 revision/epoch 未变的检查 |
| C3/D3 | 每个 hard-safe candidate 在共同时间窗扫 self/peers、camera/static/dynamic LOS | 属有价值质量计算；必须在冻结的安全结果上一次完成，当前后续再 reanchor 使部分工作过期 |
| 日志/诊断 | 四份 rosout 轮转约 413 MB 未压缩；SCP 每点/每平面/每行诊断密集，PO:5201 还重复 logDirectionalVisibilityCost | 有运行开销嫌疑但未 profiler，不能定量归为主因 |

第一例 SCP 只有 25 个变量却有 337 行；144 行是 seed tube。主 QP primal infeasible 后再做 no-P/no-T/no-PT，反复求含相同人工限制的系统。先去错误/重复行、修模型/Jacobian和初值，比加 max_iter 有因果依据。连续 active set 的思想比 plane×25 全铺合理，但必须先修 checker 与采样坐标。

### 10.2 EXECUTION_DEADLINE 的证据边界

历史 run：`runs/20260926_105704_628281`。以 visibility_trajectory.csv 的 target speed>0.1 m/s 定义运动区间，最后样本 world_time=1790391509.8080027，time_s=78.231871。首个 SIDE deadline=1790391516.123276；从该时刻起 CSV target speed 最大值 **0.0**。因此：

- 运动期：117 个 handoff、100 build fail、16 static-corridor SCP，**SIDE minco deadline=0**。
- 停止后：SIDE minco deadline=197，优化耗时 P50=0.067 ms、max3.309 ms，其中195<1 ms。
- 其他 SIDE optimize 610 次，P50=2.075 ms、max429.190 ms。此耗时包含 optimizeTrajectory 整段，不是纯 OSQP。
- 807 次 SIDE optimize 中 success=518、failure=289；安全 initializer fallback used=244，其中197对应 deadline，另外47在 minco status 空字符串分支。45次无可用 initializer。应补全状态，而不是把空 reason 当“其它物理不可行”。

首个 deadline 的短链（archive rosout.log.2）：

| 行/世界时间 | 证据 |
|---|---|
| 22240 / 1790391516.118969 | drone2 active296，coverage0.146939，planning_budget0.046939 |
| 22245 / .119447 | OPTIONAL_REFINEMENT_STOP，但 validated_local_successor=0 |
| 22251 / .119698 | fresh seed distance0.000060，duration0.1，dynamics_valid=1，目标已停 |
| 22254 / .122798 | N checkpoint 取消；deadline=.165848，now=.122736，尚有约43ms但不足 call_reserve |
| 22269 / .123276 | SIDE_PLUS EXECUTION_DEADLINE，optimization_ms=.073 |
| 22270 / .123323 | same-side feasible initializer used=1，ABSOLUTE_SAFE |
| 22302 / .124413 | candidate529 revision1 hard preflight pass |
| 22306 / .124719 | 再 reanchor 为 revision2 |
| 22309 / .124819 | successor297 commit/handoff 成功 |

POH:893 的定义是 `now+execution_call_reserve < deadline`，所以这里的 label 是“下一调用没有足够预留量”，并非 solver 已执行到绝对 deadline。停点附近反复短静止 seed/rolling coverage 与普通任务终止语义混用，应先澄清。此次不能据197这个大数推出“运动避障期 OSQP超时主导”。

对题列各原因的判定：candidate 扩增和重复 validation 有源码证据；A* 实测很小；corridor build 是运动期主要供给断点但耗时占比未知；MINCO 本体不是已证主耗时；corridor SCP 确有失败/长尾；max_iter 只是16次中的3个最终原因；**坏初值、错误 checker/时间导数、冗余 hard tube 引发无效求解**是最值得先修的结构因素。不能提供没有计时证据的总 CPU 百分比，也不建议加 budget。

## 11. 原始 run 证据与 telemetry 校验

### 11.1 数据来源和复算

使用 [run 目录](../runs/20260926_105704_628281/) 的 ros_log.tar.gz，逐行读取其中 `ros_log/f4abb994-b955-11f1-85da-e3ccd32c8b93/rosout.log.3/.2/.1/rosout.log`，保留节点与 timestamp。未解压写入文件。主统计以这些完整、分节点日志为准，不混加 roslaunch_stdout.log.gz。

[feedback120_metrics.json](artifacts/feedback120_metrics.json) 及历史分析器仅作对照，没有执行。stdout 存在行交错/截断，所以旧 timing 分母116/109/108不能替代完整 rosout 的117。三机累计数按**每节点最后一条**取值，不把重复累计快照求和，也不把全文件 last3 当三台机器。

| 指标 | 独立复算 | 能说明什么 / 不能说明什么 |
|---|---:|---|
| ASTAR_REQUIRED / raw点检查通过 | 117 / 117 | 搜索入口及节点检查；不是连续边安全证明 |
| SFC handoff | build fail100；active16；not-required1 | 100个在 corridor front-end 终止，16个进入静态 SFC-SCP |
| A_STAR_TO_MINCO | corridor_valid0=100；valid1=17 | 与 handoff 一致；不是“建成117个走廊” |
| 仍生成 timing seed | 117 | 100个已知 corridor failed 的分支仍做 MINCO/timing，是确定浪费 |
| NO_STATIC_FEASIBLE_SIDE | 137=106 chain failed+31 no-valid-side-seed | 106=100 corridor fail+6 no collision interval；这137不等于137物理无路 |
| corridor SCP | 16 attempted，0 success | 这条 pipeline 未提供成功出口；不证明环境无解 |
| SCP最终原因 | TRUE_CONSTRAINT_INFEASIBILITY5；QP_MAX_ITER_EXHAUSTED3；DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED6；LOCAL_SFC_FINAL_VIOLATION1；P_TRUST_TOO_SMALL1 | 数值/模型/表示失败占全部；“TRUE”名字仍不是原问题证书 |
| local-geometry-candidate | 1756记录；safe1=1609；safe0=147（dynamics71、static52、dynamic24） | 包含不同 kind，只是该记录阶段，不与SIDE807做无身份拼接 |
| SIDE optimizer | 807；success518；failure289 | 与 fallback/执行成功不是同一指标 |
| SIDE initializer fallback | used244 | 在带独立检查的初值上继续供给；不能据此称优化健康 |
| LOS soft QP 累计 | attempted741、solved741、nonzero slack441 | 支持渐进 slack 分支运行；不证明 pre/post 轨迹始终 hard-safe，也不证明全域零 hidden gate |

### 11.2 FIRST CAUSAL DIVERGENCE 的准确范围

**本报告定位的是该 run 首个可证的 A*→SFC→MINCO 分支断点，不声称它是整个系统所有指标的最早异常。**

`rosout.log.3:8427`，drone2，world_time=1790391438.277240：`SIDE_PLUS segment=0 CORRIDOR_BUILD_FAILED`。紧接 `8428`（.2772818）记录 obstacle6、astar_static_free=1、semantic_valid=1、plane_count=0、segments=0、subdivisions=0、handoff_allowed=0。此前 raw path22、节点 static_bad=0，搜索耗时约0.202 ms。随后仍生成19 pieces，j≈632.501→90.528、λ≈3.162、duration≈4.671→14.772，最终 GUIDE_UNAVAILABLE/NO_STATIC_FEASIBLE_SIDE。

这说明此事件第一断点在 SFC build，不能倒因为后来 jerk、SCP 或 checker。日志未记录 try_build_segment 的具体返回子句，因此“宽度太窄/box audit hit/cap/窗/退化段”的精确子因 **CANNOT_CONCLUDE**。源码中发现的 z/中点/细分问题也不能逐条套作这100次的唯一原因。

第一次进入 corridor SCP 的另一个事件是 drone0 SIDE_PLUS obstacle7，1790391463.821654，20planes/5segments；其先前 handoff成功，才有 checker和QP问题。五组窗口在t=3.372875处同时激活20planes，sample min_margin≈-0.050273；continuous 报≈34.382150（受已证bug污染）。QP25变量337行，primal_infeasible700iterations，primal residual≈0.028822、dual≈657.004，三个去trust probe仍 infeasible。`rosout.log.3:33330` 最终报TRUE，但外层 telemetry iterations/active_points/continuous均为0，明显不是内部实际过程。

两个事件分开：**前一个证明前端供给断裂；后一个证明进入SCP后的模型/初值/验证合同仍不闭合。** 不能只抓第二例的 solver标签继续修补。

### 11.3 确定 telemetry/validator bug

| 缺陷 | SOURCE | 后果 |
|---|---|---|
| corridor build失败计数恒漏 | PM:11804 先把 repaired_side_valid=false；PM:12078 却仅在该值true时加failed计数 |100条显式失败，三机累计仍0；误导根因优先级 |
| continuous 数学错误 | PO:333、355、233 | 运行里的“连续 violation”非真实几何量，既可误拒也可漏检 |
| FALSE物理不可行命名 | PO:4333；PM:12270 | 线性QP/构造/数值/budget混成TRUE/NO_STATIC |
| SCP早退 telemetry默认0 | PO中的final telemetry填充在后段；PM:12670输出默认字段 | 内部已迭代，外层看似iterations0/residual0 |
| scp_success字段混入fallback | PM:12831 附近 `result.success || feasible_initializer_fallback_used` | 成功执行初值不等于SCP成功 |
| LOS hard计数监测已退役label | PM:12656 | 不能捕获R23/R24路径，零不是证明 |
| SIDE id/type误记N | PM:12526、10731 vs10143 | 不能形成唯一candidate→QP→preflight→activation身份链 |
| coverage过度认证 | PM:2388 vs2256/2780 | 未验尾部被用作validated deadline依据 |
| old analyzer证据局限 | [feedback120_analyze.py](artifacts/feedback120_analyze.py) 中的历史统计逻辑 | stdout不完整；swarm contact以distance≤0统计不能代替机体clearance；safety_validated字段不是独立验真 |

反馈中的 build/run exit=0、contact计数0、SCP label变化均不作为本次系统验收。本文没有重新验证整段执行安全，也没有宣称本次所有前端/SCP bug已在每次执行中造成碰撞。

## 12. 根因树

```text
ROOT PROBLEM: 静态自由几何、数值可解性、观察任务、执行证书没有形成单一合同；
              同一候选在多个阶段被不同表示/时标/阈值重复判定。
├─ DESIGN ISSUE
│  ├─ 内近似SFC/seed tube/topology/task quality 与 physical safety 混层
│  ├─ 不可行seed允许进入，却要求SCP有限小步立即全可行
│  └─ 可选LOS polish和K3质量门可以剥夺已有安全供给
├─ IMPLEMENTATION MISMATCH
│  ├─ SFC不是认证3D子集；细分all-leaves判定错误；piece绑定缺失
│  ├─ static A*入口仍受dynamic seed约束；N仍有hard偏好半空间
│  ├─ static/global-progress/piece-local/world-time映射及导数不一致
│  └─ batch排名后再次reanchor；coverage与实际验证时域不一致
├─ NUMERICAL ISSUE
│  ├─ 固定PVA下错误套用统一λ缩放规律，密集短piece造成尺度失衡
│  ├─ 多重硬tube/caps/有限trust、不可行线性化难恢复
│  └─ tracking时间梯度、dynamic world-time Jacobian错误影响模型一致性
├─ VALIDATOR / TELEMETRY BUG
│  ├─ 连续quintic checker错原点且漏quartic项
│  ├─ corridor failed=0、早退SCP字段0、成功字段混fallback、错candidate id
│  └─ swarm硬检查含non-worsening例外；未知数据/有限采样被过度认证
├─ COMPUTATION ISSUE
│  ├─ 失败SFC仍生成MINCO；native A*与显式A*重复
│  ├─ 相同revision重复检查；质量polish后重验/再改head
│  └─ 无效QPs/probes/retries，且停点后deadline计数混入运动期诊断
└─ EXPECTED PHYSICAL INFEASIBILITY
   ├─ 已验证的具体polynomial真实碰静态/动态/peer，或违反PVAJ
   ├─ 边界PVA与受限可用空间、真实handoff时限冲突
   └─ 两机同时遮挡/K2不可恢复是任务不可行；不等于飞行无路
```

| 根类 | source evidence | run evidence | affected failures | confidence |
|---|---|---|---|---|
| DESIGN ISSUE | R18/R24/R36；PO:4660；PM:8164 | 首SCP tube144行；初始化兜底244 | QP/SCP失败、safe集合排空、间接preflight失败、错误安全声明 | HIGH（结构）；具体事件贡献MEDIUM/未知 |
| IMPLEMENTATION MISMATCH | PM:11548/11747、10478；PO:2383/3398/3800；PM:6514/2780 | 100buildfail、16SCP全失败，时间窗重叠实例 | NO_STATIC/LOCAL_SFC/TRUE/TRUST/DYNAMICS_MODEL_MISMATCH | HIGH（bug）；100次具体子因CANNOT_CONCLUDE |
| NUMERICAL ISSUE | PM:11425/11949；PO:7866/7907、3319/4660 | 97/117 j_after>20；短piece比例28.3；6 model-mismatch、3 maxiter | seed耗尽、SCP失败、低有效迭代率 | HIGH（公式/观测）；最优可行解存在性未知 |
| VALIDATOR/TELEMETRY | PO:333/355/439；PM:12078/12831；PM:2256 | 100失败但计数0，continuous34.38与sample差异，外层iterations0 | 假/漏corridor violation、错误active-set、失真归因；潜在wrapper失败越界 | HIGH；未证明本run触发wrapper越界 |
| COMPUTATION | PO:5345/5550；PM:11845/6522；FSM:1138–1274 | A*max.512ms；195deadline<1ms且全在停点后；非deadline长尾429ms | EXECUTION_DEADLINE、重复求解、后继准备延迟 | HIGH（已测部分）；阶段CPU份额CANNOT_CONCLUDE |
| EXPECTED PHYSICAL INFEASIBILITY | PM:2330/7923/7951/8097，具体边界与几何 | 147次prepared preflight失败：71动力学、52static、24dynamic（checker口径） | 该revision不可执行 | MEDIUM（采样checker结果）；不能外推所有路线/所有时标无解 |

**根因优先级**：ROOT_CAUSE_1=route→SFC→piece/time合同断裂（最早运动期供给瓶颈）；ROOT_CAUSE_2=连续checker/时标/Jacobian和诊断不可信；ROOT_CAUSE_3=固定PVA-aware timing缺失与SCP恢复/人工hard约束冲突。Authority重复、质量越权和coverage缺陷贯穿三者，应同一轮接口重构处理。

不提供“设计X%、数值Y%”伪精确饼图：同一候选可同时有三类问题，且缺逐系数/完整QP/执行身份replay。当前证据足以否定“多数NO_STATIC标签证明物理无路”，不足以保证修复后每条都可执行。

## 13. 健康版本：最少机制的结构方案

### KEEP

- N/L/R 三种局部 intent、bounded static A*、同一 inflated map 和现有 Local-SFC 容器。
- quintic MINCO及其固定真实head PVA；SCP+OSQP、迭代trust region、最终非线性安全验证。
- 弹性band/bearing/合围/deep-risk作为soft objective，LOS observation-side允许slack和渐进恢复。
- Local K3 / Team M2分工；共同world-time评价、Local realization exact payload ACK、executor lineage/PVA gate。
- 一份已验证的执行前缀和同候选最后hard-safe结果；这些是合法持续执行/可选优化中断语义。

### FIX

1. **先修证据基础**：连续checker两处数学bug、失败step越界、SFC全leaf判定、失败计数和id/status。这样才有可靠诊断，不先调参数。
2. **修一个现有静态接口**：A*输入资格只看static；route边/anchors认证；Local-SFC有实际凸体子集证明、覆盖范围、必要z界、相邻有效交集；segment/piece/local-time绑定在同一结构中。
3. **修一个数值接口**：PVA-aware timing/segmentation共享；修MINCO/SCP/动态/观察的时间导数；seed是初值，内部恢复有明确residual/状态；人工seed tube不再hard。
4. **修一个执行接口**：一次冻结head/activation和polynomial revision，再preflight，再quality比较，再提交同一payload。迟到不能悄悄改出另一个已排名轨迹；若必须重生成则其证书/质量一起失效，用现有流程重新认证。
5. **安全证书带真实时域**：checked_until≤实际静态/动态/swarm共同验证末端，持续滚动更新；不能把prediction-only尾部当coverage。统一peer和per-object geometry，显式处理缺数据与已有违规。
6. **任务状态和求解状态分开**：搜索无路、搜索受限、初始化失败、线性QP不可行、SCP未收敛、budget不足、具体trajectory不安全、quality未提升分别返回；不再用一个candidate failed吞掉因果。

### REMOVE

- N A*的hard target-facing半空间、SCP终局seed tube、LBFGS的1.25倍swarm buffer硬验收、非capture的同blocker安全类INVALID、Team side sign的飞行安全身份。
- sampled SFC残余LOS gate；postsolve质量修改不能覆盖已认证polynomial；K3 baseline/K2不能撤销唯一安全供给。
- 表中重复timing/frame/flag/FOV/整批重试；已完成SFC仍再靠native A* repairs保持静态可行的双重链。
- `TRUE_CONSTRAINT_INFEASIBILITY` 的物理无解暗示、静态构造失败后继续生成MINCO、旧外部Team polynomial proposal authority。
- normal hard-safe certificate中的隐式swarm non-worsening放行。保留明确真实初始条件，不许把违反物理门的结果换名认证为safe。

### MERGE

- `static route + corridor + polynomial containment` 的几何定义共用同一地图/坐标/覆盖合同；A*证明路径、SFC证明内近似、preflight证明具体输出，三者是不同证明对象，不要求机械删成一个函数。
- dynamics/static/dynamic/swarm的predicate与容差统一供optimizer residual和final preflight调用。optimizer负责找解；只有final preflight授予执行权。变了revision必须重验，没变则可复用带snapshot的证书。
- initializer fallback与deadline保留统一为“保持最后已认证revision”，previous suffix统一为“当前有效prefix”。不要让多个模块各自制造一个所谓safe seed。
- capture/noncapture生产入口统一；Team沿用Local生产、预检和exact payload提交。

### DO NOT ADD

新planner、space-time A*、FIRI/IRIS、第二套SFC、新Team optimizer、新状态机、multi-blocker 2^M topology tree、新fallback链、更多λ重试、通过降低clearance/放宽verifier或加budget掩盖错误。

### HEALTHY_TARGET_PIPELINE

```text
统一任务/地图/动态/peer快照 + 真实head PVA + activation/coverage
  → risk/conflict只授予N/L/R局部intent
  → static route/edge repair（需要时同一A*）
  → 认证Local-SFC + 明确piece/local-time映射和未覆盖域
  → PVA-aware P/T initializer（失败记数值，seed不拥有执行权）
  → 同一MINCO求解流程（现有LBFGS fast path / corridor SCP，修正导数与恢复）
       + visibility/LOS/合围软目标，不能消灭已有hard-safe incumbent
  → 冻结最终polynomial、activation、revision
  → 一次权威hard preflight：static/dynamic/swarm/PVAJ/lineage/checked_until
  → 仅在hard-safe集合中比较C3/D3/K2等任务质量
  → 提交同一payload（Team只能选择/回传Local realization）
  → executor identity/PVA activation
  → executed truth确认K3、实际coverage驱动下一次rolling
```

先给源码合同和数学修复定稿，再以离线解析/几何/边界用例验证，最后才有资格重新编译和运行验收。这是后续工作的建议，本轮未进行这些构建/运行。无需等待下一条failure label再增加分支。

## 14. 什么才叫 Local 健康

| 维度 | 必须可验证的健康标准 | 不能替代它的东西 |
|---|---|---|
| ARCHITECTURE | 每种事实有唯一语义owner；执行权仅final preflight授予；每个reject带具体对象/阶段/原因；quality/topology不得藏hard safety gate；任务拒绝不关闭必要安全供给 | LOS_HARD_REJECT_COUNT=0、某failure标签减少 |
| GEOMETRY | raw点与边、simplified连接、认证corridor、piece polynomial同坐标/同地图版本；每个必要leaf被覆盖；选定window覆盖真正piece；未覆盖区明确检查 | A*成功、plane数>0、打印subset=1 |
| NUMERICS | 固定真实PVA的seed尺度合理；时长/短段不会制造无谓病态；位置/时长/worldtime导数与被评价函数一致；SCP区分模型失败、trust限制、QP数值/不可行、deadline；连续checker通过解析极值/降阶/多piece/非均匀T/裁剪边界审查 | “再放大λ”、QP退出码0、checker与自身采样相互印证 |
| RUNTIME | 可执行successor在已认证prefix耗尽前完成并激活；统计ready/commit/activation各时刻；deadline单列运动/终点阶段、是否进入solver、耗时与reserve；可选质量工作让位且保留已有结果 | 每次deadline都靠initializer/terminal hold兜住、放大budget |
| SAFETY | 同一revision/activation/prediction/map/peer集合下static、dynamic、swarm、PVAJ谓词一致；checked_until不超过真实验证域；缺数据和已有违规不默许认证；执行payload与证书完全一致 | trajectory.safety_validated字段、wrapper exit=0、distance≤0才算swarm collision |
| VISIBILITY | C3/D3只比较同epoch下hard-safe最终revision；slack>0可存活；M2任务失败不取消Local安全；K3恢复使用绑定executor身份的实际position/yaw二值真值，并区分目标/障碍预测与实测 | planned margin过零、selected/committed就写RECOVERED、单周期必须全可见 |

建议后续验收的最小证据集合（不是新增runtime gate）：连续checker解析反例与quartic全部根覆盖；3D/斜段/voxel角/失败leaf/相邻corridor交集；固定非零head V/A和不同segment尺度；非均匀T更新的piece/window及dynamic worldtime导数；quality求解失败/预算不足保留原safe revision；end-to-end同id/revision/payload的preflight→commit→activate→executed witness。物理无解用例必须输出物理原因，预算/数值用例不得输出“NO_PATH”。

## 15. 最终状态

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

DESIGN_STATUS: SOUND_BUT_INCOMPLETE
IMPLEMENTATION_FIDELITY: NOT_PRECISE_MULTIPLE_MISMATCHES
LOCAL_ARCHITECTURE_STATUS: NEEDS_REFACTOR_AUTHORITY_AND_CONTRACTS

SFC_DESIGN_STATUS: NEEDS_REDESIGN
ASTAR_ROLE_STATUS: STATIC_CORE_SOUND_FRONTEND_CONTRACT_NOT_CLEAN
MINCO_TIMING_STATUS: PVA_AWARE_TIMING_REQUIRED_KEEP_MINCO
SCP_OSQP_STATUS: SCP_MODEL_AND_WRAPPER_NEED_FIX_NO_SOLVER_REPLACEMENT_EVIDENCE
CONTINUOUS_CHECKER_STATUS: WRONG_FALSE_POSITIVE_AND_FALSE_NEGATIVE
LOS_AUTHORITY_STATUS: NOT_CLEAN_DIRECT_CONDITIONAL_AND_INDIRECT_REJECTS
SIDE_TOPOLOGY_AUTHORITY_STATUS: PARTIALLY_CLEAN_HARD_TUBE_AND_LEGACY_GATES_REMAIN
FINAL_PREFLIGHT_STATUS: NECESSARY_BUT_INCONSISTENT_COVERAGE_AND_PREDICATES

TOTAL_REJECT_AUTHORITIES: 52_STAGE_PREDICATE_FAMILIES
TOTAL_FALLBACK_REPAIR_PATHS: 34_FAMILIES_32_REACHABLE_OR_CONDITIONAL_2_DORMANT
HIDDEN_HARD_REJECTS: 11_SOURCE_MECHANISM_FAMILIES_NOT_RUNTIME_COUNTS

ROOT_CAUSE_1: STATIC_ROUTE_SFC_PIECE_TIME_CONTRACT_BREAK
ROOT_CAUSE_2: CHECKER_TIME_JACOBIAN_AND_DIAGNOSTIC_ERRORS
ROOT_CAUSE_3: FIXED_PVA_TIMING_AND_SCP_FEASIBILITY_RESTORATION_MISMATCH

DESIGN_ISSUES: ARTIFICIAL_HARD_TOPOLOGY_QUALITY_VETO_MULTIPLE_FEASIBILITY_AUTHORITIES
IMPLEMENTATION_BUGS: CHECKER_SFC_PARTIAL_COVER_TIME_MAPPING_QP_STEP_GUARD_ID_COUNTER_COVERAGE
NUMERICAL_ISSUES: INVALID_DILATION_ASSUMPTION_SHORT_PIECES_WRONG_TIME_GRADIENTS_OVERCONSTRAINED_LINEARIZATION
EXPECTED_PHYSICAL_FAILURES: POSSIBLE_SPECIFIC_UNSAFE_TRAJECTORIES_NOT_PROVEN_MAIN_ROOT

WHAT_TO_KEEP: NLR_STATIC_ASTAR_LOCAL_SFC_MINCO_SCP_OSQP_SOFT_VISIBILITY_LOCAL_K3_TEAM_M2_EXECUTOR_LINEAGE
WHAT_TO_FIX: GEOMETRY_TIME_PVA_CONTRACTS_CHECKERS_WRAPPER_COVERAGE_IDENTITY
WHAT_TO_REMOVE: HIDDEN_QUALITY_TOPOLOGY_HARD_GATES_REDUNDANT_RETRIES_EXTERNAL_POLYNOMIAL_PROPOSAL
WHAT_TO_MERGE: SAFETY_PREDICATES_INITIALIZER_INCUMBENT_PREFIX_CERTIFICATE
WHAT_NOT_TO_ADD: NEW_PLANNER_SFC_TEAM_OPTIMIZER_STATE_MACHINE_TOPOLOGY_TREE_FALLBACK_CHAIN

HEALTHY_TARGET_PIPELINE: SNAPSHOT_INTENT_STATIC_ROUTE_CERTIFIED_SFC_PVA_TIMING_MINCO_FINAL_PREFLIGHT_QUALITY_COMMIT_ACTIVATE_EXECUTED_TRUTH

SHOULD_REFACTOR: YES
SHOULD_TUNE_PARAMETERS_NOW: NO
SHOULD_CHANGE_SFC: YES_EXISTING_GEOMETRY_AND_PIECE_TIME_CONTRACT
SHOULD_CHANGE_MINCO: TIMING_AND_INTERFACE_YES_QUINTIC_REPRESENTATION_NO
SHOULD_CHANGE_SCP: YES_TIME_JACOBIAN_ACTIVE_SET_FEASIBILITY_AND_AUTHORITY
SHOULD_CHANGE_OSQP: SOLVER_NO_WRAPPER_GUARD_AND_STATUS_YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
