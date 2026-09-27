# Feedback 7

## 结论摘要

本轮只读审计确认：当前生产源码中，SIDE 双侧求解、semantic-v2 主判据、A* downstream rejoin、Local-SFC zero-plane 语义、repair 弧长/曲率时间初始化、SCP rollback 和 stale-head 0.5 m commit gate 均已落入真实控制流；但动态安全与生命周期合同仍没有闭环，且当前 ALP **没有** visibility-aware SIDE ranking。

Visibility Stress V2 不能作为正式的“ALP topology 带来可见性优势”benchmark。它失败并非单一原因：全局遮挡预算不足；理论 observer 只描述 Gate 局部 5 s 窗口；Gradient 的真实 target trajectory 不同；当前 planner 不按 LOS/FOV/range 排名 PLUS/MINUS；V1 遮挡柱带又造成 UAV3 recovery 静态不可行/后端失败和长时间无有效 trajectory。UAV3 的 53.50% 可见性中，主要损失是 **32.87% out-of-range**，其次才是 **13.63% static LOS**，不是 FOV 或 dynamic LOS。

ALP 的两次静态碰撞也不是同一种机制：第一次是 INVALID nominal fallback 的 command 在进入柱体时有约 0.016 m 轻微穿入，odom 最深穿入 0.150 m；第二次 command 对该柱仍有 1.308 m clearance，而 odom 与 command 偏差约 2.54 m 后撞入柱体中心。场景使用的 planner cloud 只采样圆柱表面，内部为空洞 voxel，这使“几何圆柱碰撞”和“planner occupancy”并非同一合同。

## 审计基线与运行源码

- 工程：`/home/bob/ALP/egov2_fc65423_constvel`
- 审计对象：当前 dirty worktree；未 reset、clean、checkout、commit。
- 当前分支：`main`，HEAD `8cef241`。相关 planner/multi_uav_formation 文件存在大量已修改和未跟踪源码，因此旧 commit 只能作历史参照，结论全部基于当前文件。
- CMakeCache 确认实际 build source：
  - `ego_planner`: `.../tracking_ws/src/planner/plan_manage`
  - `traj_opt`: `.../tracking_ws/src/planner/traj_opt`
  - `path_searching`: `.../tracking_ws/src/planner/path_searching`
  - `plan_env`: `.../tracking_ws/src/planner/plan_env`
  - `multi_uav_formation`: `ros_ws/src/multi_uav_formation`
- 冻结场景 SHA256 重新计算为 `c7c28831b4306899b3305f4b4aace136a164191429749c8fc9417d5f7b27261a`，与运行元数据一致。
- 正确 V2 工件目录为 `scene_geometry_visibility_v2_20260902/visibility_benchmark/`；Native、Gradient、ALP 三轮 `run_meta.json` 均引用同一 scene hash。
- 离线只读重建脚本：`feedback_7_audit/audit_visibility.py`；结果：`feedback_7_audit/audit_metrics.json`。脚本未接触 ROS/planner 控制流。

### PROJECT MAP

1. `native_egov2_rviz.launch` 启动 scene、target coordinator 和三套 EGO planner；UAV nominal offsets 分别是 `(-1.5,-0.85)`、`(-1.7,0)`、`(-1.5,+0.85)`。
2. `native_egov2_rviz_scene.py` 读取 scene JSON，发布 `/map_generator/global_cloud`、moving obstacle marker/pose/prediction，并订阅三机 odom、PositionCommand、target odom 生成 trajectory/visibility CSV。
3. `target_state_coordinator.py` 等三机进入 TRACK 后生成动态避障分段 target speed，再发布 `/object_odom`。
4. `local_sensing_node/pcl_render_node` 从 global cloud 按 UAV 当前位置截取 5 m 球形 local cloud；没有前向 yaw 裁剪。
5. `plan_env/GridMap` 接收 local cloud，建立 0.1 m occupancy，实际 XY inflation 为 `ceil(0.099/0.1)*0.1 = 0.1 m`。
6. `EGOReplanFSM` 从当前 trajectory 或 odom 构造 planning head，调用 `EGOPlannerManager::reboundReplan()`。
7. manager 先求 NOMINAL，按动态风险触发 PLUS/MINUS，必要时执行 A*、semantic-v2、Local-SFC、repair timing、MINCO/SCP/OSQP。
8. manager 对 candidate 做三类安全分类、选择、previous persistence、post-check 和 stale-head gate，再写入 local trajectory/cache。
9. FSM 发布 `PolyTraj`；`traj_server` 接收后按绝对 `start_time` 每 0.01 s 生成 PositionCommand。
10. SO3 controller/simulator 执行 PositionCommand 并反馈 odom；scene node使用实际 odom/yaw计算碰撞、tracking和visibility。

对应源码：`native_egov2_rviz.launch:54-58,82-210`，`advanced_param.xml:120-135`，`run_in_sim.launch:132-139`，`simulator.xml:71-128`，`native_egov2_rviz_scene.py:140-177,230-343,375-559`。

## 设计逻辑与当前源码对照

| 设计要求 | 当前实现 | 分类 | 影响 | 证据 |
|---|---|---|---|---|
| `d_trigger=1.1+0.15=1.25 m` | clearance 与 margin 相加，`min_distance<trigger` | MATCH | 触发合同一致 | `planner_manager.cpp:740-742,781-782` |
| nominal/SIDE/previous 用统一 epoch | 初始 risk 均传 `planning_prediction_epoch` | MATCH | 同周期初始比较一致 | `planner_manager.cpp:684,1466-1474,2982,3275,3686,3735` |
| risk 覆盖可执行 trajectory | 非 touch-goal 先截为 2/3，再截到 2.0 s | CONTRACT_GAP | 3–6 s trajectory 尾段没有动态证明 | `planner_manager.cpp:735-739`; `poly_traj_optimizer.cpp:5282-5287` |
| prediction invalid 不 fail-open | 只要任一 obstacle 有 finite prediction 就令 risk valid；缺失 id 被跳过 | CONTRACT_GAP | 部分 prediction 缺失可被当成有效风险结果 | `planner_manager.cpp:751-766` |
| PLUS/MINUS 都求解 | preferred 仅决定调用顺序，随后无条件求另一侧 | MATCH | bilateral 已闭环 | `planner_manager.cpp:3940-3945` |
| PLUS=physical RIGHT，MINUS=LEFT | `normal=tangent.cross(Z)`；PLUS 要求正 lateral | MATCH | 符号语义一致 | `planner_manager.cpp:2168-2188` |
| semantic-v2 conflict-region 判据 | closest、distance weighted、ratio 0.60、eps 0.05 | MATCH | 旧 whole-path hard gate 未使用 | `planner_manager.cpp:2131-2135,2186-2217,2371` |
| semantic fallback 不伪装高置信 topology | 数据不足直接 `return true`，CandidateResult 不携带 confidence | CONTRACT_GAP | 明显 opposite topology 在缺数据时仍可进入后端 | `planner_manager.cpp:2137-2147,2159-2165,2171-2177,2203-2209` |
| 第一个 A* SUCCESS 不阻止 rejoin | 最多三个 rejoin，只有 `raw_path_usable` 才 break | MATCH | usable/static/semantic 合同一致 | `planner_manager.cpp:2241-2249,2287-2348` |
| A* path state 不 stale | 每次 trial 用局部 `trial_path`，只保存本次 first success | MATCH | 未发现跨周期 stale path | `planner_manager.cpp:2287-2329` |
| zero-plane SFC 合法 | required=false 且空 plane => `SFC_NOT_REQUIRED`、valid | MATCH | zero-plane 可 handoff | `planner_manager.cpp:2542-2650` |
| 真 SFC failure 仍 reject | invalid input/NaN/required-but-empty 单独 invalid | MATCH | 没有把构造错误当成功 | `planner_manager.cpp:2542-2631` |
| repair 按弧长/曲率配时 | `eta_v=eta_a=0.8`，segment length、curvature、vertex speed 决定 T | MATCH | index 均分已不在 repair path | `planner_manager.cpp:2667-2775` |
| repair 增长后 downstream time 后移 | shift=`new_repair_duration-old_window` | MATCH | 几何/时间连续 | `planner_manager.cpp:2784-2829` |
| SCP P/T trust 分离 | P 用 meter trust；T 用 relative physical-time trust | MATCH | 两类 trust 未混用 | `poly_traj_optimizer.cpp:1745-1754` |
| rejected trial rollback 同一 base | failure 恢复 base P、virtual T、real T 并 regenerate | MATCH | retry_same_base 真实生效 | `poly_traj_optimizer.cpp:1936-1948,2375-2393,2402-2410` |
| MAX_ITER 不等于约束无解 | max_iter 分类为 numerical/retry exhausted；primal infeasible另做 no-trust probe | MATCH | 错误码语义正确 | `poly_traj_optimizer.cpp:1807-1909` |
| no-trust/model agreement 影响控制流 | probe 决定 grow/shrink/TRUE_INFEASIBLE；model agreement决定 accept/reject | MATCH | 非 diagnostic-only | `poly_traj_optimizer.cpp:1814-1882,2090-2345` |
| failed candidate 不残留 success | CandidateResult 默认 false，每次 cycle 局部新建，optimize 结果覆盖 | MATCH | 未发现 stale success=true | `planner_manager.h:55-79`; `planner_manager.cpp:1417-1419,3653` |
| post-check 改轨迹后重算 risk/class | dynamics/static/swarm retry会改写 `committed_MJO`，之后直接 freshness/commit | CONFIRMED_BUG | commit trajectory 的 class/risk可对应修改前轨迹 | `planner_manager.cpp:4297-4334`; retry 实际赋值见 `planner_manager.cpp:885-1015` |
| ABS/IMPROVED/INVALID 阈值 | ABS>=1.1；SIDE improvement>=nominal+0.10；nominal不可 IMPROVED | MATCH | 核心三类判据一致 | `planner_manager.cpp:1428-1457` |
| previous ABS 防 unsafe 覆盖 | 新 candidate 非 ABS 时保留 previous ABS | MATCH | IMPROVED/INVALID 保护成立 | `planner_manager.cpp:4142-4160,4207-4214` |
| previous ABS 不被更差 same-class 覆盖 | 任何新 ABS 都可越过 previous；previous 不参与同级 clearance 比较 | CONTRACT_GAP | 可出现 ABS margin erosion | `planner_manager.cpp:3949-3978,4142-4145` |
| non-triggered nominal 不静默降级 SIDE | non-triggered 默认 nominal；只要 nominal ABS 就可替换 previous SIDE ABS | CONTRACT_GAP | 安全余量/拓扑可被周期性重置 | `planner_manager.cpp:1492-1506,4142-4145` |
| unsafe fallback 不进入 previous-safe/warm cache | 所有成功 commit 均无条件写 cache；cache无 safety_class字段 | CONFIRMED_BUG | INVALID fallback 会成为 warm seed并被日志称为 `USED_PREVIOUS_SAFE` | `planner_manager.h:195-200`; `planner_manager.cpp:546-632,4434-4444` |
| nominal solve失败仍考虑 previous/SIDE | nominal `flag_success=false` 立即 return | CONFIRMED_BUG | NOMINAL backend failure会绕过 previous revalidation和双侧 recovery | `planner_manager.cpp:1390-1405` |
| odom position/expiry resync 与FOV解耦 | position error和expiry均无FOV gate | MATCH | 已实现通用位置同步 | `ego_replan_fsm.cpp:251-259,723-743` |
| velocity resync 与FOV解耦 | velocity error仍要求 `enable_fov_tracking_` | CONTRACT_GAP | FOV OFF 时纯速度失配不会触发 odom start | `ego_replan_fsm.cpp:251-253,736-738` |
| commit stale-head gate在最终轨迹后 | post-check 后取最终 `committed_MJO.getTraj().getPos(0)`，>0.5 return false | MATCH | stale candidate不会进入本次 commit/cache | `planner_manager.cpp:4333-4388` |
| prediction epoch与实际 start一致 | risk使用cycle-start epoch，commit后 start_time设为now | CONTRACT_GAP | 典型差约28 ms，最大1.240 s；head fresh不等于dynamic epoch fresh | `planner_manager.cpp:727-750,509-533,4337-4367` |
| traj_server拒绝旧generation/start | 只校验order/coeff数量，随后无条件覆盖 | LIKELY_RISK | 旧/重复消息具备覆盖能力 | `traj_server.cpp:47-88` |
| retained previous 不产生伪“新发布” | manager保留previous时return true，FSM仍重新发布相同PolyTraj | LIKELY_RISK | 重复receive/activation；当前同start_time未证明回滚 | `planner_manager.cpp:4230-4236,4280-4295`; `ego_replan_fsm.cpp:660-675` |
| target-facing yaw 与path visibility分层 | yaw仅在traj_server算PositionCommand yaw | MATCH | 只解决朝向，不改变轨迹 | `traj_server.cpp:150-190,245-283` |
| visibility进入SIDE ranking | ranking仅用safety class、dynamic clearance、side hysteresis | DESIGN_NOT_IMPLEMENTED | planner不会因open LOS主动选侧 | `planner_manager.cpp:3949-4024` |
| static scene几何与planner map同口径 | scene cloud仅采样圆柱表面，GridMap只膨胀点；圆柱内部为空洞 | CONFIRMED_BUG | 几何 evaluator判“在柱内”时planner voxel可能为free | `native_egov2_rviz_scene.py:90-104,140-143`; `grid_map.cpp:818-880` |
| static checker覆盖被执行轨迹 | 非 touch-goal 只检查前2/3 | CONTRACT_GAP | replanning失败时未检查尾段仍可能被执行 | `planner_manager.cpp:840-863` |
| latency统计口径统一 | benchmark解析optimizer内部 `total_t(ms)`，freshness记录cycle epoch→commit | DIAGNOSTIC_ONLY | 0.4 ms与27–49 ms并不矛盾，但不可混作同一latency | `analyze_benchmark.py:202-213`; `planner_manager.cpp:4360-4368` |
| visibility loss cause互斥 | benchmark离线脚本对range/FOV/static/dynamic分别累加 | DIAGNOSTIC_ONLY | 同一样本会被多重归因，和runtime early-return顺序不同 | `analyze_benchmark.py:168-197`; runtime顺序 `native_egov2_rviz_scene.py:443-464` |

## 已确认的代码偏差

按生产安全优先级，而不是按本次场景的表面数值排序：

1. **Post-check reclassification缺失**：post-check retry会改变最终 P/T，但没有对最终轨迹重新计算 dynamic risk、class与selection contract。这是直接的安全合同破口。
2. **Accepted cache没有safety class**：INVALID emergency fallback只要commit就进入cache；后续日志和warm-start语义把它称为previous safe。cache也无法阻止旧obstacle context进入新周期。
3. **Nominal failure提前return**：previous ABS与SIDE都在此return之后，故nominal solver failure会绕过本应存在的恢复链。
4. **动态安全时间未闭环**：risk同时受2/3 executable-prefix和2 s horizon截断；prediction epoch又没有平移到实际commit/start time。
5. **Same-class/非trigger覆盖缺少previous margin比较**：新ABS即使1.11 m也可替代previous 2.0 m，non-triggered nominal同样如此。
6. **Static cylinder cloud是空心壳**：planner map并不表达scene JSON声明的实心圆柱体积；offline collision与planner occupancy合同不一致。
7. **Velocity resync仍受FOV gate控制**，且traj_server没有local generation/start monotonic gate。
8. **Visibility-aware path selection未实现**：这不是隐藏bug，而是明确缺失的功能层。

本轮没有证据证明 semantic-v2 主判据、A* rejoin、zero-plane SFC 或 arc-length timing 的当前实现与既定主合同不一致；semantic fallback的confidence语义除外。

## Visibility Stress V2 场景尸检

### 全局可见性预算

ALP visibility采样时长为95.102 s。若希望整场可见率降到：

| 目标可见率 | 至少需要遮挡时间 |
|---:|---:|
| 90% | 9.510 s |
| 85% | 14.265 s |
| 80% | 19.020 s |

场景设计说明称每Gate约1.7 s，三Gate总预算约5.1 s，理论最大整场下降仅5.36个百分点。即使使用离线5 s窗口内的理论blocked ratio累加，UAV1/UAV2/UAV3也只有约8.614/10.396/8.713 s，对应整场理论可见率下限约90.94%/89.07%/90.84%。因此该几何从全局预算上就不足以稳定制造“三机均低于90%，两机75–85%”的正式stress。

VISIBILITY_STRESS_GLOBAL_BUDGET_SUFFICIENT: NO

### 理论 LOS 与运行 LOS 差异

离线表的30–44%是每个Gate局部5 s窗口中、固定nominal formation observer的visibility，不是整场visibility。运行时还存在四个偏差：

1. 实际UAV不一定经过nominal observer。Native/Gradient尚能在Gate局部显示明显遮挡；ALP UAV3在V1后则完全脱离代表区域。
2. 理论设计用`LOS margin=0.15 m`，runtime evaluator用`radius+0.08 m`；口径相差0.07 m。
3. 设计审计用`uav_inflated_radius=0.65 m`判断bypass，而生产GridMap实际只有约0.1 m point-cloud inflation，且cloud是圆柱壳。
4. target coordinator按每种方法进入TRACK时的动态障碍相位重新选速度，Gradient的实际target时空轨迹不同。

THEORETICAL_TO_RUNTIME_VISIBILITY_MISMATCH_ROOT_CAUSE: 局部5秒理论指标被当作整场预算；实际observer/target时序不同；runtime LOS和planner static geometry口径不同。

### 三方法局部表现

| Gate | Method | UAV1 vis | UAV2 vis | UAV3 vis | 关键事实 |
|---|---|---:|---:|---:|---|
| V1 | Native | 46.0% | 22.0% | 60.7% | static band确实产生局部遮挡 |
| V1 | Gradient | 81.3% | 10.7% | 46.0% | observer轨迹不同，UAV2/UAV3受遮挡 |
| V1 | ALP | 31.5% | 95.3% | 47.7% | UAV3开始偏离并进入恢复失败链 |
| V2 | Native | 82.0% | 54.7% | 39.3% | 局部stress存在 |
| V2 | Gradient | 100% | 49.0% | 60.3% | target时序/observer不同 |
| V2 | ALP | 100% | 43.3% | 0% | 139/150为out-of-range，11/150才是static LOS主判因 |
| V3 | Native | 49.3% | 46.0% | 100% | band只覆盖两架实际observer |
| V3 | Gradient | 48.3% | 38.4% | 100% | UAV3未受stress |
| V3 | ALP | 48.3% | 39.6% | 100% | UAV3恢复后走到无遮挡侧，但非visibility ranking证据 |

这说明场景局部遮挡几何并非完全无效，但它没有形成持续的三机全局stress，也没有把“open LOS机会”与当前planner实际selection objective连接起来。

## ALP UAV3 可见性与静态碰撞时间线

### 可见性原因拆分

对原始`alp_visibility.csv` 2854个样本按runtime evaluator的真实判定顺序（range→FOV→occlusion）重建：

| 原因 | 样本 | 全程占比 |
|---|---:|---:|
| Visible | 1527 | 53.50% |
| Out-of-range | 938 | 32.87% |
| Static LOS | 389 | 13.63% |
| FOV primary | 0 | 0% |
| Dynamic LOS primary | 0 | 0% |

旧汇总中的FOV 4.75%、static LOS 18.22%、dynamic LOS 4.70%是三机所有不可见样本的**非互斥**离线重算；它允许同一样本同时记入range、FOV和LOS。对UAV3，runtime最先命中的主因是range/static，而不是FOV/dynamic LOS。

最长不可见段：

- static LOS：31.629–34.195 s，2.566 s；35.063–35.762 s，0.700 s；36.130–45.196 s，9.066 s。
- out-of-range：45.229–76.462 s，31.233 s。
- static LOS：76.495–76.996 s，0.500 s。

### 因果时间线

| Relative time | UAV3/target状态 | Planner/command事件 | 因果判断 |
|---:|---|---|---|
| 28–31 s | UAV3从(-9.68,3.05)接近V1，target约(-8.29,1.78)→(-7.3,0.6) | repeated SIDE static infeasible；previous ABS逐步降至1.1附近 | V1 band开始挤压实际recovery，而非纯LOS统计 |
| 30.629 s | previous clearance降至1.0997 | PLUS A* path 5.580 m/29 pieces；初值a=25.89、jerk=431.09，retime后jerk仍29.25；进入SFC/SCP | repair timing虽按新公式，但此几何仍产生病态高曲率初值 |
| 30.746–32.9 s | UAV3在V1入口附近低速/停顿 | 多次`NO_STATIC_FEASIBLE_SIDE`；INVALID nominal fallback被commit | scene static band开始造成真实可飞/后端压力 |
| 32.70–33.21 s | odom在快速抬高/偏离 | 两次长solve 0.551/0.510 s后分别STALE_HEAD；第二次head error 1.154 m | stale gate正确拒绝，但没有及时可执行replacement |
| 35.153–58.340 s | UAV3基本停在(-8.08,0.96,2.65)，target持续前进 | 24次`local trajectory expired, replan from odom`；traj_server过期分支不再publish command | 直接形成长期tracking failure和后续out-of-range |
| 45.229 s | target已离UAV3超过8 m | visibility进入连续31.233 s out-of-range | V2的UAV3=0%在到达V2前已注定 |
| 58.689 s | planner终于选trajectory 153，UAV3重新移动 | 后续大量INVALID nominal fallback | recovery恢复，但UAV3落后约20 m |
| 59.996–61.295 s | 穿过V1 band_1 | static collision episode 1 | command入口轻微碰柱，odom偏差扩大穿透 |
| 76.563–76.930 s | UAV3追赶至x≈19 | static collision episode 2 with cylinder_33 | command安全，odom显著落后并撞柱 |
| 80 s | UAV3追到target附近 | V3 UAV3 visibility 100% | 不是visibility-aware topology成功证据 |

日志证据：`alp.launcher.log:19040-19055`（V1 A*/repair），`:19121-19276`（drone2 NO_STATIC_FEASIBLE_SIDE），`:21687,22164`（stale-head），`:25584-47156`（24次expiry），`:47385`（trajectory 153恢复）。

ALP_UAV3_LOW_VISIBILITY_PRIMARY_CAUSE: TRACKING_FAILURE → OUT_OF_RANGE，次因STATIC_LOS；分类为MIXED，但主导项是OUT_OF_RANGE。

## ALP 两次静态碰撞尸检

### Episode 1 — visibility_V1_band_1

- UAV：UAV3 / planner drone_id=2。
- 时间：59.995788–61.295411 s；40 samples。
- 最深点：60.662017 s，odom=(-7.747355,0.271386,2.927096)，raw surface penetration=0.149623 m。
- 目标：target=(12.388816,2.25,1.5)，UAV-target distance=20.283 m。
- active trajectory 175：NOMINAL，duration 4.714111 s，predicted dynamic clearance 0.981229 m。
- 三类日志明确把NOMINAL、PLUS、MINUS、previous全部标为INVALID，并以`UNSAFE_FALLBACK` commit trajectory 175；见`alp.launcher.log:48710-48837`。
- command在episode入口附近对同柱最小surface clearance约-0.016 m；在最深odom时command已经离柱，command–odom偏差约1.0 m量级。
- 因而不是纯`COMMAND_SAFE_ODOM_COLLIDES`：command本身有边缘静态穿入，execution error扩大了持续时间/深度。
- scene cloud确实包含该occluder，但只包含圆柱表面；planner occupancy与实心几何不等价。

分类：**INVALID_FALLBACK + STATIC_CHECKER/MAP_REPRESENTATION_MISMATCH + EXECUTION_TRACKING amplification**。

原始数据：`alp_trajectory.csv:5461`，`alp_visibility.csv:1821`，`alp.launcher.log:48740-49159`。

### Episode 2 — long_forest_cylinder_33

- 时间：76.562784–76.930244 s；12 samples。
- 最深点：76.731465 s，odom=(18.984545,-3.198854,3.543492)，penetration=0.534503 m，几乎位于r=0.55柱中心。
- target=(25.393173,0.688611,1.5)，UAV-target distance=7.769 m。
- active trajectory 243：NOMINAL，ABSOLUTE_SAFE（dynamic clearance 4.791784 m），head freshness=0.175 m并正常commit。
- 同一时段command对cylinder_33最小clearance约+1.308 m；command–odom差约2.54 m。
- 这是明确的`COMMAND_SAFE_ODOM_COLLIDES`。由于柱体内部没有occupancy points，UAV一旦进入圆柱壳内部，planner static query还可能重新看到“free interior”。

分类：**COMMAND_SAFE_ODOM_COLLIDES**，空心static map合同为加重因素。

原始数据：`alp_trajectory.csv:6907`，`alp_visibility.csv:2303`，`alp.launcher.log:65570-65649`。

ALP_STATIC_COLLISION_ROOT_CAUSE: MIXED；Episode 1为INVALID fallback/静态地图口径/跟踪共同导致，Episode 2为COMMAND_SAFE_ODOM_COLLIDES。

OCCLUDER_NOT_IN_PLANNER_MAP: NO；它在global cloud中，但以空心surface shell表达。

## Target Trajectory 公平性

三轮配置target_speed均为0.928 m/s，但`target_state_coordinator.py`在三机ready后才调用`prepare_dynamic_safe_speeds(stamp)`。每段障碍查询时间使用：

`stamp - motion_start_time + accumulated_target_elapsed + segment_fraction*duration`

因此方法启动/进入TRACK的时刻变化会改变dynamic obstacle phase，再改变选中的segment speed。源码：`target_state_coordinator.py:140-161,196-250`。

真实speed set：

- Native/ALP：`[0.45472, 0.56608, 0.67744, 0.91872, 0.95584]`
- Gradient：`[0.43616, 0.60320, 0.64032, 0.88160, 0.91872]`

按共同`time_s`重采样target position：

| Pair | RMS position difference | Max difference |
|---|---:|---:|
| ALP vs Native | 0.015 m | 0.032 m |
| ALP vs Gradient | 0.258 m | 0.687 m |
| Native vs Gradient | 0.257 m | 0.683 m |

V1/V2/V3设计encounter time约30.056/40.834/74.507 s；Native/ALP实际target encounter约31.762/47.331/80.962 s，V2/V3偏移约6.5 s。Gradient又与二者不同。

STRICT_TARGET_TRAJECTORY_MATCH: NO

TARGET_SPEED_MISMATCH_ROOT_CAUSE: target分段速度优化绑定“各方法进入TRACK时刻相对scene motion_start的相位”，而不是预先冻结的统一绝对target schedule；callback/启动顺序使Gradient得到不同速度与p_T(t)。

## 根因排序

### 当前生产代码不符合既定逻辑

| 项目 | 排序 | 说明 |
|---|---|---|
| post-check后不rerisk/reclass | PRIMARY | 最终commit P/T与class可不对应 |
| INVALID fallback进入无class warm cache | PRIMARY | safe cache语义不成立 |
| nominal solve failure提前return | PRIMARY | 可绕过previous/SIDE完整恢复链 |
| 2/3 + 2 s dynamic horizon与旧epoch | PRIMARY | 完整执行轨迹动态安全未闭环 |
| same-class margin erosion/non-trigger overwrite | SECONDARY | previous ABS保护只覆盖unsafe class |
| empty-shell static map | PRIMARY for static benchmark correctness | planner/static evaluator合同不一致 |
| visibility-aware ranking不存在 | DESIGN_NOT_IMPLEMENTED | open LOS只是一种机会，不是planner objective |
| semantic fallback无confidence | SECONDARY | fallback可接纳wrong topology |
| velocity resync FOV gate/traj_server无monotonic gate | CONTRIBUTING RISK | 本轮未证明为首发原因 |

### Visibility Stress V2 失败

| 候选原因 | 排序 | 证据 |
|---|---|---|
| 三个短Gate的全局遮挡预算不足 | PRIMARY | 5.1 s只能使95.1 s任务下降约5.36个百分点 |
| ALP没有visibility-aware topology ranking | PRIMARY | selection无LOS/FOV/range字段 |
| V1 band造成UAV3 static infeasibility/recovery collapse | PRIMARY for ALP anomaly | 24次expiry，随后31.23 s out-of-range |
| theoretical observer与真实UAV不一致 | SECONDARY | ALP UAV3完全脱离nominal observer；V3实际UAV3 100% |
| target trajectory三方法不一致 | SECONDARY | Gradient target max差0.687 m，Gate时序不严格公平 |
| scene/planner static geometry口径不一致 | SECONDARY | 0.65 m设计inflation vs 0.1 m shell-point inflation；实心/空心不同 |
| Gate phase与实际encounter错位 | CONTRIBUTING | V2/V3约+6.5 s，dynamic challenge时序改变 |
| visibility cause分析非互斥 | CONTRIBUTING diagnostic error | 把V2 UAV3的range样本同时记为static/FOV |
| dynamic LOS本身造成UAV3 53.5% | NOT_CAUSAL | exclusive primary count为0 |
| target-facing yaw不足 | NOT_CAUSAL as primary | UAV3 loss窗口relative bearing不是主限制 |

## 最小后续建议

### 代码问题

1. 在所有post-check retry完成后，对最终`committed_MJO`重新做dynamic risk、static/dynamics checks、三类classification，并重新执行previous protection；不要沿用修改前class。
2. accepted cache增加safety class、prediction/obstacle context；INVALID fallback不得进入“previous-safe”cache，日志名称同步修正。
3. 把nominal failure纳入统一CandidateResult，继续previous revalidation和SIDE，而不是提前return。
4. 明确完整动态合同：至少覆盖实际可执行remaining prefix，并将prediction epoch对齐实际trajectory start/activation。
5. static cloud改为实心occupancy或在GridMap中做solid cylinder contract；offline evaluator与planner inflation用同一半径/高度/voxel定义。

### 场景问题

1. 不要继续通过微调pillar位置“刷ALP visibility”。在visibility-aware ranking实现前，open LOS topology没有被planner利用。
2. 若未来重做formal stress，先按整场时长分配每架UAV所需9.5/14.3/19.0 s遮挡预算；局部Gate报告与整场指标分开。
3. 用生产static occupancy/inflation做三机可飞性验证，不能只用代表lane与0.65 m抽象圆盘。

### 指标/公平性问题

1. 预先冻结并回放同一target p_T(t),v_T(t)，不要在每种方法启动后按实时dynamic phase重新选segment speed。
2. visibility原因按runtime early-return顺序输出互斥primary cause，同时可保留多标签overlap作为附表。
3. 分开报告optimizer kernel latency和planning epoch→commit latency；本轮ALP分别为median 0.391 ms/P95 2.774 ms/max 24.558 ms，以及median 27.882 ms/P95 46.501 ms/max 1239.748 ms。

## Final Fields

PROJECT_RUNTIME_SOURCE_CONFIRMED: YES

CURRENT_DESIGN_CONTRACT_UNDERSTOOD: YES

SIDE_BILATERAL_LOGIC_MATCH: YES

SEMANTIC_V2_LOGIC_MATCH: NO

ASTAR_REJOIN_LOGIC_MATCH: YES

LOCAL_SFC_LOGIC_MATCH: YES

REPAIR_TIME_INITIALIZATION_MATCH: YES

THREE_CLASS_SELECTION_MATCH: PARTIAL

PREVIOUS_SAFE_CACHE_LOGIC_MATCH: NO

STALE_HEAD_LOGIC_MATCH: YES

POST_CHECK_RECLASSIFICATION_CORRECT: NO

FULL_DYNAMIC_HORIZON_CLOSED: NO

VISIBILITY_AWARE_SIDE_RANKING_IMPLEMENTED: NO

PRIMARY_CODE_DEVIATIONS:
1. Post-check修改最终P/T后没有重新计算dynamic risk和safety class。
2. INVALID fallback可进入不携带safety class的accepted/warm cache；nominal failure还会提前绕过previous/SIDE。
3. 动态风险只覆盖非terminal轨迹前2/3且最多2 s，并使用早于实际commit/start的planning epoch。
4. Static planner map以空心圆柱表面点表达实心scene cylinder。
5. Visibility-aware SIDE/path ranking未实现。

VISIBILITY_STRESS_GLOBAL_BUDGET_SUFFICIENT: NO

THEORETICAL_RUNTIME_VISIBILITY_MISMATCH: YES

ALP_UAV3_LOW_VISIBILITY_PRIMARY_CAUSE: TRACKING_FAILURE → OUT_OF_RANGE（32.87%），次因STATIC_LOS（13.63%）

ALP_STATIC_COLLISION_ROOT_CAUSE: MIXED；Episode1=INVALID_FALLBACK + STATIC_MAP/CHECKER_MISMATCH + tracking amplification，Episode2=COMMAND_SAFE_ODOM_COLLIDES

STRICT_TARGET_TRAJECTORY_MATCH: NO

TARGET_SPEED_MISMATCH_ROOT_CAUSE: target safe-speed schedule依赖各方法ready/start时刻相对dynamic motion_start的相位，Gradient因此生成不同segment speeds和p_T(t)

THREE_UAV_VISIBILITY_STRESS_ACTUALLY_CREATED: NO

VISIBILITY_SCENE_VALID_AS_FORMAL_BENCHMARK: NO

PRIMARY_VISIBILITY_SCENE_FAILURE: 全局遮挡预算不足，且当前ALP没有visibility-aware selection；ALP异常值又被V1 recovery collapse/out-of-range污染

SECONDARY_VISIBILITY_SCENE_FAILURE: theoretical observer/生产static geometry/target时序不一致，occluder band还诱发静态不可行与tracking collapse

BENCHMARK_FAIRNESS: FAIL

NEXT_MINIMAL_CODE_ACTION: post-check最终轨迹rerisk/reclass并给accepted cache加入真实safety class；随后消除nominal early return

NEXT_MINIMAL_SCENE_ACTION: 暂停继续移动visibility occluder；先冻结严格一致的target trajectory，并在生产static occupancy合同下重新做整场三机遮挡预算

PRODUCTION_CODE_CHANGED: NO

SCENE_CHANGED: NO

PREVIOUS_MAX_FEEDBACK_INDEX: 6

CURRENT_FEEDBACK_FILE: feedback_7.md
