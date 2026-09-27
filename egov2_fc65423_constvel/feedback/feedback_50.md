# All3 损失根因审计、候选排序修复试验与单次 FULL ON 验证

日期：2026-09-12  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
承接：`feedback_49.md`

## 1. 最终结论

Run1 / Run2 / Run5 的高 K2、低 All3 不是一个单一权重问题。38 个
`All3: 1 -> 0` 且 K2 保持 1 的事件累计 28.052913 s，第三架失视的
直接物理原因以 static LOS 为主；同时存在一个源码级本地候选排序
缺陷：安全、blackout、K2 等价时，包含已饱和方向分散度的
`team_utility` 和微小 `diversity` 差值排在 mean-visible/All3 前面。
因此系统多次已经拥有更高第三架可见时间的安全候选，却选择了较低
mean-visible/All3 的候选。

本轮只针对这个简单、证据闭合的排序合同做了最小试验修复；没有调
权重，也没有把 All3 变成 hard gate。构建、梯度/合同和受控 topology
测试全部通过，试验运行中的同类错误选择从 92 次降到 0。

但是唯一一次 FULL ON 物理验收失败：两架 UAV 在约 40 s 后先后失去
可执行 successor 并进入 terminal hold，All3、K2、Q_dir、合围与连续性
同时显著退化。该失败模式在 `feedback_49.md` 的 Run3/4/5 已经存在，
但唯一一次新运行无法证明排序改动没有参与具体路径演化。依据“不为
All3 牺牲 K2、连续运动或安全”，本轮试验性 production 改动已完整
回退；没有启动第二次运行。

```text
ALL3_LOSS_ROOT_CAUSE: STATIC_LOS_DOMINANT + LOCAL_VISIBILITY_ORDERING_DEFECT
ALL3_OPTIMIZATION_PHYSICALLY_VALIDATED: NO
PRODUCTION_FIX_RETAINED: NO
FULL_ON_RUNS_THIS_ROUND: 1
SIMULATION_LEFT_RUNNING: NO
```

## 2. Run1 / Run2 / Run5 的逐事件审计

可复核产物：

- `all3_visibility_optimization_20260912/analyze_all3_loss_events.py`
- `all3_visibility_optimization_20260912/baseline_good_run_events.json`

事件定义是前一样本三机可见、当前样本恰有两机可见，并持续统计到
visible count 不再为 2。原因按 `static LOS -> dynamic LOS -> range ->
HFOV -> VFOV` 做独占时间归类，避免一份时间重复计入多个原因。

| 直接原因 | 累计时间 | 占 28.052913 s |
|---|---:|---:|
| static LOS | 16.866024 s | 60.12% |
| dynamic LOS | 5.471918 s | 19.51% |
| HFOV | 1.674803 s | 5.97% |
| VFOV | 4.006359 s | 14.28% |
| range | 0.033809 s | 0.12% |

按运行分别有 Run1=21、Run2=7、Run5=10 个事件。关键入口结果：

```text
EVENT_COUNT = 38
TRIGGER_WITHIN_PREVIOUS_3P5S = 37/38
SAFE_SIDE_WITHIN_PREVIOUS_1P5S = 34/38
EVENTS_STARTING_ON_TERMINAL_HOLD = 0/38
EVENTS_STARTING_ON_UNVALIDATED_EXECUTION = 0/38
```

所以三次好运行中的主要解释不是 shared-risk-support 没有触发，也不是
SIDE 总是生成失败，更不是 lifecycle 已先失效：绝大部分事件开始时仍
在执行已验证轨迹，且失视前已有 trigger 和安全 SIDE。问题集中在：

1. static/dynamic obstacle 的观察拓扑本身仍会使第三架短时失视；
2. 安全 SIDE 虽存在，但路线不一定足以消除实际 LOS/FOV loss；
3. 更高第三架可见时间的本地候选会被旧排序压制；
4. 少数事件确实没有及时形成安全 SIDE，不能由排序修复解决。

### 2.1 明确的本地排序缺陷

旧 `planner_manager.cpp::visibility_better` 在 none、K2、K-of-N 后按以下
顺序比较：

```text
team_utility (含 0.15 * diversity)
-> diversity
-> min pairwise angle
-> weakest-camera visibility
-> All3
```

三次好运行重建出 92 次如下决策：被选候选与另一个候选相比 K2 不高、
none 不低、mean-visible/All3 更差，但因 `team_utility/diversity` 更高而被
选中。其中 21/38 个 All3-loss onset 的前 1.5 s 内出现过至少一次这种
选择。

典型真实记录：

```text
selected NOMINAL:  K2=1, mean=2.761905, All3=0.761905, diversity=1.000000
available SIDE-:   K2=1, mean=3.000000, All3=1.000000, diversity=0.992707
old decision: BASELINE_ALREADY_VISIBILITY_BEST
```

这不是要求固定 120°，也不是破坏 Q_dir；两个候选都已经通过当前
revision 的 hard safety，且 K2 相同。缺陷只是把几乎饱和的 spread 微差
放在真实 camera-time 增益前面。

### 2.2 joint/global selector

日志中还能找到少量 selected mean-visible 低于 near-best-K2 集合最大值
的 global joint snapshot，但当前 telemetry 没有记录每个组合完整的
recovery-target 和 geometry cost。`TopologyCoordinatorCore` 会在
visibility 前保护 safety、recovery 和 25°/170° geometry；没有证据证明
这些 global 决策是错误过滤。因此本轮没有改 coordinator/joint selector，
避免用 All3 越权覆盖 Q_dir 或合围恢复。

## 3. 试验修改及回退

试验只改过：

`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`

逻辑为：

```text
hard safety/executable set unchanged
-> none/blackout protection unchanged
-> exact K2/K-of-N protection unchanged
-> equal K2 时 mean-visible、All3 优先
-> weakest-camera / max-loss
-> spread / min-pairwise-angle 作为后续 tie-break
```

实现复用现有 `betterTeamVisibilityWithinNearBestK2` 合同，局部调用使用
K2 tolerance=0，所以不允许用 All3 换取任何 K2 下降。没有修改：

- safety clearance 或 final current-revision preflight；
- J_vis deep-risk、Stage2/3A/3B、joint P/T/yaw；
- shared-risk-support trigger、SIDE/A*/Local-SFC；
- 25°/170°、Q_dir 或 recovery geometry；
- planner brake/stop、coordinator、ACK/commit；
- 任何 All3/K2/blackout 经验触发阈值或 utility weight。

试验运行中 `LOCAL_SELECTED_LOWER_MEAN_VISIBLE_COUNT` 从历史 92 降为 0，
证明代码合同按预期生效。但物理验收不通过，故上述 include 和 comparator
变动均已回退。最终 production 行为与本轮开始时一致；本轮最终新增的
持久文件只有审计脚本、JSON、运行证据和本报告。

## 4. Build、gradient、contract 与受控测试

试验补丁状态：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2
All 6 requested/dependency packages succeeded, no failures.
```

通过的相关测试：

- topology coordinator：16 项 PASS，包含 safety-first、near-best K2 内
  mean-visible 优先、spread 不能压过 camera time、single combination；
- team visibility optimizer：P、tau、piece boundary、K2、blackout、
  K2-continuity、J_acc、P/T/yaw 等 analytic vs finite-difference PASS；
- multiview：25°/170° 原定义、permutation、完整 MINCO P/T/yaw gradient、
  camera regression protection PASS；
- encirclement geometry、cooperative viewpoint、local execution PASS；
- Elastic visibility：deep-risk non-decaying extension PASS；
- visibility topology production：8 项 PASS，包含 loss 前 shared-risk trigger、
  双侧 dispatch、target-side/mirror、simple-side fail 后 A*/Local-SFC/MINCO
  rescue、统一 final preflight、loss 前 activation。

回退后再次执行 `catkin build ego_planner -j2`，5 个请求/依赖包 PASS、无
warning/failure；并复跑 topology、team optimizer gradients、multiview、
Elastic 以及受控 production topology，全部 PASS。`git diff --check` 和
审计脚本 Python compile 均 PASS。

## 5. 唯一一次 FULL ON

运行：

`scenario_a_20260911_234722/all3_visibility_20260912_run1`

分析：

- `all3_visibility_optimization_20260912/analysis/run1/all3_visibility_20260912_run1_metrics.json`
- `all3_visibility_optimization_20260912/analysis/run1/all3_loss_events.json`

场景 SHA256 保持
`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`；
FULL ON、native RViz、Stage2/3A/3B、joint P/T/yaw、deep-risk J_vis、
shared-risk topology 全开。target 80.204541 s 路线正常完成，launcher
退出码 0。

### 5.1 与 Run1/2/5 好运行均值比较

| 指标 | Run1/2/5 均值 | 本轮唯一 run | 判定 |
|---|---:|---:|---|
| camera visible time | 227.483786 camera-s | 160.343858 | 退化 |
| mean visible | 2.837703 | 1.999910 | 退化 |
| K2 | 0.985196 | 0.589127 | 严重退化 |
| All3 | 0.852768 | 0.429505 | 未提高 |
| longest K2 loss | 0.611448 s | 31.407941 s | 严重退化 |
| longest blackout | 0.020931 s | 0.767737 s | 退化 |
| actual Q_dir mean | 0.972633 | 0.817232 | 退化 |
| executed encirclement | 0.729853 | 0.265226 | 严重退化 |
| same semicircle | 0.183559 | 0.647536 | 退化 |

本轮三机 LOS/FOV loss（各机 loss 可重叠，不能相加解释 wall time）：

```text
STATIC_LOS_LOSS_TOTAL  = 21.673154 s
DYNAMIC_LOS_LOSS_TOTAL = 31.001289 s
HFOV_LOSS_TOTAL        = 40.164664 s
VFOV_LOSS_TOTAL        = 0.733005 s
```

在 K2 尚保持的 12 个新 All3-loss event 中，直接原因是 static LOS
4.842072 s、dynamic LOS 4.399335 s、HFOV 1.589041 s、VFOV 0.733005 s；
12/12 已提前 trigger，11/12 已有近期安全 SIDE，试验 comparator 的错误
选择为 0。之后两机 terminal hold 使 K2 本身丢失，已超出“第三架短时
失视”问题。

### 5.2 joint 闭环仍真实存在

```text
MULTIVIEW_JOINT_ATTEMPT_COUNT = 43
MULTIVIEW_JOINT_SUCCESS_COUNT = 15
MULTIVIEW_COMMIT_COUNT = 13
THREE_ACK_COUNT = 13
THREE_UAV_ADOPTION_COUNT = 13
COMPLETE optimizer->proposal->3ACK->commit->3adoption->optimized-yaw chains = 13
OPTIMIZED_YAW_EXECUTED solution IDs = 13
joint planning time ms mean/p50/p95/max = 2.029/0/6.1423/12.078
```

所以本轮失败不是 coordinator/ACK/commit 全部失效；完整 joint/yaw 执行
在 terminal hold 前已经多次发生。

### 5.3 continuity 与 safety

```text
MOVING_SUCCESSOR_STARVATION_COUNT = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT = 0
TERMINAL_HOLD_COUNT = 2
UNVALIDATED_EXECUTED_SAMPLES = 2381

COLLISION_SAMPLES = 0
SWARM_CLEARANCE_VIOLATION_SAMPLES = 0
MIN_STATIC_CLEARANCE = 0.118013 m
MIN_DYNAMIC_CLEARANCE = 0.418030 m
MIN_SWARM_DISTANCE = 0.913653 m
```

没有观测到物理碰撞或 clearance violation，但 terminal hold 后的执行
不再绑定当前安全证书，故执行合同层仍判安全/连续性回归。

第一个直接断点：

```text
drone2: last valid successor id=146, activation ~= mission 38.391 s,
        duration=1.5 s; subsequent nominal dynamic clearance fell below the
        1.1 m hard admission and both SIDE seeds were dynamically invalid;
        previous safe suffix was retained until expiry; terminal hold at 39.896 s.

drone1: last valid successor id=152, activation ~= mission 39.673 s,
        duration=1.5 s; repeated nominal/side candidates remained invalid;
        terminal hold at 41.197 s.
```

这与 feedback_49 Run3/4 的“两机 hold 后长时间 unvalidated execution”同类，
不是通过降低 static/dynamic/swarm 阈值可以修的。本轮范围是 All3，且用户
要求第一次不安全/不连续时不要盲目重复，因此没有继续改 lifecycle、
recovery 或 safety，也没有启动第二次仿真。

## 6. 最终状态

```text
ALL3_LOSS_ROOT_CAUSE:
  STATIC_LOS_DOMINANT;
  SAFE_SIDE_USUALLY_AVAILABLE;
  LOCAL_TEAM_UTILITY/DIVERSITY_ORDER_CAN_SUPPRESS_HIGHER_ALL3;
  EXISTING_TERMINAL_HOLD_INSTABILITY_BLOCKS_REPEATABLE_VALIDATION

ALL3_INCREASED: NO
K2_MAINTAINED: NO
Q_DIR_MAINTAINED: NO
ENCIRCLEMENT_MAINTAINED: NO
CONTINUITY_REGRESSION_OBSERVED: YES
PHYSICAL_COLLISION_OR_CLEARANCE_REGRESSION: NO
EXECUTION_SAFETY_CONTRACT_REGRESSION: YES

SIMPLE_ORDERING_DEFECT_IDENTIFIED: YES
ORDERING_FIX_CONTRACT_PASS: YES
ORDERING_FIX_PHYSICAL_ACCEPTANCE_PASS: NO
PRODUCTION_FIX_RETAINED: NO

FULL_ON_RUNS_THIS_ROUND: 1
SECOND_RUN_STARTED: NO
PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO
SAFETY_THRESHOLDS_CHANGED: NO
J_VIS_CHANGED: NO
STAGE2_3A_3B_CHANGED: NO
JOINT_COORDINATOR_CHANGED: NO
RRCT_FILESYSTEM_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

下一步若继续，不应再靠单一排序试验直接跑整场；应先把 feedback_49 和
本轮共同复现的 terminal-hold successor 失效链独立隔离，否则 All3
candidate policy 的物理收益会被约 40 s 后的 lifecycle 崩塌完全淹没。

```text
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_50.md
```
