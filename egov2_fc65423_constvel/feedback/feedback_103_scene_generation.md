# Feedback 103 — Natural Team Stress 场景生成与筛选

## 范围与复用

本轮只处理 ALP 场景生成、筛选和 FULL 运行验证，没有修改 planner 算法。复用了现有森林生成器 `ros_ws/src/multi_uav_formation/scripts/generate_occlusion_forest_scene.py`、静态路线校验器 `validate_target_route_static.py`、原生轨迹/可见性分析器 `analyze_native_egov2_metrics.py` 和正式入口 `run_on.sh` / `scripts/run_alp_full_on.sh`。

在现有生成器上做了小幅扩展：保持旧模式默认参数不变，增加 natural Team stress 模式、seed/小幅静态树保留率、批量 seed 预筛、自然 S 型 waypoint 生成和可选独立动态相位 seed。M2 预筛沿用当前 FULL 运行中的编队偏移样本和 visibility 风险函数，只作为候选排序 proxy；最终 Team 风险以 coordinator/runtime 日志为准。没有新建 runner、场景格式或单独分析器。

## 场景 gate 审计

- `scripts/lib/alp_params.sh` 从场景 JSON 读取 `nMovingObstacle`，并将同一值传入 `MOVING_OBJ_NUM`，这是预测器数据一致性要求。当前森林已有 10 个动态障碍，场景和 production 参数都使用 10。
- 未发现 production 或 run script 要求 `dynamic_obstacle_count == 8`、固定 waypoint 数量或固定森林障碍数。
- `static_los_wall_contract_test.cpp` 中 11 cylinders / 8 walls 的断言只在显式传入场景路径时检查生成的 B benchmark fixture；CMake 的常规测试调用不传路径。保留了这项 fixture 检查和静态 LOS 有效性检查，没有把它当作通用 scene gate。
- 没有删除或放宽任何 gate。

## 搜索方式与候选

源场景使用现有 `long_cylinder_forest.json` 的圆柱森林与动态障碍配置。第一阶段保持全部静态树，筛了 3,000 个 route/forest seeds；仅 5 个通过路线几何筛选，LOS proxy 没有找到兼具足够 crossing 数和舒适路线净空的 full-density 候选。另做 500 组动态相位筛选，没有得到更合适的 full-density 候选。随后以每棵既有静态树 95% 的保留概率筛了 10,000 个 route/forest seed 组合，7 个通过路线安全/形状预筛。

natural 模式沿原森林坐标生成低频、平滑的 S 型 target 路径，不按某几根树的位置摆障碍。动态障碍数量保持原场景的 10 个；中心、半径、高度、振幅、周期不变，仅由 seed 决定其现有运动相位。每条路径 26 个 waypoint，速度 0.928 m/s。独立相位 seed 只为复现实验及筛选使用，不改变 obstacle count。

本轮实际 FULL 的两个候选，以及未运行的另一个双 crossing 候选：

| seed | 静态树 | 路长 / 最大折角 | target 静态净空余量 | target–dynamic 最小表面净空 | LOS-only M2 proxy |
|---:|---:|---:|---:|---:|---|
| 20335853 | 34 | 79.414 m / 37.547° | 0.139704 m | 0.475847 m | 2 proxy crossings；最小 −0.849562 |
| 20335086 | 32 | 82.720 m / 59.132° | 0.137690 m | 0.303724 m | 2 proxy crossings；最小 −1.351798 |
| 20338582 | 33 | 74.668 m / 45.086° | 0.164940 m | 0.428963 m | 1 proxy crossing；最小 −0.434940 |

20335086 的 proxy 最低值更负、target–dynamic 净空只有 0.304 m，且路线折角更大，因此没有进入 FULL。20338582 有较温和的 proxy crossing、较宽的静态/动态路线净空，成为第二轮候选并最终入选。其路径通过现有静态校验；动态障碍布局、保留的静态树中心/半径/高度都来自原森林。图示保持散布森林与宽缓 S 型路线，没有门洞、障碍墙或专为单架 UAV 设计的遮挡：

![seed 20338582 森林与 target 路线](../runs/natural_team_stress_layout.png)

proxy 只计数 crossing 和记录 M2 数值，不判定 solver 可行性。候选 20335853 有两次 proxy crossing，但首轮 FULL 运行在 UAV1 轨迹记录中出现 43 个 moving clearance 为零的采样点，最小 moving clearance 为 0 m，并发生 1 次 `MOVING_SUCCESSOR_STARVATION` 和 1 次 `TERMINAL_HOLD_ENTER`，因此淘汰。对该路线另外扫描了 1,000 组相位；保留至少一个 proxy crossing 且 target 动态净空合格的 137 组，在首轮实际 UAV 轨迹回放下仍都有零 moving clearance，没有用相位变化把它转成更安全的运行候选。

## FULL 运行

运行均使用 `./run_on.sh --ablation full --timeout 240`。第一轮运行时 seed 20335853 位于 scene 输出路径；筛选后将该场景按 seed 复生成档于 `runs/natural_team_stress_candidate_20335853.json`。第二轮使用 `runs/natural_team_stress_candidate_20338582.json`；其 simulator-facing 字段与最终保存场景一致，之后只更新了 `naturalTeamStress.preScreen` 中的 proxy 字段名称/口径。两轮均到达 BOOT-12、target 和三架 UAV 开始移动，runner 最终退出码为 0，cleanup 正常。

| Run ID / seed | 运行时 M2 limiting contract | Team 修复结果 | 安全与可见性摘要 |
|---|---|---|---|
| `20260923_165537_566526` / 20335853 | 1 个 contract，limiting margin −1.170677 | PT PASS 0；3 个 Team refinement 结果均 `SOLVER_INFEASIBLE`；ADOPTED 0 | min static / moving clearance 0.308671 / 0 m；43 个 moving-clearance 零值；MIN_BUDGET 60，starvation 1，terminal hold 1；3-UAV 同时可见率 92.02% |
| `20260923_170933_586498` / 20338582 | **2 个浅层 coordinator M2 contract**，limiting margin −0.192729、−0.702163 | `TEAM_PT_ATTEMPT_COUNT=256`；合同相关 3 次 PT refinement 全为 `SOLVER_INFEASIBLE`；T/PT PASS 0；ADOPTED / ACTIVATED 均 0 | min static / moving clearance 0.302110 / 0.372670 m；clearance ≤0 样本 0；MIN_BUDGET、starvation、terminal hold、PVA mismatch、partial activation、unvalidated execution 均 0；3-UAV 同时可见率 94.67% |

最终候选的可见性 summary：UAV1/2/3 分别为 97.09% / 100.00% / 97.58%；2 架可见 130/2,439 个样本，1 架或 0 架可见为 0；最长 K2 loss 和 blackout 均为 0。静态路线验证 26 个 waypoint、0 处 target 静态相交、最小 target 额外净空余量 0.164940 m。实际轨迹中没有非正静态或动态 clearance，也没有 collision/swarm violation、PVA mismatch、部分 Team 激活或未验证执行标记。

3 条 `[RELAY_LATENCY]` 终态记录的 detect→receive 为 27.164 / 27.333 / 31.137 ms（p50 27.333 ms，max 31.137 ms）；三条终态均为 `SOLVER_INFEASIBLE`。

第二轮记录到 2 个浅层运行时 M2 contract，但修复未闭环：3 次 PT refinement 都以 `TRUE_CONSTRAINT_INFEASIBILITY` 失败。按 Feedback 099 的历史样本，deficit ≤0.230 曾成功，≥1.847 的样本均失败；本轮 solver 输入 deficit 为 0.703、1.307、2.159。前两个落在历史未定区间；2.159 来自 contract 2 的已声明候选 drone 0，达到既有 infeasible 区间，因此计为 1 个 deep-infeasible candidate event。限于 forecast margin 与 Local solve deficit 的语义不完全相同，不把这三次 solve 当成三个额外 M2 events。TEAM_T/PT PASS、ADOPTED、ACTIVATED 均为 0。因此此场景适合复现 relay / Team PT 风险触发与失败路径，尚不能作为 Team SCP 可恢复修复的通过样例。预筛 proxy 预测的单个风险时刻也没有精确预测 FULL 运行时刻，runtime contract 是最终统计口径。

## 结论

最终场景保留了自然散布的原森林和全部 10 个既有动态障碍，仅随机稀疏 3 棵原静态树并更换 waypoint 路线和动态相位。与首轮候选相比，选定场景保留了两次浅层 runtime M2 风险，同时没有复现动态障碍几何重叠或 Local restart starvation。当前主要未闭环项是 Team PT 的约束求解失败；本轮没有改动该算法。

生成器筛选报告：`runs/natural_team_stress_seed_screen_95pct.json`；最终生成报告：`runs/natural_team_stress_generation.json`。两轮 FULL 日志、CSV、manifest、exit status 和 `native_metrics.json` 分别位于相应 `runs/<RUN_ID>/` 目录。

```text
NEW_SCENE_FRAMEWORK_CREATED: NO
EXISTING_GENERATOR_REUSED: YES
EXISTING_ANALYSIS_REUSED: YES

OBSOLETE_SCENE_GATES_REMOVED:
NONE

FIXED_DYNAMIC_OBSTACLE_COUNT_REQUIREMENT_REMOVED: NOT_PRESENT

SEEDS_SCREENED: 13000 route/tree seeds; 1500 phase-only trials

FINAL_SCENE: /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/natural_team_stress.json

FULL_RUN_COMPLETED: YES

SHALLOW_TEAM_RISK_EVENTS: 2  # runtime coordinator M2 margins: -0.193, -0.702
DEEP_INFEASIBLE_EVENTS: 1  # one SCP candidate deficit 2.159 in Feedback099's known failed range
TEAM_T_PASS: 0
TEAM_PT_PASS: 0
TEAM_REFINEMENT_ADOPTED: 0

SAFETY_REGRESSION: NO  # final selected run; no physical clearance or execution-integrity violation
NATURAL_FOREST_STRUCTURE_PRESERVED: YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
