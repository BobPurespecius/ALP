# Feedback 108 — 自然森林 Team stress 地图生成过程与结果

## 目标

从已有森林场景出发，构造一张保留自然树列结构、目标路线可通行、并能在实际运行中产生可核对 LOS/K2 影响的 Team stress 地图。筛选标准逐步从“静态障碍数量和离线 proxy”收紧为“实际运行轨迹上命中 UAV—target 射线，并检查 K2、Team 事件和安全净空”。本次只整理场景，不改 planner、安全约束或 runner。

## 地图如何生成

### 1. 复用自然森林，不新造地图框架

起点是已有 [long_cylinder_forest_visibility_stress.json](../ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json) 与 [generate_occlusion_forest_scene.py](../ros_ws/src/multi_uav_formation/scripts/generate_occlusion_forest_scene.py)。生成器的 `--natural-team-stress` 模式用独立 seed 生成 target S 路线、静态树稀疏化/补充和动态障碍相位；新增自然树沿原森林的纵向范围与树行分布抽样，不读取 target 路线。候选先做路线净空、障碍几何和 LOS-only Team M2 proxy 筛选，再用 FULL 运行检验。

基础 38 树候选使用 scene/route seed `20402929`、静态稀疏 seed `21402929`、动态相位 seed `22402929`。它保留原森林 36 棵静态树并补 2 棵，总路线 26 个 waypoint、约 71.878 m；目标路线最大转角约 22.613°，最大横向偏移约 3.088 m。

### 2. 先增加密度，再用运行射线检查有效性

早期按树数搜索 40/39/38 棵布局：40 棵没有通过净空门槛的候选，39 棵仅有 1/12,000 几何候选且 proxy 深 crossing，均淘汰。38 棵布局有 4/15,000 个通过筛选，选出的 proxy 最低 M2 为 `-0.104232`，target 静态额外净空约 `0.097619 m`。该图能触发 Team contract，但最初运行 PT/T PASS 与 ADOPTED 均为 0。

之后固定路线和动态相位，从 38 增加到 44 棵，测试确实改变了 proxy，但 FULL 轨迹上新增 6 棵树命中 UAV—target LOS 的次数为 **0**。这说明“树更多”或“树离路线更近”不能代替真实视线检查。44 棵布局未被保留为精准 K2 场景。

### 3. 改为按实际 UAV—target 射线定点放置

按已有 run 的实际 UAV、target、visibility 轨迹计算圆柱与视线线段的相交；同时检查 target route、UAV 轨迹、既有静态树和动态障碍扫掠净空。之后每次实际运行都重新检查命中数，固定轨迹反事实只作为预筛，不视为运行结论。

| 迭代 | 场景方法与实际发现 | 结论 |
|---|---|---|
| targeted v2–v4 | 移动 gate 固定轨迹预筛预测出 K2 窗口，但 v3 实际轨迹未命中 gate；v4 出现 0.230039 s K2-loss，来源为既有静态树与 moving obstacle 9，新增 gate 仍未命中 | 证明要从运行轨迹复核，不能用 proxy 代替 |
| v5，run `20260924_161801_1172559` | 加入两个按旧 run 视线选择的静态圆柱。id36 命中 UAV1 的 21 条射线；id37 命中数为 0；K2-loss 为 0 | 保留 id36，重新放置无命中的 id37 |
| **v6，run `20260924_163326_1175833`** | 将 id37 移到新 run 的晚段 UAV3—target 射线。实际命中 UAV3 射线 23 次，其中 16 次与 K2-loss 窗口重合 | **本次实际运行产生 0.527694 s K2-loss；多次复跑稳定性尚未验证** |

## 最终地图 v6

场景文件：[natural_team_stress_dense_38_targeted_k2_v6.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json)。它包含 38 个静态圆柱和 11 个动态圆柱（原森林动态障碍加 targeted gate id10）。两个定点静态圆柱为：

| ID | ENU 中心 (m) | 半径 / 高度 (m) | v6 实际 LOS 命中 |
|---|---:|---:|---:|
| 36 | `(-25.920, -3.730)` | `0.30 / 3.60` | UAV1：21 次，约 `8.149–8.816 s` |
| 37 | `(32.842, 1.067)` | `0.28 / 3.60` | UAV3：23 次，约 `73.649–74.382 s`；其中 16 次落在 K2-loss 窗口 |

target route 静态校验为 26 个 waypoint、0 个相交；全图最小目标静态净空为 `0.097619 m`。v6 布局与关键瞬间射线图：

![v6 自然森林地图与实际 K2-loss 射线](../runs/natural_team_stress_dense_38_targeted_k2_v6_layout.png)

## 实际运行结果

v6 使用 FULL headless 运行，BOOT-12 成功、`FINAL_EXIT_CODE=0`。2348 个可见性样本覆盖 78.248 s；UAV1/2/3 可见率为 `94.63% / 98.55% / 97.19%`。最长 K2-loss 为 `0.527694 s`，无全队 blackout。单机可见的 16 个样本位于 `73.887839–74.382185 s`。

在 K2-loss 快照 `t=74.015453 s`，UAV1 可见；moving gate id10（`targeted_k2_gate_10_v4`）遮挡 UAV2 的 dynamic LOS；新静态 id37 遮挡 UAV3 的 static LOS。即 id37 在真实轨迹上直接挡线，并与既有动态遮挡共同令可见数由至少 2 降为 1。

Team 层有 3 个 contract、128 次 PT attempt、1 次 PT PASS 和 1 次 ADOPTED；该 PASS/ADOPTED 属于较早的 contract 1。与晚段 K2-loss 同时的 contract 3 对 STATIC 与 DYNAMIC_LOS 候选做了 PT 尝试，结果均为 `SOLVER_INFEASIBLE`。所以 v6 是**已验证的实际 K2-loss 与 Team 活动 stress 地图**，不是晚段 K2-loss 被 Team 成功修复的样例。

安全与活性计数：最小静态/移动轨迹净空 `0.316780 / 0.302913 m`，非正净空样本 0；`MIN_BUDGET_APPLIED`、terminal hold、successor starvation、PVA mismatch、partial activation、unvalidated execution 均为 0。24,555 条轨迹记录都标记为 `safety_validated=true`。

## 复跑和日志

```bash
./scripts/run_alp_full_on.sh --ablation full --headless --timeout 240 --boot-timeout 180 \
  --scenario /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json
```

本次 `ROS_LOG_DIR` 为 `runs/20260924_163326_1175833/ros_log`，manifest 标记为持久化 run 目录、tmpfs disabled。runner 登记的 29 个自有进程全部退出，强杀 0 个，也没有杀其他进程。

## 可追溯文件

- 本总结：[Feedback 108](feedback_108_scene_map_build_process.md)
- 密度生成与 44 棵筛选：[Feedback 104](feedback_104_dense_team_stress.md)
- 实际射线 gate 离线筛选：[Feedback 105](feedback_105_targeted_k2_gate_offline.md)
- v3/v4 Team 与 K2 运行对照：[Feedback 106](feedback_106_targeted_k2_team_runs.md)
- v6 实际圆柱命中与安全结果：[Feedback 107](feedback_107_runtime_targeted_k2_cylinder.md)
- 预筛与运行复核数据：[v6 screen](../runs/natural_team_stress_dense_38_targeted_k2_v6_screen.json)
- 正式运行证据：[exit status](../runs/20260924_163326_1175833/exit_status.txt)、[visibility summary](../runs/20260924_163326_1175833/visibility_summary.csv)、[cleanup record](../runs/20260924_163326_1175833/process_status.txt)、[run manifest](../runs/20260924_163326_1175833/run_manifest.txt)
