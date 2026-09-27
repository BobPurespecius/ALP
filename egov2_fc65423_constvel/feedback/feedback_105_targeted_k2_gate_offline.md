# Feedback 105 — 按实测 LOS 放置的 K2 targeted gate（离线筛选）

## 目的与边界

承接 Feedback 104 的自然森林 stress 场景。针对“障碍离目标太远、没有挡到有效视线”的问题，用 run `20260924_001344_694615` 的实测 UAV/target 轨迹和 visibility CSV 做固定轨迹几何筛选。本轮**没有运行仿真**；source run 本身也是被中止的部分轨迹，数据只到 `78.236254 s`。

本轮不再增加一批静态树，而是替换旧的 `targeted_los_gate_10`。旧 gate 在实测轨迹上没有截中任何当时可见的 UAV-target ray。候选 gate 沿末段 UAV1-target LOS 移动，目标是产生明确、可量化的 K2 loss。

## 反事实核对

从日志中的 range/FOV/static-LOS 分量出发，按目标起始时刻同步重建原有 10 个 moving obstacles，并排除旧 gate id 10。再加回旧 gate 后，重建可见性与日志 **6204/6204 个 UAV 样本逐项一致**；旧 gate 对基线可见射线的拦截数为 **0**。因此旧 gate 在这条实测轨迹上确实无效，不是可见性 CSV 口径造成的差异。

基线 K2（同时可见 UAV 数 ≥2）为 `1750/2068 = 84.62%`。这条轨迹中主要持续 K2=2 的窗口为 `11.034–12.065 s`（UAV2/3）、`25.298–26.467 s`（UAV2/3）和 `73.336–78.236 s`（UAV1/2）。前两个窗口的 UAV2 距 target 仅约 1.1 m，安全圆柱无法放入二者之间；候选搜索因此转向末段 UAV1-target 的视线走廊。

## 候选场景与离线结果

候选场景：[natural_team_stress_dense_38_targeted_k2_v2.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v2.json)。它仍是 38 棵静态树和 11 个 moving obstacles；只替换 id 10，没有增加无关障碍。

| 参数/指标 | 固定轨迹反事实结果 |
|---|---:|
| gate centerENU | `(34.802654, -0.248890) m` |
| axisENU | `(0.981921, -0.189292)` |
| amplitude / period / phase | `2.740426 m / 17.0 s / -2.908096 rad` |
| radius / height | `0.28 m / 3.6 m` |
| 旧场景 K2 | `1750/2068 = 84.62%` |
| 新 gate 后 K2 | `1626/2068 = 78.63%` |
| 新增 K2 loss | `124` 个样本，连续 `4.142 s`，`73.736–77.878 s` |
| target 最小表面净空 | `0.692 m`，`t=74.702 s` |
| UAV 最小表面净空 | `0.485 m`，UAV1，`t=71.876 s` |
| 最近静态树表面净空 | `0.483 m`，tree 15，`t=71.603 s` |
| 最近原有 moving obstacle 表面净空 | `2.557 m`，obstacle 9 |
| 地图边界最小净空 | `1.226 m` |

表面净空按候选圆柱物理半径扣除，针对记录的 target/UAV 中心轨迹和其他圆柱几何计算；不包含机体外形半径，也不表示 planner 已验证。净空都为正，但 UAV/静态树最近净空约 `0.48 m`，会与现有避障膨胀区发生交互。它是**值得后续验证的 targeted candidate**，不能据此宣称运行安全、Team trigger 必然触发、PT/T 必然通过或会 ADOPTED。

[候选布局图](../runs/natural_team_stress_dense_38_targeted_k2_v2_layout.png)展示全路线和末段视线；[离线筛选明细](../runs/natural_team_stress_dense_38_targeted_k2_v2_offline_screen.json)保存了几何重建、计数、时窗和净空数据。

## 结论与限制

- 旧 targeted gate：在可核对的实测轨迹上没有挡住有效可见射线，应淘汰。
- 新候选：不是在森林里盲目加树，而是替换成直接跨越观测射线的移动 gate；固定轨迹反事实新增约 4.14 s 的 K2 loss。
- 这个反事实假设 target/UAV 路径不变。换 gate 后 planner 与 target 路径会改变，真实 K2、Team contract、SCP 与安全活性结果均未验证。
- 按此前“不许再次仿真”的要求，本轮停在离线筛选；没有启动 ROS、Gazebo 或 RViz。
