# 120°合围 + 最大化团队可见时间：Scenario A 实际效果

实验日期：2026-09-10  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
场景：`long_cylinder_forest_visibility_stress.json`（Scenario A，LOS/FOV 与 moving-obstacle 压力）  
场景 SHA-256：`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`

## 实验条件

OFF 使用固定 tracking reference、原有 planner，并关闭 cooperative encirclement、team visibility selection/optimizer 及 joint P/T/yaw。ON 使用当前生产代码的 120° soft encirclement、cooperative φ0/hypothesis、directional visibility、team candidate selection、joint P/T/yaw、Stage 2 continuity/blackout 与 Stage 3A/3B。场景、目标轨迹、moving-obstacle、初始状态、camera FOV、动力学和安全阈值保持一致。没有访问或修改 `/home/bob/RRCT`。

Build：`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2`，6 个包成功，failed=0。

## 每次有效运行结果

| RUN | MODE | MEAN_VISIBLE | ACCUMULATED_CAMERA_VISIBLE_TIME (camera·s) | K2 | ALL3 | NONE | LONGEST_K2_LOSS (s) | LONGEST_BLACKOUT (s) | TRACKING_MEAN (m) | TRACKING_P95 (m) | MIN_SWARM_DISTANCE (m) | MIN_STATIC_CLEARANCE (m) | MIN_DYNAMIC_CLEARANCE (m) | PLANNING_P95 (ms) | COLLISION_SAMPLES | EMERGENCY_STOPS | HEARTBEAT_LOSSES |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| off_1 | OFF | 2.849595 | 222.917938 | 0.960375 | 0.889220 | 0.000000 | 1.565256 | 0.000000 | 2.055392 | 4.555419 | 0.151147 | 0.139305 | 0.609080 | 9.236040 | 0 | 12 | 0 |
| off_2 | OFF | 2.936941 | 229.771984 | 0.991478 | 0.946314 | 0.000852 | 0.666388 | 0.067186 | 1.584147 | 1.984365 | 0.575971 | 0.206784 | 0.202060 | 1.391813 | 0 | 0 | 0 |
| off_3 | OFF | 2.929302 | 229.261466 | 0.994889 | 0.934412 | 0.000000 | 0.399697 | 0.000000 | 1.590964 | 2.244538 | 0.562497 | 0.199811 | 0.858786 | 1.532391 | 0 | 0 | 0 |
| on_1 | ON | 2.559438 | 200.228377 | 0.979122 | 0.580315 | 0.000000 | 1.163743 | 0.000000 | 0.641812 | 1.860456 | 0.543895 | 0.217537 | 0.000000 | 4.575718 | 16 | 10 | 0 |
| on_2 | ON | 2.771295 | 216.917292 | 0.985520 | 0.785775 | 0.000000 | 0.665950 | 0.000000 | 0.580387 | 1.287865 | 1.000000 | 0.129079 | 0.000000 | 4.574198 | 5 | 12 | 0 |
| on_3 | ON | 2.686542 | 210.271267 | 0.988927 | 0.697615 | 0.000000 | 0.431011 | 0.000000 | 0.498318 | 1.415309 | 1.000000 | 0.000000 | 0.398841 | 4.357530 | 11 | 0 | 0 |

## 组统计（均值 ± 样本标准差；min–max）

| 指标 | OFF | ON |
|---|---:|---:|
| mean_visible | 2.905279 ± 0.048375 (2.849595–2.936941) | 2.672425 ± 0.106632 (2.559438–2.771295) |
| accumulated visible time (camera·s) | 227.317129 ± 3.818353 (222.917938–229.771984) | 209.138979 ± 8.401876 (200.228377–216.917292) |
| K2 | 0.982248 ± 0.019019 | 0.984523 ± 0.004978 |
| All3 | 0.923316 ± 0.030121 | 0.687902 ± 0.103074 |
| None | 0.000284 ± 0.000492 | 0.000000 ± 0.000000 |
| longest K2 loss (s) | 0.877114 ± 0.610685 | 0.753568 ± 0.374141 |
| longest blackout (s) | 0.022395 ± 0.038790 | 0.000000 ± 0.000000 |
| tracking mean (m) | 1.743501 ± 0.270127 | 0.573506 ± 0.071994 |
| tracking P95 (m) | 2.928107 ± 1.415284 | 1.521210 ± 0.300627 |
| planning P95 (ms) | 4.053415 ± 4.488836 | 4.502482 ± 0.125534 |
| collision samples | 0 | 10.667 |
| emergency stops | 4.000 | 7.333 |
| heartbeat losses | 0 | 0 |

`MIN_SWARM_DISTANCE` 由轨迹逐时刻最近两机距离计算：OFF 组为 0.429872 m（min 0.151147，max 0.575971），ON 组为 0.847965 m（min 0.543895，max 1.000000）。

## OFF→ON 组均值变化

- `mean_visible`：2.905279 → 2.672425，绝对变化 **-0.232855**，相对 **-8.015%**。
- `accumulated_camera_visible_time`：227.317129 → 209.138979 camera·s，绝对变化 **-18.178150 camera·s**，相对 **-7.997%**。
- K2：0.982248 → 0.984523，**+0.002275**（+0.232%）。
- All3：0.923316 → 0.687902，**-0.235414**（-25.497%）。
- longest K2 loss：0.877114 → 0.753568 s，缩短 0.123546 s（-14.085%）。
- longest blackout：0.022395 → 0，缩短 0.022395 s。

ON 的 tracking mean/P95 明显下降，swarm distance 组最小值提高；但执行轨迹出现 collision samples（3 次 ON 均非零）以及 static/dynamic clearance 触及 0 的运行，故存在安全回归，不能把可见性结果解释为安全改进。

## 结论

核心累计可见时间和 mean_visible 均下降，因此：

```text
MAXIMIZED_VISIBLE_TIME_IMPROVED: NO
ENCIRCLEMENT_PLUS_VISIBILITY_EFFECTIVE: NO
K2_IMPROVED: YES (轻微)
ALL3_IMPROVED: NO
LONGEST_K2_LOSS_IMPROVED: YES
BLACKOUT_IMPROVED: YES
VISIBLE_TIME_IMPROVED_WITH_CONTINUITY_TRADEOFF: NO
SAFETY_REGRESSION_OBSERVED: YES
```

本结论基于完整有效运行区间的 executed visibility，而不是单一 team-solution event。6/6 运行均完成 target replay；没有删除失败运行，也没有增加额外运行。

```text
OFF_RUNS_VALID: 3
ON_RUNS_VALID: 3
MEAN_VISIBLE_OFF_MEAN: 2.905279
MEAN_VISIBLE_ON_MEAN: 2.672425
MEAN_VISIBLE_RELATIVE_IMPROVEMENT: -8.015%
ACCUMULATED_VISIBLE_TIME_OFF_MEAN: 227.317129 camera·s
ACCUMULATED_VISIBLE_TIME_ON_MEAN: 209.138979 camera·s
ACCUMULATED_VISIBLE_TIME_RELATIVE_IMPROVEMENT: -7.997%
K2_OFF_MEAN: 0.982248
K2_ON_MEAN: 0.984523
ALL3_OFF_MEAN: 0.923316
ALL3_ON_MEAN: 0.687902
LONGEST_K2_LOSS_OFF_MEAN: 0.877114 s
LONGEST_K2_LOSS_ON_MEAN: 0.753568 s
LONGEST_BLACKOUT_OFF_MEAN: 0.022395 s
LONGEST_BLACKOUT_ON_MEAN: 0 s
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
