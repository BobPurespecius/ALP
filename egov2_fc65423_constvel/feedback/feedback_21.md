# Feedback 21 — 初始森林三方式可见性历史运行精确匹配

日期：2026-09-04  
审计方式：只读搜索历史运行工件；未修改源码、参数或场景；未启动新仿真。

## 1. 匹配结论

给定指纹精确命中：

| 指标 | Gradient 指纹 | 历史聚合值 | ALP 指纹 | 历史聚合值 |
|---|---:|---:|---:|---:|
| UAV1 visibility | 98.01% | 98.01% | 97.82% | 97.82% |
| UAV2 visibility | 98.19% | 98.19% | 99.37% | 99.37% |
| UAV3 visibility | 98.13% | 98.13% | 99.40% | 99.40% |
| All-3 visibility | 95.26% | 95.26% | 96.88% | 96.88% |
| None-visible | 0% | 0.00% | 0% | 0.00% |

对应历史实验为：

- 场景：`long_cylinder_forest.json`（初始森林地图）。
- 日期：2026-09-01。
- 配置：target-facing yaw ON，RViz OFF。
- 重复次数：Native 2 次、Gradient 2 次、ALP 4 次。
- 截止口径：检测目标到 `(36.0, 0.25)` 且速度为零连续 6 个样本后截止；排除截止后的 WAIT_TARGET/hover。
- 三组全部到达最终目标，全部进程退出码为 0。

匹配根目录：

`/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901`

核心证据：

- `SPEED50_REPEATABILITY_AND_METRICS_REPORT.md`：直接包含完整指纹表。
- `speed50_repeatability_20260901/repeatability_metrics.json`：逐轮与分组聚合数据。
- 各运行目录中的 `*_visibility_summary.csv`、`*_visibility.csv`、`*_trajectory.csv`、`*.launcher.log`、`run_meta.json`。
- `analyze_speed50_repeatability.py`：历史聚合口径实现。

## 2. 三方式完整聚合对比

下表均为“每种方法的重复运行均值”；括号为该方法各轮的 `[min, max]`。百分比指纹因此是重复运行均值，不是一条单独运行。

| Metric | Native (n=2) | Gradient (n=2) | ALP (n=4) |
|---|---:|---:|---:|
| Task complete | 2/2 | 2/2 | 4/4 |
| UAV1 visibility | 97.67% [97.09, 98.25] | 98.01% [97.38, 98.63] | 97.82% [97.42, 98.01] |
| UAV2 visibility | 98.46% [98.42, 98.50] | 98.19% [98.17, 98.21] | 99.37% [98.21, 100.00] |
| UAV3 visibility | 96.90% [96.88, 96.93] | 98.13% [98.09, 98.17] | 99.40% [97.59, 100.00] |
| All-3 visible | 94.10% [93.27, 94.93] | 95.26% [94.52, 96.01] | 96.88% [94.14, 98.01] |
| At-least-2 visible | 98.94% [98.71, 99.17] | 99.07% [99.00, 99.13] | 99.70% [99.09, 100.00] |
| At-least-1 visible | 100.00% | 100.00% | 100.00% |
| None-visible | 0.00% | 0.00% | 0.00% |
| Tracking mean (m) | 1.549 [1.474, 1.625] | 1.458 [1.455, 1.461] | 1.443 [1.430, 1.455] |
| Tracking P95 (m) | 2.094 [1.655, 2.533] | 1.631 [1.626, 1.635] | 1.659 [1.626, 1.730] |
| Tracking max (m) | 4.352 [2.721, 5.984] | 2.145 [2.099, 2.191] | 2.171 [2.091, 2.359] |
| Mean path length (m) | 77.012 [76.489, 77.535] | 76.391 [76.137, 76.645] | 77.253 [76.622, 77.523] |
| Minimum dynamic clearance (m) | 0.000 [0.000, 0.000] | 0.000 [0.000, 0.000] | 0.200 [0.076, 0.395] |
| Minimum static clearance (m) | 0.210 [0.206, 0.214] | 0.207 [0.195, 0.218] | 0.186 [0.178, 0.195] |
| Dynamic collision episodes/run | 2.00 [2, 2] | 3.00 [3, 3] | 0.00 [0, 0] |
| Dynamic collision samples/run | 33.5 [31, 36] | 40.5 [38, 43] | 0 [0, 0] |
| Static collision episodes/run | 0 | 0 | 0 |
| Static collision samples/run | 0 | 0 | 0 |
| Planning latency median (ms) | 0.302 [0.294, 0.310] | 0.391 [0.389, 0.393] | 0.389 [0.382, 0.392] |
| Planning latency P95 (ms) | 0.659 [0.614, 0.705] | 0.773 [0.755, 0.790] | 0.780 [0.768, 0.791] |
| Planning latency max (ms) | 32.072 [8.273, 55.871] | 10.113 [2.419, 17.807] | 20.574 [16.602, 23.416] |

## 3. 历史执行平滑度

这些数据来自 trajectory CSV 中 UAV odometry twist 的有限差分。表值先在每轮内对 3 架 UAV 求均值，再对重复运行求均值；括号是运行间 `[min, max]`。

| Executed metric | Native | Gradient | ALP |
|---|---:|---:|---:|
| Velocity max (m/s) | 2.823 [2.370, 3.275] | 1.825 [1.798, 1.853] | 2.357 [2.147, 2.510] |
| Acceleration max (m/s²) | 8.255 [6.088, 10.423] | 6.211 [6.196, 6.226] | 6.905 [5.769, 7.607] |
| Jerk RMS (m/s³) | 37.069 [23.356, 50.782] | 20.226 [19.086, 21.366] | 25.747 [23.906, 27.263] |
| Jerk P95 (m/s³) | 43.664 [42.940, 44.389] | 44.374 [43.080, 45.668] | 55.045 [50.674, 59.696] |
| Jerk max (m/s³) | 931.638 [304.593, 1558.683] | 188.416 [184.283, 192.549] | 225.640 [170.573, 247.341] |

完整 active MINCO 多项式系数和 piece partition 未保存在这组历史工件中，因此 planned analytic velocity/acceleration/jerk 为 N/A。上述 executed finite-difference jerk 不能与规划器 `max_jer=22` 直接比较。

## 4. 指标口径说明

- Visibility：读取原始 `*_visibility.csv`，只保留 target-stop cutoff 前样本；UAV 单机为 `visible_uav*` 均值，All-3/At-least-2/None 由 `visible_count` 统计。
- Tracking mean/P95：trajectory CSV 的 `centroid_target_distance_m`，在 cutoff 前全部 UAV 样本上统计。
- Path length：各 UAV odom 位置相邻采样距离求和，再取三机均值；只接受 `1e-5 < dt <= 0.5 s` 的相邻段。
- Dynamic/static clearance：直接使用 trajectory CSV 的 `moving_clearance_m` 和 `static_clearance_m`。碰撞判定为 clearance `<= 0`；该历史 CSV 的动态碰撞深度被钳制到 0，因此 Native/Gradient 的 `0.000 m` 表示发生接触/穿入，不能解释为真实 penetration depth 恰好为零。
- Collision count：主表使用 collision episode/run；同时单列 collision samples/run，二者不能混用。
- Planning latency：从 launcher log 中的 `total_t(ms)` 解析；是该日志字段口径，不等同于后来报告中的 planning epoch→commit latency。
- Smoothness：executed odometry finite difference；不是 planned MINCO analytic derivative。

## 5. 对应运行目录

### Native

- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/native_1`
- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/native_2`

### Gradient

- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/gradient_1`
- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/gradient_2`

### ALP

- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_1`
- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_2`
- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_3`
- `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_4`

## 6. 最终字段

MATCHED_RUN_FOUND: YES

MATCHED_RUN_DIRECTORY_NATIVE: `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/native_1`, `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/native_2`

MATCHED_RUN_DIRECTORY_GRADIENT: `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/gradient_1`, `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/gradient_2`

MATCHED_RUN_DIRECTORY_ALP: `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_1`, `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_2`, `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_3`, `/home/bob/ALP/egov2_fc65423_constvel/speed50_repeatability_20260901/alp_4`

CURRENT_FEEDBACK_FILE: `/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_21.md`
