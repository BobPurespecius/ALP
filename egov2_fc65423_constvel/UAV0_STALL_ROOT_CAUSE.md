# ALP uav0 停摆根因追踪（FIRST REAL DIVERGENCE）

运行：`runs/20260919_143444_142602` / `captures/20260919_143444_142553`（0.2 s 抽帧，1194 帧）
工作区：`/home/bob/ALP/egov2_fc65423_constvel`

---

## 0. 先纠正一个我自己的判读错误

`visibility_trajectory.csv` 的 `uav_id` 由 `native_egov2_rviz_scene.py:1167`
写成 `index + 1`，而 odom 订阅是
`'/drone_{}_visual_slam/odom'.format(index)`（同文件 `:516`）。
所以 **`uav_id=1` 就是 `drone_0`，即屏幕上的 magenta/uav0**。映射确认无误。

---

## 1. FIRST REAL DIVERGENCE（唯一分界点）

以 `drone_0` 的**轨迹发布**为分界：

| | 末次时刻 | 之后是否还有 publish |
|---|---|---|
| drone_0 | 绝对 1789799749.30 | **0 次**（commit / publish / LOCAL_SUCCESSOR_READY 全为 0） |
| drone_1 | 持续到 1789799778 | 同窗口 **96 次** |
| drone_2 | 持续到 1789799774 | 同窗口 75 次 |

odom 侧对应：`uav_id=1` 的速度 **<0.05 m/s 连续段为 time_s 63.1..231.7（168.6 s）**，
位置冻结在 **x=17.80**。而目标在该区间从 x≈21 走完全程到 36.0，
最终 drone_0 距目标 **18.20 m**。

**结论：不是"视觉看不出"，而是 drone_0 从绝对 1789799749.3 起完全没有新轨迹，
末条轨迹（`trajectory_id=257`）只有 2.90 s，跑完就停在原地。**

---

## 2. 根因链（逐层）

### 第一层：末次发布之后，规划每轮都在跑，但候选在**初始化阶段**全被否

末次发布后 drone_0 的日志计数：

| 事件 | 次数 |
|---|---|
| `risk-candidate` | 756 |
| `los-plane-audit` | 608 |
| `los-occlusion-interval` | 602 |
| `raw-los-truth` | 602 |
| **`candidate-preinit-failure`** | **597** |
| `current-los-blocked` | 306 |
| `moving-candidate-reject` | 22 |
| `planner-traj-commit` / `publish` | **0** |

失败原因分布（597 条）：

| 原因 | 次数 |
|---|---|
| **`NO_STATIC_FEASIBLE_SIDE`** | **593** |
| `SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION` | 2 |
| `SIDE_INIT_RETIMED_STATIC_COLLISION` | 1 |
| `STATIC_INFEASIBLE`（触发 A* 修复） | 1 |
| `moving-candidate-reject: reason=static` | 22 |

`NO_STATIC_FEASIBLE_SIDE` 的含义：SIDE 候选在**冲突窗口内找不到静态可行的横向偏移**，
连 `side-astar-repair` 也修不出来（日志 `:1789799756.93` 有
`trigger_reason=STATIC_INFEASIBLE base_offset=0.700 collision_start=0.618939 collision_end=1.856816`）。

即：drone_0 停在 (17.80, −0.31)、静态障碍 `obstacle_id=1` 恰好卡在冲突窗口里，
SIDE 两个方向都做不出静态可行解。

### 第二层：SIDE 全失败 → 回退 NOMINAL，但 NOMINAL 也没能提交

日志里 `[side-both-failed-fallback] drone=0 ... selected=NOMINAL` 反复出现，
说明回退逻辑**被走到了**（这正是之前那轮"L/R 全失败无条件回退"修复的路径）。
但 `planner-traj-commit` 依然是 0 —— 说明 NOMINAL 也没进入提交。

### 第三层：为什么 NOMINAL 提交不了（上一轮已定位的同一机制）

同一 run 的早期日志（绝对 1789799690.35，drone_1 首次冻结时）给出了这个机制的完整形态：

```
[fresh-time-allocation] drone=1 distance=0.499999 nominal_tracking_speed=0.000000
                        v_ref=1.650000 total_time=0.577350
[fresh-initializer]     drone=1 distance=0.499999 duration=0.577350 pieces=2
                        required_speed=0.866025 required_acc=8.660255
                        required_jerk=155.884725 dynamics_valid=0
[moving-candidate-reject] reason=SHORT_STATIONARY_HYPOTHESIS duration=0.021634 required=0.490000
```

机制描述：

1. 无人机已静止、目标继续远离 → 规划的 local target 距离很小（0.5 m 量级）、
   且 `nominal_tracking_speed=0.000000`；
2. `fresh-time-allocation` 按 `distance / v_ref` 分配时长，得到极短时长，
   由此反算的 `required_jerk=155.9`（远超上限）→ `dynamics_valid=0`；
3. 即使某轮生成了轨迹，`finalizeCapturedCandidates` 会用
   `SHORT_STATIONARY_HYPOTHESIS`（duration < 0.490 s）把它判为
   "近零时长静止假设"并拒绝；
4. 于是既没有合法的运动候选、也没有合法的静止候选 → **本轮无提交**；
5. 车辆执行完手上最后一条轨迹后停在原地，下一轮回到第 1 步 → **自锁**。

这是一个**规划器自锁**：静止状态的无人机在"目标已远离"的几何下，
无法产生任何被自身验收规则接受的 successor。

---

## 3. 与"未合围"的因果关系

drone_0 停摆直接造成合围退化：
- `[B] SPIN_IN_PLACE` 6 段，drone_0 占 3 段（14.63 s / 7.83 s / 1.23 s）
- `[C] NO_ENCIRCLEMENT` 58.8% 帧最大相邻方位间隔 >180°，最大 337.7°（连续 20.4 s）
- 末态三机方位跨度 196°，而 drone_0 距目标 18.2 m、另两机 1~2 m

即：**先有单机停摆，后有合围崩溃**，不是两个独立问题。

---

## 4. 与"该绕不绕导致视线丢失"的关系

本次运行 **未发生** 视线丢失：`visible_count=0` 共 **0 帧**，
K2=97.5%、BLACKOUT=0%、可见数分布 3机 68.8% / 2机 28.7% / 1机 2.5%。
所以这一条本次**未观测**，不能与停摆混为一谈。

但注意 `[current-los-blocked] drone=0 blocker_id=16 dynamic=0` 出现 **306 次**、
`los-plane-audit` 608 次 —— 停摆期间 drone_0 **持续处于静态 LOS 被挡状态**，
说明它卡在了一个"既动不了、视线又被挡"的位置。

---

## 5. 待验证的下一步（按优先级）

1. **给 `SHORT_STATIONARY_HYPOTHESIS` 与 `NO_STATIC_FEASIBLE_SIDE` 同时成立时补一条
   可执行的恢复出口**。当前两者叠加就是死锁；
   注意这属于"让规划器能产出合法 successor"，不是新增 brake/stop 架构。
2. **核对 `fresh-time-allocation` 在 `nominal_tracking_speed≈0` 时的退化**：
   它用 `distance / v_ref` 分配时长，当距离因静止而变小时会把时长压到
   `required_jerk` 爆表（155.9），这是自锁的第一推动力。
3. **确认 `planner-traj-commit=0` 与 `capture_output`(deferred/team) 路径的关系**：
   我此前只在直接提交分支埋了 SIDE_BOTH_FAILED 分级计数，
   `capture_output != nullptr` 返回 `PENDING` 的路径未埋点，
   所以"零提交"目前只有 `planner-traj-commit` 一个证据源，需要补第二个独立证据。

---

## 6. 本轮顺带修掉的采集缺陷（避免再误判）

| 缺陷 | 症状 | 修法 |
|---|---|---|
| 视口坐标写死 `x 220..980` | 真实视口右边界在 ~1520，遮挡/冻结检测看错区域，漏报上一轮 46% 遮挡 | 改为目视确证 `(235,35,1520,980)` + 「高填充亮色大矩形」判据；在**已知有遮挡的旧帧上验证报 46.3%**，干净帧 0.0% |
| `m/px = visible_h / frame_h` | 把整窗高当投影高，比例偏 15% | 改为用覆盖球已知半径 0.45 m 自标定（本次 m/px=0.02782，nominal=0.02362） |
| 只统计"未检出"，不区分原因 | uav0 被误判为"识别失败" | 新增 `miss_by_vehicle` 分机计数；结合 odom 判定"出画"（uav0 在目标后方 18.2 m，超出 ±15.2 m 视场半宽） |
| 无录制健康度汇总 | 上一轮 72% 画面冻结未被发现 | `capture_summary.json` 增加 `occluded_frames` / `frozen_frames` / `frozen_ratio`（本次 frozen=735/1194=61.6%，occluded=0） |

三类现象的检测器：`scripts/capture/analyze_alp_phenomena.py`
（判据全用 odom，像素只定位）。
