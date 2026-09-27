# 动态障碍遮挡根因（rviz1 运行，2026-09-18）

## 现象

uav1（drone0）在 t=55.83–58.03 s（2.20 s）丢失目标视线：

| 判据 | 值 |
|---|---|
| `uav1_static_los_clear` | 1（静态视线通畅） |
| `uav1_camera_fov_valid` / `fov_margin` | 1 / 42.3°（目标在画面正中） |
| `uav1_dynamic_los_clear` | **0**（动态障碍遮挡） |
| `uav1_planned_visible` | 0（规划器预测到了，但没动作） |

遮挡几何（可完全复现）：动态障碍 id=5，`centerENU=[18.8, 0.2]`，沿 y 轴摆动
`y(t)=0.2+0.65·sin(2πt/8+4.71)`，半径 0.28 m。t≈57.7 s 时该障碍 y 穿过
-0.37，而 uav1 正位于 (18.5, -0.37)，目标在 (16.8, 0)，**障碍正好横穿
uav1→目标视线**。uav2 在后方 (15.9, -0.15)，视线不含该 x，因此未受影响。

结论：这是**真实几何遮挡**，不是度量假象。

## 因果链（全部来自真实日志）

**1. 遮挡被提前 1.8 s 正确预测**

```
[raw-los-truth] drone=0 static_blocked=0 dynamic_blocked=1
                dynamic_time=1.800000 dynamic_id=5 authority_end=2.0
[conflict-descriptor] drone=0 reason_mask=3 body_motion=2 body_id=5 body_time=2.000
                                 los_motion=2 los_id=5 los_time=1.800
```

`authority_end=2.0` 且 `los_time=1.8` 在权限范围内，`reason_mask=3` 表示 BODY+LOS
都进了描述符。所以"没看见"不成立——感知与描述符都到位了。

**2. 观察面（LOS 半空间）构建窗口只有 ±0.45 s，覆盖不到遮挡时刻**

```
[side-space] drone=0 candidate=PLUS obs=5 conflict_window=[0.000,0.450]
[observation-topology-plane] drone=0 candidate=SIDE_PLUS  blocker=5 side=1  status=WINDOW_LIMITED
[observation-topology-plane] drone=0 candidate=SIDE_MINUS blocker=5 side=-1 status=RECOVERY_NOT_REACHED
[los-plane-lifecycle] stage=BEFORE_SCP reason_mask=2 blocker_id=5
                      los_plane_count=0 action=NO_PLANE_FOR_CANDIDATE
```

源码 `planner_manager.cpp:7171-7180`：

```cpp
constexpr double kGuidanceWindowSeconds = 0.45;
double conflict_window_start = max(0.0, risk.conflict_time - 0.45);
double conflict_window_end   = min(nominal_duration, risk.conflict_time + 0.45);
```

LOS 遮挡发生在 `t = 1.8 s`，而窗口是围绕"当下风险时刻"（本例 `conflict_time=0`）
展开的 `[0, 0.45]`。观察面的 `active_start/active_end` 直接由这个窗口截取
（`planner_manager.cpp:7569-7570`），于是**该 LOS 半空间从未覆盖真正的遮挡区间**。

`side_init_candidate_traj` 在窗口内不会越过观察半空间，于是 `reached_side=false`，
`exit_time <= enter_time`，函数在 7606 行报 `WINDOW_LIMITED` /
`RECOVERY_NOT_REACHED` 并**放弃创建该平面**——`los_plane_count=0`。

**3. 没有 LOS 约束的候选被提交，遮挡按原样发生**

```
[visibility-topology-trigger] directional_dynamic=25.1576   ← J_vis 连续代价在起作用
[candidate-visibility] SIDE_PLUS  dynamic_clearance=0.922558
                       uav1_visibility=0.714286 all3=0.333333
[candidate-visibility] SIDE_MINUS dynamic_clearance=0.922558
                       uav1_visibility=0.714286 all3=0.333333
[planner-traj-selected] candidate_type=SIDE_PLUS trajectory_id=200
```

`SIDE_PLUS` 与 `SIDE_MINUS` 的可见性量测**完全相同**（0.714/0.333），说明两个方向
在该窗口内对遮挡毫无区别；选择等于抛硬币。而真正的遮挡要等 1.8 s 后才发生，
根本不在两个候选被比较的量测窗口里。

**4. 为什么"本可避免"却仍然发生**

- 预测端没问题：提前 1.8 s 就锁定 blocker=id5。
- 动作端失效：唯一能把"绕开视线"变成硬约束的通道——observation plane——因为
  ±0.45 s 的窗口被整体放弃。
- 更关键的是，在遮挡前约 1.0–1.6 s 那几次规划里，`conflict_time` 已经被写成 0
  （危险进入"当下"），此时再想横向绕开，纵向机动余量已经不够；正确的动作时机
  是更早的几次规划，而那几次的观察面同样因为窗口问题没有建立。
- uav1 与目标的几何其实允许两种可行解：①保持在障碍 x 位置之前（目标 x 在
  54.63+0.928t，需要 uav1_x > 16.9+0.928t，当时 uav1 在 18.48 且以约 1.0 m/s
  前进，被越拉越近）；②退到障碍之后（uav1_x < 17.7+0.928t）。两者都需要
  **提前 1 s 以上**决断并写进轨迹约束——正是被丢弃的那条通道。

## 结论

这不是"看不见"，也不是"来不及"，而是：

> **LOS 观察面沿用了 BODY 碰撞的时间窗（risk.conflict_time ± 0.45 s），
> 而 LOS 遮挡是一个持续 2.2 s、且通常在 1.8 s 之后才发生的区间事件。
> 窗口不覆盖遮挡区间 → 观察面无法建立 → LOS 约束从未进入任何候选 →
> 遮挡按预测的几何原样发生。**

修复方向（本轮未改，等你确认）：观察面的 `active_start/active_end` 应由
**遮挡区间本身**决定——复用 `raw-los-truth` 扫描里已经在算的
`segmentIntersectsVerticalCylinder` 进出时刻（blocker 进入/离开视线的世界时间），
而不是 `conflict_time ± kGuidanceWindowSeconds`；同时保证窗口上限覆盖
`raw_dynamic_los_time` 之后的整段遮挡。这样候选在生成阶段就带上正确的
LOS 半空间，`SIDE_PLUS/SIDE_MINUS` 才会在可见性量测上出现真实差异，
"提前 1.8 s 已知的遮挡"才能被真正绕开。
