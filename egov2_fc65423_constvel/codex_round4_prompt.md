# 任务：解锁 swarm 安全后验检查中的自锁硬拒绝

工作区：`/home/bob/ALP/egov2_fc65423_constvel`
规划器源码：`/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/`

**禁止访问 `/home/bob/RRCT`。注释与报告用中文。**

## 一、已确认的缺陷（不要重新论证，按此实施）

`src/planner_manager.cpp` 的 `EGOPlannerManager::checkTrajectorySwarmSafety()`
（约 4897-4945 行）当前逻辑：

```cpp
const double clearance = getSwarmClearance();          // 0.5 m
const double clearance2 = clearance * clearance;
for (每 0.03 s 采样 t < check_end)
  for (每架 peer)
    const Eigen::Vector3d delta = pos - other_pos;
    const double ellip_dist2 = delta.head<2>().squaredNorm() + delta(2)*delta(2)*0.25;
    if (ellip_dist2 < clearance2) { reason = "SWARM"; return false; }   // 无条件硬拒绝
```

实测后果（真实日志 `sim_run_round4.log`）：
- 两架 UAV 已进入彼此安全泡后（最小间距实测 **0.0079 m**，阈值 0.58 m），
  双方**所有**候选轨迹都被拒绝：飞近更差、飞远在 1.5~2.0 s 窗口内仍然 < 0.58 m，
  唯一能过的是"瞬间跳到 0.58 m 外"，违反动力学不可能。
- 全场 `swarm-hard-conflict` 1282 次，冻结窗口内 67 次（57 次为 drone0↔drone1），
  期间 `finalizeCapturedCandidates` 永远无法提交候选，前驱到期 →
  `TERMINAL_HOLD_ENTER` 连续 11 次（t+41.35~51.39 s，drone0），
  直到 peer 轨迹过期才自行解锁。
- 这不是互斥量问题：规划器路径上没有 mutex，回调线程一直在正常跑。

## 二、必须实现的语义：互不恶化（mutual non-worsening）

把"必须立刻回到 clearance 之外"改为"**不得让已有违规恶化**"：

```text
若某采样时刻 候选椭圆距离² < clearance²：
    1) 先算当前 (this active traj vs peer) 在同一时刻的椭圆距离² current_d2
       （用同一椭圆度量：dx²+dy²+0.25*dz²，this=traj_.local_traj.traj，
        peer=other_pos，t 用同一 global_time 采样）
    2) 若 候选椭圆距离² >= current_d2  → 该采样点放行（不恶化）
       否则                              → 仍然硬拒绝
若候选在整个 check_end 窗口内的最小椭圆距离² >= clearance² → 直接放行（原行为）
```

要点（必须严格遵守）：
1. **不修改 `clearance` 数值、不修改 `swarm_clearance`、不修改
   `temporal_swarm_trigger_margin`、不修改椭圆度量 `0.25` 的竖直系数。**
   这只是把"绝对阈值"改成"绝对阈值 OR 互不恶化"。
2. 该放宽**只在双方已经处于违规状态时生效**；一旦间距 >= clearance，
   行为与现在完全一致（仍然硬拒绝任何 < clearance 的采样）。
3. 该放宽对双方是对称的：两边都只在"自己在改善（或至少不恶化）"时才被接受，
   因此两机同时朝远离对方的方向规划时能够逐步分离，死锁自然解除。
4. `checkTrajectorySwarmSafety` 只用于**候选后验检查**；不要动
   `evaluateTrajectorySwarmConflict`（优化器内部的软代价/预测）与
   `traj_.swarm_traj` 的写入逻辑。
5. 只提交仍通过既有其余检查的候选：本次改动不得跳过后验的其它门
   （dynamics / static / dynamic / `validateExecutionTrajectory`）。

## 三、必须新增的遥测（可 grep，便于验收）

1. 每次因"恶化"而被拒绝时（原来会拒绝、现在仍拒绝）：

```
[swarm-mutual-clearance] drone=%d other_drone=%d action=REJECT_WORSENING
  candidate_min_distance=%.6f current_min_distance=%.6f trigger_distance=%.6f
  conflict_time=%.6f peer_traj_id=%d
```

2. 每次因"互不恶化"而被放行时：

```
[swarm-mutual-clearance] drone=%d other_drone=%d action=ACCEPT_NON_WORSENING
  candidate_min_distance=%.6f current_min_distance=%.6f trigger_distance=%.6f
  conflict_time=%.6f peer_traj_id=%d
```

3. 计数器（每 5 秒 + 结束时各打印一次，前缀固定）：

```
[swarm-unlock-audit] drone=%d SWARM_MUTUAL_WORSENING_REJECT_COUNT=%lu SWARM_MUTUAL_NON_WORSENING_ACCEPT_COUNT=%lu SWARM_ABSOLUTE_REJECT_COUNT=%lu
```

- `SWARM_ABSOLUTE_REJECT_COUNT`：因绝对阈值且未处于违规状态而拒绝的次数（原路径）。
- 计数放在 `mutable` 成员里（函数是 const）。

## 四、边界与正确性要求

- 若 `traj_.local_traj` 为空/段数为 0，或 peer 的 `executionAt()` 返回的
  `duration <= 0`，则**退回原来的绝对阈值拒绝逻辑**（不要因为算不出 current
  就一律放行）。
- `current_d2` 必须用与候选**同一时刻、同一椭圆度量**计算，禁止用不同时间点近似。
- 若 `current_d2 < clearance2` 但候选 `min_d2 >= current_d2`，放行；并保证放行后
  仍会走完 `validateExecutionTrajectory` 的其它检查。
- 数值上使用 `>= current_d2 - 1e-9` 的容差，避免浮点抖动造成误拒。

## 五、编译

只允许：

```bash
source /opt/ros/noetic/setup.bash
source /home/bob/ALP/guidance/ros_ws/devel/setup.bash
cd /home/bob/ALP/egov2_fc65423_constvel
catkin build traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

不要跑测试矩阵，不要启动仿真（仿真由我来跑）。

## 六、完成后用中文回复

- 改动的文件与行号范围；
- 编译结果；
- 新旧语义对比（一段话）；
- 确认 `clearance` / `swarm_clearance` / `temporal_swarm_trigger_margin` /
  椭圆竖直系数**均未改动**（贴出 diff 证据）；
- 新增日志前缀与计数器名称清单；
- `CODE_MODIFIED_THIS_TURN: YES`
- `RRCT_ACCESSED: NO`
- `RRCT_CHANGED: NO`
