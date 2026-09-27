# Feedback 15 — SIDE 静态碰撞核心与 A* raw point 回插溯源

日期：2026-09-04  
检查范围：仅 `/home/bob/ALP/egov2_fc65423_constvel`。未访问 `/home/bob/RRCT`，未修改源码，未运行仿真或 benchmark。

## 结论

当前实现最接近题目选项 **C**，但更精确地说：`first_collision` 和 `last_collision` 是连续 SIDE 初始轨迹上 25 个等时间采样点的索引；源码先把两个索引换成轨迹时间，取时间中点，再调用连续多项式轨迹的 `getPos()` 得到 collision midpoint。它不是首尾碰撞位置的坐标平均，也不是某个离散 `side_path[index]`。随后所有 A* `raw_path` 点按其世界坐标到该 midpoint 的平方欧氏距离排序，最多回插两个尚未存在于 simplified path 的点。

## 1. `first_collision` / `last_collision` 的来源

文件：

`egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`

外层函数：

`EGOPlannerManager::reboundReplan(...)`，定义于 `planner_manager.cpp:1470-1477`。

SIDE 局部流程：

`optimize_side(const int side, CandidateResult &result)` lambda，开始于 `planner_manager.cpp:2011`。

### SIDE 轨迹对象

用于静态碰撞带检测的不是一个名为 `side_path` 的离散容器，而是连续多项式轨迹：

```cpp
poly_traj::Trajectory astar_base_traj;
```

当某个 SIDE 初始化轨迹满足动态改善条件时，它来自：

`planner_manager.cpp:2246-2258`

```cpp
DynamicRiskInfo trial_risk = evaluateDynamicRisk(
    side_init_mjo.getTraj(), planning_prediction_epoch, touch_goal);
...
if (trial_dynamic_valid &&
    (!astar_base_valid || trial_offset > astar_base_offset + 1.0e-6))
{
  astar_base_valid = true;
  astar_base_traj = side_init_mjo.getTraj();
  astar_base_risk = trial_risk;
  astar_base_offset = trial_offset;
}
```

因此，下文的 `first_collision` / `last_collision` 是 `astar_base_traj` 的等时间采样索引。

### 索引生成代码

`planner_manager.cpp:2320-2352`

```cpp
const int astar_samples = 25;
const double astar_duration = astar_base_traj.getTotalDuration();
const double side_scope_start = 0.0;
const double side_scope_end = astar_duration;
int first_collision = -1;
int last_collision = -1;

for (int sample = 0; sample < astar_samples; ++sample)
{
  const double alpha = static_cast<double>(sample) /
                       static_cast<double>(astar_samples - 1);
  const double sample_time = side_scope_start +
                             alpha * (side_scope_end - side_scope_start);
  const int occupied = grid_map_->getInflateOccupancy(
      astar_base_traj.getPos(sample_time));
  if (occupied != 0)
  {
    if (first_collision < 0)
      first_collision = sample;
    last_collision = sample;
  }
}
```

数学上，设采样数 `N=25`，SIDE 基础轨迹为 \(x_S(t)\)，总时域为 \([t_s,t_e]=[0,T]\)：

\[
t_k=t_s+\frac{k}{N-1}(t_e-t_s),\qquad k=0,\ldots,24.
\]

占用集合为：

\[
K_{occ}=\{k\mid \operatorname{InflatedOccupancy}(x_S(t_k))\ne0\}.
\]

若集合非空：

\[
k_{first}=\min K_{occ},\qquad
k_{last}=\max K_{occ}.
\]

occupancy 接口明确是：

```cpp
grid_map_->getInflateOccupancy(...)
```

所以检测的是 **inflated static occupancy**，不是原始未膨胀 occupancy。

## 2. Collision midpoint / collision core 的准确公式

首尾 occupied 样本的轨迹时间在 `planner_manager.cpp:2388-2395` 计算：

```cpp
const double static_collision_start = side_scope_start +
    static_cast<double>(first_collision) *
        (side_scope_end - side_scope_start) /
        static_cast<double>(astar_samples - 1);

const double static_collision_end = side_scope_start +
    static_cast<double>(last_collision) *
        (side_scope_end - side_scope_start) /
        static_cast<double>(astar_samples - 1);
```

真正的 midpoint 赋值在 `planner_manager.cpp:2836-2839`：

```cpp
const double collision_mid_time =
    0.5 * (static_collision_start + static_collision_end);
const Eigen::Vector3d collision_midpoint =
    astar_base_traj.getPos(collision_mid_time);
```

精确数学公式：

\[
t_{first}=t_s+\frac{k_{first}}{N-1}(t_e-t_s),
\]

\[
t_{last}=t_s+\frac{k_{last}}{N-1}(t_e-t_s),
\]

\[
t_{core}=\frac{t_{first}+t_{last}}{2},
\]

\[
c_{core}=x_S(t_{core}).
\]

由于采样时间均匀，也可以写成：

\[
t_{core}=t_s+
\frac{(k_{first}+k_{last})/2}{N-1}(t_e-t_s).
\]

结论：

- 不是 A：没有计算 \(0.5(x_S(t_{first})+x_S(t_{last}))\)；对于弯曲的多项式轨迹，这通常不等于当前结果。
- 不是 B：没有取某个整数 `side_path[(first+last)/2]`。
- 最接近 C：使用首尾索引对应的**分数索引中点时间**，但不是对离散点线性插值，而是直接求连续 `astar_base_traj` 在该时间的位置。
- 代码中的 collision core 是整个首尾 occupied 时间包络的时间中心位置。

## 3. 多个不连续碰撞簇的语义

当前代码没有 collision cluster/interval 列表。

扫描时：

- `first_collision` 只在第一次 occupied 时赋值；
- `last_collision` 在每次 occupied 时更新；
- 因而它们包住 25 个样本中**所有** occupied 点。

若占用序列为：

```text
free occupied occupied free free occupied free
```

则：

- `first_collision` 指向第一簇的第一个 occupied；
- `last_collision` 指向第二簇的最后一个 occupied；
- 中间 free gap 仍包含在 `[first_collision,last_collision]` 包络中；
- midpoint 可能落在 free gap；
- 不会只处理第一簇，也没有分别计算多个 interval 或多个 collision midpoint。

A* repair 的 anchor 范围也由这个全局 first/last 包络向前后扩展采样点得到，见 `planner_manager.cpp:2366-2387`。

## 4. Collision midpoint 如何用于回插 raw A* point

代码位于 `planner_manager.cpp:2828-2879`。

### 距离与排序

```cpp
std::vector<size_t> nearest_indices(raw_path.size());
std::iota(nearest_indices.begin(), nearest_indices.end(), 0);
std::sort(nearest_indices.begin(), nearest_indices.end(),
          [&](const size_t lhs, const size_t rhs) {
            return (raw_path[lhs] - collision_midpoint).squaredNorm() <
                   (raw_path[rhs] - collision_midpoint).squaredNorm();
          });
```

实际比较的是平方欧氏距离：

\[
d_j^2=\|r_j-c_{core}\|_2^2.
\]

它与按 \(d_j=\|r_j-c_{core}\|_2\) 排序完全等价，但避免开平方。所有 `raw_path` 索引先按距离从小到大排列。

### 跳过已存在点与最多两个

```cpp
for (const size_t raw_index : nearest_indices)
{
  if (reinsertion_points >= 2)
    break;
  const Eigen::Vector3d &point = raw_path[raw_index];
  if (std::find_if(simplified_path.begin(), simplified_path.end(),
                   [&](const Eigen::Vector3d &existing) {
                     return (existing - point).norm() <
                            0.5 * map_resolution;
                   }) != simplified_path.end())
    continue;
  simplified_path.push_back(point);
  ++reinsertion_points;
}
```

规则是：

1. 按距 collision midpoint 最近到最远遍历 raw point；
2. 若某点与任意已存在 simplified point 的距离小于 `0.5 * map_resolution`，视为已经存在并跳过；
3. 每成功加入一个点，`reinsertion_points++`；
4. 达到 2 后立即停止。

“最多两个”是源码中的硬编码上限 `reinsertion_points >= 2`。代码注释说明其目的是保留碰撞带附近最可能面向障碍的转折点，同时避免第二次搜索和无界点增长；没有其它动态数量计算。

### 恢复 raw path 顺序

回插完成后，`planner_manager.cpp:2861-2879` 再次排序 `simplified_path`。对每个 simplified point \(s\)，遍历整个 raw path，寻找世界坐标平方距离最小的 raw index：

\[
I(s)=\arg\min_j\|r_j-s\|_2^2.
\]

随后按 `I(s)` 升序排列：

```cpp
return raw_index(lhs) < raw_index(rhs);
```

因此恢复的是沿 A* raw path 的原始前进顺序。它不是按 collision midpoint 距离保留最终顺序。

## 5. SIDE sample index 与 A* raw path index 的关系

二者没有直接对应关系。

- `first_collision/last_collision`：固定 25 个等时间样本在连续 `astar_base_traj` 上的索引。
- `raw_path[j]`：A* 在静态栅格中从 `anchor_start` 到某个 rejoin `anchor_goal` 搜出的离散世界坐标序列，点数和索引由地图分辨率、搜索扩展及 rejoin 结果决定。

一个索引是“SIDE 多项式时间采样编号”，另一个是“A* 搜索路径节点编号”，没有一一映射，也通常没有相同点数。源码因此先把 SIDE collision interval 转换为世界坐标 `collision_midpoint`，再通过三维欧氏距离把它与 `raw_path` 的世界坐标连接起来。

## 6. 小型数值例子

假设：

```text
astar_samples = 25
side_scope_start = 0 s
side_scope_end = 24 s
first_collision = 3
last_collision = 6
```

按源码：

\[
t_{first}=3\cdot24/24=3\ s,
\]

\[
t_{last}=6\cdot24/24=6\ s,
\]

\[
t_{core}=0.5(3+6)=4.5\ s.
\]

假设连续 SIDE 多项式在该时刻给出：

\[
c_{core}=x_S(4.5)=(5,3).
\]

注意，这个 `(5,3)` 必须来自 `astar_base_traj.getPos(4.5)`，不是自行平均 first/last 两个位置。

令 A* raw path 的四个候选点为：

\[
r_0=(4,3),\quad
r_1=(5.2,3.1),\quad
r_2=(5.8,3.2),\quad
r_3=(8,3).
\]

距离为：

\[
d_0=\sqrt{(4-5)^2+(3-3)^2}=1.0,
\]

\[
d_1=\sqrt{0.2^2+0.1^2}\approx0.224,
\]

\[
d_2=\sqrt{0.8^2+0.2^2}\approx0.825,
\]

\[
d_3=\sqrt{3^2}=3.0.
\]

排序为：

```text
r1, r2, r0, r3
```

若四点都尚未存在于 `simplified_path`，优先回插 `r1` 和 `r2`。若 `r1` 已在 simplified path 的 `0.5 * map_resolution` 邻域内，则跳过 `r1`，改为回插 `r2` 和 `r0`。最后所有 simplified points 会按其最近 raw index 重新排序，恢复 A* 路径顺序。

## 7. 3～5 句话总结

`first_collision` 和 `last_collision` 来自对连续 SIDE 基础轨迹做 25 个等时间采样，并用 `getInflateOccupancy()` 找到全部 occupied 样本的最小、最大索引。源码把这两个索引换算为时间，取时间中点，再用 `astar_base_traj.getPos()` 得到 collision core；它不是首尾碰撞坐标的平均。A* raw path 与 SIDE 采样索引没有直接对应关系，因此代码用世界坐标欧氏距离把两者桥接。raw points 按到 collision core 的平方距离排序，跳过 simplified path 已有点，最多回插两个，再按最近 raw index 恢复路径顺序。
