# Feedback 12 — Local SFC `point / q_i` 源码溯源

日期：2026-09-03  
范围：仅只读检查 `/home/bob/ALP/egov2_fc65423_constvel`；未访问 `/home/bob/RRCT`，未修改源码、参数或场景，未运行仿真。

## 结论先行

当前 Local SFC 平面的生产构造公式是：

\[
M=\max(\text{grid map resolution},\ \text{optimization obstacle clearance})
\]

\[
g_i=\text{simplified\_path}[i]
\]

\[
p_i^{probe}=g_i-2M n_i
\]

\[
q_i=\text{LocalSfcPlane.point}=g_i-M n_i
\]

\[
c_i=\text{LocalSfcPlane.clearance}=0
\]

最终约束为：

\[
n_i^T(p-q_i)\ge c_i=0.
\]

所以，`q_i` 不是 guide，也不是 probe；当 `M>0` 时，它恰好位于 guide 与 probe 的中点。当前代码已经把 margin 写入 `q_i`，而 `clearance` 为零，不会再把边界二次平移。

## 1. `LocalSfcPlane.point` 在哪里赋值

### 1.1 字段默认初始化

文件：

`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h:24-35`

结构体：`ego_planner::LocalSfcPlane`

```cpp
struct LocalSfcPlane
{
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d guide{Eigen::Vector3d::Zero()};
  ...
  double clearance{0.0};
};
```

这里的零值只是默认初始化，不是有效 Local SFC 几何的来源。

### 1.2 唯一生产赋值位置

文件：

`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1469-1477,2010,2888-2967`

外层函数：

`bool EGOPlannerManager::reboundReplan(...)`

构造所在局部函数：

`optimize_side(const int side, CandidateResult &result)` lambda

关键代码：`planner_manager.cpp:2894-2897,2905-2927,2949-2957`

```cpp
const double map_resolution = grid_map_->getResolution() > 1.0e-6
                                  ? grid_map_->getResolution() : 0.1;
const double margin = std::max(map_resolution,
                               ploy_traj_opt_->getObsClearance());

Eigen::Vector3d tangent = simplified_path[path_index + 1] -
                          simplified_path[path_index - 1];
tangent.z() = 0.0;
tangent.normalize();

Eigen::Vector3d normal = tangent.cross(Eigen::Vector3d::UnitZ());
normal.z() = 0.0;
normal.normalize();
normal *= static_cast<double>(side);

const Eigen::Vector3d guide = simplified_path[path_index];
const Eigen::Vector3d probe = guide - normal * (2.0 * margin);
if (grid_map_->getInflateOccupancy(probe) == 0)
  continue;

LocalSfcPlane plane;
plane.normal = normal;
plane.point = guide - normal * margin;
plane.guide = guide;
plane.clearance = 0.0;
```

全源码树搜索没有发现第二个有效的 `LocalSfcPlane.point` 构造赋值。之后只是容器复制和传递：

- `planner_manager.cpp:2967`：放入 `repaired_local_sfc_planes`；
- `planner_manager.cpp:3817-3826`：复制为 `active_local_sfc`，retiming 只修改 `active_start/end`；
- `poly_traj_optimizer.cpp:5176-5180`：`setCandidateLocalSfc()` 复制整个 plane vector。

这些位置均不重新计算或改变 `point`。

## 2. `point` 的精确数学来源

记：

- `M = margin`；
- `g_i = simplified_path[i]`；
- `n_i = normal`。

源码精确对应：

\[
M=\max(r_{map},d_{obs})
\]

其中：

- \(r_{map}\) 来自 `grid_map_->getResolution()`；若它不大于 `1e-6`，回退为 `0.1`；
- \(d_{obs}\) 来自 `ploy_traj_opt_->getObsClearance()`，该 getter 在 `poly_traj_optimizer.h:342` 返回 `obs_clearance_`；
- `obs_clearance_` 在 `poly_traj_optimizer.cpp:4955` 从 ROS 参数 `optimization/obstacle_clearance` 读取。

随后：

\[
q_i=g_i-Mn_i.
\]

这不是推测，而是 `planner_manager.cpp:2951` 的直接翻译。

当前常用 launch 默认值可用于理解量级：`target_tracking_platform_single.launch:28-30` 给出 map inflation `0.35`、`ego_obstacle_clearance=0.50`；`advanced_param.xml:213,306` 将 grid resolution 设为 `0.1` 并传入 obstacle clearance。因此在未被上层覆盖的这条启动链中，\(M=\max(0.1,0.5)=0.5\,m\)。公式本身不依赖这个默认值。

## 3. guide、probe、point、normal、clearance 的真实关系

### guide / \(g_i\)

`planner_manager.cpp:2923`：

\[
g_i=\text{simplified\_path}[i].
\]

它是 A* repaired path 经简化/必要点回插后的内部 guide 点；首尾点不生成该平面，因为循环范围是 `i=1 ... size-2`。

### normal / \(n_i\)

设相邻跨点差为：

\[
\tilde t_i=g_{i+1}-g_{i-1}.
\]

源码先令其 z 分量为零，再归一化：

\[
t_i=\frac{(\tilde t_{ix},\tilde t_{iy},0)}{\|(\tilde t_{ix},\tilde t_{iy},0)\|}.
\]

基础法向：

\[
n_i^0=t_i\times e_z=(t_{iy},-t_{ix},0).
\]

它随后再次做 z 清零和归一化，再乘 SIDE sign：

\[
n_i=\operatorname{side}\,\frac{n_i^0}{\|n_i^0\|}.
\]

证据：`planner_manager.cpp:2905-2922`。

`side > 0` 对应 `SIDE_PLUS`，`side < 0` 对应 `SIDE_MINUS`，见 `planner_manager.cpp:2010-2012,4253-4265`。因此 sign 是在 `normal.normalize()` 之后通过 `normal *= side` 乘入的。

### probe

`planner_manager.cpp:2926` 的真实实现确实是：

\[
p_i^{probe}=g_i-2Mn_i.
\]

probe 只用于稀疏构造和方向验证：只有 `getInflateOccupancy(probe) != 0` 时才创建 plane。probe 本身没有存入 `LocalSfcPlane`。

### point / \(q_i\)

`planner_manager.cpp:2951`：

\[
q_i=g_i-Mn_i.
\]

因此：

\[
q_i=\frac{g_i+p_i^{probe}}{2}.
\]

逐项回答：

- `q_i` 是否就是 guide：**否**；除非非正常地 `M=0`，而构造要求 `margin > 0`。
- `q_i` 是否就是 probe：**否**。
- `q_i` 是否位于 guide 与 probe 之间：**是，且精确位于中点**。
- `q_i` 是否经过 margin 偏移：**是**，从 guide 沿 `-normal` 偏移一个 `margin`。

### clearance / \(c_i\)

生产构造在 `planner_manager.cpp:2955` 明确设置：

\[
c_i=0.
\]

通用约束若 `normal` 为单位向量，则 `clearance=c` 会把实际边界从经过 `q_i` 的平面沿 `+normal` 再平移 `c`：

\[
n_i^T(p-q_i)\ge c
\iff
n_i^T\bigl(p-(q_i+c n_i)\bigr)\ge0.
\]

但当前 Local SFC 的 `c_i=0`，所以**实际代码没有二次平移**；边界就经过 `q_i`。

## 4. probe 的 margin 来源与用途

真实公式：

\[
p_i^{probe}=g_i-2\max(r_{map},d_{obs})n_i.
\]

它不是 `guide - normal * clearance`，也不是 plane boundary。其用途是探测 guide 的 `-normal` 一侧、距离 `2M` 的膨胀栅格占用：

```cpp
if (grid_map_->getInflateOccupancy(probe) == 0)
  continue;
```

probe 不占用时，该 guide 不生成 Local SFC plane；probe 占用时，才把 `local_sfc_required` 设为 true 并创建位于 guide 与 probe 中点的边界点 `q_i`。

构造后还有方向 sanity check，`planner_manager.cpp:2958-2960`：

```cpp
plane.normal.dot(guide - plane.point) >= 0
plane.normal.dot(probe - plane.point) < 0
```

代入源码公式后分别为 `M` 和 `-M`：guide 在允许侧，probe 在拒绝侧。

## 5. 后端最终如何使用约束

结构体注释已在 `poly_traj_optimizer.h:20-23` 声明约定：

```cpp
normal.dot(position - point) >= clearance
```

真正 SCP 行构造位于：

`traj_opt/src/poly_traj_optimizer.cpp:1387-1432`

函数：

`PolyTrajOptimizer::runCandidateHardCorridorSCP(...)`

关键代码：

```cpp
const Eigen::Vector3d normal = raw_plane.normal / normal_norm;
const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(t);
const double h = raw_plane.clearance -
                 normal.dot(p - raw_plane.point);
...
const Eigen::RowVectorXd grad_h =
    -mincoSampleGradientWrtX(..., normal, SAMPLE_POSITION, ...);
appendLinearUpperBound(grad_h, -h, rows, upper);
```

后端会再次把 normal 归一化。它以不等式函数

\[
h(p)=c_i-n_i^T(p-q_i)\le0
\]

加入 QP，等价于：

\[
n_i^T(p-q_i)\ge c_i
\]

或标准半空间：

\[
n_i^Tp\ge n_i^Tq_i+c_i.
\]

同一合同还由 `poly_traj_optimizer.cpp:521-552` 的 `maxLocalSfcViolation()` 独立检查：

```cpp
violation = plane.clearance - normal.dot(p - plane.point);
```

当 \(n_i\) 是单位向量时，\(n_i^T(p-q_i)\) 正是点 `p` 到“经过 `q_i`、法向为 `n_i` 的平面”的有符号距离：正值在 `+normal` 一侧，负值在 probe/障碍一侧。

## 6. 二维数值例子

给定：

\[
g=(5,3),\qquad n=(0,-1).
\]

按照源码，令 \(M=\max(r_{map},d_{obs})>0\)：

\[
probe=g-2Mn=(5,3+2M),
\]

\[
q=g-Mn=(5,3+M),
\]

\[
c=0.
\]

对任意位置 \(p=(x,y)\)，最终约束是：

\[
(0,-1)\cdot((x,y)-(5,3+M))\ge0,
\]

即：

\[
3+M-y\ge0
\iff
y\le3+M.
\]

因此允许区域是：

\[
\boxed{y\le3+M}.
\]

若采用上述当前常用 launch 默认的 map resolution `0.1 m`、obstacle clearance `0.5 m`，则 `M=0.5 m`：

- guide = `(5, 3)`；
- point/q = `(5, 3.5)`；
- probe = `(5, 4.0)`；
- 最终允许区域：`y <= 3.5`。

guide 位于允许区内并距边界 `0.5 m`；probe 位于拒绝区，距边界 `0.5 m`。

## 7. 对既有理解的源码纠正

- “probe = guide - normal * (2 * margin)”：**与当前源码一致**。
- “q_i 就是 guide”：**错误**；当前是 `guide - margin * normal`。
- “q_i 就是 probe”：**错误**；当前 q 是 guide 与 probe 的中点。
- “clearance 会在 q_i 基础上再留一层距离”：通用公式可以如此解释，但**当前生产 plane 的 clearance 明确为 0**，所以当前边界没有第二次偏移。
- margin 不是 `clearance` 字段；margin 已经用于生成 `point/q_i`，两者在当前实现中承担不同角色。

## 8. 可直接口述的答案

`q_i` 来自 A* repaired simplified path 的内部 guide 点：代码先用相邻 guide 算出 XY 切向量，再取 `tangent.cross(UnitZ())`、乘 SIDE sign 得到单位法向；随后在 guide 的障碍侧探测 `probe = guide - 2·margin·normal`。只有 probe 落在膨胀占用区时才建平面，而 `q_i` 就取 `guide - margin·normal`，也就是 guide 与 probe 的中点。当前 `clearance=0`，因此最终半空间边界直接经过 `q_i`。
