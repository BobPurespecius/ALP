# Feedback 13 — Local SFC 首次 inflate 边界扫描

日期：2026-09-03  
范围：仅 ALP `/home/bob/ALP/egov2_fc65423_constvel`。未访问或修改 `/home/bob/RRCT`；未修改 SIDE、A*、MINCO、SCP、OSQP、动态风险、安全分类或 viewpoint manager。

## 1. 修改文件

1. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`
   - 将 Local SFC 的单点 endpoint probe 替换为 `[0, 2M]` 首次 free→occupied 扫描。
   - `normal`、guide、active interval 和后端接口保持不变。
2. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/local_sfc_boundary_scan.h`
   - 新增 68 行纯标量、无 ROS/GridMap 依赖的轻量扫描 helper。
3. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/test/local_sfc_boundary_scan_contract_test.cpp`
   - 新增 72 行合同测试，覆盖要求的四种占据序列和非整数步长 endpoint 覆盖。

未修改 CMake：合同测试使用独立 `g++` 命令编译，避免扩大现有 dirty CMake 改动。

## 2. 修改前公式

原生产逻辑：

\[
M=\max(r_{map},d_{obs})
\]

\[
probe=g_i-2Mn_i
\]

只有 `occupancy(probe) != 0` 时生成 plane：

\[
q_i=g_i-Mn_i,\qquad c_i=0.
\]

因此若射线中间出现 occupied、但 `2M` 端点重新 free，旧逻辑会漏掉该障碍段。

## 3. 修改后公式

扫描射线：

\[
r(\lambda)=g_i-\lambda n_i,\qquad \lambda\in[0,2M].
\]

从 `lambda=0` 的 guide 开始，寻找第一组：

\[
occ(r(\lambda_{free}))=free,
\]

\[
occ(r(\lambda_{occ}))=occupied.
\]

边界估计：

\[
\lambda_b=\frac{\lambda_{free}+\lambda_{occ}}{2}.
\]

Plane point：

\[
q_i=g_i-\lambda_b n_i.
\]

因为查询的是 `getInflateOccupancy()` 的膨胀占据边界：

\[
c_i=0.
\]

未叠加第二次 obstacle clearance。

源码位置：`planner_manager.cpp:2924-3022`，核心赋值位于 `2984,2988`：

```cpp
plane.point = guide - normal * lambda_boundary;
plane.clearance = 0.0;
```

## 4. scan step 与 endpoint

```cpp
const double map_resolution = grid_map_->getResolution() > 1.0e-6
                                  ? grid_map_->getResolution() : 0.1;
const double scan_step = map_resolution;
const double max_scan_distance = 2.0 * margin;
```

因此：

- 正常时 `scan_step = GridMap resolution`；
- resolution 非法或过小时沿用原有 `0.1 m` fallback；
- 原有 `margin > 0` 构造检查保留；
- helper 每步使用 `min(max_distance, previous + scan_step)`，最后一次显式落在 `2M`，不会因步长不能整除而遗漏 endpoint。

## 5. 首次 transition 的确定

helper `scanFirstLocalSfcBoundary()`：

1. 首先查询 `occupancy(guide)`；
2. 沿 `-normal` 逐步查询；
3. 一旦上一采样 free、当前采样 occupied，立即返回；
4. 不继续寻找第二或第三个 occupied 区间；
5. 即使 `2M` endpoint 最后为 free，中间的第一次 transition 仍已被捕获。

旧逻辑：

```cpp
if (grid_map_->getInflateOccupancy(probe) == 0)
  continue;
```

已从正常构造路径移除。源码搜索确认不再存在旧的固定 `probe`/固定 `q=guide-Mn` 组合。

## 6. 边界条件

### Guide 已 occupied

A* guide 理论上应 free。异常出现时不新增 hard failure：

- 记录一次 `GUIDE_OCCUPIED_LEGACY_FALLBACK` warning；
- 保留旧 endpoint probe 行为；
- endpoint free：不建 plane；
- endpoint occupied：使用旧 `lambda_boundary=M`；
- 不阻塞主链，不引入 FAILED/E-stop。

### `[0,2M]` 全部 free

不生成 plane，与旧 endpoint free 语义一致。

### 中间 occupied、endpoint 又 free

第一次 free→occupied 时立即返回并生成 plane，不再遗漏。

### 第一个 scan sample occupied

使用：

\[
\lambda_{free}=0,
\quad \lambda_{occ}=scan\_step,
\quad \lambda_b=scan\_step/2.
\]

### 多段 occupied

只使用第一道 inflate obstacle boundary。

## 7. Sanity check 与后端合同

现有方向 sanity check 保留并改为使用真实 bracket sample：

```cpp
normal.dot(guide - point) >= 0
normal.dot(occupied_point - point) < 0
normal.dot(free_point - point) > 0
```

最后一项只用于正常 transition；guide-occupied compatibility fallback 不套用不存在的 free bracket。

后端 `traj_opt/src/poly_traj_optimizer.cpp:1417-1432` 未修改：

```cpp
h = clearance - normal.dot(position - point);
```

QP 仍实施：

\[
n_i^T(p-q_i)\ge c_i=0.
\]

SCP/OSQP 接口和数据结构均未改变。

## 8. 日志

只在实际生成 plane 时记录一条：

```text
[local-sfc-boundary-scan]
guide_index=...
margin=...
scan_step=...
lambda_free=...
lambda_occ=...
lambda_boundary=...
q=[...]
normal=[...]
result=FIRST_FREE_TO_OCCUPIED
```

Guide occupied 异常只额外记录一条简短 warning。没有逐 sample 日志。

## 9. Contract/self-test

编译方式：

```bash
g++ -std=c++14 -Wall -Wextra -pedantic \
  -I plan_manage/include \
  plan_manage/test/local_sfc_boundary_scan_contract_test.cpp \
  -o /tmp/local_sfc_boundary_scan_contract_test
```

结果：

```text
LOCAL_SFC_BOUNDARY_SCAN_CONTRACT=PASS
case_a_no_plane=1
case_b_first_transition=1
case_c_terminal_free_still_found=1
case_d_first_sample_occupied=1
endpoint_explicitly_covered=1
```

覆盖情况：

- Case A `free free free free`：无 plane；
- Case B `free free occupied occupied`：边界为最后 free/第一 occupied 中点；
- Case C `free occupied occupied free`：endpoint free 仍捕获第一次 transition；
- Case D `free occupied`：使用 `[0, scan_step]` 中点；
- 附加测试：`max_distance=0.25, step=0.1`，确认显式查询 `0.25` endpoint。

## 10. Build

命令：

```bash
catkin build ego_planner --no-status -j2
```

结果：

```text
All 5 packages succeeded
Warnings: None
Failed: No packages failed
```

增量依赖为 `plan_env / traj_utils / path_searching / traj_opt / ego_planner`；未删除 build/devel，未进行全 workspace rebuild。

BUILD: PASS

## 11. Smoke

使用 `long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json`、RViz OFF、`max_jer=22` 启动约 25 秒：

- 三架 planner、traj_server、controller、simulator、scene 节点成功启动；
- planner 正常持续生成并发布轨迹；
- 未发现 `LOCAL_SFC_INVALID_PLANE`、`SFC_BUILD_FAILED` 或启动级崩溃；
- 25 秒后由 `timeout` 发送 SIGINT，`SMOKE_EXIT=124` 是预期 timeout 状态；
- 退出后无 ROS/ALP 残留进程。

该短窗口没有实际产生 `[local-sfc-boundary-scan]` plane 日志，因此 smoke 只证明启动链和普通规划未被立即破坏，不声称已经完成运行时 Local SFC 性能验证。未运行完整 benchmark，符合本轮“小范围轻度自测”要求。

首次 smoke 尝试因只 source 顶层 devel、launch 找不到 `swarm_bridge` 而在解析阶段退出；修正为 tracking workspace + 顶层 overlay 后重试成功。这是运行环境问题，不是源码编译或 Local SFC 逻辑失败。

## 12. 新问题与限制

- 未发现本次扫描 helper、plane 构造或后端接口的新代码问题。
- 强制 SIGINT 结束时出现已有 ROS shutdown 噪声（Python closed-topic/Boost mutex teardown）；没有残留进程，且发生在 timeout shutdown 阶段。本轮未扩大到无关 shutdown 模块。
- 尚未通过完整运行统计新 plane 的数量或实际规划收益；本轮没有进行大规模实验。

## 13. Git diff 摘要

本轮任务范围内：

- 修改 1 个既有文件：`planner_manager.cpp` 的 Local SFC 构造局部 hunk和一个 include；
- 新增 1 个 68 行 helper header；
- 新增 1 个 72 行合同测试；
- 未修改 `traj_opt`、A*、SCP、OSQP、launch、参数或场景。

仓库在本轮开始前已高度 dirty；完整 `git diff --stat` 包含大量用户既有修改，不能作为本轮独立行数统计。本轮没有 reset、checkout、clean、commit、格式化或覆盖既有修改。

## Final

LOCAL_SFC_SINGLE_ENDPOINT_PROBE_REPLACED: YES

FIRST_FREE_TO_OCCUPIED_SCAN_IMPLEMENTED: YES

SCAN_STEP_SOURCE: grid_map_resolution_with_0.1m_fallback

SCAN_ENDPOINT_2M_EXPLICITLY_COVERED: YES

INTERMEDIATE_OCCUPIED_TERMINAL_FREE_HANDLED: YES

PLANE_POINT_FORMULA: q=guide-lambda_boundary*normal

LOCAL_SFC_CLEARANCE: 0.0

BACKEND_HALFSPACE_UNCHANGED: YES

SIDE_NORMAL_DEFINITION_CHANGED: NO

SCP_OSQP_INTERFACE_CHANGED: NO

CONTRACT_TEST: PASS

BUILD: PASS

SMOKE: PASS_STARTUP_ONLY

FULL_BENCHMARK_RUN: NO

NEW_ALGORITHM_OR_HARD_GATE_ADDED: NO

PREVIOUS_MAX_FEEDBACK_INDEX: 12

CURRENT_FEEDBACK_FILE: feedback_13.md
