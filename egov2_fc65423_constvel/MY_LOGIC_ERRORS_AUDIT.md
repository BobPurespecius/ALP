# 我本会话引入的逻辑错误：彻查与修复

工作区：`/home/bob/ALP/egov2_fc65423_constvel`
RRCT_ACCESSED: NO / RRCT_CHANGED: NO

本文件只记录**我自己在本会话中引入**的错误，以及它们的证据与修复。
不重复引用其它历史报告结论。

---

## 错误 1（最严重）：唯一入口把 25 个启动参数丢成 1 个 ⇒ 实际一直以 OFF 启动

### 现象
用户反复指出"你启动的是 off"。我一开始以为参数是对的，因为我写的
`resolved_params.txt` 里明明全是 `true`。

### 真因
`scripts/run_alp_full_on.sh` 里我写了：

```bash
read -r -a launch_args <<< "$(printf '%s\n' "${ALP_LAUNCH_ARGS[@]}")"
```

`read` **只消费第一行**，所以 26 个元素只留下第 1 个。
最小复现（实测）：

```bash
A=( 'a:=1' 'b:=2' 'c:=x y' )
read -r -a out <<< "$(printf '%s\n' "${A[@]}")"
echo "${#out[@]}   # -> 1
```

### 铁证
上一轮运行的 roslaunch 进程真实 argv：

```
roslaunch .../native_egov2_rviz.launch scene_file:=... rviz:=true
```

只有 2 项。所有 `enable_team_visibility_optimizer` / `enable_joint_topology_coordination`
等开关全部退回 launch 默认值（`false`）⇒ **等价 OFF**。
（`scene_file` 与 `rviz` 之所以还在，是因为它们是在 `bash -c` 里直接拼的，
不走那个数组。）

### 修复
```bash
launch_args=("${ALP_LAUNCH_ARGS[@]}")     # bash 数组原样展开
```
并新增 `runs/<RUN_ID>/roslaunch_argv.txt`：把真正传给 roslaunch 的 argv
原样打印落盘。以后任何"看着是 ON 其实是 OFF"一眼可查。

---

## 错误 2（造成卡顿）：我强行打开了生产从未启用的 hard-corridor SCP

### 我做了什么
为了让"SIDE 候选携带的硬平面真正进入求解器"，我把判定门从

```cpp
if (candidate_hard_corridor_scp_enabled_ && side_candidate)
```

改成

```cpp
if (side_candidate && (candidate_hard_corridor_scp_enabled_ ||
                       !candidate_local_sfc_planes_.empty()))
```

即"SIDE 只要带平面就必然跑 SCP"。而该开关在全部生产 launch 中都是 `false`。

### 证据：这条路径的解明显更糙
同一次运行内对比（`max_jerk`，上限 20.0）：

| 路径 | n | p50 | p90 | max |
|---|---|---|---|---|
| 普通 NOMINAL（fresh-init） | 1837 | 12.20 | 19.88 | 21.00 |
| 我强行打开的 SCP 路径 | 1300 | **16.15** | 19.97 | **75.83** |

SCP 产出的 jerk 中位数高 32%，且出现 75.83 这种离群值。
这不是"物理阈值越界"，而是**解的质量显著变差** —— 直接对应可见卡顿。

### 结论
这是用"打开一条从未被验证的生产链路"来掩盖问题。已**完整撤回**：

```cpp
candidate_hard_scp_executed_last_ =
    side_candidate && candidate_hard_corridor_scp_enabled_;   // 语义不变
```

同时 `scripts/lib/alp_params.sh` 里 `hard_corridor_scp` 恢复为
`${NATIVE_EGOV2_HARD_CORRIDOR_SCP:-false}`。

### 真正该修的断点（已修）
链路真正断在这里：`CandidateSideScope` 析构会调用
`clearCandidateSideBias()`，其内部执行
`candidate_local_sfc_planes_.clear()`。而前端在 scope **之外**才读

```cpp
result.local_sfc_planes = ... : ploy_traj_opt_->getCandidateLocalSfc();
```

所以它**永远拿到空集**，"求解器工作过的平面集"作为诊断信息彻底丢失。
修复：在 scope 内、优化完成后立即快照 `solved_local_sfc_planes`，scope 之外使用快照。
这只是如实取出诊断信息，**不改变任何求解路径**。

---

## 错误 3（静默放宽物理硬门）：Team 侧物理硬间距被我设成了只有机体半径

### 我做了什么
为"统一 Local/Team 物理硬间距语义"，我把 Team 侧判定从
`params_.dynamic_clearance`（1.10）改成 `dynamic_margin_body_radius_`（0.384）。

### 真因
Local 的**权威**物理硬阈值是

```cpp
PolyTrajOptimizer::getMovingObjHardClearance()
  = moving_obj_hard_body_radius + max_obj(0.5*max(scale_obj))
  = 0.384 + 0.28 = 0.664 m        // 实测 absolute_threshold=0.664
```

我只保留了机体半径 0.384，**丢掉了障碍物半径** ⇒ 把物理硬门从 0.664 放宽到 0.384。
"统一到另一个数"仍然是错的。

### 修复
两侧现在共用**同一个公式与同一个参数**：

* `moving_obj_hard_body_radius` = 0.384（planner 与 team 同值）
* `moving_obj_hard_obstacle_radius` = 0.28（新增，planner 与 team 同值）
* Team 判定用 `dynamicPhysicalHardClearance() = body + obstacle = 0.664`
* 1.10 m 只保留"期望/排序"语义，不再当物理碰撞硬门

---

## 错误 4（把约束硬塞给不可能满足的候选）：我删掉了 LOS 观察面的 `reached_side` 门

### 我做了什么
为"平面区间必须等于真实遮挡区间，不能被 seed 裁剪"，我把建面条件从
`reached_side && window_overlaps_occlusion` 改成
`occlusion_window_available && window_overlaps_occlusion`，即**完全删除** `reached_side` 门。

### 为什么错
seed 完全在错误侧、候选在一个 SCP 步内根本不可能换侧时，仍强行建面并当硬约束，
会大量产出不可行候选。这与"缩短约束来迁就 seed"是两个不同的错误，我用一个错误
去修了另一个错误。

### 修复（两个语义分开）
* 遮挡区间**已知**时：约束区间 = 真实遮挡区间（保留我要修的正确部分），
  seed 未到正确侧仍建面，交给既有 hard 校验照常拒绝 —— 不额外放宽也不额外收紧；
* 遮挡区间**未知**时：沿用工程既有语义，必须有 `reached_side` 才建面，
  区间取 `[max(到达侧时刻, 冲突窗起点), 冲突窗终点]`。

---

## 未改动但已核实的地方（避免误伤）

| 项 | 核实结论 |
|---|---|
| `trackingGradCostP` cost↔gradient 同源 | 有限差分 `gradp ?= ∂costp/∂p`，误差 5e-9（PASS） |
| 时间项链式律 `gradt ?= ∂costp/∂t + gradp·v` | 误差 6.7e-9（PASS） |
| 前缀真实时间判定 | 用积分点绝对时间 `elapsed_t`，不再用采样点序号；T 优化时前缀归属自动跟随 |
| Joint effective horizon 端到端 | selector `H_eval=min(H_configured,H_common)` ⇒ optimizer 内部同一个数 |
| L/R 全失败计数分级 | SELECTED / SUBMITTED / ACTIVATED 三级，提交在真正写回 `traj_` 时才记 |
| rolling/terminal fallback | 硬编码 `true` 改为 `active_execution_touch_goal_` |
| RViz 生命周期 | `required="false"`，RViz 退出不再拖死仿真核心 |
| cleanup 脚本 | 按 RUN_ID ownership；并修掉"清理脚本杀掉自己调用者"的自匹配缺陷 |

---

## 教训

1. **绝不能"为了让某条链路生效"而强行打开生产里默认关闭的开关。** 那是在改
   被测系统的语义，不是在修链路的断点。真正该修的是断点本身。
2. **"统一两个定义"必须统一到同一个数**，不能统一到一个新数。改之前先把两侧的
   权威取值打印出来（本场景是 0.664，不是 0.384，也不是 1.10）。
3. **启动参数必须自证**：`resolved_params.txt` 只证明"我打算传什么"，
   `roslaunch_argv.txt` 才证明"我真的传了什么"。两者必须都由脚本自动产出。
4. 修 A 错误时不要顺手删掉 B 条件的门 —— 先写清 A 与 B 是不是同一个问题。


---

## 验证运行（修复后）

运行：`runs/20260918_215447_1595495`（唯一入口 `./run_on.sh` = `scripts/run_alp_full_on.sh`）

```
LAST_BOOT_STAGE            = BOOT-12
SIMULATION_REACHED_BOOT_12 = YES
CORE_EXIT_CODE             = 0
CLEANUP_STATUS             = OK   (自有 31 个进程全清，外部进程 0)
```

参数自证 `roslaunch_argv.txt` = 25 项，`enable_team_visibility_optimizer:=true`、
`enable_joint_topology_coordination:=true`、`enable_candidate_hard_corridor_scp:=false`、`rviz:=true`。

### 目标运动阶段（唯一有效口径 t ≤ 76.5 s）

| UAV | 速度 P10 | P50 | P90 | <0.30 | <0.45 | 低速段(>0.3s) | 停顿秒 | 方位误差 P50 | P90 | MAX |
|---|---|---|---|---|---|---|---|---|---|---|
| uav0 | 0.371 | 0.980 | 2.059 | 4.97% | 15.6% | 5 | 5 | 2.0° | 10.1° | 179.6° |
| uav1 | 0.421 | 1.032 | 2.149 | 5.10% | 11.4% | 6 | 4 | 1.6° | 8.1° | 38.0° |
| uav2 | 0.413 | 1.018 | 2.091 | 3.57% | 12.7% | 4 | 3 | 1.0° | 4.7° | 63.4° |

### 高度（目标高度 1.5 m，观测带 ±0.20 m）

| UAV | \|z-1.5\| P50 | P90 | MAX | 超出观测带 | >0.50 m | 最大爬升率 |
|---|---|---|---|---|---|---|
| uav0 | 0.00 | 0.03 | 0.34 | 1.7% | 0.0% | 0.85 m/s |
| uav1 | 0.01 | **0.24** | **0.91** | **23.2%** | 3.4% | 0.81 m/s |
| uav2 | 0.00 | 0.03 | 0.63 | 2.3% | 1.1% | 0.85 m/s |

### 仍未解决的问题（如实记录）

1. **uav1 高度异常**：23.2% 时间超出 ±0.20 m 观测带，最大偏离 0.91 m，
   最大爬升率 0.81 m/s —— 这就是"向上绕行"。uav0/uav2 正常（1.7% / 2.3%）。
2. **uav1 追踪半径偏大**：目标运动阶段末水平半径 2.79 m，期望 1.70 m。
3. **终点之后（不在本次验收口径内）**：uav1 在 t≈175 s 后完全静止约 300 s，
   uav0/uav2 收敛到 0.28 m/s；`SHORT_STATIONARY_HYPOTHESIS` 大量出现。
   这些事件**全部发生在目标停止之后**（首次出现 t≈220 s，末次 t≈480 s）。
4. **能力边界（必须明说）**：`SIDE_BOTH_FAILED_COUNT` 的分级计数显示
   `SELECTED=348 / SUBMITTED=0 / ACTIVATED=0`。需要说清楚的是，
   `SUBMITTED` 只在**直接提交**分支自增；`reboundReplan` 的
   `capture_output` 分支返回 `PENDING`（不提交），实际提交由 team transaction
   完成。因此 `SUBMITTED=0` 不能证明"NOMINAL 从未进入提交链"，但**本轮没有找到
   任何代码证据**能证明它真的进入了提交链。这一项应当标记为**未证实**，
   而不是"已修复"。
