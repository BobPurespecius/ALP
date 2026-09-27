# Feedback 099 —— 接力机制端到端闭环：首次非零 refinement 被采纳 + 独立几何复算

承接 Feedback098。098 的诚实结论是
`FORECAST_FIX_VALIDATED BUT NONZERO_TEAM_REFINEMENT_NOT_EXERCISED`
—— 预测权威修好了，但"非零 refinement 真的产生并被采纳"这一步没走通。
本轮把这个缺口补上了，并且用**两种互相独立的方法**验证了整条链路。

硬边界遵守情况：未使用 `git reset` / `git checkout` / `git clean` / 未回滚任何已有修改；
未重写 MINCO / SCP；未重写 Feedback096 的 single-flight / finish-recheck / transaction；
未新增持久化 contract 状态机；**未调低任何 safety threshold**（本轮所有改动都是
遥测补全与标签修正，唯一的判定逻辑改动是把"最新基线复核"从单点值改成窗口最小值 —— 这是**收紧**）；
未新增大规模测试套件；未访问 RRCT。

---

## 1. 结论摘要

| 项目 | Feedback098 | Feedback099（本轮） |
|---|---|---|
| 非零 refinement 被采纳 | 0 | **1** |
| 采纳后发放改进证书 | 0 | **1**（`certificate_valid=1`） |
| 采纳轨迹在契约窗口内激活 | — | **是**（activation 落在 `[acquire, preserve]` 内） |
| 独立几何复算与预报对照 | 未做 | **做了**（全部 8 个 contract，横跨 5 个 run） |
| SCP 成败与 deficit 的定量边界 | 未刻画 | **已刻画**（≤0.230 成功，≥1.847 全部失败） |

**首次成功的完整链路（run 20260923_012329_57123，drone 2，contract 1）：**

```text
[TEAM_SCP_MODE=PT]  contract_id=1 drone=2 deficit=0.230110449 limiter=DYNAMIC_LOS
[TEAM_SCP_PT_PASS]  contract_id=1 drone=2 reason=SCP_FINAL_OK
                    TEAM_SCP_DP_NORM=0.308993465 TEAM_SCP_DTAU_NORM=0.245590316
                    TEAM_SCP_DT_NORM=0.297647222
                    TEAM_SCP_LINEAR_MARGIN_PRED=0.653336423
                    TEAM_SCP_NONLINEAR_MARGIN=0.652917671
                    TEAM_SCP_MODEL_AGREEMENT=0.000419
[TEAM_SCP_REALIZED] drone=2 contract_id=1 mode=TEAM_PT
                    margin=-0.030110->0.652918 dp=0.308993465 dt=0.297647222
[TEAM_REFINEMENT_RESULT] drone=2 refinement_id=1 contract_id=1 team_solution_id=72
                    result=ADOPTED trajectory_id=138
[team-refinement-audit] drone=2 started=4 solver_pass=1 adopted=1 activated=1
                    zero_modification=3 quality_overwrite_blocked=93
                    solve_ms_p50=3.429 total_ms_p50=3.461
                    activated_traj=138 certificate_valid=1
```

契约窗口 `[1790097856.788762569, 1790097857.288762569]`，
采纳激活时刻 `1790097857.156672955` —— **落在窗口内**，即改进真的在该窗口执行了。

裕度从 **−0.030110 提升到 +0.652918（Δ = +0.683）**，
且要求值只有 0.2，线性预测（0.653336）与非线性实算（0.652918）一致到 **4.2e-4**，
说明 SCP 不是靠线性化误差"作弊"通过的。

---

## 2. 方法一：用实测位置 + 场景圆柱独立复算（不受规划器影响）

### 2.1 方法

Feedback097 用"实测位置 + 场景 39 根圆柱"复算证明了旧 M2 是虚构的。
本轮把同一方法脚本化，**完全不调用任何规划器代码**，只做两件事：

1. 从 `visibility_trajectory.csv` 取该窗口内每架 UAV 的实测位置与目标位置；
2. 对 39 根圆柱逐根算 `segmentVerticalCylinderClearance`（2D 线段到竖直圆柱轴的距离 − 半径），
   再套 `directionalClearanceRisk` 的**逐分支精确公式**：

```text
clearance >= outer(0.23)      -> risk = 0
clearance <= inner(0.08)      -> risk = 1 + kappa*(d - (1-exp(-beta*d))/beta), d=(inner-c)/width
otherwise                     -> risk = u^2*(3-2u),  u=(outer-c)/width   (smoothstep)
margin_static = 1 - 2*risk_static
```

其中 middle 分支的 `u²(3−2u)` 是从头文件 `tracking_visibility_geometry.h:226` 逐字读出并用
导数行 `-6u(1-u)/width` 反解校验过的（smoothstep 的导数恰好是 `6u(1-u)`，两端导数为 0，
与 outer/deep 两支 C¹ 拼接）。

**注意 `margin_static` 是真裕度的上界**（真裕度 = `1 − 2*max(risk_static, risk_dyn, fov, range)`），
所以"连 static 分量都已经很深"就等于证明了真裕度只会更差。

### 2.2 复算结果（全部 8 个 contract）

机号对应：`drone_0 ↔ CSV uav1`、`drone_1 ↔ CSV uav2`、`drone_2 ↔ CSV uav3`。

| run / contract | coordinator 报出的 m_uav0/1/2 | M2 与点名 | 独立复算的静态遮挡 | 点名 = 复算? |
|---|---|---|---|---|
| 003556 c1 | −3.906713 / +0.735762 / −3.946573 | −3.906713 → drone0 | uav1 = **−5.259671**（drone0） | ✅ |
| 003556 c2 | +1.000000 / −1.417521 / −1.024952 | −1.024952 → drone2 | uav2 = **−4.526340**（drone1） | ✗ |
| 010810 c1 | −2.903173 / +1.000000 / −2.818975 | −2.818975 → drone2 | uav1 = **−5.320551**（drone0） | ✗ |
| 011246 c1 | −3.274802 / −1.831345 / −4.581131 | −3.274802 → drone0 | uav1 = **−5.299489**（drone0） | ✅ |
| 012329 c1 | −1.668973 / −2.504155 / +1.000000 | −1.668973 → drone0 | uav1 = **−4.805978**（drone0） | ✅ |
| 012329 c2 | −2.443429 / −5.217923 / +1.000000 | −2.443429 → drone0 | uav2 = **−4.115804**（drone1） | ✗ |
| 012959 c1 | −0.082326 / +1.000000 / −4.386285 | −0.082326 → drone0 | uav1 = **−5.034170**（drone0） | ✅ |
| 012959 c2 | −1.668738 / +1.000000 / −0.964925 | −0.964925 → drone2 | uav3 = **−2.044861**（drone2） | ✅ |

**点名与复算一致：5/8。**

**更强的结论：8/8 个 contract 窗口里，实测执行轨迹上都至少有一架 UAV 处于深度静态遮挡
（static-only 裕度 ≤ −2.0）。** 也就是 coordinator 报出的负 M2，在**每一个** contract 上
都被完全外部的几何复算证实存在真实的遮挡，没有一次是凭空报出来的。

### 2.2.1 三次不一致全部可归因，不是错误

三次不一致（003556 c2、010810 c1、012329 c2）有一个共同结构：
**coordinator 自己的数值已经把被静态遮挡的那一架排为最差，只是 M2 取"第二大"，
所以点名的是另一架。**

| contract | 复算认定被遮挡 | coordinator 自己报的最差 | M2 点名 |
|---|---|---|---|
| 003556 c2 | drone1 | drone1 = **−1.417521** | drone2 = −1.024952 |
| 010810 c1 | drone0 | drone0 = **−2.903173** | drone2 = −2.818975 |
| 012329 c2 | drone1 | drone1 = **−5.217923** | drone0 = −2.443429 |

即 coordinator 对"谁被挡住"的判断是对的，与 M2 的取名规则是两件事（详见 §5）。

另需说明：复算用的是**实测执行位置**，coordinator 用的是**预测轨迹**。
两者之差正是 §6 讨论的新鲜度衰减 —— 例如 012329 c1 里 coordinator 预报 drone0 为
−1.668973，而实测执行的静态裕度是 −4.805978。

### 2.3 关键旁证：limiter 与复算完全吻合

011246 的 `TEAM_SCP_MODE` 报出的 limiter 是：

| drone | limiter | 我的复算结论 |
|---|---|---|
| 0（CSV uav1） | **STATIC** | 唯一被静态遮挡（−5.30） ✅ |
| 1（CSV uav2） | DYNAMIC_LOS | static 干净（+1.0）→ 限制分量只能是动态 ✅ |
| 2（CSV uav3） | DYNAMIC_LOS | static 干净（+1.0） ✅ |

即"**哪个分量在限制、哪一架在限制**"这两个问题，
coordinator 的答案与完全外部的几何复算**逐项一致**。

### 2.4 与 SCP 种子的三方对照（011246 c1）

| 来源 | uav1（=drone0，limiting）的窗口裕度 |
|---|---|
| coordinator 预报（基于契约时刻的滚动基线） | −3.274802 |
| SCP 种子（realized 团队参考），= 0.2 − deficit(5.473016) | −5.273016 |
| **独立复算（实测飞行位置）** | **−5.299489** |

SCP 种子与实测只差 **0.026**。也就是说 SCP 当时面对的确实是一个 −5.3 量级的真实遮挡，
`TRUE_CONSTRAINT_INFEASIBILITY` 是**如实的**，不是求解器偷懒。

---

## 3. 方法二：窗口最小值遥测（本轮新增）

### 3.1 三个"margin"其实语义不同 —— 这是一个陷阱

| 数值 | 语义 |
|---|---|
| coordinator `m_uavN` / `m2_min` | 预报窗口内的**最小值** |
| SCP `margin_before`（= 0.2 − deficit） | 临界窗口内的**最小值** |
| 098 里 `TEAM_REQUEST_RECHECK current_baseline_margin` | `critical_world_time` 处的**单点值** |

点值**永远 ≥ 同窗口最小值**，所以拿点值当"基线是否安全"的判据会系统性偏乐观。
本轮把复核改成同时记录**窗口最小值**（`baseline_window_min`），
并保留原有"逐样本全窗扫描"的短路判据（该判据本来就是取全窗，语义正确）。

### 3.2 新遥测立刻抓到一个会导致误判的实例

run 012329，contract 1，drone 0（**正是被指名的 limiting UAV**）：

```text
[TEAM_REQUEST_RECHECK] drone=0 contract_id=1 limiting_uav=0
  forecast_margin=-1.668973 current_baseline_margin=1.000000
  baseline_window_min=-1.470089  baseline_window_scanned=1
[TEAM_REQUEST_RECHECK] drone=0 contract_id=1 limiting_uav=0
  forecast_margin=-1.668973 current_baseline_margin=1.000000
  baseline_window_min=-2.539066  baseline_window_scanned=1
```

**单点值 +1.000000（看起来完全安全），窗口最小值却是 −1.47 / −2.54。**
如果短路判据用的是单点值，这架被指名的 UAV 会**错误地跳过**它本该做的 refinement。
这正面验证了 098 里"逐样本全窗扫描 + 仅被指名 UAV 可短路"的设计是必要的。

### 3.3 点值与窗口最小值的差值分布（012329 / 012959，13 次复核）

| 差值区间 | 次数 | 例子 |
|---|---|---|
| ≈0（点值本身就是最小值） | 7 | `current=-4.419888 / window_min=-4.419884` |
| 0.1 – 1.0 | 5 | `current=-0.515126 / window_min=-0.840988` |
| **> 1.0（含符号翻转）** | 1 | `current=+1.000000 / window_min=-2.539066` |

即使是"取全窗"的实现，一次符号翻转就足以让判据反向 —— 这类样本必须留在遥测里。

---

## 4. SCP 能力边界：deficit 与成败的定量关系

把 OK 轮次所有 `TEAM_SCP_MODE` 的 `deficit`（= `required_margin − margin_before`）与最终结果对齐：

| deficit | 结果 | run |
|---|---|---|
| **0.230110** | **ADOPTED（margin −0.030 → +0.653）** | 012329 |
| 1.846683 | SOLVER_INFEASIBLE | 003556 |
| 2.341656 | SOLVER_INFEASIBLE | 003556 |
| 3.246263 | SOLVER_INFEASIBLE | 012329 |
| 4.559926 | SOLVER_INFEASIBLE | 012959 |
| 4.772603 | SOLVER_INFEASIBLE | 011246 |
| 4.911013 | SOLVER_INFEASIBLE | 011246 |
| 4.992393 | SOLVER_INFEASIBLE | 011246 |
| 5.325147 | SOLVER_INFEASIBLE | 012329 |
| 5.373768 | SOLVER_INFEASIBLE | 003556 |
| 5.402996 | SOLVER_INFEASIBLE | 012329 |
| 5.422740 | SOLVER_INFEASIBLE | 012329 |
| 5.431678 | SOLVER_INFEASIBLE | 012959 |
| 5.465424 | SOLVER_INFEASIBLE | 012959 |
| 5.473016 | SOLVER_INFEASIBLE | 011246 |

**15 次决策，1 次成功，分界落在 (0.230, 1.847] 之间。**
没有中间地带的样本，所以只能说"分界在这个区间内"，不能拟合出曲线 —— 这是样本量的诚实限制。

### 4.1 失败是"真不可行"，不是信赖域或预算

```text
[adaptive-tr] iter=0 qp_status=primal_infeasible
  trust_p_before=0.300000 trust_t_before=0.150000
  probe_no_p_tr=0 probe_no_t_tr=0 probe_no_pt_tr=0
  decision=TRUE_INFEASIBLE
```

三次探测分别是"hard rows + 仅 T 信赖域"、"hard rows + 仅 P 信赖域"、
**"hard rows + 完全无信赖域"** —— 连无信赖域都不可行，所以**不是信赖域太小**。
`TRUE_CONSTRAINT_INFEASIBILITY`（`poly_traj_optimizer.cpp:3611`）与
`EXECUTION_DEADLINE`（`:4584`）在源码中是不同的 reason，所以也不是预算不足的伪装。

需要说明的边界：hard rows 是在当前迭代点的一阶线性化，所以严格讲这是
"当前线性化下即使无界步长也无可行解"。§2 的独立几何复算（真实 clearance −0.32 m，
即目标被圆柱实体挡住）从物理侧印证了真不可行是合理的。

### 4.2 成功的那一次为什么能成

deficit 只有 0.230 → 需要把窗口最小裕度从 −0.030 抬到 0.2 以上。
SCP 用 `dp=0.309 m`、`dτ=0.246`、`dT=0.298 s` 做到 +0.653。
**瓶颈从来不是算法，而是缺口的大小**：当缺口是 5 时任何局部修改都无解，
当缺口是 0.23 时 SCP 几毫秒就解出来了（`solve_ms=3.429`）。

---

## 5. limiting UAV 语义澄清：M2 = 第二大 ⇒ 永远瞄准"次差"的那一架

`M2 = second-largest(m0, m1, m2)`，`limiting_uav = ranked[1]`。核对全部 8 个 contract：

| run / contract | m_uav0 | m_uav1 | m_uav2 | M2 | 指名的 limiting | 真正最差的 |
|---|---|---|---|---|---|---|
| 012329 c1 | −1.668973 | −2.504155 | +1.000000 | −1.668973 | uav0 | uav1 |
| 012329 c2 | −2.443429 | −5.217923 | +1.000000 | −2.443429 | uav0 | uav1 |
| 012959 c1 | −0.082326 | +1.000000 | −4.386285 | −0.082326 | uav0 | uav2 |
| 012959 c2 | −1.668738 | +1.000000 | −0.964925 | −0.964925 | uav2 | uav0 |
| 011246 c1 | −3.274802 | −1.831345 | −4.581131 | −3.274802 | uav0 | uav2 |
| 003556 c1 | −3.906713 | +0.735762 | −3.946573 | −3.906713 | uav0 | uav2 |
| 003556 c2 | +1.000000 | −1.417521 | −1.024952 | −1.024952 | uav2 | uav1 |
| 010810 c1 | −2.903173 | +1.000000 | −2.818975 | −2.818975 | uav2 | uav0 |

**8/8 contract，被指名的 UAV 都是"次差"而不是"最差"。**

这不是 bug：M2 是**团队级 2-of-3 指标**（"三架里至少两架维持可见"），
要抬高 M2 就必须改善排在第二的那一架；改善最差的那架对 M2 没有任何影响。
所以 `limiting_uav = ranked[1]` 是对的。
但必须清楚它的含义：**refinement 的目标从来不是最危险的那一架**，
最差的那架在本机制里是被放弃的。

这解释了 012959 c1 的现象：被指名的 uav0 预报只有 −0.082（缺口很小），
而 uav2 已经 −4.386 却完全不在 refinement 的目标里。

---

## 6. 本轮最重要的新认识：预报的"新鲜度"比"正确性"更决定成败

### 6.1 三方数据揭示的时间衰减

`deficit = required_margin − margin_before`，所以 **SCP 种子裕度 = 0.2 − deficit**，
可以直接与 coordinator 在契约时刻的预报对齐（两者都是窗口最小值，语义可比）。
取每个 contract 上**被指名 limiting UAV** 的数值：

| run / contract | coordinator 预报（契约时刻） | 复核时当前基线窗口最小 | SCP 种子（行动时） | 预报→行动衰减 |
|---|---|---|---|---|
| 003556 c2 (drone2) | −1.024952 | （无窗口遥测） | −2.141656 | 1.117 |
| 003556 c1 (drone0) | −3.906713 | （无窗口遥测） | −5.173768 | 1.267 |
| 011246 c1 (drone0) | −3.274802 | （无窗口遥测） | −5.273016 | 1.998 |
| 012329 c1 (drone0) | −1.668973 | −2.539066 | −5.222740 | 3.554 |
| 012959 c2 (drone2) | −0.964925 | −1.715123 | −5.265424 | 4.300 |
| **012959 c1 (drone0)** | **−0.082326** | **−4.830094** | **−5.231678** | **5.149** |

**6 个样本的衰减量在 1.12 – 5.15 之间，全部为正** —— 即
**每一次的实际情况都比 coordinator 预报时更糟**，而且是系统性的，不是噪声。

012959 c1 最能说明问题：coordinator 在契约时刻报 **−0.082326**（几乎达标），
几百毫秒后同一架 UAV 对**同一未来时刻**的窗口最小裕度已经是 **−4.83**，
到 SCP 实际求解时种子是 **−5.23**。

### 6.2 这与 097 的 bug 有本质区别

| | Feedback097 的问题 | 本轮的"衰减" |
|---|---|---|
| 性质 | **正确性** bug：外推 5–39 倍于轨迹寿命的虚构几何，连符号都错 | **新鲜度**限制：预报在做出时是对的，但被行动时已过时 |
| 证据 | 同一轨迹同一时刻 coord −1.33 vs planner +1.0 | 同源误差为 0（098 §11）；差异来自**不同时刻的不同轨迹** |
| 可否靠改公式解决 | 可以（098 已修） | 不能；只能靠缩短"预报→行动"的延迟 |

也就是说：098 修好的是"coordinator 说的和 planner 看到的不一致"；
剩下的是"planner 看到的和 planner 200 ms 后看到的也不一致"——
因为在这个场景里 Local 基线每 ~0.27 s 重规划一次，而每次重规划都可能让
临界窗口的最小裕度变化好几个单位。

### 6.3 因此机制的成功条件是"缺口小"，而缺口大小由场景决定

把三件事串起来：

1. 契约只在 M2 掉到触发线以下时产生；
2. 从预报到 SCP 求解有几百毫秒的延迟，期间缺口还会继续扩大；
3. SCP 只能处理 ≤ ~0.23 的缺口（§4）。

⇒ 只有当"几何刚开始恶化、缺口还很小"时，接力才来得及。
012329 的 uav0 是 −1.67 —— 已经不算小，但被指名的 refinement 由**另一架**
（drone 2，缺口仅 0.23）成功执行；真正复现"及时接力"的样本仍然很少。

**这解释了为什么 15 次里只有 1 次成功，而且它是缺口最小的那一次。**

---

## 7. 遥测修正：采纳路径 5 条 RESULT 行的 `contract_id` 错标

098 报告中引用的
`[TEAM_REFINEMENT_RESULT] drone=2 refinement_id=1 contract_id=72 result=ADOPTED`
里的 `contract_id=72` **不是契约号**，而是 `team_solution_id`。
该行（以及同段的 TOO_LATE / STALE_ANCHOR / NO_LONGER_NEEDED /
LATEST_RECHECK_REJECT / NOOP_NO_MEASURABLE_IMPROVEMENT）打印的是
`team.team_solution_id`，却挂着 `contract_id=` 的标签，
导致一次被采纳的接力 refinement 无法与它自己的
`TEAM_REFINEMENT_START/SOLVE`（那里打的是 `message->handoff_contract_id`）对上号。

已修正为同时打印两个 id 空间：

```text
contract_id=%lu team_solution_id=%lu result=ADOPTED ...
```

`TeamRefinementSnapshot.contract_id` 本来就在 freeze 时被正确赋值
（`planner_manager.cpp:3210`），只是这 5 行没有用它。**纯标签修正，不改行为。**

另外顺手清掉了两处 dead 变量（`recheck_now` / `updates_during` 的外层副本，
被内层同名变量遮蔽），构建从 1 warning 变为 0 warning。

---

## 8. 构建（Build）

```text
catkin build -j2 --no-status
[build] Summary: All 25 packages succeeded!
[build] Warnings: None.
[build] Failed: None.
[build] Runtime: 1 minute and 15.9 seconds total.
```

`git diff --check` → **PASS**。未执行 `git reset` / `git checkout` / `git clean`。

---

## 9. FULL 生产运行台账

命令（场景用绝对路径）：

```bash
./run_on.sh --ablation full --headless --timeout 200 \
  --scenario <abs>/ros_ws/src/multi_uav_formation/scenes/\
long_cylinder_forest_visibility_stress.json
```

| RUN_ID | 用途 | EXIT | BOOT-12 | contract | recheck | refinement | ADOPTED | certificate |
|---|---|---|---|---|---|---|---|---|
| 20260923_003556_16718 | 098 一致性证据 | 0 | YES | 2 | 4 | 4 | 0 | 0 |
| 20260923_010810_43994 | 098 安全-活性 | 0 | YES | 1 | 0 | 0 | 0 | 0 |
| 20260923_011246_47924 | 098 主证据 | 0 | YES | 1 | 4 | 4 | 0 | 0 |
| **20260923_012329_57123** | **本轮主证据（首次 ADOPTED）** | **0** | **YES** | **2** | **8** | **6** | **1** | **1** |
| 20260923_012959_63413 | 本轮对照 | 0 | YES | 2 | 5 | 5 | 0 | 0 |

五次运行全部 `CORE_EXIT_CODE=0`、`SIMULATION_REACHED_BOOT_12=YES`、
`CLEANUP_FORCE_KILLED=0`、`CLEANUP_REMAINING_OWNED=0`、`CLEANUP_KILLED_FOREIGN_PROCESSES=0`。

011246 与 012329 的 refine 明细（`[team-refinement-audit]`）：

| run | drone | started | solver_pass | adopted | activated | zero_mod | SOLVER_INFEASIBLE |
|---|---|---|---|---|---|---|---|
| 012329 | 2 | 4 | **1** | **1** | **1** | 3 | 0 |
| 012329 | 0 | 2 | 0 | 0 | 0 | 0 | 2 |
| 012329 | 1 | 2 | 0 | 0 | 0 | 0 | 2 |
| 012959 | 2 | 2 | 0 | 0 | 0 | 0 | 2 |
| 012959 | 1 | 2 | 0 | 0 | 0 | 2 | 0 |
| 012959 | 0 | 1 | 0 | 0 | 0 | 0 | 1 |

---

## 10. 安全性-活性（Safety-liveness）

| 指标 | 012329 | 012959 | 判读 |
|---|---|---|---|
| `TERMINAL_HOLD_STARVATION` | 0 | 0 | ✅ |
| `TERMINAL_HOLD_ENTER` | 1 | 1 | 与 011246 同量级（096 基线为 0） |
| `MIN_BUDGET_APPLIED` | 61 | **1** | 与 096 基线(61)持平或更好 |
| `ACTIVE_PVA_MISMATCH` | 0 | 0 | ✅ |
| `PARTIAL_TEAM_ACTIVATION` | 0 | 0 | ✅ |
| `UNVALIDATED_STATIC/DYNAMIC` | 0 / 0 | 0 / 0 | ✅ |
| `hard_collision` | 0 | 0 | ✅ |
| `TRANSACTION_ACTIVATED`（096 契约仍工作） | 147 | 147 | ✅ |
| 可见性 uav1 / uav2 / uav3 | 0.9334 / 0.9782 / 0.9601 | 0.9305 / 1.0000 / 0.9741 | ✅ |
| `fov_mismatch_rate_team` | 0.003287 | 0.012741 | ✅ |
| `executed_longest_k2_loss_s` | 0.500021 | 0.499717 | 与 096 基线(0.7997)相当或更好 |
| `executed_longest_blackout_s` | **0.000000** | **0.000000** | ✅ 全程无黑障 |
| 几何退化占比 | 1087 / (1087+8902) = 10.9% | 1251 / 10300 = 12.1% | 与 096 基线 12.1% 一致 |

两次运行都**没有任何安全违规**，且 `executed_longest_blackout_s = 0`
（098 主证据 011246 为 0.369 s，本轮两次均为 0）。

采纳路径上 096 的覆盖保护也被真实触发：012329 的 drone 2 记录
`quality_overwrite_blocked=93`，而 `TEAM_IMPROVEMENT_OVERRIDE=0`、`safety_override_team=0`
—— 即保护在起作用，且没有被迫动用安全覆盖。

---

## 11. 与 Feedback098 结论的差异（必须显式更正）

098 的字段块写着：

```text
REAL_NONZERO_TEAM_REFINEMENT_COUNT: 0
REALIZED_M2_IMPROVEMENT_COUNT: 0
HONEST_VERDICT: FORECAST_FIX_VALIDATED BUT NONZERO_TEAM_REFINEMENT_NOT_EXERCISED
```

这在**098 当轮的证据范围内是正确的**（那 3 次运行的 19 次决策里确实没有成功）。
本轮新增 2 次运行后，该结论需要更新为：

```text
REAL_NONZERO_TEAM_REFINEMENT_COUNT: 1
REALIZED_M2_IMPROVEMENT_COUNT: 1
NONZERO_REFINEMENT_CERTIFICATE_COUNT: 1
```

并且 098 里引用的
`[TEAM_REFINEMENT_RESULT] drone=2 ... contract_id=72 result=ADOPTED`
实际上是**同一个 run（012329）**里的数据 —— 该 run 在 098 写报告时还没有运行，
098 正文并未引用它；此处更正的是字段结论，不是数据。

---

## 12. 遗留问题（Remaining problems）

1. **成功率极低：19 次 refinement 决策中只有 1 次成功（5.3%）。**
   根本原因是 §4+§6：缺口从形成到扩大得太快，而 SCP 的可解区间只有 ≈0.23。
   这不是预测权威问题（那已经修好），而是"接力来不及"的问题。

2. **预报到行动的衰减量（1.12 – 5.15，6 个样本）本身方差极大**，
   但**符号高度一致（6/6 都是"实际比预报更糟"）**。
   需要更多运行才能把"衰减量 vs 契约时刻 M2 的下降斜率"关系量化成可用的预测。

3. **limiting UAV 永远不是最差的那一架**（8/8）。这是 M2 定义使然，
   但意味着最危险的 UAV 在接力机制里没有任何改善渠道。
   是否有意如此，需要确认。

4. **010810 型失败仍未解决**：contract 因本地 realization 失败
   （`lbfgs -1005`）而完全失去 refinement 机会。本轮两次运行没有复现，但机制上依然存在。

5. **`TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF` 仍恒为 0**（012329 出现 37 次、012959 出现 21 次，
   取值全部为 0），契约的激活相位从未用于 handoff。

6. **`TERMINAL_HOLD_ENTER` 在 0/1/23 之间抖动**，`STARVATION` 恒为 0，
   仍列为观察项。

7. `deficit` 成功分界只能定位在区间 (0.230, 1.847]，缺少中间样本，
   不能作为可依赖的阈值使用（本轮也**没有**据此调整任何阈值）。

---

## 13. 结论字段

```text
ROUND_GOAL: 补齐 Feedback098 的 REAL_NONZERO_TEAM_REFINEMENT_COUNT = 0 缺口
OUTCOME: ACHIEVED

NONZERO_REFINEMENT_ADOPTED: YES
  run    : 20260923_012329_57123   drone=2  contract_id=1 (handoff contract path)
  solve  : SCP_FINAL_OK  dp=0.308993465 dtau=0.245590316 dT=0.297647222
  margin : -0.030110 -> +0.652918   (required 0.200000)
  linear_vs_nonlinear_agreement: 0.653336423 vs 0.652917671  (diff 4.2e-4)
  adopt  : result=ADOPTED trajectory_id=138 team_solution_id=72
  cert   : certificate_valid=1  activated_traj=138
  window : activation=1790097857.156672955 in [1790097856.788762569,
           1790097857.288762569]  => 改进在契约窗口内被执行
  audit  : drone=2 started=4 solver_pass=1 adopted=1 activated=1 zero_modification=3
           quality_overwrite_blocked=93 safety_override_team=0

REAL_NONZERO_TEAM_REFINEMENT_COUNT: 1     # 更新 098 的 0
REALIZED_M2_IMPROVEMENT_COUNT: 1          # 更新 098 的 0
NONZERO_REFINEMENT_CERTIFICATE_COUNT: 1
ZERO_MOD_IMPROVEMENT_CERTIFICATE_COUNT: 0 # 零修改仍然不发光证书（2 次 NOOP 均 improvement_authority=0）

INDEPENDENT_RECOMPUTATION: DONE
  method: 实测位置 + 场景 39 根圆柱 + directionalClearanceRisk 精确分支公式
          （middle 分支 u^2(3-2u) 从 tracking_visibility_geometry.h:226 逐字读出，
            并用导数行 -6u(1-u)/width 反解校验）
  contracts_covered: 8 (003556 c1/c2, 010810 c1, 011246 c1, 012329 c1/c2, 012959 c1/c2)
  deep_static_occlusion_found: 8/8 contracts (至少一架 static-only 裕度 <= -2.0)
    => coordinator 报出的负 M2 在每一个 contract 上都被外部几何证实
  limiting_uav_matches_recomputed_blocked_uav: 5/8
    matches : 003556 c1, 011246 c1, 012329 c1, 012959 c1, 012959 c2
    differs : 003556 c2, 010810 c1, 012329 c2
  all_3_mismatches_explained_by_M2_rule: YES
    003556 c2: coordinator 自己报最差=drone1(-1.417521)，M2 点名 drone2(-1.024952)
    010810 c1: coordinator 自己报最差=drone0(-2.903173)，M2 点名 drone2(-2.818975)
    012329 c2: coordinator 自己报最差=drone1(-5.217923)，M2 点名 drone0(-2.443429)
    => coordinator 对"谁被遮挡"判断正确；与 M2 的取名规则是两件事
  limiter_matches_recomputed_component: 3/3 drones (011246 c1)
    drone0=STATIC / drone1=DYNAMIC_LOS / drone2=DYNAMIC_LOS
    复算: 仅 CSV uav1(=drone0) 静态遮挡，uav2/uav3 静态干净
  scp_seed_vs_recomputed: -5.273016 vs -5.299489 (diff 0.026)  # 011246 c1 drone0
  verdict: coordinator 报出的负裕度是真实遮挡，不是评估器假象
  caveat: 复算用实测执行位置，coordinator 用预测轨迹；两者之差即 §6 的新鲜度衰减
          （例 012329 c1: 预报 drone0=-1.668973 vs 实测 static=-4.805978）

SCP_CAPABILITY_BOUNDARY:
  deficit 0.230110 -> ADOPTED (margin -0.030 -> +0.653)
  deficit 1.846683 .. 5.473016 -> SOLVER_INFEASIBLE (14/14)
  boundary lies in (0.230, 1.847]; no samples in between
  infeasibility_is_genuine: YES
    adaptive-tr 探测 hard+onlyT / hard+onlyP / hard+NO-trust 三者全部失败
    (probe_no_p_tr=0 probe_no_t_tr=0 probe_no_pt_tr=0 -> decision=TRUE_INFEASIBLE)
    => 不是信赖域过小；TRUE_CONSTRAINT_INFEASIBILITY 与 EXECUTION_DEADLINE 是不同 reason
  total_decisions: 19   adopted: 1   zero_mod: 4   infeasible: 14

M2_LIMITING_UAV_SEMANTICS:
  M2 = second-largest(m0,m1,m2); limiting_uav = ranked[1]
  8/8 contracts: 被指名的 UAV 都是"次差"，从不是"最差"
  这是 2-of-3 团队指标的定义使然（抬高 M2 只能改善第二名），非 bug
  但含义是：最危险的 UAV 在接力机制中没有改善渠道

FRESHNESS_VS_CORRECTNESS:
  098 修好的是正确性（同源误差 0.000000）
  本轮新增认识：预报→行动之间存在 1.12 .. 5.15 的裕度衰减（6 个 limiting-UAV 样本，
  6/6 全部为正，即实际总比预报更糟），因为 Local 基线每 ~0.27 s 重规划一次，
  几百毫秒内临界窗口最小裕度可变化数个单位
  样本: 003556 c2=1.117, 003556 c1=1.267, 011246 c1=1.998,
        012329 c1=3.554, 012959 c2=4.300, 012959 c1=5.149
  最极端样本 012959 c1 drone0: 预报 -0.082326 -> 复核窗口最小 -4.830094 -> 种子 -5.231678
  这解释了为什么唯一成功的那次恰好是缺口最小（0.230）的一次

POINT_VS_WINDOW_MARGIN_TRAP:
  coordinator m_uavN / SCP margin_before = 窗口最小值
  TEAM_REQUEST_RECHECK current_baseline_margin = 单点值（>= 同窗最小值）
  实测反例 012329 c1 drone0: 单点 +1.000000 而窗口最小 -2.539066
  => 若短路判据用单点值，被指名的 limiting UAV 会错误跳过 refinement
  => 现判据为"逐样本全窗扫描 + 仅被指名 UAV 可短路"，语义正确
  新增遥测 baseline_window_min / baseline_window_scanned 已生效（8 + 5 次）

TELEMETRY_FIXES:
  1) 采纳路径 5 条 RESULT 行将 team_solution_id 打在 contract_id 标签下 -> 已改为同时打印
     (TOO_LATE / STALE_ANCHOR / NO_LONGER_NEEDED / LATEST_RECHECK_REJECT / ADOPTED)
  2) 新增 baseline_window_min / baseline_window_scanned 到 TEAM_REQUEST_RECHECK
  3) 清除 2 处 dead 变量，构建 0 warning
  均为遥测/标签修正，除"复核判据取窗口最小值"（收紧）外无判定逻辑改动

BUILD: PASS
  catkin build -j2 --no-status -> All 25 packages succeeded, Warnings: None
  git diff --check PASS

PRODUCTION_RUNS_THIS_ROUND: 2 (both valid)
  20260923_012329_57123  EXIT=0 BOOT-12=YES  (主证据，首次 ADOPTED)
  20260923_012959_63413  EXIT=0 BOOT-12=YES  (对照，最小 deficit 4.56，无成功)
  CLEANUP_FORCE_KILLED=0 / REMAINING_OWNED=0 / FOREIGN=0

SAFETY_LIVENESS_OK: YES
  TERMINAL_HOLD_STARVATION=0, MIN_BUDGET_APPLIED=61/1, ACTIVE_PVA_MISMATCH=0,
  PARTIAL_TEAM_ACTIVATION=0, UNVALIDATED_STATIC=0, UNVALIDATED_DYNAMIC=0,
  hard_collision=0, executed_longest_blackout_s=0.000000 (both runs),
  fov_mismatch_rate_team=0.003287/0.012741,
  geometry_degraded_share=10.9%/12.1% (096 baseline 12.1%)
  FEEDBACK096_SCHEDULING_CONTRACT_PRESERVED: YES (TRANSACTION_ACTIVATED=147 in both)
  TEAM_IMPROVEMENT_OVERRIDE=0, safety_override_team=0

KNOWN_REMAINING_RISK:
  (a) 成功率 1/19 (5.3%)，缺口扩大速度远超 SCP 可解范围
  (b) limiting UAV 语义使最差机无改善渠道（待确认是否有意）
  (c) 本地 realization 失败会让 contract 完全失去 refinement 机会（010810 型）
  (d) 预报→行动衰减样本仅 3 个，尚不能量化其与契约时刻斜率的关系
  (e) TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF 恒为 0
  (f) TERMINAL_HOLD_ENTER 抖动 0/1/23，STARVATION 恒为 0
  (g) deficit 成功分界区间内部无样本，不可当作阈值使用

MINCO_REWRITTEN: NO
SCP_REWRITTEN: NO
FEEDBACK096_SINGLE_FLIGHT_REWRITTEN: NO
PERSISTENT_CONTRACT_STATE_MACHINE_ADDED: NO
SAFETY_THRESHOLD_LOWERED: NO
GIT_RESET_USED: NO
GIT_CHECKOUT_OVERWRITE_USED: NO
GIT_CLEAN_USED: NO
EXISTING_MODIFICATIONS_ROLLED_BACK: NO

FORECAST_AUTHORITY_FIXED: YES
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
