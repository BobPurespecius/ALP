# feedback_129 — ALP K3 loss → LOS threat → SIDE 触发链专项只读审计

- 工作区：`/home/bob/ALP/egov2_fc65423_constvel`（dirty worktree 为源码唯一权威）
- 分析 run：`runs/20260926_225042_830329`（BOOT-12，exit 0；K2 loss 0.400 s / K3-ALL3 loss 8.233 s——两者全程区分统计）
- 方法：源码逐行审计（visibility 原语、limiter 合成、raw LOS 扫描、ConflictDescriptor、dispatch 授权）+ 该 run 全部日志/CSV/scene 逐 episode 复盘。未编译、未仿真、未改任何 production 文件。
- Feedback128 的结论（selector 忠实于窗口指标、K2 gate 非主因）直接沿用，不再重证。

---

## 0. 结论速览

1. **K3 loss 全部是真实 LOS 遮挡**：7/7 物理 loss episode 由 STATIC/DYNAMIC LOS 造成（FOV/RANGE 为 0）。
2. **production 检测链全程健康**：7/7 episode 真实遮挡被 raw LOS 检出、blocker id 与几何真值逐一相符、ConflictDescriptor 获得 LOS 位、SIDE dispatch 获得授权。
3. **任务假设（§14：limiter/FOV 盖掉 LOS authority、两套 LOS 几何不一致）被否定**——limiter 只是单一最大风险分量的诊断标签，SIDE authority 完全不走 limiter；两套静态 LOS 几何同源同语义。
4. **K3 loss 的残留主导机制是 H（执行翻转）**：SIDE 候选在每个 episode 内被反复 commit，但执行在 SIDE↔NOMINAL↔FALLBACK 间翻转，6/7 episode 的损失段大部分时间由 NOMINAL 执行飞越阴影。这与 Feedback128 的截断窗口发现同根：窗内指标证明不了 SIDE 更早恢复 → 平局/更差 → ORIGINAL_ORDER 翻回 NOMINAL。
5. 次要发现：EP01 执行 SIDE_MINUS 从 PLUS 侧 (+0.76 m) 穿轴到对侧，加深遮挡后恢复（方向次优嫌疑，INFERENCE）；d0E4 被 M2 抢占 1 次（sibling 事件 E5 接管，非损失点）。

---

## 1. 源码审计一：binary visibility 的四分量定义（§4、§12）

`EGOPlannerManager::evaluateLocalVisibilitySampleAtWorldTime()`（plan_manage/src/planner_manager.cpp L7538-7674）：

| 分量 | 几何 | binary 判据 | 连续 risk |
|---|---|---|---|
| STATIC_LOS | `queryStaticLosClearance(observer,target)` → `StaticLosGeometry`（**直读 scene 顶层 obstacleData 圆柱 + wallData**） | clearance ≥ `visibility_occlusion_margin`(0.08) | `directionalClearanceRisk(cl, 0.08, 0.08+smoothing)` |
| DYNAMIC_LOS | 逐目标 `evaluateConstVel(id, world_time)` 圆柱，radius=0.5·max(scale.x,scale.y)，`segmentVerticalCylinderClearance` | clearance < 0.08 → 不可见 | 同上，逐目标取最大 |
| FOV | `targetInTrackingCameraFov`（yaw 推进模型，水平 85°/垂直 54.5°，range 0.2–8 m） | fov.valid | `trackingCameraFovDirectionalRisk` |
| RANGE | 3D 距离对相机带 [0.2, 8.0] | range2d ∈ [vis_min, vis_max]（跟踪带） | near/far risk 取大 |

`binary_visible = valid && range && static_los && dynamic_los && fov`（L7671-7672）。

**§12 的明确回答（SOURCE PROVEN）：`LIMITER_STATIC` 就是 static-LOS 遮挡风险**（components[0] 由 static LOS clearance 驱动），不是 body 碰撞风险，也不是 visibility static clearance 的其它含义。body 碰撞（`getInflateOccupancy`）是另一条独立证据（CONFLICT_BODY_SAFETY），与 limiter 无关。

## 2. 源码审计二：limiter 的真实语义（§7）

`composeVisibilityMargin`（multi_uav_formation/include/multi_uav_formation/tracking_visibility_geometry.h L85-114）：

```text
margin   = 1 − 2·max(r_static, r_dyn, r_fov, r_range)
limiter  = 取最大风险的那“一个”分量的标签（平局按 STATIC→DYN→FOV→RANGE 稳定排序）
```

**limiter 只是“当前最差 margin 的标签”，不是“存在哪些物理 threat”的并集** —— §7 的结构怀疑在源码层成立。但是，**该标签不参与任何 authority 判定**：

- K3 事件检测只看 `margin < k3_margin_trigger(0.20)`，limiter 仅作为 `detect_limiter` 遥测记录（updateLocalK3RecoveryEvent L3671-3676、L3794）。
- SIDE 授权（`escalation_grant`，L9241-9294）要求：`los_descriptor_authority && k3_local_event_authority && (raw_static|dynamic los blocked) && witness_world_time ∈ [K3窗begin−0.5, K3窗end+0.5]`。
- `CONFLICT_LOS_OCCLUSION` 位只来自 raw LOS witness（nominal 轨迹扫描，首触 ≤ `visibility_authority_horizon`，L9171-9185、L9385-9433 静态 / L9561-9580 动态），与 limiter 无关。

因此**“STATIC blocked + FOV 更差 → limiter=FOV → 无 LOS authority → 不 dispatch”这条链在当前代码中不存在**：limiter 丢的是标签，不是 authority。（RUN 佐证：EP06 uav3 同时 STATIC+DYN 遮挡、事件 limiter=DYNAMIC_LOS，SIDE 照样 dispatch——d2E6 有 grant。）

## 3. 源码审计三：raw LOS / ConflictDescriptor 几何（§6、§9）

- raw LOS 未来扫描（L8900-9001）：observer=nominal 轨迹位置，target=epoch 位置+速度×相对时间（world-time 传播），static 用同一 `queryStaticLosClearance`，dynamic 用同一 `obj_predictor_->evaluateConstVel` 圆柱（radius **+0.08 膨胀**）。静态只记首次命中，动态逐 blocker 记 enter/exit 区间。
- current 判定（L9015-9100）：observer=**当前 odom（start_pt）**，独立于未来扫描；命中则 `current_raw_los_blocked_=true`、记 blocker id（static=witness.primitive_index，dynamic=obj id）、钉 recovery 窗 `[0, horizon]`，打 `[current-los-blocked]`。
- `visibility_authority_horizon = nominal 时长 ×(2/3 rolling) ∩ 预测地平线`（L8879-8887）。
- 软 J_vis 明确**无** topology 权限（L9319-9346 `soft-visibility-continuous-only ... NO_TOPOLOGY_AUTHORITY`）。

**两套 LOS 定义一致性（§6 结论）**：
- 静态：planner evaluator（clearance ≥ margin）⟺ tracker（segment vs radius+margin 圆柱）⟺ raw scan（clearance ≤ margin）——三者同一 scene 源、同一 0.08 margin，语义等价（阈值上的 margin ⟺ 半径膨胀 margin）。**未发现不一致**。
- 动态：planner evaluator 与 raw scan 同用 `obj_predictor_->evaluateConstVel`；tracker 用场景正弦真值位置。二者在 ≤2 s 视野内差异 = constVel 对正弦的局部线性误差（RUN 验证见 §5 blocker 匹配：7/7 一致）。
- 唯一不对等：**binary dynamic evaluator 不膨胀半径、raw scan 膨胀 +0.08**——这使 raw LOS 比二值真值更早 1 个采样格报警，方向是保守的，不是漏检。

## 4. 物理 loss episode 真值（§3、§4）

episode 定义：visibility.csv 中任一 UAV binary=0 的连续区间（合并 <0.2 s 间隙）。**7 个 episode，合计 8.233 s**（与汇总 ALL3 loss 精确相等，校验通过）。执行真值 = 逐样本四 flag（tracker 端与 planner 端静态几何同源，见 §3）。

| EP | 区间 (s) | 时长 | loss UAV | RANGE | STATIC_LOS | DYN_LOS | FOV | true cause | 真实 blocker（几何） |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 7.91–8.81 | 0.90 | uav1(drone0) | ok | **blocked**(min −0.298) | ok | ok | STATIC_LOS | 静态 36 |
| 2 | 10.68–11.95 | 1.27 | uav1 | ok | **blocked**(−0.542) | ok | ok | STATIC_LOS | 静态 26 |
| 3 | 25.22–26.51 | 1.30 | uav1 | ok | **blocked**(−0.537) | ok | ok | STATIC_LOS | 静态 28 |
| 4 | 35.51–36.48 | 0.97 | uav3(drone2) | ok | **blocked**(−0.412) | ok | ok | STATIC_LOS | 静态 6 |
| 5 | 63.95–65.05 | 1.10 | uav1 | ok | **blocked**(−0.426) | ok | ok | STATIC_LOS | 静态 13 |
| 6 | 70.68–72.48 | 1.80 | uav3(drone2) | ok | **blocked**(−0.424, 自 71.38) | **blocked**(动态 10/9 扫过) | ok | MULTIPLE(STATIC,DYN) | 静态 14 + 动态（10 系）|
| 7 | 73.48–74.38 | 0.90 | uav2(drone1) | ok | ok | **blocked**(动态 10) | ok | DYNAMIC_LOS | 动态 10（targeted_k2_gate_10_v4）|
|   |            |      | uav3(drone2) | ok | **blocked**(−0.258) | ok | ok | STATIC_LOS | 静态 37 |

- first_lost_uav：EP1-5=uav1，EP4/6/7(uav3 部分)=uav3，EP7 双机先后（uav2 DYN、uav3 STATIC，两机**不同原因**）。
- FOV_CAUSED=0、RANGE_CAUSED=0。K3 事件中的 FOV 事件（d0E8）与 RANGE 事件（d1E6）**均未产生任何 binary loss**——只是连续 margin 下探。
- 动态真值备注：动态障碍位置 = 场景正弦模型，elapsed 基准比 CSV 可见性时间超前 ≈+1.5 s（`dynamic_elapsed=motion_start` 基差）；以该偏移验证，uav2 的动态 10 号遮挡成立（min −0.17），production 与 tracker 同基一致。

## 5. 逐 episode production 链路追踪（§5、§13）

对 7 个 episode 逐一核对 production 证据链（RUN PROVEN，blocker id 与 §4 几何真值比较）：

| EP | EXECUTED_LOS_BLOCKED | RAW_LOS_DETECTED | production blocker (current-los-blocked) | 与几何真值 | DESCRIPTOR/trigger | K3 事件 (limiter) | SIDE dispatch grant | SIDE commit 活动 | 执行序列（损失段） |
|---|---|---|---|---|---|---|---|---|---|
| 1 | YES | YES | d0 b36 S | **一致** | 10 次 trigger | d0E1/d1E1/d2E1 (STATIC) | YES (d0E2/E3) | PLUS 提交后翻 MINUS (traj21/22) | SIDE_MINUS→**NOMINAL**→FALLBACK→NOMINAL→SIDE_MINUS |
| 2 | YES | YES | d0 b26 S | **一致** | 5 | d0E3 (STATIC) | YES | PLUS commit traj33/25 | SIDE_PLUS→FALLBACK→**NOMINAL**(余下 1.07s) |
| 3 | YES | YES | d0 b28 S ×3 | **一致** | 8 | d0E4 (STATIC, **M2_PREEMPTED**)+d0E5 | YES (d0E5) | 9 次 commit (traj77–84) | SIDE_MINUS→FALLBACK(损失中段 0.8s)→NOMINAL |
| 4 | YES | YES | d2 b6 S ×2 | **一致** | 7 | d2E4 (STATIC) | YES | PLUS/MINUS commit (traj115–121) | SIDE_MINUS(损失前)→**NOMINAL(整个损失)**→SIDE_PLUS(恢复后) |
| 5 | YES | YES | d0 b13 S ×2 | **一致** | 7 | d0E9 (STATIC, 初始窗早于实际损失 0.34s，grant 窗已刷新) | YES ×3 | 5 次 commit (traj219–223)，全在损失前 | **NOMINAL(整个损失)** |
| 6 | YES | YES | d2 b10 **D**→d2 b14 S | **一致**（先动态后静态，与遮挡演进同步） | 33 | d2E5(EARLY_CLOSE)+d2E6 (DYNAMIC_LOS) | YES ×3 | SIDE_MINUS traj245 commit@损失前 1.2s | FALLBACK→NOMINAL→**FALLBACK/SIDE谱系(损失中段 1.1s)**→NOMINAL |
| 7 | YES | YES | d1 b10 **D**; d2 b37 S | **一致**（uav2 动 10 / uav3 静 37，两机各自正确） | 41 | d1E4 (DYNAMIC_LOS) + d2E7 (STATIC) | YES ×2 | SIDE_PLUS/SIDE_MINUS commit | uav2: **NOMINAL(整个损失)**; uav3: SIDE_MINUS(损失前)→**NOMINAL(整个损失)** |

**正面对照（非 LOS 的 margin 下探被正确拒绝 dispatch）**：d0E7（limiter=DYNAMIC_LOS）、d0E8（FOV）、d1E6（RANGE）→ raw scan 无 witness → `stage=NO_SIDE_DISPATCH`，且均无 binary loss。这证明 authority 闸门按物理证据工作，不是按 limiter 标签。

**blocker 身份正确率：7/7（含双机双障碍的 EP6/7 逐一对应），RAW_LOS_MISSED=0。**

## 6. 绕行方向几何核验（§10）

blocker-target frame：`e = normalize(C_blocker − P_target)`，`n_R = (e.y, −e.x)`（与源码 L9424-9425 frame_right 定义一致）；`s(t) = n_R·(P_uav − C_blocker)`，正 = PLUS 侧。逃逸原则：从当前符号侧向外逃逸（同侧逃逸不穿阴影轴）。

| EP | uav | s(loss begin) | s(mid) | 正确几何侧（同侧逃逸） | 执行 | 判定 |
|---|---|---|---|---|---|---|
| 1 | uav1 | **+0.76** | −0.01 | PLUS | SIDE_MINUS（穿轴，损失中段 min −0.298 加深） | **对侧执行，方向次优嫌疑（INFERENCE）**；0.90s 恢复 |
| 2 | uav1 | +0.93 | +0.02 | PLUS | 损失段 NOMINAL | UNRESOLVED（side 未执行） |
| 3 | uav1 | +1.07 | +0.12 | PLUS | 前 0.2s SIDE_MINUS + 中段 SIDE 谱系 | SIDE 谱系同向（MINUS 与 PLUS 判定冲突——两次 commit 换了侧），UNRESOLVED |
| 4 | uav3 | **+2.39** | +2.33 | PLUS（本已深在同侧） | 损失段 NOMINAL（SIDE_MINUS 在损失前，SIDE_PLUS 在恢复后） | UNRESOLVED；注意 uav3 本就深在 PLUS 侧 2.4m，±0.7m side bulge 增益天然有限 |
| 5 | uav1 | +0.70 | −0.13 | PLUS（同侧）| 损失段 NOMINAL | UNRESOLVED |
| 6 | uav3 | +0.37* | +2.08 | PLUS | SIDE 谱系守住中段 | 执行侧与逃逸侧一致（对静态 14 而言） |
| 7 | uav3 | −0.96 | −0.20 | MINUS（同侧） | 损失段 NOMINAL | UNRESOLVED |

\* EP6 的 s 以静态 14 号为参考；动态 10 号扫过时段的侧别参考系不稳定，标 UNRESOLVED。

**SIDE 标签 ≠ 实际几何**的实例：EP4 中 d2E4 的 SIDE 请求为 PLUS、几何同侧 PLUS，但损失段执行的是 NOMINAL；EP1 请求/执行 MINUS 而几何同侧是 PLUS。未选候选的多项式不存在 → 其 actual side 全部 **UNRESOLVED_DUE_TO_MISSING_CANDIDATE_TRAJECTORY**，本表只用 executed 轨迹。

## 7. 触发时机 timeline（§15）

| EP | 事件创建（对实际损失的提前量） | 首个 grant | 首个 SIDE commit | 损失开始 | 恢复 | 损失段执行 |
|---|---|---|---|---|---|---|
| 1 | d0E1 −1.57s | −0.31s | −1.28s (MINUS) | 7.91 | 8.81 | 翻转（见 §5） |
| 2 | d0E3 −1.49s | −0.98s | −0.16s (PLUS) | 10.68 | 11.95 | FALLBACK 0.2s→NOMINAL |
| 3 | d0E4 −2.34s | −1.68s | −1.20s | 25.22 | 26.51 | SIDE 谱系中段 0.8s |
| 4 | d2E4 −2.29s | −1.68s | −0.20s | 35.51 | 36.48 | NOMINAL |
| 5 | d0E9 −2.24s（预测 crossing 63.51，实际 63.95） | −0.95s | −2.05s（5 次连续 commit） | 63.95 | 65.05 | NOMINAL |
| 6 | d2E5 −2.25s | −1.98s | −1.19s (SIDE_MINUS traj245) | 70.68 | 72.48 | SIDE 谱系中段 1.1s |
| 7 | d1E4/d2E7 −1.0s | −0.6s | −0.17s | 73.48 | 74.38/74.05 | NOMINAL |

回答 §15 的四选一：**不是“没有识别”，也不是“识别太晚”**（创建提前 1.0–2.3s，grant 提前 0.3–2.0s）；**是“识别并授权后，已提交的 SIDE 守不住执行权”**（6/7 episode 在损失段翻回 NOMINAL），叠加约 1s 的物理穿出阴影时间。场景元数据佐证：场景按 fixed-trace 设计的 K2 损失窗（如 [8.06,8.527]=0.47s），实际 EP1 执行损失 0.90s ≈ 设计的 1.9 倍（INFERENCE：翻转损耗，非受控对比）。

## 8. 第一处分歧 funnel（§11，每 episode 一个主类）

```text
A. LOS truth 未被检出            = 0/7
B. raw LOS 检出但 blocker 错      = 0/7
C. raw LOS 对但 descriptor 无 LOS = 0/7
D. descriptor 有 LOS 但事件/授权未触发 = 0/7   （M2 抢占 1 次: d0E4，sibling E5 接管，非损失点）
E. dispatch 了但 PLUS/MINUS 构造失败   = 0/7   （7/7 episode 均有 frozen SIDE 候选）
F. 构造成功但 hard safety 拒绝         = 0/7   （f128: 61 batch 有 hard-safe SIDE）
G. hard-safe SIDE 存在但 selection 没选 = 0/7 （按 f128 定义无指标违背；平局翻回计入 H）
H. SIDE 已选/commit 但被翻回 NOMINAL   = 6/7   （EP1,2,4,5,7 整段/大段翻回；EP3 部分守住）
I. SIDE 已执行但方向错误/恢复失败       = 1/7   （EP1 穿轴，INFERENCE，未失败）
J. 非 LOS loss（本不该触发）           = 0/7
```

## 9. 总 funnel（§17）

```text
TOTAL_K3_LOSS_EPISODES = 7 (8.233 s)

LOS_CAUSED = 7/7 (100%)
  STATIC_LOS = 5 (EP1,2,3,4,5)
  DYNAMIC_LOS = 1 (EP7-uav2)
  MULTIPLE = 2 (EP6: 静14+动；EP7: uav2 动10 + uav3 静37)
  FOV = 0    RANGE = 0

LOS_CAUSED episodes:
  RAW_LOS_DETECTED      = 7/7      RAW_LOS_MISSED = 0
  DESCRIPTOR_LOS_AUTHORITY = 7/7   DESCRIPTOR_LOS_MISSING = 0
  SIDE_DISPATCHED       = 7/7      NO_SIDE_DISPATCH = 0
  SIDE_GENERATED        = 7/7      SIDE_BOTH_FAILED = 0
  HARD_SAFE_SIDE_EXISTS = 7/7 episode 级（batch 级 61/135）
  CORRECT_SIDE_IDENTIFIABLE = 3 (EP1,3,6 executed side spans)
  CORRECT_SIDE_SELECTED/HELD = 2 (EP3,6 中段守住)
  WRONG_SIDE_SELECTED   = 1 (EP1 穿轴, INFERENCE)
  UNRESOLVED            = 4 (EP2,4,5,7: 损失段 NOMINAL，反事实不可得)
```

## 10. 四选一主结论（§18）

```text
1. LOS DEFINITION WRONG          — REFUTED。静态三处实现同源同语义（§3）；动态同 predictor；
                                    blocker id 与几何真值 7/7 一致（含双机双障碍）。
2. LOS DETECTION/AUTHORITY WRONG — REFUTED as primary。检出 7/7、descriptor 7/7、dispatch 7/7；
                                    非 LOS 下探被正确拒派（3 个阳性对照）；M2 抢占 1 次由 sibling 接管。
3. SIDE GENERATION/DIRECTION WRONG — 部分成立（唯一的残留失效类）：
                                    生成与 blocker-relative 构造 7/7 发生，但 (a) 6/7 episode 已提交
                                    SIDE 被翻回 NOMINAL（保持失败，非生成失败）；(b) 1/7 穿轴执行
                                    （EP1, INFERENCE）。即"触发了、生成了、但证明不了并守不住"。
4. LOS IS NOT THE MAIN K3 LOSS CAUSE — REFUTED。7/7 全部 LOS 造成；FOV/RANGE 0。
```

```text
PRIMARY_ROOT_CAUSE   = 3（精确化：SIDE 生成/触发正确，失效在“保持与证明”——
                        截断窗内指标证明不了 SIDE 更早恢复 → 平局/更差 → ORIGINAL_ORDER 翻回 NOMINAL，
                        6/7 episode 损失段由 NOMINAL 飞越阴影）
SECONDARY_ROOT_CAUSE = EP1 穿轴执行（1/7，方向次优嫌疑）；d0E4 M2 抢占 1 次（已由 sibling 覆盖）
```

## 11. 与 Feedback128 的衔接与修正（§13）

- blocker 索引（36/26/28/13/6/14/37/10）全部经 scene+executed 几何重新验证成立。
- 修正：f128 中 EP3/EP6 的“executed=NOMINAL”（按样本聚合）实为“SIDE 谱系守住了损失中段”（FALLBACK lineage），EP1/2/4/5/7 维持 NOMINAL 主导。
- f128 的“窗口截断”发现与本案自洽：截断窗使 SIDE 的恢复优势不可证 → 翻回 NOMINAL（H 的机制根源）。

## 证据等级图例

SOURCE PROVEN：§1、§2、§3 全部源码结论。RUN PROVEN：§4 episode 真值、§5 链路追踪、§7 timeline、funnel 计数。INFERENCE：EP1 穿轴次优、EP1 损失 1.9× 设计窗。UNRESOLVED：未执行候选的实际侧别（EP2,4,5,7 反事实）、EP6 动态扫过时段侧别。

---

```text
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO

TOTAL_K3_LOSS_EPISODES: 7
LOS_CAUSED_EPISODES: 7
STATIC_LOS_CAUSED: 5
DYNAMIC_LOS_CAUSED: 1
FOV_CAUSED: 0
RANGE_CAUSED: 0
MULTI_CAUSE: 2

RAW_LOS_MISSED_COUNT: 0
LOS_DESCRIPTOR_MISSING_COUNT: 0
LOS_NO_SIDE_DISPATCH_COUNT: 0
LOS_SIDE_BOTH_FAILED_COUNT: 0
LOS_HARD_SAFE_SIDE_EXISTS_COUNT: 7 (episode-level; 61/135 batch-level)

CORRECT_SIDE_PROVEN_COUNT: 3 (executed side spans: EP1,3,6)
CORRECT_SIDE_SELECTED_COUNT: 2 (EP3,6 held through loss mid-section)
WRONG_SIDE_SELECTED_COUNT: 1 (EP1 axis-crossing, INFERENCE)
SIDE_DIRECTION_UNRESOLVED_COUNT: 4 (EP2,4,5,7 counterfactual unavailable)

PRIMARY_ROOT_CAUSE: class 3 refined — LOS trigger/generation healthy end-to-end; committed SIDE candidates cannot prove earlier recovery inside the truncated K3 window and get flipped back to NOMINAL (6/7 episodes), so the executed path flies through the remaining shadow (~1 s physical passage per episode)
SECONDARY_ROOT_CAUSE: one axis-crossing SIDE execution (EP1, direction suboptimal, INFERENCE); one M2 preemption (d0E4, covered by sibling event)

LOS_DEFINITION_STATUS: CONSISTENT (static: same scene source & margin-equivalent semantics in evaluator/raw-scan/tracker; dynamic: same predictor base; blocker identity 7/7 match)
LOS_AUTHORITY_CHAIN_STATUS: HEALTHY (detect 7/7, descriptor 7/7, dispatch 7/7; 3 positive controls correctly refused)
SIDE_DIRECTION_STATUS: GENERATED 7/7; direction provably correct in 2/3 executed spans, 1 suspected crossing; retention is the failure, not direction computation

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

**一句话回答：K3 loss 时全部 7 个 episode 都是真实 LOS 遮挡（5 静 + 1 动 + 2 混合），production 从 raw LOS 检出、blocker 识别、descriptor LOS 位到 SIDE dispatch 授权全链 7/7 正确——没有触发失败；损失保留是因为已提交的 SIDE 在截断的 K3 窗口内证明不了更早恢复而被翻回 NOMINAL（6/7 episode），加上约 1 s 的物理穿出阴影时间，即“绕行 topology 被触发了，但守不住执行权”，而不是“没有触发正确的绕行 topology”。**
