# Feedback 124 — F123 方法评判、循环证书修复与一次 FULL 尝试

日期:2026-09-26。工作区:`/home/bob/ALP/egov2_fc65423_constvel`。
任务:阅读 Feedback 123,评判其方法,修复其 FIRST_CAUSAL_FAILURE(peer `checked_until` 递归取 min 导致三机证书无法续期),并运行仿真。
FULL 尝试:run `20260926_164803_727379`(BOOT-07 卡死,证据充分后手动终止;**失败的 FULL 尝试,非有效 production run**)。FULL 后未修改 production。
未访问 /home/bob/RRCT;未执行任何 git reset/checkout/restore/clean。

---

## 1. 对 Feedback 123 方法的评判

**方向正确、实现有一处结构性错误。**

正确的部分:
- 显式验证证书(`checked_until`)随轨迹上 wire(PolyTraj/MINCO float64 + payload hash + revision)是正确的架构方向——它把"执行到哪一刻是已验证的"从隐式约定变成显式契约,executor 在证书终点停止(fail-closed)是安全的。
- 移除 seed-preservation tube 的 hard row/final gate(与 F122 的 D 门槛一致)、Team speculative 只按已验证 horizon 认证——方向正确。
- UNKNOWN fail-closed(MISSING_MAP_PREDICTION_OR_PEER)语义正确并保留。

结构性错误(本次修复对象):
- `hardCheckedUntil()` 对每个 peer 取 `min(peer_end, peer.checked_until)`,而 peer 的 `checked_until` 又包含对我的 `checked_until` 的 min——**证书定义是相互递归的**。bootstrap 时每个本机 horizon 有限(≈2 s),该有限上界经由 peer 项永久传播:任何一方的新 commit 都以其他方的旧证书为上界,证书永远无法越过首次 bootstrap 的 cap;一旦 wall clock 越过该 cap,`peer.checked_until <= activation` 触发 fail-closed → 全员无法 commit → executor 全部 TERMINAL_HOLD。F123 run 的观测(三机 `FINAL_CHECKED_UNTIL` 全部冻结在 1790402091.744443、三机 candidate 明明有 active_end 2094.5–2097.6)与该机制完全吻合。

## 2. 修复(唯一 production 改动,`planner_manager.cpp hardCheckedUntil`)

peer 贡献从"peer 的证书"重建为**纯机械事实 + 显式 hold 校验**,消除递归:

1. `swarm_until = min(swarm_until, peer_end)` —— 只用 peer 已广播多项式的视界(`checkTrajectorySwarmSafety` 实际核验的同一范围,含其越界末端 hold 模型),**不再读取 peer.checked_until 作为上界**。
2. peer 执行器停止点 `fly_until = min(peer_end, max(peer.checked_until, activation))` 决定 peer 在哪里停;`fly_until` 之后 peer 被建模为**静态 hold 点** `p_hold = peer.traj.getPos(...)`(已停止的 peer 冻结在其停止点,而不是广播多项式按时间推进的位置),并对 `(fly_until, swarm_until]` 窗口逐 0.03 s 校验我的轨迹与 `p_hold` 的椭圆距离 ≥ swarm clearance,且 `p_hold` 本身 inflate-free。**peer 多项式末端从不被冒充为已验证末端**——越界部分是被验证的静止点。
3. fail-closed 全部保留:map/predictor 缺失、observed-but-no-prediction、预测窗无效、更低 ID peer 从未发布、peer 轨迹不覆盖 activation、peer 从未有证书(`checked_until <= start_time`)、hold 点被占、hold 点分离违反——任一命中返回 activation(候选 UNKNOWN)。
4. 新增 `[certificate-hold-model]` 节流日志,hold 校验真实触发时有运行时证据。

关键语义变化:peer 证书**过期**(已停止执行)不再阻塞我的续期——它只是把 peer 的运动模型从"飞行多项式"切换为"已验证的静止点"。循环依赖被打断:我的证书只依赖自身 horizon(每周期重锚)与 peer 的机械事实(随 peer 的下一次 commit 前移)。

## 3. 离线确定性验证(/tmp/f124_certificate_check.cpp,14/14 PASS)

复刻修复后的 peer 块,在三机证书交换协议上验证:
- [1] **F123 死锁重放**:ID 序 bootstrap 证书 = 2.1(与 F123 观测一致);cycle 2 续期到 2.6(peer 多项式视界)、cycle 3 到 3.2——证书逐周期前移,旧代码在 cycle 2 即返回 activation。
- [2] 自身 horizon 永远封顶(peer 存活 50 s 时证书仍 = 2.1)——不冒充 peer 末端。
- [3] peer 提前停止:不封顶我的证书(3a);穿越其 hold 点 fail-closed(3b);hold 点在障碍内 fail-closed(3c)。
- [4] 证书已过期的冻结 peer 不阻塞续期(4a);穿越其冻结点 fail-closed(4b)。
- [5] fail-closed 全集保留:缺失更低 ID peer / 过期轨迹 / 从未认证 / 未启动更高 ID peer 跳过(5a–5d)。

`git diff --check` CLEAN;`catkin build -j2 --no-status` **25/25 packages succeeded, warnings 0**。

## 4. FULL 尝试结果(run 20260926_164803_727379)

canonical 命令 + 同场景。BOOT-01→07 正常,BOOT-08 的三机激活检查永不满足,runner 无限重试 BOOT 循环(`--boot-timeout 180` 未生效,console 写到 887 MB),证据充分后手动终止(roslaunch SIGINT + 残留清理为 0;stdout/console 已 gzip,ros_log 归档,run 目录 1.8 GB→433 MB,磁盘可用 4.3 GB)。

**证书修复本身的证据(本轮目标达成)**:
- bootstrap 后 `FINAL_CHECKED_UNTIL` 持续前移:1790412494.69 → 1790412501.28(样本跨度 ≈6.6 s,而 F123 冻结在单一时刻);
- 32 次 `execution-activated`、drone 1 连续 commit 至 trajectory 22;
- `[certificate-hold-model]` 触发 22 次(fly_until < peer_end 的停止点校验真实工作);
- `TERMINAL_HOLD_ENTER` = 4(F123:三机全部进入且永不退出);
- CANDIDATE_CERTIFICATE UNKNOWN=302 全部为 MISSING_MAP_PREDICTION_OR_PEER(fail-closed 正确工作,集中在 bootstrap 与后期 peer 轨迹过期时段)。

**新的 FIRST_CAUSAL_FAILURE(第二层死锁,非证书问题)**:三机起步后 4–7 s(sim t≈8–12)全部停止 commit(drone 0=traj 6、drone 1=traj 22、drone 2=traj 7)。直接证据:drone 0 的候选流从 t≈1790412502 起全部被拒——
- SIDE_PLUS/SIDE_MINUS 优化成功(MINCO_OK)但 `[local-geometry-candidate] safe=0 reason=swarm drone=1 t≈1.5 ellip_dist2=0.17–0.25`(clearance²=0.25,物理间距 ≈0.41–0.50 m);
- NOMINAL 候选 `safe=0 reason=DYNAMICS_FAIL`(fresh-init 快照本身 dynamics_valid,超限发生在优化产物)。
机制:三机起步 offsets 互距 ≈0.85 m(swarm clearance 0.5 m,余量仅 0.35 m),向 target 起步的运动使预测轨迹互相进入 0.5 m 内;**F122 删除了 swarm checker 的隐式 NON_WORSENING 放行后,物理 clearance 是唯一硬判据,没有任何逃逸通道**→ 所有候选被拒 → 三机停在已过期的旧轨迹上 → BOOT-08 死等。这是 F122 "普通 swarm hard checker 删除隐式 NON_WORSENING 放行"的直接后果,与证书无关(证书在该时段正确地随 peer 事实前移/失败)。

诊断性可见性(非有效 run 指标):sim t∈[0,5] K2=100%、ALL3=46.3%;visible<2 的最后时刻 t=10.57 s(此后三机执行停止);全程 K2=13.3% 为失败状态描述,不与 F119/F120 比较。

## 5. 对任务的回答

1. **F123 方法对不对?** 架构方向(显式证书 + float64/hash + executor 停在证书终点)正确;peer 递归 min 是结构性错误,已修复并有离线+运行证据。
2. **问题解决了吗?** F123 的 FIRST_CAUSAL_FAILURE(证书不可续期)已解决:离线 14/14 + run 内证书前移 6.6 s / TERMINAL_HOLD 4 / hold 模型 22 次。
3. **仿真跑了吗?** 跑了一次(canonical 命令),BOOT-07 卡死后手动终止,判定为失败的 FULL 尝试;失败首因已前移到起步 swarm 挤压死锁(F122 删除 NON_WORSENING 放行的后果)。

## 6. NEXT_STEP(仅记录;FULL 后未改 production)

1. **起步/挤压场景的 swarm 逃逸通道**:恢复一个有界的、物理意义明确的放行——例如"候选与 incumbent 相比 swarm 分离不恶化时放行"(F122 删除的 NON_WORSENING 的受控回归),或对互相挤压对允许切向分离分量约束。这是恢复 BOOT-12 的必要条件。
2. NOMINAL DYNAMICS_FAIL 的归因(优化产物超限的具体 v/a/j 分布)。
3. runner BOOT timeout 未生效的 wrapper bug 单独修(本轮 -boot-timeout 180 未终止 BOOT-08 重试循环)。

## 7. 最终字段

```text
PRODUCTION_CODE_CHANGED: YES  (hardCheckedUntil peer block only)
BUILD_PASS: YES (25/25, warnings 0)
GIT_DIFF_CHECK: CLEAN
OFFLINE_CHECKS: PASS (certificate renewal harness 14/14; F123 deadlock replay renewed)
F123_METHOD_VERDICT: DIRECTION_CORRECT_PEER_MIN_STRUCTURALLY_WRONG
CERTIFICATE_RENEWAL_REPAIRED: YES (non-circular peer facts + hold-point verification)
PEER_POLYNOMIAL_END_IMPERSONATION: NO (hold point verified, never trusted)
FAIL_CLOSED_SEMANTICS_PRESERVED: YES (UNKNOWN paths unchanged)

FULL_ATTEMPT_COUNT_THIS_TASK: 1
FULL_RUN_COUNT_THIS_TASK: 0 EFFECTIVE (BOOT-07 stuck; manually terminated)
REPEATED_FULL_SIMULATION_USED: NO
CERTIFICATE_RENEWAL_EVIDENCE: FINAL_CHECKED_UNTIL advanced 1790412494.69->1790412501.28
TERMINAL_HOLD_ENTER: 4 (F123: all three, permanent)
EXECUTION_ACTIVATED: 32; drone1 committed to trajectory 22
HOLD_MODEL_ENGAGED: 22
CANDIDATE_CERTIFICATE_UNKNOWN: 302 (all MISSING_MAP_PREDICTION_OR_PEER)

EARLY_K2_T0_5: 1.0000   EARLY_ALL3_T0_5: 0.4631 (diagnostic only)
LAST_VISIBLE_GE2: t=10.57s (fleet execution stopped)
EXECUTED_ALL3/K2 full-run: NOT_COMPARABLE (failed attempt)

FIRST_CAUSAL_FAILURE_THIS_RUN: startphase swarm squeeze deadlock (all candidates
  swarm-hard-rejected at ellip_dist2 0.17-0.25 < 0.25 after F122 removed the
  implicit NON_WORSENING pass; offsets spacing 0.85 m vs clearance 0.5 m)
NEXT_STEP: bounded physical swarm escape channel; NOMINAL DYNAMICS_FAIL attribution;
  runner boot-timeout wrapper fix
STATIC_CONTACT: 0   DYNAMIC_CONTACT: 0   SWARM_VIOLATION_LOGS: reject-path only
PVA_MISMATCH: 0
ARCHITECTURE_HEALTH: FAIL (second-layer deadlock)
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO (all owned processes verified 0; logs archived)
```
