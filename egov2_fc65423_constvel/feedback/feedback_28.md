# ALP 轨迹连续性只读快速审计 — 2026-09-10

结论：没有足够保存数据确认MINCO内部C2破坏、轨迹头切换不连续或command与polynomial不一致。采样到的最强切换邻域瞬态出现在JOINT_TEAM_SOLUTION退出；最高速度事件则在另一个joint进入约0.67 s后。不能用odom邻域有限差分把这两者合并为已证实的同一根因。

输入为最近完整Scenario A FULL ON的原始trajectory/visibility CSV、canonical_rosout日志及源码。证据输出：[trajectory_continuity_audit_20260910](/home/bob/ALP/egov2_fc65423_constvel/trajectory_continuity_audit_20260910)。没有运行仿真、没有修改生产代码。

## 1. 数据可用性与估计口径

记录中没有rosbag、完整PolyTraj coefficient payload，也没有PositionCommand position/velocity/acceleration时间序列；日志仅保存轨迹id、start/activation time、duration/piece_num、source等摘要。visibility CSV保留commanded yaw/yaw_dot，可检查相邻采样变化，但不是完整PositionCommand。不能由fresh-init候选摘要或team级max_velocity反推出实际时刻的速度向量。

沿用完整运行的有效任务区间：mission t=0.051156–80.204540745 s。末尾超速统计不包含target停止后和关闭阶段。切换邻域拟合使用原始CSV，必要时读取窗口外相邻样本以支持边界估计，事件计数只在上述有效区间。

扫描全部[executed-trajectory-source]通知并按UAV排序，以通知中的timestamp作为新source开始执行时刻，共124次可观测切换。19次旧/new trajectory_id和activation_time均相同，是source标签变化，不能直接计为polynomial重置；其余105次身份/时间切换。重复traj-server receive日志按(id,start_time)排重：全原始日志331次receive只有100种(id,start_time)，不把重发当新切换。scheduled activation另由执行source日志覆盖。若同id/start_time的系数被改写，当前摘要无法检测，故不声称穷尽未记录的payload变化。

真正要求的Δp=||p_new(0)-p_old(t_switch)||、Δv、Δa无法计算，精确值均N/A。下表明确是 **odom切换邻域同一时刻外推差（proxy）**：切换前/后各取最多3个实际采样，限制在±0.13 s；分别线性拟合v(t)=v0+a(t-t_switch)，用最近p/v及拟合a作二阶位置外推；两侧在t_switch的p/v/a之差为proxy。它不是polynomial head jump，也不是控制命令跳变。位置点可能是重复发布的最近odom，CSV未保留odom自身header stamp，外推/加速度尤其受采样不同步影响。

heading为两侧拟合水平速度方向的wrap角差，任一水平速度<0.15 m/s则N/A。yaw为unwrap实际yaw两侧线性拟合角差，yaw_rate为该拟合斜率差；另存visibility CSV相邻command yaw/yaw_dot差。后者也包含正常时间演化，不能直接证明yaw不连续。所有124次均有至少两侧2点可拟合。P50/P95为线性分位数。

## 2. Source transition 分类

下列每个三元组均为P50/P95/max，Δp单位m、Δv m/s、Δa m/s²、heading度；均是上述proxy。

| transition | count | Δp | Δv | Δa | Δheading |
|---|---:|---|---|---|---|
| NOMINAL→NOMINAL | 64 | 0.007675/0.032391/0.045748 | 0.025374/0.076502/0.119155 | 0.400065/1.497770/4.323516 | 0.139312/1.171504/5.253197 |
| NOMINAL→SIDE | 2 | 0.005042/0.009336/0.009814 | 0.031408/0.038559/0.039354 | 0.071713/0.112870/0.117443 | 0.143401/0.143401/0.143401 |
| SIDE→NOMINAL | 3 | 0.008550/0.010696/0.010935 | 0.022552/0.066442/0.071319 | 0.896278/1.386691/1.441181 | 0.059441/0.326996/0.356725 |
| NOMINAL→PERSISTENCE_FALLBACK | 15 | 0.013906/0.035361/0.043816 | 0.034187/0.051614/0.059489 | 0.283958/1.731981/2.425485 | 0.091172/0.975398/1.503400 |
| PERSISTENCE_FALLBACK→NOMINAL | 16 | 0.001902/0.030914/0.047285 | 0.022416/0.083003/0.085672 | 0.853043/3.612211/4.377055 | 0.031981/1.014148/1.141183 |
| ANY→JOINT_TEAM_SOLUTION | 12 | 0.018010/0.039633/0.042648 | 0.031202/0.132076/0.215985 | 0.617693/3.061895/3.126768 | 0.263745/1.514321/1.595557 |
| JOINT_TEAM_SOLUTION→ANY | 9 | 0.022092/0.062261/0.065020 | 0.040964/0.260304/0.357506 | 1.274296/3.983952/5.242362 | 0.548177/2.107230/2.492487 |
| 其他 | 3 | 0.005649/0.021808/0.023603 | 0.051202/0.055860/0.056377 | 0.112800/1.569196/1.731018 | 0.187528/1.303534/1.427535 |



| transition | Δyaw ° P50/P95/max | Δyaw_rate rad/s P50/P95/max |
|---|---|---|
| NOMINAL→NOMINAL | 0.263204/1.609550/2.713356 | 0.081713/0.426333/0.536041 |
| NOMINAL→SIDE | 0.268894/0.483143/0.506949 | 0.061834/0.107679/0.112773 |
| SIDE→NOMINAL | 0.286740/1.030747/1.113415 | 0.201976/0.223394/0.225773 |
| NOMINAL→PERSISTENCE_FALLBACK | 0.122680/1.228269/1.319249 | 0.073471/0.281607/0.458243 |
| PERSISTENCE_FALLBACK→NOMINAL | 0.389164/1.422070/1.919252 | 0.088812/0.743828/1.155136 |
| ANY→JOINT_TEAM_SOLUTION | 0.408427/1.958857/2.465401 | 0.330239/0.514338/0.556185 |
| JOINT_TEAM_SOLUTION→ANY | 0.184871/3.834549/5.157275 | 0.168268/5.238613/6.630278 |
| 其他 | 0.221909/1.301826/1.421817 | 0.100899/0.153955/0.159850 |



ANY→JOINT优先归类，其后JOINT→ANY；SIDE_PLUS/MINUS合并SIDE，其余不在指定组合者入“其他”。详细old/new activation、generation、team、hypothesis、相邻原始Δv和command yaw差全部保存在switches.json。

## 3. 单条MINCO内部连续性

MINCO_INTERNAL_CONTINUITY_NUMERIC_CHECK: N/A

position/velocity/acceleration piece-junction最大residual：均N/A；未保存该次执行系数，不能用离线重新生成另一条轨迹冒充原运行验证。

源码确认：`poly_traj_utils.hpp:1255–1302`的MinJerkOpt::generate对相邻piece构造p_left(T)-p_right(0)=0、v_left(T)-v_right(0)=0、a_left(T)-a_right(0)=0的线性方程，A.solve(b)求解；同时约束更高阶导数。这说明算法显式保证C2（数值误差/系数损坏仍需实际payload验证）。单个piece是五次多项式，内部解析光滑。

`traj_server.cpp:220–225`读取coef_x/y/z，:625–628直接getPos/getVel/getAcc/getJer，:676附近将这些量传给publish_cmd。正常执行路径没有独立重插值位置曲线；terminal hold和scheduled activation有单独分支，不能由正常路径代码替代全运行command/poly一致性证明。

## 4. 全部speed>3 m/s事件

有效区间仅1个连续episode，9个样本，UAV1（CSV uav_id=2），t=79.917500–80.185372 s。共同身份：JOINT_TEAM_SOLUTION、trajectory_id35、generation35、team_solution_id6、encirclement_generation56、hypothesis0。generation35是执行source字段，不是ACK中的planning_generation76，不混用。

最近一次切换：绝对时间1789041298.931963，NOMINAL/id34/g34/team0/enc54 → JOINT_TEAM_SOLUTION/id35/g35/team6/enc56。9个超速样本全部位于该切换后0.406116–0.673988 s，全部不在±0.2 s内；区间结束前无下一次切换。

| timestamp | mission s | UAV | source/id/generation/team | odom speed m/s | planner instantaneous v | PositionCommand v | switch±0.2s | 判定 |
|---|---:|---:|---|---:|---|---|---|---|
| 1789041299.338078976 | 79.917500 | 1 | JOINT/35/35/6 | 3.004530 | N/A | N/A | NO | UNKNOWN |
| 1789041299.375792027 | 79.955213 | 1 | JOINT/35/35/6 | 3.450237 | N/A | N/A | NO | UNKNOWN |
| 1789041299.404911041 | 79.984332 | 1 | JOINT/35/35/6 | 3.450237 | N/A | N/A | NO | UNKNOWN |
| 1789041299.439887524 | 80.019309 | 1 | JOINT/35/35/6 | 3.727382 | N/A | N/A | NO | UNKNOWN |
| 1789041299.471569777 | 80.050991 | 1 | JOINT/35/35/6 | 3.910183 | N/A | N/A | NO | UNKNOWN |
| 1789041299.504376650 | 80.083798 | 1 | JOINT/35/35/6 | 4.075698 | N/A | N/A | NO | UNKNOWN |
| 1789041299.537890196 | 80.117311 | 1 | JOINT/35/35/6 | 4.075698 | N/A | N/A | NO | UNKNOWN |
| 1789041299.582093716 | 80.161515 | 1 | JOINT/35/35/6 | 4.294588 | N/A | N/A | NO | UNKNOWN |
| 1789041299.605951309 | 80.185372 | 1 | JOINT/35/35/6 | 4.578530 | N/A | N/A | NO | UNKNOWN |



峰值odom速度向量=(3.733026, 0.925410, -2.484163) m/s，模长4.578530 m/s，明显包含向下速度-2.484163 m/s，不能只用平面heading解释。每个样本完整身份、最近切换前后source/generation、向量数据见overspeed_samples.csv/json。

同一solution6的planner nonlinear日志记载 **team级整条轨迹max_velocity=3.143593 m/s**，已高于名义3.0；这是POLYNOMIAL_ALREADY_OVERSPEED的team级证据，但不是UAV1在峰值时刻的predicted velocity，不能填到上表的instantaneous列。实际4.578530又显著高于该预测峰值，提示跟踪瞬态/命令执行层值得优先核对；缺少完整command与系数，不能断言CONTROLLER/ODOM_OVERSHOOT为主因，更不能凭切换窗口之外就排除延迟控制响应。逐样本保守判UNKNOWN。

## 5. 最严重10个切换邻域事件

按proxy Δv降序，依次用Δa、heading、Δp作同值排序；不是按未保存的polynomial jump排序。异常列在该UAV切换±0.2 s内检查：static<0.1、dynamic<1.1、swarm椭球<0.5及collision≤0。

| mission s / UAV | old source/id/g → new source/id/g | Δp m | Δv m/s | Δa m/s² | heading ° | speed>3 | clearance异常 |
|---|---|---:|---:|---:|---:|---|---|
| 9.515097 / 0 | JOINT_TEAM_SOLUTION/7/g7 → NOMINAL/8/g8 | 0.022092 | 0.357506 | 5.242362 | 2.492487 | NO | 无 |
| 28.125216 / 0 | NOMINAL/20/g20 → JOINT_TEAM_SOLUTION/21/g21 | 0.003233 | 0.215985 | 2.091591 | N/A | NO | 无 |
| 37.005542 / 2 | NOMINAL/19/g19 → NOMINAL/20/g20 | 0.023997 | 0.119155 | 0.564763 | 0.171261 | NO | 无 |
| 9.521482 / 1 | JOINT_TEAM_SOLUTION/5/g5 → NOMINAL/6/g6 | 0.058122 | 0.114501 | 2.096338 | 0.684768 | NO | 无 |
| 16.431403 / 1 | NOMINAL/8/g8 → NOMINAL/9/g9 | 0.008270 | 0.110583 | 4.323516 | 1.836912 | NO | 无 |
| 62.521761 / 1 | NOMINAL/26/g26 → NOMINAL/27/g27 | 0.019810 | 0.092556 | 0.626574 | 0.407161 | NO | 无 |
| 57.431560 / 1 | PERSISTENCE_FALLBACK/24/g24 → NOMINAL/25/g25 | 0.001391 | 0.085672 | 3.357264 | 0.276933 | NO | near_swarm_violation |
| 45.371426 / 1 | PERSISTENCE_FALLBACK/20/g20 → NOMINAL/21/g21 | 0.047285 | 0.082113 | 0.971584 | 1.141183 | NO | 无 |
| 15.401377 / 1 | PERSISTENCE_FALLBACK/7/g7 → NOMINAL/8/g8 | 0.000407 | 0.078191 | 2.929366 | N/A | NO | 无 |
| 22.625986 / 2 | NOMINAL/12/g12 → NOMINAL/13/g13 | 0.045748 | 0.076910 | 0.428512 | 1.176287 | NO | 无 |



最强proxy事件：UAV0，t=9.515097 s，JOINT退出，Δv≈0.357506 m/s、Δa≈5.242362 m/s²、Δp≈0.022092 m。这支持“该切换邻域存在明显跟踪瞬态”，不支持“精确C2头状态跳跃已证实”。未给“位置连续OK、速度加速度坏”作YES判断，因为位置proxy非零也不是C0 residual，且没有统一的精确头状态阈值验证。

已知static collision发生t≈25.42–25.78 s、swarm违规t≈56.88–57.35 s；它们不是上述9个超速样本所在事件。当前审计没有把不同时间的安全问题强行合成一个原因。

## 6. 结论与最小修复方向

下列NO表示未证实，不表示已证明无问题。

```text
IS_MINCO_INTERNAL_C2_BREAK_CONFIRMED: N/A
IS_GENERATION_SWITCH_DISCONTINUITY_CONFIRMED: NO
IS_POSITION_CONTINUITY_OK_BUT_VELOCITY_ACCELERATION_BAD: NO
IS_COMMAND_POLYTRAJ_MISMATCH_CONFIRMED: NO
IS_ODOM_OVERSHOOT_PRIMARY: NO
OVERSPEED_PRIMARY_CAUSE: UNKNOWN（team polynomial已有轻度超速；实际额外增速无法在command/控制跟踪间唯一分离）
WORST_SWITCH_TYPE: JOINT_TEAM_SOLUTION→ANY（本次最差实例为→NOMINAL，按odom proxy）
```

一句根因总结：现有证据显示问题集中在跨轨迹执行/跟踪层的瞬态及joint实际速度超出预测，而不是已证实的单条MINCO内部C2断裂，但缺少poly与command原始载荷使主因仍不能唯一定位。

最小方向：

- **P/V/A head compatibility gate：优先核对并补齐实际active→new在同一activation时刻的P/V/A兼容性。** 现有joint `planner_manager.cpp:1234`已验证proposed对source suffix的P/V/A（1e-5），不能重复实现；:1335对当前active到proposed主要检查位置，普通commit :1949也是位置freshness，不能把source suffix相容当实际active C2相容。

- **C2 transition/blending：暂不认定需要。** 先取得actual active/new系数和PositionCommand，证实头状态不相容后再考虑，不能凭odom finite difference直接加平滑器。

- **Persistence fallback切换前重验证：需要核对既有重验证的时间/实际active身份与共同world-time是否匹配。** 日志已有previous-safe revalidation/KEEP_PREVIOUS_SAFE，不应声称完全没做；建议补齐activation时刻的P/V/A与动态/机间安全一致性，而非只加重复静态检查。

- **Generation switch限制：保留现有stale/identity检查，暂不增加任意限频或禁止joint退出。** 最小动作是加强active-generation+activation head的一致性观测/校验；是否限频取决于直接残差证据。

本轮仅新增离线审计脚本/统计和本报告，没有落实上述生产修复。

```text
PRODUCTION_SOURCE_CHANGED: NO
SIMULATION_RUN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
