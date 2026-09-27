# Scenario A FULL ON + native RViz：完整运行记录

本次用户另行要求的单次完整仿真，使用当前最终工作树，未修改生产代码或调参。指定 catkin build 全部6包通过。目标路线80.204541秒正常结束，统一分析窗口80.154617秒；native RViz已启动并目视核验场景渲染，截图、完整ROS日志/PolyTraj/PositionCommand/odom/Target/Guide/team solution及运行hash均在 `scenario_a_20260911_234722/`。10项离线分析与世界时间对齐完成。

| 指标 | 结果 |
|---|---:|
| 合围 / 同半平面 | 53.47% / 25.27% |
| Camera-time / mean visible | 230.397 camera·s / 2.8744 |
| K2 / All3 / None | 99.67% / 87.77% / 0% |
| 最长K2 loss / blackout | 0.266s / 0s |
| 实际activation Hz | 3.892, 4.130, 4.192 |
| 排除brake后的activation Hz | 3.393, 3.743, 3.655 |
| Team proposal / commit / adoption | 1 / 0 / 0 |
| 共同参考覆盖 | 0s / 0% |
| terminal hold / 提前耗尽 | 0 / 0 |
| collision / swarm violation / 未验证执行采样 | 0 / 0 / 0 |
| planner / command / odom速度峰值 | 3.1481 / 3.1481 / 3.3037 m/s |
| P/V/A残差最大值 | 0.00011056m / 0.00309418m/s / 0.05704699m/s² |

“两前一后”实际持续60.290秒，local曲线中持续60.749秒；前方二机bearing差小于25°同时一机在后，实际2.502秒、local 2.994秒。无已接受team参考，不能归因于team目标函数；布局主要已存在于local曲线。

持续协同未验证成功：第4次joint attempt（任务约7秒）进入后未返回，coordinator不再处理新输入；此前唯一proposal因两机NO_PENDING_TOPOLOGY_GENERATION拒绝。关闭期间有boost mutex异常日志，尚不能把它当作运行期停滞根因。用户已要求继续定位并修复，后续另行记录。

与feedback_38第二次原始运行描述性比较：合围16.90%→53.47%、同半平面64.98%→25.27%、camera-time165.347→230.397，hold与接触采样本次均0；两次共同参考覆盖仍均0。这是不同运行结果，不是因果证明。名义3m/s超速仍存在，本次不改限制或brake策略。

原数据 `scenario_a_20260911_234722/run`，统计 `scenario_a_20260911_234722/analysis`。本轮ROS/RViz已结束；RRCT未访问，历史数据及工作树保留。
