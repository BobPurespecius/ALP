# Scenario A 单次完整仿真

使用当前 workspace 已构建程序，复用上一轮 FULL ON + native RViz 参数；未修改生产代码或调参。目标完整路线时长80.204541秒，正常结束，退出码0。

- 实际合围比例：60.77%（25°/170°）。
- 同半平面比例：21.97%。
- 三机实际 activation：4.080, 4.641, 4.267 Hz。
- terminal hold / 轨迹提前耗尽 / collision samples / swarm violation samples / 未验证执行 samples：0 / 0 / 0 / 0 / 0。
- planner / command / odom 速度峰值：3.147 / 3.147 / 3.212 m/s。名义3m/s超速仍存在。

完整数据、启动参数、ROS日志、PolyTraj、PositionCommand、odom、Target/Guide记录及RViz窗口截图保存于 `/home/bob/ALP/egov2_fc65423_constvel/scenario_a_20260911_140639/run/`。统计在 `analysis/`；安全数值为记录采样结果。本轮进程已退出，独立ROS端口已关闭，见 cleanup.json。仅运行一次，没有重复试验。
