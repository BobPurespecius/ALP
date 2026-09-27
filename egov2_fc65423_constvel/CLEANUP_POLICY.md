# 仿真数据与构建产物清理规定（长期遵守）

本文件是本工作区的**长期清理规定**，每次运行前后都按此执行。

## 一、清理原则

**仿真数据在完成必要的记录与分析后即可删除。**
判定顺序：先确认"必要记录"是否已经落到报告或紧凑摘要中；已落盘的，原始数据即可清除。

## 二、必须保留

| 类别 | 内容 |
|---|---|
| 源码 | `ros_ws/src/**` |
| 报告 | 根目录 `*.md`（审计、验收、根因、设计报告） |
| 反馈 | `feedback/`、`feedback_7_audit/` |
| 场景 | `ros_ws/src/multi_uav_formation/scenes/*.json` |
| 运行必需 | `ros_ws/devel/**`（含 symlink 与实库）、`ros_ws/build/**`（增量编译）、`.catkin_tools/` |
| 运行脚本 | `run_*.sh`、`cleanup_own_nodes.sh`、`rosconsole_info_only.cfg` |
| 当轮紧凑摘要 | 当前结论所依赖的小体积 CSV/摘要 |

## 三、应当清除

| 类别 | 判据 |
|---|---|
| `sim_*.log`、`*_runner.log` | 指标已归档到报告 → 删除（单个可达数百 MB） |
| `.ros_home/log/*` | rosout 滚动日志，纯运行产物 → 每次运行前后清空 |
| `ros_ws/logs/*` | catkin 每次编译的日志 → 可随时清空 |
| 旧运行目录 | 如 `los_semantics_run*`、`semantic_los_run*`、`target_route_clear_run`、`dynamic_target_*_run`、`nlr_restore_run_*`：报告未引用即可删 |
| 失败运行残留 | 只有表头的 0/4 KB `vis_*.csv`、空日志 |
| `__pycache__/`、`Testing/`、`ros_logs_current/` | 生成物 |
| 大体积轨迹 CSV | 仅在确认指标已抽取后删除；报告直接引用的最新一份保留 |

## 四、无效构建产物的判据

- **不删 `ros_ws/build` 与 `ros_ws/devel`**：删掉会强制全量重编译，而本会话全量构建曾长时间占用，
  属于"有效增量构建产物"。仅在确认需要彻底重建时才动。
- **可删**：`ros_ws/logs/*`（编译日志）、`build/**/CMakeFiles/**/*.o` 之外的孤儿目录、
  已被删除包留下的 build/devel 残留。
- 删除前用 `ls ros_ws/devel/lib/<pkg>/` 验证运行二进制仍是有效 symlink。

## 五、运行期磁盘保护

- 运行前确保空闲 ≥ 2 GB（`.ros_home/log` 的 rosout 会持续增长）。
- 已知风险：`native_egov2_rviz.launch:711` 的 RViz 是 `required="true"`，
  磁盘不足导致 RViz 退出时，roslaunch 会**整体关停**（表现为"目标和无人机都不动"）。
  因此磁盘不足时优先用 `rviz:=false` 跑数据，不要带 RViz 硬跑。

## 六、执行记录

| 日期 | 清理内容 | 释放 |
|---|---|---|
| 2026-09-18 | `sim_run_round2.log`(935M)、`.ros_home/log`(1.1G)、`ros_ws/logs`、14 个失败残留 CSV、10 个旧运行目录(约 450M)、`__pycache__`/`Testing`/`ros_logs_current` | 224M → 2.6G |

## 七、运行方式的强制约定（重要）

**必须使用 `rviz:=true` 路径**（即 `run_round2_full_on.sh` 默认，或 `run_round2_full_on.sh rviz:=true`）。

原因（本轮实测）：
- `run_constvel_gradient_rviz.sh` 的 `rviz:=false` 分支只传 `rviz:=false`，
  而该分支**没有**显式传 `sync_dynamic_motion_to_target_start:=true`，
  实测任务长期停在 `mission_started=0`（目标不动、轨迹 CSV 为空）。
- 已验证成功、能跑完全程到终点的是 `rviz:=true` 分支。

配套要求：
- 运行前空闲 ≥ 2 GB；运行中必须有后台清理器删除已轮转的 `rosout.log.[0-9]*`，
  否则 rosout 会以约 1 GB/4min 的速度吃满磁盘。
- 可配合 `ROSCONSOLE_CONFIG_FILE=rosconsole_quiet.cfg` 降低刷屏量。

## 八、启动失败的真正根因（已彻底解决，2026-09-18）

**根因：UDP 8081 端口被残留 bridge 进程瓜分。**

`swarm_bridge/bridge_node_udp.cpp:88-91` 用 `SO_REUSEPORT` 绑定 8081。历次运行
残留的 `bridge_node_udp` 会同时绑定同一端口，内核把 UDP 包**轮询分发**给所有
socket。于是本次运行的 bridge 平均只收到 1/N 的轨迹包：

```
/broadcast_traj_from_planner  3.55 Hz        ← drone0 正常广播
/broadcast_traj_to_planner    no new messages ← 接收侧收不到
/target_tracking/tracking_ready/uav0=True  uav1=False  uav2=False
→ drone1/drone2 卡在 SEQUENTIAL_START（缺 have_recv_pre_agent_）
→ startup-mission-start 永不发布 → 目标不动 → “仿真启动不了”
```

遗留进程越多成功率越低，所以表现为"时好时坏"。

**永久措施（已实施）**：
1. `cleanup_own_nodes.sh` 的匹配模式已补入 `bridge_node_udp|bridge_node_tcp|
   multi_uav_topology_coordinator|cooperative_viewpoint_manager|target_state_coordinator`；
2. 清理后**强制校验 8081 无监听者**，有则按 pid 强杀，输出 `udp8081_leftover=0`；
3. 任何运行前必须先执行该清理脚本。

**日志策略（2026-09-24 修订）**：ROS 日志不得写入 `/dev/shm` 或其它 tmpfs。
每个 run 使用独立的 `runs/<RUN_ID>/ros_log` 持久目录；旧 run 结束后保留或压缩归档，
不得把日志留在 RAM-backed tmpfs 中。ROS 的 rosout 轮转日志可能达到 GB 级，
反复累积会耗尽 swap 并触发全局 OOM。
