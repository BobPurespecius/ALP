# ALP 目标跟踪、door/stair 与 Rao 2025 复现工作指南

更新时间：2026-08-10  
宿主机工作区：`/home/bob/ALP/guidance/ros_ws`  
容器：`px4_noetic_dev`  
容器内工作区：`/app/guidance/ros_ws`

本文汇总本轮交互中的目标、约束、概念结论、修改过程、验证数据和未完成事项。后续工作应先阅读本文，再按实际代码和最新测试结果继续，不能只根据某次口头结论推断当前状态。

当前状态（2026-08-06）：

- door/stair 启动链路的 6 个阻断性问题已经修复，并完成静态检查、ROS 话题级测试、四种 launch 解析和 20 秒启动级 smoke test。
- 当前没有运行中的 `px4_noetic_dev` 容器。
- narrow door/stair climb 的四组 110 秒真实飞行回归尚未完成，因此不能把“可以启动”写成“场景实验已经验收”。
- 宿主机 Livox 配置和已有 devel/CMake 缓存仍含 `/app/guidance/ros_ws/...` 容器路径；这是当前完整宿主机回归的环境阻碍，不是本轮链路修复已经消除的问题。
- Rao 复现第一阶段已建立论文参数、随机种子、简单重规划和统计协议；旧 `rao_2025_full_reproduction.py` 被明确降级为历史诊断原型，后续实现不得继续向该单节点堆逻辑。

## 1. 用户目标

本轮工作包含两条相互关联但必须隔离的链路。

### 1.1 现有 EGO/EGOv2 door/stair 目标跟踪链路

- 修正 `./batch_run_target_tracking.sh narrow_door 1 110 egov2` 的启动异常；`ego` 只保留为兼容别名，不再作为正式入口。
- 检查 narrow door 和 stair climb 启动链路，防止 FOV/EGOv2 修改误伤纯 EGO、door 或 stair。
- 保留现有 EGO/EGOv2 架构，不使用 SRW，不重写 planner。
- FOV 模式是在 EGOv2 基础上增加可见性、共享和优化代价；纯 EGO 模式不能被改坏。
- narrow door 和 stair climb 都必须验证，不能只针对单个地图调参。

### 1.2 独立 Rao et al. 2025 论文复现

- 论文：`Multi-UAV trajectory planning with field-of-view sharing mechanism in cluttered environments: application to target tracking`。
- 使用单独启动脚本，不干扰现有 door/stair、EGO/EGOv2 启动架构。
- 可以复用现有 Docker 容器，并用 RViz 展示场景、目标、UAV、轨迹、视线和安全走廊。
- 需要实际运行、检查数据、说明偏差原因，不能只写启动文件或声称“已复现”。
- 结果不好时必须如实记录，不以调统计口径掩盖算法缺陷。

旧原型入口（只用于历史诊断）：

```bash
cd /home/bob/ALP/guidance/ros_ws
./run_rao_2025_reproduction.sh
```

无 RViz 的限时测试：

```bash
RAO_RVIZ=false RAO_DURATION=20 \
RAO_CSV_PATH=/tmp/rao_2025_result.csv \
./run_rao_2025_reproduction.sh
```

## 2. 运行和修改边界

1. 不关闭或删除用户正在使用的 `px4_noetic_dev` 容器。
2. ROS、Gazebo、PX4/MAVROS 命令优先在容器内执行。
3. 不回退用户已有修改，不使用 `git reset --hard` 或大范围覆盖。
4. 独立 Rao harness 只能修改以下独立文件，除非用户另行授权：
   - `run_rao_2025_reproduction.sh`
   - `src/target_tracking/launch/rao_2025_full.launch`
   - `src/target_tracking/config/rao_2025_full.yaml`
   - `src/target_tracking/scripts/rao_2025_full_reproduction.py`
   - `src/target_tracking/rviz/rao_2025_reproduction.rviz`
   - `src/target_tracking/docs/rao_2025_reproduction.md`
   - `run_rao_2025_stage1.sh`
   - `src/target_tracking/config/rao_2025_protocol.yaml`
   - `src/target_tracking/launch/rao_2025_stage1.launch`
   - `src/target_tracking/scripts/rao_2025_protocol.py`
   - `src/target_tracking/test/test_rao_2025_protocol.py`
   - `src/target_tracking/docs/rao_2025_stage1_protocol.md`
5. 不用独立 harness 替换 `batch_run_target_tracking.sh`，也不让其发布到现有 planner 使用的 `/object_odom` 等话题。
6. FOV/EGOv2 和纯 EGO 共用文件时，参数必须有模式开关；纯 EGO 默认行为必须保持。
7. 每次算法修改先做语法检查和短测，再跑 20 秒或更长统计测试。
8. 评价结果至少同时检查可见性、碰撞、机间距、EKF 误差、跟踪误差和实际采样频率。

## 3. `local_position`、目标位姿和先验

### 3.1 `local_position` 是什么

`local_position` 通常表示 UAV 自身在本地 ENU/NED 坐标系中的实时位置，不是目标位置。它由 PX4 状态估计器融合 IMU、GPS、视觉或其他定位源得到，经 MAVROS 发布。项目中常见控制输出话题为：

```text
/uav_X/mavros/setpoint_position/local
```

EGO 使用的 UAV 状态在本实验中由 `TargetTrack.py` 发布到：

```text
/drone_<id>_visual_slam/odom
```

当 `use_gazebo_truth_for_feedback=true` 时，这个 odometry 的位置来自 Gazebo pose bridge；否则来自飞行器反馈状态。

### 3.2 为什么目标位姿不能直接用 `local_position`

`local_position` 描述观察者 UAV 自身状态。目标位姿描述被跟踪对象状态，两者不是同一物理量。跟踪控制通常计算：

```text
跟踪误差 = 目标位姿或相对跟踪点 - UAV 自身 local position
```

因此必须分别获得 UAV 位姿和目标位姿，不能把 UAV 的 `local_position` 当作目标。

### 3.3 当前实验中的目标位姿来源

现有 door/stair 目标跟踪链路由单独的 `target_state_coordinator.py` 生成唯一目标状态：

- `target_mode=waypoints`：从场景/launch 配置的目标航点按给定速度插值得到。
- `target_mode=ellipse`：按配置的椭圆运动模型生成。
- 协调节点等待三架 UAV 都进入 `TRACK`，再发布唯一的 ROS 开始时间。
- 三个 `TargetTrack.py` 都订阅 `/target_tracking/ground_truth`，不再各自推进目标轨迹。

目标信息流分为真值、局部观测和共享估计三层：

```text
/target_tracking/ground_truth
  -> /target_tracking/observations/uav1..uav3
  -> target_state_coordinator
  -> /object_odom
```

`planner_backend=egov2` 是显式 baseline：协调节点将真值标记为 `baseline_truth_target` 后送入 `/object_odom`。`planner_backend=fov` 只接受通过实际 UAV yaw、距离和 LOS 检查的局部观测；协调节点选择最新观测，做有限时长的常速度预测，再发布共享 `/object_odom`。没有新鲜共享目标时，控制桥保持，不得用 ground truth 生成 fallback。

### 3.4 是否使用先验

需要区分“仿真目标如何产生”和“planner 如何获得目标”：

- 仿真层使用预设 waypoints、速度或运动模型，这属于实验场景先验。
- planner 运行时只接收 `/object_odom`；FOV 模式下该话题来自最近一次可见观测的共享估计。
- planner 不应直接读取完整未来 waypoint 列表作为规划真值。
- 当前局部观测仍是无噪声仿真测量，只增加了 FOV/距离/LOS gating，尚未加入真实检测器噪声、漏检和通信延迟。

## 4. 启动异常 `RLException: unused args`

原始报错列出了：

```text
broadcast_traj_send_topic
broadcast_traj_recv_topic
max_jer
relative_tracking_x/y/z
use_fov_tracking
use_fov_costs
fov_*
object_velocity_deadband
swarm_clearance
```

ROS `include` 的规则是：调用方传入的每一个 `<arg>` 都必须在被 include 的 launch/XML 顶层声明。出现 `unused args` 说明调用方与实际解析到的 `advanced_param.xml` 版本不一致，常见原因包括：

1. include 到了 EGO v1 的 `advanced_param.xml`，却传入了只在 EGOv2/FOV 文件声明的参数。
2. `ROS_PACKAGE_PATH` 顺序错误，`$(find ego_planner)` 找到了另一套工作区。
3. 调用 launch 新增了参数，但被 include 文件没有同步声明。
4. 宿主机和容器挂载、build/devel 环境解析到了不同文件。

检查命令：

```bash
docker exec px4_noetic_dev bash -lc '
source /opt/ros/noetic/setup.bash
cd /app/guidance/ros_ws
rospack find ego_planner
roslaunch --files multi_uav_formation target_tracking_platform_single.launch
'
```

再逐项检查实际文件：

```bash
rg -n '<arg name="(broadcast_traj|max_jer|relative_tracking|use_fov|fov_|object_velocity|swarm_clearance)' \
  src/ego-planner/src/planner/plan_manage/launch/advanced_param.xml \
  src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/launch/advanced_param.xml
```

目标跟踪启动已固定使用 EGOv2 tracking workspace 的 `advanced_param.xml`，batch 同时验证 `rospack find ego_planner` 的实际路径。每个 planner 进程还通过 `verify_egov2_exec.py` 校验 ROS package 路径和实际二进制目录，任何 EGOv1/EGOv2 混搭都会以状态 78 终止 launch。`ego` 仅保留为带警告的兼容别名，正式命令使用 `egov2`。修改后必须分别验证：

```bash
./batch_run_target_tracking.sh narrow_door 1 110 egov2
./batch_run_target_tracking.sh stair_climb 1 110 egov2
./batch_run_target_tracking.sh narrow_door 1 110 fov
./batch_run_target_tracking.sh stair_climb 1 110 fov
```

### 4.1 六项阻断修复

本轮修复的是启动链路和目标信息流的根因，不是仅针对 `unused args` 增加参数：

1. **package/XML/binary 隔离**：目标跟踪显式使用 EGOv2 tracking workspace 的 package、planner binary 和 `advanced_param.xml`；启动前及每个 planner 进程内都检查实际解析路径，禁止 EGOv1/EGOv2 混搭。
2. **backend 命名统一**：正式名称改为 `egov2`；旧 `ego` 只做兼容映射并打印警告。`setup_runtime_env.sh` 已支持 `EGO_PLANNER_VARIANT=egov2`。
3. **FOV 模式真正启用**：batch 的 `fov` 模式启用 FOV planning/costs、候选目标、可见安全区域和目标安全投影，并使用非零 FOV 距离、高度、角度和 yaw 权重；`egov2` baseline 通过 `use_fov_tracking=false` 和 `use_fov_costs=false` 关闭 FOV 总功能。
4. **单一目标时钟**：`target_state_coordinator.py` 是唯一目标轨迹推进者；等待三架 UAV 全部进入 `TRACK` 后只发布一次共享 ROS 开始时间，三个 `TargetTrack.py` 不再各自推进目标。
5. **可见观测共享**：FOV 模式只融合 `/target_tracking/observations/uav1..uav3` 中通过可见性检查的观测，并做有限时长常速度预测；不再由三架 UAV 分别直接发布 `/object_odom`，也不允许丢失观测后回退到真值。
6. **可见性顺序和 stale hold**：先使用实际反馈位置/yaw 计算可见性，再发布观测；共享目标过期时控制桥保持当前位置并拒绝残留 planner command。CSV 增加 `feedback_yaw_rad`，分析脚本优先使用该字段。

## 5. 现有 door/stair 与 FOV/EGOv2 注意事项

- `planner_backend=egov2` 是 EGOv2 baseline；旧名称 `ego` 会映射到 `egov2` 并打印警告。
- `planner_backend=fov` 是 EGOv2 加 FOV planning、FOV costs 和可见观测共享。
- baseline 的 `/object_odom` 是明确标记的仿真真值；FOV 的 `/object_odom` 只能来自局部可见观测及有限时长预测。
- FOV 观测超时后协调节点停止发布共享目标，控制桥忽略残留 planner 指令并保持；不能静默回退到 ground truth。
- `target_visible` 必须用实际反馈 yaw 计算，不能用尚未执行的 yaw command；CSV 同时记录 `yaw_rad` 和 `feedback_yaw_rad`。
- 通过 batch 启动时，`fov_planning_enabled`、候选目标、可见安全区域和目标投影在 FOV 模式默认打开；baseline 通过 backend 条件关闭 `use_fov_tracking` 和 `use_fov_costs`，因此候选项即使保留 launch 默认值也不会进入 FOV 分支。底层 XML 自身的默认值不能替代 batch/launch 的显式模式配置。
- `TargetTrack.target_visible_from_self()` 当前检查水平距离、水平视场角和 LOS；尚未实现独立的垂直 FOV 角度 gating，不能在实验结论中声称已经验证垂直 FOV。
- narrow door 可锁定跟踪高度；stair climb 必须允许 z 变化，距离和障碍判断不能只使用 x-y。
- `L_ag`、`L_yaw`、候选目标和可见安全区域此前曾造成贴墙、过冲或规划失败。它们现在按 FOV 模式要求默认启用，但在四组完整回归前只能视为“功能已接通”，不能宣称参数稳定或 3D 安全性已经验证。
- 固定相对跟踪点只是 EGO 输入/编队参考，不应被描述为 Rao 论文的核心创新。
- 窄门中多机依次通过可以接受；碰撞、进入膨胀障碍或长期没有新轨迹不可接受。
- 机间安全距离在现有 door/stair 链路中按用户意见约 0.5 m；独立论文 harness 当前使用 1.0 m，两者不要混用。

详细历史记录仍可参考：

- `FOV_EGOv2_handoff_zh.txt`
- `agent_handoff_requirements_checklist_zh.txt`

### 5.1 当前话题契约

```text
/target_tracking/ground_truth              唯一仿真目标真值
/target_tracking/ready/uav1..uav3          三架 UAV 的 TRACK 就绪状态
/target_tracking/target_start_time         三架都就绪后发布的唯一目标开始时间
/target_tracking/observations/uav1..uav3   各 UAV 通过可见性检查后的局部观测
/object_odom                               planner 唯一目标输入
```

发布约束：

- `target_state_coordinator.py` 独占当前 door/stair 链路的 `/target_tracking/ground_truth`、`/target_tracking/target_start_time` 和 `/object_odom` 发布职责。
- 三个 `TargetTrack.py` 发布各自的 `ready` 和 `observations`，但其 launch 参数 `publish_object_odom=false`，不得绕过 coordinator 直接喂 planner。
- `egov2` baseline 的 `/object_odom` 明确标记为 `baseline_truth_target`；`fov` 模式使用 `shared_target_from_uavN` 标记观测来源。
- FOV 观测超过有限预测时限后 coordinator 停止共享输出；控制侧必须保持，不能消费过期命令或静默切换到 ground truth。

## 6. 独立 Rao 2025 旧原型

历史 full 单节点近似包含以下组件：

1. bicycle target model。
2. constant-velocity EKF。
3. 三机几何 FOV/LOS 与目标坐标共享。
4. 离散加速度 motion primitives 和 kinodynamic beam search。
5. 基于预测目标姿态旋转的相对编队目标。
6. 遮挡时搜索无遮挡跟踪位置的 `W_n` 近似。
7. safe flight corridor 近似。
8. 分段 quintic、MINCO-like 轨迹表示。
9. SciPy L-BFGS-B 后端优化。
10. formation、distance、FOV、LOS、yaw、jerk、动力学、障碍、corridor 和 reciprocal avoidance 代价。
11. 异步滚动规划、计划重锚定、安全投影。
12. RViz markers、Path 话题和 CSV 指标。

这不是论文算法的合格实现。当前 safe corridor 是轴对齐 box，所谓 MINCO 只优化固定时间的少量中间点，目标没有 Hybrid A*，规划是三机集中顺序执行且执行层会投影修正碰撞。禁止再通过局部调权重修补这条路径。

## 7. 已完成的 Rao 修正过程

### 7.1 启动隔离

- 新建独立 `run_rao_2025_reproduction.sh`。
- 默认启动 `rao_2025_full.launch`。
- ROS 话题位于 `/rao_2025` 命名空间。
- 不启动 PX4、Gazebo、EGO/EGOv2、door 或 stair。

### 7.2 算法链路补全

由最初的简单运动学演示扩展为 full 节点，加入前端搜索、走廊、五次轨迹、L-BFGS-B、多种 penalty、异步规划和 CSV 指标。

### 7.3 性能和几何修正

- `_line_clearance()` 改为 NumPy 批量计算，避免 Python 障碍循环拖慢规划。
- LOS inflation 与 UAV 碰撞半径分开，避免用机体半径误判相机视线。
- 规划目标被遮挡时围绕目标搜索最近无遮挡槽位。
- 后端加入 LOS clearance penalty。
- 异步计划接收时按当前 EKF 目标位置平移并以当前 UAV 状态重锚定。
- 删除执行层对优化轨迹和目标点的硬混合，避免破坏优化结果。
- 机间安全投影改为所有 UAV 更新后进行对称 reciprocal projection。
- 可见性和 EKF measurement 改为 UAV 执行移动后计算。
- annulus LOS、点碰撞、后端障碍 penalty 和安全投影开始区分环内自由空间，当前以 0.10 m 环壁作保守近似。

### 7.4 参数对齐修正

full 配置原先使用：

```text
tracking_distance=2.8
relative z=1.2
```

这与论文初始同高编队和项目已有 FOV 参数不一致，现已改为：

```yaml
tracking_distance: 1.68
relative_positions:
  - [-1.45, 0.85, 0.0]
  - [-1.70, 0.0, 0.0]
  - [-1.45, -0.85, 0.0]
```

搜索和优化规模已从 `beam_width=10`、`max_iterations=2`、`sample_count=4` 调到 `24/8/8`。该调参尚未证明可见性更好。

## 8. 测试结果和结论

### 8.1 door/stair 修复验证

已完成：

- EGOv2 三个 include 的参数闭包一致，调用方传入但 XML 未声明的参数数目为 0。
- Python `py_compile`、shell `bash -n`、XML 解析和 `git diff --check` 通过。
- ROS 话题级测试通过：无观测时没有 `/object_odom`；三架 ready 后只有一个开始时间；收到 UAV2 观测后输出标记为 `shared_target_from_uav2`；超过预测时限后停止共享输出。
- `narrow_door/stair_climb` 与 `egov2/fov` 的四种 launch 组合均能解析。
- 宿主机进行了 20 秒 FOV 启动级 smoke test：三个 EGOv2 planner、三个 `TargetTrack.py` 和 coordinator 均启动，未出现 `RLException`、`unused args` 或 EGOv1/EGOv2 混搭错误。

尚未完成：

- 当前没有 `px4_noetic_dev` 容器，无法按原容器链路执行四组 110 秒飞行回归。
- 宿主机 Livox 参数仍引用 `/app/guidance/ros_ws/src/livox_laser_simulation/scan_mode/mid360.csv`，已有 devel/CMake 配置也残留 `/app/...` 路径。
- 20 秒 smoke test 只证明启动链路和进程组合成立，不证明 UAV 已完成过门/爬楼、FOV 持续锁定、轨迹无碰撞或统计指标合理。

恢复正确容器或重建无 `/app` 残留的宿主机工作区后，必须运行第 4 节列出的四组 110 秒命令，并使用 `analyze_target_tracking_runs.py` 统一分析结果。

### 8.2 独立 Rao harness 数据

论文 Table 1：

```text
3-visible: 57.6%
2-visible: 32.2%
1-visible: 10.2%
0-visible: 0.0%
```

此前最好的 20 秒结果：

```text
samples=390, rate=19.5 Hz
3-visible=47.2%
2-visible=34.6%
1-visible=10.3%
0-visible=7.9%
mean EKF error=0.065 m
mean tracking error=2.095 m
min pair distance=1.001 m
obstacle collisions=0
UAV collisions=0
shared target available=100%
```

该结果的 2-visible、1-visible 和安全性合理，但 0-visible 未满足论文结果。

移动后再计算可见性的 5 秒结果：

```text
samples=101
3-visible=26.7%
2-visible=32.7%
1-visible=24.8%
0-visible=15.8%
mean EKF error=0.065 m
mean tracking error=1.994 m
min pair distance=1.530 m
collisions=0/0
```

严格按论文 Eq. 34 让 yaw 跟随速度，并对齐 1.68 m 编队、扩大优化规模后，3 秒诊断结果为：

```text
samples=61
visible[3/2/1/0]=[0,4,50,7]
visibility rejects: range=0, fov=122, los=3
```

因此当前主要问题不是最大距离或障碍遮挡，而是速度方向与目标方位夹角超出 ±42.5°。跟踪槽位滞后、规划 handover 和单独 yaw 轨迹缺失共同导致 FOV 失锁。

## 9. 独立 Rao harness 未完成状态

以下内容不能标记为已完成：

1. 最新 annulus 几何修改只通过了短测，没有经过完整 20 秒回归。
2. yaw 已改成纯速度方向，短测证明可见性明显变差。
3. 计划中的 hybrid yaw 修正尚未写入代码：正常时跟随速度，若目标将离开 FOV，则受最大 yaw rate 限制地转向目标以恢复观测。
4. 优化规模 `24/8/8` 没有改善短测结果，可能只增加规划延迟，需要测量 plan worker 完成时间。
5. 当前 CSV 不记录 UAV position/yaw/FOV margin，定位 FOV 失锁还不够直接。
6. 论文中的独立 yaw MINCO trajectory 尚未实现；当前只对位置轨迹求解，再在执行层更新 yaw。
7. annulus 的真实倾角 ±0.5 rad 和精确内外径未建模，当前 0.10 m 环壁只是工程近似。
8. 目标自身的 hybrid A* 避障没有复现，当前 bicycle target 只按解析转向/加速度运动。
9. 尚未用 RViz 做最新代码的桌面/持续运行检查。

## 10. 独立 Rao harness 下一步

按以下顺序继续，避免无依据调权重：

1. 给 CSV 增加每架 UAV 的位置、yaw、目标 bearing、FOV margin、LOS clearance 和 plan age。
2. 实现独立 yaw trajectory 或最小化 velocity-yaw 与 target-bearing 的联合约束；不能用瞬时直指目标冒充论文 Eq. 34。
3. 检测 planner worker 延迟，避免接收已经过期的 2.4 秒轨迹。
4. 将 `W_n` 从单一最近候选改为每个预测时刻的可见区域约束，并保证至少一架 UAV 的候选始终可见。
5. 短测通过后再跑 20 秒，并与论文 Table 1 对比。
6. 最后以默认 RViz 启动，人工检查轨迹、视线、走廊和 marker 是否同步。

语法检查：

```bash
python3 -m py_compile \
  /home/bob/ALP/guidance/ros_ws/src/target_tracking/scripts/rao_2025_full_reproduction.py
```

5 秒 smoke test：

```bash
RAO_RVIZ=false RAO_DURATION=5 \
RAO_CSV_PATH=/tmp/rao_smoke.csv \
/home/bob/ALP/guidance/ros_ws/run_rao_2025_reproduction.sh
```

20 秒统计：

```bash
RAO_RVIZ=false RAO_DURATION=20 \
RAO_CSV_PATH=/tmp/rao_20s.csv \
/home/bob/ALP/guidance/ros_ws/run_rao_2025_reproduction.sh
```

CSV 当前字段：

```text
time_s, visible_count, visible_uav1, visible_uav2, visible_uav3,
sharing_source, shared_target_available, ekf_error_m,
mean_tracking_error_m, min_pair_distance_m, obstacle_collision,
uav_collision, front_end_nodes, corridor_count, backend_cost
```

验收时至少满足：

- 稳态频率接近 20 Hz。
- obstacle collision 和 UAV collision 均为 0。
- 独立 harness 的最小机间距不低于 1.0 m。
- `0-visible` 为 0 或能明确解释与论文实现差异。
- EKF 和跟踪误差有限且不持续发散。
- RViz 中实际存在目标、三架 UAV、规划轨迹、safe corridor 和可见性连线。

## 11. 工作树注意事项

当前仓库存在大量用户修改和未跟踪文件，涉及 EGO v1、EGOv2、multi-UAV formation、door/stair、target tracking 和分析脚本。不要把 `git status` 中的所有变化视为本轮 Rao harness 修改，也不要为清理工作树而回退它们。

本轮 door/stair 与 EGOv2/FOV 链路修复的核心文件是：

```text
batch_run_target_tracking.sh
setup_runtime_env.sh
src/multi_uav_formation/launch/target_tracking_platform_single.launch
src/multi_uav_formation/scripts/TargetTrack.py
src/multi_uav_formation/scripts/target_state_coordinator.py
src/multi_uav_formation/scripts/verify_egov2_exec.py
src/multi_uav_formation/scripts/analyze_target_tracking_runs.py
src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/launch/advanced_param.xml
```

本轮独立 Rao harness 的核心文件是：

```text
run_rao_2025_reproduction.sh
src/target_tracking/config/rao_2025_full.yaml
src/target_tracking/launch/rao_2025_full.launch
src/target_tracking/scripts/rao_2025_full_reproduction.py
src/target_tracking/rviz/rao_2025_reproduction.rviz
src/target_tracking/docs/rao_2025_reproduction.md
```

任何后续结论都应附带实际运行命令、CSV 路径和统计结果。

## 12. Rao 复现第一阶段（2026-08-10）

完整后续实施方案见 [rao_2025_reproduction_plan.md](/home/bob/ALP/guidance/ros_ws/src/target_tracking/docs/rao_2025_reproduction_plan.md)。

第一阶段新增独立入口 `run_rao_2025_stage1.sh`，只校验并发布实验协议，不启动 planner、仿真器或 RViz。协议文件为 `src/target_tracking/config/rao_2025_protocol.yaml`，输出 `/rao_2025/protocol/ready`、`/rao_2025/protocol/manifest` 和 JSON manifest。

论文实验必须区分两个 profile：

```text
simulation  = Section 4.1, 65 x 30 x 4 m, 140 obstacles, Figure 5-7
comparison  = Section 4.2, 50 x 30 x 5 m, 155 obstacles, Figure 8-10/Table 1
```

Table 1 的 `57.6/32.2/10.2/0.0%` 只能与 `comparison` profile 比较。论文没有公布地图种子、障碍类型比例、目标初态/路线、规划周期、预测周期、运行时长、编队向量、噪声和权重；这些值在协议里单独标记为 reproduction choices，不能写成论文参数。

重规划契约保持简单：每架 UAV 固定 0.5 秒周期；空闲才开始一次规划；忙时跳过 tick；成功后在下一控制 tick 替换；失败时执行旧轨迹到其有效期，随后 hold；目标更新不额外触发重规划；执行层禁止安全投影。

可见率按执行位姿和共享前的真实传感器几何在 20 Hz 采样，完整评估窗口所有样本进入唯一一个 `3/2/1/0-vis` 桶。共享观测只能帮助规划，不能把没有实际看见目标的 UAV 计为 visible。

宿主机协议测试已通过 6 项单元测试，并分别成功生成 4.1/4.2 manifest；宿主机 Noetic 也已实际启动节点并验证 latched 话题。当前 `px4_noetic_dev` 未运行，且宿主机 catkin 的旧 CMake cache 仍指向不存在的 `/app/guidance/ros_ws/src`，所以本阶段没有伪造 catkin 构建通过结论。容器内 ROS 节点验证需在容器恢复后执行：

```bash
RAO_PROFILE=simulation ./run_rao_2025_stage1.sh
RAO_PROFILE=comparison ./run_rao_2025_stage1.sh
```

### 阶段 2：场景与目标（2026-08-11）

阶段 2 已新增独立 `run_rao_2025_stage2.sh`，仅启动协议、场景和目标节点，不启动 PX4、Gazebo、EGO/EGOv2 或现有目标话题。场景按 profile 固定尺寸和障碍数，环形障碍保留外半径、内半径、壁厚和倾角；目标使用论文 8 状态 bicycle model，并用离散 steering/acceleration motion primitives 做 kinodynamic Hybrid A*。

阶段 2 话题：

```text
/rao_2025/scene/geometry
/rao_2025/scene/markers
/rao_2025/scene/ready
/rao_2025/target/ground_truth
/rao_2025/target/path
/rao_2025/target/state
/rao_2025/target/marker
/rao_2025/target/ready
```

协议节点是 manifest 的唯一构建者和写入者。scene/target 节点不再各自读取 YAML，而是等待同一个 latched manifest；scene geometry 同时携带 profile 和协议 SHA-256，target 在运行前拒绝不匹配的场景。这避免了多个节点使用不同配置或并发覆盖 manifest。

目标模型严格按论文式 (46) 的 8 状态顺序发布。特别是 `dot(a_tgn) = v_tg^2 * kappa_tg` 必须积分，不能把 `v^2*kappa` 直接赋给状态，也不能漏掉速度平方。motion primitive 在搜索、碰撞检查、预测终点和真实执行中统一用 20 Hz 固定步长积分，避免 planner 用单步 `0.3 s`、执行用 `0.05 s` 所造成的状态分叉。

目标规划保持一段前瞻：执行当前 waypoint 段时最多存在一个下一段规划线程，路段结束后原子切换；不在主循环中同步阻塞，也不使用直线 fallback。到达 waypoint 除位置容差外，还必须存在一个能够进入新离散搜索状态的无碰撞后继，防止下一段从不可推进的终点开始。RViz 配置为 `src/target_tracking/rviz/rao_2025_stage2.rviz`。

目标初态、wheelbase、clearance、goal tolerance、离散搜索参数、cost weights、环形障碍三维解释和 waypoint 均未由论文公开，必须继续称为 reproduction choices。当前两套 waypoint 是在各自确定性场景内验证过的连续闭环，不是论文作者路线，也不是 UAV planner 可读取的未来目标先验。

当前已验证 12 项协议/模型测试，包括式 (46)、规划/执行同积分器、场景确定性和两个 profile 连续两轮闭环。真实 ROS/RViz 验证结果：

```text
simulation: 300 samples, 20.0010 Hz, obstacle collision samples = 0
comparison: 400 samples, 20.0067 Hz, obstacle collision samples = 0
simulation/comparison: 60 s 定时运行均 clean exit，无下一段预规划超时或无路径
RViz 1.14.26: 配置加载成功，OpenGL 4.1，scene/path/odometry/heading 显示节点正常启动
```

运行命令：

```bash
RAO_PROFILE=simulation RAO_DURATION=30 ./run_rao_2025_stage2.sh
RAO_PROFILE=comparison RAO_DURATION=30 ./run_rao_2025_stage2.sh
```

`duration > 0` 时 target 到时正常退出；由于 launch 将其标记为 `required`，roslaunch 会显示红色 `process has died` 关闭横幅，但同时应出现 `process has finished cleanly` 且脚本退出码为 0。这不是规划失败。真正失败会由 target 节点抛出 Hybrid A*、场景一致性或预规划错误。
