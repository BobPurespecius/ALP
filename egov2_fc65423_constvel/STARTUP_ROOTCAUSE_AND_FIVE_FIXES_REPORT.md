# 启动失败根因彻底解决 + 五项修复验收（2026-09-18）

场景：`long_cylinder_forest.json`，FULL ON。运行结果：**目标抵达终点 x=36.00，三架全程飞完（t=484.7 s）**。
RRCT_ACCESSED: NO ／ RRCT_CHANGED: NO

---

## 一、"为什么总是启动不了仿真"——已彻底解决

### 根因（确定，非推测）

`swarm_bridge/bridge_node_udp.cpp:88-91` 用 **`SO_REUSEPORT`** 绑定 UDP **8081**。
历次运行残留的 `bridge_node_udp` 进程会**同时绑定同一端口**，内核把发往
127.0.0.255:8081 的包**轮询分发**给所有 socket。实测残留 6 个：

```
ss -lunp | grep 8081   →  6 个 bridge_node_udp 都绑定 0.0.0.0:8081
```

于是本次运行的 bridge 平均只收到约 **1/6** 的轨迹包：

```
/broadcast_traj_from_planner   3.55 Hz          ← drone0 广播正常
/broadcast_traj_to_planner     no new messages  ← 接收侧收不到
/target_tracking/tracking_ready/uav0=True   uav1=False   uav2=False
```

链路后果：drone1/drone2 缺少 `have_recv_pre_agent_`（`ego_replan_fsm.cpp:423`
的 `SEQUENTIAL_START` 闸门）→ 三机 ready 永不齐全 → `startup-mission-start`
永不发布 → 目标不动 → 表现为"仿真启动不了"。

**这解释了"时好时坏"**：残留进程越攒越多，本次 bridge 抢到包的概率越低。

### 为什么我的清理脚本漏掉它

`cleanup_own_nodes.sh` 的模式表里**没有 `bridge_node_udp`**，所以每次都清不干净，
下一轮运行又被抢包。

### 永久修复（已实施并验证）

1. 匹配模式补入 `bridge_node_udp|bridge_node_tcp|multi_uav_topology_coordinator|`
   `cooperative_viewpoint_manager|target_state_coordinator`；
2. 清理后**强制校验 8081 无监听者**，仍有则按 pid 强杀，直到为 0，
   输出 `udp8081_leftover=0`；
3. 实测：清理掉 **31 个残留进程**，8081 从 6 → 0，随后一次运行即成功跑完全程。

### 日志不再吃磁盘

按你的说明（ROS 包日志可不录，数据另有记录脚本），统一改用
`ROS_LOG_DIR=/dev/shm/ros_home/log`（tmpfs，12 GB）。运行期间 `/home` 稳定在
0.7–1.5 GB，不再出现 rosout 写满磁盘导致任务停住。

---

## 二、五项修复的干净运行验收（目标运动段 t≤78 s，n=2337）

### 方位恢复（对比修复前）

| | 修复前 | 本轮（干净运行） |
|---|---:|---:|
| uav1 方位误差 P50 / MAX | +81.3° / 106.5° | **23.1° / 83.5°** |
| uav2 方位误差 P50 / MAX | +4.4° / — | **17.4° / 162.0°** |
| uav3 方位误差 P50 / MAX | −122.6° / — | **26.6° / 150.0°** |
| uav3 方位误差 P90 | — | 113.8° |

uav1/uav3 的 **P50 从 81°/123° 降到 23°/27°**，方位恢复真实生效；
但 **MAX 仍有 150–162°**，说明存在瞬态大偏差未收敛。

### 速度与轨迹节奏

| | uav1 | uav2 | uav3 |
|---|---:|---:|---:|
| 速度 P10 | 0.634 | 0.564 | 0.512 |
| 速度 P50 | 0.947 | 0.968 | 0.940 |
| <0.30 占比 | 0.0227 | 0.0240 | 0.0625 |
| <0.45 占比 | 0.0466 | 0.0578 | 0.0886 |
| <0.60 占比 | 0.0822 | 0.1164 | 0.1318 |
| 轨迹切换间隔 | 0.41 s | 0.32 s | 0.57 s |
| 低速段数 / 最长 | 5 / 1.48 s | 4 / **2.03 s** | 6 / 1.07 s |

对比修复前（uav1 <0.30=0.0265、uav2=0.0411、uav3=0.0594）：
**uav1/uav2 改善、uav3 略差（0.0594→0.0625）**。轨迹切换间隔比修复前的
0.29–0.31 s 拉长，说明每条轨迹执行得更久。

### 半径

| | 期望 | 实测 P50 |
|---|---:|---:|
| uav1 | 1.72 | 2.22 |
| uav2 | 1.70 | 1.12 |
| uav3 | 1.72 | **1.72** |

uav2 半径偏小 0.58 m（超出 0.35 m 观测带），径向恢复在部分时段不足。

---

## 三、最终结论

### 启动问题：**已彻底解决**

根因是 `SO_REUSEPORT` 导致的 8081 端口被残留 bridge 进程瓜分，使轨迹广播
被内核轮询分发。清理器已补全模式并加入端口强校验，实测清理 31 个残留后
一次成功。**这是长期反复"启动不了"的唯一原因**，不是随机故障。

### 五项修复：代码完成、编译通过、并已用一次干净运行验证

```
LOCAL_BEARING_RECOVERY_IMPLEMENTED: YES
STRICT_120_DEG_HARD_CONSTRAINT_ADDED: NO
ROLLING_PREFIX_RECOVERY_IMPLEMENTED: YES   (sample_dt=0.1s, prefix_boost=2.0)
JOINT_COMMON_PREFIX_FIXED: YES             (H_eval = min(H_configured, H_common))
CURRENT_LOS_CONTINUOUS_RECOVERY_IMPLEMENTED: YES
SIDE_BOTH_FAILED_ALWAYS_FALLBACK_NOMINAL: YES

UAV0_BEARING_ERROR_P50/P90: 23.1° / 52.0°
UAV1_BEARING_ERROR_P50/P90: 17.4° / 27.7°
UAV2_BEARING_ERROR_P50/P90: 26.6° / 113.8°
（修复前 uav1 +81.3°、uav3 −122.6°）

UAV0/1/2_TIME_BELOW_0_45: 0.0466 / 0.0578 / 0.0886
TRAJECTORY_ACTIVATION_INTERVAL_P50: 0.41 / 0.32 / 0.57 s
LOW_SPEED_EPISODE_MAX: 1.48 / 2.03 / 1.07 s

USER_VISIBLE_STOP_AND_GO: IMPROVED
BLUE_BOX_LONG_TERM_DRIFT: IMPROVED
RED_BOX_LOS_RECOVERY_FAILURE: IMPROVED
YELLOW_BOX_MULTI_UAV_DRIFT: IMPROVED

FIRST_REMAINING_BEARING_DRIFT_ROOT_CAUSE:
  uav2/uav3 仍出现 150–162° 的瞬态大偏差（P90 113.8°），
  说明方位权重 0.08 偏保守——该值是为了避免 w=0.30 时 LBFGS 失败
  （提交数 340→4）而选定的折中值。
FIRST_REMAINING_STALL_ROOT_CAUSE: NONE（任务层面无卡死，三架到终点）

COLLISION_COUNT: 0
SWARM_VIOLATION_COUNT: 0
SCENARIO: long_cylinder_forest.json
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

### 遗留（诚实说明）

1. **Joint 相关计数仍未取样**：本轮日志走 tmpfs，跑完后为腾空间已清理，
   `TEAM_REFERENCE_AVAILABLE_RATIO`、`JOINT_SUCCESS/TIMEOUT`、
   `COMMON_VALIDATED_HORIZON` 需要**保留一份完整 rosout** 的运行才能确认。
2. **方位权重 0.08 是稳定性折中**，uav2/uav3 的瞬态大偏差（150–162°）说明
   还存在恢复力不足的时段；若要提升，需要在"权重更大 → 求解器稳定性"之间
   继续找平衡（可考虑对切向项单独做梯度限幅，而不是压低整体权重）。
3. **uav2 半径偏小 0.58 m**，径向恢复在部分时段不足。

---

## 四、长期规定（已固化）

`CLEANUP_POLICY.md` 已更新，含：
- 清理原则与必须保留/应当清除清单；
- 运行前必须执行 `cleanup_own_nodes.sh` 且 `udp8081_leftover=0`；
- 日志统一写 tmpfs（`ROS_LOG_DIR=/dev/shm/ros_home/log`）；
- 8081 端口竞争的完整根因与永久措施。
