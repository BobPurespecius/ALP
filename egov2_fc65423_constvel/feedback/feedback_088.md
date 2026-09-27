# Feedback 088 — 集中式单变量 ablation mode

日期：2026-09-20  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
范围：只建立论文正式消融基础设施；未增加 recovery、planner authority 或历史组合开关

## 0. 结论

本轮把论文使用的七个科学开关收敛为一个共享的、严格单选的
`AblationMode`。正式运行只需：

```text
./run_on.sh --headless --timeout <seconds> --ablation <mode>
```

八个 mode 由 `traj_utils/ablation_config.h` 唯一定义，Planner、traj_opt 和
team coordinator 读取同一个 `/alp/ablation_mode`，不再各自解释字符串。正式 mode
始终锁定 encirclement、cooperative reference 和 physical safety，始终关闭 joint yaw；
任何 mode 只相对 FULL 关闭一个科学模块。

```text
ABLATION_INFRASTRUCTURE_IMPLEMENTED: YES
CENTRAL_ABLATION_MODE: YES
FORMAL_MODE_COUNT: 8
ENCIRCLEMENT_LOCKED_ON: YES
COOPERATIVE_REFERENCE_LOCKED_ON: YES
PHYSICAL_SAFETY_UNCHANGED: YES
JOINT_YAW_LOCKED_OFF: YES
```

最终构建、集中配置 contract、6 个现有核心 contracts、FULL 60 秒以及其余 7 个
mode 的顺序短时 branch sanity 均通过。所有运行均到达 BOOT-12、`CORE_EXIT_CODE=0`、
`CLEANUP_STATUS=OK`；没有并行启动仿真，也没有遗留 ROS/RViz/Gazebo 进程。

## 1. 集中配置与单变量矩阵

| Mode | LocalVis | TeamPT | LOSTopo | DynBodyTopo | TargetA* | SeedRetain | LocalGap |
|---|---:|---:|---:|---:|---:|---:|---:|
| FULL | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| NO_LOCAL_VIS | 0 | 1 | 1 | 1 | 1 | 1 | 1 |
| NO_TEAM_PT | 1 | 0 | 1 | 1 | 1 | 1 | 1 |
| NO_LOS_TOPOLOGY | 1 | 1 | 0 | 1 | 1 | 1 | 1 |
| NO_DYNAMIC_BODY_TOPOLOGY | 1 | 1 | 1 | 0 | 1 | 1 | 1 |
| NO_TARGET_FACING_ASTAR | 1 | 1 | 1 | 1 | 0 | 1 | 1 |
| NO_SAFE_SEED_RETENTION | 1 | 1 | 1 | 1 | 1 | 0 | 1 |
| NO_LOCAL_GAP | 1 | 1 | 1 | 1 | 1 | 1 | 0 |

每行额外固定：

```text
ENCIRCLEMENT=1
COOPERATIVE_REFERENCE=1
PHYSICAL_SAFETY=1
JOINT_YAW=0
```

`ablation_config_contract_test` 枚举全部八行，验证每个非 FULL 行与 FULL 的 Hamming
distance 恰好为 1，并验证四个实验不变量。逗号组合 mode 与
`--ablation no_team_pt --pt` 均在启动仿真前被拒绝，返回码均为 2。

## 2. Production 接线

共享定义位于已有公共依赖 `traj_utils`：

```text
traj_utils/include/traj_utils/ablation_config.h
traj_utils/include/traj_utils/ablation_runtime.h
traj_utils/src/ablation_config_cli.cpp
traj_utils/test/ablation_config_contract_test.cpp
```

接线语义如下：

- `NO_LOCAL_VIS`：只在 traj_opt 入口令 local directional `J_vis` 及其 P/T 梯度为
  0；raw static/dynamic LOS、ConflictDescriptor、LOS/BODY topology、metrics 和 Team
  P/T 不变。
- `NO_TEAM_PT`：只跳过 `TeamVisibilityOptimizer` 连续 P/T refine；三机 bundle、
  discrete tuple evaluation/ranking、local rolling commit 和安全检查继续运行。
- `NO_LOS_TOPOLOGY`：raw LOS 检测、J_vis 和可见性 telemetry 不变，只撤销 LOS
  descriptor 的 SIDE/topology dispatch authority。
- `NO_DYNAMIC_BODY_TOPOLOGY`：dynamic prediction 与 hard collision preflight 不变，
  只撤销 dynamic BODY descriptor 的 SIDE/topology dispatch authority；static BODY 与
  LOS 不变。
- `NO_TARGET_FACING_ASTAR`：SIDE 仍可调用原 A*，但不再设置 target-facing bias，运行
  日志实际出现 `UNBIASED_ASTAR_COUNT>0`。
- `NO_SAFE_SEED_RETENTION`：只禁止 target-facing A* refinement 失败时继续保留已有
  hard-safe MinJerk seed；raw A* 仍不得直接执行，最终 preflight 不变。
- `NO_LOCAL_GAP`：只令 local circular-gap cost/gradient 为 0；120° cooperative soft
  reference、radial/height tracking、weak bearing recovery、J_vis 和 Team P/T 不变。

没有修改任何权重、clearance、dynamics limit、current-revision、handoff、executor 或
ACK/commit 安全语义。

### LocalVis 计数口径

FULL 不得改变既有算法行为，因此没有把 NOMINAL 强行改成新的 J_vis authority。
`LOCAL_VIS_COST_EVAL_COUNT` 统计 traj_opt 对 local J_vis 模块的求值调用；原有
guidance/context predicate 仍决定该 candidate 是否得到非零 directional cost。
`NO_LOCAL_VIS` 在这一调用之前返回，所以其计数严格为 0。该定义既验证消融接线，又不借
ablation 基础设施改写 FULL。

## 3. Runner、manifest 与辅助工具

正式 `--ablation` 由 canonical runner 单点解析；launch 只接收已验证 mode，并设置唯一
全局参数 `/alp/ablation_mode`。无 `--ablation` 时保留旧 `--pt` 行为；正式 mode 出现后，
Team P/T 由 mode 单独决定，冲突旧参数直接报错。

每次正式运行保存：

```text
runs/<RUN_ID>/ablation_manifest.txt
runs/<RUN_ID>/ablation_counters.txt
runs/<RUN_ID>/roslaunch_argv.txt
```

新增：

```text
scripts/list_ablation_modes.sh
scripts/print_ablation_commands.sh
docs/ABLATION_MODES.md
```

两个脚本只列模式/打印命令，不自动并行运行实验。

## 4. Runtime branch sanity

计数器在第一次命中及 2 的幂次命中时节流记录。因此下表中 `0` 是精确的“未进入”，
正值是该分支至少执行到该观测值的运行证据，不冒充进程退出时的精确总次数。

| Mode / Run ID | LocalVis | TeamPT | LOSTopo | DynBodyTopo | TargetA* | SeedRetain | LocalGap |
|---|---:|---:|---:|---:|---:|---:|---:|
| FULL / `20260920_130154_594353` | 32768 | 256 | 256 | 512 | 32 | 8 | 262144 |
| NO_LOCAL_VIS / `20260920_130357_598123` | **0** | 128 | 32 | 64 | 32 | 4 | 131072 |
| NO_TEAM_PT / `20260920_130453_601811` | 8192 | **0** | 16 | 32 | 128 | 4 | 131072 |
| NO_LOS_TOPOLOGY / `20260920_130548_605462` | 4096 | 128 | **0** | 64 | 64 | 1 | 65536 |
| NO_DYNAMIC_BODY_TOPOLOGY / `20260920_130641_609160` | 8192 | 128 | 16 | **0** | 64 | 1 | 131072 |
| NO_TARGET_FACING_ASTAR / `20260920_130736_612872` | 8192 | 128 | 128 | 128 | **0** | 0* | 131072 |
| NO_SAFE_SEED_RETENTION / `20260920_130831_616572` | 8192 | 128 | 32 | 64 | 64 | **0** | 131072 |
| NO_LOCAL_GAP / `20260920_130925_620262` | 4096 | 128 | 32 | 64 | 32 | 2 | **0** |

`*` `NO_TARGET_FACING_ASTAR` 的 SeedRetain 配置仍为 1，但本轮计数为 0：该机制只消费
target-facing A* 的 hard-safe seed，上游 target-facing attempt 被本模式唯一关闭后，没有
可消费事件。这是因果上的未触发，不是第二个配置位被关闭。相同运行中
`UNBIASED_ASTAR_COUNT>0`，证明 SIDE A* 已恢复 ordinary unbiased A*。

附加运行证据：

- `NO_LOCAL_VIS` 中 raw LOS 非零审计仍出现，LOS topology、Team P/T 均非零；
- `NO_TEAM_PT` 中 topology bundle 接收和 discrete tuple attempt 持续出现；
- `NO_LOS_TOPOLOGY` 中 raw LOS telemetry 和 LocalVis 仍工作；
- `NO_DYNAMIC_BODY_TOPOLOGY` 中 dynamic raw descriptor/风险与 hard checker仍工作；
- 所有模式的 manifest 与三个 runtime component 报告的 mode 一致。

## 5. FULL 60 秒验证

```text
命令：./run_on.sh --headless --timeout 60 --ablation full
FULL_SANITY_RUN_ID: 20260920_130154_594353
BOOT_12: YES
CORE_EXIT_CODE: 0
CLEANUP_STATUS: OK
REAL_ROSLAUNCH_ARGV_HAS_ABLATION_MODE: YES（ablation_mode:=full）
ABLATION_MANIFEST: MATCH_RUNTIME
三机开始运动: YES
TEAM_PT_REAL_ATTEMPT: YES
```

FULL 的 manifest 精确对应 7 个 ON、三项实验不变量 ON、joint yaw OFF；三个生产组件
均记录 `mode=FULL formal=1`。

## 6. 构建与测试

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws
BUILD_PASS: YES

ABLATION_CONFIG_CONTRACT: PASS
EXISTING_CORE_CONTRACTS: PASS
fresh_moving_initializer_contract_test: PASS
local_execution_contract_test: PASS
trajectory_lifecycle_contract_test: PASS
visibility_topology_production_test: PASS
topology_coordinator_contract_test: PASS
team_visibility_optimizer_contract_test: PASS
git diff --check: PASS
```

第一次直接串行运行现有 contracts 时，`visibility_topology_production_test` 等待不存在的
ROS master；该调用被终止。使用临时 `roscore` 重跑后全部通过，临时 master 随即清理。

## 7. 复杂度约束

```text
NEW_SCIENTIFIC_SWITCH_COUNT: 7（集中在一个 AblationConfig，不是7个CLI开关）
OLD_HISTORICAL_SWITCHES_REACTIVATED: 0
NEW_RECOVERY_MECHANISM_ADDED: NO
NEW_PLANNER_AUTHORITY_ADDED: NO
NEW_ROS_PACKAGE_ADDED: NO
NEW_PARALLEL_EXPERIMENT_RUNNER_ADDED: NO
```

## 8. 最终 11 项回答

1. **是。** 一个 `--ablation <mode>` 可选择全部八组，非法组合在启动前拒绝。
2. **是。** contract 证明每个消融相对 FULL 恰好一个 scientific bit 不同。
3. **是。** encirclement 在全部正式 mode 锁定为 1，旧参数不能覆盖。
4. **是。** static/dynamic/swarm/dynamics/current-revision/handoff/executor 安全链未由
   ablation 切换。
5. **是。** `NO_LOS_TOPOLOGY` 只撤销 LOS topology authority；detection、J_vis、metrics
   保留。
6. **是。** `NO_DYNAMIC_BODY_TOPOLOGY` 只撤销 dynamic BODY SIDE authority；dynamic
   prediction 和 hard safety 保留。
7. **是。** `NO_TEAM_PT` 的 bundle、team discrete tuple evaluation/ranking 仍运行。
8. **是。** `NO_TARGET_FACING_ASTAR` 中 target-facing attempt 为 0，ordinary
   `UNBIASED_ASTAR_COUNT>0`。
9. **是。** `NO_SAFE_SEED_RETENTION` 只改变 refinement failure 后的安全 seed 保留；raw
   A* 不执行，final preflight 不绕过。
10. **是。** `NO_LOCAL_GAP` 仍完整保留 cooperative soft encirclement、radial/height 和
    bearing recovery。
11. **是。** 没有把 recovery、shadow planner、transaction、joint yaw 或其他历史机制
    重新接回 production。

```text
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_088.md
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```
