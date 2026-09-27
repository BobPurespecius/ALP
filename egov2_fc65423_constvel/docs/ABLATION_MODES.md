# ALP 正式单变量消融模式

**Encirclement is locked ON for every formal ablation.**

正式实验只使用 `./run_on.sh --ablation <mode>`。一次只能选择一个模式；所有模式执行相同的三机目标追踪任务，并始终保持 cooperative 120° soft slot reference、radial/height tracking、weak bearing recovery、物理安全、rolling execution。120° 不是 hard constraint，Joint yaw 始终关闭。

## 配置矩阵

| Mode | LocalVis | TeamPT | LOSTopo | DynBodyTopo | TargetA* | SeedRetain | LocalGap |
|---|---:|---:|---:|---:|---:|---:|---:|
| `full` | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| `no_local_vis` | 0 | 1 | 1 | 1 | 1 | 1 | 1 |
| `no_team_pt` | 1 | 0 | 1 | 1 | 1 | 1 | 1 |
| `no_los_topology` | 1 | 1 | 0 | 1 | 1 | 1 | 1 |
| `no_dynamic_body_topology` | 1 | 1 | 1 | 0 | 1 | 1 | 1 |
| `no_target_facing_astar` | 1 | 1 | 1 | 1 | 0 | 1 | 1 |
| `no_safe_seed_retention` | 1 | 1 | 1 | 1 | 1 | 0 | 1 |
| `no_local_gap` | 1 | 1 | 1 | 1 | 1 | 1 | 0 |

每一行都固定：`encirclement=1`、`cooperative_reference=1`、`physical_safety=1`、`joint_yaw=0`。

## 模式定义

### `full`

- 科学问题：最终完整方法的 production baseline。
- 唯一关闭机制：无。
- 保持不变：全部七个科学机制与全部安全合同。
- 计数：七个 counter 均允许增长。
- 命令：`./run_on.sh --headless --timeout 200 --ablation full`

### `no_local_vis`

- 科学问题：Local directional `J_vis` 及其 P/T gradient 的独立贡献。
- 唯一关闭机制：Local continuous visibility cost/gradient。
- 保持不变：raw static/dynamic LOS detection、LOS/BODY topology、Team P/T、metrics 与 hard safety。
- 计数：`LOCAL_VIS_COST_EVAL_COUNT=0`。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_local_vis`

### `no_team_pt`

- 科学问题：三机连续 Joint P/T 相对 Local + team discrete tuple selection 的增益。
- 唯一关闭机制：`TeamVisibilityOptimizer` continuous P/T refinement。
- 保持不变：coordinator、最多 27 个 tuple 的 screening/ranking、Local N/L/R 与 local first-safe commit。
- 计数：`TEAM_PT_ATTEMPT_COUNT=0`。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_team_pt`

### `no_los_topology`

- 科学问题：连续 `J_vis` 能否替代 LOS 离散 topology。
- 唯一关闭机制：raw LOS witness 的 L/R 与 `LOS_OBSERVATION_SIDE` authority。
- 保持不变：raw LOS detection/metrics、Local `J_vis`、BODY topology 与全部 safety。
- 计数：`LOS_TOPOLOGY_DISPATCH_COUNT=0`。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_los_topology`

### `no_dynamic_body_topology`

- 科学问题：DYNAMIC BODY 是否需要提前产生 N/L/R。
- 唯一关闭机制：动态 BODY 的 SIDE authority。
- 保持不变：dynamic prediction、soft risk/cost、physical hard clearance、final dynamic preflight；STATIC BODY 和 LOS topology 不变。
- 计数：`DYNAMIC_BODY_TOPOLOGY_DISPATCH_COUNT=0`。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_dynamic_body_topology`

### `no_target_facing_astar`

- 科学问题：target-facing homotopy 是否优于 ordinary unbiased A*。
- 唯一关闭机制：A* 的 target-side-first half-space search。
- 保持不变：普通 A*、Local-SFC、MINCO、N/L/R、`J_vis`、Team P/T 和 hard preflight。
- 计数：`TARGET_FACING_ASTAR_ATTEMPT_COUNT=0`；`UNBIASED_ASTAR_COUNT` 可增长。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_target_facing_astar`

### `no_safe_seed_retention`

- 科学问题：MINCO refinement 数值失败后保留 target-facing MinJerk initializer 的贡献。
- 唯一关闭机制：refinement failure 后的 initializer retention。
- 保持不变：raw A* 永不执行；成功 refinement 和所有 dynamics/static/dynamic/swarm/revision/handoff/final preflight 不变。
- 计数：`SAFE_SEED_RETENTION_USED_COUNT=0`。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_safe_seed_retention`

### `no_local_gap`

- 科学问题：在 cooperative 120° soft reference 和 weak bearing recovery 已存在时，Local 25°–170° circular gap 的独立贡献。
- 唯一关闭机制：Local `J_gap` cost/gradient。
- 保持不变：完整 cooperative soft encirclement、radial/height/bearing、Local `J_vis`、Team P/T 与 safety。
- 计数：`LOCAL_GAP_COST_EVAL_COUNT=0`。
- 命令：`./run_on.sh --headless --timeout 200 --ablation no_local_gap`

## 证据

每次正式运行保存：

- `runs/<RUN_ID>/ablation_manifest.txt`：共享 `AblationConfig` 导出的最终 effective config；
- `runs/<RUN_ID>/roslaunch_argv.txt`：包含真实 `ablation_mode:=...`；
- `runs/<RUN_ID>/ablation_counters.txt`：从 runtime 日志提取的七个 branch counter；
- `runs/<RUN_ID>/resolved_params.txt`：最终 launch 参数。

列出模式：`scripts/list_ablation_modes.sh`。打印而不执行全部正式命令：`scripts/print_ablation_commands.sh`。
