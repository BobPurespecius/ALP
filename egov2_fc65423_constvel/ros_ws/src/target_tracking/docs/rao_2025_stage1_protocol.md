# Rao 2025 第一阶段：实验协议

第一阶段只固定实验事实、随机性、重规划契约和统计口径，不实现或替换规划算法。其输出是后续所有节点必须共同使用的 run manifest。

## 两个论文场景

| profile | 论文位置 | 场景 | 障碍数 | 用途 |
|---|---|---:|---:|---|
| `simulation` | 4.1 | 65 x 30 x 4 m | 140 | Figure 5-7 的方法演示 |
| `comparison` | 4.2 | 50 x 30 x 5 m | 155 | Figure 8-10 和 Table 1 对比 |

只有 `comparison` 结果允许与 Table 1 的 `57.6/32.2/10.2/0.0%` 比较。此前在 4.1 场景得到的可见率只能作为开发诊断数据。

论文明确给出的共同参数被锁定在 `config/rao_2025_protocol.yaml` 的 `paper` 区域。论文未给出随机种子、障碍类型比例、目标初态和路线、预测周期、规划频率、实验时长、编队向量、噪声及优化权重；这些项目明确列在 `paper.unreported_parameters`，本项目选择的值只能放入 `reproduction` 区域。

## 简单重规划契约

每架 UAV 使用同一种固定周期循环：到期且 planner 空闲时，以当前执行状态和同一时刻的目标预测做一次完整规划；成功结果在下一控制 tick 原子替换；失败时继续执行上一条仍有效的轨迹；规划尚未结束时跳过 tick。目标消息不会额外触发规划，也不允许同一 UAV 同时存在多个规划任务。

执行层不做碰撞投影或轨迹平移。碰撞监视器只记录事实，不能在执行后修正状态来掩盖规划失败。

## 统计口径

可见性从三架 UAV 的实际执行位姿和相机几何计算，发生在共享之前。共享坐标让遮挡 UAV 可以继续规划，但不会把该 UAV 记成“看见”。每个 20 Hz 控制采样只落入 `3-vis/2-vis/1-vis/0-vis` 一个桶，分母包含完整 60 秒评估窗口的所有样本，不剔除失锁或规划失败样本。

论文没有报告原始随机种子和运行时长，因此相同代码可以确定性复跑本项目实验，但在取得作者场景/日志前，不能声称随机地图和 Table 1 完全同源。

## 使用

宿主机直接校验，不需要 ROS：

```bash
python3 src/target_tracking/scripts/rao_2025_protocol.py \
  --check \
  --config src/target_tracking/config/rao_2025_protocol.yaml \
  --profile comparison \
  --manifest /tmp/rao_2025_comparison_manifest.json
```

在已有 `px4_noetic_dev` 容器中发布 latched manifest：

```bash
RAO_PROFILE=comparison ./run_rao_2025_stage1.sh
```

节点发布：

```text
/rao_2025/protocol/ready
/rao_2025/protocol/manifest
```

第一阶段本身不启动 RViz，因为尚无场景、传感器或规划轨迹可显示。后续阶段的所有节点必须读取同一 manifest，RViz 才能展示与统计数据一致的状态。

实现中只有协议节点允许读取 YAML、构建并原子写入 manifest。scene、target 及后续节点只能消费 latched `/rao_2025/protocol/manifest`；派生数据必须携带 profile 和 `config_sha256`，消费者在启动时拒绝不一致数据。这样 manifest 才是一次运行的唯一事实源。
