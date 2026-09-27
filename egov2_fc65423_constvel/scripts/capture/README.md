# ALP 录制 / 抽帧 / 分析工作流(RRCT 五段闭环的 ALP 适配版)

对应 RRCT 工作流的 S0..S5,全部脚本在本目录。一次改动只跑一次;抓帧链路只读运行;
派生 RViz 配置只写 capture 目录;生产配置 / 场景 / 阈值一个字节不动。

## 0. 一键运行

```bash
cd /home/bob/ALP/egov2_fc65423_constvel
bash scripts/capture/run_alp_capture_once.sh --timeout 300
# 输出 capture_dir= 与 run_dir=,runner 退出后自动停抓帧并写 manifest.json
```

产物:
```
captures/<stamp>/  frames/*.jpg + frames.jsonl + manifest.json
                   rviz_capture.rviz + rviz_config_changes.json(派生配置与全部改动清单)
                   viz_clock.jsonl(wall_ns↔ros_ns 时钟采样)
                   capture_summary.json(帧数/失败率/degraded 标记)
runs/<RUN_ID>/     canonical runner 的全部证据(roslaunch_stdout.log、visibility*.csv、
                   roslaunch_argv.txt、exit_status.txt)
```

分析:
```bash
python3 scripts/capture/analyze_alp_pixels.py   --capture-dir captures/<stamp>     # S3:纯像素出"现象",不读日志
python3 scripts/capture/analyze_alp_correlate.py --capture-dir captures/<stamp> \
                                                 --run-dir runs/<RUN_ID>           # S4:odom 权威复核 + 日志拉行
```

## 1. 五段闭环(ALP 适配点)

| 段 | RRCT 原版 | ALP 版 | 关键差异 |
|---|---|---|---|
| S0 | run_a2_once.sh | run_alp_capture_once.sh → canonical runner --headless | 仿真核心由唯一入口管;RViz 由本工作流自己起(生产 launch 的 rviz 配置路径是硬编码的,所以用 --headless + 独立 rviz 加载派生配置) |
| S1 | rviz_make_capture_config.py | 同名 | **追踪目标红球**:Orbit 视角 Pitch=π/2、Target Frame=alp_capture/target(由 annotator 广播的 TF;ALP 没有 target TF,不能用 RRCT 的现成帧),距离按 H_view/(2·tan(fov/2)) 反算并自检;**必须把 /alp_capture/vehicle_overlays 注册成 MarkerArray Display**(rviz 只渲染 Displays 里登记过的话题);窗口 1848x1016;面板折叠;全部改动写 rviz_config_changes.json |
| S2 | rviz_capture_frames.py | 同名 + alp_capture_annotator.py | **多 RViz 共存**:窗口按 `_NET_WM_PID == 我们的 rviz PID` 锁定,xprop/xwininfo/xwd 的 `-display` 必须放参数最前;master 就绪判定用 `rosnode list`(裸 TCP 只说明端口 bind,XML-RPC 未必就绪);**`export DISABLE_ROS1_EOL_WARNINGS=1`**(Noetic EOL 公告弹窗恰好盖住画面中心的追踪区) |
| S3 | analyze_a2_pixels.py | analyze_alp_pixels.py | 载具身份:生产三机是**同一套同色 mesh**,像素法分不开 → annotator 在 /alp_capture/vehicle_overlays 发三个实心覆盖球(纯可视化,不喂 planner);HSV 表见下;顶部 130 px UI 条带遮罩(工具栏 + 折叠面板头红色关闭按钮落在 rose 波段) |
| S4 | roslaunch_stdout.log 按时刻拉行 | analyze_alp_correlate.py + 手工 grep | 对齐钟:ALP 日志时间戳是 **wall epoch**,frames.jsonl 的 wall_ns 直接对齐,无需插值;权威数字来自 runs/<RUN_ID>/visibility_trajectory.csv 的 odom(timestamp 列同为 wall epoch) |
| S5 | 回源码 | 同 | 按现象时刻 grep,不通读 |

## 2. 载具调色板(HSV,OpenCV H 单位 0-179)

| 身份 | 颜色 | 来源 | HSV 区间 |
|---|---|---|---|
| target | 红球 (1.0, 0.05, 0.02) | 生产自带 | H 0-10, S≥120, V≥80 |
| uav0 (drone_0) | magenta (1.0, 0, 0.65) | 覆盖球 | H 150-167 |
| uav1 (drone_1) | lime (0.6, 1.0, 0) | 覆盖球 | H 36-50 |
| uav2 (drone_2) | rose (1.0, 0.15, 0.35) | 覆盖球 | H 168-178 |

选色原则:避开场景已有色相(森林绿 H≈67、动态障碍琥珀 H≈20、可见性墙蓝 H≈111、
路径线蓝/橙/青、FOV 线青/橙/紫),三机之间互相远离。覆盖球半径 0.45 m、alpha 0.92、
lifespan 0.6 s,发布 20 Hz。

## 3. 像素法成立的前提与常数

相机固定俯视 + 跟随目标 ⇒ 像素↔世界是**固定比例 + 恒定偏移**的相似变换:

* 米/像素 = visible_height(24 m) / 帧高(实测逐帧取),默认 0.02362 m/px;
* 实测跟随质量:目标中位偏移 (4.12, 0.38) m,**p90 漂移 0.06 m**(恒定偏移不破坏比例,只记录不修);
* 静止判据:< 0.14 m/s(≈6 px/s)持续 ≥ 2.0 s;贴近上报:两两 ≤ 0.6 m;
* blob 过滤:面积 40-30000、fill ≥ 0.58(去拖尾/树冠/字幕框)、长宽比 ≥ 0.45。

两条铁律沿用 RRCT:
1. 安全数字一律来自 odom / trajectory(visibility_trajectory.csv),像素只定位
   "哪一段、哪几架、大概多久";
2. S3 不接受任何日志输入 —— 它读了日志就只是日志的另一种排版。

## 4. 本次落地过程踩过的坑(前车之鉴)

| 现象 | 根因 | 修法 |
|---|---|---|
| runner 在 BOOT-12 **通过后** exit 1 | 成功路径引用从未赋值的 `BOOT12_TARGET` 等,`set -u` 杀掉 runner(run 20260919_023458 实证) | run_alp_full_on.sh:初始化 BOOT12_* 并在检测函数里置 YES(分项累积语义) |
| 721 帧全是"静止画面" | ROS Noetic **EOL 公告弹窗**盖住画面中心(不是 No Master!) | `export DISABLE_ROS1_EOL_WARNINGS=1` |
| 三机覆盖球从未渲染 | rviz 只渲染 Displays 登记过的话题,派生配置里没有该 Display | S1 里自动追加 rviz/MarkerArray display |
| (333,71) 常驻 rose 色块 100% "检出" | 折叠面板头的**红色关闭按钮**落在 rose 波段 | 分析器遮掉顶部 130 px |
| 抓帧找不到窗口 | xprop 参数里 `-display` 放在 `-id` 之后,`:0` 被当成属性名 | `-display` 一律放最前 |
| rviz 弹 master 错误框的风险 | 裸 TCP 就绪 ≠ XML-RPC 就绪 | `rosnode list` 做真实就绪判定,重试 60 s |
| wrapper 启动即死 | `set -u` 下 ROS setup.sh 引用未绑定 `ROS_DISTRO` | source 前后 `set +u / set -u` |
| 后台命令无声消失 | `pkill -f "rosmaster.*11361"` 匹配到**自己的命令行** | pkill/grep 的 pattern 不要包含会出现在自身命令行里的字样;清理用精确 PID |
| BOOT-12 门假失败(exit 12)2/5 次 | 门是"3 s 内 x 位移>0.05 m"的瞬时判据;目标停止后永远不可能通过 | **未修完**(run 20260919_030527 就是假失败,但仿真与数据完整);应改成累积位移/轨迹 id 推进 |

## 5. 纪律清单(沿用 + ALP 化)

1. 一次改动只跑一次;wrapper 自带并发拒绝(master 端口 + 同名进程)。
2. 现象先用像素确认,再读日志,最后回源码,顺序不可颠倒。
3. 安全数字只认 odom / trajectory / verifier;像素只用于定位。
4. 派生 RViz 配置只写 capture 目录;生产配置、场景、路线、阈值一律不动。
5. annotator 只发布 /alp_capture/* 与 alp_capture/* TF,生产链路无消费者;红线同
   "ground-truth 工具不进生产"。
6. 抓帧降级(capture_summary.degraded)的运行要标注,不与完整 capture 混着下结论。
7. 停滞统计要分段:目标停止(76.5 s 后)的静止是保持/泊车行为,与运动阶段的停滞
   性质不同,必须分开统计。
8. 每次运行至少保留 capture_dir 与 run_dir 两处证据,manifest.json 绑定两者。
