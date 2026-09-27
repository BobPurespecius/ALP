# 唯一任务几何 authority（阶段 B）。
# 唯一 producer: cooperative_viewpoint_manager。
# 消费者: FSM/PlannerManager（软合围参考上下文）、multi_uav_topology_coordinator
# （Team target-centered 的 desired_radius/bearing/height 基线）。
# Team 禁止再用 phi0 + 2*pi*i/3 自造任务方位 —— 一切以本消息的 slot 映射结果为准。
Header header            # frame_id = world
uint64 generation        # 单调递增；producer 每次状态更新 +1
bool valid

float64 phi0             # 当前公共相位（rate-limited 转换后的值）
int32[3] slot_for_uav    # UAV -> slot 的真实置换（slot s 的名义方位 = phi0 + 2pi/3 * s）
float64[3] desired_radius    # 每机名义观察半径（target-centered）
float64[3] desired_bearing   # 每机名义方位（已经过 slot 映射，target-centered, wrap 到 [-pi,pi]）
float64[3] desired_height    # 每机名义观察高度

string reason
