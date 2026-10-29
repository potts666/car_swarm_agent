# E2 固定路径纵向联合 QP：B0/B1



|指标|B0 原控制|B1 联合 QP|
|---|---|---|
|completed|True|True|
|validation_passed|True|True|
|failure|None|None|
|completion_sim_s|79|81.2|
|minimum_sampled_gap_m|1.2|1.2|
|distance_m|175.471107|175.471107|
|reverse_distance_m|9.00252095e-15|5.98475683e-10|
|QP_build_calls|0|406|
|QP_calls|0|401|
|future_diagnostic_QP_calls|0|274|
|cycle_mean_ms|79.8528786|39.3574069|
|cycle_max_ms|254.163847|4602.45122|
|cycle_over_200ms|14|2|
|solve_mean_ms|None|6.23705752|
|solve_max_ms|0|110.463911|
|min_hessian_eigenvalue|None|0.243980915|
|maximum_affine_violation|None|9.75289298e-08|
|maximum_mapping_position_m|None|1.44511958e-10|
|maximum_mapping_speed_m_s|None|3.97928218e-14|
|maximum_mapping_heading_rad|None|3.81028542e-13|
|solve_rejections|0|0|



同一 entry_bicycle.json、初态、地图、车辆能力与目标；Flag 均为 A→B，实际 ROS 2 三节点消息。B0 每次重新运行，不读取历史轨迹作为成功依据。

B1 的几何路径为原直线/双圆弧；B0 申请后的制动及接近合并为同一直线上的纵向优化。B1 在曲率断点停车转向，固定 N=32、h=0.2 s，不优化几何、顺序或时域。

联合 QP 在 B 初始直线阶段包含双方进度的耦合间距行；圆弧阶段采用整段车身扫掠的保守固定 x 分离界；B 停靠后采用已检验的横向净空。模型定义见 mathematics.md。

速度是沿固定段的非负路径速度，映射自行车有符号 v=d*v_path、a=d*a_path；运动时 delta 固定、omega=0，段间在静止时按原 steer() 限速转向。没有直接把路径坐标作为车辆位置写回。

全部控制通过原 RK4 非线性自行车积分，以 0.02 s 采样重放分量方程、旋转车身、道路、双方间距、原目标、未授权进入及完整未来释放见证。

释放前的完整未来见证再次以同一 QP 和自行车模型滚动至双方目标，诊断成本单独报告；该计算纳入所在执行周期的总墙钟，不能只用 QP 单次耗时声称实时。

本轮为单场景、理想通信、仿真冻结计算的集中式 B1。道路扫掠边界是 0.01 m 几何采样，执行是 0.02 s 采样；不构成连续时间安全、递归可行性或真实车辆可行证明。

QP_calls 统计有自由输入时的实际 OSQP 数值调用；QP_build_calls 还包括所有输入已固定、直接求值的阶段。solve_mean_ms 是每轮 QP 构造、求解及残差检查墙钟均值，不含后续自行车全预测和映射检查。

源码/配置哈希、版本、环境与命令见 provenance.json/source/；逐次失败原样保留在 solves.jsonl、events.jsonl。

首轮 [joint_e2_001](../joint_e2_001/report.md) 虽完成任务但未通过统一验收：B 的约 1e-8 m/s 负残速在下一段起步时触发原速度符号跳变检查。当前轮增加实际制动周期消除残速，未放宽统一验收容差、未覆盖实际状态。首轮原始数据和源码快照保持不变。

许可接收不等于立即起步：B 第 174 周期收到 g=1，先静止转向，至第 177 周期开始返回移动段；等待时进度上界 0.0 m，移动段开放上界 4.336405486880494 m、速度界 1.0 m/s。完整参数见 permission_transition.json。

A、B 两次释放见证与随后实际执行的样本数均一致，五维状态最大误差为 0，详见 future_execution_consistency.json。统一独立分量重放检查同时通过。新增 5 项测试、原自行车 11 项及 E1 接口 3 项测试通过。
