# E2 固定路径纵向联合 QP：B0/B1



|指标|B0 原控制|B1 联合 QP|
|---|---|---|
|completed|True|True|
|validation_passed|True|False|
|failure|None|None|
|completion_sim_s|79|80.8|
|minimum_sampled_gap_m|1.2|1.20000001|
|distance_m|175.471107|175.471107|
|reverse_distance_m|9.00252095e-15|3.59072259e-09|
|QP_calls|0|404|
|future_diagnostic_QP_calls|0|277|
|cycle_mean_ms|81.6810831|38.5218532|
|cycle_max_ms|262.815519|4319.72045|
|cycle_over_200ms|18|2|
|solve_mean_ms|None|6.1344141|
|solve_max_ms|0|104.725868|
|min_hessian_eigenvalue|None|0.243980915|
|maximum_affine_violation|None|9.752896e-08|
|maximum_mapping_position_m|None|5.97134977e-08|
|maximum_mapping_speed_m_s|None|1.96266699e-08|
|maximum_mapping_heading_rad|None|9.51051682e-10|
|solve_rejections|0|0|



同一 entry_bicycle.json、初态、地图、车辆能力与目标；Flag 均为 A→B，实际 ROS 2 三节点消息。B0 每次重新运行，不读取历史轨迹作为成功依据。

B1 的几何路径为原直线/双圆弧；B0 申请后的制动及接近合并为同一直线上的纵向优化。B1 在曲率断点停车转向，固定 N=32、h=0.2 s，不优化几何、顺序或时域。

联合 QP 在 B 初始直线阶段包含双方进度的耦合间距行；圆弧阶段采用整段车身扫掠的保守固定 x 分离界；B 停靠后采用已检验的横向净空。模型定义见 mathematics.md。

速度是沿固定段的非负路径速度，映射自行车有符号 v=d*v_path、a=d*a_path；运动时 delta 固定、omega=0，段间在静止时按原 steer() 限速转向。没有直接把路径坐标作为车辆位置写回。

全部控制通过原 RK4 非线性自行车积分，以 0.02 s 采样重放分量方程、旋转车身、道路、双方间距、原目标、未授权进入及完整未来释放见证。

释放前的完整未来见证再次以同一 QP 和自行车模型滚动至双方目标，诊断成本单独报告；该计算纳入所在执行周期的总墙钟，不能只用 QP 单次耗时声称实时。

本轮为单场景、理想通信、仿真冻结计算的集中式 B1。道路扫掠边界是 0.01 m 几何采样，执行是 0.02 s 采样；不构成连续时间安全、递归可行性或真实车辆可行证明。

源码/配置哈希、版本、环境与命令见 provenance.json/source/；逐次失败原样保留在 solves.jsonl、events.jsonl。
