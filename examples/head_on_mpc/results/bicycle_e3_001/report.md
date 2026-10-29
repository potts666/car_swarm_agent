# E3：时变仿射自行车联合 QP 对照



|指标|B1|B2 单次|B2 最多三次更新|
|---|---|---|---|
|completed|True|False|False|
|unified_validation_passed|True|True|True|
|ROS_validation_passed|True|False|False|
|failure|None|joint_QP_rejected|joint_QP_rejected|
|completion_sim_s|81.2|None|None|
|cycle_acceptance_fraction|1|0.988372093|0.988372093|
|maximum_position_model_error_m|1.44511958e-10|2.67245005e-05|2.67245005e-05|
|maximum_heading_model_error_rad|3.81028542e-13|7.35758547e-06|7.35758547e-06|
|maximum_speed_model_error_m_s|3.97928218e-14|2.24265051e-14|2.24265051e-14|
|minimum_sampled_gap_m|1.2|27.4066403|27.4066403|
|normalized_execution_effort|20.6969246|7.5402372|7.5402372|
|main_reference_numeric_QPs|401|86|86|
|main_bicycle_numeric_QPs|0|85|85|
|future_reference_numeric_QPs|274|0|0|
|future_bicycle_numeric_QPs|0|0|0|
|sequential_update_cycles|0|0|0|
|deadline_rejections|0|0|0|
|hessian_min_eigenvalue|0.243980915|0.222153875|0.222153875|
|maximum_affine_violation|9.75289298e-08|2.44280751e-07|2.44280751e-07|
|prediction_planning_mean_ms|6.30826947|67.1996851|66.3280338|
|cycle_mean_ms|40.4196608|73.9362802|73.0610703|
|cycle_max_ms|4817.22229|153.925975|161.745511|
|cycles_over_200ms|2|0|0|
|future_check_wall_s|5.72507707|0|0|



同一原场景、地图、目标、车辆能力、固定路径阶段和 A→B ROS Flag 顺序。B1 重新运行；B2_single 与 B2_sequence 仅在线性化更新次数上不同，误差阈值和走廊/信赖域相同。

B2 先调用一次 B1 纵向参考 QP，再求解保留 x/y/ψ/v/δ 的 128 输入变量联合 QP。参考生成的 QP 数量和耗时计入全部主周期与未来诊断成本，不能把 B2 称为只需一次总 QP。

每轮固定全部凸道路走廊、保护区分离半空间、双方分离轴及初始参考的航向车身包络。序列更新只重建仿射动态与局部信赖域，不在子问题内优化走廊选择。

单次与序列采用相同非线性误差阈值：位置 1e-4 m、航向 5e-5 rad、速度及转角 1e-6。阶段衔接的原停车/转向和完整未来释放检查保留。

prediction_planning_mean_ms 含参考生成、走廊、所有 B2 更新及非线性重演。cycle_max_ms 还包含完整未来释放预演；不能用子问题求解耗时声称在线实时。

normalized_execution_effort=Σ sample_s·[(a/1 m/s²)²+(ω/1 rad/s)²]，各方法同口径。内部优化目标不同，不直接比较内部 objective_without_constant。失败分支的控制成本和最小间距只覆盖已执行前缀，不能当作完成任务的收益。

每周期 QP 数、每次更新状态/残差/模型误差见各分支 solves.jsonl；线性预测、固定走廊、信赖域与实际控制均保存。单场景的周期验收比例不是场景成功概率。

单个子问题是凸 QP；序列求解不表示原非线性问题全局凸或全局最优。统一验收为原非线性方程、旋转车身、道路/双方间距/权限、原目标和完整未来见证的 0.02 s 采样重放。

本轮依据完整任务验收后最小实测最长完整周期选择后续内层方法：B1；其余保留对照。选择限于当前场景，不是一般性能排序。

数学推导、有效域和固定参数见 mathematics.md；版本、准确命令和源码快照见 provenance.json/source/。
