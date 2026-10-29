# E3：时变仿射自行车联合 QP 对照



|指标|B1|B2 单次|B2 最多三次更新|
|---|---|---|---|
|completed|True|False|False|
|unified_validation_passed|True|False|False|
|executed_prefix_validation_passed|True|True|True|
|ROS_validation_passed|True|False|False|
|failure|None|joint_QP_rejected|joint_QP_rejected|
|completion_sim_s|81.2|None|None|
|cycle_acceptance_fraction|1|0.98245614|0.991803279|
|maximum_position_model_error_m|1.44511958e-10|1.31174841e-05|8.92959239e-06|
|maximum_heading_model_error_rad|3.81028542e-13|3.74135966e-06|3.63398356e-06|
|maximum_speed_model_error_m_s|3.97928218e-14|2.28705943e-14|2.28705943e-14|
|minimum_sampled_gap_m|1.2|42.6033038|8.74989085|
|normalized_execution_effort|20.6969246|6.02836952|9.76426041|
|main_reference_numeric_QPs|401|57|122|
|main_bicycle_numeric_QPs|0|57|127|
|future_reference_numeric_QPs|274|0|0|
|future_bicycle_numeric_QPs|0|0|0|
|sequential_update_cycles|0|0|6|
|reference_rejections|0|0|1|
|rejected_subproblems|0|0|0|
|deadline_rejections|0|0|0|
|hessian_min_eigenvalue|0.243980915|0.222153875|0.222153875|
|maximum_affine_violation|9.75289298e-08|2.44290623e-07|2.44290623e-07|
|prediction_planning_mean_ms|34.4175905|74.6719179|70.3793983|
|cycle_mean_ms|40.3688183|80.9160629|76.175007|
|cycle_max_ms|4737.77013|127.821614|168.307474|
|cycles_over_200ms|2|0|0|
|future_check_wall_s|5.68569484|0|0|



同一原场景、地图、目标、车辆能力、固定路径阶段和 A→B ROS Flag 顺序。B1 重新运行；B2_single 与 B2_sequence 仅在线性化更新次数上不同，误差阈值和走廊/信赖域相同。

B2 先调用一次 B1 纵向参考 QP，再求解保留 x/y/ψ/v/δ 的 128 输入变量联合 QP。参考生成的 QP 数量和耗时计入全部主周期与未来诊断成本，不能把 B2 称为只需一次总 QP。

每轮固定全部凸道路走廊、保护区分离半空间、双方分离轴及初始参考的航向车身包络。序列更新只重建仿射动态与局部信赖域，不在子问题内优化走廊选择。

单次与序列采用相同非线性误差阈值：位置 1e-5 m、航向 5e-6 rad、速度及转角 1e-6。阶段衔接的原停车/转向和完整未来释放检查保留。

prediction_planning_mean_ms 来自共用入口，含阶段/ROS处理、完整未来释放预演、参考生成、走廊、所有更新及预测重演。cycle_max_ms 还包含随后执行段验收与提交；不能用子问题求解耗时声称在线实时。

normalized_execution_effort=Σ sample_s·[(a/1 m/s²)²+(ω/1 rad/s)²]，各方法同口径。内部优化目标不同，不直接比较内部 objective_without_constant。失败分支的控制成本和最小间距只覆盖已执行前缀，不能当作完成任务的收益。

每周期 QP 数、每次更新状态/残差/模型误差见各分支 solves.jsonl；线性预测、固定走廊、信赖域与实际控制均保存。单场景的周期验收比例不是场景成功概率。

单个子问题是凸 QP；序列求解不表示原非线性问题全局凸或全局最优。统一验收为原非线性方程、旋转车身、道路/双方间距/权限、原目标和完整未来见证的 0.02 s 采样重放。

本轮依据完整任务验收后最小实测最长完整周期选择后续内层方法：B1；其余保留对照。选择限于当前场景，不是一般性能排序。

unified_validation_passed 同时要求完整任务和 ROS 正常交接；executed_prefix_validation_passed 只描述已执行前缀的原物理验收。中止模拟不等于已验证硬件安全停车。

数学推导、有效域和固定参数见 mathematics.md；版本、准确命令和源码快照见 provenance.json/source/。

失败定位：单次 B2 第56周期（11.2 s）弯道候选位置误差1.31175e-5 m超过1e-5 m；序列 B2 在6个周期使用第二次更新恢复候选，最终第121周期（24.2 s）参考QP达到20000次迭代上限。参考层该次 primal residual=2.616e-8、dual residual=1.195e-7，未把未收敛状态改报成功，不能据此断言物理不可行。见 failure_analysis.json。

B2尚未运行到B授权和完整通行，预定顺序仍为A→B；两个失败前缀都通过原物理重放验收。其成本及较小最长周期不代表完成任务的性能。B1重新运行的actual.csv与E2逐字节一致。

首轮结果 ../bicycle_e3_001/ 使用前一版本参数，不与本轮比例混合。开发自检见 exploratory/，不作为正式对照样本。
