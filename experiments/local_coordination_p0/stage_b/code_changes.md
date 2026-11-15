# B新增代码与边界

原A/P0源文件不修改。B代码及其引用依赖使用独立目录。

- common.py：冻结依赖布局、实际锚点身份、状态序列化、内容寻址对象和数值候选/K装配。
- derivatives.py：实际逐拍变分区间、前缀矩阵、参考缓存范围诊断、实际锚点局部G/H和共同几何守卫。
- socp.py：三组相同自由终端锥问题、Clarabel原始解、固定有理化回退、精确行验收及A方向比例诊断。
- execute_case.py：同锚点分组运行、原子共同提交、实际执行/K/尾策略、JL移位再认证及原停车换权。
- validate_numeric.py：控制器外推导终端行参数，核对梯度误差/Hessian来源、信赖中心及原始数值解到实际输入的来源。
- accept_case.py：复用独立几何Replay，按实际多次提案日志检查状态、K、移位、预算和提交，不使用A“恰好一次构造候选”断言。
- prepare_anchors.py、test_interfaces.py、test_rejections.py及checks/single_step_exact.py：B0、精确校准、覆盖与损坏测试。
- run_b.py：有界顺序执行、身份检查、日志、退出码/失败层次，失败停止；保留普通候选拒绝的续行。
- analysis/及CSV/JSON：只读后处理与证据汇总，不生成候选。

数据、导数、原始求解及最终输入由objects中的内容ID引用，同一导数跨组共享一次。sources中的原地图生成器未改写，连同区间模型和实际引用的协议源随包提供。

B_GATE不是全部通过：JL新输入执行和“接受JL后再认证”仍未达标。数值误差不得用标记继承成功来掩盖。
