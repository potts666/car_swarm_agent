# M1 固定运动实现：选择B2-C作为M2集中式参考

**M1完成。选定B2-C/M1-v1，B1保留历史完成诊断。** 本轮未改变运动、求解或验收代码，未新增闭环实验/积分/回归。按任务书WP2复用M0_closure_001的repaired_r1/r2两轮已完整独立验收结果；新增的是来源/哈希/原始字段审计、冻结副本、入口打包小检查和M2依赖交接。

## 选择依据与证据身份

1. 两轮到达原目标，原完整独立验收、ROS授权/清空/释放、未来见证与实际执行一致性均通过。
2. 17个运行入口的本地依赖与关闭版source逐字节相同，完整scene及已声明Python/NumPy/SciPy/OSQP/Linux/CPU/线程匹配；原始实际输入与最终候选及提交样本逐一核对，另核查796次原始变量热启动确实来自上次提交参考并按当前界处理。分析用冻结/局部重演脚本的后续变化不在运行依赖内。
3. 可明确区分单车状态/动力学/输入和当前联合参考、耦合约束、分离轴及全车未来预演，足以交给M2作架构设计。
4. 选择基于任务、验收和可追溯性；没有证明它比B1普遍更优，也不要求此阶段消除长尾。

source/config/acceptance版本完整哈希见selection.json；运行来源、维护路径及原始日志指纹见source_manifest.json。真实维护目录是examples/head_on_mpc/scripts，本目录source是可复现证据，不是后续长期开发目录。当前工作区有未提交改动，因此不以git HEAD单独定义版本。

任务书中的D:/dsh-test和T1别名不是此环境实际路径；已通过provenance/哈希确认对应Linux仓库及B1&B2-T1。原M0_001到达目标但三个跨零失败、T1机制结果和closure人工语义反例均保留，不重新开启该研究问题。

## 验收分别表述

- artifact_integrity_passed：本轮检查来源/源码/原始文件哈希、配置、日志计数/控制一致性，通过。
- both_goals_reached、original_full_acceptance_passed、protocol_passed、timing_complete：所选两轮均通过，独立积分及几何/权限验收复用closure记录，本轮没有重新积分。
- observed_deadline_misses：每轮2个完整周期超过200 ms，0个0.5 s预测预算拒绝。不能将其合成一个无条件passed。
- 包装小检查：冻结入口--help、模块解析、ROOT资源布局、AST与配置一致性通过；未调用predict、未启动车辆积分或新仿真。
- 34项原回归的模块/版本及历史source/git位置均已核对，只保存清单，不复制测试正文；本轮未复制执行一整套旧回归。人工移动WAIT案例只说明分量方程检查不负责模式合法性，阶段条件由Agent.advance测试，不能互相替代。

原独立积分是标量方程与两半步RK4，覆盖逐区间及从原初态累计重演；几何审计继续复用原函数，不宣称完全独立的几何验证。旧summary.controller='B1'为公共入口遗留标签，源码、构造器和config才确定本候选。

## 复用的完整实现代价

|复用运行|任务s|统一控制代价|最小净空m|主修复次数|主参考/自行车QP|未来参考/自行车QP|计时count|P50/P95/P99/最大ms|>200ms|
|---|---|---|---|---|---|---|---|---|---|
|repaired_r1|80.4|19.93486090754438|1.199922161|6|399/411|272/278|403|56.86/95.72/130.73/12393.03|2|
|repaired_r2|80.4|19.93486090754438|1.199922161|6|399/411|272/278|403|56.06/93.01/128.73/12808.35|2|

每轮402次提交加1次最终完成检查，全部403个周期计时；主执行12个周期需要第二次更新，其余390个周期一次。主参考/自行车QP分别399/411，未来分别272/278；实际数值QP与所有输入固定的直接检查分开统计。未来预演的更新分布、修复次数和拒绝另存metrics.json，不与主执行计数混合。参考拒绝、自行车数值拒绝与预测超时均为0，历史失败对照仍见comparison.csv，不将其到达目标写成通过验收。

统一控制代价为Σ0.02·[(a/1 m/s²)²+(omega/1 rad/s)²]，由原actual.csv重新汇总核对；不比较不同控制器省略常数后的目标。80.4 s是此场景观测，不是未来每次必须逐字达到的承诺。两轮实际轨迹逐字节相同只支持单场景确定性检查，不估计场景成功率或长尾分布。

端点误差：

|运行|车|位置m|航向rad|速度m/s|转角rad|
|---|---|---|---|---|---|
|repaired_r1|A|1.21191199e-07|0|2.60208521e-18|0|
|repaired_r1|B|6.00186403e-05|1.77635684e-15|2.60208521e-18|5.73016097e-16|
|repaired_r2|A|1.21191199e-07|0|2.60208521e-18|0|
|repaired_r2|B|6.00186403e-05|1.77635684e-15|2.60208521e-18|5.73016097e-16|

阶段等待与停后再动：按已有abs(v)<=0.025 m/s口径统计低速区间/状态转换，不把低速统计自动称为严格静止或死锁。

|运行|车|申请到授权s|低速到运动转换数|分阶段低速占用s|
|---|---|---|---|---|
|repaired_r1|A|0.0|1|{"COMPLETE":36.0,"NAVIGATE":0.28}|
|repaired_r1|B|27.60000000000001|6|{"DEPART":0.84,"ENTRY":2.16,"MUTEX_WAIT":10.6,"QUEUE_APPROACH":0.48,"RETURN":2.16}|
|repaired_r2|A|0.0|1|{"COMPLETE":36.0,"NAVIGATE":0.28}|
|repaired_r2|B|27.60000000000001|6|{"DEPART":0.84,"ENTRY":2.16,"MUTEX_WAIT":10.6,"QUEUE_APPROACH":0.48,"RETURN":2.16}|

每车任务完成时间、授权/释放时刻、完整阶段细节、原QP状态直方图、模型误差、6次修复的原/新输入见metrics.json及原日志；A完成约44.4 s、B约80.4 s。本M1没有改通道优先级或模式反馈，也没有验证失败时在线备用控制。

## 计时分段的实际可观测范围

下表是各主周期累计秒数。**多行是包含关系，不能相加为总时间**：decision字段包含future，prediction包含reference、B2和内部重演；subproblem的solver_wall_s包含OSQP setup+warm-start+solve，参考solver_wall_s仅solve。

|已有字段/组合项（s）|repaired_r1|repaired_r2|
|---|---|---|
|decision_communication_and_prepare_including_future_s|15.142300|15.256719|
|future_witness_subset_s|15.128295|15.235735|
|reference_build_setup_solve_check_s|3.569774|3.486986|
|reference_OSQP_solve_only_s|2.418080|2.347488|
|reference_OSQP_setup_s|0.305438|0.300630|
|bicycle_subproblem_OSQP_setup_and_solve_subset_s|2.870143|2.812914|
|prediction_including_reference_bicycle_and_internal_replay_s|22.576154|22.082659|
|outer_acceptance_buffered_record_and_commit_s|2.255721|2.238909|
|total_cycle_s|39.974175|39.578288|
|total_policy_including_bulk_export_s|40.630541|40.239429|

无法从旧记录精确拆开自行车构造、参考稠密重演、内部非线性检查、记录和提交的各自耗时；保留组合项，不补造数字。外层acceptance包含稠密预测审计、权限、实际段检查、缓冲记录和提交；最终文件批量导出/ROS发现不属于每周期控制器计时，policy总墙钟另列。

P99约130 ms不能掩盖约12–13 s最大周期。两个超周期来自保留的完整未来预演所在主周期；0.5 s是预测预算而非端到端预算，且本身大于0.2 s控制周期。没有实时保证。

## B1的保留方式

原bicycle_e3_002/B1的轨迹、控制、求解、事件、完整验收及source/provenance存在且来源哈希核对通过；81.2 s完成并通过其既定验收。B1仍是固定路径联合纵向QP，未被额外加上B2热启动或停车输入修复，也不是未经验证的在线失败回退。

本轮只作历史诊断引用，不新增B1性能实验。其旧周期计时漏最终完成出口、CPU/线程证据未完整记录，与当前全出口计时不能作公平性能排序；comparison.csv中的B1行明确标记historical，未计算提速倍数。若以后需要性能比较，须同机/同线程/同地图/同端到端口径成对两轮，且说明可行域、目标和运动生成方式不同，不能称为同一个数学问题的求解器对比。

## 交付与停止点

- baseline_spec.md、frozen_config.json：模型、全部数值/物理要求、输入修复、缓存、原验收与预算。
- source/、config/、source_manifest.json：冻结源码、配置、34项回归版本、维护归属及可靠原始文件指向。
- verification.json、selection.json：分列材料/任务/协议/计时与选型；new_experiments_executed=false。
- provenance.json、repeatability.json、comparison.csv、metrics.json：声明环境、原命令、重复性、历史诊断与所有代价。
- dependency_map.md、M2_handoff.md：现有本地/邻车/通道/联合依赖、局部化缺口及待决项。
- reproduction.md：同一Linux环境准备、临时依赖恢复、冻结现有入口的准确复现/独立重读命令。不存在runs/新增日志，因为没有新闭环。

固定后停止运动调参与求解器探索。M2从双车局部信息、预测/承诺交换与协调设计开始：参考和自行车层仍联合求解、分离轴读双方阶段、释放仍复制双方未来，不是切分变量向量就完成DMPC。没有预定交换轮次/算法，也未实现四/六车、Q1/Q2、延迟、备用或实车部署。

未证明普遍最优、任意状态停车修复有效、连续时间安全、递归可行、局部DMPC或硬实时；后续新问题另开版本，不回写本冻结版。
