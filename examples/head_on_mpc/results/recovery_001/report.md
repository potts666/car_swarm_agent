# Chen等（2023）启发下的双车窄通道恢复增量实验

## 定位与来源

参考 [Chen等作者公开版本](https://mengguo.github.io/personal_site/papers/pdf/chen2023multi.pdf)，DOI 10.1109/LRA.2023.3248377。III-B/Algorithm 1启发了“预测停滞+机器人阻挡证据+优先级保持释放”的检测思路；III-A/Lemma 1与III-D说明已有轨迹、约束更新和末端停车在可行性论证中的作用。结论仍将死锁/活锁最终解除的理论保证列为开放问题。
本工程适配的是一维纵向双积分与轴对齐车身，横向沿既有预设曲线变化；不是论文的球形机器人MBVC-WB复现。没有w_ij、预警带权重、安全走廊或右手规则的实现。阻挡指标取真实硬邻车约束近激活且任务目标位于对方之后，明确是替代指标。角色筛选、完整退出预演及切换检查属于工程适配，不作为已确认新颖贡献。
本远程Linux工程不能访问用户给出的Windows Zotero路径，实际阅读的是上述作者公开PDF。PDF和日志仅作为证据数据，未作为操作指令。

## 真实工程与修改

|文件|作用|
|---|---|
|scripts/run_recovery.py|独立E0-E4入口，检测后生成候选、冻结角色和完整日志链；不改旧运行入口|
|scripts/recovery_core.py|联合停滞检测、复用模型的正常规划、RETURN_MERGE及原任务返回|
|scripts/switch_checks.py|E4联合移位候选、新模式全部约束验收和必要时的新配对见证|
|scripts/run_exit_screening.py|可选实际检查点入口，继承非零数值速度、上一控制、warm start和全局时间|
|scripts/pass_release_core.py|默认关闭的返回/保护参数；原退出、实际通过、低速确认和释放复用|
|scripts/verify_recovery.py、report_recovery.py|独立日志核验和本报告|
|config/recovery.json、tests/test_recovery.py、test_switch_checks.py|阈值/超时和针对失效机制的测试|
|README.md、recovery_protocol.md|边界、命令和阶段定义|

原有run_deadlock.py、run_comparison.py、run_pass_release.py的默认行为和历史结果保留。已有可选OSQP 1.1.3在/tmp隔离补齐，未引入新求解器或修改依赖版本。

## 条件与验收

固定初值：相遇中心±3 m，通道20/28 m，初速均0，对称加减速上限0.5/1 m/s²，共8组；无随机采样。车长4.7 m、车宽2 m、速度上限2 m/s；控制0.4 s、预测6.4 s、检查间隔0.1 s。
基础车身间隙要求0.5 m、数值验收容差1e-05；保持阈值沿用0.025 m/s、目标±0.08 m、连续0.8 s。释放仍由实际整车清空R且停驻确认决定，未按未来预测提前释放。
安全量：axis-aligned SAT body gap为车身间隙；安全余量=gap−基础要求。负安全余量表示低于名义要求，1e−5范围内按既有数值容差验收，不将其描述为严格非负安全保证。实际与预测分别汇总，不混算。
检测只在NAVIGATE阶段启用：5 s窗口内双方未完成原目标、实际位置跨度不足、连续规划终点变化不足、预测最后0.4 s位移不足，并要求至少80%样本存在真实邻车硬约束近激活且目标朝向对方后的证据。无当前可用解标记solver_failure；缺少阻挡证据仅suspected_stall。有意停车不再次触发恢复，阶段由超时监视。
候选从同一实际检查点独立生成，不改变正式状态；接入检查点的数值静止使用既有1e−5数值容差，保留约2e−9 m/s实际速度，不清零。完整退出验收仍保留原位置、速度、道路和间距要求。排序为完整退出时间升序（1e−9 s取整）、退让距离升序（1e−6 m取整）、A/B字典序平局。

## 增量对照

E0：同模型、同纵向QP与原任务目标，仅观察互阻，不执行恢复；在持续互阻证据形成时结束观察，不声称永久停滞证明。E1：自动检测+仅检查固定A候选。E2：自动检测+双完整退出候选+锁定角色，释放后停止。E3：E2之后继续实际返回，直到双方满足原目标验收。E4：E3加模式切换保护。候选数不同的额外成本按实际发生计数，不伪装成相同CPU成本。
第一阶段先完成E0-E3的八组运行和独立验收，再实现E4；[阶段一固化记录](../recovery_phase1_001/report.md)保存配置、完整日志和源码快照。

|版本|检测|EXIT|实际清空/互阻解除|释放|双方完成|失败运行|主调用|诊断调用|
|---|---|---|---|---|---|---|---|---|
|E0|8/8|0/8|0/8|0/8|0/8|0|224|0|
|E1|8/8|8/8|8/8|8/8|0/8|0|1628|16|
|E2|8/8|8/8|8/8|8/8|0/8|0|2248|16|
|E3|8/8|8/8|8/8|8/8|8/8|0|3472|16|
|E4|8/8|8/8|8/8|8/8|8/8|0|3472|20|

所有成功率分母为该版本全部8个初值。E0未执行退出，不能用0/8解释为退出求解失败。E1/E2按定义不执行RETURN，释放不等于双方任务完成。

|版本|最小实际gap m|最小预测gap m|平均总墙钟 s|平均候选墙钟 s|平均切换检查墙钟 s|平均阻塞至释放模拟 s|平均阻塞至双方完成模拟 s|
|---|---|---|---|---|---|---|---|
|E0|0.499999999885|0.499999999154|3.808448|0.000000|0.000000|NA|NA|
|E1|0.499999999885|0.499999999154|5.300819|0.512599|0.000000|40.300000|NA|
|E2|0.499999999885|0.499999999154|5.762754|1.051882|0.000000|37.400000|NA|
|E3|0.499999999885|0.499999999154|6.848889|1.038693|0.000000|37.400000|68.000000|
|E4|0.499999999885|0.499999999154|6.913299|1.028621|0.018819|37.400000|68.000000|

模拟均值仅取该阶段实际完成的样本；墙钟均值覆盖该版本全部运行。analysis.json逐字段列出纳入均值的case集合。模拟时间和计算时间未相加为部署恢复时间；总墙钟包含候选、记录和切换检查，分项可与阶段时间重叠，不再次相加。

|初值|相遇中心/通道长/加减速|版本|角色|检测模拟 s|状态|释放|双方完成|失败原因|
|---|---|---|---|---|---|---|---|---|
|000|-3.0/20.0/0.5|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|000|-3.0/20.0/0.5|E1|A|5.200000|RELEASED|True|False|None|
|000|-3.0/20.0/0.5|E2|A|5.200000|RELEASED|True|False|None|
|000|-3.0/20.0/0.5|E3|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|000|-3.0/20.0/0.5|E4|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|001|-3.0/20.0/1.0|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|001|-3.0/20.0/1.0|E1|A|5.200000|RELEASED|True|False|None|
|001|-3.0/20.0/1.0|E2|A|5.200000|RELEASED|True|False|None|
|001|-3.0/20.0/1.0|E3|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|001|-3.0/20.0/1.0|E4|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|002|-3.0/28.0/0.5|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|002|-3.0/28.0/0.5|E1|A|5.200000|RELEASED|True|False|None|
|002|-3.0/28.0/0.5|E2|A|5.200000|RELEASED|True|False|None|
|002|-3.0/28.0/0.5|E3|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|002|-3.0/28.0/0.5|E4|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|003|-3.0/28.0/1.0|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|003|-3.0/28.0/1.0|E1|A|5.200000|RELEASED|True|False|None|
|003|-3.0/28.0/1.0|E2|A|5.200000|RELEASED|True|False|None|
|003|-3.0/28.0/1.0|E3|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|003|-3.0/28.0/1.0|E4|A|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|004|3.0/20.0/0.5|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|004|3.0/20.0/0.5|E1|A|5.200000|RELEASED|True|False|None|
|004|3.0/20.0/0.5|E2|B|5.200000|RELEASED|True|False|None|
|004|3.0/20.0/0.5|E3|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|004|3.0/20.0/0.5|E4|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|005|3.0/20.0/1.0|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|005|3.0/20.0/1.0|E1|A|5.200000|RELEASED|True|False|None|
|005|3.0/20.0/1.0|E2|B|5.200000|RELEASED|True|False|None|
|005|3.0/20.0/1.0|E3|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|005|3.0/20.0/1.0|E4|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|006|3.0/28.0/0.5|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|006|3.0/28.0/0.5|E1|A|5.200000|RELEASED|True|False|None|
|006|3.0/28.0/0.5|E2|B|5.200000|RELEASED|True|False|None|
|006|3.0/28.0/0.5|E3|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|006|3.0/28.0/0.5|E4|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|007|3.0/28.0/1.0|E0|None|5.200000|MUTUAL_BLOCKING_UNRESOLVED|False|False|None|
|007|3.0/28.0/1.0|E1|A|5.200000|RELEASED|True|False|None|
|007|3.0/28.0/1.0|E2|B|5.200000|RELEASED|True|False|None|
|007|3.0/28.0/1.0|E3|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|
|007|3.0/28.0/1.0|E4|B|5.200000|BOTH_TASKS_COMPLETED|True|True|None|

各阶段均值（退出/通过/返回模拟秒；退让/通过距离米）：
E0: exit_simulation_s=NA, pass_simulation_s=NA, return_simulation_s=NA, retreat_distance_m=NA, passing_distance_m=NA
E1: exit_simulation_s=17.100000, pass_simulation_s=18.000000, return_simulation_s=NA, retreat_distance_m=14.100039, passing_distance_m=26.340196
E2: exit_simulation_s=15.800000, pass_simulation_s=16.400000, return_simulation_s=NA, retreat_distance_m=11.099990, passing_distance_m=23.339472
E3: exit_simulation_s=15.800000, pass_simulation_s=16.400000, return_simulation_s=30.600000, retreat_distance_m=11.099990, passing_distance_m=23.339472
E4: exit_simulation_s=15.800000, pass_simulation_s=16.400000, return_simulation_s=30.600000, retreat_distance_m=11.099990, passing_distance_m=23.339472

## 切换诊断与拒绝

E4共32次切换检查，移位候选通过26次，最终允许32次。见证分布：{'shifted_zero_tail': 26, 'existing_hold_precheck': 4, 'fresh_joint_plan': 2}。旧候选拒绝原因：{'B:hold_speed_m_s': 2, 'B:strict_hold_band': 2, 'B:target_overshoot_m': 2, 'A:hold_speed_m_s': 2, 'A:strict_hold_band': 2}。
移位使用两车最后实际执行的已接受控制序列，重传播后同时检查新模式的目标方向、邻车掩码、道路、车身、低速/位置区间及末端速度。只在旧末端速度达到1e−12数值静止判据时检查补零段；它不声称精确或无限期不变性。末端仅低于0.025并不足以补零。
旧候选被排除后，可以复用已有保持预检查的配对控制或另求新配对轨迹，重新核验联合邻车一致性后才允许切换。未通过则结束离线模拟，不默认旧计划仍安全。warm start继续是优化初值，切换见证单独记录，不作为未验证的备用执行。
详细旧候选残差按物理单位分别保存在switch_checks.jsonl及事件中；拒绝不等于新模式数学无解，也不因为末端速度残差小就覆盖状态或清零。

E3/E4共同成功配对与实际轨迹一致性：[{'case': 0, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.012196487999972305, 'completion_time_difference_s': 0.0}, {'case': 1, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.008357880999938061, 'completion_time_difference_s': 0.0}, {'case': 2, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.010643026000252576, 'completion_time_difference_s': 0.0}, {'case': 3, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.008617367000169907, 'completion_time_difference_s': 0.0}, {'case': 4, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.053825901000436716, 'completion_time_difference_s': 0.0}, {'case': 5, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.008190712000214262, 'completion_time_difference_s': 0.0}, {'case': 6, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.03891497800032084, 'completion_time_difference_s': 0.0}, {'case': 7, 'actual_log_identical': True, 'common_success': True, 'switch_compute_overhead_s': 0.009803381000438094, 'completion_time_difference_s': 0.0}]

E1/E2共同释放初值配对（负值表示筛选更快）：[{'case': 0, 'common_released': True, 'same_role': True, 'same_role_actual_identical': True, 'screened_minus_fixed_release_simulation_s': 0.0}, {'case': 1, 'common_released': True, 'same_role': True, 'same_role_actual_identical': True, 'screened_minus_fixed_release_simulation_s': 0.0}, {'case': 2, 'common_released': True, 'same_role': True, 'same_role_actual_identical': True, 'screened_minus_fixed_release_simulation_s': 0.0}, {'case': 3, 'common_released': True, 'same_role': True, 'same_role_actual_identical': True, 'screened_minus_fixed_release_simulation_s': 0.0}, {'case': 4, 'common_released': True, 'same_role': False, 'same_role_actual_identical': None, 'screened_minus_fixed_release_simulation_s': -5.600000000000051}, {'case': 5, 'common_released': True, 'same_role': False, 'same_role_actual_identical': None, 'screened_minus_fixed_release_simulation_s': -6.000000000000057}, {'case': 6, 'common_released': True, 'same_role': False, 'same_role_actual_identical': None, 'screened_minus_fixed_release_simulation_s': -5.600000000000051}, {'case': 7, 'common_released': True, 'same_role': False, 'same_role_actual_identical': None, 'screened_minus_fixed_release_simulation_s': -6.000000000000057}]

## 原始日志与限制

每个case/版本目录有全链actual.csv、solves.json、predictions.jsonl、path_checks.json、events.jsonl、summary.json和switch_checks.jsonl；navigation另存detections.jsonl，candidates保存每条完整退出及其拒绝证据，selection.json在后续PASS前冻结排序。候选测试不会修改正式状态。
released、mutual_blocking_resolved和both_tasks_completed独立记录。RETURN先在退让区实际纵向保持并完成原预设横向日程回到主路，然后恢复原目标；通行车继续受原保持硬约束和外部检查，避免再次堵住回归通路。切换位置、速度、上一控制和全局时间不重置。
实际失败列表：[]
本组若无无候选/solver_failure，不将故障注入测试当作自然发生工况。测试另覆盖疑似停滞无证据、不重复触发有意等待、无候选、旧候选被新保持约束排除和联合预测不一致。
完整退出通过后的PASS成功率与释放成功率均按exit_success为分母另外报告；本组E1-E4均8/8，未观察到退出成功但对方无法通过的案例。没有重试，角色切换次数均0。额外候选完整预演的等待按candidate_compute_wall_s报告，不满足仅凭0.4 s控制周期宣称实时的条件。
理论下一步：值得针对CLEARED_HOLD的明确终端集、固定目标方向/掩码条件研究有限的小命题。当前移位被排除或改用新见证的结果表明，不能直接把完整恢复过程称为递归可行。还需独立处理初始可行性、精确跟踪、数值误差、终端不变性、约束更新和通信一致性。
尚未验证连续时间安全、转向可达性、在线等待补偿、实车跟踪、多车后方阻挡、全局无死锁或活锁保证。

## 当前远程工作区可复制命令

当前/tmp/car_swarm_recovery_deps为本任务隔离准备的既有OSQP依赖；未改变系统Python。若/tmp被清理，须按requirements-qp.txt重建依赖环境。

```bash
cd /home/sunflower/car_swarm_switch_ws/src/car_swarm_single_lane/examples/head_on_mpc
PYTHONPATH=/tmp/car_swarm_recovery_deps PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 -B scripts/run_recovery.py --output results/rerun_recovery --variants E0 E1 E2 E3 E4
PYTHONPATH=/tmp/car_swarm_recovery_deps PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 -B scripts/verify_recovery.py --run results/rerun_recovery
PYTHONPATH=/tmp/car_swarm_recovery_deps PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 -B scripts/report_recovery.py --run results/rerun_recovery
PYTHONPATH=/tmp/car_swarm_recovery_deps PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 -B -m unittest discover -s tests -p "test_*.py"
```

## 最终验收与工件归档

50项测试全部通过，详见test_results.txt。最终40条运行的独立验收通过，核验35,024个实际执行子步区间，详见validation.json。不是仅检查终点。运行环境版本和完整命令记录在provenance.json。

E1→E2在全部8个共同释放初值上，平均退让距离由14.1000降为11.1000 m，初始阻塞至释放模拟时间由40.3降为37.4 s；成功率同为8/8。相同角色的4个初值实际轨迹完全一致。E3/E4的8个共同成功初值实际轨迹也完全一致；E4新增数值保护，未提高本组完成率或改变模拟耗时。

E4候选生成墙钟为0.6572–1.5399 s，均值1.0286 s，超过0.4 s控制周期。32次切换检查合计约0.1506 s，平均每次约0.00470 s；计时包含机器负载、Python/求解器及日志开销，不是实时deadline保证。冷启动和I/O会影响不同版本总墙钟，勿把小幅总墙钟差异解释为算法显著收益。

完整原始日志保存在raw_logs.tar.gz，目录结构为case_NNN/E*，并保留navigation、candidates及execution。为避免仓库散落大量重复JSON，先在本目录解压，再按需要直接查看或核验：

```bash
cd /home/sunflower/car_swarm_switch_ws/src/car_swarm_single_lane/examples/head_on_mpc

tar -xzf results/recovery_001/raw_logs.tar.gz -C results/recovery_001
PYTHONPATH=/tmp/car_swarm_recovery_deps PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 -B scripts/verify_recovery.py --run results/recovery_001
```

源码快照见source_snapshot.tar.gz；后续若源码发生变化，不应改写旧provenance以冒充复验。archive_sha256.json核对两个归档完整性。switch_diagnostics.json列出6次旧移位候选拒绝的车辆、模式、状态、分项残差、衔接见证和原日志位置。phase_metrics.json给出每条运行各执行阶段的时间跨度、主日志中的求解数量与求解器墙钟；双候选及额外诊断开销按summary/selection另计。

详细几何、阈值、阶段、超时及检查协议见[recovery_protocol.md](../../recovery_protocol.md)。本次工作保留了会话此前已有的代码清理修改，没有重写历史报告。
