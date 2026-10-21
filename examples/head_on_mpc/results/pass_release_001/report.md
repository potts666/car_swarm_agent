# 退出停驻 → 对方实际通过 → 释放通行权

完整离线闭环模拟预演。本次只验证一辆退让、另一辆通过、释放指定冲突区域的占用权；不启动退让车回归，不标记双方任务完成。

## 几何、目标与释放

R为原单通道主路矩形：[channel.xmin,channel.xmax] × [road.ymin,road.ymax]。通道长20/28 m时R纵向为[-10,10]/[-14,14] m，横向[-1.4,1.4] m。使用原道路、原退让区和原goals_x，不移动边界。
A朝+x通过、B朝−x通过。停驻目标分别为±21.75 m或±25.75 m，仍须全车身容纳。释放裕度明确为0.5 m；A要求x−2.35≥R.xmax+0.5，B要求x+2.35≤R.xmin−0.5。
释放还要求实际到目标±0.08 m内、速度≤0.025 m/s，并连续保持0.8 s；退让车保持实际停驻参考，配对实际与最新全预测通过，预测未来也不会重新进入R，通行权持有者匹配。

## 逐初值、逐基线

|初值|相遇中心 m|通道长 m|加减速 m/s²|策略|退让角色|EXIT|PASS|RELEASED|失败|
|---|---|---|---|---|---|---|---|---|---|
|000|-3.0|20.0|0.5|fixed_A|A|True|False|False|cleared_hold_actual_condition_lost|
|000|-3.0|20.0|0.5|screened|A|True|False|False|cleared_hold_actual_condition_lost|
|001|-3.0|20.0|1.0|fixed_A|A|True|True|True|None|
|001|-3.0|20.0|1.0|screened|A|True|True|True|None|
|002|-3.0|28.0|0.5|fixed_A|A|True|False|False|cleared_hold_actual_condition_lost|
|002|-3.0|28.0|0.5|screened|A|True|False|False|cleared_hold_actual_condition_lost|
|003|-3.0|28.0|1.0|fixed_A|A|True|True|True|None|
|003|-3.0|28.0|1.0|screened|A|True|True|True|None|
|004|3.0|20.0|0.5|fixed_A|A|True|True|True|None|
|004|3.0|20.0|0.5|screened|B|True|False|False|cleared_hold_actual_condition_lost|
|005|3.0|20.0|1.0|fixed_A|A|True|True|True|None|
|005|3.0|20.0|1.0|screened|B|True|True|True|None|
|006|3.0|28.0|0.5|fixed_A|A|True|True|True|None|
|006|3.0|28.0|0.5|screened|B|True|False|False|cleared_hold_actual_condition_lost|
|007|3.0|28.0|1.0|fixed_A|A|True|True|True|None|
|007|3.0|28.0|1.0|screened|B|True|True|True|None|

## 实现文件

|文件|作用|
|---|---|
|scripts/pass_release_core.py|从实际EXIT日志重建检查点；停驻与连续性检查；PASS、CLEARED_HOLD、RELEASED状态机与完整日志|
|scripts/run_pass_release.py|八组初值两候选生成、冻结退出选角、独立运行两基线后续阶段、结果和计时报告|
|config/pass_release.json|释放裕度、停驻确认时长、停驻保持阈值、EXIT/PASS/总超时|
|scripts/verify_pass_release.py|独立从日志核验实际动力学、真实车身清空、分项验收、阶段连续性与角色一致性|
|tests/test_pass_release.py|中途违规、漂移、预测提前清空、车尾未清空、超时与不改选等失效机制测试|
|pass_release_protocol.md、README.md|几何、状态、日志、运行与结论说明|
原DMPC复现入口、原退出/QP求解器和历史报告均未修改。新增结果使用独立目录。

## 汇总与配对比较

|策略|EXIT成功|退出后PASS成功|释放成功|完整链成功|平均退出 s|平均通过 s|平均阻塞至释放 s|平均退让 m|平均通过 m|
|---|---|---|---|---|---|---|---|---|
|fixed_A|8/8|6/8|6/8|6/8|17.2667|18.4667|35.7333|15.1001|27.3421|
|screened|8/8|4/8|4/8|4/8|15.0000|16.2000|31.2000|11.1000|23.3445|

上述性能均值的样本集合为各自完整链成功运行；PASS成功率以退出成功样本为分母，释放与整体成功率以全部初值为分母。
共同成功初值：[1, 3, 5, 7]。
共同成功集合 screened−fixed_A 的平均 retreat_distance_m 差：-3.000000949457373。
共同成功集合 screened−fixed_A 的平均 pass_simulation_s 差：-1.4000000000000128。
共同成功集合 screened−fixed_A 的平均 initial_to_release_simulation_s 差：-3.000000000000014。

## 失败与原始证据

- case_000/fixed_A: CLEARED_HOLD / cleared_hold_actual_condition_lost；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。
- case_000/screened: CLEARED_HOLD / cleared_hold_actual_condition_lost；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。
- case_002/fixed_A: CLEARED_HOLD / cleared_hold_actual_condition_lost；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。
- case_002/screened: CLEARED_HOLD / cleared_hold_actual_condition_lost；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。
- case_004/screened: CLEARED_HOLD / cleared_hold_actual_condition_lost；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。
- case_006/screened: CLEARED_HOLD / cleared_hold_actual_condition_lost；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。

退出成功但未完成PASS停驻确认的运行数：6。
本次失败分支均已实际整车离开R，且曾进入目标/低速阈值；但CLEARED_HOLD下一段0.4 s积分检查显示速度将再次超过0.025 m/s，整段确认不通过。该段未提交到实际模拟状态，path_checks.json标记applied_to_simulation=false；故不是车身无法穿过通道的几何证明。没有改变控制、放宽阈值或重新选角来补救。
实际最小间隙只覆盖actual.csv中的已提交状态；拒绝执行的积分段与未执行预测不计入minimum_executed_gap_m。

## 逐初值计算量

|初值|选角前墙钟 s|两EXIT生成墙钟 s|两EXIT公共主调用|策略|EXIT归属主调用|PASS主调用|HOLD主调用|PASS计算墙钟 s|HOLD计算墙钟 s|诊断/重试|
|---|---|---|---|---|---|---|---|---|---|---|
|000|1.247371|1.247343|166|fixed_A|78|74|2|0.736290|0.006964|0/0|
|000|1.247371|1.247343|166|screened|78|74|2|0.732058|0.007508|0/0|
|001|0.617633|0.617622|156|fixed_A|70|72|4|0.442762|0.016317|0/0|
|001|0.617633|0.617622|156|screened|70|72|4|0.414784|0.015290|0/0|
|002|1.300662|1.300651|186|fixed_A|88|84|2|0.893789|0.007683|0/0|
|002|1.300662|1.300651|186|screened|88|84|2|0.928543|0.021388|0/0|
|003|0.718420|0.718409|176|fixed_A|80|82|4|0.502625|0.015423|0/0|
|003|0.718420|0.718409|176|screened|80|82|4|0.517291|0.018192|0/0|
|004|1.174735|1.174725|166|fixed_A|88|92|4|1.047172|0.015336|0/0|
|004|1.174735|1.174725|166|screened|78|74|2|0.730014|0.007306|0/0|
|005|0.616843|0.616832|156|fixed_A|86|86|4|0.510926|0.014575|0/0|
|005|0.616843|0.616832|156|screened|70|72|4|0.389720|0.014906|0/0|
|006|1.184359|1.184344|186|fixed_A|98|102|4|1.017454|0.015645|0/0|
|006|1.184359|1.184344|186|screened|88|84|2|0.829569|0.007574|0/0|
|007|0.679177|0.679165|176|fixed_A|96|96|4|0.574800|0.016090|0/0|
|007|0.679177|0.679165|176|screened|80|82|4|0.477534|0.014525|0/0|

同角色案例的actual.csv完全一致：{0: True, 1: True, 2: True, 3: True}。

每个策略目录保存 EXIT 至终态的 actual.csv、predictions.jsonl、solves.json、path_checks.json、events.jsonl、summary.json。选择结果在任何PASS模拟前写入 case_*/selection.json；只依据完整退出成功和退让距离。相同角色的两基线独立继续积分，actual.csv哈希一致性见summary.json的same_role_check。
两条EXIT来自同初值的独立候选；所选候选实际执行日志作为各策略EXIT前缀，后续从该日志终态继承时间、x/v/y、上一实际加速度及warm controls。没有重置状态、覆盖速度或控制。通过期间退让参考取实际终态，非理想退让点。
邻车掩码按横向占用几何重算并逐样本记录，保留原纵向邻车线性约束的掩码构造；PASS时横向间距3 m超过2.5 m纵向约束激活阈值，外部配对车身检查始终生效。
局部独立分项验收、配对全预测、实际执行子步、阶段完成条件均必须通过。预测提前清空不产生释放；实际车尾清空时间另记，停驻确认和释放更晚。

## 时间、预算与边界

选角前墙钟等待范围 0.616843–1.300662 s，包含两条完整EXIT预演与日志写入，不含进程启动。控制周期0.4 s、预测6.4 s、检查间隔0.1 s；主OSQP每次20000迭代上限，无诊断调用，无重试。
EXIT/PASS/整个模拟超时分别100.0/100.0/200.0 s；PASS超时包括CLEARED_HOLD。超时终止模拟，未实现实时deadline中断或物理备用停车。各阶段计算墙钟时间和主调用详见每分支summary.json，候选生成公共成本见selection.json，不能把公共生成成本与每条分支的EXIT重计为实际CPU总量。
模拟时间与计算墙钟时间分别报告，未相加成部署恢复时间。方法仍为离线预演，未补偿真实车辆等待期间的环境变化，不能声称满足在线实时预算。
数值残差仅支持返回解在给定容差下通过独立验收；采样安全仅支持本组实际执行样本通过模型道路与间距检查；阶段结果只支持本组初值、当前模型下的一次完整退出/通过/释放。未证明连续时间安全、转向可达性、递归可行性或双方任务完成。

## 分项数值验收与失败记录索引

|验收项目|已接收主解最大残差|
|---|---|
|dynamics_position_m|2.8421709e-14|
|dynamics_velocity_m_s|4.4964032e-15|
|speed_m_s|2.2202444e-08|
|acceleration_m_s2|2.7437044e-08|
|terminal_velocity_m_s|3.0951139e-10|
|road_longitudinal_m|0|
|neighbor_m|0|
|target_overshoot_m|1.1984369e-07|

已提交执行子步独立动力学最大位置偏差 7.1054274e-15 m，速度偏差 4.4408921e-16 m/s。
全体策略已提交实际采样最小车身间隙 0.5 m；完整受检预测最小间隙 0.5025 m。二者独立汇总。
实际CPU主调用合计 2732：两EXIT候选公共生成 1368，两策略独立后续阶段 1364；诊断与重试均0。EXIT归属成本不能再次叠加。

按全部EXIT成功集合（两基线均8例）统计：
- fixed_A：平均EXIT模拟时间 17.1000 s，平均退让距离 14.1000 m；实际整车清空R 8/8。
- screened：平均EXIT模拟时间 15.8000 s，平均退让距离 11.1000 m；实际整车清空R 8/8。

下表为失败的最后一条分段验收，均未提交执行；阈值为0.025 m/s。失败不能被安全终点或预测清空覆盖。

|初值/策略|path_checks数组索引|检查段开始全局时间 s|段内最大速度 m/s|记录|
|---|---|---|---|---|
|000/fixed_A|154|130.4|0.02648386|[path_checks.json](case_000/fixed_A/path_checks.json)|
|000/screened|154|130.4|0.02648386|[path_checks.json](case_000/screened/path_checks.json)|
|002/fixed_A|174|134.4|0.02648386|[path_checks.json](case_002/fixed_A/path_checks.json)|
|002/screened|174|134.4|0.02648386|[path_checks.json](case_002/screened/path_checks.json)|
|004/screened|154|130.4|0.02648386|[path_checks.json](case_004/screened/path_checks.json)|
|006/screened|174|134.4|0.02648386|[path_checks.json](case_006/screened/path_checks.json)|

[独立日志核验](validation.json) · [测试输出](test_results.txt) · [机器可读分析](analysis.json)
