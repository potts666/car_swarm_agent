# C0_fix_002 连续两轮实际重新优化

本轮通过：弯道phase5/6和直线phase10/11均重新生成实际状态锚点、导数、模型并真正调用Clarabel，各两份数值新候选完成完整证书认证、四车共同提交和实际首拍输出修改。两个接口均完成240拍及独立验收。总共4次新求解，无定点追加、无缓存旧求解结果替代第一轮。

四次原始状态仍为AlmostSolved。受理采用certificate_suboptimal_v1，严格Solved修复仍未通过。此结果只支持两个既有接口的连续两轮，不推广为任意后续拍持续SOCP可行或全程性能保证。

## 理论、实现与观察分层

理论依据为instructions/theory.md §7 的R-S/R-R/R-F条件性定理，以及previous_instructions/theory.md §8 的完整证书受理命题。新前缀与可执行后续在共同提交前匹配，拒绝保留旧策略，执行真实首拍后预算下降；区域预约R1–R9仍属设计，没有新增地图证书。用户“先来后到”仅为研究先后顺序，车辆优先规则仍待设计。

实现调用链：anchor_for(实际五状态/history/已移位旧计划) → build(32子段) / verify → rows_for → Q预留 → 65维居中assembly → Clarabel新求解 → raw对象 → admit_current / admit → 原行、33拍ZOH、盒/K/流像/真实后续/能量/预算认证 → 独立来源核对 → A.commit → 执行端共同编号核对 → 实际execute → A.shift。

新求解入口和事件ID绑定在fix_socp.py；调度在execute_fix.py；validate_fix.py从每次raw_result_id读取来源，移除固定probe键；accept_fix.py独立重建实际状态、导数、终端行、K、四份公开计划及组合、移位和预算。damage_checks.py保留复制分支原始证据。模型、权限、分组、h=.2、N=32、rho=.001、Q预留、scale=1e-4、容差1e-10、300迭代/5秒以及停车换权均保持。

## 逐轮结果（各轮32拍预测窗重叠，不相加）

|接口|轮 / phase|求解状态|原始残差|绝对gap|alpha|本轮同窗预测输入能量改善|新接收/K phase|
|---|---|---|---:|---:|---:|---:|---:|
|curve|1 / 5|AlmostSolved|7.92e-09|8.85e-11|1|0.050054%|37|
|curve|2 / 6|AlmostSolved|1.39e-12|4.67e-08|1|0.052689%|38|
|straight_acceleration|1 / 10|AlmostSolved|1.63e-07|3.02e-11|1|0.063600%|42|
|straight_acceleration|2 / 11|AlmostSolved|2.28e-10|3.81e-08|1|0.060940%|43|

四个首拍均与当时旧候选的首拍不同，精确有理输入、改动范数、对偶残差、相对gap、预算、solver/admission/pipeline开销在comparison.csv。第一轮仅首拍实际执行，其余预测前缀被第二轮替换，不能把第一轮整窗预测改善当作已执行收益。第二轮32拍前缀实际执行完毕，随后执行其新K。

## 重叠接收/K整体替换

第一轮保存K37/42。移位后它位于第二轮32拍窗口的最后一拍。第二轮以旧真实尾段为优化锚点，重新认证整个32拍前缀及K38/43，使后继进入原流像39/44，保留第二轮offset33之后实际旧尾段。旧K37/42及旧接收盒义务随第二轮完整合同一起被新合同替换；旧时刻的输入由新前缀覆盖，不强行宣称仍精确执行原K。权限仍为当前车，车组触发不变。

四车共同计划的receiver、pending_reset、权限版本、witness、budget及plan_id在同一context里。执行端重新计算四份公开计划身份并核对共同编号，避免把旧提交配新K。事件显式保存run/interface/phase/history/parent/anchor/derivative/assembly/raw/policy/accepted_plan和commit ID；第二轮parent是第一轮共同受理后从实际后继得到的移位计划，且两轮锚点、导数与组装内容不同。

拒绝分支各自从第二轮实际状态复制：陈旧anchor、陈旧derivative、旧phase、损坏K，以及混合提交context。十个分支全部零非法提交/输出，拒绝后保存旧pending_reset全部字段及父链，实际执行旧首拍并沿尾段执行32拍，包含旧K37/42和后继流像核对。分支控制记录与主分支分开，未计入求解数或性能。主分支新K到期时还注入一次内容身份损坏，拒绝后实际执行该已认可K。

## 实际收益、连续安全和预算

在同一个不重叠的实际34拍区间（第一轮首拍+第二轮32拍前缀+新K）重算输入平方代价，与原认证连接在相同绝对拍号的命令比较：

|接口|实际计费区间phase|输入代价改善|道路净空下界m|六对车身净空下界m|
|---|---|---:|---:|---:|
|curve|[5,39)|0.082634%|0.498477|0.501991|
|straight_acceleration|[10,44)|0.120038%|0.498523|0.502028|

这里只比较固定时长的输入代价，无加速结论。单接口239拍连接+下一持有车首拍=240拍，预算1279→1039，每拍精确减1。当前8连接同版完成见证保持d038db36d63515f7775265ffff5907dfe4a337a08d9672a4a43f7fb30377ca4e，未上调预算或另取文档常数。独立Replay使用4/8子段、连续角点误差包络和六对矩形检查，核对源域、盒、实际仿射v/delta复位、继承流像、停车粗节点及下一车首拍。

## 分项验收与证据

- 实际来源更新：通过，四次新求解，第二轮实际history和移位parent逐事件复核；results/solver_calls.json、event_index.json。
- 连续重新优化受理：通过，各2次认证、2次共同受理、2次真实首拍改变；各results/<interface>/events.json。
- 实际安全接续：通过，各240拍、一个实际新K、停车换权，预算1039；execution.jsonl.gz和independent_acceptance.json。
- 拒绝续行：通过，5类×2接口复制分支，含实际旧K续行；damage_forks/和独立报告。
- 严格Solved数值修复：未通过，完整认证策略与数值状态分开保存。

四轮原约束及Q组装的严格内点均有精确有理见证（self_checks.json），本次第二轮未丢失可行内空间。历史A/B/C/C0_fix_001共1552个非缓存文件SHA未变；新包指令清单核对通过；103个本地内容对象ID复核通过。原P0、论文、分支保持，无Git提交。

## 复核、限制与后续

在本目录使用requirements.txt所列版本，复核已保存结果：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_stage.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
```

run_stage通过内容与执行代码checkpoint复用本次运行，不追加求解。verify_delivery验证分发清单、四个逐事件来源及两个独立接续和拒绝重放，不求解。要做新的数值重复，须另建目录与计划，不覆盖本次证据。

仍是名义ZOH受控重放、每拍一辆活动车、进程内同步协议；h=.2是逻辑采样步长。逐轮构造/认证开销在CSV中，不构成0.2秒硬实时声明。原始状态AlmostSolved，不提供严格最优性、扰动稳定性或任意拍数持续局部SOCP可行性证明。

未执行区域预约、多活动车、C2或正式批量；下一步先审查本次连续来源与K整体替换证据。资源边界、车辆优先规则、多车初始安排、持续物理时钟和独立进程并行均另立任务。
