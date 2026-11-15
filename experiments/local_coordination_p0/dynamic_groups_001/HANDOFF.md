# 运行交接（2026-11-14）

目标：dynamic_groups_001 按用户任务书落实三臂两轮两车、完整四车任务、独立审计与便携证据。历史只读，无Git提交；最多12次新Clarabel.solve，调用先登记，不能重新数值求解已完成事件。

根目录即本文件所在目录。历史与理论真实路径及哈希在source_inventory.json。零调用A/B前置门禁已通过：pre_registration.json（721/722，A1/B1）、results/generation_regression.json、results/terminal_hold.json、results/protocol_checks.json。

F-S已完成4次调用，两轮各两候选受理、两辆实际运动且首拍改变，2802拍到目标。solver_calls.json为唯一账本。F-S原独立验收在换权源快照served可变列表处失败，日志results/F-S_accept.stage.log保留；common.state_record现已复制served。原execution.jsonl.gz保留，execution_source_reconstructed.jsonl.gz仅从既存逐拍served链重建4处源字段；变更清单source_snapshot_reconstruction.json。数值源/命令/状态/history及期限未改。

原执行代码按内容保存source_archive/<sha>/<name>，executed_source_versions.v1.json为首版本。当前源在executed_source_versions.json。审计使用*_accept_v2阶段名，旧失败不覆写。尚须完成F-S独立验收、F-P/D-P各4次调用与独立验收、基线及拒绝分支独立审计、报告和搬移复核。

恢复命令（先核对账本，不追加已完成调用）：
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 stage_driver.py

现有/tmp/dg_baseline.py是零调用完整基线审计入口；曾为避免干扰数值计时中断，日志保留。全部数值阶段结束后再运行，禁止与数值计时并发。

## 数值阶段完成后的更新
三臂均完成4次求解且完整2802拍执行、各自独立8子段验收通过。全部12次为AlmostSolved，完整证书受理12/12，两轮均两车实际首拍改变。F-P/D-P每轮均有实际worker重叠。solver_calls.json预算耗尽；不得追加或重跑。后处理只允许零调用。数值第二版本源码保存executed_source_versions.solver_v2.json及source_archive/；stage_driver现支持核对归档源后复用既有阶段。

正在完成旧基线与真实数值候选回退分支的独立验收。postsolve_rejections.py已通过15项实际候选复制测试，选择失败A1锚点/旧合同同时保留B1新候选，完整尾段均已保存。剩余入口：delivery_tools/accept_baseline.py、delivery_tools/accept_forks.py、delivery_tools/timing_replay.py、delivery_tools/summarize.py及verify_delivery.py。最终复核需先生成SHA256SUMS；搬移副本复核不能再启动新优化。

## 最终结果
GATE.json核心六行均通过。三臂每臂4/4受理，两轮各2车首拍改变，物理输出三臂逐项相同。实际[721,755)含新K的输入平方代价改善0.0961090558%；完整任务改善0.00783543465%。全部12次AlmostSolved。主线各2802拍，拒绝分支各2081拍，独立8子段全部通过。历史882份文件哈希未变。
默认最终恢复/复核：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py。无新求解，完整重放另加--full-replay。证据归档位于同级dynamic_groups_001_evidence.tar.gz及.archive.json。后续最小任务已在report.md排序。
