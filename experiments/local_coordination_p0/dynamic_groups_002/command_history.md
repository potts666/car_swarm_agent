# 实际命令与保留证据

所有数值执行设置OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps。独立stage stdout/stderr不裁剪；results/commands.jsonl及*.stage.json保存命令、退出码与证据路径。

1. 任务包`verify_package.py`通过14文件；`math_checks.py`通过5类/625期限代数，均0调用；001的`verify_delivery.py`只读内容和前缀复核通过。来源及结果在source_inventory.json。
2. `python3 precheck.py`→results/precheck.attempt_1.log，完整扫描原/修正构造并认证41/42首窗；原不足η的扫描保留scan_initial.json。
3. `python3 freeze_registration.py`→results/freeze_registration.attempt_1.log，M3用完整实际尾段代价更正早期诊断32拍排序，来源未变，0调用。
4. `python3 check_regression.py`及`check_protocol.py`→results/regression.attempt_1.log、protocol.attempt_1/2.log；首次图册区间常数减法失败已保留，修复全圆弧端点回归后通过。
5. `python3 stage_driver.py`→results/stage_driver.log；R-old完整执行/独立审计后，父驱动为零调用全池门禁驻留，未释放数值调用。
6. `python3 delivery_tools/certify_pool.py`→results/pool_certification.log及.v2.log，16份完整候选逐一认证，13有效/3资源重叠拒绝。初进程为避免重复接收几何重放而中断，完成对象内容核对复用，历史日志不删。
7. `python3 delivery_tools/refresh_receivers.py`→results/refresh_receivers.log；封存完整合同几何余量下本车私有参考接收器，后续首4/次3零锚点重新通过。`check_regression.py`、`check_protocol.py`的新回归分别在*.receiver_v2.log。
8. `python3 delivery_tools/certify_atlas.py`→results/atlas_certification.initial.log及.log；60图册、历史弯道真实源的非线性进度Taylor零锚点通过。
9. 恢复驱动后C-new-v1发生6次新调用，全部未受理而安全执行新合同锚点；原raw/rows/量化失败和两轮真实源保留。父驱动再次驻留，L-new未释放。
10. `python3 delivery_tools/repair_registration.py`→results/repair_registration.log；0新调用修近距离分离轴选择、零半径终端保和闭包。旧源码/预登记分版本归档。
11. `python3 delivery_tools/repair_checks.py`→results/repair_checks.initial.log及.log；保留原始向量仅作为数学回归夹具，不是新求解；补充来源保护拒绝把旧raw重贴新源。共同物理模型、数值容差和Q网格未改。
12. L-v3源码冻结后恢复驱动，L-new两轮各3次真求解全部受理；累计12调用。全部三臂执行/独立审计完成。stage外层wall含驻留，原内部计时另列。
13. `python3 delivery_tools/postsolve_checks.py`→results/rejection/postsolve.log；复用L真候选，0求解验证worker异常/超时、新物理迁移缺ACK、次轮独立块拒绝。
14. `python3 delivery_tools/accept_rejections.py`→results/rejection/independent_acceptance.log；全旧/全锚点轨迹内容核对复用全程独立审计，相关Gi/K和G_in140交接重新独立重放。
15. `python3 delivery_tools/check_old_contract_exclusion.py`→results/old_contract_exclusion.log；明确旧A1–A2分离面70拍余量负0.02588343m，旧源有符号许可拒绝新首拍。
16. `python3 stage_driver.py`再次执行→results/stage_reuse_check.log，六个完成阶段只核对原输出与归档源码，0新增调用。
17. `python3 delivery_tools/summarize.py`→results/final_summary.log，生成实际计数、唯一拍号代价、报告和partial门禁。默认与搬移复核结果在verification/。

本轮没有安装新依赖、修改历史或论文、提交Git。不得为掩盖C失败重复原数值事件。新增受控对照需新版本和6次完整三车两轮预算，当前余4及C余2不能满足原口径。
18. 主实验结束后`python3 delivery_tools/four_car_fork.py`→results/four_car_fork.log；已有W_hold后缀省去2拍恒等反馈，39拍旧3→新4、40拍3，四份零锚点/新合同/完整控制至目标通过。它不在原主池，O为空，0新求解。
19. `python3 delivery_tools/accept_four_car_fork.py`→results/four_car_fork/independent_acceptance.log；独立99拍覆盖四个K71、G_in138和实际4→3活动。此诊断没有被追加到C/L主数值结果或伪称4车新求解。
