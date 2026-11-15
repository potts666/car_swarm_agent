# 运行交接

目标：v2统一生成规则，A/B各一次倒车求解，保持历史001/002、A/P0及论文；无Git提交，无正式批量。
源版本：见plan.json/source_commit；v1未启动，见v1_audit.json。
当前证据：results/rule_checks_initial.json保存首次构造反例；rule_checks.json为修正后无求解共同规则；solver_calls.json为真实数值调用账；每个*.stage.json保存阶段完成/退出码，日志同名.stage.log。
恢复命令（不追加求解，成功阶段复用）：
`OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 stage_driver.py`
若阶段失败先读对应stage.log和原始对象，不删除日志，不重跑数值调用。两个调用用尽后只允许复算验收。
未解决及最终结论以report.md/GATE.json为准。更多顺序、组合、物理时钟和真实独立进程保持排队。

本轮完成：五项GATE、A/B各1次受理、各2802拍8子段独立验收、16损坏拒绝及2条99拍旧尾段。原始导入失败与构造反例保留。
本轮未解决：A数值状态AlmostSolved（通过完整证书而非配置精度）、域构造保守、速度/轮角不确定性鲁棒部署与更多组合未试。
复核：`OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py`。
`--full-replay`可重新完整物理验收而不调用求解器。正常恢复stage_driver只校验内容检查点，不重跑已完成求解。
