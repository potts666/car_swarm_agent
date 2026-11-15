# 新阶段交接：区域资料已读，接入尚未实施

目标：保留认证图引导，逐步接入五状态区域/K、实际导数、可变时长，同时满足连续安全、递归继承和有限完成。
约束：本轮仅资料、轻量复核与方案；无控制器修改、无新闭环实验、无正式批量、无论文/历史改写、无Git提交。
原源码：ebc3d29，feature/deadlock-experiment；旧门禁P0_GATE_v2.json及15次证据身份保持。
新来源：Downloads/DMPC_region_handoff_proof及DMPC_map_interface_certificates；完整路径/SHA见source_inventory.json。
已执行：指定verify_certificates.py，退出0；4个重写JSON与原件语义相同，新旧输出分别保存后恢复原字节，整包64项清单通过。
证据：verify_certificates.log、verification_stage.json、source_inventory.json、identity_and_observations.json。
报告：review_and_minimal_plan.md；结构化计划：minimal_plan.json。
最近代码增量：仅做A，一个直线和一个弯道，固定m，接新盒/K及独立移位验收；A通过后B，再C。三个增量均未开始。
重点风险：矩阵范围仅该phase盒/反馈输入矩形，不能盲拼成优化轨迹；末端等式/修复/提交/验收及恒输入预消元要一起改；缩时允许能量上升；R预算不能覆盖旧较小预算；旧完整尾策略始终保留。
本机未完整重建所有连续/导数证书；交付方portability记录不是本机Ubuntu完整复算。没有新控制收益结论。
精确只读恢复：

```bash
cat experiments/local_coordination_p0/reviews/region_handoff_20261112/review_and_minimal_plan.md
cat experiments/local_coordination_p0/reviews/region_handoff_20261112/verify_certificates.log
```

不要重跑旧15次或使用旧门禁声称新功能已验证；实施应使用新门禁和新结果目录，原资料与失败记录保留。
