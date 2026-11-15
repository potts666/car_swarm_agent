# dynamic_groups_002：多活动车合同协商

日期：2026-11-14。本包包含新增理论与Ubuntu实验指令。Windows本轮只做推导、文档及代数检查。

Ubuntu Codex按序阅读：mainline_checkpoint.md、theory_extension.md、experiment_instructions.md、previous_theory/theory.md。先定位本机dynamic_groups_001原始交付并核对，再创建dynamic_groups_002。

目标：优先探索3/4车有效推进，构造真实改变物理边界或资源时段的新合同，释放旧方案中保持车辆的动作，完成连续两轮本车优化、安全接续与全程完成验证。

数值调用上限16。先完成零调用合同构造和预登记；两个数值臂各两轮、每轮最多4车；R-old重放使用0次新求解。复用已有模型、依赖与历史，保留论文，不提交Git。

本包未附Ubuntu的001代码与轨迹。若原工程缺失，先列实际查找路径并请求材料。新理论条件与数学证明见theory_extension；旧模型、接口和DOI来源在previous_theory。

解压后核对：

```bash
python3 verify_package.py
```

精确代数自检：

```bash
python3 math_checks.py
```

两项入口均为零优化器调用。overview.html为阅读导图，完整执行要求以experiment_instructions.md为准。
