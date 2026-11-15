# C0理论诊断与最小修复任务

日期：2026-11-13。先读本文件，再读`theory.md`和`experiment_instructions.md`。

用户确定：先解释并解决C实验新候选未受理，再推进区域预约、可停车等待、多活动车与联合完成策略。本包给出只读归档分析、理论修复和可执行实验指令；新数值求解与实际控制留给Ubuntu执行。

## 文件

- `theory.md`：停止指标、严格内点、等价锥表达和完整证书受理证明。
- `experiment_instructions.md`：Ubuntu实施、定点对照、实际接续和验收标准。
- `overview.html`：可视化解释，浏览器直接打开。
- `stop_conditions.csv`：12次停止指标复算。
- `diagnostics.json`：原模型/数值锥内点与锥几何记录。
- `audit_archive.py`：仅Python标准库的只读精确复算，含断言。

本包引用已有`stage_c/objects/`，该工程数据保持在现有Ubuntu工程。请按实验指令核对对象ID。

## 只读复算

定位真实stage_c目录以后，可运行：

```bash
python3 audit_archive.py --source /home/sunflower/car_swarm_switch_ws/src/car_swarm_single_lane/experiments/local_coordination_p0/stage_c
```

预期12例均通过严格内点检查，停止失败分类为4/4/4。该命令只读取归档，向终端输出JSON。

## 提交给Codex的起始指令

阅读本包`README_UBUNTU.md`、`theory.md`和`experiment_instructions.md`。按指令先做E0原问题重放，再做E1等价锥表达对照，优先保留Solved和1e-10。必要时独立评估E2完整证书受理规则。随后在两个接口实际验证新候选、移位、K、拒绝续行与换权，独立验收通过后给出关闭结论。保存原A/B/C和原门禁，暂存C2、区域预约与多活动车实现。先核对Ubuntu对象及调用链，再修改实验隔离目录。
