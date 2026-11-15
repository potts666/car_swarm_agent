# autonomy_init_001

结果：**partial**。自主初始化通过；两轮数值更新未通过。

从白名单地图、车型、真实初态、目标和连接模板生成两份请求相关策略，
Omega_DAG实际依赖为B2→A2，B0=1955。两份均独立完整重放通过。

初次数值版本保留八次worker装配错误和1847拍安全锚点执行；最终检查还
发现该版本A1期限475→500，不满足完整合同接收条件，已撤回相应通过声明。
共同规则修复后，独立零调用分支两拍均四车有效推进、1822拍全部完成，
各车绝对期限不增。修复分支没有新的数值求解或数值受理结果。

入口：report.md、GATE.json、input_lineage.json、results/repair_v2/。

```bash
python3 verify_delivery.py
python3 verify_delivery.py --full-replay
```

两条验证命令均零新增求解；默认返回完整性与实验结果两个字段，partial
不会被改成passed。完整物理重放约数分钟。可在任意目录执行绝对脚本路径。

solver_calls.json记录8/8预算内worker尝试；都在native solve之前失败，
实际到达solve入口0次。按保守登记口径不再追加请求。禁止重跑numerical
阶段；修复后的数值重复应另开版本及预算。002同版C/L任务和预算未使用。

sources/为物理/几何原件副本；source_archive/numerical_v1为实际运行源码，
当前根源码含零调用修复。两套身份分别保留，原失败不改名为新版本结果。
历史和论文未改写，无Git提交。
