# 阶段交接：P0 v2及15次开发运行已完成

目标：用户设计与三项定理前提核对、P0运动终端/路线交接认证与小型核对、三例×五组开发运行。
约束：无正式留出批量、无论文修改；历史控制入口、结果和本地大归档保留；未自动提交/推送。
源码基准：ebc3d29，feature/deadlock-experiment；新增仅在experiments/local_coordination_p0/。
完成：原对象/预算重建、33条前提矩阵、真实SOCP锚点、输入移位K、三车组合并拆分、拒绝续行、停车换权后新车首拍；15次完整四车任务及15次独立验收通过。
当前门禁：P0_GATE_v2.json。冻结源码/依赖/模型/案例/证据SHA已复核。
主报告：development_report.md；完整指标：development_metrics.json；参数登记：pending_parameters.json。
原始证据：results/pilot_032/status.json，30阶段均passed；每次summary、独立验收、实际执行gzip及策略版本均保留。
P0：source_rebuild_005、moving_objects_006、final_units_020、final_conic_021、p0_moving_v2_028、p0_stopped_v2_029、acceptance_damage_v2_031。
失败历史：007/008 AlmostSolved拒绝；025第47拍整数/有理数零哈希缺陷，026诊断；修正后身份回归及P0重新通过。没有覆盖失败证据。
保留复核：results/preservation_034/checks.json。原14段951拍/190.2秒见证重验通过，比当前保守候选库更快。
未解决：正式分布/样本/统计、停止节点闭环敏感性及资源接口强基线、兼容运动并行/计算并行、全四车组消融、外部适配、持续时钟、规模实验；当前粗节点v/delta0，任意五状态开域未实现。非零交接在同车路线内部，车间换权仍停车。

无需重跑已通过实验。精确只读恢复检查：

```bash
python3 - <<'CHECK'
import json, hashlib
from pathlib import Path
p=Path('experiments/local_coordination_p0')
g=json.loads((p/'P0_GATE_v2.json').read_text())
assert all(hashlib.sha256((p/n).read_bytes()).hexdigest()==v for n,v in g['algorithm_sha256'].items())
s=json.loads((p/'results/pilot_032/status.json').read_text())
assert s['passed'] and len(s['stages'])==30
print((p/'development_report.md').read_text())
CHECK
```

显式独立重跑入口及新输出目录示例见README.md；需使用新目录，不覆盖本轮。临时依赖/tmp/dmpc-p0-deps，版本/SHA见requirements-development.txt及dependency_provenance/。
后续若要正式实验，应先根据pending_parameters.json冻结正式方案；本轮不启动。可在新会话从本交接继续。
