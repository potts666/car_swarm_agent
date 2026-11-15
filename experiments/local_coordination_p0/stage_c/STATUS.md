# C阶段交接：C0未过，C1通过，C2按任务书停止

目标：严格Solved策略下补齐JL接受/执行/移位，再验证解析及地图可变时长。
约束：原A/B/P0及门禁/失败记录不改；无Git、论文、正式批量。两修复变体各最多3配置，5秒/300迭代/1e-10，回退仅1、1/2、1/4。
源版本：ebc3d29，feature/deadlock-experiment。任务书副本sources/C_instructions.md；冻结B副本及必要依赖base_b。
完成：归档初始/shift四问题66维P/q/A/b和物理残差导出；实际导数重算；等价逐行旋转SOC/舍入界/零余量检查；两接口12次有界单问题求解；两次真实旧首拍续行及独立重演；C1精确缩时和独立重算。
结果：R/Q各三配置全部AlmostSolved，C0受理/新JL执行均0。Q的alpha1原行及预测模型复验通过，也不能绕过Solved规则。最后probe参数不代表已选定，repair_selection.json的selected=null。
门禁：C_GATE_v1.json，C0=false，C1=true，C2未启动。没有较短地图候选或实际节省拍数。
C1：同一五状态终点，20→11拍，4→2.2秒，节省9拍/1.8秒；能量0→9/8；同一3拍合法制动后续，预算23→14，首拍后13。
证据：report.md，C0_comparison.csv，analysis/numerical_levels.json，results/diagnostics_001，numeric_checks_001，C1_001，zero_slack_rounding_extra.json。原始大矩阵/求解点在objects按内容ID引用。
C2：仅只读重建b=197(含K)+1040=1237，规划m32/31/30；C2_comparison.csv均not_run，不能当成已认证/执行。
尚未解决：合格JL数值配置、接受后真实移位/K/换权、同配置ZG/ZL回归、C2变时长维数/时钟/提交/执行及拒绝核对。达到本轮参数上限，不继续自行扩展搜索或放宽容差。

只读恢复：

```bash
python3 - <<'CHECK'
import json,hashlib
from pathlib import Path
p=Path('experiments/local_coordination_p0/stage_c')
g=json.loads((p/'C_GATE_v1.json').read_text())
assert not g['C0_JL_nonzero_numerical_acceptance_passed'] and g['C1_exact_20_to_11_passed']
assert not g['C2_map_experiments_started']
assert all(hashlib.sha256((p/n).read_bytes()).hexdigest()==h for n,h in g['runtime_code_sha256'].items())
print((p/'report.md').read_text())
CHECK
```

无需重跑已保存试验。若另行明确授权重做诊断，使用新输出并加180秒外部限制；probe_repairs会再次执行全部预登记配置，不应在本轮继续调用：

```bash
PYTHONPATH=/tmp/dmpc-p0-deps OPENBLAS_NUM_THREADS=1 timeout 180s \
python3 experiments/local_coordination_p0/stage_c/diagnose.py --output /tmp/C-diagnosis-repeat
```

异目录恢复核对见relocation_check.json；归档DMPC_stage_C_2026-11-13.tar.gz。原历史A/B/P0包留原目录。可在新会话读取本文件继续分析未通过项；C2仍受C0门禁约束。
