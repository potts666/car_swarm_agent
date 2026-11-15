# B阶段交接：实验已执行，JL接受与接受后移位项未达标

目标：B0实际锚点导数、B1自由终端SOCP、两接口×三组同锚点对照、真实执行和移位认证。
约束：h=.2/m32/239拍/单活动车/phase%64分组/停车换权/原能量权重/Mc15/当前完成见证不变；无新时长、正式批量、全四车方法对照、论文更新、Git提交。
来源：sources/README.md、A_review.md、B_instructions.md；源版本ebc3d29，A_GATE/P0_GATE和原始文件保留。
已完成：B0全部实际源矩形/复合矩阵/局部界；校准与覆盖/身份拒绝；真实数值66变量SOCP；6组+2锚点参照+2JL追加运行及独立验收；8类数值损坏拒绝。
门禁：B_GATE_v1.json逐项记录，all_B_acceptance_requirements_passed=false，不允许正式批量。
实际结果：ZG/ZL四组Solved、有理化alpha1/2后非零输入及K真实执行；JL初始弯道AlmostSolved、直线NumericalError，均拒绝后续行。JL真实旧首拍后新锚点再认证通过，再求解仍拒绝。未证明“接受JL后移位”。
预算：版本1，8条当前完成边，1279拍；每条测试实际240拍后剩1039拍。不用C/R增加旧预算。
证据：results/comparison_002/status.json的21阶段全退出0；注意安全续行通过不等于数值提案接受。objects/保存唯一引用对象；comparison.csv、measurements.json、analysis/raw_candidate_diagnostics.json、report.md分别保存性能及分层结论。
失败保留：probe_001转换错误；execution_probe_001/002验收比较/字段名错误；comparison_001停止；各缩放、8/16/32加密失败及最终JL拒绝均保留。最终只放行comparison_002所绑定代码。
未解决：JL在原1e-10/Solved策略下的数值接受，接受JL后真实移位；不放宽状态策略，不以A构造点替代原始解。下一阶段应先分析锥行条件和gap/残差；不自动启动C或正式实验。

只读恢复：

```bash
python3 - <<'CHECK'
import json,hashlib
from pathlib import Path
p=Path('experiments/local_coordination_p0/stage_b')
g=json.loads((p/'B_GATE_v1.json').read_text())
assert all(hashlib.sha256((p/n).read_bytes()).hexdigest()==h for n,h in g['runtime_code_sha256'].items())
assert not g['all_B_acceptance_requirements_passed'] and not g['formal_batch_allowed']
print((p/'report.md').read_text())
CHECK
```

如明确要独立重跑已冻结对照，必须新输出目录；不必重跑已有通过证据：

```bash
PYTHONPATH=/tmp/dmpc-p0-deps OPENBLAS_NUM_THREADS=1 \
python3 experiments/local_coordination_p0/stage_b/run_b.py \
 --index experiments/local_coordination_p0/stage_b/results/enrichment_001/index.json \
 --output /tmp/dmpc-B-repeat
```

临时依赖清理后按requirements-development.txt和dependency_provenance恢复。分发包含必要依赖源码、数据、A参照及B证据，历史P0大批仍在原目录。移植验收见relocation_check.json。
