# A阶段交接：两个固定时长接口验证已通过

目标：新盒/K接入、构造候选、真实共同提交/执行/移位、保留真实旧尾策略与当前同版完成见证，独立验收。
范围：仅A接口验证；不改变时长、目标权重、phase%64分组、单活动车和停车换权，不开展B或正式批量。
源码基准：ebc3d29，feature/deadlock-experiment。新代码位于本stage_a/；旧P0 v2及历史、论文保持。
接口：原基准M-T首边A2-0-2，37拍弯道和42拍直线加速；各32拍前缀。当前W版本1，8条实际边，旧1301/已认可1279拍；不使用C/R替代预算。
结果：results/interfaces_003/，两控制器运行+两独立验收+七类损坏证据核对，共5阶段全部通过；每条实际240拍（239拍连接+停车换权后新车首拍），剩余预算1039拍。
拒绝：每条6次，含K/phase/对象/域/期限，以及在新盒已提交、旧首拍正是K时再次损坏K；均拒绝后实际执行旧首拍。实际K回到下拍参考v/delta，非强制归零。
门禁：A_GATE_v1.json，仅A接口验证通过；B、变时长和正式批量不放行。
证据：report.md、measurements.json、results/interfaces_003/status.json、各目录execution.jsonl.gz/summary.json/independent_acceptance.json、acceptance_damage/checks.json。
历史：results/curve_001及interfaces_002保留其当时源码/结果。最终门禁只绑定003和当前代码。没有提交或推送。
未解决：新候选为构造输入，经完整区间验证；没有数值SOCP自由终端求解。B先满足B_ANCHOR_COVERAGE.md中的实际逐拍范围/反馈链式求导/信赖原点条件，再另做接入。没有新四车全任务对照、时长优化、并行、实时或新分组算法。
共享：新旧执行与验收共享已认证区间/三角/传播原语；独立几何和接收/K/移位/预算逻辑另写，非完全独立积分实现。

无需重复已通过运行。只读恢复核对：

```bash
python3 - <<'CHECK'
import json, hashlib
from pathlib import Path
p=Path('experiments/local_coordination_p0/stage_a')
g=json.loads((p/'A_GATE_v1.json').read_text())
assert g['A_interface_validation_passed'] and not g['formal_batch_allowed'] and not g['B_derivatives_connected']
assert all(hashlib.sha256((p/n).read_bytes()).hexdigest()==h for n,h in g['code_sha256'].items())
assert hashlib.sha256((p/g['input_manifest']).read_bytes()).hexdigest()==g['input_manifest_sha256']
print((p/'report.md').read_text())
CHECK
```

如明确需要独立复跑，使用新输出目录（不要覆盖003）：

```bash
PYTHONPATH=/tmp/dmpc-p0-deps OPENBLAS_NUM_THREADS=1 \
python3 experiments/local_coordination_p0/stage_a/run_stage_a.py \
  --inputs experiments/local_coordination_p0/stage_a/inputs_v2 \
  --rebuild experiments/local_coordination_p0/results/source_rebuild_005 \
  --output /tmp/dmpc-stage-a-repeat
```

依赖版本沿用原requirements-development.txt及/tmp/dmpc-p0-deps；临时目录被清理后先恢复。后续可从本文件在新会话接续，B尚未授权实施。
