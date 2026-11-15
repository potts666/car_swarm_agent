#!/usr/bin/env python3
"""Resume interrupted offline archival without rerunning any control experiment."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from m2b_diagnostic_archive import archive,restore

SCRIPTS=Path(__file__).resolve().parent

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,d):p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')

def run(root):
 assert read(root/'execution_status.json')['completed']
 for name,digest in read(root/'formal_freeze.json')['source_sha256'].items():assert sha(SCRIPTS/name)==digest,name
 names=('actual.jsonl.gz','plans.jsonl.gz','messages.jsonl.gz','solves.jsonl.gz','events.jsonl.gz','agreements.jsonl.gz','planning.jsonl.gz','config.json','summary.json','independent_validation.json')
 current={str((d/name).relative_to(root)):sha(d/name) for g in ('C-FIFO','L-FIFO','L-ALT') for d in sorted((root/g).glob('r[12]')) for name in names}
 baseline=root/'archival_resume_baseline.json'
 if baseline.exists():assert read(baseline)['input_sha256']==current
 else:write(baseline,dict(input_sha256=current,scope='recorded at offline resume before remaining archival; earlier archive code only writes diagnostics and objects',controller_runs_repeated=False))
 write(root/'archive_resume_status.json',dict(starting_checkpoints=len(list(root.glob('*/r[12]/diagnostic_archive.json'))),control_experiments_already_complete=True))
 archive(root)
 with tempfile.TemporaryDirectory(prefix='m2b_restore_resume_') as temp:restore(root,Path(temp)/'full')
 assert all(sha(root/name)==digest for name,digest in current.items())
 write(root/'archival_input_integrity.json',dict(acceptance_inputs_unchanged=True,input_sha256=current,baseline_scope=read(baseline)['scope'],diagnostic_logical_bytes_recovered=True,controller_runs_repeated=False))
 print('all acceptance inputs unchanged; all diagnostic logical hashes recovered',flush=True)
 analysis=root/'analysis_source';analysis.mkdir(exist_ok=True)
 sources=('check_m2b_delivery.py','m2b_storage_post.py','m2b_diagnostic_archive.py','plot_m2b.py','check_m2b_seeds.py','m2b_timing_post.py','finalize_m2b.py','finalize_m2b_resume.py')
 for name in sources:shutil.copy2(SCRIPTS/name,analysis/name)
 write(root/'analysis_source_manifest.json',{p.name:sha(p) for p in analysis.iterdir() if p.is_file()})
 target=root/'tests';target.mkdir(exist_ok=True)
 for p in (SCRIPTS.parent/'tests').glob('test_m2b*.py'):shutil.copy2(p,target/p.name)
 (target/'fixtures').mkdir(exist_ok=True);shutil.copy2(SCRIPTS.parent/'tests/fixtures/m2b_stop_prefix.json',target/'fixtures/m2b_stop_prefix.json')
 write(root/'frozen_config.json',read(root/'formal_freeze.json')['scene'])
 metadata=root/'shared_storage_footprint.json';files=[p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts];inodes={}
 for p in files:inodes[(p.stat().st_dev,p.stat().st_ino)]=p.stat().st_size
 footprint=dict(unique_file_bytes=sum(inodes.values()),apparent_file_bytes=sum(p.stat().st_size for p in files),soft_200MiB_met=sum(inodes.values())<=200*2**20,all_debug_counterexamples_retained=True,diagnostics_full_matrices_recoverable=True)
 write(metadata,footprint)
 metrics=read(root/'metrics.json');repeat=read(root/'repeatability.json');same=repeat['L-FIFO']['permission_orders']==repeat['L-ALT']['permission_orders'];a=read(root/'diagnostic_archive_manifest.json')
 report=root/'report.md';text=report.read_text();marker='## 最终交付复核'
 if marker not in text:
  table='\n'.join(f"|{n}|{m['cycle_s']['p50']*1000:.1f}|{m['cycle_s']['p95']*1000:.1f}|{m['cycle_s']['p99']*1000:.1f}|{m['cycle_s']['max']*1000:.1f}|{m['cycles_over_200ms']}|" for n,m in metrics.items())
  text+=f'''

{marker}

六次正式运行均完成并独立验收通过。32项交付测试通过（冻结前30项，另加无损诊断恢复/损坏拒绝及“输入修复后重算ID但未重预测”2项边界检查）；归档续做回归亦通过。种子前缀/尾部补充重演六轮均通过，最大差7.11e-14。原M1物理核心及原控制器文件哈希未变；旧34项M1回归只复用冻结证据，不冒充本轮新跑。

两种局部调度顺序相同：{same}，均为A1→B1→A2→B2；L-FIFO与L-ALT均176.4 s，各组两次实际轨迹逐字节一致。本场景未区分出调度收益，不调整初态制造赢家。后续可另设同侧先集中就绪、异侧后到达序列，本轮未执行。

完整提交周期采用相邻启动时刻实测间隔，包含提交后的精简记录、关键诊断整理及进度输出；原在线开始至提交结束计时保留为logged_commit_end字段。最后无输入的完成出口单列。所有实际提交周期均超过200 ms；不满足硬实时。

|运行|P50 ms|P95 ms|P99 ms|最大 ms|超过200 ms周期|
|---|---|---|---|---|---|
{table}

控制执行代价为统一的输入平方积分Σh(a²+ω²)，不是QP总目标，也不直接最小化任务时间。中央/局部的变量与耦合表达、预算作用域、消息认可工作量不同，不作等算力或纯架构因果结论。详细每车完成、端点误差、排队/许可等待、停走/补位、QP迭代/更新/修复/误差、通信字节和在线/离线成本见metrics.json。

原独立工件每轮约45–47 MiB，大于30 MiB软目标；最大项是交接、停车修复及其必要邻接周期的完整诊断约23–24 MiB，其次为完整最终321节点及可恢复种子的计划约19 MiB。未为达标删除矩阵或降低精度。后处理按字节相同的计划gzip文件硬链接共享，并把重复QP _details正文在diagnostic_objects/按内容ID保存一份；{a['objects']}个数值正文均可恢复，各诊断完整原JSONL字节SHA256逐轮一致。接续时先重验4份已完成归档，再完成余下2份，未重跑控制。所有验收实际输入、计划、协议和验收文件字节保持不变，见archival_input_integrity.json；其哈希基线时间及范围明确标注，不冒充最初运行时哈希。

共享存储唯一文件数据约{footprint['unique_file_bytes']/2**20:.2f} MiB，表观路径大小约{footprint['apparent_file_bytes']/2**20:.2f} MiB；200 MiB软目标：{footprint['soft_200MiB_met']}。共享对象需随全包交付；展开硬链接复制会回涨，不能把只含引用的单轮目录当独立完整小包。原始、仅gzip及引用gzip的逐轮实测见storage_measurements.json，节省不直接相加。存储操作均在线结束后执行，不算在线求解提速。

恢复完整诊断正文（输出必须是新目录）：

```bash
python3 examples/head_on_mpc/scripts/m2b_diagnostic_archive.py --run examples/head_on_mpc/results/M2b_fourcar_001 --restore-to /tmp/M2b_full_diagnostics
```

正常实际/协议验收可直接读取gzip和计划引用；完整诊断恢复还需要全包diagnostic_objects。正式源码按版本在sources/只存一份；补充种子/时序/存储与归档检查源码在analysis_source/另有哈希，不属于冻结控制算法的新版本。debug_v1 cycle43、debug_v2 cycle33、debug_c1 cycle689反例均保留；debug_v3完整运动通过但其早期诊断导出失败的身份亦保留，不替代正式六轮完整交付。

![实际轨迹及有效owner时间线](fourcar_trajectories.png)

未实施六车、批次、额外延迟、在线备用或四车中央全未来诊断。有限采样、理想同步通信、两次确定性重复不证明连续安全、递归可行、延迟鲁棒、硬实时、实车可靠或持续到车无饥饿。计算期间状态冻结，停止离线仿真不是安全停车。
'''
  report.write_text(text)
 subprocess.run([sys.executable,SCRIPTS/'check_m2b_delivery.py','--run',root],check=True)
 write(root/'finalization_status.json',dict(completed=True,all_six_acceptance_passed=True,offline_archival_resume=True,logical_diagnostics_restored=True))
 write(root/'artifact_manifest.json',{str(p.relative_to(root)):sha(p) for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='artifact_manifest.json'})
 print('M2b resumed finalization complete',flush=True)

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run(a.run)
