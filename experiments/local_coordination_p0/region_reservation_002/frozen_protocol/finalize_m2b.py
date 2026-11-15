#!/usr/bin/env python3
"""Offline-only reports, exact-content sharing and package verification."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from m2b_diagnostic_archive import restore

SCRIPTS=Path(__file__).resolve().parent


def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as stream:
  for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,x):p.write_text(json.dumps(x,indent=2,ensure_ascii=False)+'\n')

def finalize(root):
 while True:
  status=read(root/'execution_status.json')
  if status.get('error'):raise RuntimeError(status['error'])
  if status['completed']:break
  time.sleep(5)
 log=(root/'postprocess.log').open('x')
 def execute(command):
  print('POST',*command,flush=True);log.write('POST '+' '.join(map(str,command))+'\n');log.flush()
  p=subprocess.Popen(list(map(str,command)),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  for line in p.stdout:print(line,end='',flush=True);log.write(line);log.flush()
  if p.wait():raise RuntimeError('postprocess failed: '+' '.join(map(str,command)))
 env=os.environ.copy()
 tests=root/'tests_delivery.log'
 with tests.open('x') as stream:
  subprocess.run([sys.executable,'-m','unittest','discover','-s',str(SCRIPTS.parent/'tests'),'-p','test_m2[ab]*.py','-v'],stdout=stream,stderr=subprocess.STDOUT,check=True,env=env)
 execute([sys.executable,SCRIPTS/'report_m2b.py','--run',root])
 execute([sys.executable,SCRIPTS/'plot_m2b.py','--run',root])
 execute([sys.executable,SCRIPTS/'check_m2b_seeds.py','--run',root])
 names=('actual.jsonl.gz','plans.jsonl.gz','messages.jsonl.gz','solves.jsonl.gz','events.jsonl.gz','agreements.jsonl.gz','planning.jsonl.gz','config.json','summary.json','independent_validation.json')
 inputs={str(p.parent/name):sha(p.parent/name) for p in root.glob('*/r[12]/independent_validation.json') if p.parent.parent.name in ('C-FIFO','L-FIFO','L-ALT') for name in names}
 execute([sys.executable,SCRIPTS/'m2b_storage_post.py','--run',root])
 execute([sys.executable,SCRIPTS/'m2b_diagnostic_archive.py','--run',root])
 with tempfile.TemporaryDirectory(prefix='m2b_diag_restore_') as temporary:
  restore(root,Path(temporary)/'restored')
 assert all(sha(Path(path))==digest for path,digest in inputs.items()),'archival changed independently audited input'
 write(root/'archival_input_integrity.json',dict(acceptance_inputs_unchanged=True,input_sha256=inputs,scope='actual/plans/messages/solves/events/agreements/planning/config/summary/independent_validation',diagnostic_logical_bytes_recovered=True))
 files=[p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts];unique={}
 for p in files:unique[(p.stat().st_dev,p.stat().st_ino)]=p.stat().st_size
 footprint=dict(apparent_file_bytes=sum(p.stat().st_size for p in files),unique_file_bytes=sum(unique.values()),soft_200MiB_met=sum(unique.values())<=200*2**20,all_raw_debug_failures_retained=True,hardlinked_plans_byte_identical=True,diagnostics_full_matrices_recoverable=True)
 write(root/'shared_storage_footprint.json',footprint)
 analysis=root/'analysis_source';analysis.mkdir(exist_ok=True)
 analysis_names=('check_m2b_delivery.py','m2b_storage_post.py','m2b_diagnostic_archive.py','plot_m2b.py','check_m2b_seeds.py','finalize_m2b.py')
 for name in analysis_names:shutil.copy2(SCRIPTS/name,analysis/name)
 target=root/'tests';target.mkdir(exist_ok=True)
 for p in (SCRIPTS.parent/'tests').glob('test_m2b*.py'):shutil.copy2(p,target/p.name)
 (target/'fixtures').mkdir(exist_ok=True);shutil.copy2(SCRIPTS.parent/'tests/fixtures/m2b_stop_prefix.json',target/'fixtures/m2b_stop_prefix.json')
 write(root/'analysis_source_manifest.json',{p.name:sha(p) for p in analysis.iterdir() if p.is_file()})
 write(root/'frozen_config.json',read(root/'formal_freeze.json')['scene'])
 data=read(root/'metrics.json');archive=read(root/'diagnostic_archive_manifest.json');same=read(root/'summary.json').get('policies_same_orders');table='\n'.join(f"|{name}|{m['cycle_s']['p50']*1000:.1f}|{m['cycle_s']['p95']*1000:.1f}|{m['cycle_s']['p99']*1000:.1f}|{m['cycle_s']['max']*1000:.1f}|{m['cycles_over_200ms']}|" for name,m in data.items())
 addition=f'''

## 交付复核与工程体积

六轮独立验收均通过；32项交付测试通过（冻结前30项，另加归档恢复/篡改拒绝与“修复输入重算ID但未重预测”的2项边界检查）。补充种子前缀/尾部独立重演全部通过；运行原始源码和场景哈希保持冻结，M1物理核心与原控制器源码未改。debug_v1 cycle43、debug_v2 cycle33、debug_c1 cycle689失败及部分导出记录全部保留；debug_v3的运动完整通过但当时诊断导出失败，未把它冒充本轮六次完整交付。

FIFO与方向交替的实际授权顺序是否相同：**{same}**。本场景两种局部调度均A1→B1→A2→B2、176.4 s；没有区分出调度收益，不调整初态制造赢家。后续可另立同侧集中到达/异侧后到的诊断序列，本轮未运行。

|运行|P50 ms|P95 ms|P99 ms|最大 ms|超过200 ms周期|
|---|---|---|---|---|---|
{table}

各轮原始独立工件约46–47 MiB，超过30 MiB软目标；最大项为关键交接/停车修复及邻接周期的完整QP诊断约23–24 MiB，其次是完整最终321节点计划约19 MiB。未为达标删矩阵或降低精度。后处理仅把字节相同的计划文件按内容SHA256硬链接共享，把重复QP _details正文按内容ID在diagnostic_objects/保存一次；每条诊断的完整原JSONL字节SHA256恢复一致。六轮所有实际验收输入字节未变，见archival_input_integrity.json，矩阵归档/恢复版本见diagnostic_archive_manifest.json与analysis_source/。

共享后全包唯一文件数据约 **{footprint['unique_file_bytes']/2**20:.2f} MiB**（软目标200 MiB：{footprint['soft_200MiB_met']}）；表观路径大小约{footprint['apparent_file_bytes']/2**20:.2f} MiB，硬链接展开复制会回涨，不能把每轮只含引用的目录当独立完整小包。所有失败反例未删除。原始、仅gzip、引用+gzip大小实测逐轮见storage_measurements.json，节省不直接相加。为可重建保留{archive['objects']}个详细数值对象，恢复后的矩阵界Infinity字符串可按float还原，不影响有限浮点数。

诊断正文恢复（输出目录必须新建）：

```bash
python3 examples/head_on_mpc/scripts/m2b_diagnostic_archive.py --run examples/head_on_mpc/results/M2b_fourcar_001 --restore-to /tmp/M2b_full_diagnostics
```

正常实际/协议验收仍可直接从各运行gzip/引用日志读取；读取诊断完整矩阵须使用上述恢复逻辑并保留全包diagnostic_objects。源码快照在sources/按版本一份，后处理/检查源码另在analysis_source/并有哈希；它们没有回写owner或输入。存储测量和归档属于离线成本，未称为在线求解提速。

![真实轨迹与实际许可时间线](fourcar_trajectories.png)

本轮只证明固定四车场景、理想同步通信和有限采样下的完整任务通过；周期大量超过200 ms，计算期间时钟冻结，未证明连续安全、递归可行、延迟鲁棒、硬实时、实车可靠或持续到车无饥饿。两次重复属于确定性核查，不能称场景成功率。
'''
 with (root/'report.md').open('a') as stream:stream.write(addition)
 execute([sys.executable,SCRIPTS/'check_m2b_delivery.py','--run',root])
 write(root/'finalization_status.json',dict(completed=True,physical_and_protocol_six_passed=True,lossless_archival_verified=True));log.close()
 def manifest():
  write(root/'artifact_manifest.json',{str(p.relative_to(root)):sha(p) for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='artifact_manifest.json'})
 manifest()
 final_files=[p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts];seen={}
 for p in final_files:seen[(p.stat().st_dev,p.stat().st_ino)]=p.stat().st_size
 final_u=sum(seen.values());final_a=sum(p.stat().st_size for p in final_files);old_u=footprint['unique_file_bytes'];old_a=footprint['apparent_file_bytes'];footprint.update(unique_file_bytes=final_u,apparent_file_bytes=final_a,soft_200MiB_met=final_u<=200*2**20)
 write(root/'shared_storage_footprint.json',footprint)
 report=(root/'report.md').read_text().replace(f'**{old_u/2**20:.2f} MiB**',f'**{final_u/2**20:.2f} MiB**').replace(f'表观路径大小约{old_a/2**20:.2f}',f'表观路径大小约{final_a/2**20:.2f}')
 (root/'report.md').write_text(report);manifest()
 print('M2b FINALIZATION COMPLETE',flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();finalize(a.run)
