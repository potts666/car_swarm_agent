"""Zero-call content verification and independent key prefixes, including partial gates."""
import sys,argparse,json,gzip,hashlib,tempfile,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT))
from common import *
from accept_case import Independent
from rules import quantize
from socp_rows import evaluate
from receivers import feedback

def full_replay(arm):
 manifest_name='executed_source_versions.C_v1.json' if arm=='C-new' else 'executed_source_versions.L_v3.json' if arm=='L-new' else 'executed_source_versions.zero_v1.json';manifest=read(ROOT/manifest_name);target=Path(tempfile.mkdtemp(prefix='dg002_replay_'))
 for name in ('inputs','sources','objects'):shutil.copytree(ROOT/name,target/name)
 shutil.copytree(ROOT/'results'/arm,target/'results'/arm)
 for name in ('domain_checks.json','rational_inputs.json'):shutil.copy2(ROOT/name,target/name)
 shutil.copy2(ROOT/('pre_registration.json' if arm=='L-new' else 'results/pre_registration.C_v1.json'),target/'pre_registration.json')
 for name,digest in manifest.items():
  path=ROOT/'source_archive'/digest/name;assert sha(path)==digest;shutil.copy2(path,target/name)
 result=subprocess.run([sys.executable,'accept_case.py','--arm',arm],cwd=target,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert result.returncode==0,result.stderr;accept=read(target/'results'/arm/'independent_acceptance.json');assert accept['passed'];return dict(arm=arm,passed=True,source_manifest=manifest_name,steps=accept['steps'],new_calls=0)

def verify(full=False):
 before=sha(ROOT/'solver_calls.json');manifest_hash=sha(ROOT/'SHA256SUMS');count=0
 for line in (ROOT/'SHA256SUMS').read_text().splitlines():
  h,relative=line.split('  ',1);assert sha(ROOT/relative)==h,relative;count+=1
 for p in (ROOT/'objects').glob('*.json.gz'):assert content_id(json.loads(gzip.decompress(p.read_bytes())))==p.name.split('.')[0],p.name
 calls=read(ROOT/'solver_calls.json');assert len(calls)==12 and len({x['event'] for x in calls})==12 and all(x['completed'] for x in calls)
 for arm in ('C-new','L-new'):assert sum(x['arm']==arm for x in calls)==6<=8
 gate=read(ROOT/'GATE.json');assert gate['status']=='partial' and not gate['passed'] and not gate['controlled_comparison_valid'];prefixes=[];replays=[]
 versions=[read(ROOT/n) for n in ('executed_source_versions.zero_v1.json','executed_source_versions.C_v1.json','executed_source_versions.L_v3.json')]
 for record in calls:assert record['source_version'] in [content_id(x) for x in versions]
 for arm in ('R-old','C-new','L-new'):
  out=ROOT/'results'/arm;summary=read(out/'summary.json');acceptance=read(out/'independent_acceptance.json');assert summary['passed'] and acceptance['passed'] and summary['steps']==2802
  sources={}
  with gzip.open(out/'execution.jsonl.gz','rt') as f:
   for row in map(json.loads,f):
    if row['event_round']:sources[row['cycle']]=row['source']
  for e in summary['events']:
   assert sources[e['cycle']]==e['actual_source'];reference=e['anchor_source'];w=get(ROOT/'objects',e['selected_witness']);base=get(ROOT/'objects',reference['witness']);k=e['cycle']
   for a in e['admissions']:
    job=a['job'];g=get(ROOT/'objects',job['generation_id']);anchor=get(ROOT/'objects',job['anchor_id']);raw=get(ROOT/'objects',a['raw_result_id']);assert raw['source']==reference==anchor['state'] and raw['generation_id']==job['generation_id'];delta=[F(x)-F(y) for u,v in zip(a['candidate']['controls'][:32],base['controls'][job['actor']][k:k+32]) for x,y in zip(u,v)]
    if a['accepted']:
     assert evaluate(get(ROOT/'objects',g['physical_rows_id']),delta)['passed'];assert any((lambda q:q[1]['passed'] and q[0]==delta)(quantize(raw['x'],alpha,anchor['contracts'])) for alpha in (F(1),F(1,2),F(1,4)))
    else:assert a['candidate']['selection']=='candidate_contract_anchor' and a['candidate']['controls']==base['controls'][job['actor']][k:]
   audit=Independent(read(ROOT/'pre_registration.json')['initial_witness']);audit.state=restore(m,e['actual_source']);audit.p8=copy.deepcopy(audit.state['poses']);checked_K=[]
   for t in range(k,k+33):
    for c,kind in w['events'].get(str(t),[]):
     if kind.startswith('generated:'):
      audit.domain(c,kind);assert tuple(map(F,w['controls'][c][t]))==feedback(audit.state,c,get(ROOT/'objects',kind.split(':')[1]));checked_K.append([c,t])
    audit.step({c:tuple(map(F,w['controls'][c][t])) for c in CARS},w['owners'][t]);audit.face_check(e['contract_id'],t)
    for c,kind in w['events'].get(str(t+1),[]):audit.domain(c,kind)
   prefixes.append(dict(arm=arm,round=e['round'],steps=33,passed=True,checked_K=checked_K,actual_numerical_accepted=sum(a['accepted'] for a in e['admissions'])))
  if full:replays.append(full_replay(arm))
 fork=read(ROOT/'results/four_car_fork/summary.json');fa=read(ROOT/'results/four_car_fork/independent_acceptance.json');assert fork['passed'] and fa['passed'] and fork['new_calls']==0 and len(fork['first_activity'])==4 and fa['steps']==99
 assert before==sha(ROOT/'solver_calls.json');return dict(passed=True,experiment_gate='partial; C numerical update and same-version comparison remain incomplete',manifest_files=count,manifest_sha256=manifest_hash,native_calls=12,limit=16,new_calls=0,key_prefixes=prefixes,full_replays=replays)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--full-replay',action='store_true');a=p.parse_args();print(json.dumps(verify(a.full_replay),indent=2))
