"""Bounded durable stages; native releases never auto-repeated."""
import os,subprocess,sys,time
from common import *
STAGES={'reference_branches':['execution.py'],'R_accept':['audit.py','results/R_old.json.gz','--trace','results/R_old.jsonl.gz','--output','results/R_old_acceptance.json'],'E_accept':['audit.py','results/E_anchor.json.gz','--trace','results/E_anchor.jsonl.gz','--output','results/E_anchor_acceptance.json'],'worker_precheck':['run_case.py','--precheck'],'protocol_precheck':['check_zero.py'],'numerical':['run_case.py'],'updates':['audit_updates.py'],'physical':['audit.py','results/updated.json.gz','--trace','results/actual.jsonl.gz','--output','results/independent_physical.json'],'round0_policy':['audit.py','results/round_0_committed.json.gz','--output','results/round0_policy_acceptance.json'],'rejections':['check_rejections.py']}
def run(name):
 sources={p.name:sha(p) for p in sorted(HERE.glob('*.py'))};stamp=HERE/f'results/{name}.stage.json'
 if stamp.exists():
  prev=read(stamp)
  if prev.get('exit_code')==0 and prev['sources']==sources:return
  if name=='numerical':raise RuntimeError('Numerical stage already released/attempted; inspect ledger, do not rerun.')
 if name=='numerical':assert read(HERE/'solver_calls.json')==[]
 log=HERE/f'results/{name}.{time.time_ns()}.log';cmd=[sys.executable]+STAGES[name];start=time.time();write(stamp,{'status':'running','sources':sources,'command':cmd,'log':str(log),'start':start})
 with log.open('w') as f:r=subprocess.run(cmd,cwd=HERE,env=os.environ.copy(),stdout=f,stderr=subprocess.STDOUT)
 code=r.returncode
 if name=='numerical' and code==0 and not read(HERE/'results/numerical_summary.json')['passed']:code=2
 out={'status':'completed' if code==0 else 'failed','exit_code':code,'process_exit_code':r.returncode,'layer':name,'log':str(log),'sources':sources,'wall_seconds':time.time()-start};write(stamp,out)
 with (HERE/'results/stage_events.jsonl').open('a') as f:f.write(json.dumps(out)+'\n')
 print(json.dumps(out),flush=True)
 if code:sys.exit(code)
if __name__=='__main__':
 for name in sys.argv[1:]:run(name)
