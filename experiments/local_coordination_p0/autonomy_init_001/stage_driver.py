"""Durable, bounded subprocess stages. Numerical stage is never auto-repeated."""
import json,os,subprocess,sys,time
from pathlib import Path
from common import HERE,sha,write
STAGES={
 'init_1_audit':[sys.executable,'audit.py','results/init_1.json.gz','--output','results/init_1_acceptance.json'],
 'preflight':[sys.executable,'preflight.py'],
 'numerical':[sys.executable,'run_update.py'],
 'updated_audit':[sys.executable,'audit.py','results/updated.json.gz','--trace','results/actual.jsonl.gz','--output','results/updated_acceptance.json'],
 'protocol':[sys.executable,'check_protocol.py'],
}
def run(name):
 cmd=STAGES[name];stamp=HERE/'results'/f'{name}.stage.json';ident={p.name:sha(p) for p in sorted(HERE.glob('*.py'))}
 if stamp.exists():
  prev=json.loads(stamp.read_text())
  if prev['exit_code']==0 and prev['sources']==ident:return prev
  if name=='numerical':raise RuntimeError('Numerical stage already attempted; inspect ledger. No automatic rerun.')
 if name=='numerical':assert json.loads((HERE/'solver_calls.json').read_text())==[]
 log=HERE/'results'/f'{name}.{time.time_ns()}.log';start=time.time();write(stamp,{'status':'running','command':cmd,'sources':ident,'log':str(log),'start':start})
 with log.open('w') as f:r=subprocess.run(cmd,cwd=HERE,stdout=f,stderr=subprocess.STDOUT,env=os.environ.copy())
 out={'status':'completed' if r.returncode==0 else 'failed','exit_code':r.returncode,'layer':name,'command':cmd,'sources':ident,'log':str(log),'wall_seconds':time.time()-start};write(stamp,out)
 with (HERE/'results/stage_events.jsonl').open('a') as f:f.write(json.dumps(out)+'\n')
 print(json.dumps(out),flush=True)
 if r.returncode:sys.exit(r.returncode)
 return out
if __name__=='__main__':
 for name in sys.argv[1:]:run(name)
