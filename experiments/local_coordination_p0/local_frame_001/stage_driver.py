"""Bounded autonomous stages with durable exit/layer/log events."""
import os,subprocess,sys,time,json
from pathlib import Path
R=Path(__file__).resolve().parent
STAGES={'frame':['frame.py'],'preflight':['check_zero.py'],'freeze':['freeze_registration.py'],'numeric':['run_case.py'],'accept':['audit_updates.py'],'delivery':['summarize.py','verify_delivery.py']}
if __name__=='__main__':
 stage=sys.argv[1];env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONPATH='/tmp/dmpc-p0-deps')
 for script in STAGES[stage]:
  beg=time.time();log=R/f'results/{script[:-3]}.log'
  with log.open('ab') as f:
   try:code=subprocess.run([sys.executable,str(R/script)],stdout=f,stderr=subprocess.STDOUT,env=env,timeout=2400).returncode
   except subprocess.TimeoutExpired:code=124
  event={'stage':stage,'script':script,'exitcode':code,'layer':stage,'seconds':time.time()-beg,'log':str(log),'event':'complete' if code==0 else 'failure'}
  with (R/'results/stage_events.jsonl').open('a') as f:f.write(json.dumps(event)+'\n')
  print(json.dumps(event),flush=True)
  if code:sys.exit(code)
