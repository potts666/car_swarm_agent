"""Single bounded stage invocation with durable success/failure evidence."""
import subprocess,sys,time,json,os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
STAGES={'freeze':['freeze_registration.py'],'numerical':['run_experiment.py'],'accept':['audit_updates.py'],'delivery':['summarize.py','verify_delivery.py']}
if __name__=='__main__':
 stage=sys.argv[1];assert stage in STAGES;env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONPATH='/tmp/dmpc-p0-deps');events=ROOT/'results/stage_events.jsonl';events.parent.mkdir(exist_ok=True)
 for script in STAGES[stage]:
  start=time.time();log=ROOT/f'results/{script[:-3]}.log'
  with log.open('ab') as f:
   try:r=subprocess.run([sys.executable,str(ROOT/script)],stdout=f,stderr=subprocess.STDOUT,env=env,timeout=1800);code=r.returncode
   except subprocess.TimeoutExpired:code=124
  event={'stage':stage,'script':script,'exitcode':code,'seconds':time.time()-start,'log':str(log),'event':'completed' if code==0 else 'failed'}
  with events.open('a') as f:f.write(json.dumps(event)+'\n')
  print(json.dumps(event),flush=True)
  if code:sys.exit(code)
