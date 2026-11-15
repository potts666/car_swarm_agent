import os,sys,subprocess,json,time
from pathlib import Path
R=Path(__file__).resolve().parent
STAGES={'analytic':['engine.py --analytic'],'preflight':['check_zero.py'],'freeze':['freeze_registration.py'],'numeric':['engine.py --numeric'],'recovery':['recover.py'],'accept':['audit_updates.py'],'delivery':['summarize.py','verify_delivery.py']}
if __name__=='__main__':
 stage=sys.argv[1];env=os.environ.copy();env.update(PYTHONPATH='/tmp/dmpc-p0-deps',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 for command in STAGES[stage]:
  script,*args=command.split();beg=time.time();log=R/f'results/{script[:-3]}_{stage}.log'
  with log.open('ab') as f:
   try:code=subprocess.run([sys.executable,str(R/script),*args],stdout=f,stderr=subprocess.STDOUT,env=env,timeout=3600).returncode
   except subprocess.TimeoutExpired:code=124
  result={'stage':stage,'exitcode':code,'layer':stage,'log':str(log),'seconds':time.time()-beg,'event':'completed' if code==0 else 'failed'}
  with (R/'results/stage_events.jsonl').open('a') as f:f.write(json.dumps(result)+'\n')
  print(json.dumps(result),flush=True)
  if code:sys.exit(code)
