"""Advance authorized stages in order; stop on concrete failure without native retries."""
import subprocess,sys,json,time,os
from pathlib import Path
R=Path(__file__).resolve().parent
if __name__=='__main__':
 env=os.environ.copy();env.update(PYTHONPATH='/tmp/dmpc-p0-deps',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 for command in [('certify_preflight.py',),('register.py',),('stage_driver.py','first32'),('stage_driver.py','remaining'),('stage_driver.py','accept'),('stage_driver.py','delivery')]:
  code=subprocess.run([sys.executable,str(R/command[0]),*command[1:]],env=env).returncode
  if code:
   print(json.dumps({'event':'failed','layer':command,'exitcode':code,'evidence':str(R/'results/stage_events.jsonl')}),flush=True);sys.exit(code)
 print(json.dumps({'event':'experiment_complete','GATE':str(R/'GATE.json')}),flush=True)
