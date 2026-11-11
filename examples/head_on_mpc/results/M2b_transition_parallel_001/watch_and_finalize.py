"""Lightweight status events and automatic postprocessing, no model-driven wait loop."""
import json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
started=time.monotonic();last=None
while True:
 state=json.loads((ROOT/'study_status.json').read_text()) if (ROOT/'study_status.json').exists() else {}
 if state.get('completed') or (ROOT/'driver_failure.json').exists():break
 lines=(ROOT/'driver.log').read_text().splitlines();stage=lines[-1] if lines else 'starting';name=stage.split()[1] if stage.startswith('START ') else None
 progress=(ROOT/(name+'.log')).read_text().splitlines() if name and (ROOT/(name+'.log')).exists() else []
 row=next((line for line in reversed(progress) if line.startswith('r1 cycle ')),None)
 marker=(stage,row)
 if marker!=last:print(time.strftime('%H:%M:%S'),stage,';',row or 'audit/export or phase boundary',flush=True);last=marker
 if time.monotonic()-started>14400:raise TimeoutError('observation watchdog, experiment limits unchanged')
 time.sleep(50)
print('DRIVER_FINISHED',bool(state.get('completed')),flush=True)
env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/tmp/m2b_stage1_repro_deps')
for name in ('additional_fault_checks.py','prediction_metrics.py','make_delivery.py'):
 with (ROOT/(name+'.log')).open('w') as log:
  result=subprocess.run(['/usr/bin/python3',str(ROOT/name)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900)
 print('POSTPROCESS',name,'exit',result.returncode,flush=True)
 if result.returncode:raise RuntimeError('postprocessing failed; see '+str(ROOT/(name+'.log')))
print('FINALIZATION_READY',flush=True)
