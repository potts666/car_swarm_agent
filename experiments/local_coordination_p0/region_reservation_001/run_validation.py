"""Bounded independent stages; do not rerun the completed numerical controllers."""
import os,sys,subprocess,json,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent
ENV={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
def main():
    for name in ('opposite','same_direction'):
        run=HERE/'results'/name;assert json.loads((run/'summary.json').read_text())['passed']
        dest=run/'independent_acceptance.json'
        if dest.exists():assert json.loads(dest.read_text())['passed'];print(name,'verified saved independent acceptance',flush=True);continue
        logfile=HERE/'results'/(name+'_accept.log')
        with logfile.open('x') as log:
            try:result=subprocess.run([sys.executable,str(HERE/'accept_case.py'),'--case',name],env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=300);code=result.returncode
            except subprocess.TimeoutExpired:code=124
        record=dict(layer='independent_'+name,exit_code=code,evidence_path=str(dest),log_path=str(logfile));(HERE/'results'/('completion_'+name+'.json')).write_text(json.dumps(record,indent=2));print(json.dumps(record),flush=True)
        if code:raise SystemExit(code)
    print('Independent completion accepted for both cases; no solver calls',flush=True)
if __name__=='__main__':main()
