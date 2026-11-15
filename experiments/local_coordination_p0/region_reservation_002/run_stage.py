"""Bounded two-source/two-case stage; no repeats or settings scans."""
import os,subprocess,sys,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ENV={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}

def run(layer,args,files,limit):
    checkpoint=ROOT/'results'/(layer+'_checkpoint.json')
    if checkpoint.exists():
        saved=json.loads(checkpoint.read_text());assert saved['exit_code']==0
        for name,digest in saved['files'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest
        print(layer,'content-checked completed stage',flush=True);return
    with (ROOT/'results'/(layer+'.log')).open('x') as stream:
        try:result=subprocess.run([sys.executable,*map(str,args)],env=ENV,stdout=stream,stderr=subprocess.STDOUT,timeout=limit);code=result.returncode
        except subprocess.TimeoutExpired:code=124
    evidence={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.exists()};saved=dict(layer=layer,exit_code=code,files=evidence,log=str(ROOT/'results'/(layer+'.log')));checkpoint.write_text(json.dumps(saved,indent=2));print(json.dumps(saved),flush=True)
    if code:raise SystemExit(code)

def main():
    for case in ('opposite','same_direction'):
        folder=ROOT/'results'/case
        run('execute_'+case,[ROOT/'run_case.py','--case',case],[folder/'summary.json',folder/'events.json',folder/'execution.jsonl.gz'],300)
        assert json.loads((folder/'summary.json').read_text())['passed']
        run('independent_'+case,[ROOT/'accept_case.py','--case',case],[folder/'independent_acceptance.json',folder/'execution.jsonl.gz'],300)
    calls=json.loads((ROOT/'results/solver_calls.json').read_text());assert len(calls)==4 and all(c['completed'] for c in calls)
    print('Four actual solves and both independent continuations completed',flush=True)
if __name__=='__main__':main()
