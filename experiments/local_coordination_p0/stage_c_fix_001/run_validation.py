"""Bounded stages: content-checked checkpoints; preserve failures and full logs."""
import os,sys,subprocess,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ENV={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
def run(layer,args,evidence):
    checkpoint=ROOT/'results'/(layer+'_checkpoint.json')
    if checkpoint.exists():
        data=json.loads(checkpoint.read_text());assert data['exit_code']==0
        for name,digest in data['evidence'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest
        print(layer,'verified checkpoint',flush=True);return
    with (ROOT/'results'/(layer+'.log')).open('x') as log:
        result=subprocess.run([sys.executable,*map(str,args)],env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    data=dict(layer=layer,exit_code=result.returncode,evidence={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence if p.exists()})
    checkpoint.write_text(json.dumps(data,indent=2));print(json.dumps(data),flush=True)
    if result.returncode:raise SystemExit(result.returncode)
def main():
    for name in ('curve','straight_acceleration'):
        dest=ROOT/'results'/name
        if not dest.exists():run('execute_'+name,[ROOT/'execute_fix.py','--index',ROOT/'base_c/base_b/results/enrichment_001/index.json','--output',dest,'--interface',name,'--method','B-JL','--shift-recert'],[dest/'summary.json',dest/'execution.jsonl.gz'])
        assert json.loads((dest/'summary.json').read_text())['passed']
        run('accept_'+name,[ROOT/'accept_fix.py','--run',dest],[dest/'independent_acceptance.json',dest/'execution.jsonl.gz'])
if __name__=='__main__':main()
