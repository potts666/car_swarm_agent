"""Four actual solves total; bounded stages, complete logs and checked checkpoints."""
import os,sys,subprocess,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ENV={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
def run(layer,args,evidence):
    checkpoint=ROOT/'results'/(layer+'_checkpoint.json')
    if checkpoint.exists():
        record=json.loads(checkpoint.read_text());assert record['exit_code']==0
        for name,digest in record['evidence'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest
        for name,digest in record['code'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'checkpoint code changed: '+name
        print(layer,'verified checkpoint',flush=True);return
    sources={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in ROOT.glob('*.py')}
    with (ROOT/'results'/(layer+'.log')).open('x') as log:
        result=subprocess.run([sys.executable,*map(str,args)],env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    record=dict(layer=layer,exit_code=result.returncode,code=sources,evidence={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in evidence if p.exists()})
    checkpoint.write_text(json.dumps(record,indent=2));print(json.dumps({k:v for k,v in record.items() if k!='code'}),flush=True)
    if result.returncode:raise SystemExit(result.returncode)
def main():
    (ROOT/'results').mkdir(exist_ok=True)
    for name in ('curve','straight_acceleration'):
        dest=ROOT/'results'/name
        run('execute_'+name,[ROOT/'execute_fix.py','--index',ROOT/'base_c/base_b/results/enrichment_001/index.json','--output',dest,'--interface',name,'--method','B-JL','--shift-recert'],[dest/'summary.json',dest/'events.json',dest/'execution.jsonl.gz'])
        assert json.loads((dest/'summary.json').read_text())['passed']
        run('accept_'+name,[ROOT/'accept_fix.py','--run',dest],[dest/'independent_acceptance.json',dest/'execution.jsonl.gz'])
    ledger=json.loads((ROOT/'results/solver_calls.json').read_text());assert len(ledger)==4 and all(x['completed'] for x in ledger)
    print('Four solver calls and two independent acceptances completed',flush=True)
if __name__=='__main__':main()
