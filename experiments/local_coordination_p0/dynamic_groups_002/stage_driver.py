"""Bounded durable phases; failures retained, completed outputs content-checked."""
import subprocess,sys,time
from common import *

def stage(name,args,outputs):
    path=HERE/'results'/(name+'.stage.json');log=HERE/'results'/(name+'.stage.log')
    if path.exists():
        prior=read(path);assert prior['completed'] and prior['exit_code']==0,'prior failure retained: '+str(path)
        for p,h in prior['outputs'].items():assert sha(HERE/p)==h,'changed result '+p
        for p,h in prior['code'].items():assert sha(HERE/p)==h or sha(HERE/'source_archive'/h/Path(p).name)==h,'missing actual executed code '+p
        print(json.dumps(dict(stage=name,reused=True,content_checked=True)),flush=True);return
    record=dict(layer=name,command=[sys.executable,*args],started=True,completed=False,evidence=str(log.relative_to(HERE)),code={p.name:sha(p) for p in HERE.glob('*.py')});write(path,record);start=time.perf_counter()
    with log.open('w') as f:r=subprocess.run([sys.executable,*args],cwd=HERE,stdout=f,stderr=subprocess.STDOUT)
    record.update(completed=True,exit_code=r.returncode,wall_s=time.perf_counter()-start,outputs={p:sha(HERE/p) for p in outputs if (HERE/p).exists()});write(path,record);small={k:v for k,v in record.items() if k not in ('code','outputs')};print(json.dumps(small),flush=True)
    with (HERE/'results/commands.jsonl').open('a') as f:f.write(json.dumps(small)+'\n')
    if r.returncode:raise RuntimeError(f'{name}: exit {r.returncode}; evidence {log}')

def main():
    assert read(HERE/'results/protocol_checks.json')['passed'] and read(HERE/'results/generation_regression.json')['passed']
    manifest={p.name:sha(p) for p in HERE.glob('*.py')};path=HERE/'executed_source_versions.json'
    if path.exists():
        for name,h in read(path).items():
            assert sha(HERE/name)==h or sha(HERE/'source_archive'/h/Path(name).name)==h,'missing actual executed source '+name
    else:write(path,manifest)
    for arm in ('R-old','C-new','L-new'):
        stage(arm+'_execute',['run_case.py','--arm',arm],[f'results/{arm}/execution.jsonl.gz',f'results/{arm}/summary.json'])
        stage(arm+'_accept',['accept_case.py','--arm',arm],[f'results/{arm}/independent_acceptance.json'])
if __name__=='__main__':main()
