"""Bounded durable stages. Completed stages are reused only after content checks."""
import subprocess,sys,time,os
from common import *

def stage(name,args,outputs):
    manifest=HERE/'results'/(name+'.stage.json');log=HERE/'results'/(name+'.stage.log')
    if manifest.exists():
        record=read(manifest);assert record['completed'] and record['exit_code']==0,'failed/incomplete stage retained: '+str(manifest)
        for rel,digest in record['outputs'].items():assert sha(HERE/rel)==digest,'stale completed output: '+rel
        for rel,digest in record['code'].items():assert sha(HERE/rel)==digest or sha(HERE/'source_archive'/digest/Path(rel).name)==digest,'missing original code for reused stage: '+rel
        print(json.dumps(dict(stage=name,reused=True,content_checked=True)),flush=True);return
    record=dict(layer=name,command=[sys.executable,*args],started=True,completed=False,evidence=str(log.relative_to(HERE)),code={str(p.relative_to(HERE)):sha(p) for p in HERE.glob('*.py')});write(manifest,record);began=time.perf_counter()
    with log.open('w') as stream:result=subprocess.run([sys.executable,*args],cwd=HERE,stdout=stream,stderr=subprocess.STDOUT)
    record.update(completed=True,exit_code=result.returncode,wall_s=time.perf_counter()-began,outputs={p:sha(HERE/p) for p in outputs if (HERE/p).exists()});write(manifest,record);print(json.dumps({k:v for k,v in record.items() if k not in ('code','outputs')}),flush=True)
    with (HERE/'results/commands.jsonl').open('a') as stream:stream.write(json.dumps({k:v for k,v in record.items() if k not in ('code','outputs')})+'\n')
    if result.returncode:raise RuntimeError(f'{name}: exit={result.returncode}; evidence={log}')

def main():
    assert read(HERE/'pre_registration.json')['passed'];assert read(HERE/'results/protocol_checks.json')['passed'];assert read(HERE/'results/generation_regression.json')['passed'];assert read(HERE/'results/terminal_hold.json')['passed']
    versions={str(p.relative_to(HERE)):sha(p) for p in HERE.rglob('*.py') if '__pycache__' not in str(p) and 'source_archive' not in p.parts}
    path=HERE/'executed_source_versions.json'
    if path.exists():
        for rel,digest in read(path).items():
            assert sha(HERE/rel)==digest or sha(HERE/'source_archive'/digest/Path(rel).name)==digest,'missing executed source: '+rel
    else:write(path,versions)
    for arm in ('F-S','F-P','D-P'):
        stage(arm+'_execute',['run_case.py','--arm',arm],[f'results/{arm}/summary.json',f'results/{arm}/execution.jsonl.gz'])
        stage(arm+'_accept_v2',['accept_case.py','--arm',arm],[f'results/{arm}/independent_acceptance.json'])
if __name__=='__main__':main()
