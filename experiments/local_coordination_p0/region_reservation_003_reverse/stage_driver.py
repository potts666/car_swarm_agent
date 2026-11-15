"""Bounded stages, durable completion/failure events, never silently resubmit solves."""
import subprocess,sys,time,json
from common import *

def stage(name,args):
    path=HERE/'results'/(name+'.stage.json');log=HERE/'results'/(name+'.stage.log')
    if path.exists():
        prior=read(path)
        if prior.get('exit_code')==0 and prior.get('completed'):
            checkpoint=read(HERE/'results/independent_checkpoints.json')
            for section in ('code','files'):
                for relative,digest in checkpoint[section].items():assert sha(HERE/relative)==digest,'stale completed stage: '+relative
            print(json.dumps(dict(stage=name,reused=True,content_checked=True)),flush=True);return
        raise RuntimeError('prior failed/incomplete stage retained: '+str(path))
    entry=dict(stage=name,command=args,started=True,completed=False);write(path,entry);started=time.perf_counter()
    with log.open('w') as out:result=subprocess.run([sys.executable,*args],cwd=HERE,stdout=out,stderr=subprocess.STDOUT)
    entry.update(completed=True,exit_code=result.returncode,wall_s=time.perf_counter()-started,evidence=str(log.relative_to(HERE)));write(path,entry);print(json.dumps(entry),flush=True)
    if result.returncode:raise RuntimeError(name+' failed; inspect '+str(log))

def main():
    assert read(HERE/'results/rule_checks.json')['passed']
    for side in ('A','B'):
        stage(side+'_execute',['run_case.py','--case',side])
        stage(side+'_accept',['accept_case.py','--case',side])
if __name__=='__main__':main()
