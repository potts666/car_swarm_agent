#!/usr/bin/env python3
"""Bounded B stages; preserve failures and stop on program/inheritance errors."""
import argparse
import os
import subprocess
import sys
import time
from common import *


def main():
    p=argparse.ArgumentParser();p.add_argument('--index',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('new output directory required')
    a.output.mkdir(parents=True);code={f.name:sha(f) for f in HERE.glob('*.py')};dependencies=read(HERE/'preflight.json')['dependencies_sha256']
    status=dict(passed=False,code_sha256=code,input_manifest_sha256=sha(HERE/'inputs/manifest.json'),index_sha256=sha(a.index),stages=[],formal_batch=False)
    write(a.output/'status.json',status);tasks=[]
    for name in ('curve','straight_acceleration'):
        for method in ('B-ZG','B-ZL','B-JL','anchor'):
            label=name+'_'+method;run=a.output/label
            tasks.extend([(label,'controller',[sys.executable,str(HERE/'execute_case.py'),'--index',str(a.index),'--output',str(run),'--interface',name,'--method',method]),(label,'independent_acceptance',[sys.executable,str(HERE/'accept_case.py'),'--run',str(run)])])
    for name in ('curve','straight_acceleration'):
        label=name+'_B-JL-shift';run=a.output/label
        tasks.extend([(label,'controller',[sys.executable,str(HERE/'execute_case.py'),'--index',str(a.index),'--output',str(run),'--interface',name,'--method','B-JL','--shift-recert']),(label,'independent_acceptance',[sys.executable,str(HERE/'accept_case.py'),'--run',str(run)])])
    tasks.append(('certificate_damage','independent_rejections',[sys.executable,str(HERE/'test_rejections.py'),'--index',str(a.index),'--output',str(a.output/'certificate_damage')]))
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1')
    for label,layer,command in tasks:
        assert code=={f.name:sha(f) for f in HERE.glob('*.py')}
        assert all(sha(HERE/n)==v for n,v in dependencies.items())
        log=a.output/(label+'_'+layer+'.log');start=time.monotonic();row=dict(label=label,layer=layer,command=command,log=str(log),state='running')
        status['stages'].append(row);write(a.output/'status.json',status)
        with log.open('w') as stream:
            try:result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=env,timeout=180).returncode
            except subprocess.TimeoutExpired:result=124
        row.update(exit_code=result,wall_s=time.monotonic()-start,state='passed' if result==0 else 'failed');write(a.output/'status.json',status)
        print(label,layer,row['state'],result,flush=True)
        if result:return result
        evidence=a.output/'certificate_damage/checks.json' if layer=='independent_rejections' else a.output/label/('summary.json' if layer=='controller' else 'independent_acceptance.json')
        assert read(evidence)['passed'];row['evidence_sha256']=sha(evidence);write(a.output/'status.json',status)
    status['passed']=True;write(a.output/'status.json',status);return 0


if __name__=='__main__':
    if not __debug__:raise SystemExit('certification needs Python assertions')
    raise SystemExit(main())
