#!/usr/bin/env python3
"""Bounded stages, strict audit gates, content-checked restart; no model polling required."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from run_entry_bicycle import write
from run_m2b_jacobi import snapshot


def drive(root):
    scripts=Path(__file__).parent;version=snapshot(root)
    preflight=[('rename_prefix','P-J','rename',80),('position_prefix','P-J','position',80)]
    formal=[('S-J_1','S-J','main',None),('P-J_1','P-J','main',None),('P-J_2','P-J','main',None),('S-J_2','S-J','main',None),('rename_full','P-J','rename',None),('position_full','P-J','position',None)]
    status=dict(source_version=version,started_wall=time.time(),stages=[],completed=False)
    for index,(name,backend,variant,limit) in enumerate(preflight+formal):
        if snapshot(root)!=version:raise ValueError('source changed during frozen study')
        if index==len(preflight):
            frozen=dict(source_version=version,max_rounds=1,simulation_limit_s=180.,wall_watchdog_s=1800.,formal_order=[q[0] for q in formal],variants={'rename':{'A1':'west_head','A2':'west_tail','B1':'east_head','B2':'east_tail','start_order':['B2','B1','A2','A1']},'position':'each initial x := original x - physical direction * 0.1m'},stopping='stop stage on any failure; never increase rounds/budget/tolerance; full audit must pass; no overlap between online and audit')
            if (root/'formal_freeze.json').exists():
                if json.loads((root/'formal_freeze.json').read_text())!=frozen:raise ValueError('frozen settings mismatch')
            else:write(root/'formal_freeze.json',frozen)
        out=root/name;checkpoint=root/(name+'.checkpoint.json');begin=time.monotonic()
        if checkpoint.exists():
            old=json.loads(checkpoint.read_text())
            for relative,digest in old['sha256'].items():
                if hashlib.sha256((out/relative).read_bytes()).hexdigest()!=digest:raise ValueError('checkpoint bytes changed '+name)
            if old['source_version']!=version:raise ValueError('checkpoint source mismatch')
            status['stages'].append(old);write(root/'study_status.json',status);continue
        if out.exists():raise ValueError('incomplete stage retained; inspect before new attempt '+str(out))
        print('START',name,'backend',backend,'variant',variant,flush=True)
        cmd=[sys.executable,str(scripts/'run_m2b_jacobi.py'),'--backend',backend,'--variant',variant,'--max-rounds','1','--output',str(out)]
        if limit:cmd+=['--max-cycles',str(limit)]
        with (root/(name+'.log')).open('w') as log:
            online=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=1900)
            if online.returncode:raise RuntimeError(f'online exit={online.returncode}; evidence={root/(name+".log")}')
            audit=subprocess.run([sys.executable,str(scripts/'verify_m2b.py'),'--run',str(out/'r1')],stdout=log,stderr=subprocess.STDOUT,timeout=900)
            if audit.returncode:raise RuntimeError(f'audit exit={audit.returncode}; evidence={root/(name+".log")}')
        summary=json.loads((out/'r1/summary.json').read_text());validation=json.loads((out/'r1/independent_validation.json').read_text())
        passed=validation['actual_prefix_acceptance'] and ((limit and summary['committed_cycles']==limit and summary['failure_layer']=='prefix_limit') or (not limit and validation['actual_full_acceptance']))
        row=dict(name=name,backend=backend,variant=variant,source_version=version,passed=bool(passed),wall_s=time.monotonic()-begin,online_exit=online.returncode,audit_exit=audit.returncode,summary=summary,validation=str(out/'r1/independent_validation.json'),sha256={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file()})
        write(checkpoint,row);status['stages'].append(row);write(root/'study_status.json',status)
        print('END',name,'passed',bool(passed),'wall_s',round(row['wall_s'],2),'failure',summary['failure'],flush=True)
        if not passed:raise RuntimeError('stage failed; preserved '+str(out))
    status['completed']=True;status['ended_wall']=time.time();write(root/'study_status.json',status);print('STUDY_COMPLETE',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args()
    try:drive(a.root)
    except Exception as e:
        write(a.root/'driver_failure.json',dict(error=type(e).__name__+': '+str(e),wall_time=time.time()));print('STUDY_FAILED',str(e),flush=True);sys.exit(1)
