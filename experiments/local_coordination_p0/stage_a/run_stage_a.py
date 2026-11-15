#!/usr/bin/env python3
"""Bounded A-only driver with full logs, immutable runs and explicit failure layer."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('inputs','rebuild','output'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();here=Path(__file__).resolve().parent
    if a.output.exists():p.error('use a new output directory')
    a.output.mkdir(parents=True)
    code={f.name:sha(f) for f in here.glob('*.py')};manifest_sha=sha(a.inputs/'manifest.json')
    status=dict(passed=False,stage='A-interface-only',code_sha256=code,input_manifest_sha256=manifest_sha,stages=[],formal_batch=False,B_connected=False)
    write(a.output/'status.json',status);base=['--inputs',str(a.inputs),'--rebuild',str(a.rebuild)]
    tasks=[]
    for label in ('curve','straight_acceleration'):
        run=a.output/label
        tasks.extend([(label,'controller',[sys.executable,str(here/'run_interfaces.py'),*base,'--output',str(run),'--interface',label]),
            (label,'independent_acceptance',[sys.executable,str(here/'accept_interfaces.py'),*base,'--run',str(run)])])
    tasks.append(('curve','acceptance_damage',[sys.executable,str(here/'test_acceptor_damage.py'),*base,'--run',str(a.output/'curve'),'--output',str(a.output/'acceptance_damage')]))
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1')
    for label,layer,command in tasks:
        oldroot=here.parent;oldgate=json.loads((oldroot/'P0_GATE_v2.json').read_text())
        assert all(sha(oldroot/n)==v for n,v in oldgate['algorithm_sha256'].items())
        assert all(sha(oldroot.parents[1]/n)==v for n,v in oldgate['repository_dependencies_sha256'].items())
        model=a.rebuild/'work/M2b_首段自由度与递归可行性/route_zoh_certificate.py'
        assert sha(model)==oldgate['interval_model_sha256']
        assert code=={f.name:sha(f) for f in here.glob('*.py')} and manifest_sha==sha(a.inputs/'manifest.json'),'frozen stage source changed'
        log=a.output/(label+'_'+layer+'.log');start=time.monotonic();row=dict(interface=label,layer=layer,command=command,log=str(log),state='running')
        status['stages'].append(row);write(a.output/'status.json',status)
        with log.open('w') as stream:
            try:exit_code=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=env,timeout=180).returncode
            except subprocess.TimeoutExpired:exit_code=124
        row.update(exit_code=exit_code,wall_s=time.monotonic()-start,state='passed' if exit_code==0 else 'failed')
        write(a.output/'status.json',status);print(label,layer,row['state'],exit_code,log,flush=True)
        if exit_code:return exit_code
        result=a.output/'acceptance_damage/checks.json' if layer=='acceptance_damage' else a.output/label/('summary.json' if layer=='controller' else 'independent_acceptance.json')
        assert json.loads(result.read_text())['passed'];row['evidence_sha256']=sha(result);write(a.output/'status.json',status)
    status['passed']=True;write(a.output/'status.json',status);return 0


if __name__=='__main__':
    if not __debug__:raise SystemExit('certification requires Python assertions')
    raise SystemExit(main())
