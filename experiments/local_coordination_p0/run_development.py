#!/usr/bin/env python3
"""Bounded 3 x 5 development runs, each followed by offline acceptance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

CASES=('asymmetric_baseline','tight_curve','exit_merge_dependency')
METHODS=('S-L','S-T','M-L','M-T','M-R')


def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('rebuild','objects','gate','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args();here=Path(__file__).resolve().parent
    if args.output.exists():p.error('preserve prior evidence; use a new output directory')
    gate=json.loads(args.gate.read_text())
    assert gate['p0_passed'] and gate['development_pilot_allowed'] and not gate['formal_batch_allowed']
    args.output.mkdir(parents=True)
    status=dict(development_only=True,formal_batch=False,gate_sha256=digest(args.gate),passed=False,stages=[])
    write(args.output/'status.json',status)
    environment=dict(os.environ,OPENBLAS_NUM_THREADS='1')
    base=['--rebuild',str(args.rebuild),'--objects',str(args.objects)]
    for case in CASES:
        for method in METHODS:
            run=args.output/(case+'_'+method)
            for layer,command in (
                ('controller',[sys.executable,str(here/'run_pilot.py'),*base,'--gate',str(args.gate),'--output',str(run),'--case',case,'--method',method]),
                ('independent_acceptance',[sys.executable,str(here/'accept_execution.py'),*base,'--run',str(run),'--case',case])):
                log=args.output/(case+'_'+method+'_'+layer+'.log');start=time.monotonic()
                row=dict(case=case,method=method,layer=layer,command=command,log=str(log),state='running')
                status['stages'].append(row);write(args.output/'status.json',status)
                with log.open('w') as stream:
                    try:code=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=environment,timeout=900).returncode
                    except subprocess.TimeoutExpired:code=124
                row.update(exit_code=code,wall_s=time.monotonic()-start,state='passed' if code==0 else 'failed')
                evidence=run/('summary.json' if layer=='controller' else 'independent_acceptance.json')
                if code==0:
                    result=json.loads(evidence.read_text());assert result['passed']
                    row['evidence_sha256']=digest(evidence)
                write(args.output/'status.json',status)
                print(case,method,layer,row['state'],code,flush=True)
                if code:return code
    assert len(status['stages'])==30 and all(s['state']=='passed' for s in status['stages'])
    status['passed']=True;write(args.output/'status.json',status)
    return 0


if __name__=='__main__':raise SystemExit(main())
