#!/usr/bin/env python3
"""Confirm that the offline acceptance rejects deliberately damaged evidence."""
import argparse
import copy
import gzip
import json
from pathlib import Path
import shutil
import tempfile

from accept_execution import accept


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('rebuild','objects','run','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('use a new output directory')
    args.output.mkdir(parents=True)
    with gzip.open(args.run/'execution.jsonl.gz','rt') as f:rows=[json.loads(x) for x in f]
    changed=next(i for i,row in enumerate(rows) if row.get('accepted_controls'))
    results=[]
    for kind in ('input','state','budget','history','receiver','group','accepted_prediction_inputs'):
        bad=copy.deepcopy(rows)
        if kind=='input':bad[0]['actual']['executed_input'][0]='2'
        if kind=='state':bad[0]['actual']['actual_pose_Q'][0][0]+=10**20
        if kind=='budget':bad[0]['budget_after']+=1
        if kind=='history':bad[0]['history']='stale'
        if kind=='receiver':bad[0]['receiver_phase']+=1
        if kind=='group':bad[0]['groups']=[['A1','A2','B1']]
        if kind=='accepted_prediction_inputs':bad[changed]['accepted_controls'][0][0]='2'
        with tempfile.TemporaryDirectory(prefix='dmpc-p0-damaged-') as temporary:
            tmp=Path(temporary);shutil.copy2(args.run/'summary.json',tmp/'summary.json')
            with gzip.open(tmp/'execution.jsonl.gz','wt') as f:
                for row in bad:f.write(json.dumps(row)+'\n')
            try:accept(args.rebuild,args.objects,tmp,'asymmetric_baseline',True)
            except (AssertionError,ValueError) as error:results.append(dict(case=kind,rejected=True,error=str(error)))
            else:raise AssertionError('damaged evidence accepted: '+kind)
    (args.output/'checks.json').write_text(json.dumps(dict(passed=True,cases=results),indent=2)+'\n')
    shutil.copy2(Path(__file__),args.output/Path(__file__).name)
    print('All seven damaged-evidence cases rejected.')


if __name__=='__main__':main()
