#!/usr/bin/env python3
"""The external acceptor must reject damaged Stage A execution evidence."""
import argparse
import copy
import gzip
import json
from pathlib import Path
import shutil
import tempfile

from accept_interfaces import accept,plan_id


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','rebuild','run','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('preserve prior checks')
    a.output.mkdir(parents=True)
    with gzip.open(a.run/'execution.jsonl.gz','rt') as f:rows=[json.loads(x) for x in f]
    selected=next(i for i,r in enumerate(rows) if r.get('plan',{}).get('branch')=='box_candidate')
    tests=[]
    for kind in ('K','domain','deadline','actual_input','actual_pose','group','inherited_tail'):
        changed=copy.deepcopy(rows)
        if kind in ('K','domain','deadline'):
            r=changed[selected];plan=r['plan']
            if kind=='K':plan['pending_reset']['input'][0]='1'
            if kind=='domain':plan['receiver']['domain']['speed_radius']='1'
            if kind=='deadline':plan['receiver']['deadline']=31
            plan['id']=plan_id(plan)
        if kind=='actual_input':changed[0]['actual']['executed_input'][0]='2'
        if kind=='actual_pose':changed[0]['actual']['actual_pose_Q'][0][0]+=10**20
        if kind=='group':changed[0]['token']['context']['groups']=[['A2','A1','B1','B2']]
        if kind=='inherited_tail':
            r=changed[selected+1];r['plan']['remaining'][0][0]='1';r['plan']['id']=plan_id(r['plan'])
        with tempfile.TemporaryDirectory(prefix='stage-a-accept-damage-') as temp:
            run=Path(temp)
            for name in ('config.json','summary.json'):shutil.copy2(a.run/name,run/name)
            with gzip.open(run/'execution.jsonl.gz','wt') as f:
                for r in changed:f.write(json.dumps(r)+'\n')
            try:accept(a.inputs,a.rebuild,run)
            except (AssertionError,ValueError,KeyError) as error:tests.append(dict(case=kind,rejected=True,error=str(error)))
            else:raise AssertionError('damaged evidence accepted: '+kind)
    (a.output/'checks.json').write_text(json.dumps(dict(passed=True,cases=tests),indent=2)+'\n')
    print('All seven damaged-evidence cases rejected independently.')


if __name__=='__main__':main()
