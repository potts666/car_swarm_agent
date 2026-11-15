#!/usr/bin/env python3
"""Run the isolated P0 conic-anchor checks against reverified archival objects."""
import argparse
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import shutil

from moving_objects import Motion
from socp_anchor import anchor_problem


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rebuild',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('use a new output directory')
    out=args.output;out.mkdir(parents=True)
    base=args.rebuild/'work/NTU_DMPC_Dissertation/graph_certificate'
    assert json.loads((args.rebuild/'summary.json').read_text())['status']=='passed'
    budget=json.loads((args.rebuild/'recomputed_budgets.json').read_text())
    for name,sha in budget['source_sha256'].items():
        assert hashlib.sha256((base/name).read_bytes()).hexdigest()==sha
    graph=json.loads((base/'joint_graph.json').read_text());data=json.loads((base/'motions.json').read_text())
    edge=next(e for e in graph['winning_edges'] if e['source']==graph['initial'] and e['actor']=='A1')
    motion=data[edge['motion']]
    obj=Motion(edge['motion'],motion,budget['source_sha256']['motions.json'],graph['metadata']['stage_radii'])
    cut=1;state=obj.domain(cut);Q=graph['metadata']['scale']
    margin=min(F(motion['road_lower_Q'],Q),
        min(F(motion['peer_gap_lower_Q'][car+':'+edge['source'][i]],Q)-F(1,2)
            for i,car in enumerate(('A1','A2','B1','B2')) if car!=edge['actor']))/2
    P,T=map(F,graph['metadata']['stage_radii'][motion['source']])
    rp=P/4;rpsi=min(T/4,P/(4*129))
    assert rp+128*rpsi<=P and rpsi<=T
    result=dict(motion=edge['motion'],cut=cut,input_object_sha256=budget['source_sha256'],
        receiver_displacement_bound_m=128,
        receiver_subset_condition='rp+128*rpsi<=P; rpsi<=T; centered at exact nominal A1 flow',
        note='domain proof for the exact nominal reference point, not an arbitrary current-state MPC call',cases=[])
    for name in ('socp_anchor.py','moving_objects.py','check_socp.py'):
        shutil.copy2(Path(__file__).with_name(name),out/name)
    try:
        for radius,yaw in ((F(0),F(0)),(rp,rpsi)):
            check=anchor_problem(obj.window(cut),F(state['v']),F(state['delta']),margin,radius,yaw)
            result['cases'].append(check)
            print(check['status'],check['numerical_increment_norm'],flush=True)
        invalid=list(obj.window(cut));invalid[0]=(F(2),F(0))
        try:
            anchor_problem(invalid,F(state['v']),F(state['delta']),margin,rp,rpsi)
        except ValueError:
            result['invalid_anchor_rejected']=True
        else:raise AssertionError('invalid input anchor was accepted')
    except Exception as error:
        result.update(passed=False,error=repr(error),exit_code=1)
        (out/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
        raise
    result.update(passed=True,exit_code=0,controller_integration=False)
    (out/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('isolated SOCP anchor checks passed; numerical plans not executed',flush=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
