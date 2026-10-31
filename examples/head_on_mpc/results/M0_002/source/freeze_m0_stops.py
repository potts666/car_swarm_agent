#!/usr/bin/env python3
"""Freeze M0 crossing states and trace reference/control/RK4 speed, without modifying logs."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from bicycle_core import CARS, rollout
from linearized_joint import BicycleController
import longitudinal_joint as lj
from run_entry_bicycle import jsonl, write


def read(path):return [json.loads(line) for line in path.read_text().splitlines()]


def freeze(source,output):
    output.mkdir(parents=True,exist_ok=False)
    scene=json.loads((source/'config.json').read_text())['scene']
    paths=json.loads((source.parent/'paths.json').read_text())
    solves=read(source/'solves.jsonl');constraints=read(source/'constraints.jsonl');predictions=read(source/'predictions.jsonl')
    states={round(p['time_s'],8):p for p in predictions}
    import csv
    actual=list(csv.DictReader((source/'actual.csv').open()))
    for cycle in (121,242,403):
        record=solves[cycle];now=record['time_s'];ctrl=BicycleController(scene,paths,3,reference_warm_start=True)
        permissions={};agents={}
        for c in CARS:
            constraint=next(r for r in constraints if r['cycle']==cycle and r['car']==c)
            previous=next(r for r in constraints if r['cycle']==cycle-1 and r['car']==c)
            a=ctrl.agents[c];a.z=np.array(states[round(now,8)]['states'][c]);a.index=constraint['segment'];a.mode=constraint['phase'];a.s=constraint['s0']
            a.stop_steps=constraint['stop_index'] if a.mode=='MOVE' and constraint['stop_at_end'] else None
            a.ready=c=='B' and a.index>=3
            a.previous=a.segment['direction']*states[round(now-scene['h'],8)]['commands'][c][0][0] if a.mode=='MOVE' and a.index==previous['segment'] and a.mode==previous['phase'] else 0.
            a.steering=[]
            ctrl.reference_previous[c]=dict(index=previous['segment'],mode=previous['phase'],solution=solves[cycle-1]['reference_record']['solution'][CARS.index(c)*32:(CARS.index(c)+1)*32])
            permissions[c]=bool(constraint['g_i'])
            actual_start=next(r for r in actual if r['car']==c and abs(float(r['time_s'])-now)<1e-8)
            np.testing.assert_allclose(a.z,[float(actual_start[k]) for k in ('x','y','psi','v','delta')],atol=2e-12,rtol=0)
            agents[c]=dict(z=a.z.tolist(),s=a.s,index=a.index,mode=a.mode,previous=a.previous,stop_steps=a.stop_steps,ready=a.ready,steering=a.steering)
        qp=lj.build_qp(ctrl.agents,permissions,scene);assert qp[-1]==record['parameters']
        H,q,A,lo,hi,E,b,bounds,_=qp
        np.savez_compressed(output/f'cycle{cycle}_reference.npz',H=H,q=q,A=A,lo=lo,hi=hi,E=E,b=b,bounds=bounds)
        acceleration={c:np.array(record['reference_record']['solution'][i*32:(i+1)*32]) for i,c in enumerate(CARS)}
        with patch.object(lj,'solve',return_value=(acceleration,copy.deepcopy(record['reference_record']))):
            ref_controls,ref_dense,_=lj.Controller.predict(ctrl,permissions)
        commands=states[round(now,8)]['commands'];dense={c:rollout(ctrl.agents[c].z,commands[c],scene) for c in CARS}
        trace=[]
        for c in CARS:
            for j in range(11):
                trace.append(dict(cycle=cycle,car=c,time_s=round(now+j*scene['sample_s'],10),
                    reference_a=ref_controls[c][0][0],bicycle_a=commands[c][0][0],omega=commands[c][0][1],
                    initial_v=ctrl.agents[c].z[3],reference_v=ref_dense[c][j,3],bicycle_v=dense[c][j,3],
                    analytic_v=ctrl.agents[c].z[3]+j*scene['sample_s']*commands[c][0][0]))
        write(output/f'cycle{cycle}.json',dict(cycle=cycle,scene=scene,paths=paths,agents=agents,permissions=permissions,
            reference_previous=ctrl.reference_previous,record=record,reference_controls=ref_controls,commands=commands,trace=trace))
    hashes={str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}
    write(output/'provenance.json',dict(source=str(source),original_hashes=hashes,
        frozen_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir()}))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();freeze(args.source,args.output)
