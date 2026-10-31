#!/usr/bin/env python3
"""Replay frozen stopping QPs with the same reference and original acceptance checks."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from bicycle_core import CARS
import linearized_joint as bj
import longitudinal_joint as lj
from run_entry_bicycle import write


def check(frozen,output):
    output.mkdir(exist_ok=False)
    results=[]
    for path in sorted(frozen.glob('cycle*.json')):
        x=json.loads(path.read_text());scene=x['scene'];ctrl=bj.BicycleController(scene,x['paths'],3,reference_warm_start=True)
        for c,a in ctrl.agents.items():
            for k,v in x['agents'][c].items():setattr(a,k,np.array(v) if k=='z' else v)
        ctrl.reference_previous=x['reference_previous']
        reference=x['record']['reference_record'];N=scene['N']
        acceleration={c:np.array(reference['solution'][i*N:(i+1)*N]) for i,c in enumerate(CARS)}
        initial={c:a.z.copy() for c,a in ctrl.agents.items()}
        with patch.object(lj,'solve',return_value=(acceleration,copy.deepcopy(reference))):
            commands,dense,record=ctrl.predict(x['permissions'])
        assert record['accepted'], (x['cycle'],record['message'])
        trace=[]
        for c in CARS:
            np.testing.assert_array_equal(ctrl.agents[c].z,initial[c])
            states=dense[c][:11];analytic=initial[c][3]+np.arange(11)*scene['sample_s']*commands[c][0][0]
            np.testing.assert_allclose(states[:,3],analytic,atol=1e-14,rtol=0)
            assert not np.any(states[:-1,3]*states[1:,3]<-scene['tolerance']**2)
            trace.append(dict(car=c,initial_speed=float(initial[c][3]),reference_first_acceleration=x['reference_controls'][c][0][0],
                old_bicycle_acceleration=x['commands'][c][0][0],new_bicycle_acceleration=commands[c][0][0],
                integrated_speeds=states[:,3].tolist(),analytic_speeds=analytic.tolist()))
        results.append(dict(cycle=x['cycle'],accepted=record['accepted'],trace=trace,record=record))
    assert len(results)==3
    write(output/'results.json',results)
    write(output/'verification.json',dict(passed=True,frozen_cycles=[r['cycle'] for r in results],
        same_reference=True,no_state_write=True,original_acceptance_retained=True,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(bj.__file__),Path(lj.__file__),Path(__file__)]}))
    print('Three frozen stopping instances passed unchanged acceptance and speed integration checks')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--frozen',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();check(args.frozen,args.output)
