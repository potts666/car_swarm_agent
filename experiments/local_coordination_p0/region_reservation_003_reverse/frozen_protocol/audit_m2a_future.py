#!/usr/bin/env python3
"""Optional old centralized future diagnostic, only after an online run is closed."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from linearized_joint import BicycleController
from longitudinal_joint import routes
from run_entry_bicycle import write
from m2a_logs import lines,raw_bytes,write_log


def audit(directory):
    if not (directory/'summary.json').exists():raise ValueError('online run must finish before offline diagnostic')
    scene=json.loads((directory/'config.json').read_text())['scene'];paths=routes(scene)
    checkpoints=lines(directory/'checkpoints.jsonl')
    protected=['actual.csv','events.jsonl','agreements.jsonl','summary.json']
    before={name:hashlib.sha256(raw_bytes(directory/name)).hexdigest() for name in protected}
    results=[];started=time.perf_counter()
    for number,point in enumerate(checkpoints):
        controller=BicycleController(scene,paths,3,reference_warm_start=True)
        for c,data in point['agents'].items():
            a=controller.agents[c]
            for key,value in data['agent'].items():setattr(a,key,np.array(value) if key=='z' else value)
            if data['reference_cache'] is not None:controller.reference_previous[c]=copy.deepcopy(data['reference_cache'])
        begin=time.perf_counter()
        try:
            witness=controller.future_witness();solves=witness.pop('solves')
            write_log(directory/f'offline_central_future_{number}_solves.jsonl',solves)
            results.append(dict(checkpoint=number,cycle=point['cycle'],**witness))
        except (ValueError,TimeoutError) as exc:
            results.append(dict(checkpoint=number,cycle=point['cycle'],failure=type(exc).__name__+': '+str(exc),check={'passed':False},wall_s=time.perf_counter()-begin))
    after={name:hashlib.sha256(raw_bytes(directory/name)).hexdigest() for name in protected}
    if before!=after:raise RuntimeError('offline audit modified online data')
    result=dict(executed=True,diagnostic_identity='M1 centralized B2-C full future from saved M2a checkpoints',
        hypothetical_permissions={'A':False,'B':True},not_local_future_proof=True,not_online_release_gate=True,
        online_records_unchanged=True,input_hashes=before,total_offline_wall_s=time.perf_counter()-started,checkpoints=results)
    write(directory/'centralized_future_diagnostic.json',result)
    print(directory.name,'offline central future',[(r['cycle'],r['check']['passed']) for r in results],flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();audit(args.run)
