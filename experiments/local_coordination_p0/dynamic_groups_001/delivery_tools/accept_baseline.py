import sys,time
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from common import *
from execution import advance
from audit_base import Audit
r=read(HERE/'pre_registration.json');s=initial_state();s['witness']=r['initial_witness'];s['contract_id']=None;a=Audit(s['witness']);a.state['contract_id']=None;a.validate_witness(s['witness']);w=get(HERE/'objects',s['witness']);began=time.perf_counter();energy=F(0)
with gzip.open(HERE/'results/baseline_execution.jsonl.gz','wt') as f:
    for k in range(w['ticks']):
        row=advance(s,s['witness']);commands={c:tuple(map(F,w['controls'][c][k])) for c in CARS};owner=w['owners'][k]
        if owner!=a.state['owner']:
            if a.state['owner']:a.domain(a.state['owner'],'G_out')
            if owner:a.domain(owner,'W_in');a.state['served'].append(owner)
            a.state['permission_version']+=1
        a.state.update(owner=owner);a.step(commands,owner,row['actual'])
        for c,kind in w['events'].get(str(k+1),[]):a.domain(c,kind)
        energy+=H*sum(x*x for u in commands.values() for x in u);f.write(json.dumps(row,separators=(',',':'))+'\n')
for c in CARS:a.domain(c,'target')
assert s['history']==a.state['history'];write(HERE/'results/baseline_independent_acceptance.json',dict(passed=True,steps=w['ticks'],new_calls=0,source_witness=r['initial_witness'],historical_provenance='baseline_provenance.json',control_input_cost=str(energy),minimum_road_m=float(a.road),minimum_pair_m=float(a.pair),wall_s=time.perf_counter()-began,shared=['interval arithmetic and ZOH'],independent=['8-subsegment road, bodies and all six pairs','resource calendar and domain membership','full command history and zero speed/steering at goal']))
print('baseline independent passed',flush=True)
