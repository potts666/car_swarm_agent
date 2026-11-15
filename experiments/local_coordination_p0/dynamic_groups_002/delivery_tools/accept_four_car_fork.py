"""Independent zero-call four-car fork prefix through K and G_in138 handoff."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from accept_case import Independent
from receivers import feedback
from socp_rows import evaluate
f=read(HERE/'results/four_car_fork/summary.json');audit=Independent(read(HERE/'pre_registration.json')['initial_witness']);audit.state=restore(m,f['source']);audit.p8=copy.deepcopy(audit.state['poses']);before=sha(HERE/'solver_calls.json');checked_K=[];activities=[];steps=0
for job in f['zero_jobs']:
 g=get(HERE/'objects',job['generation_id']);assert evaluate(get(HERE/'objects',g['physical_rows_id']),[F(0)]*64)['passed']
with gzip.open(HERE/'results/four_car_fork/execution.jsonl.gz','rt') as stream:
 for row in map(json.loads,stream):
  s=audit.state;k=s['cycle'];assert row['source']==normal(state_record(s));source=copy.deepcopy(s);w=get(HERE/'objects',row['witness_id']);owner=w['owners'][k]
  if owner!=s['owner']:
   if s['owner']:audit.domain(s['owner'],'G_out')
   if owner:audit.domain(owner,'W_in');s['served'].append(owner)
   s['permission_version']+=1
  for c,kind in w['events'].get(str(k),[]):
   if kind.startswith('generated:'):
    audit.domain(c,kind);cert=get(HERE/'objects',kind.split(':')[1]);audit.inclusion(cert);assert tuple(map(F,w['controls'][c][k]))==feedback(s,c,cert);checked_K.append([c,k])
  s.update(witness=row['witness_id'],owner=owner);u={c:tuple(map(F,w['controls'][c][k])) for c in CARS};audit.step(u,owner,row['actual']);A,V=audit.progress_check(source,w,u,row['progress']);audit.face_check(f['contract_id'],k)
  for c,kind in w['events'].get(str(k+1),[]):audit.domain(c,kind)
  for c,quota in get(HERE/'objects',f['contract_id'])['obligations'].get(str(k),{}).items():assert c in A
  activities.append(dict(cycle=k,A=A,V=V,O=[]));steps+=1;assert row['B_after']==max(row['B_before']-1,0)
  if s['cycle']==138:break
assert steps==99 and len(activities[0]['A'])==4 and len(activities[1]['A'])==3 and {c for c,k in checked_K}==set(CARS)
audit.domain('A1','G_in')
for c in CARS:
 if c!='A1':audit.domain(c,'W_in')
assert before==sha(HERE/'solver_calls.json');write(HERE/'results/four_car_fork/independent_acceptance.json',dict(passed=True,new_calls=0,scope='diagnostic fork only; all four zero anchors, independent actual first two activity counts, 99 ticks through four K and G_in138',steps=99,stop_cycle=138,activities=activities,checked_K=checked_K,minimum_road_m=float(audit.road),minimum_pair_m=float(audit.pair),complete_controller_trace_to_goal=True,not_a_fresh_numerical_experiment=True));print(dict(passed=True,first_A=4,second_A=3,independent_steps=99,new_calls=0),flush=True)
