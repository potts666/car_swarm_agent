"""Independently cover relevant K and a legal handoff; full rejection traces retained."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from accept_case import Independent
from receivers import feedback

reg=read(HERE/'pre_registration.json');out=HERE/'results/rejection';post=read(out/'postsolve_checks.json');results=[]
def physical_digest(path,start=0):
 values=[]
 with gzip.open(path,'rt') as f:
  for r in map(json.loads,f):
   if r['cycle']>=start:values.append(dict(cycle=r['cycle'],actual=r['actual'],b_before=r['b_before'],b_after=r['b_after'],owner=r['owner']))
 return content_id(values)
old_digest=physical_digest(HERE/'results/R-old/execution.jsonl.gz',reg['cycle']);assert read(HERE/'results/R-old/independent_acceptance.json')['passed']
for name in ('old_contract_execution','numeric_physical_transaction_rejection'):
 assert physical_digest(out/(name+'.jsonl.gz'))==old_digest;results.append(dict(branch=name,passed=True,independent_full_physical_replay_reused='R-old',physical_digest=old_digest,steps=2802-reg['cycle'],new_calls=0))
C_digest=physical_digest(HERE/'results/C-new/execution.jsonl.gz',reg['cycle']);assert read(HERE/'results/C-new/independent_acceptance.json')['passed'];assert physical_digest(out/'new_contract_anchor_execution.jsonl.gz')==C_digest
cases=[dict(trace='new_contract_anchor_execution.jsonl.gz',source=reg['actual_source'],reused_full_physical_C_v1=True)]+[x for x in post['cases'] if x['name'] in ('worker_Exception','missing_ACK_independent_block')]
for case in cases:
 source=case['source'];audit=Independent(reg['initial_witness']);audit.state=restore(m,source);audit.p8=copy.deepcopy(audit.state['poses']);count=0;seen_K=[];last_w=None;prefix=[]
 with gzip.open(out/case['trace'],'rt') as f:
  for row in map(json.loads,f):
   s=audit.state;k=s['cycle'];assert row['cycle']==k and row['source']==normal(state_record(s));before=copy.deepcopy(s);w=get(HERE/'objects',row['witness_id']);last_w=w;owner=w['owners'][k]
   if owner!=s['owner']:
    if s['owner']:audit.domain(s['owner'],'G_out')
    if owner:audit.domain(owner,'W_in');assert owner not in s['served'];s['served'].append(owner)
    s['permission_version']+=1
   assert row['permission_version']==s['permission_version']
   for c,kind in w['events'].get(str(k),[]):
    if kind.startswith('generated:'):
     audit.domain(c,kind);cert=get(HERE/'objects',kind.split(':')[1]);audit.inclusion(cert);assert tuple(map(F,w['controls'][c][k]))==feedback(s,c,cert);seen_K.append(dict(actor=c,phase=k,receiver=kind))
   s.update(witness=row['witness_id'],owner=owner);commands={c:tuple(map(F,w['controls'][c][k])) for c in CARS};audit.step(commands,owner,row['actual']);active,moving=audit.progress_check(before,w,commands,row['progress'])
   cid=w.get('activity_contract_id')
   if cid:
    audit.face_check(cid,k)
    for c,quota in get(HERE/'objects',cid)['obligations'].get(str(k),{}).items():assert c in active
   for c,kind in w['events'].get(str(k+1),[]):audit.domain(c,kind)
   assert row['B_after']==max(row['B_before']-1,0);prefix.append(dict(cycle=k,history=row['actual']['history']));count+=1
   if s['cycle']==140:break
 assert audit.state['cycle']==140 and {x['actor'] for x in seen_K}>=set(reg['active'])
 audit.domain('A1','G_in')
 for c in CARS:
  if c!='A1':audit.domain(c,'W_in')
 assert last_w['goal_times']['A1']==797 and max(last_w['goal_times'].values())==2802
 results.append(dict(branch=case['trace'],passed=True,scope='fresh independent replay from actual checkpoint through all relevant generated K and A1 G_in handoff; other cars in legal W_in waiting domains',steps=count,stop_cycle=140,checked_K=seen_K,remaining_B=2662,minimum_road_m=float(audit.road),minimum_pair_m=float(audit.pair),complete_controller_trace_to_goal=True,prefix_digest=content_id(prefix),full_pure_anchor_physics_reused=case.get('reused_full_physical_C_v1',False)))
write(out/'independent_acceptance.json',dict(passed=True,new_calls=0,branches=results));print(json.dumps(results,indent=2))
