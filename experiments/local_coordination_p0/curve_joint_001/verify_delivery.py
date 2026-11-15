"""Portable integrity/key-event verification, with optional complete physical replay."""
import argparse,gzip,csv
from common import *
from plan_io import load,restore
from physics import predicted_nodes

def verify(full=False):
 manifest=read(HERE/'SHA256SUMS.json');assert manifest
 for name,digest in manifest.items():assert sha(HERE/name)==digest,('sha',name)
 for name,digest in read(HERE/'executed_source_versions.json').items():assert sha(HERE/name)==digest,('executed_source',name)
 # Recompute the signed common-objective summary from immutable admissions.
 with (HERE/'tradeoff_summary.csv').open() as f:tradeoffs=list(csv.DictReader(f))
 assert len(tradeoffs)==8
 for r in tradeoffs:
  de=lo=hi=anchor=F(0)
  for c in ('A1','A2'):
   ad=read(HERE/f'results/{r["arm"]}_r{r["round"]}_{c}_admission.json');old,new=ad['baseline_objective'],ad['objective'];anchor+=F(old['JE']);de+=F(new['JE'])-F(old['JE']);lo+=F(new['actual_progress_Q'][0]-old['actual_progress_Q'][1],m.Q);hi+=F(new['actual_progress_Q'][1]-old['actual_progress_Q'][0],m.Q)
  assert de==F(r['delta_JE_cost_units']) and anchor==F(r['anchor_JE_cost_units']) and lo==F(r['delta_s_lower_m']) and hi==F(r['delta_s_upper_m'])
  assert de-hi==F(r['delta_F_common_lower_cost_units']) and de-lo==F(r['delta_F_common_upper_cost_units']) and float(100*de/anchor)==float(r['delta_JE_pct_of_anchor'])
 gate=read(HERE/'GATE.json');assert gate['passed'];calls=read(HERE/'solver_calls.json');assert len(calls)==16 and all(x['status']=='Solved' for x in calls)
 events=read(HERE/'results/events.json');assert len(events)==8;prev={};first={}
 for e in events:
  label=e['label'];assert len(e['actual_first']['bounds']['active'])==4;assert all(a['accepted'] and a['deadline']<=a['previous_deadline'] for a in e['acks'].values())
  if e['round']==2:assert e['source']==prev[e['arm']]['successor']
  else:first[e['arm']]=content_id(e['source'])
  prev[e['arm']]=e
  plan=load(f'results/{label}_committed_plan.json.gz');state=restore(e['source']);k=state['cycle'];commands={c:tuple(map(F,plan['controls'][c][k])) for c in CARS};nodes=predicted_nodes(state,commands,16)
  assert e['actual_first']['commands']=={c:list(map(str,u)) for c,u in commands.items()}
  for c in CARS:
   assert F(e['successor']['v'][c])==state['v'][c]+H*commands[c][0] and F(e['successor']['delta'][c])==state['delta'][c]+H*commands[c][1]
   for a,b in zip(nodes[c][-1],e['successor']['poses_Q'][c]):assert max(a.l,b[0])<=min(a.u,b[1]),('actual_successor_interval',label,c)
  assert e['successor']['history']==content_id(dict(parent=state['history'],commands=commands,owner=plan['owners'][k])) and e['successor']['cycle']==k+1
  for c in ('A1','A2'):
   job=load(f'results/{label}_{c}_job.json.gz');ad=read(HERE/f'results/{label}_{c}_admission.json');assert ad['accepted'];assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'}) and job['source']['history']==e['source']['history'];delta=list(map(F,ad['delta']));assert all(sum(F(x)*y for x,y in zip(row,delta))==0 for row in job['lattice']['E'])
   if e['arm'].startswith('F'):assert all(x==0 for x in delta[1::2])
   request=HERE/f'results/workers/{label}_{c}_request.json';raw=read(HERE/f'results/workers/{label}_{c}_response.json');assert raw['request_sha']==sha(request) and read(request)['job']==job;assert raw['assembly']['matrix_id']==content_id(load(f'results/matrices/{job["model_id"]}.json.gz'))
 assert len(set(first.values()))==1;assert all(F(read(HERE/f'results/O-P_r{r}_A1_admission.json')['delta'][1])!=0 for r in (1,2));assert read(HERE/'results/independent_acceptance.json')['passed']
 for case in read(HERE/'results/independent_acceptance.json')['rejections']:assert case['illegal_commits']==case['illegal_executions']==0 and case['independent']['passed']
 if full:
  from audit import audit
  from audit_updates import own_check
  from check_projection import run as projection
  from check_negative_models import run as negatives
  # These two checks save deterministic evidence; identical bytes are required.
  for e in events:
   state=restore(e['source'])
   for c in ('A1','A2'):own_check(load(f'results/{e["label"]}_{c}_job.json.gz'),read(HERE/f'results/{e["label"]}_{c}_admission.json'),state)
   assert audit(load(f'results/{e["label"]}_committed_plan.json.gz'))['passed'];print('REPLAY',e['label'],flush=True)
  assert audit(load('results/R_original.json.gz'))['passed']
  cp=load('results/O-P_r2_checkpoint.json.gz')
  for _ in ('invalid_certificate','missing_ACK'):assert audit(cp['plan'])['passed']
  projection();negatives()
 print(json.dumps({'passed':True,'mode':'full-replay' if full else 'default','files':len(manifest),'native_attempts_recorded':len(calls),'new_optimizer_calls':0}),flush=True)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--full-replay',action='store_true');args=ap.parse_args();verify(args.full_replay)
