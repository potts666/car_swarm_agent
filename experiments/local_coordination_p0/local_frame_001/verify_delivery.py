"""Portable integrity/protocol/algebra checks; full mode independently replays physics."""
import argparse,csv,gzip
from common import *
from plan_io import load,restore
from frame import physical_id

def verify(full=False):
 manifest=read(HERE/'SHA256SUMS.json')
 for n,h in manifest.items():assert sha(HERE/n)==h,('delivery SHA',n)
 for n,h in read(HERE/'executed_source_versions.json').items():assert sha(HERE/n)==h and sha(HERE/'source_archive/numerical_v1'/n)==h,('numeric source',n)
 for n,h in read(HERE/'acceptance_source_versions.json').items():assert sha(HERE/n)==h,('acceptance source',n)
 gate=read(HERE/'GATE.json');assert gate['passed'] and all(v['status']=='pass' for v in gate['criteria'].values());calls=read(HERE/'solver_calls.json');assert len(calls)==12 and all(r['status']=='Solved' for r in calls);events=read(HERE/'results/events.json');assert len(events)==4;previous={};plans={};rawstates=[]
 from audit_updates import actual_successor
 for e in events:
  label=e['label'];org=e['organization'];p=load(f'results/{label}_committed.json.gz');cp=load(f'results/{label}_checkpoint.json.gz');assert p['outer_frame_id']==cp['old']['outer_frame_id']==read(HERE/'outer_frame.json')['outer_frame_id'];assert actual_successor(e,p);assert len(e['actual_first']['bounds']['active'])==4;assert e['transaction']['X_committed'] and e['transaction']['D_committed'];assert set(e['transaction']['ACK'])==set(CARS if org=='C' else ['A1','B1','A2']);assert e['transaction']['external_ack_still_valid']
  ack=e['transaction']['external_ack'];assert ack['outer_frame_id']==p['outer_frame_id'] and ack['per_car_plan_id']==content_id(p['controls']['A2']) and ack['valid_from']<=e['source']['cycle']<=ack['valid_through']
  if e['round']==2:assert e['source']==previous[org]['successor'] and content_id(cp['old'])==previous[org]['new_id']
  previous[org]=e;plans[physical_id(p)]=p
  for c in ('A1','B1','A2'):
   job=load(f'results/{label}_{c}_job.json.gz');ad=read(HERE/f'results/{label}_{c}_admission.json');raw=read(HERE/f'results/workers/{label}_{c}_response.json');request=HERE/f'results/workers/{label}_{c}_request.json';assert ad['accepted'] and job['arm']=='O-E' and job['objective']['kind']=='energy';assert job['parent']==content_id(cp['anchor']['controls'][c]) and job['source']['history']==e['source']['history'];assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'});assert raw['request_sha']==sha(request) and read(request)['job']==job and raw['model_id']==job['model_id'];assert raw['assembly']['matrix_id']==content_id(load(f'results/matrices/{job["model_id"]}.json.gz'));assert F(ad['delta'][0])!=0 and tuple(map(F,ad['commands'][0]))==tuple(map(F,p['controls'][c][e['source']['cycle']]))
   if org=='L':assert job==load(f'results/C_r{e["round"]}_{c}_job.json.gz')
   rawstates.append((job,ad,restore(e['source'])))
  assert len(e['mixes'])==4 and {(m['D_new'],m['X_new']) for m in e['mixes']}=={(False,False),(False,True),(True,False),(True,True)}
  for mix in e['mixes']:
   mp=load(mix['plan']);assert read(HERE/mix['certificate'])['passed'];plans[physical_id(mp)]=mp
 assert physical_id(load('results/C_final_plan.json.gz'))==physical_id(load('results/L_final_plan.json.gz'))
 post=read(HERE/'results/post_branches.json');assert post['passed'] and set(post['extension']['closure']['participants'])==set(CARS);assert post['extension']['post_frame_independent_blocks']==[['A1','B1'],['A2'],['B2']]
 for e in post['rejections']:
  assert e['external_new_first_executed'] and e['external_changed_output_ticks'] and e['illegal_commits']==e['illegal_executions']==0;assert e['transaction']['X_committed'];p=load(e['plan']);actual_successor(e,p);plans[physical_id(p)]=p
  if e['name']=='D_missing_ACK':assert not e['transaction']['D_committed']
 for e in post['extension']['branches']:
  p=load(e['plan']);plans[physical_id(p)]=p;assert e['new_frame_committed']==(set(e['ACK'])==set(CARS));assert p['outer_frame_id']==read(HERE/('extended_frame.json' if e['new_frame_committed'] else 'outer_frame.json'))['outer_frame_id']
 for path in (HERE/'results/wire').glob('*.jsonl'):
  for line in path.read_text().splitlines():
   r=json.loads(line);assert r['bytes']==len(r['wire_json'].encode());payload=json.loads(r['wire_json']);assert payload['sender']==r['sender'] and payload['receiver']==r['receiver']
 with (HERE/'local_tradeoff.csv').open() as f:
  for row in csv.DictReader(f):
   de=lo=hi=F(0)
   for c in ('A1','B1','A2'):
    ad=read(HERE/f'results/{row["organization"]}_r{row["round"]}_{c}_admission.json');a,b=ad['baseline_objective'],ad['objective'];de+=F(b['JE'])-F(a['JE']);lo+=F(b['actual_progress_Q'][0]-a['actual_progress_Q'][1],m.Q);hi+=F(b['actual_progress_Q'][1]-a['actual_progress_Q'][0],m.Q)
   assert de==F(row['delta_JE_cost_units']) and lo==F(row['delta_s_lower_m']) and hi==F(row['delta_s_upper_m']);assert de-hi==F(row['delta_F_common_lower_cost_units']) and de-lo==F(row['delta_F_common_upper_cost_units'])
 assert read(HERE/'results/independent_acceptance.json')['passed']
 tracecheck=read(HERE/'results/actual_trace_acceptance.json');assert tracecheck['passed']
 for org in ('C','L'):
  data=gzip.decompress((HERE/f'results/actual_{org}.jsonl.gz').read_bytes());assert hashlib.sha256(data).hexdigest()==tracecheck['full_trace_uncompressed_sha'][org];p=load(f'results/{org}_final_plan.json.gz');rows=data.splitlines();assert len(rows)==p['B0'];history=content_id(TASKS)
  for k,line in enumerate(rows):
   row=json.loads(line);commands={c:tuple(map(F,p['controls'][c][k])) for c in CARS};history=content_id({'parent':history,'commands':commands,'owner':p['owners'][k]});assert row['history']==history and row['cycle']==k+1 and row['commands']=={c:list(map(str,u)) for c,u in commands.items()}
 assert read(HERE/'results/effective_activity_certificate.json')['passed']
 if full:
  from audit import audit
  from audit_updates import own_check,trace_inclusion
  for job,ad,state in rawstates:own_check(job,ad,state)
  for name in ('old','new'):p=load(f'results/{name}.json.gz');plans[physical_id(p)]=p
  for key,p in plans.items():assert audit(p)['passed'];print('REPLAY',key[:12],p['B0'],flush=True)
  for e in events:
   for mix in e['mixes']:trace_inclusion(load(mix['plan']),read(HERE/mix['certificate']))
  for e in post['rejections']+post['extension']['branches']:trace_inclusion(load(e['plan']),read(HERE/e['certificate']))
 print(json.dumps({'passed':True,'mode':'full-replay' if full else 'default','files':len(manifest),'formal_attempts':len(calls),'unique_physical_plans':len(plans),'new_optimizer_calls':0}),flush=True)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--full-replay',action='store_true');args=ap.parse_args();verify(args.full_replay)
