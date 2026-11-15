"""Portable immutable evidence, raw/event/history algebra and full physical replay."""
import argparse,gzip,csv
from common import *
from plan_io import load,restore
from contracts import physical_id

def verify(full=False):
 manifest=read(HERE/'SHA256SUMS.json')
 for n,h in manifest.items():assert sha(HERE/n)==h,('SHA',n)
 for n,h in read(HERE/'executed_source_versions.json').items():assert sha(HERE/n)==h and sha(HERE/'source_archive/numerical_v1'/n)==h,('numeric source',n)
 for n,h in read(HERE/'acceptance_source_versions.json').items():assert sha(HERE/n)==h,('audit source',n)
 gate=read(HERE/'GATE.json');assert gate['passed'] and all(v['status']=='pass' for v in gate['criteria'].values());calls=read(HERE/'solver_calls.json');assert len(calls)==8 and all(c['status']=='Solved' for c in calls)
 requests=[p for p in (HERE/'results/workers').glob('*_request.json') if read(p)['mode']=='solve'];assert len(requests)==len(calls),'every formal attempt counted, including pre-ready failures'
 from audit_updates import actual_first,release_events,own_check
 from frames import validate_publication,RESERVE
 from quantization import quantize
 plans={};traces={};summaries={};numeric=[]
 for name in ('R-fixed','E-anchor','E-MPC','recovery_missing_ACK','recovery_stale_K'):
  s=read(HERE/f'results/{name}_summary.json');p=load(s['plan']);summaries[name]=s;key=physical_id(p);plans[key]=p;path=s.get('full_trace',s['trace']);traces[key]=HERE/path;history=content_id(TASKS);v={c:F(0) for c in CARS};count=0
  with gzip.open(HERE/path,'rt') as f:
   for k,line in enumerate(f):
    row=json.loads(line);cmd={c:tuple(map(F,p['controls'][c][k])) for c in CARS};history=content_id({'parent':history,'commands':cmd,'owner':p['owners'][k]});assert row['cycle']==k+1 and row['history']==history and row['commands']=={c:list(map(str,u)) for c,u in cmd.items()};count+=1
  assert count==p['B0'];release_events(s)
  dt=load(f'results/{name}_deadline_trace.json.gz')
  for r in dt:assert r['B_after']==max(0,r['B_before']-1) and r['Phi']==sum(r['deadlines'].values())
 assert summaries['E-anchor']['goals']==summaries['E-MPC']['goals'];assert summaries['E-MPC']['final_cycle']<summaries['R-fixed']['final_cycle'];successes=[e for e in summaries['E-MPC']['events'] if e['transaction']['accepted']];assert len(successes)>=2 and {'internal','external'}<={e['meta']['kind'] for e in successes}
 previous={}
 for win in (1,2):
  for rnd in (1,2):
   label=f'e{win}_r{rnd}';e=read(HERE/f'results/numeric/{label}_event.json');p=load(f'results/numeric/{label}_committed.json.gz');cp=load(f'results/numeric/{label}_checkpoint.json.gz');assert actual_first(e,p);plans[physical_id(p)]=p
   if rnd==2:assert e['source']==previous[win]['successor'] and physical_id(cp['plan'])==previous[win]['plan_after']
   previous[win]=e
   for c in e['actors']:
    j=load(f'results/numeric/{label}_{c}_job.json.gz');a=read(HERE/f'results/numeric/{label}_{c}_admission.json');raw=read(HERE/f'results/workers/{label}_{c}_response.json');request=HERE/f'results/workers/{label}_{c}_request.json';assert a['accepted'] and j['objective']['kind']=='energy' and j['arm']=='O-E';assert raw['model_id']==j['model_id']==content_id({k:v for k,v in j.items() if k!='model_id'});assert raw['request_sha']==sha(request) and read(request)['job']==j;assert j['source']['history']==e['source']['history'] and j['source']['poses_Q']==e['source']['poses_Q'][c];assert j['parent']==content_id(cp['plan']['controls'][c]) and j['receiver']['K_id']==content_id(cp['plan']['controls'][c][e['source']['cycle']+32:]);d,q=quantize(raw['x'][:64],F(a['alpha']),j['lattice']);assert d==list(map(F,a['delta'])) and q['integer_coordinates']==a['quantization']['integer_coordinates'];assert raw['assembly']['matrix_id']==content_id(load(f'results/matrices/{j["model_id"]}.json.gz'));numeric.append((j,a,restore(e['source'])))
   assert len(e['mixes'])==4
   for mix in e['mixes']:q=load(mix['plan']);plans[physical_id(q)]=q;assert read(HERE/mix['certificate'])['passed']
 for name in ('recovery_missing_ACK','recovery_stale_K'):
  s=summaries[name];assert not s['events'][0]['transaction']['accepted'] and s['events'][1]['transaction']['accepted'];assert s['events'][1]['cycle']==s['events'][0]['cycle']+1 and s['events'][0]['source']['history']!=s['events'][1]['source']['history'];assert s['illegal_commits']==s['illegal_outputs']==0
 for path in (HERE/'results/frames').glob('*.json'):
  frame=read(path);validate_publication(frame);assert frame['passed'] and all(F(v)>=0 for v in frame['minimum'].values());faces=load(frame['faces'])
  for k,v in faces['objects'].items():assert content_id(v)==k and sum(F(a)**2 for a in v['n'])<=1 and F(v['margin'])==F(v['gap'])-2*RESERVE-SAFE and F(v['margin'])>=0
  assert all(set(row['face_ids'])<=set(faces['objects']) for row in faces['links'])
 for path in (HERE/'results/wire').glob('*.jsonl'):
  for text in path.read_text().splitlines():r=json.loads(text);assert r['bytes']==len(r['wire_json'].encode());msg=json.loads(r['wire_json']);assert msg['sender']==r['sender'] and msg['receiver']==r['receiver']
 with (HERE/'tradeoff.csv').open() as f:
  for row in csv.DictReader(f):assert F(row['delta_F_common_lower'])==F(row['delta_JE_cost_units'])-F(row['delta_s_upper_m']) and F(row['delta_F_common_upper'])==F(row['delta_JE_cost_units'])-F(row['delta_s_lower_m'])
 assert read(HERE/'results/independent_acceptance.json')['passed'] and read(HERE/'results/raw_adoption_certificate.json')['passed'] and read(HERE/'results/directory_union_acceptance.json')['passed']
 if full:
  from audit import audit
  for job,a,state in numeric:own_check(job,a,state)
  for key,p in plans.items():assert audit(p,trace=str(traces[key]) if key in traces else None)['passed'];print('REPLAY',key[:12],p['B0'],flush=True)
 print(json.dumps({'passed':True,'mode':'full-replay' if full else 'default','files':len(manifest),'formal_attempts':len(calls),'unique_complete_physical_plans':len(plans),'new_optimizer_calls':0}),flush=True)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--full-replay',action='store_true');verify(ap.parse_args().full_replay)
