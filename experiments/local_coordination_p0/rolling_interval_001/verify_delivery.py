"""Portable standard-library verification; --full-replay adds actual physical audit."""
import argparse,gzip
from common import *
from plan_io import load

def run(full=False):
 manifest=read(HERE/'SHA256SUMS.json')
 for name,h in manifest.items():
  p=(HERE/name).resolve();assert p.is_relative_to(HERE.resolve()) and sha(p)==h,('manifest',name)
 summary=read(HERE/'results/R-rolling_summary.json');ledger=read(HERE/'budget_ledger.json');accept=read(HERE/'results/rolling_independent.json');ns=[json.loads(x) for x in (HERE/'results/rolling_cycles.jsonl').read_text().splitlines()];assert len(ns)==summary['cycles'];assert accept['car_cycles']==4*len(ns) and accept['passed'];assert len(ledger)<=2048 and len(ns)<=512;assert len({(r['cycle'],r['actor'],r['source_version']) for r in ledger})==len(ledger)
 for a,b in zip(ns,ns[1:]):assert b['cycle']==a['cycle']+1 and b['source']==a['successor'] and b['parent']==a['node_id']
 for n in ns:
  assert set(n['rows'])==set(CARS);assert content_id({k:v for k,v in n.items() if k!='node_id'})==n['node_id']
  for c,r in n['rows'].items():
   assert r['classification'] in ('numerical_update','unique_control','certified_optimum','fallback_after_failure','goal_hold')
   for field in ('model_object','raw_object','admission_object','obligation_object'):
    if field in r:assert content_id(load(f'results/objects/{r[field]}.json.gz'))==r[field],(n['cycle'],c,field)
 requests=[]
 for p in (HERE/'results/workers').glob('*_request.json.gz'):
  request=json.loads(gzip.decompress(p.read_bytes()))
  if request['mode']=='solve':requests.append(request)
 assert len(requests)==len(ledger),'every solve worker release accounted, including failures'
 cases=[('E-anchor','results/E-anchor_final_plan.json.gz','results/E-anchor_actual.jsonl.gz'),('R-rolling','results/R-rolling_final_plan.json.gz','results/R-rolling_actual.jsonl.gz')]
 for r in read(HERE/'results/rolling_recovery.json')['branches']:cases.append((r['name'],r['plan'],r['full_trace']))
 for name,pn,tn in cases:
  plan=load(pn);history=content_id(TASKS);count=0
  with gzip.open(HERE/tn,'rt') as f:
   for k,line in enumerate(f):
    row=json.loads(line);commands={c:tuple(map(F,plan['controls'][c][k])) for c in CARS};history=content_id({'parent':history,'commands':commands,'owner':plan['owners'][k]});assert row['history']==history and row['cycle']==k+1 and row['commands']=={c:list(map(str,u)) for c,u in commands.items()};count+=1
  assert count==plan['B0']
  if full:
   from audit import audit
   assert audit(plan,trace=str(HERE/tn))['passed']
 print(json.dumps({'passed':True,'mode':'full-replay' if full else 'default','files':len(manifest),'car_cycles':4*len(ns),'formal_attempts':len(ledger),'actual_trajectories':len(cases),'new_optimizer_calls':0}),flush=True)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--full-replay',action='store_true');args=ap.parse_args();run(args.full_replay)
