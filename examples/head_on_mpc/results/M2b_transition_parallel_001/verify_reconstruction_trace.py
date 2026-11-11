"""Restore every baseline queue using only packaged actual inputs/public reports."""
import copy,gzip,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
version=(ROOT/'current_source_version.txt').read_text().strip();sys.path.insert(0,str(ROOT/'sources'/version/'scripts'));sys.path.append(str(ROOT/'supplemental_sources/scripts'))
from m2b_scene import Agent,CARS
from m2b_control import commit_agent
from m2b_protocol import public_report,canonical
scene=json.loads((ROOT/'S-J_1/r1/config.json').read_text())['scene'];routes=json.loads((ROOT/'S-J_1/routes.json').read_text());agents={c:Agent(c,routes[c],scene) for c in CARS};selected={}
with gzip.open(ROOT/'reconstruction/cases.jsonl.gz','rt') as f:
 for line in f:
  r=json.loads(line);selected[r['cycle'],r['car']]=r
count=0;cases=0
with gzip.open(ROOT/'reconstruction/baseline_trace.jsonl.gz','rt') as f:
 for line in f:
  row=json.loads(line);cycle=row['actual']['cycle'];a=row['actual'];ctx=next(iter(a['reports'].values()))['context']
  for c in CARS:
   w=copy.deepcopy(agents[c]);snap=row['snapshots'];w.advance(snap[c]['context']['owner']==c,{p:r for p,r in snap.items() if p!=c})
   if ctx['conditional']:w=copy.deepcopy(agents[c]);w.advance(ctx['owner']==c,{p:r for p,r in row['active_reports'].items() if p!=c})
   assert public_report(w,ctx)==a['reports'][c],(cycle,c,'public stage mismatch')
   parameters=w.parameters(ctx['owner']==c);assert parameters==row['reference'][c]['parameters'][c],(cycle,c,'reference mismatch')
   if (cycle,c) in selected:
    case=selected[cycle,c];assert canonical(w.steering)==canonical(case['steering_queue']);assert parameters==case['parameters'];cases+=1
   commit_agent(w,[a['controls'][c]],a['states'][c]);agents[c]=w
  count+=1
assert cases==len(selected), (cases,len(selected))
result=dict(passed=True,restored_cycles=count,matched_cases=cases,source='packaged actual controls and public snapshots; no current optimized final used to infer queues')
(ROOT/'reconstruction/trace_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
