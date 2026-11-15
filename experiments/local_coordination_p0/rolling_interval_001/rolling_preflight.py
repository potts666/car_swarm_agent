"""Entire anchor interval, same all-car model path; assemble first phase samples."""
import gzip,time,traceback,csv,shutil
from common import *
from plan_io import load,save,restore,record
from rolling_model import generate,classify,mode
from contracts import binding
from local_mpc import rollout,check_receiver
from transport import exchange
from audit import audit

def interval():
 events=read(HERE/'results/E-anchor_events.json');first=next(e for e in events if e['transaction']['accepted'] and e['meta']['kind']=='internal');target=first['meta']['incoming'];second=next(e for e in events if e['transaction']['accepted'] and e['meta']['outgoing']==target);successor=second['meta']['incoming'];rows={}
 with gzip.open(HERE/'results/E-anchor_actual.jsonl.gz','rt') as f:
  for line in f:r=json.loads(line);rows[r['cycle']]=r
 p=load(second['candidate']);movement=next(k for k in range(second['cycle'],p['B0']) if abs(H*(2*F(rows[k]['v'][successor])+H*F(p['controls'][successor][k][0]))/2)>=F(1,50));return {'target':target,'predecessor':first['meta']['outgoing'],'successor':successor,'start':first['cycle'],'anchor_clear':second['cycle'],'successor_effective_motion':movement,'end_exclusive':movement+32,'max_cycles':512,'max_attempts':2048},events,rows

def run():
 assert not read(HERE/'budget_ledger.json');reg,events,states=interval();write(HERE/'interval_registration.json',reg)
 p=load('results/initial_bound.json.gz');ei=0;rows=[];samples=set();failures=[];started=time.monotonic();seen_hist=False
 # Fresh reconstructed references independently replayed before semantic historical read.
 for n in ('R-fixed','E-anchor'):audit(load(f'results/{n}_final_plan.json.gz'),trace=str(HERE/f'results/{n}_actual.jsonl.gz'),output=HERE/f'results/{n}_independent.json')
 historical=HERE.parent/'slack_release_001/results/E-anchor_final_plan.json.gz';import json as jj
 old=jj.loads(gzip.decompress(historical.read_bytes()));fresh=load('results/E-anchor_final_plan.json.gz');assert all(old[q]==fresh[q] for q in ('controls','owners','per_car_policies'))
 lineage=read(HERE/'input_lineage.json');lineage.update(rolling_historical_analytic_comparison={'path':str(historical.resolve()),'sha':sha(historical),'after_fresh_rebuild_and_both_independent_replays':True,'semantic_equal':True},historical_numeric_input=False);write(HERE/'input_lineage.json',lineage)
 for k in range(reg['start'],reg['end_exclusive']):
  while ei<len(events) and events[ei]['cycle']<=k:
   if events[ei]['transaction']['accepted']:p=load(events[ei]['candidate'])
   ei+=1
  state=restore(states[k]);cycle=[]
  for c in CARS:
   begin=time.monotonic();entry={'cycle':k,'actor':c,'mode':mode(p,c,k),'source_history':state['history']}
   try:
    job,cert=generate(p,state,c);binding(job,p,state);kind,proof=classify(p,state,c,job);base=rollout(job,state,[[F(job['reference'][2*j]),F(job['reference'][2*j+1])] for j in range(32)]);rec=check_receiver(job,base)
    key=(c,entry['mode'],kind)
    if key not in samples:
     stem=HERE/f'results/preflight_samples/k{k}_{c}';stem.parent.mkdir(parents=True,exist_ok=True);assembled,meta=exchange(job,'assemble',stem);save(str(stem.relative_to(HERE))+'_job.json.gz',job);samples.add(key);entry['assembly']=assembled['assembly']
    entry.update(classification=kind,proof=proof,receiver=rec,model_id=job['model_id'],passed=True)
   except Exception as error:
    entry.update(passed=False,classification='fallback_after_failure',limitation=repr(error),traceback=traceback.format_exc());failures.append(entry);save(f'results/preflight_failures/k{k}_{c}_source.json.gz',{'plan':p,'state':record(state)})
   entry['wall_seconds']=time.monotonic()-begin;rows.append(entry);cycle.append(entry)
  with (HERE/'results/preflight_cycles.jsonl').open('a') as f:
   for row in cycle:f.write(json.dumps(row,default=str)+'\n')
  write(HERE/'results/preflight_progress.json',{'cycle':k,'end_exclusive':reg['end_exclusive'],'car_cycles':len(rows),'failures':len(failures),'seconds':time.monotonic()-started})
  if (k-reg['start'])%32==0:print('PREFLIGHT',k,'failures',len(failures),flush=True)
 result={'passed':not failures,'interval':reg,'car_cycles':len(rows),'expected_calls':sum(r.get('classification')=='numerical_update' for r in rows),'phase_samples':len(samples),'failures':len(failures),'first_failures':failures[:8],'new_optimizer_calls':0,'wall_seconds':time.monotonic()-started};write(HERE/'results/rolling_preflight.json',result)
 assert not failures,('preflight blocking conditions',len(failures))
 print('PASS entire zero-call interval',reg,result['expected_calls'],flush=True)
if __name__=='__main__':run()
