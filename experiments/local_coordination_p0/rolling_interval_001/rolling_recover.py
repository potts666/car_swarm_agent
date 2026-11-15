"""Fault branches from genuine main checkpoints, no worker/optimizer import."""
import copy,gzip
from common import *
from plan_io import load,save,restore,record
from engine import run
from contracts import binding,physical_id

def objects(key):return load(f'results/objects/{key}.json.gz')
def nodes():return [json.loads(s) for s in (HERE/'results/rolling_cycles.jsonl').read_text().splitlines()]
def plan_at(cycle):
 reg=read(HERE/'interval_registration.json');event=next(e for e in read(HERE/'results/E-anchor_events.json') if e['cycle']==reg['start']);p=load(event['candidate'])
 for n in nodes():
  if n['release'] and n['release']['transaction']['accepted']:p=load(n['release']['candidate'])
  assert physical_id(p)==n['plan_before']
  if n['cycle']==cycle:return p
  for c,r in n['rows'].items():
   if r['adopted']:p['controls'][c][n['cycle']:n['cycle']+32]=objects(r['admission_object'])['commands']
  assert physical_id(p)==n['plan_after']
 raise ValueError(cycle)

def complete(name,plan,source,fault=None):
 r=run(name,'anchor',{'plan':plan,'state':source},fault);prefix=[]
 with gzip.open(HERE/'results/R-rolling_actual.jsonl.gz','rb') as f:
  for line in f:
   if json.loads(line)['cycle']>source['cycle']:break
   prefix.append(line)
 trace=f'results/{name}_full_actual.jsonl.gz';data=b''.join(prefix)+gzip.decompress((HERE/r['trace']).read_bytes());(HERE/trace).write_bytes(gzip.compress(data,mtime=0));assert len(data.splitlines())==r['final_cycle'];r['full_trace']=trace;write(HERE/f'results/{name}_summary.json',r);return r

def run_all():
 ledger=sha(HERE/'budget_ledger.json');ns=nodes();reg=read(HERE/'interval_registration.json');eligible=[n for n in ns if sum(r['adopted'] for r in n['rows'].values())>=2];assert eligible,'no genuine two-adopted checkpoint available for raw/K fault';boundaries=[b['cycle'] for a,b in zip(ns,ns[1:]) if any(a['rows'][c]['mode']!=b['rows'][c]['mode'] for c in CARS)];n=next((x for x in eligible if len(x['actual_activity'])>=2 and any(0<=x['cycle']-b<=2 for b in boundaries) and sum(r.get('first_a_changed',False) or r.get('first_omega_changed',False) for r in x['rows'].values())>=2),eligible[0]);p=plan_at(n['cycle']);chosen=next(c for c,r in n['rows'].items() if r['adopted']);job=objects(n['rows'][chosen]['model_object']);bad=copy.deepcopy(job);bad['receiver']['K_id']='damaged_actual_K';bad.pop('model_id');bad['model_id']=content_id(bad)
 try:binding(bad,p,restore(n['source']))
 except AssertionError:pass
 else:raise AssertionError('corrupted K passed')
 selected=copy.deepcopy(p);other=[]
 for c,row in n['rows'].items():
  if row['adopted'] and c!=chosen:selected['controls'][c][n['cycle']:n['cycle']+32]=objects(row['admission_object'])['commands'];other.append(c)
 from wire import Wire
 from frames import blocks
 wire=Wire('recovery_phase_K');wire.send(chosen,'group_directory','reject_corrupted_K',{'source_history':n['source']['history'],'actor':chosen,'retained_own_plan':content_id(p['controls'][chosen])},'fault')
 for group in blocks(p,n['cycle']):
  if not set(group)&set(other):continue
  commit=content_id({'source':n['source'],'selected':physical_id(selected),'group':group});messages=[]
  for c in group:
   msg={'actor':c,'group':group,'commit_id':commit,'history':n['source']['history'],'selected_physical_id':physical_id(selected),'own_plan_id':content_id(selected['controls'][c]),'outer_slot':selected['outer_slots'][c]};wire.send(group[0],c,'prepare_selected_mix',msg,'fault');messages.append(msg)
  ack=[wire.send(msg['actor'],group[0],'ACK_selected_mix',msg,'fault') for msg in messages];assert {m['actor'] for m in ack}==set(group)
  for msg in messages:wire.send(group[0],msg['actor'],'commit_selected_mix',msg,'fault')
 a=complete('recovery_phase_K',selected,n['source']);changed=[c for c in other if tuple(map(F,selected['controls'][c][n['cycle']]))!=tuple(map(F,p['controls'][c][n['cycle']]))];assert changed,'other actual first input must change near transition'
 with gzip.open(HERE/a['full_trace'],'rt') as trace:
  first=next(json.loads(line) for line in trace if json.loads(line)['cycle']==n['cycle']+1)
 assert all(tuple(map(F,first['commands'][c]))==tuple(map(F,selected['controls'][c][n['cycle']])) for c in CARS)
 a.update(fault_cycle=n['cycle'],rejected_actor=chosen,other_accepted_actors_continue=other,other_changed_first_executed=changed,corrupted_K_rejected=True,all_selected_mix_ACKs_before_activation=True);write(HERE/f'results/{a["name"]}_summary.json',a)
 events=read(HERE/'results/R-rolling_summary.json')['events'];e=next(e for e in events if e['meta']['outgoing']==reg['target']);b=complete('recovery_release_ACK',load(e['old_plan']),load(e['checkpoint']),'missing_ACK');assert not b['events'][0]['transaction']['accepted'] and b['events'][1]['transaction']['accepted'];assert b['events'][1]['cycle']==b['events'][0]['cycle']+1;assert b['events'][1]['source']['history']!=b['events'][0]['source']['history'];b['fresh_retry_after_old_step']=True
 assert sha(HERE/'budget_ledger.json')==ledger;write(HERE/'results/rolling_recovery.json',{'passed':True,'branches':[a,b],'new_optimizer_calls':0,'budget_sha':ledger})
if __name__=='__main__':run_all()
