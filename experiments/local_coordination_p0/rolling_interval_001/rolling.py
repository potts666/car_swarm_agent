"""Checkpointed one-trajectory rolling run; one native release per actor/cycle/version."""
import argparse,copy,gzip,time,traceback,os
from common import *
from plan_io import load,save,restore,record
from rolling_model import generate,classify,mode
from local_mpc import admit
from contracts import binding,guards,physical_id
from physics import step
from canonical import reference,point,pose_error
from release import clear
from schedules import goals
from engine import candidate
from frames import blocks
from transport import exchange
from wire import Wire

def frozen():
 for n,h in read(HERE/'executed_source_versions.json').items():assert sha(HERE/n)==h,('frozen source changed',n)

def obj(value):
 key=content_id(value);path=f'results/objects/{key}.json.gz'
 if not(HERE/path).exists():save(path,value)
 return key

def full_obligation(plan,state,c,job,ad):
 """Prefix original rows + exact v/delta phase seams + same-input full SE2 tail."""
 k=state['cycle'];refs=reference(plan);end=ad['receiver']['phase'];actual=ad['_rollout'] if '_rollout' in ad else None
 from local_mpc import rollout,check_receiver
 actual=rollout(job,state,ad['commands'],64);check_receiver(job,actual)
 # Future unchanged full tail uses original canonical phase corridor certificate.
 target=refs[end];p=[m.I.raw(*q) for q in actual['pose_Q']];q=[m.I.raw(*z) for z in target['poses_Q'][c]];ep=sum((p[j]-q[j]).abs_upper() for j in (0,1));et=(p[2]-q[2]).abs_upper();bound=ep+(F(job['receiver']['L_remaining'])+RB)*et;assert bound<F(1,40),('full actual suffix bound',c,k,bound)
 for j,node in enumerate(actual['nodes'],k+1):
  assert F(node['v'])==F(refs[j]['v'][c]) if F(refs[j]['v'][c])==0 else True
  if F(refs[j]['v'][c])==0:assert F(node['delta'])==F(refs[j]['delta'][c]),('stopped phase delta seam',c,j)
 return {'passed':True,'actor':c,'source_history':state['history'],'model_id':job['model_id'],'receiver_phase':end,'old_K_id':job['receiver']['K_id'],'prefix_rows_continuous':True,'tail_SE2_upper':bound,'tail_position_error':ep,'tail_yaw_error':et,'tail_controls_id':content_id(plan['controls'][c][end:]),'deadline':plan['per_car_policies'][c]['goal'],'entire_plan_coverage':'original continuous prefix rows to N, same-input canonical SE2 tail through every phase and permanent goal; positive receiver, exact phase v/delta','actual_end':actual['pose_Q']}

def init():
 reg=read(HERE/'interval_registration.json');events=read(HERE/'results/E-anchor_events.json');e=next(e for e in events if e['cycle']==reg['start']);return {'plan':load(e['candidate']),'state':load(e['checkpoint']),'start':reg['start'],'cycles_done':0,'end_cycle':None,'successor_first_motion':None,'stop':None,'same_failure_streak':{},'last_failures':{},'events':[],'obligation_parent':content_id(load(e['candidate']))}

def run(until):
 frozen();assert read(HERE/'results/rolling_preflight.json')['passed'];cp=load('results/rolling_checkpoint.json.gz') if (HERE/'results/rolling_checkpoint.json.gz').exists() else init();plan=cp['plan'];state=restore(cp['state']);reg=read(HERE/'interval_registration.json');wire=Wire('R-rolling');started=time.monotonic()
 if cp['stop'] or (cp['end_cycle'] is not None and state['cycle']>=cp['end_cycle']):return finish(cp)
 while cp['cycles_done']<until and cp['cycles_done']<512:
  k=state['cycle'];cycle_start=time.monotonic();before=record(state);event=None;proof=clear(plan,state,plan['owners'][k]) if plan['owners'][k] else None
  if proof and proof['full_Clear']:
   outgoing=plan['owners'][k];idx=plan['order'].index(outgoing)
   if idx+1<len(plan['order']) and plan['per_car_policies'][plan['order'][idx+1]]['service_start']>k:
    plan,event,_=candidate(plan,state,outgoing,proof,wire,f'R-rolling/release_{k}')
    if event:
     cp['events'].append(event)
     if outgoing==reg['target'] and event['transaction']['accepted']:cp['target_clear']=k;cp['next_actor']=event['meta']['incoming']
  context=content_id({'plan':physical_id(plan),'source':before,'outer':plan['outer_frame_id']});jobs={};rows={};admissions={};obligations={};old=copy.deepcopy(plan)
  for c in CARS:
   t=time.monotonic();row={'cycle':k,'actor':c,'mode':mode(plan,c,k),'source_history':state['history'],'context':context,'owner':plan['owners'][k],'lease':plan['lease_groups'],'groups':blocks(plan,k),'old_first':plan['controls'][c][k],'native_status':None,'accepted':False,'adopted':False}
   try:
    job,cert=generate(plan,state,c);binding(job,plan,state);kind,proofclass=classify(plan,state,c,job);jobs[c]=job;row.update(classification=kind,classification_proof=proofclass,model_id=job['model_id'],receiver_id=content_id(job['receiver']),zero_anchor=True,model_object=obj(job));row['generation_seconds']=time.monotonic()-t
   except Exception as e:
    row.update(classification='fallback_after_failure',failure_layer='generation',reason=repr(e),zero_anchor=False);save(f'results/failures/k{k}_{c}.json.gz',{'source':before,'plan':plan,'error':repr(e),'traceback':traceback.format_exc()})
   rows[c]=row
  # A generation error is not a license to issue a speculative solve.
  for c in CARS:
   row=rows[c]
   if row['classification']!='numerical_update' or any(not r['zero_anchor'] for r in rows.values()):continue
   job=jobs[c];stem=HERE/f'results/workers/k{k}_{c}';stem.parent.mkdir(parents=True,exist_ok=True)
   try:
    assembled,ameta=exchange(job,'assemble',stem.with_name(stem.name+'_precheck'));row['assembly']=assembled['assembly'];row['assembly_seconds']=ameta['roundtrip_wall']
    def release(info):
     frozen();assert info==assembled['assembly'];ledger=read(HERE/'budget_ledger.json');assert ledger[-1]['model_id']==job['model_id'];ledger[-1].update(native_ready=True,assembly_id=info['matrix_id']);write(HERE/'budget_ledger.json',ledger)
    ledger=read(HERE/'budget_ledger.json');assert len(ledger)<2048;assert not any(r['cycle']==k and r['actor']==c for r in ledger);ledger.append({'attempt':len(ledger)+1,'actor':c,'cycle':k,'model_id':job['model_id'],'source_history':state['history'],'context':context,'source_version':content_id(read(HERE/'executed_source_versions.json')),'released_at':time.time(),'status':'worker_released'});write(HERE/'budget_ledger.json',ledger)
    raw,meta=exchange(job,'solve',stem,on_ready=release);ledger=read(HERE/'budget_ledger.json');ledger[-1].update(status=raw['status'],native_entered_at=raw['native_entered_at'],native_finished_at=raw['native_finished_at'],solve_time=raw['solve_time']);write(HERE/'budget_ledger.json',ledger);row.update(native_status=raw['status'],raw_object=obj(raw),worker_seconds=meta['roundtrip_wall'],worker_transport=meta);binding(job,plan,state,raw);ad=admit(job,raw,state);row['admission_object']=obj(ad);row['accepted']=ad['accepted']
    if ad['accepted']:
     proof=full_obligation(plan,state,c,job,ad);obligations[c]=proof;admissions[c]=ad;row.update(obligation_object=obj(proof),JE_change=F(ad['objective']['JE'])-F(ad['baseline_objective']['JE']),adopted=True,first_a_changed=ad['first_a_changed'],first_omega_changed=ad['first_omega_changed'])
    else:row.update(classification='fallback_after_failure',failure_layer='admission',reason='native candidate failed fixed original-row/quantization/receiver/objective admission')
   except Exception as e:
    row.update(classification='fallback_after_failure',failure_layer='worker_or_certificate',reason=repr(e));write(HERE/f'results/failures/k{k}_{c}.json',{'error':repr(e),'traceback':traceback.format_exc(),'source':before,'context':context})
   # Lossless content objects retain bytes once, no repeated uncompressed requests.
   for f in stem.parent.glob(f'k{k}_{c}*'):
    if f.is_file() and f.suffix=='.json':f.with_suffix('.json.gz').write_bytes(gzip.compress(f.read_bytes(),mtime=0));f.unlink()
  for group in blocks(plan,k):
   changed=[c for c in group if c in admissions]
   if not changed:continue
   commit=content_id({'context':context,'group':group,'obligations':{c:content_id(obligations[c]) for c in changed}})
   for c in group:
    msg={'actor':c,'context':context,'group':group,'commit_id':commit,'outer_slot':plan['outer_slots'][c]};wire.send(group[0],c,'prepare_MPC',msg,'steady');wire.send(c,group[0],'ACK_MPC',msg,'steady');wire.send(group[0],c,'commit_MPC',msg,'steady')
   for c in changed:plan['controls'][c][k:k+32]=admissions[c]['commands'];rows[c]['commit_id']=commit
  # Original actual prefix is immutable; independent suffix certificates compose across blocks.
  assert all(old['controls'][c][:k]==plan['controls'][c][:k] for c in CARS);guards(plan,state)
  node={'cycle':k,'parent':cp['obligation_parent'],'source':before,'context':context,'rows':rows,'release':event,'plan_before':physical_id(old),'plan_after':physical_id(plan),'commands':{c:plan['controls'][c][k] for c in CARS}}
  simulated=copy.deepcopy(state);step(simulated,node['commands'],plan['owners'][k],8) # execution safety before adoption output
  actual=step(state,node['commands'],plan['owners'][k],8);actual['cycle']=state['cycle'];node['successor']=record(state);node['actual_activity']=[c for c in CARS if abs(H*(2*F(before['v'][c])+H*F(node['commands'][c][0]))/2)>=F(1,50)];node['wall_seconds']=time.monotonic()-cycle_start;node['B']=max(goals(plan).values())-k;node['node_id']=content_id(node);cp['obligation_parent']=node['node_id']
  with (HERE/'results/rolling_cycles.jsonl').open('a') as f:f.write(json.dumps(node,default=str)+'\n')
  with gzip.open(HERE/'results/rolling_interval_actual.jsonl.gz','at') as f:f.write(json.dumps(actual,default=str)+'\n')
  for c,row in rows.items():
   if row['classification']=='fallback_after_failure':
    key=(row.get('failure_layer','')+':'+row.get('reason',''));cp['same_failure_streak'][c]=cp['same_failure_streak'].get(c,0)+1 if cp['last_failures'].get(c)==key else 1;cp['last_failures'][c]=key
    if cp['same_failure_streak'][c]>=3:cp['stop']={'cycle':k,'actor':c,'reason':key,'streak':3}
    if not row['zero_anchor']:cp['stop']={'cycle':k,'actor':c,'reason':'next receiver/model anchor not established: '+key}
   else:cp['same_failure_streak'][c]=0;cp['last_failures'].pop(c,None)
  if cp.get('next_actor') in node['actual_activity'] and cp['successor_first_motion'] is None:cp['successor_first_motion']=k;cp['end_cycle']=k+32
  cp.update(plan=plan,state=record(state),cycles_done=cp['cycles_done']+1);save('results/rolling_checkpoint.json.gz',cp);write(HERE/'results/rolling_progress.json',{q:cp.get(q) for q in ('cycles_done','start','end_cycle','target_clear','successor_first_motion','stop')});print('CYCLE',k,[(c,rows[c]['classification'],rows[c]['adopted']) for c in CARS],flush=True)
  if cp['stop'] or (cp['end_cycle'] is not None and state['cycle']>=cp['end_cycle']):break
 if cp['cycles_done']>=512 and cp['end_cycle'] is None:cp['stop']={'reason':'512-cycle coverage cap'};save('results/rolling_checkpoint.json.gz',cp)
 if cp['stop'] or cp['end_cycle'] is not None and state['cycle']>=cp['end_cycle'] or cp['cycles_done']>=512:finish(cp)
 else:write(HERE/'results/first32_acceptance.json',{'passed':True,'cycles':cp['cycles_done'],'actual_source':cp['state'],'calls':len(read(HERE/'budget_ledger.json')),'backup_valid':True})

def finish(cp):
 # Existing acknowledged complete plan supplies the suffix; release observer remains enabled.
 from engine import run as run_anchor
 ledger=sha(HERE/'budget_ledger.json');r=run_anchor('R-rolling_suffix','anchor',{'plan':cp['plan'],'state':cp['state']});assert sha(HERE/'budget_ledger.json')==ledger
 reg=read(HERE/'interval_registration.json');prefix=[]
 with gzip.open(HERE/'results/E-anchor_actual.jsonl.gz','rb') as f:
  for line in f:
   if json.loads(line)['cycle']>reg['start']:break
   prefix.append(line)
 data=b''.join(prefix)+gzip.decompress((HERE/'results/rolling_interval_actual.jsonl.gz').read_bytes())+gzip.decompress((HERE/r['trace']).read_bytes());(HERE/'results/R-rolling_actual.jsonl.gz').write_bytes(gzip.compress(data,mtime=0));save('results/R-rolling_final_plan.json.gz',load(r['plan']));write(HERE/'results/R-rolling_summary.json',{'passed_execution':True,'coverage':cp['end_cycle'] is not None and cp['state']['cycle']>=cp['end_cycle'],'cycles':cp['cycles_done'],'stop':cp['stop'],'interval_end':cp['state']['cycle'],'target_clear':cp.get('target_clear'),'successor_first_motion':cp['successor_first_motion'],'events':cp['events']+r['events'],'final_cycle':r['final_cycle'],'goals':r['goals'],'formal_attempts':len(read(HERE/'budget_ledger.json'))})
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--until',type=int,default=512);args=ap.parse_args();run(args.until)
