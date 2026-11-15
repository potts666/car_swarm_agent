"""Two real successor rounds,8 durable releases, identical worker precheck path."""
import copy,gzip,time,sys
from common import *
from physics import step
from local_mpc import prepare,admit
from transaction import check_binding,select
from discovery import certify
from transport import exchange
from plan_io import load,save,record,restore,source
from execution import gate

def check_models(plan,state,prefix):
 jobs={c:prepare(plan,state,c) for c in CARS};checks={}
 for c,j in jobs.items():
  check_binding(j,plan,state,c,j['receiving_certificate']);write(HERE/f'{prefix}_{c}_model.json',j)
  result,meta=exchange(j,'assemble',HERE/f'{prefix}_{c}_zero');checks[c]={'assembly':result['assembly'],'transport':meta}
  assert result['assembly']['variables']==64
 return jobs,checks

def precheck():
 assert read(HERE/'solver_calls.json')==[] and read(HERE/'results/event_scan.json')['passed']
 plan=load('results/E_anchor.json.gz');event=read(HERE/'event_registration.json');state=source(load('results/R_old.json.gz'),event['source_cycle']);rounds=[]
 for r in range(2):
  k=event['source_cycle']+r
  jobs,checks=check_models(plan,state,f'results/precheck_round_{r}')
  # Exercise the actual increment quantizer on explicitly synthetic nullspace
  # inputs; these are fixtures, never counted as numerical solve results.
  fixtures={}
  for c,j in jobs.items():
   ref=list(map(F,j['reference']));T=j['T_columns'];score=[sum(ref[t]*col[t] for t in range(64)) for col in T];idx=max(range(len(T)),key=lambda t:abs(score[t]));assert score[idx]!=0
   eps=F('0.00000314159265358979')*(-1 if score[idx]>0 else 1);raw={'status':'Solved','model_id':j['model_id'],'x':[float(eps*t) for t in T[idx]],'fixture':True};a=admit(j,raw);assert a['accepted'];fixtures[c]={'passed':True,'native_calls':0,'scope':'synthetic quantizer fixture'}
  rounds.append({'cycle':k,'round':r,'source':record(state),'models':{c:j['model_id'] for c,j in jobs.items()},'checks':checks,'fixtures':fixtures,'round1_scope':'forecast anchor source only; actual models rechecked after first executed input' if r else 'actual initial event source'})
  step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4)
 write(HERE/'results/worker_precheck.json',{'passed':True,'models':8,'native_calls':0,'rounds':rounds})

def run():
 reg=read(HERE/'pre_registration.json');ledger=read(HERE/'solver_calls.json');assert ledger==[] and reg['limit']==8 and reg['passed']
 assert reg['executed_sources']=={p.name:sha(p) for p in sorted(HERE.glob('*.py'))}
 assert read(HERE/'results/worker_precheck.json')['passed'] and read(HERE/'results/event_scan.json')['passed'] and read(HERE/'results/protocol_precheck.json')['passed']
 plan=load('results/E_anchor.json.gz');base=load('results/R_old.json.gz');event=read(HERE/'event_registration.json');state=initial_state();rounds=[];release_events=[]
 assert content_id(plan)==reg['selected']
 # Initial contract admission is checked against the autonomous baseline,
 # including all individual deadlines, before any updated control executes.
 acks=read(HERE/'results/anchor_acks.json');assert all(m['new_contract']==content_id(plan) and m['old_contract']==content_id(base) and m['accepted'] for m in acks.values())
 assert select(base,plan,CARS,list(acks),{})[0] is plan
 with gzip.open(HERE/'results/actual.jsonl.gz','wt') as trace:
  for t in range(event['source_cycle']):
   row=step(state,{c:base['controls'][c][t] for c in CARS},base['owners'][t],4);row['cycle']=state['cycle'];trace.write(json.dumps(row,default=str)+'\n')
  assert content_id(record(state))==event['source_id']
  for r in range(2):
   k=event['source_cycle']+r
   save(f'results/round_{r}_plan.json.gz',plan);write(HERE/f'results/round_{r}_source.json',record(state))
   jobs,checks=check_models(plan,state,f'results/round_{r}')
   index=certify(base if r==0 else plan,plan,participants=CARS,cycle=k);save(f'results/round_{r}_index.json.gz',index);results={}
   for c,j in jobs.items():
    stem=f'results/round_{r}_{c}';call={'id':len(ledger)+1,'round':r,'actual_cycle':k,'actor':c,'model_id':j['model_id'],'source_id':content_id(record(state)),'released_at':time.time(),'status':'released','native_entered':False}
    assert len(ledger)<8;ledger.append(call);write(HERE/'solver_calls.json',ledger)
    def ready(info):
     assert info==checks[c]['assembly'];call.update(native_entering_durable_at=time.time(),status='native_entering');write(HERE/'solver_calls.json',ledger)
    raw,meta=exchange(j,'solve',HERE/stem,ready)
    call.update(status=raw['status'],finished_at=time.time(),native_entered='native_entered_at' in raw,transport=meta,native_entered_at=raw.get('native_entered_at'));write(HERE/'solver_calls.json',ledger);write(HERE/(stem+'_raw.json'),raw)
    # Infrastructure errors stop remaining releases; retained native outcomes
    # are never relabeled or automatically repeated.
    assert raw['status']!='worker_error',('worker failure',stem)
    assert raw['assembly']==checks[c]['assembly']
    try:result=admit(j,raw)
    except (AssertionError,KeyError) as e:result={'accepted':False,'reason':repr(e),'model_id':j['model_id']}
    results[c]=result;write(HERE/(stem+'_admission.json'),result)
   proposed=copy.deepcopy(plan)
   for c,result in results.items():
    if result['accepted']:proposed['controls'][c][k:k+32]=result['commands']
   selected,selection=select(plan,proposed,CARS,CARS,results)
   assert all(selected['per_car_policies'][c]['goal']<=base['per_car_policies'][c]['goal'] for c in CARS)
   # All numerical proposals are registered before checking the commit.
   final_index=certify(plan,selected,participants=CARS,cycle=k);save(f'results/round_{r}_commit_index.json.gz',final_index)
   plan=selected;save(f'results/round_{r}_committed.json.gz',plan)
   actual=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);actual['cycle']=state['cycle'];trace.write(json.dumps(actual,default=str)+'\n');trace.flush()
   row={'cycle':k,'round':r,'source':read(HERE/f'results/round_{r}_source.json'),'parent':content_id(load(f'results/round_{r}_plan.json.gz')),'models':{c:j['model_id'] for c,j in jobs.items()},'admissions':results,'actual_successor':record(state),'selection':selection,'ACK':list(CARS),'closure':list(CARS),'pre_index_id':content_id(index),'commit_index_id':content_id(final_index),'B_before':plan['B0']-k,'B_after':plan['B0']-k-1}
   rounds.append(row);write(HERE/'results/rounds.json',rounds)
  for k in range(event['source_cycle']+2,plan['B0']):
   release=gate(plan,state)
   if release:release_events.append(release)
   row=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);row['cycle']=state['cycle'];trace.write(json.dumps(row,default=str)+'\n')
 save('results/updated.json.gz',plan);write(HERE/'results/actual_events.json',release_events)
 summary={'passed':all(r['accepted'] for row in rounds for r in row['admissions'].values()),'native_calls':sum(c['native_entered'] for c in ledger),'releases':len(ledger),'accepted':[sum(r['accepted'] for r in row['admissions'].values()) for row in rounds],'first_changed':[sum(r.get('first_changed',False) for r in row['admissions'].values()) for row in rounds],'steps':state['cycle']}
 write(HERE/'results/numerical_summary.json',summary);print(json.dumps(summary))
if __name__=='__main__':
 if '--precheck' in sys.argv:precheck()
 else:run()
