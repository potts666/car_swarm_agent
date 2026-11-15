"""One-shot bounded numerical experiment; ledger release precedes worker start."""
import copy,gzip,multiprocessing as mp,sys,time
from common import *
from physics import step
from local_mpc import prepare,worker,admit
from transaction import check_binding,select
from discovery import certify

def load(n):return json.loads(gzip.decompress((HERE/n).read_bytes()))
def save(n,v):(HERE/n).write_bytes(gzip.compress(json.dumps(v,sort_keys=True,default=str).encode(),mtime=0))
def run(zero=False):
 registration=read(HERE/'pre_registration.json');assert registration['passed']
 if not zero:
  assert read(HERE/'results/protocol_checks.json')['passed'] and read(HERE/'results/zero_two_rounds.json')['passed']
  assert registration['executed_sources']=={p.name:sha(p) for p in sorted(HERE.glob('*.py'))}
 plan=load('results/parallel_anchor.json.gz');state=initial_state();rounds=[];ledger=read(HERE/'solver_calls.json')
 assert ledger==[]
 ctx=mp.get_context('spawn')
 trace=gzip.open(HERE/('results/zero_actual.jsonl.gz' if zero else 'results/actual.jsonl.gz'),'wt')
 for k in range(2):
  source=copy.deepcopy(state);parent=content_id(plan);jobs={c:prepare(plan,state,c) for c in CARS};results={}
  index=certify(plan,plan,participants=CARS,cycle=k);index['actual_cycle']=k;index['actual_source']={c:pose_rows(state['poses'][c]) for c in CARS};save(f'results/{"zero_" if zero else ""}round_{k}_index.json.gz',index)
  for c,job in jobs.items():
   check_binding(job,plan,state,c,job['receiving_certificate'])
   stem=f'results/{"zero_" if zero else ""}round_{k}_{c}';write(HERE/(stem+'_model.json'),job)
   if zero:results[c]={'accepted':False,'reason':'zero-call source/anchor check'};continue
   call={'id':len(ledger)+1,'round':k,'actor':c,'source':content_id({'poses':{x:pose_rows(state['poses'][x]) for x in CARS},'v':state['v'],'delta':state['delta'],'cycle':state['cycle']}),'model_id':job['model_id'],'released_at':time.time(),'status':'released'}
   assert len(ledger)<8;ledger.append(call);write(HERE/'solver_calls.json',ledger)
   parent_conn,child=ctx.Pipe();process=ctx.Process(target=worker,args=(child,));process.start();child.close();payload=json.dumps(job).encode();parent_conn.send_bytes(payload)
   if parent_conn.poll(60):response=parent_conn.recv_bytes();raw=json.loads(response)
   else:process.terminate();response=b'';raw={'status':'worker_timeout'}
   process.join(10);assert not process.is_alive();parent_conn.close()
   call.update(status=raw['status'],finished_at=time.time(),sent_bytes=len(payload),received_bytes=len(response),worker_exitcode=process.exitcode);write(HERE/'solver_calls.json',ledger);write(HERE/(stem+'_raw.json'),raw)
   try:result=admit(job,raw)
   except (AssertionError,KeyError) as e:result={'accepted':False,'reason':repr(e)}
   results[c]=result;write(HERE/(stem+'_admission.json'),result)
  # Each participant owns only its 32 local input pairs. One closure commits
  # accepted candidates and certified new-contract anchors for rejected workers.
  proposed=copy.deepcopy(plan)
  for c,r in results.items():
   if r['accepted']:proposed['controls'][c][k:k+32]=r['commands']
  plan,selection=select(plan,proposed,CARS,CARS,results)
  actual=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);actual['cycle']=state['cycle'];trace.write(json.dumps(actual,default=str)+'\n')
  rounds.append({'cycle':k,'parent':parent,'actual_source':{c:pose_rows(source['poses'][c]) for c in CARS},'models':{c:j['model_id'] for c,j in jobs.items()},'admissions':results,'actual_successor':{c:pose_rows(state['poses'][c]) for c in CARS},'closure':list(CARS),'ACK':list(CARS),'selection':selection,'index_id':content_id(index),'B_before':plan['B0']-k,'B_after':plan['B0']-k-1})
 if zero:
  trace.close();write(HERE/'results/zero_two_rounds.json',{'passed':True,'rounds':rounds,'optimizer_calls':0});return
 for k in range(2,plan['B0']):
  actual=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);actual['cycle']=state['cycle'];trace.write(json.dumps(actual,default=str)+'\n')
 trace.close();save('results/updated.json.gz',plan);write(HERE/'results/rounds.json',rounds);write(HERE/'results/numerical_summary.json',{'passed':all(r['accepted'] for row in rounds for r in row['admissions'].values()),'calls':len(ledger),'accepted':[sum(r['accepted'] for r in row['admissions'].values()) for row in rounds],'first_changed':[sum(r.get('first_changed',False) for r in row['admissions'].values()) for row in rounds],'actual_steps':state['cycle']})
 print(json.dumps(read(HERE/'results/numerical_summary.json')))
if __name__=='__main__':run('--zero' in sys.argv)
