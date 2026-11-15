"""Bounded four-arm runner: 16 durable releases, no retries, real successor rebuild."""
import copy,time,traceback
from common import *
from plan_io import load,save,source,record
from curve_model import generate
from transport import exchange
from local_mpc import admit
from contract import validate_binding,patch,replay_suffix,guard
from physics import step

def frozen():
 for name,digest in read(HERE/'executed_source_versions.json').items():assert sha(HERE/name)==digest,name

def run():
 frozen();reg=read(HERE/'pre_registration.json');assert reg['passed'] and not read(HERE/'solver_calls.json');p0=load('results/R_original.json.gz');k=reg['source_cycle'];actors=reg['actors'];all_events=[]
 for arm in reg['arms']:
  plan=copy.deepcopy(p0);state=source(plan,k)
  for rnd in (1,2):
   label=f'{arm}_r{rnd}';start=time.monotonic();before=copy.deepcopy(plan);checkpoint=record(state);save(f'results/{label}_checkpoint.json.gz',{'plan':plan,'state':checkpoint});jobs=[];ads=[];workers=[]
   # All models seal the same joint checkpoint and parent before either solve.
   for c in actors:
    job,cert=generate(plan,state,c,arm);validate_binding(job,plan,state);save(f'results/{label}_{c}_job.json.gz',job);save(f'results/{label}_{c}_derivatives.json.gz',cert);jobs.append(job)
   for job in jobs:
    c=job['actor'];stem=HERE/f'results/workers/{label}_{c}';stem.parent.mkdir(parents=True,exist_ok=True);assembled,_=exchange(job,'assemble',stem.with_name(stem.name+'_precheck'))
    def release(info):
     frozen();ledger=read(HERE/'solver_calls.json');assert len(ledger)<16 and not any(x['label']==label and x['actor']==c for x in ledger);assert info==assembled['assembly'];ledger.append({'attempt':len(ledger)+1,'label':label,'arm':arm,'round':rnd,'actor':c,'cycle':state['cycle'],'model_id':job['model_id'],'parent':job['parent'],'source_history':state['history'],'released_at':time.time(),'assembly_id':info['matrix_id'],'status':'released','new_native_attempt':True});write(HERE/'solver_calls.json',ledger)
    raw,meta=exchange(job,'solve',stem,on_ready=release);ledger=read(HERE/'solver_calls.json');ledger[-1].update(status=raw['status'],native_entered_at=raw['native_entered_at'],native_finished_at=raw['native_finished_at'],solve_time=raw['solve_time'],r_prim=raw['r_prim'],r_dual=raw['r_dual'],obj_val=raw['obj_val'],obj_val_dual=raw['obj_val_dual']);write(HERE/'solver_calls.json',ledger);validate_binding(job,plan,state,raw)
    admission=admit(job,raw,state) if raw['status'] in ('Solved','AlmostSolved') else {'accepted':False,'model_id':job['model_id'],'reason':raw['status']};write(HERE/f'results/{label}_{c}_admission.json',admission);ads.append(admission);workers.append({'actor':c,'status':raw['status'],'accepted':admission['accepted'],'transport':meta,'solve_time':raw['solve_time']});print('WORKER',label,c,raw['status'],'accepted',admission['accepted'],flush=True)
   proposed=patch(plan,state,jobs,ads);save(f'results/{label}_proposed_plan.json.gz',proposed)
   suffix=replay_suffix(proposed,state,f'continuations/{label}');assert suffix['passed']
   # Contract closure contains both changed endpoints and all four envelope users.
   acks={c:{'accepted':True,'own_controls':content_id(proposed['controls'][c]),'contract':content_id(proposed),'deadline':proposed['per_car_policies'][c]['goal'],'previous_deadline':plan['per_car_policies'][c]['goal']} for c in CARS};assert all(a['deadline']<=a['previous_deadline'] for a in acks.values());plan=proposed
   parent=record(state);guard(plan,state);t=time.monotonic();actual=step(state,{c:plan['controls'][c][state['cycle']] for c in CARS},plan['owners'][state['cycle']],8);execution_time=time.monotonic()-t
   event={'label':label,'arm':arm,'round':rnd,'source':checkpoint,'source_plan':content_id(before),'jobs':[j['model_id'] for j in jobs],'workers':workers,'new_plan':content_id(plan),'acks':acks,'obligation_replacement':{'old_32_tail':content_id(before),'new_32_tail':content_id(plan),'old_deadlines':[before['per_car_policies'][c]['goal'] for c in CARS],'new_deadlines':[plan['per_car_policies'][c]['goal'] for c in CARS],'common_K_for_after_phase':{j['actor']:j['receiver']['K_id'] for j in jobs}},'successor':record(state),'actual_first':actual,'execution_wall':execution_time,'round_wall':time.monotonic()-start,'full_continuation_certificate':f'results/continuations/{label}_certificate.json'};write(HERE/f'results/{label}_event.json',event);all_events.append(event);write(HERE/'results/events.json',all_events);save(f'results/{label}_committed_plan.json.gz',plan);print('COMMIT',label,'actual successor',state['cycle'],'active',actual['bounds']['active'],flush=True)
  save(f'results/{arm}_final_plan.json.gz',plan)
 write(HERE/'results/numerical_complete.json',{'passed':True,'attempts':len(read(HERE/'solver_calls.json')),'events':len(all_events),'arms':reg['arms']})
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/numerical_failure.json',{'layer':'numerical_protocol','error':repr(e),'traceback':traceback.format_exc(),'attempts':len(read(HERE/'solver_calls.json'))});raise
