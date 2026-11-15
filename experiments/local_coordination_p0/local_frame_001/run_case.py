"""C/L same physical workers; independent L transactions from actual successors."""
import copy,time,traceback
from common import *
from plan_io import load,save,source,record
from curve_model import generate
from contract import validate_binding,patch,replay_suffix,guard
from frame import validate_frame_plan,node_inclusion
from protocol import Wire,choose,mixed
from physics import step
from transport import exchange
from local_mpc import admit

def frozen():
 for n,h in read(HERE/'executed_source_versions.json').items():assert sha(HERE/n)==h,n

def run():
 frozen();reg=read(HERE/'pre_registration.json');assert reg['passed'] and not read(HERE/'solver_calls.json');frame=read(HERE/'outer_frame.json');sel=read(HERE/'results/source_selection.json');D=sel['D'];X=sel['X'];actors=D+[X];events=[]
 for org in ('C','L'):
  old=load('results/old.json.gz');anchor=load('results/new.json.gz');state=source(old,sel['cycle']);wire=Wire(org)
  for rnd in (1,2):
   start=time.monotonic();label=f'{org}_r{rnd}';save(f'results/{label}_checkpoint.json.gz',{'old':old,'anchor':anchor,'state':record(state)});jobs={};ads={};workerrecords=[]
   for c in actors:
    job,cert=generate(anchor,state,c,'O-E');validate_binding(job,anchor,state);save(f'results/{label}_{c}_job.json.gz',job);save(f'results/{label}_{c}_derivatives.json.gz',cert);jobs[c]=job
    wire.send(c,'frame_directory','query_dependency',{'outer_frame_id':frame['outer_frame_id'],'from':state['cycle'],'permanent_goals':True},'steady');wire.send('frame_directory',c,'valid_frame',{'outer_frame_id':frame['outer_frame_id'],'valid_through':frame['valid_through'],'separation_certificate':frame['separation_faces']},'steady')
   if org=='L':
    for c in actors:assert jobs[c]==load(f'results/C_r{rnd}_{c}_job.json.gz'),('C/L model divergence',rnd,c)
   for c in actors:
    j=jobs[c];stem=HERE/f'results/workers/{label}_{c}';stem.parent.mkdir(parents=True,exist_ok=True);assembled,ameta=exchange(j,'assemble',stem.with_name(stem.name+'_precheck'))
    def release(info):
     frozen();ledger=read(HERE/'solver_calls.json');assert len(ledger)<12 and not any(r['label']==label and r['actor']==c for r in ledger);assert info==assembled['assembly'];ledger.append({'attempt':len(ledger)+1,'label':label,'organization':org,'round':rnd,'actor':c,'cycle':state['cycle'],'model_id':j['model_id'],'outer_frame_id':frame['outer_frame_id'],'source_history':state['history'],'released_at':time.time(),'assembly_id':info['matrix_id'],'status':'released'});write(HERE/'solver_calls.json',ledger)
    raw,meta=exchange(j,'solve',stem,on_ready=release);ledger=read(HERE/'solver_calls.json');ledger[-1].update(status=raw['status'],native_entered_at=raw['native_entered_at'],native_finished_at=raw['native_finished_at'],solve_time=raw['solve_time'],r_prim=raw['r_prim'],r_dual=raw['r_dual'],obj_val=raw['obj_val'],obj_val_dual=raw['obj_val_dual']);write(HERE/'solver_calls.json',ledger);validate_binding(j,anchor,state,raw)
    admission=admit(j,raw,state) if raw['status'] in ('Solved','AlmostSolved') else {'accepted':False,'model_id':j['model_id'],'reason':raw['status']};ads[c]=admission;write(HERE/f'results/{label}_{c}_admission.json',admission);workerrecords.append({'actor':c,'status':raw['status'],'accepted':admission['accepted'],'transport':meta,'assembly_transport':ameta,'solve_time':raw['solve_time']});wire.send(c,'A1' if c in D or org=='C' else X,'own_candidate',{'model_id':j['model_id'],'outer_frame_id':frame['outer_frame_id'],'accepted':admission['accepted'],'per_car_plan_id':j['per_car_plan_id'],'receiver':j['receiver'],'commands':admission.get('commands')},'steady');print('WORKER',label,c,raw['status'],'accepted',admission['accepted'],flush=True)
   candidate=patch(anchor,state,list(jobs.values()),[ads[c] for c in actors]);validate_frame_plan(candidate,frame);save(f'results/{label}_candidate.json.gz',candidate);mixes=[]
   for takeD in (False,True):
    for takeX in (False,True):
     p=mixed(old,candidate,takeD,takeX,D,X);validate_frame_plan(p,frame);name=f'mixes/{label}_D{int(takeD)}X{int(takeX)}';save(f'results/{name}_plan.json.gz',p);cert=replay_suffix(p,state,name);mixes.append({'D_new':takeD,'X_new':takeX,'plan':f'results/{name}_plan.json.gz','certificate':f'results/{name}_certificate.json','passed':cert['passed']})
   selected,transaction=choose(old,candidate,state,frame,D,X,org,wire);assert transaction['D_committed'] and transaction['X_committed'];save(f'results/{label}_committed.json.gz',selected);before=record(state);guard(selected,state);t=time.monotonic();actual=step(state,{c:selected['controls'][c][state['cycle']] for c in CARS},selected['owners'][state['cycle']],8);execution=time.monotonic()-t;node_inclusion(selected,state)
   event={'label':label,'organization':org,'round':rnd,'source':before,'anchor_id':content_id(anchor),'old_id':content_id(old),'new_id':content_id(selected),'workers':workerrecords,'transaction':transaction,'mixes':mixes,'jobs':{c:j['model_id'] for c,j in jobs.items()},'successor':record(state),'actual_first':actual,'execution_wall':execution,'round_wall':time.monotonic()-start,'outer_frame_unchanged':selected['outer_frame_id']==old['outer_frame_id'],'obligation_replacement':{c:{'old_K':content_id(anchor['controls'][c][before['cycle']+32:]),'receiver_phase':before['cycle']+32,'new_plan':content_id(selected['controls'][c]),'absolute_deadline':selected['per_car_policies'][c]['goal']} for c in CARS}};write(HERE/f'results/{label}_event.json',event);events.append(event);write(HERE/'results/events.json',events);old=selected;anchor=copy.deepcopy(selected);print('COMMIT',label,'actual',state['cycle'],'activity',actual['bounds']['active'],flush=True)
  save(f'results/{org}_final_plan.json.gz',selected);save(f'results/{org}_final_source.json.gz',record(state))
 write(HERE/'results/numerical_complete.json',{'passed':True,'attempts':len(read(HERE/'solver_calls.json')),'C_L_models_equal':True,'events':4,'actors':actors});print('PASS numerical12 bounded',flush=True)
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/numerical_failure.json',{'layer':'numerical_protocol','error':repr(e),'traceback':traceback.format_exc(),'attempts':len(read(HERE/'solver_calls.json'))});raise
