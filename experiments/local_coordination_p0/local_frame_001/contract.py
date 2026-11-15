"""Common source-bound contract, certified mode domains and atomic pair updates."""
import copy,gzip
from common import *
from plan_io import load,save,record
from physics import step,body
from receivers import reference,check_domain

def guard(plan,state):
 k=state['cycle'];ref=reference(plan)[k];out=[]
 for c in CARS:
  if k and F(ref['v'][c])==0 and F(ref['delta'][c])!=0:
   out.append(check_domain(state,c,k,plan))
 event=plan.get('encounter')
 if event and k==event['first_out']:
  c,d=event['passing'],event['yielding'];out.append(check_domain(state,c,k,plan));box=m.axis_box(body(state['poses'][c],c));clear=F(box[0],m.Q)-TUBE>16 if direction(c)>0 else F(box[1],m.Q)+TUBE<-16
  assert clear and state['v'][d]==state['delta'][d]==0 and plan['owners'][k-1]==c and plan['owners'][k]==d
  out.append({'event':'exit_domain_body_clear_release','cycle':k,'passing':c,'yielding':d,'actual_body_clear':True})
 return out

def validate_binding(job,plan,state,raw=None):
 c=job['actor'];k=state['cycle'];assert job['parent']==content_id(plan['controls'][c]) and job['cycle']==k
 assert job['source']==json.loads(json.dumps({'poses_Q':pose_rows(state['poses'][c]),'v':state['v'][c],'delta':state['delta'][c],'history':state['history']},default=str))
 assert job['outer_frame_id']==plan['outer_frame_id'] and job['per_car_plan_id']==content_id(plan['controls'][c])
 assert job['receiver']['phase']==k+32 and job['receiver']['K_id']==content_id(plan['controls'][c][k+32:])
 assert job['model_id']==content_id({key:v for key,v in job.items() if key!='model_id'})
 if raw is not None:assert raw['model_id']==job['model_id']
 return True

def patch(plan,state,jobs,admissions):
 assert len(jobs)>=1 and len(set(j['actor'] for j in jobs))==len(jobs);new=copy.deepcopy(plan);k=state['cycle']
 for j,a in zip(jobs,admissions):
  validate_binding(j,plan,state);assert a['model_id']==j['model_id']
  if a['accepted']:new['controls'][j['actor']][k:k+32]=a['commands']
 assert new['owners']==plan['owners'] and new['per_car_policies']==plan['per_car_policies'];return json.loads(json.dumps(new,default=str))

def replay_suffix(plan,state,stem,sub=8):
 # Direct actual physical replay complements the uniform common-domain proof.
 from local_mpc import check_receiver
 from frame import node_inclusion,physical_id
 key=content_id({'physical':physical_id(plan),'source':record(state),'sub':sub});cached=HERE/f'results/continuations/by_id/{key}_certificate.json'
 if cached.exists():
  result=read(cached);write(HERE/f'results/{stem}_certificate.json',result);return result
 requested_stem=stem;stem=f'continuations/by_id/{key}'
 state=copy.deepcopy(state);guards=[];maximum_domain={c:F(0) for c in CARS};minimum={'road':F(100),'pair':F(100)};start=state['cycle'];goals={};trace=HERE/f'results/{stem}.jsonl.gz';trace.parent.mkdir(parents=True,exist_ok=True)
 with gzip.open(trace,'wt') as f:
  while state['cycle']<plan['B0']:
   k=state['cycle'];errors=node_inclusion(plan,state)
   for c in CARS:maximum_domain[c]=max(maximum_domain[c],errors[c])
   guards.extend(guard(plan,state));row=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],sub);row['cycle']=state['cycle'];f.write(json.dumps(row,default=str)+'\n')
   for key in minimum:minimum[key]=min(minimum[key],row['bounds'][key])
   for c in CARS:
    if state['cycle']==plan['per_car_policies'][c]['goal']:
     ep=sum((state['poses'][c][j]-F(TASKS[c]['goal'][j])).abs_upper() for j in (0,1));et=state['poses'][c][2].abs_upper();assert ep<F(2,25) and et<F(1,100) and state['v'][c]==state['delta'][c]==0;goals[c]={'cycle':state['cycle'],'position_error':ep,'yaw_error':et}
 assert len(goals)==4
 for c in CARS:assert state['v'][c]==state['delta'][c]==0
 hold=step(copy.deepcopy(state),{c:(0,0) for c in CARS},None,sub);assert hold['bounds']['active']==[]
 result={'passed':True,'actual_occupancy_inclusion':True,'maximum_node_body_domain_error':maximum_domain,'source':start,'goals':goals,'minimum':minimum,'guards':guards,'final':record(state),'B_final':0,'all_six_pairs':True,'trace':str(trace.relative_to(HERE)),'optimizer_calls':0};write(HERE/f'results/{stem}_certificate.json',result);write(HERE/f'results/{requested_stem}_certificate.json',result);return result
