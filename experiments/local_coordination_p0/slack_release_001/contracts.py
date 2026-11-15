"""Actual full continuation, common-mode guards and immutable own-model binding."""
import copy,gzip
from common import *
from canonical import reference,point,pose_error
from receivers import check_domain
from plan_io import record
from physics import step
from release import clear

def binding(job,plan,state,raw=None):
 c=job['actor'];assert job['parent']==content_id(plan['controls'][c]) and job['outer_frame_id']==plan['outer_slots'][c];assert job['cycle']==state['cycle'] and job['source']==json.loads(json.dumps({'poses_Q':pose_rows(state['poses'][c]),'v':state['v'][c],'delta':state['delta'][c],'history':state['history']},default=str));assert job['receiver']['phase']==state['cycle']+32 and job['receiver']['K_id']==content_id(plan['controls'][c][state['cycle']+32:]);assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'})
 if raw:assert raw['model_id']==job['model_id']
 return True

def guards(plan,state):
 k=state['cycle'];refs=reference(plan);out=[]
 for c in CARS:
  if F(refs[k]['v'][c])==0 and F(refs[k]['delta'][c])!=0:out.append(check_domain(plan,state,c))
 if k and k<len(plan['owners']) and plan['owners'][k]!=plan['owners'][k-1] and plan['owners'][k-1]:
  outgoing=plan['owners'][k-1];proof=clear(plan,state,outgoing);assert proof['full_Clear'],('actual owner exit guard',k,outgoing,proof);out.append({'resource_release':proof})
 return out

def physical_id(plan):return content_id({k:plan[k] for k in ('controls','owners','per_car_policies','B0')})
def continuation(plan,state,stem):
 key=content_id({'plan':physical_id(plan),'source':record(state)});cached=HERE/f'results/continuations/{key}.json'
 if cached.exists():r=read(cached);write(HERE/f'results/{stem}_certificate.json',r);return r
 s=copy.deepcopy(state);goalhits={};minimum={'road':F(100),'pair':F(100)};maxerror=F(0);events=[]
 for k in range(s['cycle'],plan['B0']):
  for c in CARS:
   p,_,_=point(c,k,plan['per_car_policies'][c]['service_start']);e=pose_error(s['poses'][c],p);assert e<=F(1,40),('published body domain',c,k,e);maxerror=max(maxerror,e)
  events.extend(guards(plan,s));r=step(s,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],8)
  for name in minimum:minimum[name]=min(minimum[name],r['bounds'][name])
  for c in CARS:
   if s['cycle']==plan['per_car_policies'][c]['goal']:
    error=sum((s['poses'][c][j]-F(TASKS[c]['goal'][j])).abs_upper() for j in (0,1));assert error<F(2,25) and s['poses'][c][2].abs_upper()<F(1,100) and s['v'][c]==s['delta'][c]==0;goalhits[c]=s['cycle']
 # Completed goals may precede source; their permanent hold is retained.
 for c in CARS:
  if plan['per_car_policies'][c]['goal']<=state['cycle']:assert state['v'][c]==state['delta'][c]==0;goalhits[c]=plan['per_car_policies'][c]['goal']
 assert len(goalhits)==4;hold=step(copy.deepcopy(s),{c:(0,0) for c in CARS},None,8);assert not hold['bounds']['active'];r={'passed':True,'source':record(state),'physical_id':physical_id(plan),'goals':goalhits,'minimum':minimum,'max_node_body_error':maxerror,'mode_and_release_guards':events,'permanent_hold':True,'final':record(s),'new_optimizer_calls':0};write(cached,r);write(HERE/f'results/{stem}_certificate.json',r);return r
