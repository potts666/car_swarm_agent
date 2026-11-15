"""Identity-bound closure checks used by both live commits and damaged fixtures."""
from common import *

def check_binding(job,plan,state,c,certificate=None):
 assert job['actor']==c and job['cycle']==state['cycle']
 assert job['parent']==content_id(plan)
 assert job['state']==json.loads(json.dumps({'poses_Q':pose_rows(state['poses'][c]),'v':state['v'][c],'delta':state['delta'][c]},default=str))
 assert job['reference']==[str(q) for u in plan['controls'][c][state['cycle']:state['cycle']+32] for q in map(F,u)]
 n=job['n_straight']
 for power,label in ((0,'terminal_v_exact'),(1,'terminal_x_exact')):
  rr=['0']*64
  for j in range(n):rr[2*j]=str(j**power)
  assert rr in job['E'] and label in job['equality_labels'],'terminal equality omitted'
 assert all(sum(F(e[j])*col[j] for j in range(64))==0 for e in job['E'] for col in job['T_columns'])
 expected=content_id({k:v for k,v in job.items() if k!='model_id'});assert job['model_id']==expected
 if certificate:
  assert certificate['parent']==job['parent'] and certificate['phase']==state['cycle']+32
  assert certificate['deadline']==plan['per_car_policies'][c]['goal'] and certificate['K_id']==content_id(plan['controls'][c][state['cycle']+32:])
 return True

def closure_required(old,new):
 changed={c for c in CARS if old['per_car_policies'][c]!=new['per_car_policies'][c] or old['controls'][c]!=new['controls'][c]}
 for a,b in zip(old['owners'],new['owners']):
  if a!=b:changed.update(x for x in (a,b) if x is not None)
 return changed

def select(old,new,participants,acks,worker_results):
 assert all(new['per_car_policies'][c]['goal']<=old['per_car_policies'][c]['goal'] for c in CARS),'individual deadline increase'
 assert closure_required(old,new)<=set(participants),'resource/changed endpoint omitted'
 if set(acks)!=set(participants):return old,'second_layer_old_contract'
 return new,'first_layer_new_anchor_or_accepted_candidates'
