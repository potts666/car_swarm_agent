"""Actual support + invariant tail + current post-exit receiver + source identity."""
from functools import lru_cache
from common import *
from canonical import base,states,point,index,exit_phase,pose_error
from physics import body,propagate

def support(c,p):
 box=m.axis_box(body(p,c));sg=1 if F(TASKS[c]['goal'][0])>16 else -1
 return F(box[0],m.Q)-16-TUBE if sg>0 else -16-F(box[1],m.Q)-TUBE
@lru_cache(None)
def future_profile(c):
 refs=states();goal=base()['per_car_policies'][c]['goal'];out=[F(0)]*(goal+1);minimum=F(1000)
 for t in range(goal,-1,-1):
  p=tuple(m.I.raw(*q) for q in refs[t]['poses_Q'][c]);minimum=min(minimum,support(c,p)-F(1,40)-15*H*H/8);out[t]=minimum
 return out

def canonical_command(c,t,start):
 p=base()['per_car_policies'][c]
 if t<start:return base()['controls'][c][t] if t<p['approach_end'] else ['0','0']
 j=p['service_start']+t-start
 return base()['controls'][c][j] if j<p['goal'] else ['0','0']
def tail_length(c,idx):
 refs=states();goal=base()['per_car_policies'][c]['goal'];return sum(abs(H*(F(refs[t]['v'][c])+F(refs[t+1]['v'][c]))/2) for t in range(idx,goal))
def actual_future(plan,state,c):
 k=state['cycle'];policy=plan['per_car_policies'][c];start=policy['service_start'];goal=policy['goal'];hold=all(tuple(map(F,u))==(0,0) for u in plan['controls'][c][goal:]);changed=[t for t in range(k,goal) if tuple(map(F,plan['controls'][c][t]))!=tuple(map(F,canonical_command(c,t,start)))];end=changed[-1]+1 if changed else k;p=state['poses'][c];v=state['v'][c];d=state['delta'][c];minimum=F(1000)
 try:
  for t in range(k,end):
   a,w=map(F,plan['controls'][c][t]);nv,nd=v+H*a,d+H*w;assert max(abs(a),abs(w))<=1 and -1<=nv<=2 and abs(nd)<=F(1,2);nodes=propagate(p,v,nv,d,nd,direction(c),8)
   minimum=min(minimum,min(support(c,z) for z in nodes)-15*(H/8)**2/8);p=nodes[-1];v,d=nv,nd
  nominal,nv,nd=point(c,end,start);ep=sum((p[j]-nominal[j]).abs_upper() for j in (0,1));et=(p[2]-nominal[2]).abs_upper();bound=ep+(tail_length(c,index(c,end,start))+RB)*et;exact=v==nv and d==nd;tail=future_profile(c)[index(c,end,start)];minimum=min(minimum,tail);valid=minimum>0 and bound<=F(1,40) and exact and hold
  return {'passed':valid,'support_min':minimum,'saved_modified_prefix_end':end,'tail_SE2_bound':bound,'exact_v_delta_at_tail':exact,'permanent_zero_hold':hold,'method':'actual saved modified prefix then certified unchanged same-input canonical tail'}
 except (AssertionError,ValueError) as e:return {'passed':False,'support_min':minimum,'restriction':repr(e),'permanent_zero_hold':hold}

def clear(plan,state,c):
 k=state['cycle'];p=plan['per_car_policies'][c];nominal,v,d=point(c,k,p['service_start']);raw=support(c,state['poses'][c]);tail=future_profile(c)[index(c,k,p['service_start'])];err=pose_error(state['poses'][c],nominal);phase=k-p['service_start']+p['approach_end'];after=phase>=plan['paths'][c]['ranges']['crossing'][1];ep=sum((state['poses'][c][j]-nominal[j]).abs_upper() for j in (0,1));et=(state['poses'][c][2]-nominal[2]).abs_upper();prefix=tail_length(c,0)-tail_length(c,index(c,k,p['service_start']));rp=F(1,400)+prefix/F(100000);domain=ep<=rp and et<=F(1,100000) and state['v'][c]==v and state['delta'][c]==d
 future=actual_future(plan,state,c) if after and raw>0 else {'passed':tail>0,'support_min':tail,'method':'pre-exit canonical necessary test only; post-exit actual suffix still required'}
 source={'cycle':k,'history':state['history'],'actor':c,'pose_Q':pose_rows(state['poses'][c]),'v':state['v'][c],'delta':state['delta'][c],'own_plan_id':content_id(plan['controls'][c]),'outer_slot':plan.get('outer_slots',{}).get(c),'contract_id':content_id(plan),'phase':phase,'deadline':p['goal'],'K_id':content_id(plan['controls'][c][k:p['goal']])};token=content_id(source)
 return {'actor':c,'geometric_clear':raw>0,'support_gap':raw,'future_clear':future['passed'],'future_min_gap':future['support_min'],'actual_future':future,'post_exit_phase':after,'receiver_valid':domain,'body_pose_error':err,'nominal_v':v,'nominal_delta':d,'full_Clear':raw>0 and future['passed'] and after and domain,'source':source,'token':token,'guard':'actual positive goal-side rectangle support; all saved post-exit input/hold remains outside with .025 domain+15h²/8; Q_phase checks actual pose and exact phase v/delta, not point or fixed tick'}
def verify_clear(plan,state,certificate):
 assert certificate==clear(plan,state,certificate['actor']),'stale state/frame/history/K Clear token';assert certificate['full_Clear'],'Clear prerequisites failed';return True
