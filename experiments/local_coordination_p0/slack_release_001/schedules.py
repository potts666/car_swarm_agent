"""Future-only integer start search and certified zero-wait splice propagation."""
import copy
from common import *
from canonical import base,point,exit_phase,pose_error

def goals(p):return {c:p['per_car_policies'][c]['goal'] for c in CARS}
def owner_intervals(p):return {c:[p['per_car_policies'][c]['service_start'],p['release_cuts'].get(c,p['per_car_policies'][c]['goal'])] for c in CARS}
def group_for(p,c):return next(g for g,v in p['lease_groups'].items() if c in v['members'])
def deadlines(old,new,k):
 a,b=goals(old),goals(new);assert all(b[c]<=a[c] for c in CARS),'deadline increase';assert any(b[c]<a[c] for c in CARS),'no strictly decreased integer deadline';return {'old':a,'new':b,'Phi_old':sum(a.values()),'Phi_new':sum(b.values()),'B_old':max(max(d-k,0) for d in a.values()),'B_new':max(max(d-k,0) for d in b.values())}
def validate_prefix(old,new,k):
 assert all(old['controls'][c][:k]==new['controls'][c][:k] for c in CARS),'executed command prefix changed';assert old['owners'][:k]==new['owners'][:k],'executed owner prefix changed'
def intervals_valid(p):
 rr=sorted((a,b,c) for c,(a,b) in owner_intervals(p).items());assert all(a<=b for a,b,c in rr)
 for (_,b,_),(a,_,_) in zip(rr,rr[1:]):assert b<=a,'overlapping half-open owner intervals'
 for c,(a,b) in owner_intervals(p).items():lo,hi=p['lease_groups'][group_for(p,c)]['window'];assert lo<=a<=b<=hi,('outside group lease',c,a,b,lo,hi)
 return True

def propose(old,state,outgoing,start):
 k=state['cycle'];order=old['order'];j=order.index(outgoing);assert j+1<len(order),'no successor';incoming=order[j+1];assert k<=start<old['per_car_policies'][incoming]['service_start'],'no earlier unexecuted start'
 w=copy.deepcopy(old);same=group_for(old,outgoing)==group_for(old,incoming);changed=[incoming] if same else order[j+1:];splices=[];starts={incoming:start}
 for n,c in enumerate(changed):
  if n:starts[c]=w['per_car_policies'][changed[n-1]]['goal']
  policy=old['per_car_policies'][c];previous=policy['service_start'];goal=policy['goal'];s=starts[c];assert k<=s<previous,'non-earlier propagation';assert all(tuple(map(F,u))==(0,0) for u in old['controls'][c][k:previous]),'cannot delete nonzero wait or steering/K'
  nominal,_,_=point(c,k,previous);ep=sum((state['poses'][c][t]-nominal[t]).abs_upper() for t in (0,1));et=(state['poses'][c][2]-nominal[2]).abs_upper();assert state['v'][c]==state['delta'][c]==0,'waiting splice requires exact v/delta0';assert ep<=F(1,400) and et<=F(1,100000),'waiting receiver missing'
  L=sum(abs(H*(F(states_v(c,t))+F(states_v(c,t+1)))/2) for t in range(base()['per_car_policies'][c]['service_start'],base()['per_car_policies'][c]['goal']));error=ep+(L+RB)*et;assert error<F(1,40),'splice full-tail SE2 inclusion'
  commands=old['controls'][c][previous:goal];assert len(commands)>0;w['per_car_policies'][c]['service_start']=s;w['per_car_policies'][c]['goal']=s+len(commands);w['controls'][c]=copy.deepcopy(old['controls'][c][:k])+[['0','0']]*(s-k)+copy.deepcopy(commands);splices.append({'car':c,'actual_source_cycle':k,'old_start':previous,'new_start':s,'old_goal':goal,'new_goal':s+len(commands),'deleted_wait':previous-s,'exact_v_delta':[0,0],'source_position_error':ep,'source_yaw_error':et,'tail_length':L,'full_tail_body_error_bound':error,'retained_service_K_id':content_id(commands),'inclusion':'Q_wait with exact v/delta -> unchanged stopped turn/real service input; wait zero preserves pose; SE2 full tail bound <.025; no service or steering control removed'})
 w['release_cuts'][outgoing]=start
 if not same:
  w['lease_groups'][group_for(w,outgoing)]['window'][1]=start
  for c in changed:
   g=group_for(w,c);assert w['lease_groups'][g]['members']==[c],'external propagation family requires singleton successor group';w['lease_groups'][g]['window']=[w['per_car_policies'][c]['service_start'],w['per_car_policies'][c]['goal']]
 B=max(goals(w).values());w['B0']=B
 for c in CARS:w['controls'][c]=w['controls'][c][:B]+[['0','0']]*max(0,B-len(w['controls'][c]))
 w['owners']=[None]*B
 for c,(a,b) in owner_intervals(w).items():
  for t in range(a,b):assert w['owners'][t] is None;w['owners'][t]=c
 w['b_i']=goals(w);w['goal_permanent']={c:[w['b_i'][c],'infinity'] for c in CARS};w['inner_version']=old['inner_version']+1;w['release_count']=old['release_count']+1;w['parent']=content_id(old);w['last_splices']=splices
 validate_prefix(old,w,k);intervals_valid(w);delta=deadlines(old,w,k);return w,{'outgoing':outgoing,'incoming':incoming,'kind':'internal' if same else 'external','start':start,'splices':splices,'deadlines':delta,'changed_cars':changed}
def states_v(c,t):
 from canonical import states
 return states()[t]['v'][c]
