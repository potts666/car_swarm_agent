"""State-derived independent requests and finite physically executable schedules."""
import copy,itertools
from common import *
from physics import propagate,body
from initialize import trapezoid

def advance(v,commands):
 distance=F(0)
 for a,w in commands:
  a=F(a);distance+=H*(2*v+H*a)/2;v+=H*a
 return distance,v

def resource_intent(plan,state,c):
 k=state['cycle'];p=state['poses'][c];v=state['v'][c];d=state['delta'][c];policy=plan['per_car_policies'][c];end=policy['approach_end']
 assert k<end and d==0
 commands=plan['controls'][c][k:end]+plan['paths'][c]['controls'][end:];intervals=[];minimum=F(100)
 for j,u in enumerate(commands):
  a,w=map(F,u);nv,nd=v+H*a,d+H*w;assert -1<=nv<=2 and abs(nd)<=F(1,2) and max(abs(a),abs(w))<=1
  nodes=propagate(p,v,nv,d,nd,direction(c),4)
  for t in range(4):
   box=m.axis_box(body(nodes[t],c)+body(nodes[t+1],c));err=TUBE+F(15)*(H/4)**2/8
   margin=F(m.road(box,[[m.I(q).l for q in r] for r in ROADS]),m.Q)-err;assert margin>=0;minimum=min(minimum,margin)
   if F(box[0],m.Q)-err<=16 and F(box[1],m.Q)+err>=-16:intervals.append([F(k+j)+F(t,4),F(k+j)+F(t+1,4)])
  p=nodes[-1];v,d=nv,nd
 assert intervals and v==d==0
 return {'car':c,'source_cycle':k,'source_pose':pose_rows(state['poses'][c]),'source_v':state['v'][c],'source_delta':state['delta'][c],'commands':commands,'resource_interval':[intervals[0][0],intervals[-1][1]],'shared_key':'core_x_closed_-16_16','road_margin':minimum,'private_physics_passed':True,'construction':'remaining actual approach + own certified service, future identity wait removed','earliest_scope':'fixed certified connection family, no whole-map time-optimality claim'}

def request_conflicts(intents):
 conflicts=[]
 for a,b in itertools.combinations(intents,2):
  x,y=a['resource_interval'],b['resource_interval'];overlap=[max(F(x[0]),F(y[0])),min(F(x[1]),F(y[1]))]
  if a['shared_key']==b['shared_key'] and overlap[0]<=overlap[1]:conflicts.append({'actors':[a['car'],b['car']],'overlap':overlap})
 return conflicts

def approve_joint_requests(intents):
 assert not request_conflicts(intents),'independent approvals overlap one exclusive resource'
 return True

def active_pairs(plan,state):
 k=state['cycle'];active=[]
 for c in CARS:
  a=F(plan['controls'][c][k][0]);progress=abs(H*(2*state['v'][c]+H*a)/2)
  if state['v'][c]!=0 and progress>=F(1,50) and k<plan['per_car_policies'][c]['approach_end'] and state['delta'][c]==0:active.append(c)
 pairs=[p for p in itertools.combinations(active,2) if direction(p[0])!=direction(p[1])]
 intents={c:resource_intent(plan,state,c) for c in active} if pairs else {}
 events=[]
 for conflict in request_conflicts(list(intents.values())):
  a,b=conflict['actors']
  if (a,b) not in pairs:continue
  events.append({'actors':[a,b],'overlap':conflict['overlap'],'intents':[intents[a],intents[b]],'sort':[conflict['overlap'][0],a,b]})
 return sorted(events,key=lambda e:e['sort']),active

def propose(old,state,passing,yielding,brake=False):
 k=state['cycle'];w=copy.deepcopy(old);paths=w['paths'];order=[passing,yielding]+[c for c in old['order'] if c not in (passing,yielding)]
 prep={c:[tuple(map(F,u)) for u in old['controls'][c][:old['per_car_policies'][c]['approach_end']]] for c in CARS}
 starts={passing:len(prep[passing])};first_out=starts[passing]+paths[passing]['ranges']['crossing'][1]-len(prep[passing]);braking=None
 if brake:
  v=state['v'][yielding];mstop=math.ceil(abs(v)/H);a=-v/(mstop*H);stop=[(a,F(0))]*mstop
  prefix=prep[yielding][:k];remaining_distance,_=advance(v,prep[yielding][k:]);stop_distance,_=advance(v,stop);resume=trapezoid(remaining_distance-stop_distance)
  wait=first_out-k-mstop;assert wait>=1
  prep[yielding]=prefix+stop+[(F(0),F(0))]*wait+resume
  braking={'from_actual_v':v,'m':mstop,'a':a,'signed_distance':stop_distance,'stop_phase':k+mstop,'release_phase':first_out,'resume_start':first_out,'resume_end':len(prep[yielding])}
 starts[yielding]=max(first_out,len(prep[yielding]))
 previous=yielding
 for c in order[2:]:starts[c]=starts[previous]+paths[previous]['ticks']-old['per_car_policies'][previous]['approach_end'];previous=c
 goals={c:starts[c]+paths[c]['ticks']-old['per_car_policies'][c]['approach_end'] for c in CARS};B=max(goals.values());controls={}
 for c in CARS:
  oldend=old['per_car_policies'][c]['approach_end'];assert starts[c]>=len(prep[c]);controls[c]=prep[c]+[(F(0),F(0))]*(starts[c]-len(prep[c]))+[tuple(map(F,u)) for u in paths[c]['controls'][oldend:]]+[(F(0),F(0))]*(B-goals[c])
  assert controls[c][:k]==[tuple(map(F,u)) for u in old['controls'][c][:k]]
 owners=[None]*B
 # First passing car releases at its outside-core straight endpoint. The next
 # grant can include its safe preparation/resume before actual core entry.
 boundaries=[starts[passing],first_out]+[goals[c] for c in order[1:]]
 for i,c in enumerate(order):
  for t in range(boundaries[i],boundaries[i+1]):owners[t]=c
 assert owners[:k]==old['owners'][:k]
 w.update(controls=controls,owners=owners,order=order,B0=B,b_i=goals,parent=content_id(old),goal_permanent={c:[goals[c],'infinity'] for c in CARS},encounter={'source_cycle':k,'passing':passing,'yielding':yielding,'first_out':first_out,'first_service_start':starts[passing],'yield_service_start':starts[yielding],'gate_path_phase':paths[passing]['ranges']['crossing'][1],'braking':braking})
 w['per_car_policies']={c:{'start':0,'approach_end':len(prep[c]),'service_start':starts[c],'goal':goals[c],'edge_id':content_id(controls[c])} for c in CARS}
 return w

def deadline_check(old,new,k):
 oldb={c:max(0,old['per_car_policies'][c]['goal']-k) for c in CARS};newb={c:max(0,new['per_car_policies'][c]['goal']-k) for c in CARS}
 return {'old_b':oldb,'new_b':newb,'block_max_old':max(oldb.values()),'block_max_new':max(newb.values()),'block_passed':max(newb.values())<=max(oldb.values()),'per_car_passed':all(newb[c]<=oldb[c] for c in CARS),'delays':{c:newb[c]-oldb[c] for c in CARS if newb[c]>oldb[c]}}

def score(plan,k):
 v={c:F(0) for c in CARS}
 for t in range(k):
  for c in CARS:v[c]+=H*F(plan['controls'][c][t][0])
 progress={c:abs(H*(2*v[c]+H*F(plan['controls'][c][k][0]))/2) for c in CARS};active=[c for c,p in progress.items() if p>=F(1,50)];P=sum(progress[c] for c in active);J=sum(F(a)**2+F(w)**2 for c in CARS for a,w in plan['controls'][c][k:])
 return {'A':len(active),'active':active,'P':P,'B':plan['B0']-k,'J':J}
