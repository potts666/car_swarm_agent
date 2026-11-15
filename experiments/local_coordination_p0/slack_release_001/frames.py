"""Finite phase unions with shared geometric certificates and discovered edits."""
import copy,itertools,time
from functools import lru_cache
from common import *
from canonical import base,states,index,exit_phase
from physics import body
from plan_io import save,load
from discovery import Directory
RESERVE=TUBE+F(1,40)+15*H*H/8
PAIR={};ROAD={};STATIC={}
@lru_cache(None)
def shape(c,a,b):
 pts=[]
 for t in (a,b):pts+=body(tuple(m.I.raw(*q) for q in states()[t]['poses_Q'][c]),c)
 return pts
@lru_cache(None)
def box(c,a,b):return m.axis_box(shape(c,a,b))
def choices(slot,c,t):
 lo,hi=slot['start_range'];return sorted({(index(c,t,s),index(c,t+1,s)) for s in range(lo,hi+1)})
def pair(c,a,b,d,e,f):
 key=(c,a,b,d,e,f)
 if key in PAIR:return PAIR[key]
 B=shape(c,a,b);C=shape(d,e,f);bb=box(c,a,b);cc=box(d,e,f)
 aa=[(F(cc[0]-bb[1],m.Q),(F(1),F(0)),F(bb[1]+cc[0],2*m.Q)),(F(bb[0]-cc[1],m.Q),(F(-1),F(0)),-F(bb[0]+cc[1],2*m.Q)),(F(cc[2]-bb[3],m.Q),(F(0),F(1)),F(bb[3]+cc[2],2*m.Q)),(F(bb[2]-cc[3],m.Q),(F(0),F(-1)),-F(bb[2]+cc[3],2*m.Q))];best=max(aa,key=lambda z:z[0])
 if best[0]<2:
  p=tuple(m.I.raw(*q) for q in states()[a]['poses_Q'][c]);q=tuple(m.I.raw(*z) for z in states()[e]['poses_Q'][d])
  for nx,ny in m.normed_axes(p)+m.normed_axes(q):
   for sg in (-1,1):
    x,y=sg*nx,sg*ny;upper=max((u*x+v*y).u for u,v in B);lower=min((u*x+v*y).l for u,v in C);value=F(lower-upper,m.Q)
    if value>best[0]:best=(value,(x,y),F(lower+upper,2*m.Q))
 PAIR[key]={'pair':[c,d],'indices':[a,b,e,f],'gap':best[0],'n':best[1],'beta':best[2],'margin':best[0]-2*RESERVE-SAFE};return PAIR[key]
def slot_descriptors(plan,initial=False):
 out={};order=plan['order']
 for c in CARS:
  p=plan['per_car_policies'][c];g=next(v for v in plan['lease_groups'].values() if c in v['members']);starts=[p['service_start']]*2;deadline=p['goal']
  if initial and c==order[1]:starts=[exit_phase(plan,order[0]),p['service_start']]
  out[c]={'actor':c,'route_family':content_id(base()['paths'][c]),'start_range':starts,'deadline':deadline,'lease':g,'body_domain':'1/40','source_receiver':{'rP':'1/400','rPsi':'1/100000','v':0,'delta':0},'permanent_goal':TASKS[c]['goal'],'hold':'infinity'}
 return out

def certify(plan,slots=None):
 started=time.monotonic();slots=slots or slot_descriptors(plan);descriptor={'slots':slots,'leases':plan['lease_groups'],'model':'same physical signed ZOH; finite canonical phase branch +.025 body node variation+.05 tube+15h²/8','permanent_goals':list(CARS)};fid=content_id(descriptor);file=HERE/f'results/frames/{fid}.json'
 if file.exists():return read(file)
 N=max(x['deadline'] for x in slots.values());links=[];records=[];minimum={'road':F(100),'pair':F(100),'group_resource':F(100)};convex_failure=None
 for t in range(N):
  ch={c:choices(slots[c],c,t) for c in CARS}
  for c in CARS:
   for a,b in ch[c]:
    key=(c,a,b)
    if key not in ROAD:ROAD[key]=F(m.road(box(c,a,b),[[m.I(x).l for x in r] for r in ROADS]),m.Q)-RESERVE
    assert ROAD[key]>=0,('branch road',c,a,b,ROAD[key]);minimum['road']=min(minimum['road'],ROAD[key]);lo,hi=slots[c]['lease']['window']
    if not lo<=t<hi:
     bb=box(c,a,b);gap=max(-16-F(bb[1],m.Q)-RESERVE,F(bb[0],m.Q)-RESERVE-16);assert gap>0,('group resource',c,t,gap);minimum['group_resource']=min(minimum['group_resource'],gap)
   if t%8==0:
    points=[p for r in range(t,min(t+8,N)) for a,b in choices(slots[c],c,r) for p in shape(c,a,b)];bb=m.axis_box(points);inflate=RESERVE+SAFE/2;records.append({'car':c,'box':[F(bb[0],m.Q)-inflate,F(bb[1],m.Q)+inflate,F(bb[2],m.Q)-inflate,F(bb[3],m.Q)+inflate],'start':t,'end':min(t+8,N),'kind':'all_phase_branches_receivers_K'})
   if convex_failure is None and len(ch[c])>1:
    bb=m.axis_box([p for a,b in ch[c] for p in shape(c,a,b)]);r=F(m.road(bb,[[m.I(x).l for x in z] for z in ROADS]),m.Q)-RESERVE
    if r<0:convex_failure={'actor':c,'cycle':t,'convex_hull_road_margin':r,'actual_branch_road_min':min(ROAD[(c,a,b)] for a,b in ch[c]),'repair':'keep finite phase union, never require convex-hull road inclusion'}
  for c,d in itertools.combinations(CARS,2):
   ids=[]
   for a,b in ch[c]:
    for e,f in ch[d]:
     cert=pair(c,a,b,d,e,f);assert cert['margin']>=0,('union pair',t,c,d,cert);minimum['pair']=min(minimum['pair'],cert['margin']);ids.append(content_id(cert))
   links.append({'cycle':t,'pair':[c,d],'face_ids':ids})
 for c in CARS:
  a=base()['per_car_policies'][c]['goal'];bb=box(c,a,a);inflate=RESERVE+SAFE/2;records.append({'car':c,'box':[F(bb[0],m.Q)-inflate,F(bb[1],m.Q)+inflate,F(bb[2],m.Q)-inflate,F(bb[3],m.Q)+inflate],'start':min(slots[c]['start_range'])+a-base()['per_car_policies'][c]['service_start'],'end':float('inf'),'kind':'permanent_goal'})
 directory=Directory()
 for r in records:directory.register(**r)
 queries=[directory.query(r['car'],r['box'],r['start'],r['end']) for r in records];assert all(directory.commit(q) for q in queries)
 save(f'results/frames/{fid}_directory.json.gz',{'records':records,'queries':queries,'events':directory.events,'all_registered_before_queries':True,'permanent_goals':True});used={x for r in links for x in r['face_ids']};save(f'results/frames/{fid}_faces.json.gz',{'links':links,'objects':{content_id(v):v for v in PAIR.values() if content_id(v) in used}})
 out={**descriptor,'outer_frame_id':fid,'outer_slots':{c:content_id(slots[c]) for c in CARS},'passed':True,'minimum':minimum,'valid_through':N,'directory':f'results/frames/{fid}_directory.json.gz','faces':f'results/frames/{fid}_faces.json.gz','construction_wall':time.monotonic()-started,'convex_conservatism_fixture':convex_failure};write(file,out);print('FRAME',fid[:10],'choices',max(x['start_range'][1]-x['start_range'][0]+1 for x in slots.values()),'wall',round(out['construction_wall'],2),flush=True);return out

def bind(plan,frame):
 p=copy.deepcopy(plan);p['outer_frame_id']=frame['outer_frame_id'];p['outer_slots']=frame['outer_slots'];p['frame_file']=f'results/frames/{frame["outer_frame_id"]}.json';return p

def contains(frame,plan):
 for c in CARS:
  slot=frame['slots'][c];policy=plan['per_car_policies'][c];assert slot['start_range'][0]<=policy['service_start']<=slot['start_range'][1] and policy['goal']<=slot['deadline'],('outside published phase/goal',c);assert next(v for v in plan['lease_groups'].values() if c in v['members'])==slot['lease'],('lease changed',c)
 return True

def blocks(plan,k):
 groups=[set([c]) for c in CARS]
 for lease in plan['lease_groups'].values():
  if lease['window'][1]<=k:continue
  members=set(lease['members']);hit=[g for g in groups if g&members];united=set().union(*hit);groups=[g for g in groups if not g&members]+[united]
 return sorted(sorted(g) for g in groups)

def static_clear(done,other):
 key=(done,other)
 if key in STATIC:return STATIC[key]
 a=base()['per_car_policies'][done]['goal'];minimum=F(100);lim=None
 for t in range(base()['per_car_policies'][other]['goal']):
  cert=pair(done,a,a,other,t,t+1)
  if cert['margin']<minimum:minimum=cert['margin'];lim=cert
 STATIC[key]={'passed':minimum>=0,'minimum':minimum,'tight_face':lim,'valid_when':'done actor in permanent .025 goal body domain; other remains in full canonical spatial route corridor at any retiming, plus original tube/chord','interface_id':content_id({'done':done,'other':other,'goal':TASKS[done]['goal'],'route':content_id(base()['paths'][other]),'reserve':RESERVE})};return STATIC[key]

def discover(old,new,oldframe,newframe,k):
 changed={c for c in CARS if oldframe['outer_slots'][c]!=newframe['outer_slots'][c]};participants=set(changed);reasons=[{'added':c,'field':'own outer slot/lease/receiver deadline changed','old':oldframe['slots'][c],'new':newframe['slots'][c]} for c in sorted(changed)];discharged=[]
 if not changed:
  changedowners={c for a,b in zip(old['owners'],new['owners']) if a!=b for c in (a,b) if c};participants|=changedowners;reasons.extend({'added':c,'field':'internal owner partition changes inside same lease'} for c in sorted(changedowners))
 else:
  # All intents/receiver/K/goals were registered by newframe.certify before this query.
  for c in CARS:
   if c in participants:continue
   proofs=[static_clear(c,d) for d in changed if d!=c]
   if old['per_car_policies'][c]['goal']<=k and all(p['passed'] for p in proofs):discharged.append({'car':c,'permanent_goal_still_registered':True,'interfaces':proofs,'outer_slot_unchanged':True})
   else:participants.add(c);reasons.append({'added':c,'field':'changed external future envelope against live old dependency','time':[k,old['per_car_policies'][c]['goal']]})
 return {'participants':sorted(participants),'changed_public_slots':sorted(changed),'reasons':reasons,'discharged_completed_dependencies':discharged,'fixed_point_reached':True,'registered_before_query':True,'blocks_before':blocks(old,k),'blocks_after':blocks(new,k)}

def own_resource_proof(plan):
 minimum=F(100)
 for t in range(plan['B0']):
  for c in CARS:
   if c==plan['owners'][t]:continue
   s=plan['per_car_policies'][c]['service_start'];a,b=index(c,t,s),index(c,t+1,s);bb=box(c,a,b);gap=max(-16-F(bb[1],m.Q)-RESERVE,F(bb[0],m.Q)-RESERVE-16);assert gap>0,('own owner/corridor',c,t,gap);minimum=min(minimum,gap)
 return {'passed':True,'min_nonowner_gap':minimum,'reserve':RESERVE,'all_allowed_MPC_node_bodies_plus_time_chords':True}

def validate_publication(frame):
 assert set(frame['permanent_goals'])==set(CARS),'permanent goal omitted'
 assert set(frame['slots'])==set(frame['outer_slots'])==set(CARS),'frame endpoint omitted'
 assert frame['outer_frame_id']==content_id({k:frame[k] for k in ('slots','leases','model','permanent_goals')}),'mixed outer frame version'
 assert frame['outer_slots']=={c:content_id(frame['slots'][c]) for c in CARS},'mixed outer/per-car slot version'
 return True

def validate_discovery(actual,expected):
 assert set(actual['participants'])==set(expected['participants']),'closure endpoint omitted'
 return True
