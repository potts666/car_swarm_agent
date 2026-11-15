"""Closed-interval occupancy directory with permanent goals and intent versions."""
import itertools
from common import *
from physics import propagate,body

def overlap(a,b):return all(a[2*j]<=b[2*j+1] and b[2*j]<=a[2*j+1] for j in range(2))
def keys(box):
 return [(x,y) for x in range(math.floor(box[0]/8),math.floor(box[1]/8)+1) for y in range(math.floor(box[2]/8),math.floor(box[3]/8)+1)]
class Directory:
 def __init__(self):self.rows={};self.versions={};self.events=[]
 def register(self,car,box,start,end,kind):
  kk=keys(box)
  for key in kk:
   self.rows.setdefault(key,[]).append((car,start,end,kind));self.versions[key]=self.versions.get(key,0)+1
  self.events.append({'event':'register_before_query','car':car,'kind':kind,'start':start,'end':end,'keys':kk})
  return kk
 def query(self,car,box,start,end):
  kk=keys(box);owners=sorted({c for key in kk for c,lo,hi,kind in self.rows.get(key,[]) if c!=car and lo<=end and start<=hi})
  out={'car':car,'keys':kk,'owners':owners,'versions':{str(key):self.versions.get(key,0) for key in kk}}
  self.events.append({'event':'query','snapshot':out});return out
 def commit(self,snap):
  kk=sorted(map(lambda s:tuple(map(int,s[1:-1].split(','))),snap['versions']))
  valid=all(self.versions.get(k,0)==snap['versions'][str(k)] for k in kk)
  self.events.append({'event':'commit' if valid else 'reject_version','car':snap['car'],'sorted_locks':kk,'versions':snap['versions'],'owners':snap['owners']})
  return valid

def records(plan,kind):
 p={c:tuple(m.I(F(q)) for q in TASKS[c]['initial'][:3]) for c in CARS};v={c:F(0) for c in CARS};d=v.copy();out=[]
 # Single chord uses global acceleration corner bound; no center-only compression.
 inflate=SAFE/2+TUBE+F(15)*H**2/8+F(1,100)
 for k in range(plan['B0']):
  for c in CARS:
   a,w=map(F,plan['controls'][c][k]);nv,nd=v[c]+H*a,d[c]+H*w;ns=propagate(p[c],v[c],nv,d[c],nd,direction(c),1);bb=m.axis_box(body(ns[0],c)+body(ns[-1],c));box=[F(bb[0],m.Q)-inflate,F(bb[1],m.Q)+inflate,F(bb[2],m.Q)-inflate,F(bb[3],m.Q)+inflate]
   out.append({'car':c,'box':box,'start':k,'end':k+1,'kind':kind});p[c]=ns[-1];v[c],d[c]=nv,nd
 for c in CARS:
  bb=m.axis_box(body(p[c],c));box=[F(bb[0],m.Q)-inflate,F(bb[1],m.Q)+inflate,F(bb[2],m.Q)-inflate,F(bb[3],m.Q)+inflate]
  out.append({'car':c,'box':box,'start':plan['per_car_policies'][c]['goal'],'end':float('inf'),'kind':'permanent_goal'})
 return out

def certify(old,new,participants=None,cycle=0):
 directory=Directory();rows=[]
 for plan,kind in ((old,'old_tail'),(new,'proposal_intent'),(new,'receiver_suffix')):rows.extend(records(plan,kind))
 for r in rows:directory.register(**r)
 # Query only after all intents have registered. Old rows remain during prepare.
 queries=[directory.query(r['car'],r['box'],r['start'],r['end']) for r in rows]
 prepared=[q for r,q in zip(rows,queries) if r['kind']=='proposal_intent' and r['start']==cycle];assert len(prepared)==4
 assert all(directory.commit(q) for q in prepared)
 discovered=set();potential=set()
 for r,q in zip(rows,queries):
  for c in q['owners']:discovered.add(tuple(sorted((r['car'],c))))
 # Offline judge enumerates all six pairs across old/proposed/receiver
 # sweeps, neighboring closed time intervals, and permanent future holds.
 bytime={};permanent=[]
 for r in rows:
  if r['kind']=='permanent_goal':permanent.append(r)
  else:bytime.setdefault(r['start'],[]).append(r)
 for k in range(max(bytime)+2):
  rr=bytime.get(k,[])+bytime.get(k-1,[])+[r for r in permanent if r['start']<=k+1]
  for a,b in itertools.combinations(rr,2):
   if a['car']!=b['car'] and a['start']<=b['end'] and b['start']<=a['end'] and overlap(a['box'],b['box']):potential.add(tuple(sorted((a['car'],b['car']))))
 assert potential<=discovered
 # Changed schedule endpoint reservations entail all four participants.
 closure=set(c for c in CARS if old['per_car_policies'][c]!=new['per_car_policies'][c])
 if participants is None:assert closure==set(CARS)
 else:closure=set(participants)
 return {'passed':True,'grid_m':8,'records':rows,'queries':queries,'potential_pairs':sorted(potential),'discovered_pairs':sorted(discovered),'closure':sorted(closure),'events':directory.events,'permanent_goals':True,'all_proposals_and_receivers_registered_before_query':True}
