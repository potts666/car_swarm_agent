"""Finite geometric path search; no whole-task witness or optimizer reads."""
import hashlib,itertools,json,math,sys,time
from fractions import Fraction as F
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'sources'))
import construct_certificate as geo
from interval_source import I,Q

def read(n):return json.loads((ROOT/n).read_text())
def write(n,v):
 p=ROOT/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,default=str)+'\n')
def identity(v):return hashlib.sha256(json.dumps(v,sort_keys=True,default=str).encode()).hexdigest()
def sqrti(q):
 q=F(q);n=math.isqrt(q.numerator*Q*Q//q.denominator)
 return I.raw(n,n+1)
def interval(lo,hi):return I.raw(I(lo).l,I(hi).u)
def parked(task,frame,done=False):
 p=task['goal' if done else 'initial'];x,y=map(F,p[:2])
 return geo.body(I(frame*x),I(frame*y),I(1),I(0),frame)
def segments(task,frame,radius):
 x,y=map(F,task['initial'][:2]);x*=frame;y*=frame
 endx=frame*F(task['goal'][0]);D=sqrti(12*radius-9);pre=I(-23)-D;endpre=I(endx)-D
 s=-1 if y>0 else 1
 return [('approach_initial','line',I(x),I(y),pre,I(y),0),
 ('merge_first','first',pre,I(y),I(-23),I(0),s),
 ('merge_second','second',pre,I(y),I(-23),I(0),s),
 ('crossing','line',I(-23),I(0),endpre,I(0),0),
 ('parking_first','first',endpre,I(0),I(endx),I(y),-s),
 ('parking_second','second',endpre,I(0),I(endx),I(y),-s)]
def moving(segment,lo,hi,radius,frame):
 name,kind,x0,y0,x1,y1,sign=segment;t=interval(lo,hi)
 if kind=='line':x,y,co,si=x0+(x1-x0)*t,y0,I(1),I(0)
 else:
  ht=I(3)/sqrti(12*radius-9);r=ht*t;r2=r*r
  co,si=(I(1)-r2)/(I(1)+r2),2*r/(I(1)+r2)
  if kind=='first':x,y=x0+radius*si,y0+sign*radius*(I(1)-co)
  else:x,y=x1-radius*si,y1-sign*radius*(I(1)-co)
  si=sign*si
 return geo.body(x,y,co,si,frame),(co,si)
def certify(segment,radius,frame,other=None,strict=F(1,10000)):
 stack=[(F(0),F(1),0)];leaves=[]
 while stack:
  lo,hi,depth=stack.pop();b,rot=moving(segment,lo,hi,radius,frame)
  margin=geo.road_clearance(b) if other is None else geo.pair_clearance(b,other,rot)-geo.SAFE
  if margin>=strict:leaves.append([str(lo),str(hi),str(margin)])
  elif depth<13:
   mid=(lo+hi)/2;stack.extend(((mid,hi,depth+1),(lo,mid,depth+1)))
  else:
   mid=(lo+hi)/2;point,rr=moving(segment,mid,mid,radius,frame)
   return {'passed':False,'segment':segment[0],'interval':[str(lo),str(hi)],'lower_margin':str(margin),'point_lower_margin':str(geo.road_clearance(point) if other is None else geo.pair_clearance(point,other,rr)-geo.SAFE),'reason':'separation not certified; conservative dependency, not proof of collision'}
 leaves.sort(key=lambda x:F(x[0]));assert leaves[0][0]=='0' and leaves[-1][1]=='1';assert all(a[1]==b[0] for a,b in zip(leaves,leaves[1:]))
 return {'passed':True,'segment':segment[0],'minimum_margin':min(F(x[2]) for x in leaves),'leaves':leaves}
def graph_cycle(cars,edges):
 adj={c:[] for c in cars}
 for a,b in edges:adj[a].append(b)
 def visit(c,path):
  if c in path:return path[path.index(c):]+[c]
  for n in adj[c]:
   r=visit(n,path+[c])
   if r:return r
 for c in cars:
  r=visit(c,[])
  if r:return r
 return None

def run():
 tasks=read('local_tasks.json');cfg=read('vehicle_configs.json');candidates=[]
 for radius in (6,5):
  cert={};edges=[];roads=[]
  for car,task in tasks.items():
   frame=cfg['frames'][car];segs=segments(task,frame,radius)
   roads.extend({'car':car,**certify(seg,radius,frame)} for seg in segs)
   for other,ot in tasks.items():
    if other==car:continue
    for done in (False,True):
     checks=[certify(seg,radius,frame,parked(ot,cfg['frames'][other],done)) for seg in segs]
     ok=all(q['passed'] for q in checks)
     edge=(car,other) if done else (other,car)
     if not ok:edges.append(edge)
     cert[f'{car}/{other}/{"goal" if done else "initial"}']={'passed':ok,'dependency':None if ok else edge,'segments':checks}
  edges=sorted(set(edges));cycle=graph_cycle(tasks,edges)
  item={'radius':radius,'roads_passed':all(x['passed'] for x in roads),'roads':roads,'dependencies':cert,'edges':edges,'cycle':cycle,'DAG_passed':cycle is None and all(x['passed'] for x in roads)}
  write(f'results/map_radius_{radius}.json',item);candidates.append({k:v for k,v in item.items() if k not in ('roads','dependencies')})
  print(json.dumps(candidates[-1]),flush=True)
 write('results/map_search.json',{'candidates':candidates,'optimizer_calls':0})
if __name__=='__main__':run()
