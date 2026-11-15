"""Independent plan/trace replay. No initialization, selection or solver imports."""
import argparse,gzip,itertools,time
from common import *
from physics import propagate,body

def audit(plan,trace=None,output=None):
 t=time.monotonic();p={c:tuple(m.I(F(q)) for q in TASKS[c]['initial'][:3]) for c in CARS};v={c:F(0) for c in CARS};d=v.copy();minimum={'road':F(100),'pair':F(100),'resource':F(100)};steps=plan['B0'];goals={};cost=F(0);activity=[]
 assert all(len(plan['controls'][c])==steps for c in CARS) and len(plan['owners'])==steps
 traces=iter(gzip.open(trace,'rt')) if trace else None
 for k in range(steps):
  nodes={};active={};commands={c:tuple(map(F,plan['controls'][c][k])) for c in CARS}
  for c,(a,w) in commands.items():
   nv,nd=v[c]+H*a,d[c]+H*w;assert max(abs(a),abs(w))<=1 and -1<=nv<=2 and abs(nd)<=F(1,2)
   nodes[c]=propagate(p[c],v[c],nv,d[c],nd,direction(c),8);active[c]=v[c]!=0 or nv!=0;cost+=a*a+w*w
  for j in range(8):
   bs={c:body(nodes[c][j],c)+body(nodes[c][j+1],c) for c in CARS};boxes={c:m.axis_box(b) for c,b in bs.items()};err={c:TUBE+(F(15)*(H/8)**2/8 if active[c] else 0) for c in CARS}
   for c,b in boxes.items():
    road=F(m.road(b,[[m.I(q).l for q in row] for row in ROADS]),m.Q)-err[c];assert road>=0,(k,c,'road',road);minimum['road']=min(minimum['road'],road)
    if c!=plan['owners'][k]:
     clearance=max(-16-F(b[1],m.Q)-err[c],F(b[0],m.Q)-err[c]-16);assert clearance>0,(k,c,'resource');minimum['resource']=min(minimum['resource'],clearance)
   for a,b in itertools.combinations(CARS,2):
    axes=m.normed_axes(nodes[a][j])+m.normed_axes(nodes[b][j]);gap=F(m.gap(bs[a],bs[b],boxes[a],boxes[b],axes),m.Q)-err[a]-err[b]
    assert gap>=SAFE,(k,a,b,gap);minimum['pair']=min(minimum['pair'],gap)
  aa=[]
  for c,(a,w) in commands.items():
   travel=abs(H*(2*v[c]+H*a)/2)
   if travel>=F(1,50):aa.append(c)
   p[c]=nodes[c][-1];v[c]+=H*a;d[c]+=H*w
   if k+1==plan['per_car_policies'][c]['goal']:
    ep=sum((p[c][j]-F(TASKS[c]['goal'][j])).abs_upper() for j in (0,1));et=p[c][2].abs_upper();assert ep<F(2,25) and et<F(1,100) and v[c]==d[c]==0;goals[c]=k+1
   if c in goals:assert commands[c]==(0,0) or k+1==goals[c]
  activity.append(aa)
  if traces:
   row=json.loads(next(traces));assert row['cycle']==k+1 and row['commands']=={c:list(map(str,u)) for c,u in commands.items()}
   # Independent sub=8 interval may be wider or narrower than execution sub=4.
   for c in CARS:
    for a,b in zip(p[c],row['poses_Q'][c]):assert max(a.l,b[0])<=min(a.u,b[1]),(k,c,'trace mismatch')
   assert row['v']=={c:str(x) for c,x in v.items()} and row['delta']=={c:str(x) for c,x in d.items()}
 assert len(goals)==4
 if traces:
  assert next(traces,None) is None
 out={'passed':True,'steps':steps,'goals':goals,'B_final':0,'all_six_pairs_every_subinterval':True,'subdivisions':8,'minimum':minimum,'cost':cost,'maximum_activity':max(map(len,activity)),'first_two_activity':activity[:2],'wall_seconds':time.monotonic()-t,'optimizer_calls':0}
 if output:write(output,out)
 return out
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('plan');ap.add_argument('--trace');ap.add_argument('--output');args=ap.parse_args();plan=json.loads(gzip.decompress(Path(args.plan).read_bytes()));print(json.dumps(audit(plan,args.trace,args.output),default=str))
