"""Whitelisted local route construction and INIT-DAG request arbitration."""
import gzip,heapq
from common import *
from physics import propagate
from map_domain import parked,geo
ALPHA=(F('0.722734247813')+F('0.722734247814'))/2
DELTA=(F('0.422853926132')+F('0.422853926134'))/2
D=F(round(3*math.sqrt(7)*10**12),10**12)

def trapezoid(length):
 n=5+max(5,math.ceil(abs(length)/H));V=F(length)/(H*(n-5))
 u=[(V if j<5 else -V if j>=n-5 else F(0),F(0)) for j in range(n)]
 assert sum(a for a,w in u)==0
 assert H*H*sum((F(n-j)-F(1,2))*a for j,(a,w) in enumerate(u))==length
 assert max(abs(a) for a,w in u)<=1
 return u

def local_path(car,task,cfg):
 sign=cfg['frames'][car];x,y=map(F,task['initial'][:2]);x*=sign;y*=sign
 endx=sign*F(task['goal'][0]);sgn=-1 if y>0 else 1
 commands=[];phases=[];ranges={};d=F(0)
 def add(name,u):ranges[name]=[len(commands),len(commands)+len(u)];commands.extend(u);phases.extend([name]*len(u))
 def steer(name,target):
  nonlocal d
  add(name,[(F(0),(target-d)/(5*H))]*5);d=target
 add('approach_initial',trapezoid(-23-D-x))
 steer('merge_turn_first',sgn*DELTA);add('merge_first',trapezoid(6*ALPHA))
 steer('merge_turn_second',-sgn*DELTA);add('merge_second',trapezoid(6*ALPHA));steer('merge_turn_zero',F(0))
 add('crossing',trapezoid(endx-D+23))
 steer('parking_turn_first',-sgn*DELTA);add('parking_first',trapezoid(6*ALPHA))
 steer('parking_turn_second',sgn*DELTA);add('parking_second',trapezoid(6*ALPHA));steer('parking_turn_zero',F(0))
 p=tuple(m.I(F(q)) for q in task['initial'][:3]);v=d=F(0);refs=[pose_rows(p)];knots=[]
 for a,w in commands:
  nv,nd=v+H*a,d+H*w;assert -1<=nv<=2 and abs(nd)<=F(1,2) and max(abs(a),abs(w))<=1
  p=propagate(p,v,nv,d,nd,sign,4)[-1];knots.append([v,nv,d,nd]);refs.append(pose_rows(p));v,d=nv,nd
 err=sum((p[k]-F(task['goal'][k])).abs_upper() for k in (0,1));yaw=p[2].abs_upper()
 assert v==d==0 and err+RB*yaw<EXTRA
 # SE(2) equivariance bounds all points of the non-singleton source set.
 rP=F(1,100000);rPsi=F(1,10**8);length=sum(abs(H*(v0+v1)/2) for v0,v1,d0,d1 in knots)
 receiver_rP=rP+length*rPsi
 assert receiver_rP+RB*rPsi+err+RB*yaw<F(1,20)
 return {'car':car,'controls':commands,'labels':phases,'ranges':ranges,'poses_Q':refs,'knots':knots,'ticks':len(commands),
 'edge_certificate':{'passed':True,'source_rP':rP,'source_rPsi':rPsi,'rv':0,'rdelta':0,'length':length,'receiver_rP':receiver_rP,'receiver_rPsi':rPsi,'reference_goal_error':err,'reference_yaw_error':yaw,'sweep_body_error_budget':'1/20','flow_proof':'same inputs, signed speed integral: rP_l=rP_0+L_prefix*rPsi; orientation offset constant; terminal exact v=delta=0', 'geometry_certificate':'results/map_radius_6.json'}}

def init_plan(map_config,vehicle_configs,local_tasks,certified_graphs,requests):
 assert certified_graphs['DAG_passed'];cars=list(local_tasks)
 paths={c:local_path(c,local_tasks[c],vehicle_configs) for c in cars}
 edges=certified_graphs['edges'];done=[];queue=[];events=[]
 while len(done)<len(cars):
  ready=[c for c in cars if c not in done and all(a in done for a,b in edges if b==c)]
  assert ready
  chosen=min(ready,key=lambda c:(requests[c],c));events.append({'ready':ready,'chosen':chosen,'requests':requests});done.append(chosen)
 controls={c:[] for c in cars};owners=[];policies={};elapsed=0
 for car in done:
  n=paths[car]['ticks'];policies[car]={'start':elapsed,'goal':elapsed+n,'edge_id':content_id(paths[car])}
  for c in cars:controls[c].extend(paths[c]['controls'] if c==car else [(F(0),F(0))]*n)
  owners.extend([car]*n);elapsed+=n
 assert elapsed==sum(p['ticks'] for p in paths.values())
 manifest={'files':{n:sha(HERE/n) for n in ('map_config.json','vehicle_configs.json','local_tasks.json','requests.json','initialize.py','map_domain.py','common.py','physics.py','sources/interval_source.py','sources/construct_certificate.py')},'parameters':{k:content_id(v) for k,v in dict(map_config=map_config,vehicle_configs=vehicle_configs,local_tasks=local_tasks,certified_graphs=certified_graphs,requests=requests).items()},'historical_whole_witness_read':False}
 return {'controls':controls,'owners':owners,'order':done,'ready_events':events,'B0':elapsed,'b_i':{c:policies[c]['goal'] for c in cars},'per_car_policies':policies,'paths':paths,'source_manifest':manifest,'receivers':{c:paths[c]['edge_certificate'] for c in cars},'goal_permanent':{c:[policies[c]['goal'],'infinity'] for c in cars}}

def run():
 graph=read(HERE/'results/map_radius_6.json')
 # All initial/goal endpoint combinations must be physically separated.
 static=[]
 for a,b in __import__('itertools').combinations(CARS,2):
  for da,db in __import__('itertools').product((False,True),repeat=2):
   gap=geo.pair_clearance(parked(TASKS[a],direction(a),da),parked(TASKS[b],direction(b),db));assert gap>SAFE
   static.append({'pair':[a,b],'done':[da,db],'gap':gap})
 write(HERE/'results/endpoint_certificate.json',{'passed':True,'static':static,'domains':{'rP':'1/100000','rPsi':'1/100000000','v':0,'delta':0}})
 for k,requests in enumerate(read(HERE/'requests.json')):
  w=init_plan(read(HERE/'map_config.json'),CFG,TASKS,graph,requests)
  raw=json.dumps(w,sort_keys=True,default=str).encode();(HERE/f'results/init_{k}.json.gz').write_bytes(gzip.compress(raw,mtime=0))
  write(HERE/f'results/init_{k}_summary.json',{key:w[key] for key in ('order','B0','b_i','per_car_policies','ready_events','source_manifest')})
 print(json.dumps({'passed':True,'optimizer_calls':0,'plans':2}))
if __name__=='__main__':run()
