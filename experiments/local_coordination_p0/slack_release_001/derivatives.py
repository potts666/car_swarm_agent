"""Reused validated interval variational ODE composed over actual ZOH inputs."""
import importlib.util,time
from common import *
from physics import propagate
from curve_bounds import profiles
spec=importlib.util.spec_from_file_location('curve_variational',HERE/'sources/certify_interfaces.py');builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)

def build(plan,state,car,sub=64):
 start=time.monotonic();p=state['poses'][car];v=state['v'][car];d=state['delta'][car];S=[[] for _ in range(5)];prefix=[];knots=[];nominal=[pose_rows(p)];single=[]
 for j,u in enumerate(plan['controls'][car][state['cycle']:state['cycle']+32]):
  a,w=map(F,u);nv,nd=v+H*a,d+H*w;one={'scale':m.Q,'direction_sign':direction(car),'knots':[[str(x) for x in (v,nv,d,nd)]],'pose_nodes_Q':[pose_rows(p)]};raw=builder.interval_jacobian(m,one,0,{},sub);J=[[m.I(0) for _ in range(7)] for _ in range(5)]
  for r,c,lo,hi in raw['nonzero_intervals']:J[r][c]=m.I.raw(m.I(F(lo,raw['scale'])).l,m.I(F(hi,raw['scale'])).u)
  S=[[sum((J[r][q]*S[q][t] for q in range(5)),m.I(0)) for t in range(2*j)]+[J[r][5],J[r][6]] for r in range(5)]
  prefix.append([[[x.l,x.u] for x in row]+[[0,0]]*(64-len(row)) for row in S]);single.append(raw);knots.append([v,nv,d,nd]);p=propagate(p,v,nv,d,nd,direction(car),16)[-1];nominal.append(pose_rows(p));v,d=nv,nd
 return {'actor':car,'cycle':state['cycle'],'source':{'pose_Q':pose_rows(state['poses'][car]),'v':state['v'][car],'delta':state['delta'][car]},'subdivisions':sub,'single_step':single,'prefix':prefix,'poses_Q':nominal,'knots':knots,'bounds':profiles(knots),'wall':time.monotonic()-start}

def matrix(cert,index=-1):return [[m.I.raw(*q) for q in row] for row in cert['prefix'][index]]

def get(plan,state,car,sub=64):
 from plan_io import save,load
 key=content_id({'car':car,'cycle':state['cycle'],'pose':pose_rows(state['poses'][car]),'v':state['v'][car],'delta':state['delta'][car],'history':state['history'],'controls':plan['controls'][car][state['cycle']:state['cycle']+32],'sub':sub,'builder_sha':sha(HERE/'sources/certify_interfaces.py')})
 name=f'results/derivatives/{key}.json.gz';p=HERE/name
 if p.exists():return load(name)
 result=build(plan,state,car,sub);result.pop('wall');result['source_key']=key;save(name,result);return load(name)
