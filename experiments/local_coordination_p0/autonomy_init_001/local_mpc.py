"""Per-car 64-variable conic update for the certified initial straight phases."""
import time
from common import *

def prepare(plan,state,c):
 import numpy as np
 k=state['cycle'];ref=[tuple(map(F,u)) for u in plan['controls'][c][k:k+32]];assert len(ref)==32
 policy=plan['per_car_policies'][c];end=policy.get('approach_end',policy['start']+plan['paths'][c]['ranges']['approach_initial'][1])
 assert policy['start']<=k<end and state['delta'][c]==0 and state['poses'][c][2].abs_upper()==0
 n=min(32,end-k);assert n>=10
 v=state['v'][c];d=state['delta'][c];values=[]
 for a,w in ref:values.extend((a,w))
 rows=[];bounds=[];labels=[];eq=[];elabels=[]
 def row(vals,b,label):rows.append([F(x) for x in vals]);bounds.append(F(b));labels.append(label)
 for j in range(64):
  rr=[F(0)]*64;rr[j]=1;row(rr,1-values[j],f'input+:{j}');row([-x for x in rr],1+values[j],f'input-:{j}')
 for j in range(32):
  rr=[F(0)]*64;rr[2*j+1]=1;eq.append(rr);elabels.append(f'fixed_steering:{j}')
  if j>=n or (k==0 and j==0):
   rr=[F(0)]*64;rr[2*j]=1;eq.append(rr);elabels.append(f'fixed_acceleration:{j}')
  rr=[F(0)]*64
  for t in range(j+1):rr[2*t]=H
  v+=H*ref[j][0];row(rr,2-v,f'v+:{j}');row([-x for x in rr],v+1,f'v-:{j}')
  rr=[F(0)]*64
  for t in range(j+1):rr[2*t]=H*H*(F(j-t)+F(1,2))
  row(rr,F(1,100),f'physical_x+:{j}');row([-x for x in rr],F(1,100),f'physical_x-:{j}')
 for power in (0,1):
  rr=[F(0)]*64
  for j in range(n):rr[2*j]=F(j**power)
  eq.append(rr);elabels.append('terminal_v_exact' if power==0 else 'terminal_x_exact')
 first=1 if k==0 else 0
 T=[]
 for j in range(first,n-2):
  col=[0]*64
  for z,t in ((j,1),(j+1,-2),(j+2,1)):col[2*z]=t
  T.append(col)
 assert all(sum(e[j]*col[j] for j in range(64))==0 for e in eq for col in T)
 assert all(b>=0 for b in bounds)
 identity={'actor':c,'cycle':k,'state':{'poses_Q':pose_rows(state['poses'][c]),'v':state['v'][c],'delta':state['delta'][c]},'parent':content_id(plan),'reference':values,'rows':rows,'bounds':bounds,'E':eq,'T_columns':T,'delta_grid':'1/1000000000000','quant_error_bound':'1/500000000000','receiver':{'rv':0,'rdelta':0,'terminal_state':'exact straight x/v and fixed yaw/delta; original certified suffix'},'labels':labels,'equality_labels':elabels,'derivative':'exact signed straight ZOH; stopped steering retains pose; all steering perturbations zero','n_straight':n,'trust':'1/1000'}
 identity['receiving_certificate']={'parent':identity['parent'],'phase':k+32,'deadline':plan['per_car_policies'][c]['goal'],'K_id':content_id(plan['controls'][c][k+32:]),'phase_domain_id':content_id(read(HERE/'results/phase_receivers.json')[c]),'rv':0,'rdelta':0}
 identity['model_id']=content_id(identity)
 return json.loads(json.dumps(identity,default=str))

def assemble(job):
 import clarabel,numpy as np,scipy.sparse as sp
 # Serialized rationals must pass through Fraction before entering float arrays.
 rows=np.array([[float(F(x)) for x in row] for row in job['rows']]);b=np.array([float(F(x)) for x in job['bounds']]);E=np.array([[float(F(x)) for x in row] for row in job['E']]);ref=np.array([float(F(x)) for x in job['reference']])
 A=sp.vstack([sp.csc_matrix(E),sp.csc_matrix(rows),sp.csc_matrix(np.vstack([np.zeros((1,64)),-np.eye(64)]))],format='csc');rhs=np.concatenate([np.zeros(len(E)),b,[.001],np.zeros(64)])
 cones=[clarabel.ZeroConeT(len(E)),clarabel.NonnegativeConeT(len(b)),clarabel.SecondOrderConeT(65)]
 settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=200;settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-10;settings.tol_feas=1e-10
 return sp.eye(64,format='csc')*2,2*ref,A,rhs,cones,settings

def native(job):
 import clarabel
 start=time.monotonic();P,q,A,rhs,cones,settings=assemble(job)
 sol=clarabel.DefaultSolver(P,q,A,rhs,cones,settings).solve()
 return {'model_id':job['model_id'],'status':str(sol.status),'x':list(sol.x),'s':list(sol.s),'z':list(sol.z),'obj_val':sol.obj_val,'iterations':sol.iterations,'r_prim':sol.r_prim,'r_dual':sol.r_dual,'solve_time':sol.solve_time,'wall':time.monotonic()-start}

def worker(conn):
 try:
  raw=conn.recv_bytes();job=json.loads(raw);out=native(job);out['received_bytes']=len(raw);data=json.dumps(out).encode();conn.send_bytes(data)
 except Exception as e:conn.send_bytes(json.dumps({'status':'worker_error','error':repr(e)}).encode())
 finally:conn.close()

def admit(job,raw):
 assert raw.get('model_id')==job['model_id'];assert raw['status'] in ('Solved','AlmostSolved')
 ref=list(map(F,job['reference']));rows=[[F(x) for x in r] for r in job['rows']];bounds=list(map(F,job['bounds']));E=[[F(x) for x in r] for r in job['E']];T=job['T_columns'];start=0 if job['cycle'] else 1
 for alpha in (F(1),F(1,2),F(1,4)):
  # Invert the consecutive second-difference basis; round its coordinates,
  # never absolute command components. Every column is in ker E.
  zz=[]
  for t in range(len(T)):
   z=alpha*F(str(raw['x'][2*(start+t)]))+(2*zz[t-1] if t>=1 else 0)-(zz[t-2] if t>=2 else 0);zz.append(z)
  ints=[round(z*10**12) for z in zz];delta=[sum(F(T[t][j]*ints[t],10**12) for t in range(len(T))) for j in range(64)]
  assert all(sum(e[j]*delta[j] for j in range(64))==0 for e in E)
  if any(sum(r[j]*delta[j] for j in range(64))>b for r,b in zip(rows,bounds)):continue
  if sum(x*x for x in delta)>F(1,1000)**2:continue
  q=[x+y for x,y in zip(ref,delta)];J=sum(x*x for x in q);J0=sum(x*x for x in ref)
  if J>=J0:continue
  return {'accepted':True,'alpha':alpha,'commands':[[q[2*j],q[2*j+1]] for j in range(32)],'delta':delta,'integer_coordinates':ints,'model_id':job['model_id'],'quantizer_id':content_id({'E':job['E'],'T':T,'q0':job['reference'],'Delta':job['delta_grid']}),'J_before':J0,'J_after':J,'first_changed':q[:2]!=ref[:2],'all_equalities_exact':True}
 return {'accepted':False,'reason':'no fixed damping factor passed exact rows, trust and strict input cost','model_id':job['model_id']}
