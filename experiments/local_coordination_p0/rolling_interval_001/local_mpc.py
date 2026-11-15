"""Common 64-input nonlinear SOCP, source-bound quantized certificate admission."""
import copy,time
from common import *
from curve_model import evaluate,norm
from quantization import quantize
from physics import propagate
from progression import coordinate

def assemble(job):
 import clarabel,numpy as np,scipy.sparse as sp
 n=65;equal=np.zeros((len(job['lattice']['E']),n));equal[:,:64]=np.array([[float(F(q)) for q in r] for r in job['lattice']['E']]);linear=[];rhs=[];socs=[];soc_rhs=[]
 for row in job['rows']:
  a=[F(q) for q in row['a']]+[F(row['epsilon'])];g=F(row['g']);h=F(row['H'])
  if h:
   assert g<0;normal=-g;ell=np.array([float(q/normal) for q in a]);root=(float(2*h/normal))**.5;A=np.vstack([ell,np.zeros(n),ell]);A[1,64]=-root;socs.append(A);soc_rhs.extend([2.,0.,0.])
  else:linear.append(list(map(float,a)));rhs.append(float(-g))
 # Exact physical trust remains rho. Reserve the common existing4e-12 per
 # input rounding budget in its cone, uniformly across all four arms.
 trust=float(F(job['rho'])-F(32,10**12));rr=[0.]*n;rr[-1]=1.;linear.append(rr);rhs.append(trust)
 cone=np.zeros((65,n));cone[0,64]=-1;cone[1:,:64]=-np.eye(64)
 A=sp.vstack([sp.csc_matrix(equal),sp.csc_matrix(linear),sp.csc_matrix(cone)]+[sp.csc_matrix(x) for x in socs],format='csc');b=np.r_[np.zeros(len(equal)),rhs,np.zeros(65),soc_rhs]
 cones=[clarabel.ZeroConeT(len(equal)),clarabel.NonnegativeConeT(len(rhs)),clarabel.SecondOrderConeT(65)]+[clarabel.SecondOrderConeT(3) for _ in socs]
 ref=np.array([float(F(x)) for x in job['reference']]);P=np.r_[np.repeat(2.,64),0.];q=np.r_[2*ref,0.]
 if job['objective']['kind']=='progress':
  q[:64]-=5*np.array([float(F(x)) for x in job['progression']['gradient']]);q[64]=5*float(F(job['progression']['epsilon']));P[64]=5*float(F(job['progression']['H']))
 settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=200;settings.tol_gap_abs=settings.tol_gap_rel=settings.tol_feas=1e-10
 return sp.diags(P,format='csc'),q,A,b,cones,settings

def construct(job):
 import clarabel,numpy as np
 P,q,A,b,cones,settings=assemble(job);assert P.shape==(65,65) and A.shape[1]==65 and all(np.isfinite(x).all() for x in (P.data,q,A.data,b))
 zero=evaluate(job,[F(0)]*64);assert zero['passed'],[r for r in zero['rows'] if r['slack']<0]
 def sparse(x):return {'shape':list(x.shape),'indptr':x.indptr.tolist(),'indices':x.indices.tolist(),'data':x.data.tolist()}
 matrix={'P':sparse(P),'q':q.tolist(),'A':sparse(A),'b':b.tolist(),'cones':[str(c) for c in cones]};info={'matrix_id':content_id(matrix),'shape':list(A.shape),'variables':64,'solver_variables':65,'model_id':job['model_id'],'zero_anchor_passed':True,'constructed':True}
 from plan_io import save
 matrix_path=f'results/matrices/{info["matrix_id"]}.json.gz'
 if not (HERE/matrix_path).exists():save(matrix_path,matrix)
 info['matrix_object']=matrix_path
 return clarabel.DefaultSolver(P,q,A,b,cones,settings),info

def native(job,notify):
 solver,info=construct(job);notify(info);began=time.time();sol=solver.solve()
 return {'model_id':job['model_id'],'assembly':info,'status':str(sol.status),'x':list(sol.x),'s':list(sol.s),'z':list(sol.z),'obj_val':sol.obj_val,'obj_val_dual':sol.obj_val_dual,'iterations':sol.iterations,'r_prim':sol.r_prim,'r_dual':sol.r_dual,'solve_time':sol.solve_time,'native_entered_at':began,'native_finished_at':time.time()}

def rollout(job,state,commands,sub=16):
 c=job['actor'];p=state['poses'][c];v=state['v'][c];d=state['delta'][c];nodes=[]
 for a,w in commands:
  a,w=F(a),F(w);nv,nd=v+H*a,d+H*w;assert max(abs(a),abs(w))<=1 and -1<=nv<=2 and abs(nd)<=F(1,2);p=propagate(p,v,nv,d,nd,direction(c),sub)[-1];nodes.append({'pose_Q':pose_rows(p),'v':nv,'delta':nd});v,d=nv,nd
 s0=m.I.raw(*job['progression']['source_s_Q']);s1=coordinate(p,job['progression']['terminal_chart']);ds=s1-s0;energy=H*sum(F(a)**2+F(w)**2 for a,w in commands)
 return {'pose_Q':pose_rows(p),'v':v,'delta':d,'progress_Q':[ds.l,ds.u],'energy':energy,'nodes':nodes}

def values(job,rollout,delta):
 ds=rollout['progress_Q'];energy=F(rollout['energy']);P=job['objective']['kind']=='progress';true_lo=energy-(F(ds[1],m.Q) if P else 0);true_hi=energy-(F(ds[0],m.Q) if P else 0)
 t=norm(delta);dref=job['progression']['reference_progress_Q'];s_lower=F(dref[0],m.Q)+sum(F(g)*x for g,x in zip(job['progression']['gradient'],delta))-F(job['progression']['epsilon'])*t-F(job['progression']['H'])*t*t/2
 return {'true_F_lower':true_lo,'true_F_upper':true_hi,'SOCP_F_upper':energy-(s_lower if P else 0),'actual_progress_Q':ds,'JE':energy,'progress_lower_bound':s_lower}

def check_receiver(job,end):
 r=job['receiver'];p=[m.I.raw(*z) for z in end['pose_Q']];pos=sum((p[j]-F(r['center_pose'][j])).abs_upper() for j in (0,1));yaw=(p[2]-F(r['center_pose'][2])).abs_upper()
 assert pos<=F(r['rP']) and yaw<=F(r['rPsi']) and F(end['v'])==F(r['center_v']) and F(end['delta'])==F(r['center_delta']),('actual_receiver',pos,yaw,r['rP'],r['rPsi'])
 return {'passed':True,'position_l1_error':pos,'yaw_error':yaw,'phase':r['phase'],'K_id':r['K_id'],'v_delta_exact':True}

def admit(job,raw,state):
 assert raw['model_id']==job['model_id'] and raw['status'] in ('Solved','AlmostSolved')
 ref=list(map(F,job['reference']));oldcmd=[[ref[2*j],ref[2*j+1]] for j in range(32)];base=rollout(job,state,oldcmd);base_values=values(job,base,[F(0)]*64);attempts=[]
 for alpha in (F(1),F(1,2),F(1,4)):
  delta,quant=quantize(raw['x'][:64],alpha,job['lattice']);rows=evaluate(job,delta);attempt={'alpha':alpha,'rows_passed':rows['passed'],'violations':[r for r in rows['rows'] if r['slack']<0]}
  if rows['passed']:
   commands=[[ref[2*j]+delta[2*j],ref[2*j+1]+delta[2*j+1]] for j in range(32)]
   try:
    actual=rollout(job,state,commands);receiver=check_receiver(job,actual);objectives=values(job,actual,delta);assert objectives['true_F_upper']<=objectives['SOCP_F_upper']+F(1,10**12),'objective upper not certified'
    budget=F(0) if job['objective']['kind']=='energy' else F(1,10**10);assert objectives['true_F_upper']<base_values['true_F_lower']-budget,'insufficient certified true objective improvement'
   except (AssertionError,ValueError) as e:attempt['failure']=repr(e)
   else:
    return {'accepted':True,'alpha':alpha,'model_id':job['model_id'],'commands':commands,'delta':delta,'quantization':quant,'row_norm':rows['norm'],'receiver':receiver,'objective':objectives,'baseline_objective':base_values,'objective_improvement_lower':base_values['true_F_lower']-objectives['true_F_upper'],'first_a_changed':delta[0]!=0,'first_omega_changed':delta[1]!=0,'a_changed':sum(x!=0 for x in delta[0::2]),'omega_changed':sum(x!=0 for x in delta[1::2]),'attempts':attempts+[attempt]}
  attempts.append(attempt)
 return {'accepted':False,'model_id':job['model_id'],'attempts':attempts,'baseline_objective':base_values}
