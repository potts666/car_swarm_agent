"""Actual curved input generator: common C4-C22 rows, objective and E/T."""
import copy,time
from common import *
from derivatives import get,matrix
from curve_bounds import sqrt_upper
from quantization import kernel
from progression import chart,coordinate,gradient,bounds
from receivers import synthesize,reference

def norm(x):return sqrt_upper(sum(F(v)**2 for v in x))
def generate(plan,state,car,arm,sub=64):
 started=time.monotonic();assert arm in ('F-E','O-E','F-P','O-P');k=state['cycle'];cert=get(plan,state,car,sub);J=matrix(cert);free=next((j+1 for j,row in enumerate(cert['knots']) if F(row[1])==0),32);assert free>=3
 weights=[F(J[2][2*j+1].l+J[2][2*j+1].u,2*m.Q) for j in range(free)];lat=kernel(free,weights,arm[0]=='F');receiver=synthesize(plan,state,car,cert);u=[F(x) for row in plan['controls'][car][k:k+32] for x in row];rows=[]
 def add(name,g,linear=None,error=0,hessian=0,kind='physical'):
  rows.append({'name':name,'g':F(g),'a':list(map(F,linear)) if linear is not None else [F(0)]*64,'epsilon':F(error),'H':F(hessian),'kind':kind})
 v=state['v'][car];d=state['delta'][car]
 for j in range(32):
  for comp in (0,1):
   rr=[F(0)]*64;rr[2*j+comp]=1;add(f'input+:{j}:{comp}',u[2*j+comp]-1,rr);add(f'input-:{j}:{comp}',-u[2*j+comp]-1,[-x for x in rr])
  v+=H*u[2*j];d+=H*u[2*j+1]
  for comp,val,low,high in ((0,v,F(-1),F(2)),(1,d,F(-1,2),F(1,2))):
   rr=[F(0)]*64
   for t in range(j+1):rr[2*t+comp]=H
   add(f'state+:{j}:{comp}',val-high,rr);add(f'state-:{j}:{comp}',low-val,[-x for x in rr])
  # Uniform Hausdorff variation is C16 with zero chosen linear projection and
  # certified full-gradient error bound; it covers every actual continuous corner.
  ref=reference(plan)[k+j+1]['poses_Q'][car];pp=[m.I.raw(*q) for q in cert['poses_Q'][j+1]];base=[m.I.raw(*q) for q in ref];offset=sum((pp[t]-base[t]).abs_upper() for t in (0,1))+RB*(pp[2]-base[2]).abs_upper();bnd=cert['bounds'][j]
  continuous_reserve=F(1,40)-offset-F(15)*(H/16)**2/4
  assert continuous_reserve>0
  add(f'continuous_body:{j+1}',-continuous_reserve,error=bnd['Gc'],hessian=bnd['Bc'],kind='continuous_geometry')
 endpose=[m.I.raw(*q) for q in cert['poses_Q'][-1]];bp=cert['bounds'][-1]
 # L1 inner ball is a conservative common position receiver, positive in all arms.
 for sx in (-1,1):
  for sy in (-1,1):
   ints=[sx*J[0][t]+sy*J[1][t] for t in range(64)];mid=[F(z.l+z.u,2*m.Q) for z in ints];eps=norm([F(z.u-z.l,2*m.Q) for z in ints]);position_error=sx*(endpose[0]-receiver['center_pose'][0])+sy*(endpose[1]-receiver['center_pose'][1]);add(f'receiving_position:{sx}:{sy}',F(position_error.u,m.Q)-F(receiver['rP']),mid,eps,F(3,2)*F(bp['Bp']),'receiver')
 for sign in (-1,1):
  ints=[sign*z for z in J[2]];mid=[F(z.l+z.u,2*m.Q) for z in ints];eps=norm([F(z.u-z.l,2*m.Q) for z in ints]);yaw_error=sign*(endpose[2]-receiver['center_pose'][2]);add(f'receiving_yaw:{sign}',F(yaw_error.u,m.Q)-F(receiver['rPsi']),mid,eps,bp['Bpsi'],'receiver')
 ch=chart(plan,car,k+32);sch=chart(plan,car,k);assert ch['kind'] in ('arc','line') and sch['kind'] in ('arc','line')
 grad=gradient(endpose,ch);gi=[sum((grad[r]*J[r][t] for r in (0,1)),m.I(0)) for t in range(64)];g=[F(z.l+z.u,2*m.Q) for z in gi];eps=norm([F(z.u-z.l,2*m.Q) for z in gi]);bounds_s=bounds(ch);Hs=F(bounds_s['G'])*F(bp['Bp'])+F(bounds_s['H'])*F(bp['Gp'])**2
 start_s=coordinate(state['poses'][car],sch);end_s=coordinate(endpose,ch);progress=end_s-start_s
 # Certified endpoint chart lies away from the curvature-switch normal seam.
 if ch['kind']=='arc':assert F(end_s.l,m.Q)-F(ch['offset'])>F(1,100),'terminal patch too near seam'
 job={'actor':car,'arm':arm,'cycle':k,'parent':content_id(plan['controls'][car]),'source':{'poses_Q':pose_rows(state['poses'][car]),'v':state['v'][car],'delta':state['delta'][car],'history':state['history']},'reference':u,'free_ticks':free,'open_moving_omega':list(range(free)) if arm[0]=='O' else [],'lattice':lat,'rows':rows,'receiver':receiver,'progression':{'source_chart':sch,'terminal_chart':ch,'projection_bounds':bounds_s,'gradient':g,'epsilon':eps,'H':Hs,'reference_progress_Q':[progress.l,progress.u],'source_s_Q':[start_s.l,start_s.u]},'rho':F(1,1000),'objective':{'kind':'progress' if arm[2]=='P' else 'energy','lambda':F(1),'h':H,'R':[1,1],'S':[1,1],'common_multiplier':F(5),'units':'JE=h sum normalized input²; progression metres; lambda JEunit/metre'},'derivative_id':content_id({key:val for key,val in cert.items() if key!='wall'})}
 job.update(outer_frame_id=plan['outer_frame_id'],per_car_plan_id=content_id(plan['controls'][car]),dependencies={'outer_frame_id':plan['outer_frame_id'],'valid_from':state['cycle'],'valid_until':'infinity_with_permanent_goals'})
 # Semantic identity contains no volatile wall timing.
 job['model_id']=content_id({key:val for key,val in job.items() if key!='generation_wall'})
 return json.loads(json.dumps(job,default=str)),cert

def evaluate(job,delta):
 delta=list(map(F,delta));t=norm(delta);checks=[]
 for r in job['rows']:
  value=F(r['g'])+sum(F(a)*d for a,d in zip(r['a'],delta))+F(r['epsilon'])*t+F(r['H'])*t*t/2;checks.append({'name':r['name'],'slack':-value})
 return {'passed':t<=F(job['rho']) and all(x['slack']>=0 for x in checks),'norm':t,'rows':checks}
