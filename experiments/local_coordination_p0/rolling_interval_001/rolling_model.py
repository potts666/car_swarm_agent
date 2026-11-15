"""All-car entry: phase-fixed stopped controls and moving-run moment lattice."""
import copy
from common import *
from plan_io import save
from canonical import reference
from derivatives import matrix
from quantization import kernel
from curve_model import generate as moving_generate,evaluate

def mode(plan,c,k):
 p=plan['per_car_policies'][c];route=plan['paths'][c];prep=p['approach_end']
 if k>=p['goal']:return 'goal_hold'
 if k<prep:return 'approach_initial'
 if k<p['service_start']:return 'wait'
 return route['labels'][min(route['ticks']-1,k-p['service_start']+prep)]

def phase_lattice(plan,state,c,cert):
 k=state['cycle'];knots=[[F(x) for x in r] for r in cert['knots']];runs=[];a=0
 while a<32:
  if knots[a][0]==knots[a][1]==0:a+=1;continue
  b=a+1
  while b<32 and not(knots[b][0]==knots[b][1]==0):b+=1
  runs.append((a,b));a=b
 E=[];names=[];T=[];blocks=[]
 def add(row,name):E.append(row);names.append(name)
 moving=set(j for a,b in runs for j in range(a,b))
 for j in range(32):
  if j not in moving:
   for q in (0,1):
    row=[F(0)]*64;row[2*j+q]=1;add(row,f'stopped_phase_fixed:{j}:{q}:{mode(plan,c,k+j)}')
 for a,b in runs:
  n=b-a;J=matrix(cert,b-1);weights=[F(J[2][2*j+1].l+J[2][2*j+1].u,2*m.Q) for j in range(a,b)]
  # Existing acceleration/heading moments are retained within each moving run.
  for comp,vals,label in [(0,[F(1)]*n,'v'),(0,list(map(F,range(n))),'a_moment'),(1,[F(1)]*n,'delta'),(1,weights,'yaw_moment')]:
   row=[F(0)]*64
   for j,val in enumerate(vals,a):row[2*j+comp]=val
   if any(row):add(row,f'run:{a}:{b}:{label}')
  for comp in (0,1):
   start=len(T);indices=[]
   for j in range(a,b-2):
    if comp==0:values=[F(1),F(-2),F(1)]
    else:
     x,y=weights[j-a]-weights[j-a+1],weights[j-a+1]-weights[j-a+2]
     assert x>0 and y>0,('nonmonotone moving weights',c,k,a,b,j,x,y)
     values=[2*y/(x+y),F(-2),2*x/(x+y)]
    col=[F(0)]*64
    for t,value in enumerate(values):col[2*(j+t)+comp]=value
    T.append(col);indices.append(2*j+comp)
   blocks.append({'component':comp,'start':start,'end':len(T),'raw_indices':indices})
 assert all(sum(x*y for x,y in zip(row,col))==0 for row in E for col in T)
 error=[sum(abs(col[j]) for col in T)/F(2*10**12) for j in range(64)]
 return {'E':E,'equality_names':names,'T':T,'blocks':blocks,'rank_free_dimension':len(T),'equality_rank':64-len(T),'component_grid_error':error,'max_grid_error':max(error),'Delta':F(1,10**12),'moving_runs':[list(r) for r in runs]}

def generate(plan,state,c):
 job,cert=moving_generate(plan,state,c,'O-E');lat=phase_lattice(plan,state,c,cert)
 job['lattice']=lat;job['free_ticks']=sum(b-a for a,b in lat['moving_runs']);job['open_moving_omega']=[j for a,b in lat['moving_runs'] for j in range(a,b)];job.pop('model_id');job['model_id']=content_id(job)
 job=json.loads(json.dumps(job,default=str));zero=evaluate(job,[F(0)]*64);assert zero['passed'],('zero_anchor',c,state['cycle'],[r for r in zero['rows'] if r['slack']<0])
 return job,cert

def classify(plan,state,c,job):
 k=state['cycle'];ref=list(map(F,job['reference']));lat=job['lattice'];energy=H*sum(x*x for x in ref);active=[r['name'] for r in job['rows'] if F(r['g'])==0];proof={'model_id':job['model_id'],'source_history':state['history'],'phase':mode(plan,c,k),'zero_anchor':True,'nullity':lat['rank_free_dimension'],'equality_rank':lat['equality_rank'],'E_T_zero':True,'active_physical_rows':active,'anchor_JE':energy,'lattice_id':content_id(lat),'mode_guard_source':'actual canonical phase; stopped a/omega fixed; exact v/delta at each moving-run seam'}
 if k>=plan['per_car_policies'][c]['goal']:
  assert all(x==0 for x in ref) and state['v'][c]==state['delta'][c]==0;kind='goal_hold';proof['permanent_goal']=plan['goal_permanent'][c]
 elif energy==0:kind='certified_optimum';proof['global_lower_bound']=0;proof['argument']='nonnegative original quadratic energy; feasible zero-input window attains zero'
 elif not lat['T']:kind='unique_control';proof['argument']='rank64 phase/equality matrix, empty nullspace, unique delta=0'
 elif all(sum(F(t)*u for t,u in zip(col,ref))==0 for col in lat['T']):
  kind='certified_optimum';proof['argument']='exact reduced gradient T^T reference=0; JE(reference+Tz)=JE(reference)+h||Tz||^2; feasible anchor minimizes full affine space';proof['exact_reduced_gradient_zero']=True
 else:kind='numerical_update'
 return kind,proof
