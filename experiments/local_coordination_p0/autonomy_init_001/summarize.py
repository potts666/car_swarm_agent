"""Evidence-only metrics, physical contract differences and separate gates."""
import gzip,itertools,time
from common import *
from physics import propagate

def load(n):return json.loads(gzip.decompress((HERE/n).read_bytes()))
def save(n,v):(HERE/n).write_bytes(gzip.compress(json.dumps(v,sort_keys=True,default=str).encode(),mtime=0))
def metrics(w):
 v={c:F(0) for c in CARS};hist={};first=[];run=0;prefix=True;cost=F(0)
 for k in range(w['B0']):
  aa=[]
  for c in CARS:
   a,d=map(F,w['controls'][c][k]);travel=abs(H*(2*v[c]+H*a)/2);v[c]+=H*a;cost+=a*a+d*d
   if travel>=F(1,50):aa.append(c)
  hist[len(aa)]=hist.get(len(aa),0)+1
  if k<2:first.append(aa)
  if prefix and len(aa)==4:run+=1
  else:prefix=False
 return {'B0':w['B0'],'seconds':H*w['B0'],'goal_ticks':{c:w['per_car_policies'][c]['goal'] for c in CARS},'activity_histogram':hist,'first_two':first,'initial_four_active_ticks':run,'input_cost':cost}

def contract(w):
 p={c:tuple(m.I(F(q)) for q in TASKS[c]['initial'][:3]) for c in CARS};v={c:F(0) for c in CARS};d=v.copy();phases=[]
 for k in range(w['B0']+1):
  planes={}
  for c in CARS:
   x,y,t=(F(q.l+q.u,2*m.Q) for q in p[c]);planes[c]={'position_planes':[[1,0,x+F(1,100)],[-1,0,-x+F(1,100)],[0,1,y+F(1,100)],[0,-1,-y+F(1,100)]],'yaw_reference':t,'yaw_radius':read(HERE/'results/phase_receivers.json')[c]['rPsi'],'v':v[c],'delta':d[c]}
  phases.append({'phase':k,'own_planes':planes,'owner':w['owners'][k] if k<w['B0'] else None})
  if k==w['B0']:break
  for c in CARS:
   a,om=map(F,w['controls'][c][k]);nv,nd=v[c]+H*a,d[c]+H*om;p[c]=propagate(p[c],v[c],nv,d[c],nd,direction(c),1)[-1];v[c],d[c]=nv,nd
 return {'kind':'phase position planes plus yaw and exact velocity/steering, swept body-error tube and resource permissions','phases':phases,'own_box_halfwidth':'1/100','body_error_budget':'1/20','physical_proof':'L1 translation <=.02 plus RB*yaw radius <.05; complete interval rectangle audit subtracts .05 per car; holds remain permanent','permanent_targets':w['goal_permanent']}

def run():
 old=load('results/init_0.json.gz');anchor=load('results/parallel_anchor.json.gz');updated=load('results/updated.json.gz')
 mm={name:metrics(w) for name,w in [('initial_serial',old),('parallel_anchor',anchor),('actual_with_worker_errors',updated)]};write(HERE/'results/comparison.json',mm)
 oc,nc=contract(old),contract(anchor);save('results/old_contract.json.gz',oc);save('results/new_contract.json.gz',nc)
 changes=[];excluded=None
 for a,b in zip(oc['phases'],nc['phases']):
  for c in CARS:
   if a['own_planes'][c]['position_planes']!=b['own_planes'][c]['position_planes']:changes.append([a['phase'],c])
   if excluded is None:
    bx=b['own_planes'][c]['position_planes'];x=F(bx[0][2])-F(1,100);y=F(bx[2][2])-F(1,100)
    for nx,ny,beta in a['own_planes'][c]['position_planes']:
     slack=F(beta)-nx*x-ny*y
     if slack<0:excluded={'phase':a['phase'],'car':c,'old_plane':[nx,ny,beta],'new_reference_xy':[x,y],'slack':slack};break
 resources=[{'phase':k,'old':old['owners'][k] if k<len(old['owners']) else None,'new':anchor['owners'][k] if k<len(anchor['owners']) else None} for k in range(max(old['B0'],anchor['B0'])) if (old['owners'][k] if k<len(old['owners']) else None)!=(anchor['owners'][k] if k<len(anchor['owners']) else None)]
 assert excluded and resources
 write(HERE/'results/physical_contract_diff.json',{'passed':True,'old_id':content_id(oc),'new_id':content_id(nc),'changed_phase_car_planes':len(changes),'first_old_face_exclusion':excluded,'resource_changes':resources,'closure':list(CARS)})
 with gzip.open(HERE/'results/progress_deadlines.jsonl.gz','wt') as f:
  for line in gzip.open(HERE/'results/actual.jsonl.gz','rt'):
   row=json.loads(line);k=row['cycle'];before={c:max(0,updated['per_car_policies'][c]['goal']-k+1) for c in CARS};after={c:max(0,updated['per_car_policies'][c]['goal']-k) for c in CARS};B0=max(before.values());B1=max(after.values());assert B1==B0-1
   f.write(json.dumps({'cycle':k,'actual_trace_row_id':content_id(row),'b_before':before,'b_after':after,'B_before':B0,'B_after':B1})+'\n')
 # Independently verify historical byte preservation against the initial inventory.
 preserved={}
 for name,info in read(HERE/'source_inventory.json').items():
  root=Path(info['path']);bad=[n for n,h in info['files'].items() if not (root/n).is_file() or sha(root/n)!=h];assert not bad
  preserved[name]={'passed':True,'files':len(info['files']),'path':str(root)}
 write(HERE/'results/preservation_check.json',preserved)
 numerical=read(HERE/'results/numerical_summary.json');calls=read(HERE/'solver_calls.json');assert len(calls)==8 and all(c['status']=='worker_error' for c in calls)
 gate={'passed':False,'status':'partial','criteria':{'input_provenance':{'status':'passed','evidence':'source manifests + instrumented read-isolated bit-identical initialization rerun'},'map_domain':{'status':'passed','construct':'Omega_DAG','dependencies':[['B2','A2']],'BUF':'not required for main DAG instance; insufficient-buffer cycle fixture rejected'},'autonomous_initialization':{'status':'passed','B0':1955,'request_orders':[old['order'],load('results/init_1.json.gz')['order']],'full_independent_replays':2},'local_discovery':{'status':'passed','scope':'synchronous in-process directory; per-car keys/owners, all tails/proposals/receivers/permanent holds, versions and sorted-lock commit checks, offline all six pairs'},'socp_anchor':{'status':'partial','zero_certification':'two actual-source rounds, exact affine E/T grid and Gi/K identities passed','native_adapter':'v1 failed rational text conversion; v2 all eight model constructions passed zero-solve regression; numerical solve untested'},'multi_car_update':{'status':'partial','first_two_activity':[4,4],'budgeted_worker_attempts':8,'accepted_numerical_candidates':[0,0],'numerical_first_changed':[0,0],'reason':'eight worker assembly errors before native solve; actual new-C anchors retained'},'safety_continuation':{'status':'passed','evidence':'1847-tick actual trace + two complete rejection traces independently checked at sub=8'},'completion':{'status':'passed','steps':1847,'B_final':0,'all_four_targets':True}},'usage':{'budgeted_attempts':8,'limit':8,'worker_errors':8,'native_solve_calls_reached':0,'native_statuses':{},'extra_attempts_after_repair':0},'DG002_same_version_CL':'separate untouched task/budget','next_required':'fresh authorized numerical budget and version: use repaired adapter, repeat two four-car rounds; do not rewrite v1 failures'}
 write(HERE/'GATE.json',gate)
 print(json.dumps({'status':'partial','B_initial':1955,'B_actual':1847,'activity':[4,4],'attempts':8,'native_solve_calls':0,'all_four_goals':True}))
if __name__=='__main__':run()
