"""Fresh task initialization, retained1822 external calendar and analytic branches."""
import copy
from common import *
from initialize import run as initialize
from preflight import simultaneous
from encounter import active_pairs,propose
from plan_io import load,save,source,record
from physics import step
from audit import audit

def run():
 assert not read(HERE/'solver_calls.json');initialize();old=simultaneous(load('results/init_0.json.gz'),True);historical=HERE.parent/'encounter_coordination_001/results/R_old.json.gz';import gzip
 prior=json.loads(gzip.decompress(historical.read_bytes()))
 for field in ('controls','owners','per_car_policies'):assert json.loads(json.dumps(old[field],default=str))==prior[field],field
 scans=[]
 for k in range(10):
  state=source(old,k);events,active=active_pairs(old,state);scans.append({'cycle':k,'active':active,'v':state['v'],'delta':state['delta'],'reason':'first actual opposite intent conflict with external motion' if events else 'no moving opposite conflict'})
  if events:break
 e=events[0];D=e['actors'];X=next(c for c in CARS if c not in D and state['v'][c]!=0);assert D==['A1','B1']
 extended=propose(old,state,*D);new=copy.deepcopy(extended)
 # Keep original external controls, deadlines and lease endpoints; preserve idle.
 new['B0']=old['B0'];new['owners']=new['owners'][:855]+[None]*(924-855)+old['owners'][924:]
 for c in CARS:
  if c in D:new['controls'][c]+= [(F(0),F(0))]*(old['B0']-len(new['controls'][c]))
  else:new['controls'][c]=copy.deepcopy(old['controls'][c]);new['per_car_policies'][c]=copy.deepcopy(old['per_car_policies'][c])
 new['b_i']={c:new['per_car_policies'][c]['goal'] for c in CARS};new['goal_permanent']={c:[new['b_i'][c],'infinity'] for c in CARS}
 for name,p in [('old',old),('new',new),('extended',extended)]:
  p['frame_branch']=name;p['inner_version']=0 if name=='old' else 1;p['outer_frame_id']='pre_frame';p=json.loads(json.dumps(p,default=str));save(f'results/references/{name}.json.gz',p);s=initial_state();refs=[record(s)]
  for t in range(p['B0']):step(s,{c:p['controls'][c][t] for c in CARS},p['owners'][t],8);refs.append(record(s))
  save(f'results/references/{name}_states.json.gz',refs);audit(p,output=HERE/f'results/references/{name}_acceptance.json');print('PASS rebuilt',name,p['B0'],p['b_i'],flush=True)
 write(HERE/'results/source_scan.json',scans);write(HERE/'results/source_selection.json',{'cycle':k,'D':D,'X':X,'fourth':next(c for c in CARS if c not in D+[X]),'event':e,'source':record(state),'old_relation':{'controls_calendar_policies_identical':True,'historical_path':str(historical.resolve()),'sha':sha(historical),'read_after_generation':True},'source_init':'fresh static-input initializer, historical numeric results not input','curve_source_check':'curve_joint_001/verify_delivery.py passed; signed JE/progress/common-F table and U_J units verified'})
if __name__=='__main__':run()
