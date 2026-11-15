"""Zero-call diagnostic: use the existing W_hold suffix instead of two identity feedbacks."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from proposals import candidate
from progress import ETA,step_progress
from physics import predicted_nodes
from contracts import build,physical_diff
from protocol import reference_state,anchor_candidate,prepare,commit,select_for_execution
from generation import generate
from execution import advance
from certify_domains import check_domain
from strategy import certify_combination
before=sha(HERE/'solver_calls.json');source=None
with gzip.open(HERE/'results/R-old/execution.jsonl.gz','rt') as f:
 for row in map(json.loads,f):
  if row['cycle']==39:source=row['source'];break
actual=restore(m,source);actor='A1';check_domain(actual,actor,'W_hold',read(HERE/'domain_checks.json')['domains']);route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][actor]);assert route['controls'][:2]==[['0','0'],['0','0']] and actual['v'][actor]==actual['delta'][actor]==0
old=get(HERE/'objects',actual['witness']);plan=candidate(old,actual,{actor:18});plan['connection_variant']=dict(fork_only=True,not_in_original_preregistered_pool=True,source_domain='W_hold',wait_advance_increments=[8,8],identity_feedback_ticks_eliminated=2,total_motion_advance=18,mathematical_reason='the two zero-speed/zero-steering feedbacks are identities on W_hold; use its already certified motion suffix')
wid=put(HERE/'objects',plan);oldcid=build(actual);ref=copy.deepcopy(actual);ref['witness']=wid;obligations={'39':{c:str(ETA) for c in CARS},'40':{c:str(ETA) for c in ('A1','A2','B2')}};cid=build(ref,obligations);c=copy.deepcopy(get(HERE/'objects',cid));difference=physical_diff(oldcid,cid);c['transition']=dict(actual_source=state_record(actual),old_contract=oldcid,closure=difference['closure']);cid=put(HERE/'objects',c);ref=reference_state(actual,cid);jobs=[];plans={}
for car in CARS:
 g,gid=generate(ref,car);assert g['passed'],g;jobs.append(dict(actor=car,generation_id=gid));plans[car]=anchor_candidate(ref,car,gid)
versions={c:0 for c in CARS};token=commit(actual,prepare(actual,plans,CARS,cid,versions),versions);chosen,choice,_=select_for_execution(actual,[token],versions,cid);fork=copy.deepcopy(actual);first=advance(fork,chosen);second=advance(fork,chosen);assert [c for c in CARS if first['progress'][c]['active']]==list(CARS);assert sum(v['active'] for v in second['progress'].values())==3
out=HERE/'results/four_car_fork';out.mkdir(exist_ok=True);proof=certify_combination(actual,chosen,out/'execution.jsonl.gz');value=dict(passed=True,new_calls=0,scope='post-experiment diagnostic fork; no numerical optimization, not either numerical arm and not original frozen pool',source=source,actor_released=actor,old_first_activity=['A2','B1','B2'],first_activity=list(CARS),second_activity=['A1','A2','B2'],first=first,second=second,zero_jobs=jobs,contract_id=cid,old_contract_id=oldcid,witness=chosen,physical_difference=physical_diff(oldcid,cid),full_controller_completion_proof_id=put(HERE/'objects',proof),main_four_car_numerical_goal_not_claimed=True)
assert before==sha(HERE/'solver_calls.json');write(out/'summary.json',value);print(dict(passed=True,first_A=4,second_A=3,numerical_calls=0,main_arm=False),flush=True)
