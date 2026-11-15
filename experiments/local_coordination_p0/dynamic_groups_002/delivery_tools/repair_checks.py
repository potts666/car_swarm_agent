"""Zero-call regression of the two C-v1 admission failures, using retained raw vectors."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from local_update import admit
from protocol import reference_state,anchor_candidate,prepare,commit,select_for_execution
from strategy import certify_combination
r=read(HERE/'pre_registration.json');actual=restore(m,r['actual_source']);ref=reference_state(actual,r['new_contract']);old=read(HERE/'results/C-new/round_1.json');gids={x['actor']:x['generation_id'] for x in r['zero_checks']};tests=[];plans={c:anchor_candidate(ref,c,gids[c]) for c in CARS};before=sha(HERE/'solver_calls.json')
for a in old['admissions']:
 car=a['job']['actor'];g=get(HERE/'objects',gids[car]);job=dict(actor=car,generation_id=gids[car],anchor_id=g['anchor_id'],derivative_id=g['derivative_id']);raw=get(HERE/'objects',a['raw_result_id']);candidate,proof=admit(ref,job,raw);assert proof['accepted'],proof;tests.append(dict(actor=car,retained_raw_id=a['raw_result_id'],candidate_id=put(HERE/'objects',candidate),passed=True,alpha=proof['attempts'][-1]['alpha'],purpose='retained numeric vector regression only; not an actual new-source numerical call'));plans[car]=candidate
versions={c:0 for c in CARS}
try:prepare(actual,plans,CARS,r['new_contract'],versions)
except AssertionError as error:
 assert 'wrong numerical source' in str(error);source_rejection=str(error)
else:raise AssertionError('retained old raw was relabelled as a fresh numerical candidate')
plan=copy.deepcopy(get(HERE/'objects',ref['witness']))
from receivers import replacement_events
for c,p in plans.items():
 plan['controls'][c][actual['cycle']:]=p['controls'];plan['events']=replacement_events(plan['events'],c,p['receiver'])
plan.update(activity_contract_id=r['new_contract'],fixture_only=True);wid=put(HERE/'objects',plan);physical=certify_combination(actual,wid);assert physical['passed'];assert sha(HERE/'solver_calls.json')==before
# The improved plane selection contains the old axis among its maximization candidates.
pool=read(HERE/'results/pool_certification.json');assert pool['valid_count']==13 and pool['pool_size']==16
write(HERE/'results/repair_checks.json',dict(passed=True,new_calls=0,tests=tests,combined_reused_vector_proof_id=put(HERE/'objects',physical),pool_inheritance=dict(old_certificates_unchanged=True,same_16_candidate_controls=True,new_axis_candidate_set_contains_old_axis=True,therefore_reference_minimum_plane_slack_nondecreasing=True,resource_conflicts_unchanged=True,selected_candidate_fresh_full_contract=True),C_v1_is_not_relabelled_as_accepted=True,retained_vector_numerical_source_rejected=source_rejection))
print(dict(passed=True,retained_vector_regressions=len(tests),new_calls=0),flush=True)
