"""Reuse actual new numerical candidates; fail one car/block without another solve."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT))
from common import *
from local_update import admit
from protocol import *
from strategy import certify_combination
calls_before=sha(ROOT/'solver_calls.json');e=read(ROOT/'results/D-P/round_1.json');s=restore(m,e['source']);versions={c:0 for c in CARS};actors=[a['job']['actor'] for a in e['admissions']];failed=actors[0];other=actors[1];plans={a['job']['actor']:a['candidate'] for a in e['admissions']};reqs={c:prepare(s,plans,[c],s['contract_id'],versions) for c in actors};tests=[]
for status in ('Timeout','Exception'):
    job=next(a['job'] for a in e['admissions'] if a['job']['actor']==failed);fallback,a=admit(s,job,dict(status=status,logical_protocol_test=True));assert not a['accepted'] and fallback['selection']=='candidate_contract_anchor'
    candidates=dict(plans);candidates[failed]=fallback;tokens=[commit(s,prepare(s,candidates,[c],s['contract_id'],versions),versions) for c in actors];wid,choice=select_for_execution(s,tokens,versions);assert choice[failed]=='candidate_contract_anchor' and choice[other]=='new_candidate'
    path=ROOT/'results/rejection/anchor_execution.jsonl.gz'
    if status=='Timeout':
        if path.exists():path.rename(path.with_name('stageB_anchor_execution.jsonl.gz'))
        proof=certify_combination(s,wid,path);proofid=put(ROOT/'objects',proof)
    tests.append(dict(name='worker_'+status,fork=True,layer=1,source=e['source'],selected_witness=wid,choice=choice,other_new_candidate_retained=True,proof_id=proofid))
try:commit(s,reqs[failed],versions,missing_ack=failed)
except ValueError as error:reason=str(error)
else:raise AssertionError('missing ACK accepted')
external=commit(s,reqs[other],versions);wid,choice=select_for_execution(s,[external],versions,failed_blocks=[failed]);assert choice[failed]=='old_contract_old_plan' and choice[other]=='new_candidate'
path=ROOT/'results/rejection/layer2_execution.jsonl.gz'
if path.exists():path.rename(path.with_name('stageB_layer2_execution.jsonl.gz'))
proof=certify_combination(s,wid,path);tests.append(dict(name='missing_ACK',fork=True,layer=2,source=e['source'],selected_witness=wid,choice=choice,other_new_candidate_retained=True,reason=reason,proof_id=put(ROOT/'objects',proof)))
# Same damaging interfaces as Stage B, now using the saved lawful numerical proposal.
base=reqs[failed]
def reject(name,mutate):
    q=copy.deepcopy(base);mutate(q)
    try:commit(s,q,versions)
    except (AssertionError,ValueError,KeyError) as error:why=str(error)
    else:raise AssertionError('corruption accepted '+name)
    tests.append(dict(name=name,fork=True,expected='reject',actual='rejected',reason=why,request_id=put(ROOT/'objects',q),selected_witness=s['witness'],continuation='layer2_execution.jsonl.gz; external independent new plan retained'))
reject('stale_source',lambda q:q['plans'][failed].update(source='stale'))
reject('stale_derivative',lambda q:q['plans'][failed].update(derivative_id='stale'))
reject('stale_phase',lambda q:q['plans'][failed].update(phase=-1))
reject('mixed_neighbor_boundary',lambda q:q['dependencies'].update({next(iter(q['dependencies'])):'mixed'}))
reject('old_anchor_contract',lambda q:q['plans'][failed].update(contract_id='old'))
reject('damaged_K',lambda q:q['plans'][failed]['receiver'].update(phase=-1))
reject('damaged_tail',lambda q:q['plans'][failed]['controls'][-1].__setitem__(0,'1'))
reject('damaged_deadline',lambda q:q['plans'][failed].update(deadline=0))
reject('conflicting_resource_grant',lambda q:q.update(resource_calendar_id='conflicting'))
contract=get(ROOT/'objects',s['contract_id']);bad=copy.deepcopy(contract);bad['edge_ids'].pop(next(iter(bad['edge_ids'])));badid=put(ROOT/'objects',bad);reject('lost_split_pair_edge',lambda q:q.update(contract_id=badid))
bad=copy.deepcopy(contract);bad['cap'][failed]=str(F(bad['cap'][failed])*2);badid=put(ROOT/'objects',bad);reject('expanded_envelope_stale_discovery',lambda q:q.update(contract_id=badid))
vv=versions.copy();vv[next(c for c in CARS if c not in actors)]=1;assert commit(s,base,vv);tests.append(dict(name='unrelated_group_version',actual='accepted',fork=True))
assert sha(ROOT/'solver_calls.json')==calls_before;write(ROOT/'results/rejection/postsolve_checks.json',dict(passed=True,new_calls=0,tests=tests,source_arm='D-P',source_round=1,actual_new_candidates_used=True));print(dict(passed=True,tests=len(tests),new_calls=0),flush=True)
