"""Check accepted overlap replacement and failed second-round inheritance evidence."""
from fix_common import *
from diagnostics import strict_interior


def main():
    checks=[]
    calls=read(HERE/'results/solver_calls.json');assert len(calls)==4 and all(c['started'] and c['completed'] for c in calls)
    for name in ('curve','straight_acceleration'):
        run=HERE/'results'/name;events=read(run/'events.json');audit=read(run/'independent_acceptance.json')
        assert audit['passed'] and audit['accepted_numeric_candidates']==2 and len(events)==2
        first,second=events;anchor=get(HERE/'objects',second['anchor_id']);old=anchor['plan'];proposal=get(HERE/'objects',second['policy_result_id'])['accepted_proposal']
        assert old['parent']==first['accepted_plan_id'] and second['parent_plan_id']==old['id']
        assert old['pending_reset']['phase']==second['phase']+31 and proposal['pending_reset']['phase']==second['phase']+32
        assert tuple(map(F,old['remaining'][31]))==tuple(map(F,old['pending_reset']['input']))
        assert content_id(proposal['remaining'][33:])==content_id(old['remaining'][33:])
        assert old['budget']==proposal['budget']==len(old['remaining'])+old['suffix_ticks']
        with gzip.open(run/'execution.jsonl.gz','rt') as stream:rows=list(map(json.loads,stream))
        old_K_phase=old['pending_reset']['phase'];new_K_phase=proposal['pending_reset']['phase']
        assert rows[old_K_phase]['plan']['pending_reset']['phase']==new_K_phase
        assert rows[old_K_phase]['actual']['executed_input']==list(map(str,proposal['remaining'][31]))
        assert rows[new_K_phase]['actual']['executed_input']==proposal['pending_reset']['input']
        assert len({e['raw_result_id'] for e in events})==len({e['policy_result_id'] for e in events})==len({e['commit_id'] for e in events})==2
        interiors=[]
        for e in events:
            raw=get(HERE/'objects',e['raw_result_id']);model=get(HERE/'objects',e['assembly_id'])
            assert raw['explicit_zero_increment_anchor']['passed']
            interiors.append(dict(phase=e['phase'],witness=strict_interior(model)))
            match=[c for c in calls if c['raw_result_id']==e['raw_result_id']];assert len(match)==1 and match[0]['binding']['phase']==e['phase']
        forks=read(run/'damage_forks/summary.json')
        assert all(c['passed'] and c['invalid_commits']==c['invalid_executions']==0 and c['actual_steps']==32 and c['old_first']==c['first_actual_input'] for c in forks['cases'])
        assert all(content_id(c['old_pending_reset'])==content_id(old['pending_reset']) and c['actual_old_K_phase']==old_K_phase for c in forks['cases'])
        checks.append(dict(interface=name,two_fresh_calls=True,source_parent_updated=True,old_K_inside_new_prefix=True,old_K_phase=old_K_phase,new_K_phase=new_K_phase,whole_contract_replaced=True,exact_saved_suffix=True,version_budget_preserved=True,rejected_forks_continue_same_old_K=5,independent_passed=True,strict_interiors=interiors))
    for name,digest in read(HERE/'historical_hashes.json').items():assert sha(HERE.parent/name)==digest
    result=dict(passed=True,solver_calls=4,additional_calls=0,checks=checks)
    write(HERE/'self_checks.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
