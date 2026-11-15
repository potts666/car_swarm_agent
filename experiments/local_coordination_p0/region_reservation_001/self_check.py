"""Trust-boundary damage, signed permission and saved-policy regression; no new solve."""
from common import *
from protocol import prepare,commit
from strategy import validate_strategy
from physics import predicted_nodes,geometry


def main():
    checks=[]
    for case in ('opposite','same_direction'):
        events=read(HERE/'results'/case/'events.json')
        for e in events:
            wid=e['new_witness'] if e['kind']=='resource_choice' else e['accepted_witness'];assert validate_strategy(wid)
        fork=read(HERE/'results'/case/'version_rejection/summary.json');source=restore(m,fork['source']);wid=fork['proposed_witness'];request=prepare(source,wid,content_id(dict(witness=source['witness'],cycle=source['cycle'],history=source['history'],remaining=2802-source['cycle'])))
        for key,value in (('permission_version',999),('resource_id','wrong'),('history','stale'),('owner',None),('certificate_id','wrong'),('parent_plan_id','stale')):
            bad=copy.deepcopy(request);bad[key]=value
            try:commit(copy.deepcopy(source),bad)
            except (ValueError,AssertionError) as error:checks.append(dict(case=case,damage=key,rejected=True,reason=str(error)))
            else:raise AssertionError('damaged request committed: '+key)
        bad=copy.deepcopy(request);bad['actions'][source['owner']]='core_reservation'
        try:commit(copy.deepcopy(source),bad)
        except ValueError as error:checks.append(dict(case=case,damage='outside_tail_permission',rejected=True,reason=str(error)))
        else:raise AssertionError('old exit permission mixed')
        wrong=copy.deepcopy(source);wrong['poses'][wrong['owner']]=(m.I(0),m.I(0),m.I(0));premature=prepare(wrong,wid,request['parent_plan_id'])
        try:commit(wrong,premature)
        except ValueError as error:checks.append(dict(case=case,damage='premature_release',rejected=True,reason=str(error)))
        else:raise AssertionError('uncleared owner released')
        unproved=copy.deepcopy(get(HERE/'objects',wid));unproved['controls']['B2'][2400][0]=str(F(unproved['controls']['B2'][2400][0])+F(1,10));bid=put(HERE/'objects',unproved)
        try:validate_strategy(bid)
        except AssertionError as error:checks.append(dict(case=case,damage='uncertified_actual_suffix',rejected=True,object_id=bid,reason=str(error)))
        else:raise AssertionError('changed uncertified suffix accepted')
    state=initial_state();state['poses']['A1']=(m.I(0),m.I(0),m.I(0));zero={c:(F(0),F(0)) for c in CARS}
    try:geometry(predicted_nodes(state,zero,4),None,4)
    except ValueError as error:checks.append(dict(damage='unreserved_resource_body',rejected=True,reason=str(error)))
    else:raise AssertionError('body enters without reservation')
    for beta in (F(0),F(1,100),F(2)):
        for tau in (-1,0,F(1,3)):
            for w in (-2,0,1,F(3,2),1-beta*tau*tau/4):assert ((2-w>=0 and (2-w)**2-w*w-beta*tau*tau>=0)==(1-w-beta*tau*tau/4>=0))
    hist=read(HERE/'historical_hashes.json')
    for name,digest in hist.items():assert sha(HERE.parent/name)==digest,name
    oldgeo=read(HERE/'inputs/geometry_certificate.json')
    assert all(F(row['gap'])-2*EXTRA>=SAFE for row in oldgeo['static_checks'])
    objects=[]
    for f in (HERE/'objects').glob('*.json.gz'):get(HERE/'objects',f.name[:-8]);objects.append(f.name[:-8])
    result=dict(passed=True,damage_checks=checks,historical_files_unchanged=len(hist),object_identities_verified=len(objects),static_contexts_with_extra_quantization_reserve=len(oldgeo['static_checks']),both_recorded_actual_runs_satisfy_strengthened_parent_policy_validation=True)
    write(HERE/'self_checks.json',result);print(json.dumps({k:v for k,v in result.items() if k!='damage_checks'},indent=2))
if __name__=='__main__':main()
