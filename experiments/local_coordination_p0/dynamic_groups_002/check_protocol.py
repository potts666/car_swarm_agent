"""Zero-call physical migration, complete closure, obligations and both fallbacks."""
from common import *
from protocol import *
from strategy import certify_combination
from local_update import admit

def main():
    assert read(HERE/'solver_calls.json')==[];reg=read(HERE/'pre_registration.json');actual=restore(m,reg['actual_source']);cid=reg['new_contract'];s=reference_state(actual,cid);versions={c:0 for c in CARS};gids={x['actor']:x['generation_id'] for x in reg['zero_checks']};plans={c:anchor_candidate(s,c,gids[c]) for c in CARS};q=prepare(actual,plans,CARS,cid,versions);token=commit(actual,q,versions);wid,choice,timing=select_for_execution(actual,[token],versions,cid);assert all(x=='candidate_contract_anchor' for x in choice.values());results=[]
    def reject(name,mutate):
        bad=copy.deepcopy(q);mutate(bad)
        try:commit(actual,bad,versions)
        except (ValueError,AssertionError,KeyError) as e:reason=str(e)
        else:raise AssertionError('corruption accepted: '+name)
        results.append(dict(name=name,expected='reject before commit',actual='rejected',reason=reason,request_id=put(HERE/'objects',bad),selected_contract=reg['old_contract'],selected_witness=actual['witness'],fork=True))
    c=reg['added'][0]
    reject('new_boundary_old_anchor',lambda b:b['plans'][c].update(contract_id=reg['old_contract']))
    reject('missing_resource_endpoint',lambda b:b.update(participants=[x for x in CARS if x!='B1']))
    broken=copy.deepcopy(get(HERE/'objects',cid));broken['edge_ids'].pop(next(iter(broken['edge_ids'])));badcid=put(HERE/'objects',broken);reject('missing_new_pair_edge',lambda b:b.update(contract_id=badcid))
    reject('old_phase',lambda b:b['plans'][c].update(phase=actual['cycle']-1))
    reject('old_progress_atlas',lambda b:b['plans'][c].update(atlas_id='old'))
    reject('damaged_K',lambda b:b['plans'][c]['receiver'].update(phase=-1))
    reject('damaged_deadline',lambda b:b['plans'][c].update(deadline=0))
    reject('missing_remaining_two_tick_quota',lambda b:b['plans'][c].update(obligations_id='lost'))
    reject('stale_actual_parent',lambda b:b['actual_source'].update(witness='stale'))
    reject('expanded_envelope_old_discovery',lambda b:b['dependencies'].update({next(iter(b['dependencies'])):'stale'}))
    try:commit(actual,q,versions,missing_ack=c)
    except ValueError as e:reason=str(e)
    else:raise AssertionError('missing ACK passed')
    oldwid,choice,_=select_for_execution(actual,[],versions,cid,fail=True);assert oldwid==actual['witness'] and all(x=='old_contract_old_plan' for x in choice.values());results.append(dict(name='missing_ACK_layer2',reason=reason,selected_witness=oldwid,choice=choice,fork=True))
    job=dict(actor=c,generation_id=gids[c],anchor_id=get(HERE/'objects',gids[c])['anchor_id'],derivative_id=get(HERE/'objects',gids[c])['derivative_id']);p,a=admit(s,job,dict(status='Exception',logical_protocol_branch=True));assert not a['accepted'] and p['controls']==plans[c]['controls'];results.append(dict(name='worker_exception_layer1',selected_witness=wid,selected_anchor_contract=cid,fork=True))
    s2=restore(m,reg['second_baseline_source']);cid2=s2['contract_id'];s2['contract_id']=None;r2=reference_state(s2,cid2);g2=reg['second_baseline_checks'][0];c2=g2['actor'];p2=anchor_candidate(r2,c2,g2['generation_id']);q2=prepare(s2,{c2:p2},[c2],cid2,versions);vv=versions.copy();vv[next(x for x in CARS if x!=c2)]+=1;assert commit(s2,q2,vv);results.append(dict(name='unrelated_group_internal_version',actual='accepted',fork=True))
    partition0=groups([]);merged=groups([get(HERE/'objects',cid)['transition']['closure']]);assert len(merged)==1 and len(partition0)==4
    write(HERE/'results/group_mechanism.json',dict(passed=True,merge=dict(old=partition0,new=merged,reason='actual changed physical face/resource endpoint closure'),split=dict(old=merged,new=partition0,reason='fixed faces preserve all six pair edges',edge_ids=get(HERE/'objects',cid2)['edge_ids']),fork=True))
    out=HERE/'results/rejection';out.mkdir(exist_ok=True);oldproof=certify_combination(actual,oldwid,out/'old_contract_execution.jsonl.gz');newproof=certify_combination(actual,wid,out/'new_contract_anchor_execution.jsonl.gz')
    write(HERE/'results/protocol_checks.json',dict(passed=True,new_calls=0,tests=results,old_continuation_id=put(HERE/'objects',oldproof),new_anchor_continuation_id=put(HERE/'objects',newproof),scope='logical timeout/exception; no persistent real-clock vehicle motion'));print(dict(passed=True,tests=len(results),new_calls=0),flush=True)
if __name__=='__main__':main()
