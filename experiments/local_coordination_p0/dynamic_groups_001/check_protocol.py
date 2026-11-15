"""Zero-call forks, migrations and damaged requests, with real fallback continuations."""
from common import *
from protocol import *
from strategy import certify_combination
from physics import step

def anchor_candidate(s,c,gid):
    w=get(HERE/'objects',s['witness']);g=get(HERE/'objects',gid);controls=w['controls'][c][s['cycle']:]
    return normal(dict(actor=c,phase=s['cycle'],contract_id=s['contract_id'],source=local_source(s,c),generation_id=gid,derivative_id=g['derivative_id'],receiver=g['receiver'],controls=controls,suffix_id=content_id(controls[33:]),deadline=max(w['goal_times'][c]-s['cycle'],0),selection='candidate_contract_anchor',raw_result_id=None))

def main():
    assert read(HERE/'solver_calls.json')==[];reg=read(HERE/'pre_registration.json');s=restore(m,reg['state']);cs=reg['actors'];checks=reg['scan'][-1]['checks'];gids={x['actor']:x['generation_id'] for x in checks if 'freedom' in x};plans={c:anchor_candidate(s,c,gids[c]) for c in cs};versions={c:0 for c in CARS};contract=get(HERE/'objects',s['contract_id']);partition=groups([]);assert len(partition)==len(CARS)
    tags={c:content_id(dict(tail=get(HERE/'objects',s['witness'])['controls'][c][s['cycle']:],receiver_chain=get(HERE/'objects',s['witness'])['events'])) for c in CARS};b={c:max(contract['goals'][c]-s['cycle'],0) for c in CARS}
    merged=groups([cs]);m1=migrate(partition,merged,contract,tags,b);m2=migrate(merged,partition,contract,tags,b);write(HERE/'results/group_events.json',[dict(fork=True,**m1),dict(fork=True,**m2)])
    reqs=[prepare(s,plans,[c],s['contract_id'],versions) for c in cs];tokens=[commit(s,q,versions) for q in reqs];wid,chosen=select_for_execution(s,tokens,versions);(HERE/'results/rejection').mkdir(exist_ok=True);proof=certify_combination(s,wid,HERE/'results/rejection/anchor_execution.jsonl.gz')
    results=[]
    def reject(name,mutate):
        q=copy.deepcopy(reqs[0]);mutate(q)
        try:commit(s,q,versions)
        except (ValueError,AssertionError,KeyError) as e:reason=str(e)
        else:raise AssertionError('corruption accepted: '+name)
        results.append(dict(name=name,expected='reject before commit',actual='rejected',reason=reason,request_id=put(HERE/'objects',q),selected_plan=s['witness'],contract_id=s['contract_id'],versions=versions,fork=True,continuation_id=put(HERE/'objects',proof)))
    c=cs[0]
    reject('stale_source_or_parent',lambda q:q['plans'][c].update(source='stale'))
    reject('stale_derivative_and_phase',lambda q:q['plans'][c].update(derivative_id='stale',phase=q['phase']-1))
    reject('mixed_neighbor_boundary_version',lambda q:q['dependencies'].update({next(iter(q['dependencies'])):'mixed'}))
    reject('old_contract_anchor_in_new_contract',lambda q:q['plans'][c].update(contract_id='old'))
    reject('damaged_receiver_K',lambda q:q['plans'][c]['receiver'].update(phase=-1))
    reject('damaged_completion_tail',lambda q:q['plans'][c]['controls'][-1].__setitem__(0,'1'))
    reject('damaged_deadline',lambda q:q['plans'][c].update(deadline=0))
    reject('conflicting_resource_grant',lambda q:q.update(resource_calendar_id='new independent conflicting grant'))
    bad=copy.deepcopy(contract);bad['edge_ids'].pop(next(iter(bad['edge_ids'])));badid=put(HERE/'objects',bad)
    reject('split_lost_pair_edge',lambda q:q.update(contract_id=badid))
    bad=copy.deepcopy(contract);bad['cap'][c]=str(F(bad['cap'][c])*2);badid=put(HERE/'objects',bad)
    reject('expanded_envelope_stale_discovery',lambda q:q.update(contract_id=badid))
    independent=versions.copy();independent[next(x for x in CARS if x not in cs)]+=1;assert commit(s,reqs[0],independent)
    results.append(dict(name='unrelated_internal_version',expected='accept',actual='accepted',fork=True,selected_plan=wid))
    for status in ('Timeout','Exception'):
        from local_update import admit
        g=get(HERE/'objects',gids[c]);job=dict(actor=c,generation_id=gids[c],anchor_id=g['anchor_id'],derivative_id=g['derivative_id']);fallback_candidate,admission=admit(s,job,dict(status=status))
        assert not admission['accepted'] and fallback_candidate['selection']=='candidate_contract_anchor' and fallback_candidate['controls']==plans[c]['controls']
        results.append(dict(name='worker_'+status,expected='layer1_candidate_contract_anchor',actual='candidate_contract_anchor',fork=True,selected_plan=wid,continuation_id=put(HERE/'objects',proof)))
    try:commit(s,reqs[0],versions,missing_ack=c)
    except ValueError as e:reason=str(e)
    else:raise AssertionError('missing ACK committed')
    wid2,choice=select_for_execution(s,tokens,versions,failed_blocks=[c]);assert choice[c]=='old_contract_old_plan' and choice[cs[1]]=='candidate_contract_anchor'
    fallback=certify_combination(s,wid2,HERE/'results/rejection/layer2_execution.jsonl.gz');results.append(dict(name='missing_ACK_layer2',expected='whole_failed_block_old_contract_old_plan; independent block continues',actual=choice,reason=reason,fork=True,selected_plan=wid2,continuation_id=put(HERE/'objects',fallback)))
    write(HERE/'results/protocol_checks.json',dict(passed=True,new_calls=0,tests=results,independent_groups=True,merge_split_label_only=True,continuation_steps=contract['stop']-s['cycle'],proof_id=put(HERE/'objects',proof)));print(dict(passed=True,tests=len(results),calls=0),flush=True)
if __name__=='__main__':main()
