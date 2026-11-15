"""Local transactions: immutable external edges, per-car ownership, two fallbacks."""
import itertools
from common import *

def groups(edit_edges):
    todo=set(CARS);out=[]
    while todo:
        component={min(todo)};changed=True
        while changed:
            changed=False
            for edge in edit_edges:
                if component.intersection(edge) and not set(edge)<=component:component.update(edge);changed=True
        todo-=component;out.append(sorted(component))
    return out

def edge_cover(contract):
    assert contract['discovery_id']==content_id({k:contract[k] for k in ('cap','edge_ids','owner_calendar_id')}),'stale envelope discovery'
    assert set(contract['edge_ids'])=={a+':'+b for a,b in itertools.combinations(CARS,2)},'missing pair edge'

def local_source(state,c):
    w=get(HERE/'objects',state['witness']);k=state['cycle']
    return content_id(dict(car=c,cycle=k,pose=pose_rows(state['poses'][c]),v=state['v'][c],delta=state['delta'][c],tail=w['controls'][c][k:],receivers={t:[x for x in e if x[0]==c] for t,e in w['events'].items() if any(x[0]==c for x in e)},goal=w['goal_times'][c]))

def prepare(state,candidates,participants,contract_id,versions,edits=()):
    contract=get(HERE/'objects',contract_id);edge_cover(contract);participants=sorted(participants)
    assert participants and set(participants)<=set(CARS)
    assert all(set(edge)<=set(participants) for edge in edits),'unclosed interface edit'
    old=get(HERE/'objects',state['witness']);k=state['cycle'];deps={e:v for e,v in contract['edge_ids'].items() if set(e.split(':')).intersection(participants)}
    plans={c:candidates[c] for c in participants}
    for c,p in plans.items():
        from strategy import validate_candidate
        validate_candidate(state,p)
        assert p['actor']==c and p['contract_id']==contract_id,'anchor outside candidate contract'
        assert p['source']==local_source(state,c) and p['phase']==k,'stale actual source or phase'
        assert p['deadline']==max(old['goal_times'][c]-k,0),'bad completion deadline'
        assert p['derivative_id']==get(HERE/'objects',p['generation_id'])['derivative_id'],'stale derivative'
        row=get(HERE/'objects',p['generation_id'])['receiver'];assert p['receiver']==row,'damaged receiver/K'
        assert len(p['controls'])==len(old['controls'][c])-k and p['suffix_id']==content_id(p['controls'][33:]),'damaged completion tail'
    return normal(dict(participants=participants,phase=k,contract_id=contract_id,dependencies=deps,sources={c:local_source(state,c) for c in participants},versions={c:versions[c] for c in participants},plans=plans,edit_edges=list(edits),resource_calendar_id=contract['owner_calendar_id'],exit_actions_id=contract['exit_actions_id']))

def validate_request(state,request,versions):
    expected=prepare(state,request['plans'],request['participants'],request['contract_id'],versions,request['edit_edges'])
    assert request==expected,'mixed dependency/version/source identity'
    w=get(HERE/'objects',state['witness']);contract=get(HERE/'objects',request['contract_id'])
    assert contract['phase']==state['cycle'] and contract['reference']==state['witness'],'stale contract reference'
    assert request['resource_calendar_id']==content_id(w['owners']),'conflicting resource grant'
    return True

def commit(state,request,versions,missing_ack=None):
    validate_request(state,request,versions);wire=[];people=request['participants']
    def send(sender,target,kind,payload):
        data=normal(dict(sender=sender,target=target,kind=kind,payload=payload));wire.append(dict(id=content_id(data),bytes=len(json.dumps(data,sort_keys=True,separators=(',',':')).encode()),sender=sender,target=target,kind=kind))
    for c in people:send(c,'group:'+','.join(people),'proposal',request['plans'][c])
    for c in people:
        if c==missing_ack:raise ValueError('missing_ACK: whole transaction keeps old plans')
        send(c,'group:'+','.join(people),'ack',content_id(request))
    for c in people:send('group:'+','.join(people),c,'commit',content_id(request))
    return dict(request=request,commit_id=content_id(request),wire=wire,messages=len(wire),bytes=sum(x['bytes'] for x in wire))

def select_for_execution(state,tokens,versions,failed_blocks=()):
    old=get(HERE/'objects',state['witness']);new=copy.deepcopy(old);chosen={c:'old_contract_old_plan' for c in CARS};seen=set();resource_edits=set()
    for token in tokens:
        req=token['request'];validate_request(state,req,versions)
        assert token['commit_id']==content_id(req),'mixed commit'
        assert not seen.intersection(req['participants']),'overlapping transactions';seen.update(req['participants'])
        for edge in req['edit_edges']:
            name=tuple(sorted(edge));assert name not in resource_edits,'conflicting interface';resource_edits.add(name)
        if set(req['participants']).intersection(failed_blocks):continue
        for c,p in req['plans'].items():
            k=state['cycle'];new['controls'][c][k:]=p['controls'];from receivers import replacement_events
            new['events']=replacement_events(new['events'],c,p['receiver']);chosen[c]=p['selection']
    new.update(parent_witness=state['witness'],joint_update=dict(phase=state['cycle'],source=state_record(state),tokens=tokens),optimization_origin=None)
    wid=put(HERE/'objects',new)
    return wid,chosen

def migrate(old_groups,new_groups,contract,plans,deadlines):
    edge_cover(contract)
    for partition in (old_groups,new_groups):assert sorted(c for g in partition for c in g)==sorted(CARS),'invalid partition'
    return normal(dict(kind='label_migration',old=old_groups,new=new_groups,edges=contract['edge_ids'],plans=plans,deadlines=deadlines,physical_contract_changed=False,passed=True))
