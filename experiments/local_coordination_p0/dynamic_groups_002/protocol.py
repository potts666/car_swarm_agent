"""Atomic physical-contract migration, then independent fixed-interface transactions."""
import itertools,time
from common import *

def groups(edits):
    todo=set(CARS);out=[]
    while todo:
        component={min(todo)};changed=True
        while changed:
            changed=False
            for edge in edits:
                if component.intersection(edge) and not set(edge)<=component:component.update(edge);changed=True
        todo-=component;out.append(sorted(component))
    return out

def edge_cover(c):
    assert c['discovery_id']==content_id({k:c[k] for k in ('cap','edge_ids','owner_calendar_id')}),'stale enlarged envelope discovery'
    assert set(c['edge_ids'])=={a+':'+b for a,b in itertools.combinations(CARS,2)},'missing pair edge'

def reference_state(actual,cid):
    c=get(HERE/'objects',cid);edge_cover(c);s=copy.deepcopy(actual);k=actual['cycle'];old=get(HERE/'objects',actual['witness']);new=get(HERE/'objects',c['reference']);expected=c.get('transition',{}).get('actual_source',c['source']);record=normal(state_record(actual));assert record==expected,'stale actual parent/source/phase'
    assert c['phase']==k and new['controls'].keys()==old['controls'].keys()
    for car in CARS:assert new['controls'][car][:k]==old['controls'][car][:k],'rewrites past actual inputs'
    assert new['owners'][:k]==old['owners'][:k],'rewrites executed permissions'
    assert max(new['goal_times'][x]-k for x in CARS)<=max(old['goal_times'][x]-k for x in CARS),'deadline max increased'
    s.update(witness=c['reference'],contract_id=cid);return s

def local_source(s,c):
    w=get(HERE/'objects',s['witness']);k=s['cycle'];return content_id(dict(car=c,cycle=k,pose=pose_rows(s['poses'][c]),v=s['v'][c],delta=s['delta'][c],tail=w['controls'][c][k:],receivers={t:[x for x in e if x[0]==c] for t,e in w['events'].items() if any(x[0]==c for x in e)},goal=w['goal_times'][c]))

def anchor_candidate(s,c,gid):
    w=get(HERE/'objects',s['witness']);g=get(HERE/'objects',gid);tail=w['controls'][c][s['cycle']:];contract=get(HERE/'objects',s['contract_id'])
    return normal(dict(actor=c,phase=s['cycle'],contract_id=s['contract_id'],source=local_source(s,c),generation_id=gid,derivative_id=g['derivative_id'],receiver=g['receiver'],controls=tail,suffix_id=content_id(tail[33:]),deadline=max(w['goal_times'][c]-s['cycle'],0),selection='candidate_contract_anchor',raw_result_id=None,atlas_id=contract['atlas_id'],obligations_id=content_id(contract['obligations'])))

def prepare(actual,candidates,people,cid,versions):
    began=time.perf_counter();s=reference_state(actual,cid);c=get(HERE/'objects',cid);people=sorted(people);assert people and set(people)<=set(CARS)
    assert set(c.get('transition',{}).get('closure',[]))<=set(people),'missing modified boundary/resource/receiver endpoint'
    w=get(HERE/'objects',s['witness']);plans={car:candidates[car] for car in people}
    for car,p in plans.items():
        from strategy import validate_candidate
        validate_candidate(s,p)
        assert p['actor']==car and p['contract_id']==cid,'old anchor in new physical contract'
        assert p['phase']==s['cycle'] and p['source']==local_source(s,car),'stale local source'
        g=get(HERE/'objects',p['generation_id']);assert p['derivative_id']==g['derivative_id'] and p['receiver']==g['receiver'],'bad derivative/receiver/K'
        assert p['atlas_id']==c['atlas_id'] and p['obligations_id']==content_id(c['obligations']),'old atlas or missing second-tick quota'
        assert p['deadline']==max(w['goal_times'][car]-s['cycle'],0) and p['suffix_id']==content_id(p['controls'][33:]),'bad tail/deadline'
    request=normal(dict(actual_source=state_record(actual),participants=people,phase=s['cycle'],contract_id=cid,dependencies={e:v for e,v in c['edge_ids'].items() if set(e.split(':')).intersection(people)},versions={car:versions[car] for car in people},plans=plans,atlas_id=c['atlas_id'],obligations_id=content_id(c['obligations']),resource_calendar_id=c['owner_calendar_id'],exit_actions_id=c['exit_actions_id'],closure=c.get('transition',{}).get('closure',[])))
    return request

def validate_request(actual,q,versions):
    expected=prepare(actual,q['plans'],q['participants'],q['contract_id'],versions);assert q==expected,'mixed dependency/version/atlas/quota';return True

def commit(actual,q,versions,missing_ack=None):
    began=time.perf_counter();validate_request(actual,q,versions);checked=time.perf_counter();wire=[];people=q['participants']
    def send(sender,target,kind,payload):
        raw=json.dumps(normal(dict(sender=sender,target=target,kind=kind,payload=payload)),sort_keys=True,separators=(',',':')).encode();decoded=json.loads(raw);wire.append(dict(id=content_id(decoded),bytes=len(raw),sender=sender,target=target,kind=kind));return decoded
    for car in people:send(car,'group:'+','.join(people),'proposal',q['plans'][car])
    for car in people:
        if car==missing_ack:raise ValueError('missing_ACK: whole physical transaction keeps old contract and all old plans')
        send(car,'group:'+','.join(people),'ack',content_id(q))
    for car in people:send('group:'+','.join(people),car,'commit',content_id(q))
    return dict(request=q,commit_id=content_id(q),messages=len(wire),bytes=sum(r['bytes'] for r in wire),wire=wire,timing=dict(dependency_check_s=checked-began,commit_s=time.perf_counter()-checked))

def select_for_execution(actual,tokens,versions,cid,fail=False):
    began=time.perf_counter();c=get(HERE/'objects',cid);reference=reference_state(actual,cid);old=get(HERE/'objects',actual['witness']);new=copy.deepcopy(get(HERE/'objects',reference['witness']));chosen={car:'candidate_contract_anchor' if c.get('transition') else 'old_contract_old_plan' for car in CARS};seen=set()
    if fail:return actual['witness'],{car:'old_contract_old_plan' for car in CARS},dict(selection_s=time.perf_counter()-began)
    for token in tokens:
        q=token['request'];validate_request(actual,q,versions);assert q['contract_id']==cid and token['commit_id']==content_id(q)
        assert not seen.intersection(q['participants']),'overlapping blocks';seen.update(q['participants'])
        for car,p in q['plans'].items():
            k=actual['cycle'];new['controls'][car][k:]=p['controls'];from receivers import replacement_events
            new['events']=replacement_events(new['events'],car,p['receiver']);chosen[car]=p['selection']
    assert not c.get('transition') or set(c['transition']['closure'])<=seen,'incomplete physical migration'
    new.update(activity_contract_id=cid,parent_witness=actual['witness'],optimization_origin=None,joint_update=dict(source=state_record(actual),contract_id=cid,tokens=tokens));wid=put(HERE/'objects',new)
    return wid,chosen,dict(selection_s=time.perf_counter()-began)
