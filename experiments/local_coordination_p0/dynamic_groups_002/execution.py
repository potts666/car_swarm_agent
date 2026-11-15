"""One simultaneous physical tick, local input ownership and deadline shift."""
from common import *
from physics import step
from certify_domains import check_domain
from strategy import deadlines

def advance(state,wid):
    oldb=deadlines(state);w=get(HERE/'objects',wid);k=state['cycle'];owner=w['owners'][k];domains=read(HERE/'domain_checks.json')['domains'];oldowner=state['owner'];source=state_record(state)
    if owner!=oldowner:
        if oldowner:check_domain(state,oldowner,'G_out',domains)
        if owner:check_domain(state,owner,'W_in',domains);assert owner not in state['served'];state['served'].append(owner)
        state['permission_version']+=1
    state.update(witness=wid,owner=owner)
    commands={c:tuple(map(F,w['controls'][c][k])) for c in CARS}
    from physics import predicted_nodes
    from progress import step_progress
    progress=step_progress(state,commands,predicted_nodes(state,commands,4),w)
    cid=w.get('activity_contract_id') or state.get('contract_id')
    if cid:
        for c,quota in get(HERE/'objects',cid).get('obligations',{}).get(str(k),{}).items():
            assert F(progress[c]['progress_lower'])>=F(quota) or progress[c]['completed_this_tick'],'remaining activity quota violated: '+c
    actual=step(state,commands,owner,4)
    for c,kind in w['events'].get(str(k+1),[]):check_domain(state,c,kind,domains)
    b=deadlines(state);assert max(b.values())==max(max(oldb.values())-1,0)
    return normal(dict(progress=progress,cycle=k,source=source,witness_id=wid,actual=actual,owner=owner,permission_version=state['permission_version'],served=state['served'],b_before=oldb,b_after=b,B_before=max(oldb.values()),B_after=max(b.values()),input_owner_ids={c:content_id(dict(car=c,cycle=k,input=commands[c],witness=wid)) for c in CARS}))
