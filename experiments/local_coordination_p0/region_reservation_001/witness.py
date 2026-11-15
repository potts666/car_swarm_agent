"""Explicit 2802-tick four-car completion policies and an experimental online chooser."""
from common import *
from certify_domains import check_domain


def make(order,parent=None,keep=None,source_cycle=0):
    data=read(HERE/'rational_inputs.json');controls={c:[(F(0),F(0))]*2802 for c in CARS};starts={};events={};owners=[None]*2802
    for c in CARS:
        prep=[tuple(map(F,u)) for u in data['preparation'][c]];controls[c][:len(prep)]=prep
    for rank,c in enumerate(order):
        start=55+rank*663;starts[c]=start;route=get(HERE/'objects',data['routes'][c]);u=[tuple(map(F,x)) for x in route['controls']];controls[c][start:start+758]=u;owners[start:start+663]=[c]*663
        for offset,kind in ((0,'W_in'),(2,'W_hold'),(101,'G_in'),(663,'G_out'),(758,'target')):events.setdefault(str(start+offset),[]).append([c,kind])
    if keep:
        for c in order[:source_cycle]:controls[c]=copy.deepcopy(keep['controls'][c])
    value=dict(version=1,order=order,controls=controls,owners=owners,starts=starts,events=events,goal_times={c:k+758 for c,k in starts.items()},ticks=2802,model_sha256=sha(MODEL_PATH),domain_certificate=read(HERE/'domain_checks.json')['certificate_id'],rational_inputs_id=content_id(data),parent_witness=parent,adoption_rank=source_cycle)
    return normal(value)


def choose(eligible,last,case):
    assert eligible
    if last:
        preferred=[c for c in eligible if (direction(c)!=direction(last))==(case=='opposite')]
        if preferred:return min(preferred)
    return min(eligible)


def propose_choice(state,old,case):
    rank=(state['cycle']-55)//663;prefix=old['order'][:rank];eligible=[c for c in CARS if c not in state['served']]
    domains=read(HERE/'domain_checks.json')['domains']
    for c in eligible:check_domain(state,c,'W_in',domains)
    chosen=choose(eligible,prefix[-1] if prefix else None,case);order=prefix+[chosen]+[c for c in old['order'][rank:] if c!=chosen]
    new=make(order,state['witness'],old,rank)
    for c in CARS:assert new['controls'][c][:state['cycle']]==old['controls'][c][:state['cycle']]
    return new,dict(eligible=eligible,chosen=chosen,previous=prefix[-1] if prefix else None,case=case,basis='experimental direction preference among currently certified unfinished waiting cars; ID breaks ties; not a frozen vehicle-priority rule')


def actions(w,k):
    result={}
    for c in CARS:
        start=w['starts'][c]
        result[c]='initial_preparation' if k<55 else 'core_reservation' if w['owners'][k]==c else 'outside_exit_tail' if start+663<=k<start+758 else 'hold'
    return result


def plan_id(state,wid):return content_id(dict(witness=wid,cycle=state['cycle'],history=state['history'],remaining=2802-state['cycle']))
