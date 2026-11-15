"""Completion deadlines are attached to checked tails, never an arbitrary countdown."""
from common import *
from physics import step
from certify_domains import check_domain

def deadlines(state,witness=None):
    w=get(HERE/'objects',witness or state['witness']);return {c:max(int(w['goal_times'][c])-state['cycle'],0) for c in CARS}

def certify_combination(state,wid,trace_path=None):
    began=__import__('time').perf_counter();w=get(HERE/'objects',wid);old=get(HERE/'objects',state['witness']);s=copy.deepcopy(state);domains=read(HERE/'domain_checks.json')['domains'];minimum_road=minimum_pair=F(100);receivers=[]
    trace=gzip.open(trace_path,'wt') if trace_path else None
    assert w['ticks']==old['ticks'] and max(w['goal_times'].values())<=max(old['goal_times'].values())
    for c in CARS:assert w['controls'][c][:s['cycle']]==old['controls'][c][:s['cycle']]
    for t in range(state['cycle'],w['ticks']):
        for c,kind in w['events'].get(str(t),[]):
            if kind.startswith('generated:'):
                cert=get(HERE/'objects',kind.split(':')[1]);from receivers import feedback
                check_domain(s,c,kind,domains);assert tuple(map(F,w['controls'][c][t]))==feedback(s,c,cert),'actual K differs';receivers.append(dict(actor=c,phase=t,certificate=kind))
        from execution import advance
        execution=advance(s,wid);actual=execution['actual']
        if trace:trace.write(json.dumps(execution,separators=(',',':'))+'\n')
        minimum_road=min(minimum_road,F(actual['bounds']['road']));minimum_pair=min(minimum_pair,F(actual['bounds']['pair']))
        for c,kind in w['events'].get(str(t+1),[]):check_domain(s,c,kind,domains)
        for c in CARS:
            if t+1>=w['goal_times'][c]:check_domain(s,c,'target',domains);assert s['v'][c]==s['delta'][c]==0
    if trace:trace.close()
    return normal(dict(passed=True,source=state_record(state),witness=wid,final=state_record(s),receivers=receivers,minimum_road=minimum_road,minimum_pair=minimum_pair,deadlines=deadlines(state,wid),wall_s=__import__('time').perf_counter()-began))

def validate_candidate(state,p):
    from protocol import local_source
    from rules import quantize
    from socp_rows import evaluate
    g=get(HERE/'objects',p['generation_id']);a=get(HERE/'objects',g['anchor_id']);assert a['contract_id']==p['contract_id'] and a['state']==normal(state_record(state)),'stale numerical source'
    assert p['source']==local_source(state,p['actor']) and g['derivative_id']==p['derivative_id']
    old=get(HERE/'objects',state['witness']);k=state['cycle'];c=p['actor'];assert p['controls'][33:]==old['controls'][c][k+33:],'completion suffix changed'
    if p['selection']=='candidate_contract_anchor':assert p['controls']==old['controls'][c][k:],'wrong contract anchor';return
    raw=get(HERE/'objects',p['raw_result_id']);assert raw['status'] in ('Solved','AlmostSolved')
    assert raw['source']==a['state'] and raw['anchor_id']==g['anchor_id'] and raw['generation_id']==p['generation_id'] and raw['derivative_id']==p['derivative_id'],'wrong numerical source/model; retained vectors are regression fixtures only'
    rows=get(HERE/'objects',g['physical_rows_id']);d=[F(x)-F(y) for u,v in zip(p['controls'][:32],old['controls'][c][k:k+32]) for x,y in zip(u,v)]
    assert evaluate(rows,d)['passed'];assert any((lambda q:q[1]['passed'] and q[0]==d)(quantize(raw['x'],alpha,a['contracts'])) for alpha in (F(1),F(1,2),F(1,4))),'raw/quantized mismatch'
