#!/usr/bin/env python3
"""Exact equivalence/rounding checks and actual old-first-step fallback evidence."""
import argparse
from c_common import *
from c_socp import nearest,reserved_rows
from socp import evaluate
from derivatives import verify


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('preserve checks')
    a.output.mkdir(parents=True);data,m=setup();cases=[]
    for Hrow in (F(0),F(1,10),F(2),F(500)):
        L=F(3,100);scale=F(1,10000);K=2*Hrow*scale*scale/L
        for R in (F(-2),F(-1,2),F(0),F(1,100),F(1),F(10)):
            for tau in (F(-1),F(0),F(1,10),F(1),F(5)):
                gap=(R+1)**2-(R-1)**2-K*tau*tau
                soc=(R+1>=0 and gap>=0)
                original=R>=Hrow*scale*scale*tau*tau/(2*L)
                assert soc==original
    cases.append(dict(test='per_row_SOC_projection_exact_algebra',passed=True))
    q=F(1,10**12);worst=[q/2]*64;error=norm_upper([a-b for a,b in zip(nearest(worst),worst)]);assert error==4*q
    for n in range(-100,101):
        raw=[F(n,3)*q+q/2]*64;rounded=nearest(raw);assert all(abs(a-b)<=q/2 for a,b in zip(raw,rounded)) and norm_upper([a-b for a,b in zip(raw,rounded)])<=4*q
    cases.append(dict(test='exact_nearest_grid_and_4q_bound',passed=True,worst_norm=error))
    zero=dict(name='v_lower_at_zero',g_upper=F(0),linear=[-H]+[F(0)]*63,gradient_error=F(0),hessian=F(0),kind='physical')
    out,reserve=reserved_rows([zero],'Q');assert out[0]['g_upper']==0 and not reserve[0]['applied'] and evaluate(out,[F(0)]*64)['passed']
    cases.append(dict(test='zero_slack_physical_row_retains_exact_original_and_bounded_recheck',passed=True))
    diagnostics=read(HERE/'results/diagnostics_001/summary.json')['cases'];coverage={};directions={}
    for name,entry in diagnostics.items():
        result=get(HERE/'objects',entry['result_id']);anchor=get(HERE/'objects',result['anchor_id']);cert=get(HERE/'objects',result['derivative_id']);verify(data,m,cert,anchor)
        old=copy.deepcopy(anchor['plan']);state=restore(m,anchor['state']);candidate,proof=A.construct(m,data,state,old,anchor['phase'])
        dA=[F(x)-F(y) for n,b in zip(candidate['remaining'][:32],old['remaining'][:32]) for x,y in zip(n,b)]
        directions[name]=dict(zero=evaluate(result['rows'],[F(0)]*64),A_direction=evaluate(result['rows'],dA),A_constructed_proof=proof)
        coverage[name]=dict(passed=True,actual_anchor_id=result['anchor_id'],derivative_id=result['derivative_id'])
    attempts=read(HERE/'results/repairs_001/attempts.json');fallback=[]
    for name in ('curve','straight_acceleration'):
        item=next(x for x in reversed(attempts) if x['interface']==name);result=get(HERE/'objects',item['result_id']);assert not result['accepted']
        anchor=get(HERE/'objects',result['anchor_id']);state=restore(m,anchor['state']);old=anchor['plan'];before=old['budget'];first=A.window(old)[0]
        token=A.commit(m,data,state,old,old,old['phase'],old['phase']);actual=legacy.execute(m,state,data.cert['car'],first,data.cfg,False);shifted,transition=A.shift(data,old,state)
        audit=Replay(m,data.cfg)
        for u in data.cert['inputs'][:old['phase']]:audit.step(data.cert['car'],tuple(map(F,u)),False)
        audit.step(data.cert['car'],first,False,actual)
        assert shifted['budget']==before-1 and actual['executed_input']==list(map(str,first)) and state['history']==audit.history
        fallback.append(dict(interface=name,rejected_result=item['result_id'],old_first=list(map(str,first)),actual=actual,transition=transition,budget_before=before,budget_after=shifted['budget'],history=state['history'],independent_actual_replay=True))
    write(a.output/'coverage_and_directions.json',dict(coverage=coverage,directions=directions))
    write(a.output/'fallback_execution.json',dict(passed=True,cases=fallback,modified_JL_K_test=False,note='No JL candidate accepted; these are actual current old first steps, not modified-JL execution'))
    write(a.output/'checks.json',dict(passed=True,cases=cases,archived_actual_anchor_certificates_reverified=4,old_first_step_replays=2))
    print('Equivalence, exact quantization, archived derivative coverage and two actual old-first-step replays passed')


if __name__=='__main__':main()
