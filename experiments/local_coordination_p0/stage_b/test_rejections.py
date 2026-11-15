#!/usr/bin/env python3
"""Independent rejection of damaged numerical certificates and actual input source."""
import argparse
from common import *
from validate_numeric import check


def main():
    p=argparse.ArgumentParser();p.add_argument('--index',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('preserve rejection checks')
    a.output.mkdir(parents=True);data,m=setup();item=read(a.index)['curve'];anchor=get(HERE/'objects',item['anchor_id']);cert=get(HERE/'objects',item['derivative_id'])
    # Use a real numerical accepted point, not the A construction fixture.
    from socp import solve
    result=solve(data,m,anchor,cert,'B-ZG');assert result['accepted'];result['derivative_id']=item['derivative_id']
    results=[]
    for kind in ('gradient_error','Hessian_source','rho','trust_origin','domain','K','deadline','actual_input'):
        bad=copy.deepcopy(result);dc=copy.deepcopy(cert)
        if kind=='gradient_error':next(r for r in bad['rows'] if r['kind']=='terminal')['gradient_error']='0'
        if kind=='Hessian_source':next(r for r in bad['rows'] if r['kind']=='terminal')['hessian']='0'
        if kind=='rho':bad['physical_trust_radius']='1/100'
        if kind=='trust_origin':dc['trust_center']='wrong-center'
        if kind in ('domain','K','deadline','actual_input'):
            proposal=bad['accepted_proposal']
            if kind=='domain':proposal['receiver']['domain']['position_l1_radius']='1'
            if kind=='K':proposal['pending_reset']['input']=['1','1']
            if kind=='deadline':proposal['receiver']['deadline']=31
            if kind=='actual_input':proposal['remaining'][0]=['1','1']
            bad['accepted_proposal']=A.seal(proposal)
        try:check(data,m,anchor,dc,bad,recompute=False)
        except (AssertionError,ValueError):results.append(dict(case=kind,rejected=True))
        else:raise AssertionError('corrupted certificate accepted: '+kind)
    write(a.output/'checks.json',dict(passed=True,cases=results,numerical_reference_id=put(HERE/'objects',result)))
    print('Independent numerical rejection checks passed:',len(results))


if __name__=='__main__':main()
