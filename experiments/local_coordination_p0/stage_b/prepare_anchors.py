#!/usr/bin/env python3
"""Build two shared actual anchors and derivative objects exactly once."""
import argparse
import time
from common import *
from derivatives import build,verify


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('preserve prior evidence')
    a.output.mkdir(parents=True);data,m=setup();index={}
    for name,phase in [('curve',5),('straight_acceleration',10)]:
        anchor=initial_anchor(data,m,phase);key=put(HERE/'objects',anchor);t=time.perf_counter()
        cert=build(data,m,anchor,8);verify(data,m,cert,anchor)
        derivative=put(HERE/'objects',cert)
        state=restore(m,anchor['state']);plan=anchor['plan']
        candidate,proof=A.construct(m,data,state,plan,phase)
        fixture=dict(anchor_id=key,candidate=candidate,proof=proof,dA=[[str(F(v)-F(u)) for v,u in zip(n,o)] for n,o in zip(candidate['remaining'][:32],plan['remaining'][:32])])
        fixture_id=put(HERE/'objects',fixture)
        index[name]=dict(anchor_id=key,derivative_id=derivative,A_reference_id=fixture_id,wall_s=time.perf_counter()-t)
        write(a.output/'index.json',index);print(name,'B0 passed',key,derivative,flush=True)
    write(a.output/'summary.json',dict(passed=True,index=index,source='actual anchors, freshly built 32-step intervals'))


if __name__=='__main__':main()
