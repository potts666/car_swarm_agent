"""Bounded, solver-free prerequisites. Native solving is a separate stage."""
from common import *
from generation import generate
from rules import envelope,contract
from derivatives import builder
from witness import make

def analytic_checks():
    out=[]
    for name,v,a,sign in [('negative',F(-1,5),F(0),1),('zero',F(0),F(0),1),('cross_zero',F(-1,20),F(1,2),1),('negative_opposite_frame',F(-1,5),F(0),-1)]:
        cert=dict(scale=m.Q,car='locator_only',direction_sign=sign,knots=[[str(v),str(v+H*a),'0','0']],pose_nodes_Q=[[[0,0],[0,0],[0,0]]]);J=builder.interval_jacobian(m,cert,0,{},32)
        entries={(r,c):(F(lo,J['scale']),F(hi,J['scale'])) for r,c,lo,hi in J['nonzero_intervals']}
        S=H*v+a*H*H/2;exact={(0,3):sign*H,(0,5):sign*H*H/2,(1,2):sign*S,(2,4):S/m.L,(2,6):(v*H*H/2+a*H**3/3)/m.L,(3,3):F(1),(3,5):H,(4,4):F(1),(4,6):H}
        for key,x in exact.items():lo,hi=entries.get(key,(F(0),F(0)));assert lo<=x<=hi,(name,key,x,lo,hi)
        bound=envelope((v,v+H*a,0,0),contract('signed'));assert bound['V_abs']>=max(abs(v),abs(v+H*a))
        out.append(dict(name=name,passed=True,exact_zoh_displacement=S,expected_signed_J=exact.__str__(),envelope=bound,jacobian_id=put(HERE/'objects',J)))
    return out

def main():
    cases=[]
    for entry in read(HERE/'regression_sources.json'):
        assert content_id(entry['source'])==entry['source_id'] and entry['old_witness']==entry['source']['witness']
        v,key=generate(restore(m,entry['source']),entry['actor']);cases.append(dict(kind=entry['kind'],case=entry.get('case'),side=entry.get('side'),generation_id=key,passed=v['passed'],failure=v.get('failure')))
    mathchecks=analytic_checks();result=dict(passed=all(v['passed'] for v in cases),cases=cases,math=mathchecks,native_solver_calls=0)
    write(HERE/'results/rule_checks_rebuilt.json',result);print(json.dumps(normal(result),ensure_ascii=False,indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
