"""Zero-call chronological registration, with the same generator used online."""
import sys,platform
from common import *
from witness import make
from contracts import build,bind,freedom
from execution import advance
from generation import generate,CFG

def main():
    assert read(HERE/'solver_calls.json')==[]
    import numpy,scipy,clarabel,osqp
    write(HERE/'environment.json',dict(python=sys.version,executable=sys.executable,platform=platform.platform(),numpy=numpy.__version__,scipy=scipy.__version__,clarabel=clarabel.__version__,osqp=osqp.__version__,dependency_path='/tmp/dmpc-p0-deps',threads=1))
    write(HERE/'frozen_config.json',dict(physics=CFG,h=H,N=32,rho=RHO,SAFE=SAFE,TUBE=TUBE,quantum='1/1000000000000',Q_reserve='4/1000000000000',scale='1/10000',policy='certificate_suboptimal_v1',solver=dict(version=clarabel.__version__,max_iter=300,time_limit=5,tol_feas=1e-10,tol_gap_abs=1e-10,tol_gap_rel=1e-10),R=[1,1],old_objective_equivalence='003 conic P and q are a common positive 1/scale multiple of normalized physical input objective',max_calls=12))
    s=initial_state();s['witness']=put(HERE/'objects',make(['A1','B1','A2','B2']));initial=s['witness'];scan=[]
    for k in range(2768):
        w=get(HERE/'objects',s['witness']);moving=[c for c in CARS if s['v'][c]!=0];record=dict(cycle=k,moving=moving,checks=[]);scan.append(record)
        eligible=[c for c in moving if any(0<abs(F(x))<1 for u in w['controls'][c][k:k+32] for x in u)]
        record['physical_gradient_prefilter']=eligible
        record['prefilter_reason']='if every nonzero input is saturated, active input rows force projected input-square gradient to zero'
        if len(eligible)>=2:
            cid=build(s);bind(s,cid);good=[]
            for c in eligible:
                g,gid=generate(s,c)
                try:assert g['passed'],g.get('failure');f=freedom(gid)
                except (ValueError,AssertionError) as e:record['checks'].append(dict(actor=c,generation_id=gid,failure=str(e)))
                else:good.append(c);record['checks'].append(dict(actor=c,generation_id=gid,freedom=f))
                write(HERE/'results/scan_progress.json',dict(cycle=k,record=record,new_calls=0))
            if len(good)>=2:
                actors=sorted(good)[:2];s2=copy.deepcopy(s);advance(s2,s2['witness']);s2['contract_id']=None;cid2=build(s2);bind(s2,cid2);second=[]
                for c in actors:
                    g,gid=generate(s2,c);assert g['passed'],g;second.append(dict(actor=c,generation_id=gid,freedom=freedom(gid)))
                write(HERE/'pre_registration.json',dict(passed=True,initial_witness=initial,rule='ascending absolute cycle; two moving vehicles; valid generator, zero anchor, positive radius and projected gradient at k and k+1; lexicographic pair',actors=actors,cycle=k,next_cycle=k+1,state=state_record(s),next_baseline_state=state_record(s2),contract_id=cid,next_baseline_contract_id=cid2,scan=scan,second_baseline=second,second_actual_recompute_required=True,calls_at_freeze=0));print(dict(passed=True,cycle=k,actors=actors,calls=0),flush=True);return
        advance(s,s['witness'])
    raise ValueError('no eligible window')
if __name__=='__main__':main()
