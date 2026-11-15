"""One fresh call per registered source; all adoption is exact certificate admission."""
import time,math
import numpy as np
import clarabel
from common import *
from generation import generate,anchor_for,data_for,CFG
from derivatives import verify
from socp_rows import rows_for,evaluate
from conic import assembly
from physics import step
from certify_domains import check_domain
from receivers import feedback,replacement_events,check_generated,synthesize
from rules import quantize,assert_step


def reserved_rows(rows):
    out=[];res=[];e=F(4,10**12)
    for r in rows:
        eta=(norm_upper(r['linear'])+F(r['gradient_error'])+F(r['hessian'])*RHO)*e+F(r['hessian'])*e*e/2;applied=F(r['g_upper'])+eta<=0
        rr=copy.deepcopy(r);rr['g_upper']=F(rr['g_upper'])+(eta if applied else 0);out.append(rr);res.append(dict(name=r['name'],eta=eta,applied=applied))
    return out,res


def certify_prefix(state,new,anchor):
    actor=anchor['actor'];k=state['cycle'];row=anchor['receiver'];certificate=get(HERE/'objects',row['certificate_id']);trial=copy.deepcopy(state);road=pair=F(100);dom=read(HERE/'domain_checks.json')['domains']
    for t in range(k,certificate['retained_target_phase']):
        if t==k+32:
            terminal=check_generated(trial,actor,row['domain']);assert tuple(map(F,new['controls'][actor][t]))==feedback(trial,actor,certificate),'bad K'
        u={c:tuple(map(F,new['controls'][c][t])) for c in CARS}
        if t<=k+32:assert_step(trial['v'][actor],trial['delta'][actor],u[actor],anchor['contracts'][t-k])
        actual=step(trial,u,new['owners'][t],4);road=min(road,actual['bounds']['road']);pair=min(pair,actual['bounds']['pair'])
        if t==k+32:
            after=state_record(trial);assert trial['v'][actor]==F(certificate['reset_v']) and trial['delta'][actor]==F(certificate['reset_delta'])
        for c,kind in new['events'].get(str(t+1),[]):check_domain(trial,c,kind,dom)
    check_domain(trial,actor,certificate['retained_target_kind'],dom)
    assert 33+certificate['suffix_ticks']==certificate['budget_before']==new['ticks']-k
    return dict(passed=True,terminal=terminal,after_K=after,retained_target=state_record(trial),minimum_road=road,minimum_pair=pair,remaining_budget=new['ticks']-k,receiver_id=row['certificate_id'])


def fields(obj):return {key:getattr(obj,key) if isinstance(getattr(obj,key),(int,float,bool,str,list)) else str(getattr(obj,key)) for key in dir(obj) if not key.startswith('_') and not callable(getattr(obj,key))}


def solve(state,actor,case,output):
    assert read(HERE/'results/rule_checks.json')['passed'],'common rule gate blocked'
    began=time.perf_counter();generation,gid=generate(state,actor);assert generation['passed'],generation
    anchor=get(HERE/'objects',generation['anchor_id']);cert=get(HERE/'objects',generation['derivative_id']);rows=get(HERE/'objects',generation['physical_rows_id']);aid=generation['anchor_id'];did=generation['derivative_id'];inner,reserves=reserved_rows(rows)
    reference=np.array([[float(F(x)) for x in u] for u in anchor['plan']['remaining'][:32]]).ravel();P,q,A,b,cones,model=assembly(inner,reference,F(1,10000));model.update(physical_rows=rows,assembled_rows=inner,rounding_reserves=reserves,generation_id=gid);mid=put(HERE/'objects',model)
    cfg=clarabel.DefaultSettings();cfg.verbose=True;cfg.max_iter=300;cfg.time_limit=5.;cfg.tol_feas=cfg.tol_gap_abs=cfg.tol_gap_rel=1e-10
    solver=clarabel.DefaultSolver(P,q,A,b,cones,cfg);solver.print_to_buffer();trace=[]
    def callback(info):trace.append(fields(info));return False
    solver.set_termination_callback(callback);ledger=HERE/'results/solver_calls.json';calls=read(ledger) if ledger.exists() else [];assert len(calls)<2 and case not in [c['case'] for c in calls]
    calls.append(dict(case=case,phase=state['cycle'],anchor_id=aid,started=True,completed=False));write(ledger,calls)
    result=solver.solve();output.mkdir();(output/'solver.log').write_text(solver.get_print_buffer());write(output/'settings.json',fields(solver.get_settings()));write(output/'trace.json',trace)
    raw=dict(case=case,round=1,phase=state['cycle'],history=state['history'],source_witness=state['witness'],anchor_id=aid,derivative_id=did,assembly_id=mid,generation_id=gid,solver_version=clarabel.__version__,status=str(result.status),x=result.x,s=result.s,z=result.z,info=fields(solver.get_info()),settings=fields(solver.get_settings()),scale='1/10000',rho=str(RHO),quantization='1/1000000000000',policy='certificate_suboptimal_v1');rid=put(HERE/'objects',raw);old=get(HERE/'objects',state['witness']);k=state['cycle'];oldE=H*sum(F(x)**2 for u in old['controls'][actor][k:k+32] for x in u);wid=state['witness'];accepted=False;attempts=[]
    if raw['status'] in ('Solved','AlmostSolved') and all(math.isfinite(x) for key in ('x','s','z') for x in raw[key]):
        for alpha in (F(1),F(1,2),F(1,4)):
            delta,quant=quantize(raw['x'],alpha,anchor['contracts']);cone=evaluate(rows,delta);attempt=dict(alpha=str(alpha),quantization=quant,cone=cone,delta=list(map(str,delta)),passed=False)
            if quant['passed'] and cone['passed']:
                proposal=copy.deepcopy(old);trial=copy.deepcopy(state)
                try:
                    for j in range(32):
                        proposal['controls'][actor][k+j]=[str(F(u)+delta[2*j+i]) for i,u in enumerate(old['controls'][actor][k+j])];step(trial,{c:tuple(map(F,proposal['controls'][c][k+j])) for c in CARS},proposal['owners'][k+j],4)
                    proposal['controls'][actor][k+32]=list(map(str,feedback(trial,actor,get(HERE/'objects',anchor['receiver']['certificate_id']))));proposal.update(parent_witness=state['witness'],optimization_origin=rid,optimization_actor=actor,optimization_phase=k,events=replacement_events(old['events'],actor,anchor['receiver']),receiver_contract=anchor['receiver'],generation_id=gid)
                    E=H*sum(F(x)**2 for u in proposal['controls'][actor][k:k+32] for x in u)
                    assert E<=oldE,'prefix energy increased';proof=certify_prefix(state,proposal,anchor)
                except (ValueError,AssertionError) as error:attempt['failure']=repr(error)
                else:
                    attempt.update(passed=True,proof=proof,energy=str(E));accepted=True;wid=put(HERE/'objects',proposal)
            attempts.append(attempt)
            if accepted:break
    policy=dict(accepted=accepted,changed=accepted and any(delta),raw_result_id=rid,anchor_id=aid,derivative_id=did,assembly_id=mid,generation_id=gid,admission_policy='certificate_suboptimal_v1',accepted_witness=wid,attempts=attempts,old_energy=str(oldE),wall_s=time.perf_counter()-began)
    pid=put(HERE/'objects',policy);calls[-1].update(completed=True,raw_result_id=rid,policy_result_id=pid,status=raw['status'],accepted=accepted);write(ledger,calls)
    return wid,dict(kind='local_socp',case=case,round=1,cycle=k,actor=actor,source_history=state['history'],parent_witness=state['witness'],generation_id=gid,anchor_id=aid,derivative_id=did,assembly_id=mid,raw_result_id=rid,policy_result_id=pid,accepted=accepted,accepted_witness=wid)


def verify_update(wid,recompute=True):
    new=get(HERE/'objects',wid);raw=get(HERE/'objects',new['optimization_origin']);old=get(HERE/'objects',raw['source_witness']);anchor=get(HERE/'objects',raw['anchor_id']);cert=get(HERE/'objects',raw['derivative_id']);state=restore(m,anchor['state']);actor=anchor['actor'];k=state['cycle']
    assert raw['phase']==k and raw['round']==1 and raw['history']==state['history'] and raw['source_witness']==state['witness']==new['parent_witness']
    assert anchor_for(state,actor)==anchor and raw['status'] in ('Solved','AlmostSolved') and raw['solver_version']=='0.11.1' and all(math.isfinite(x) for key in ('x','s','z') for x in raw[key])
    assert new['generation_id']==raw['generation_id'] and get(HERE/'objects',new['generation_id'])['anchor_id']==raw['anchor_id']
    settings=raw['settings'];assert settings['tol_feas']==settings['tol_gap_abs']==settings['tol_gap_rel']==1e-10 and settings['time_limit']==5 and settings['max_iter']==300
    verify(data_for(anchor),m,cert,anchor,recompute);rows=rows_for(data_for(anchor),m,anchor,cert,'B-JL');inner,reserves=reserved_rows(rows);model=get(HERE/'objects',raw['assembly_id']);reference=np.asarray([[float(F(x)) for x in u] for u in old['controls'][actor][k:k+32]]).ravel();rebuilt=assembly(inner,reference,F(1,10000))[-1]
    for key in ('P','q','A','b','cone_dimensions'):assert model[key]==rebuilt[key],key
    assert content_id(model['physical_rows'])==content_id(rows)
    delta=[F(x)-F(y) for u,v in zip(new['controls'][actor][k:k+32],old['controls'][actor][k:k+32]) for x,y in zip(u,v)]
    matches=[alpha for alpha in (F(1),F(1,2),F(1,4)) if (lambda dq:dq[0]==delta and dq[1]['passed'])(quantize(raw['x'],alpha,anchor['contracts']))];assert matches and evaluate(rows,delta)['passed']
    for c in CARS:
        if c==actor:assert new['controls'][c][:k]==old['controls'][c][:k] and new['controls'][c][k+33:]==old['controls'][c][k+33:]
        else:assert new['controls'][c]==old['controls'][c]
    assert new['owners']==old['owners'] and new['events']==replacement_events(old['events'],actor,anchor['receiver']) and new['ticks']==old['ticks']
    assert new['receiver_contract']==anchor['receiver'];assert H*sum(F(x)**2 for u in new['controls'][actor][k:k+32] for x in u)<=H*sum(F(x)**2 for u in old['controls'][actor][k:k+32] for x in u)
    certify_prefix(state,new,anchor);return True
