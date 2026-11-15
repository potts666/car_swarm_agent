"""One 65-variable candidate per case on a genuine two-mover anchor."""
import time,math
import numpy as np
import clarabel
from common import *
from derivatives import build,verify
from socp_rows import rows_for,evaluate
from conic import assembly
from physics import step
from certify_domains import check_domain
from receivers import receiver,replacement_events

CFG=dict(roads=ROADS,h=H,L=F(27,10),body=[[-1,F(7,2)],[-F(9,10),F(9,10)]],v_bounds=[-1,2],delta_abs=F(1,2),a_abs=1,omega_abs=1)

def data_for(actor,phase):
    row=receiver(actor,phase)
    return SimpleNamespace(cfg=CFG,cert={'car':actor,'source':0},row=lambda k:row if k==phase else (_ for _ in ()).throw(ValueError('wrong receiving phase')))

def anchor_for(state,actor):
    w=get(HERE/'objects',state['witness']);k=state['cycle'];data=data_for(actor,k+32)
    value=dict(state=state_record(state),model_sha256=sha(MODEL_PATH),physics_sha256=content_id(CFG),actor=actor,coordinate_branch='A:psi=theta; B:psi=pi+theta',direction_sign=direction(actor),m=32,rho=str(RHO),phase=k,receiver_phase=k+32,receiver_row_sha256=content_id(data.row(k+32)),witness=state['witness'],permission_owner=state['owner'],peer_horizon_id=content_id({c:w['controls'][c][k:k+33] for c in CARS if c!=actor}),plan=dict(phase=k,remaining=w['controls'][actor][k:],budget=2802-k,witness=state['witness']))
    return normal(value)

def reserved_rows(rows):
    out=[];res=[];e=F(4,10**12)
    for r in rows:
        eta=(norm_upper(r['linear'])+F(r['gradient_error'])+F(r['hessian'])*RHO)*e+F(r['hessian'])*e*e/2;applied=F(r['g_upper'])+eta<=0
        rr=copy.deepcopy(r);rr['g_upper']=F(rr['g_upper'])+(eta if applied else 0);out.append(rr);res.append(dict(name=r['name'],eta=eta,applied=applied))
    return out,res

def certify_prefix(state,new,actor):
    k=state['cycle'];trial=copy.deepcopy(state);trial['witness']=content_id(new);dom=read(HERE/'domain_checks.json')['domains'];road=pair=F(100)
    for j in range(33):
        if j==32:terminal=check_domain(trial,actor,receiver(actor,k+32)['kind'],dom)
        cmds={c:tuple(map(F,new['controls'][c][k+j])) for c in CARS};result=step(trial,cmds,new['owners'][k+j],4)
        road=min(road,result['bounds']['road']);pair=min(pair,result['bounds']['pair'])
        for c,kind in new['events'].get(str(k+j+1),[]):check_domain(trial,c,kind,dom)
    assert trial['v'][actor]==F(1,2) and trial['delta'][actor]==0
    return dict(passed=True,terminal=terminal,after_K=state_record(trial),minimum_road=road,minimum_pair=pair,suffix_certificate=new['domain_certificate'],completion_ticks=2802-k)

def fields(obj):
    return {key:getattr(obj,key) if isinstance(getattr(obj,key),(int,float,bool,str,list)) else str(getattr(obj,key)) for key in dir(obj) if not key.startswith('_') and not callable(getattr(obj,key))}


def solve(state,actor,case,output):
    began=time.perf_counter();anchor=anchor_for(state,actor);aid=put(HERE/'objects',anchor);data=data_for(actor,anchor['receiver_phase']);cert=build(data,m,anchor,32);verify(data,m,cert,anchor,recompute=False);did=put(HERE/'objects',cert)
    rows=rows_for(data,m,anchor,cert,'B-JL');inner,reserves=reserved_rows(rows);reference=np.array([[float(F(x)) for x in u] for u in anchor['plan']['remaining'][:32]]).ravel()
    P,q,A,b,cones,model=assembly(inner,reference,F(1,10000));model.update(physical_rows=rows,assembled_rows=inner,rounding_reserves=reserves);mid=put(HERE/'objects',model)
    cfg=clarabel.DefaultSettings();cfg.verbose=True;cfg.max_iter=300;cfg.time_limit=5.;cfg.tol_feas=cfg.tol_gap_abs=cfg.tol_gap_rel=1e-10
    solver=clarabel.DefaultSolver(P,q,A,b,cones,cfg);solver.print_to_buffer();trace=[]
    def callback(info):trace.append(fields(info));return False
    solver.set_termination_callback(callback);calls=read(HERE/'results/solver_calls.json') if (HERE/'results/solver_calls.json').exists() else [];assert len(calls)<4 and (case,state['cycle']) not in [(c['case'],c['phase']) for c in calls]
    calls.append(dict(case=case,phase=state['cycle'],anchor_id=aid,started=True,completed=False));write(HERE/'results/solver_calls.json',calls)
    result=solver.solve();info=fields(solver.get_info());output.mkdir();(output/'solver.log').write_text(solver.get_print_buffer());write(output/'settings.json',fields(solver.get_settings()));write(output/'trace.json',trace)
    raw=dict(case=case,round=state['cycle']-786,phase=state['cycle'],history=state['history'],source_witness=state['witness'],anchor_id=aid,derivative_id=did,assembly_id=mid,solver_version=clarabel.__version__,status=str(result.status),x=result.x,s=result.s,z=result.z,info=info,settings=fields(solver.get_settings()),scale='1/10000',rho=str(RHO),quantization='1/1000000000000',policy='certificate_suboptimal_v1');rid=put(HERE/'objects',raw);accepted=False;attempts=[];old=get(HERE/'objects',state['witness']);oldE=H*sum(F(x)**2 for u in old['controls'][actor][state['cycle']:state['cycle']+32] for x in u);wid=state['witness'];proposal=None
    if raw['status'] in ('Solved','AlmostSolved') and all(math.isfinite(x) for key in ('x','s','z') for x in raw[key]):
        for alpha in (F(1),F(1,2),F(1,4)):
            delta=[F(round(alpha*F(1,10000)*F.from_float(float(x))*10**12),10**12) for x in raw['x'][:64]];cone=evaluate(rows,delta);attempt=dict(alpha=str(alpha),cone=cone,delta=list(map(str,delta)),passed=False)
            if cone['passed']:
                proposal=copy.deepcopy(old);k=state['cycle'];trial=copy.deepcopy(state)
                for j in range(32):
                    proposal['controls'][actor][k+j]=[str(F(u)+delta[2*j+i]) for i,u in enumerate(old['controls'][actor][k+j])]
                    commands={c:tuple(map(F,proposal['controls'][c][k+j])) for c in CARS};step(trial,commands,proposal['owners'][k+j],4)
                K=((F(1,2)-trial['v'][actor])/H,-trial['delta'][actor]/H);proposal['controls'][actor][k+32]=list(map(str,K));proposal.update(parent_witness=state['witness'],optimization_origin=rid,optimization_actor=actor,optimization_phase=k,events=replacement_events(old['events'],actor,k),receiver_contract=receiver(actor,k+32))
                E=H*sum(F(x)**2 for u in proposal['controls'][actor][k:k+32] for x in u)
                try:
                    if E>oldE:raise ValueError('prefix energy increased')
                    proof=certify_prefix(state,proposal,actor)
                except (ValueError,AssertionError) as error:attempt['failure']=repr(error)
                else:
                    attempt.update(passed=True,proof=proof,energy=str(E));attempts.append(attempt);accepted=True;wid=put(HERE/'objects',proposal);break
            attempts.append(attempt)
    policy=dict(accepted=accepted,changed=accepted and any(delta),raw_result_id=rid,anchor_id=aid,derivative_id=did,assembly_id=mid,admission_policy='certificate_suboptimal_v1',accepted_witness=wid,attempts=attempts,old_energy=str(oldE),wall_s=time.perf_counter()-began)
    pid=put(HERE/'objects',policy);calls[-1].update(completed=True,raw_result_id=rid,policy_result_id=pid,status=raw['status'],accepted=accepted);write(HERE/'results/solver_calls.json',calls)
    return wid,dict(kind='local_socp',case=case,round=state['cycle']-786,cycle=state['cycle'],actor=actor,source_history=state['history'],parent_witness=state['witness'],anchor_id=aid,derivative_id=did,assembly_id=mid,raw_result_id=rid,policy_result_id=pid,accepted=accepted,accepted_witness=wid)


def verify_update(wid,recompute=True):
    """Recompute binding and exact rows; no numerical solver called."""
    new=get(HERE/'objects',wid);raw=get(HERE/'objects',new['optimization_origin']);old=get(HERE/'objects',raw['source_witness']);anchor=get(HERE/'objects',raw['anchor_id']);cert=get(HERE/'objects',raw['derivative_id']);state=restore(m,anchor['state']);actor=anchor['actor'];k=state['cycle']
    assert raw['phase']==k and raw['round']==k-786
    assert raw['source_witness']==state['witness']==new['parent_witness'] and new['optimization_actor']==actor and new['optimization_phase']==k and raw['history']==state['history']
    assert anchor_for(state,actor)==anchor and raw['status'] in ('Solved','AlmostSolved') and raw['solver_version']=='0.11.1' and all(math.isfinite(x) for key in ('x','s','z') for x in raw[key])
    settings=raw['settings'];assert settings['tol_feas']==settings['tol_gap_abs']==settings['tol_gap_rel']==1e-10 and settings['time_limit']==5 and settings['max_iter']==300
    data=data_for(actor,k+32);verify(data,m,cert,anchor,recompute);rows=rows_for(data,m,anchor,cert,'B-JL');inner,reserves=reserved_rows(rows);model=get(HERE/'objects',raw['assembly_id']);reference=np.asarray([[float(F(x)) for x in u] for u in old['controls'][actor][k:k+32]]).ravel();rebuilt=assembly(inner,reference,F(1,10000))[-1]
    for key in ('P','q','A','b','cone_dimensions'):assert model[key]==rebuilt[key],key
    delta=[F(x)-F(y) for u,v in zip(new['controls'][actor][k:k+32],old['controls'][actor][k:k+32]) for x,y in zip(u,v)]
    matches=[alpha for alpha in (F(1),F(1,2),F(1,4)) if delta==[F(round(alpha*F(1,10000)*F.from_float(float(x))*10**12),10**12) for x in raw['x'][:64]]];assert matches and evaluate(rows,delta)['passed']
    for c in CARS:
        if c==actor:assert new['controls'][c][:k]==old['controls'][c][:k] and new['controls'][c][k+33:]==old['controls'][c][k+33:]
        else:assert new['controls'][c]==old['controls'][c]
    assert new['owners']==old['owners'] and new['events']==replacement_events(old['events'],actor,k) and new['ticks']-k<=old['ticks']-k
    assert new['receiver_contract']==normal(receiver(actor,k+32))
    trial=copy.deepcopy(state)
    for j in range(32):step(trial,{c:tuple(map(F,new['controls'][c][k+j])) for c in CARS},new['owners'][k+j],4)
    K=((F(1,2)-trial['v'][actor])/H,-trial['delta'][actor]/H);assert tuple(map(F,new['controls'][actor][k+32]))==K
    assert H*sum(F(x)**2 for u in new['controls'][actor][k:k+32] for x in u)<=H*sum(F(x)**2 for u in old['controls'][actor][k:k+32] for x in u)
    certify_prefix(state,new,actor);return True
