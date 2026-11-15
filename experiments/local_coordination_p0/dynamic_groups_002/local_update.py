"""Parent registers every native call; workers own one 64-input local problem."""
import time,math,os
import numpy as np
import clarabel
from common import *
from generation import generate

from conic import assembly
from rules import quantize,assert_step
from socp_rows import evaluate
from receivers import feedback
from physics import step
from protocol import local_source

def fields(obj):
    return {k:getattr(obj,k) if isinstance(getattr(obj,k),(int,float,bool,str,list)) else str(getattr(obj,k)) for k in dir(obj) if not k.startswith('_') and not callable(getattr(obj,k))}

def reserved_rows(rows):
    result=[];reserves=[];e=F(4,10**12)
    for row in rows:
        eta=(norm_upper(row['linear'])+F(row['gradient_error'])+F(row['hessian'])*RHO)*e+F(row['hessian'])*e*e/2
        rr=copy.deepcopy(row);applied=F(row['g_upper'])+eta<=0;rr['g_upper']=F(rr['g_upper'])+(eta if applied else 0);result.append(rr);reserves.append(dict(name=row['name'],eta=eta,applied=applied))
    return result,reserves

def prepare_job(state,c,gid=None):
    began=time.perf_counter()
    if gid is None:g,gid=generate(state,c)
    else:g=get(HERE/'objects',gid)
    assert g['passed'];f=dict(zero_anchor=True,progress_compiled=True);anchor=get(HERE/'objects',g['anchor_id']);rows=get(HERE/'objects',g['physical_rows_id']);inner,reserves=reserved_rows(rows);ref=np.asarray([float(F(x)) for u in anchor['plan']['remaining'][:32] for x in u]);assembly_start=time.perf_counter();model=assembly(inner,ref,F(1,10000))[-1];model.update(physical_rows=rows,assembled_rows=inner,rounding_reserves=reserves)
    return dict(actor=c,generation_id=gid,anchor_id=g['anchor_id'],derivative_id=g['derivative_id'],assembly_id=put(HERE/'objects',model),freedom=f,source=state_record(state),generation_s=g['wall_s'],assembly_s=time.perf_counter()-assembly_start,prepare_s=time.perf_counter()-began)

def native(pipe):
    from worker_io import Channel
    pipe=Channel(pipe)
    import scipy.sparse as sparse
    boot=time.time_ns();pipe.send(dict(ready=True,pid=os.getpid(),ready_ns=boot))
    job=pipe.recv();model=get(HERE/'objects',job['assembly_id']);P=sparse.csc_matrix(model['P']);A=sparse.csc_matrix(model['A']);q=np.asarray(model['q']);b=np.asarray(model['b']);dims=model['cone_dimensions'];cones=[clarabel.NonnegativeConeT(dims[0])]+[clarabel.SecondOrderConeT(d) for d in dims[1:]]
    cfg=clarabel.DefaultSettings();cfg.max_threads=1;cfg.verbose=True;cfg.max_iter=300;cfg.time_limit=5.;cfg.tol_feas=cfg.tol_gap_abs=cfg.tol_gap_rel=1e-10
    solver=clarabel.DefaultSolver(P,q,A,b,cones,cfg);solver.print_to_buffer();trace=[]
    def callback(info):trace.append(fields(info));return False
    solver.set_termination_callback(callback);pipe.send(dict(assembled=True,pid=os.getpid()));release=pipe.recv();started=time.time_ns();cpu=time.process_time()
    try:
        result=solver.solve();raw=dict(status=str(result.status),x=result.x,s=result.s,z=result.z,info=fields(solver.get_info()),settings=fields(cfg),solver_version=clarabel.__version__,log=solver.get_print_buffer(),trace=trace)
    except Exception as error:raw=dict(status='Exception',error=repr(error))
    raw.update(pid=os.getpid(),started_ns=started,ended_ns=time.time_ns(),cpu_s=time.process_time()-cpu,release_ns=release);pipe.send(raw);pipe.close()

def admit(state,job,raw):
    began=time.perf_counter();g=get(HERE/'objects',job['generation_id']);anchor=get(HERE/'objects',job['anchor_id']);rows=get(HERE/'objects',g['physical_rows_id']);old=get(HERE/'objects',state['witness']);c=job['actor'];k=state['cycle'];attempts=[];rid=put(HERE/'objects',raw);selected=old['controls'][c][k:];accepted=False
    if raw['status'] in ('Solved','AlmostSolved') and all(math.isfinite(x) for key in ('x','s','z') for x in raw[key]):
        for alpha in (F(1),F(1,2),F(1,4)):
            delta,quant=quantize(raw['x'],alpha,anchor['contracts']);proof=evaluate(rows,delta);attempt=dict(alpha=str(alpha),delta=list(map(str,delta)),quantization=quant,rows_id=put(HERE/'objects',proof),passed=False)
            if quant['passed'] and proof['passed']:
                proposal=copy.deepcopy(old);trial=copy.deepcopy(state)
                try:
                    for j in range(32):
                        proposal['controls'][c][k+j]=[str(F(x)+delta[2*j+z]) for z,x in enumerate(old['controls'][c][k+j])];cmd={a:tuple(map(F,proposal['controls'][a][k+j])) for a in CARS};assert_step(trial['v'][c],trial['delta'][c],cmd[c],anchor['contracts'][j]);step(trial,cmd,proposal['owners'][k+j],4)
                    certificate=get(HERE/'objects',anchor['receiver']['certificate_id']);from receivers import check_generated
                    check_generated(trial,c,anchor['receiver']['domain']);proposal['controls'][c][k+32]=list(map(str,feedback(trial,c,certificate)))
                    step(trial,{a:tuple(map(F,proposal['controls'][a][k+32])) for a in CARS},proposal['owners'][k+32],4)
                    assert trial['v'][c]==F(certificate['reset_v']) and trial['delta'][c]==F(certificate['reset_delta'])
                    oldE=H*sum(F(x)**2 for u in old['controls'][c][k:k+32] for x in u);newE=H*sum(F(x)**2 for u in proposal['controls'][c][k:k+32] for x in u);assert newE<=oldE,'energy increase'
                except (AssertionError,ValueError) as error:attempt['failure']=repr(error)
                else:accepted=True;selected=proposal['controls'][c][k:];attempt.update(passed=True,old_energy=str(oldE),new_energy=str(newE))
            attempts.append(attempt)
            if accepted:break
    contract=get(HERE/'objects',state['contract_id'])
    candidate=dict(atlas_id=contract['atlas_id'],obligations_id=content_id(contract['obligations']),actor=c,phase=k,contract_id=state['contract_id'],source=local_source(state,c),generation_id=job['generation_id'],derivative_id=job['derivative_id'],receiver=anchor['receiver'],controls=selected,suffix_id=content_id(selected[33:]),deadline=max(old['goal_times'][c]-k,0),selection='new_candidate' if accepted else 'candidate_contract_anchor',raw_result_id=rid)
    value=dict(job=job,raw_result_id=rid,accepted=accepted,candidate=candidate,attempts=attempts,certificate_policy='certificate_suboptimal_v1',predicted_changed=selected[:32]!=old['controls'][c][k:k+32],first_changed=selected[0]!=old['controls'][c][k],moving=state['v'][c]!=0 or F(old['controls'][c][k][0])!=0,admission_s=time.perf_counter()-began)
    return candidate,value
