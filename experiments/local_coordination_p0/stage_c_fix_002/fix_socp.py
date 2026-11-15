"""Centered SOC equivalent; fixed Q rows, scale and strict solver settings."""
import math,time
import numpy as np
from scipy import sparse
import clarabel
from fix_common import *
from c_socp import assembly as old_assembly,nearest,reserved_rows
from socp import rows_for,evaluate


def assembly(rows,reference,scale):
    P,q,a,b,cones,export=old_assembly(rows,reference,scale)
    a=a.toarray();b=b.copy();offset=export['cone_dimensions'][0]+65;branches=[];errors=[]
    for row in rows:
        g=F(row['g_upper']);h=F(row['hessian'])
        if not h:branches.append('linear');continue
        if g<0:
            ell=[scale*F(x)/(-g) for x in row['linear']]+[scale*F(row['gradient_error'])/(-g)]
            beta=2*h*scale**2/(-g);root=math.sqrt(float(beta))
            a[offset]=list(map(float,ell));a[offset+1]=0;a[offset+1,64]=-root;a[offset+2]=a[offset];b[offset:offset+3]=(2,0,0)
            errors.append(dict(name=row['name'],maximum_coefficient_error=str(max(abs(F.from_float(float(x))-x) for x in ell)),root_squared_error=str(F.from_float(root)**2-beta)))
            branches.append('centered')
        else:branches.append('old_valid_cone_nonpositive_margin')
        offset+=3
    assert offset==len(b)
    export.update(A=a.tolist(),b=b.tolist(),representation='centered_equivalent_v1',branches=branches,centered_conversion=errors)
    return P,q,sparse.csc_matrix(a),b,cones,export


def admit(data,m,anchor,cert,source,policy):
    record=copy.deepcopy(source);record.update(admission_policy=policy,accepted=False)
    rows=rows_for(data,m,anchor,cert,'B-JL');assert content_id(rows)==content_id(record['rows'])
    allowed=('Solved',) if policy=='strict_solved_v1' else ('Solved','AlmostSolved') if policy=='certificate_suboptimal_v1' else ()
    if record['status'] not in allowed or not all(math.isfinite(x) for key in ('raw_solution','raw_slack','raw_dual') for x in record[key]):
        return record|dict(failure_layer='candidate_numeric',rejection='status/finite whitelist')
    assert record['anchor_id']==content_id(anchor) and record['derivative_id']==content_id(cert)
    scale=F(record['numerical_variable_scale']);raw=[scale*F.from_float(float(x)) for x in record['raw_solution'][:64]]
    state=restore(m,anchor['state']);old=anchor['plan'];attempts=[]
    for alpha in (F(1),F(1,2),F(1,4)):
        begin=time.perf_counter();delta=nearest([alpha*x for x in raw]);error=norm_upper([d-alpha*x for d,x in zip(delta,raw)]);assert error<=F(4,10**12)
        cone=evaluate(rows,delta);attempt=dict(alpha=str(alpha),rational_delta=list(map(str,delta)),rounding_error=str(error),cone=cone,repair_wall_s=time.perf_counter()-begin)
        if cone['passed']:
            commands=[tuple(F(x)+delta[2*j+k] for k,x in enumerate(u)) for j,u in enumerate(old['remaining'][:32])];proposal=numerical_plan(data,state,old,commands);check=time.perf_counter()
            try:proof=A.validate_candidate(m,data,state,old,proposal,old['phase'])
            except ValueError as exc:attempt.update(passed=False,failure_layer='nonlinear_geometry_terminal_K_energy_budget',rejection=str(exc))
            else:
                attempt.update(passed=True,nonlinear_proof=proof,validation_wall_s=time.perf_counter()-check);attempts.append(attempt)
                return record|dict(accepted=True,changed=any(delta),accepted_proposal=proposal,accepted_delta=list(map(str,delta)),accepted_alpha=str(alpha),attempts=attempts,accepted_prefix_energy=str(H*sum(sum(x*x for x in u) for u in commands)),candidate_source='Clarabel raw solution, exact-rational nearest grid and bounded backoff',rebuilt_t=str(norm_upper(delta)),rebuilt_s=str(norm_upper(delta)**2))
        else:attempt.update(passed=False,failure_layer='rational_cone',rejection=cone['violations'])
        attempts.append(attempt)
    return record|dict(attempts=attempts,failure_layer=attempts[-1]['failure_layer'],rejection=attempts[-1]['rejection'])


def admit_current(data,m,state,old,anchor,cert,raw):
    """Bind a proposal to the current executed state before the existing A_cert."""
    from derivatives import verify
    current=anchor_for(data,m,state,old)
    if content_id(current)!=content_id(anchor):raise ValueError('stale actual anchor')
    verify(data,m,cert,anchor,recompute=False)
    expected=dict(phase=old['phase'],history=state['history'],parent_plan_id=old['id'],anchor_id=content_id(anchor),derivative_id=content_id(cert))
    if any(raw.get(k)!=v for k,v in expected.items()):raise ValueError('stale raw source binding')
    return admit(data,m,anchor,cert,raw,'certificate_suboptimal_v1')


def solve_current(data,m,state,old,anchor,cert,binding,output):
    """Exactly one fresh solve on each actual anchor; raw and policy objects separate."""
    from diagnostics import fields,exact_audit
    from derivatives import verify
    assert content_id(anchor_for(data,m,state,old))==content_id(anchor)
    verify(data,m,cert,anchor,recompute=False)
    rows=rows_for(data,m,anchor,cert,'B-JL');zero=evaluate(rows,[F(0)]*64)
    inner,reserves=reserved_rows(rows,'Q');reference=np.array([[float(F(x)) for x in u] for u in old['remaining'][:32]]).ravel()
    P,q,a,b,cones,model=assembly(inner,reference,F(1,10000));model.update(physical_rows=rows,assembled_rows=inner,rounding_reserves=reserves)
    assembly_id=put(HERE/'objects',model)
    cfg=clarabel.DefaultSettings();cfg.verbose=True;cfg.max_iter=300;cfg.time_limit=5.;cfg.tol_feas=cfg.tol_gap_abs=cfg.tol_gap_rel=1e-10
    solver=clarabel.DefaultSolver(P,q,a,b,cones,cfg);solver.print_to_buffer();trace=[]
    def callback(info):trace.append(fields(info));return False
    solver.set_termination_callback(callback)
    ledger=HERE/'results/solver_calls.json';calls=read(ledger) if ledger.exists() else []
    assert len(calls)<4 and not any(x['binding']==binding for x in calls),'fixed four-call budget or duplicate actual source'
    calls.append(dict(binding=binding,anchor_id=content_id(anchor),derivative_id=content_id(cert),assembly_id=assembly_id,started=True,completed=False));write(ledger,calls)
    solution=solver.solve();info=fields(solver.get_info())
    output.mkdir();(output/'solver.log').write_text(solver.get_print_buffer());write(output/'trace.json',trace);write(output/'settings.json',fields(solver.get_settings()));write(output/'exact_audit.json',exact_audit(model,solution))
    raw=dict(**binding,history=state['history'],parent_plan_id=old['id'],method='B-JL',anchor_id=content_id(anchor),derivative_id=content_id(cert),assembly_id=assembly_id,rows=rows,solver_called=True,variables=65,input_variables=64,numerical_variable_scale='1/10000',physical_trust_radius=str(RHO),solver_tolerance=1e-10,solver_version=clarabel.__version__,status=str(solution.status),primal_residual=solution.r_prim,dual_residual=solution.r_dual,raw_solution=solution.x,raw_slack=solution.s,raw_dual=solution.z,raw_objective=solution.obj_val,raw_dual_objective=solution.obj_val_dual,iterations=solution.iterations,solver_wall_s=solution.solve_time,info=info,representation='centered',rounding_reserves=reserves,explicit_zero_increment_anchor=zero,settings_id=put(HERE/'objects',fields(solver.get_settings())),source_code_sha256=sha(Path(__file__)))
    raw_id=put(HERE/'objects',raw);began=time.perf_counter();policy=admit_current(data,m,state,old,anchor,cert,raw);policy.update(raw_result_id=raw_id,admission_wall_s=time.perf_counter()-began)
    policy_id=put(HERE/'objects',policy);calls[-1].update(completed=True,raw_result_id=raw_id,policy_result_id=policy_id,status=raw['status'],accepted=policy['accepted']);write(ledger,calls)
    return raw_id,policy_id,policy
