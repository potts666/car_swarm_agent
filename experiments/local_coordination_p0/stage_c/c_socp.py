"""Equivalent per-row SOC projection, with optional proven quantization reserve."""
import time,math
import numpy as np
from scipy import sparse
import clarabel
from c_common import *
from socp import rows_for,evaluate,diagnostic


def nearest(delta):return [F(round(F(x)*10**12),10**12) for x in delta]


def reserved_rows(rows,variant):
    eq=F(4,10**12);out=[];reserves=[]
    for row in rows:
        eta=(norm_upper(row['linear'])+F(row['gradient_error'])+F(row['hessian'])*RHO)*eq+F(row['hessian'])*eq*eq/2
        applied=variant=='Q' and F(row['g_upper'])+eta<=0
        new=copy.deepcopy(row);new['g_upper']=F(row['g_upper'])+(eta if applied else 0);out.append(new)
        reserves.append(dict(name=row['name'],eta=eta,applied=applied,reason='nearest-grid proof' if applied else 'equivalent original row' if variant=='R' else 'anchor margin insufficient; original row and bounded fallback retained'))
    return out,reserves


def assembly(rows,reference,scale):
    nonneg=[];rhs=[];socblocks=[];socrhs=[];cones=[];normalizers=[];conversions=[]
    for row in rows:
        a=list(map(F,row['linear']));g=F(row['g_upper']);e=F(row['gradient_error']);Hess=F(row['hessian'])
        L=max(abs(g),*(abs(x*scale) for x in a),abs(e*scale),Hess*scale*scale/2,F(1,10**100))
        linear=[x*scale/L for x in a]+[e*scale/L];rconst=-g/L;normalizers.append(str(L))
        errors=[abs(F.from_float(float(x))-x) for x in linear]
        if Hess==0:
            nonneg.append(list(map(float,linear)));rhs.append(float(rconst))
            conversions.append(dict(name=row['name'],normalizer=str(L),maximum_coefficient_error=str(max(errors)),constant_error=str(abs(F.from_float(float(rconst))-rconst))))
        else:
            K=2*Hess*scale*scale/L;root=math.sqrt(float(K));block=np.zeros((3,65));block[0]=list(map(float,linear));block[2]=block[0];block[1,64]=-root
            socblocks.append(block);socrhs.extend([float(rconst+1),0.,float(rconst-1)])
            conversions.append(dict(name=row['name'],normalizer=str(L),root_squared_error=str(abs(F.from_float(root)**2-K)),maximum_coefficient_error=str(max(errors)),constant_plus_error=str(abs(F.from_float(float(rconst+1))-(rconst+1))),constant_minus_error=str(abs(F.from_float(float(rconst-1))-(rconst-1)))))
    norm=np.zeros((65,65));norm[0,64]=-1;norm[1:,:64]=-np.eye(64)
    blocks=[np.asarray(nonneg),norm,*socblocks];b=np.r_[rhs,np.zeros(65),socrhs]
    cones=[clarabel.NonnegativeConeT(len(nonneg)),clarabel.SecondOrderConeT(65)]+[clarabel.SecondOrderConeT(3) for _ in socblocks]
    P=sparse.diags([float(2*H*scale)]*64+[0.],format='csc');q=np.r_[2*float(H)*reference,0.]
    return P,q,sparse.csc_matrix(np.vstack(blocks)),b,cones,dict(row_normalizers=normalizers,coefficient_conversion=conversions,cone_dimensions=[len(nonneg),65]+[3]*len(socblocks),P=P.toarray().tolist(),q=q.tolist(),A=np.vstack(blocks).tolist(),b=b.tolist(),representation='R=-f_linear/L; (R+1,sqrt(2*Hrow*scale^2/L)*tau,R-1) in SOC')


def solve(data,m,anchor,cert,method):
    settings=read(HERE/'selected_repair.json');variant=settings['variant'];scale=F(settings['input_scale']);start=time.perf_counter()
    rows=rows_for(data,m,anchor,cert,method);zero=evaluate(rows,[F(0)]*64)
    record=dict(method=method,anchor_id=content_id(anchor),rows=rows,explicit_zero_increment_anchor_passed=zero['passed'],solver_called=False,accepted=False,variant=variant)
    if not zero['passed']:return record|dict(status='free_box_temporarily_unavailable',failure_layer='terminal_anchor',violations=zero['violations'],assembly_wall_s=time.perf_counter()-start)
    inner,reserves=reserved_rows(rows,variant);innerzero=evaluate(inner,[F(0)]*64);assert innerzero['passed']
    reference=np.array([[float(F(x)) for x in u] for u in anchor['plan']['remaining'][:32]]).ravel()
    P,q,A,b,cones,export=assembly(inner,reference,scale);export['physical_rows']=rows;export['assembled_rows']=inner;export['rounding_reserves']=reserves
    assembly_id=put(HERE/'objects',export)
    cfg=clarabel.DefaultSettings();cfg.verbose=False;cfg.max_iter=300;cfg.time_limit=5.;cfg.tol_feas=cfg.tol_gap_abs=cfg.tol_gap_rel=1e-10
    record.update(assembly_wall_s=time.perf_counter()-start,solver_called=True,variables=65,input_variables=64,constraint_rows=A.shape[0],cone_dimensions=export['cone_dimensions'],assembly_id=assembly_id,physical_trust_radius=str(RHO),numerical_variable_scale=str(scale),objective_divisor=str(scale),solver_version=clarabel.__version__,solver_tolerance=1e-10,rounding_reserves=reserves,quantization_error_bound='1/250000000000')
    result=clarabel.DefaultSolver(P,q,A,b,cones,cfg).solve()
    record.update(status=str(result.status),primal_residual=result.r_prim,dual_residual=result.r_dual,iterations=result.iterations,solver_wall_s=result.solve_time,raw_solution=result.x,raw_objective=result.obj_val,raw_dual_objective=result.obj_val_dual,absolute_objective_gap=abs(result.obj_val-result.obj_val_dual))
    if str(result.status)!='Solved':return record|dict(failure_layer='candidate_numeric',rejection='solver status')
    raw=[scale*F.from_float(float(x)) for x in result.x[:64]];record['raw_physical_delta']=list(map(float,raw));record['raw_physical_delta_exact']=list(map(str,raw))
    record['raw_candidate_energy']=float(H*sum((F(u)+d)**2 for u,d in zip((x for row in anchor['plan']['remaining'][:32] for x in row),raw)))
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
                return record|dict(accepted=True,changed=any(delta),accepted_proposal=proposal,accepted_delta=list(map(str,delta)),accepted_alpha=str(alpha),attempts=attempts,accepted_prefix_energy=str(H*sum(sum(x*x for x in u) for u in commands)),candidate_source='Clarabel raw solution, exact-rational nearest grid and bounded backoff')
        else:attempt.update(passed=False,failure_layer='rational_cone',rejection=cone['violations'])
        attempts.append(attempt)
    return record|dict(attempts=attempts,failure_layer=attempts[-1]['failure_layer'],rejection=attempts[-1]['rejection'])
