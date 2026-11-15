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
