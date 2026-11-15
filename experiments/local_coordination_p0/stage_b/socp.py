"""Three free-terminal SOCPs on the same actual anchor and geometry guards."""
from fractions import Fraction as F
import time
import numpy as np
from scipy import sparse
import clarabel

from common import *
from derivatives import unpack
from socp_anchor import derivative_bounds as global_bounds

METHODS=('B-ZG','B-ZL','B-JL')


def rows_for(data,m,anchor,cert,method):
    assert method in METHODS and cert['anchor_id']==content_id(anchor)
    u=[tuple(map(F,x)) for x in anchor['plan']['remaining'][:32]];c=anchor['actor'];state=restore(m,anchor['state'])
    v,d=state['v'][c],state['delta'][c];rows=[]
    def add(name,g,linear=None,error=0,hessian=0,kind='physical'):
        rows.append(dict(name=name,g_upper=F(g),linear=[F(0)]*64 if linear is None else list(map(F,linear)),gradient_error=F(error),hessian=F(hessian),kind=kind))
    for j,(a,w) in enumerate(u):
        for component,value in enumerate((a,w)):
            for sign in (-1,1):
                gradient=[F(0)]*64;gradient[2*j+component]=F(sign)
                add(f'input_{j}_{component}_{sign}',sign*value-1,gradient)
        v+=H*a;d+=H*w
        for component,value,lower,upper in ((0,v,F(0),F(2)),(1,d,F(-1,2),F(1,2))):
            for sign,bound in ((1,upper),(-1,-lower)):
                gradient=[F(0)]*64
                for k in range(j+1):gradient[2*k+component]=sign*H
                add(f'state_{j}_{component}_{sign}',sign*value-bound,gradient)
    end=cert['terminal_state'];r=data.row(anchor['receiver_phase'])['domain'];pose=[m.I.raw(*p) for p in end['poses_Q'][c]];center=[F(*p) for p in r['center_pose_Q']]
    G=global_bounds() if method=='B-ZG' else {k:F(x) for k,x in cert['local_profiles'][-1].items()}
    matrix=unpack(m,cert['prefix_jacobians'][-1])
    terminal=[]
    for sx in (-1,1):
        for sy in (-1,1):terminal.append((f'position_{sx}_{sy}',(sx*(pose[0]-center[0])+sy*(pose[1]-center[1])).u,F(r['position_l1_radius']),[sx*matrix[0][j]+sy*matrix[1][j] for j in range(64)],F(3,2)*G['Gp'],F(3,2)*G['Hp']))
    for sign in (-1,1):terminal.append((f'yaw_{sign}',(sign*(pose[2]-center[2])).u,F(r['yaw_radius']),[sign*x for x in matrix[2]],G['P1'],G['P2']))
    for name,upper,radius,J,gradient_bound,hessian in terminal:
        if method=='B-JL':
            mid=[F(x.l+x.u,2*m.Q) for x in J];epsilon=norm_upper([F(x.u-x.l,2*m.Q) for x in J])
        else:mid=[F(0)]*64;epsilon=gradient_bound
        add(name,F(upper,m.Q)-radius,mid,epsilon,hessian,'terminal')
    for component,key,radius_name,refname in ((0,'v','speed_radius','center_v'),(1,'delta','steering_radius','center_delta')):
        value=F(end[key][c])-F(r[refname]);radius=F(r[radius_name])
        for sign in (-1,1):
            gradient=[F(0)]*64
            for j in range(32):gradient[2*j+component]=sign*H
            add(f'terminal_{key}_{sign}',sign*value-radius,gradient)
    for j,(profile,margin) in enumerate(zip(cert['local_profiles'],cert['geometry_margins'])):
        add(f'geometry_{j+1}',-F(margin),error=F(profile['Gc']),hessian=F(profile['Hc']),kind='geometry')
    add('hard_trust',-RHO,error=1,kind='trust')
    return rows


def evaluate(rows,delta):
    delta=list(map(F,delta));t=norm_upper(delta);s=t*t;checks=[]
    for row in rows:
        constant=F(row['g_upper']);linear=sum(F(a)*b for a,b in zip(row['linear'],delta));error=F(row['gradient_error'])*t;second=F(row['hessian'])*s/2
        checks.append(dict(name=row['name'],kind=row['kind'],constant=constant,linear=linear,gradient_uncertainty=error,second_order=second,slack=-(constant+linear+error+second)))
    return dict(passed=all(x['slack']>=0 for x in checks),norm_upper=t,rows=checks,violations=[x['name'] for x in checks if x['slack']<0])


def solve(data,m,anchor,cert,method):
    start=time.perf_counter();rows=rows_for(data,m,anchor,cert,method);zero=evaluate(rows,[F(0)]*64)
    record=dict(method=method,anchor_id=content_id(anchor),derivative_id=None,rows=rows,explicit_zero_increment_anchor_passed=zero['passed'],solver_called=False,accepted=False)
    if not zero['passed']:
        return record|dict(status='free_box_temporarily_unavailable',failure_layer='terminal_anchor',violations=zero['violations'],assembly_wall_s=time.perf_counter()-start)
    # Numerical scaling does not change the physical 1e-3 trust ball.
    scale=F(1,10000);matrix=[];rhs=[]
    for row in rows:
        coeff=np.array([float(F(x)*scale) for x in row['linear']]+[float(F(row['gradient_error'])*scale),float(F(row['hessian'])*scale*scale/2)])
        bound=-float(F(row['g_upper']));normalizer=max(abs(bound),float(np.max(abs(coeff))),1e-300)
        matrix.append(coeff/normalizer);rhs.append(bound/normalizer)
    cones=[clarabel.NonnegativeConeT(len(rows))]
    norm=np.zeros((65,66));norm[0,64]=-1;norm[1:,:64]=-np.eye(64)
    rotated=np.zeros((3,66));rotated[0,65]=-1;rotated[1,64]=-2;rotated[2,65]=-1
    M=sparse.csc_matrix(np.vstack([matrix,norm,rotated]));b=np.r_[rhs,np.zeros(65),[1.,0.,-1.]]
    cones.extend([clarabel.SecondOrderConeT(65),clarabel.SecondOrderConeT(3)])
    reference=np.asarray([[float(F(x)) for x in u] for u in anchor['plan']['remaining'][:32]]).ravel()
    P=sparse.diags([float(2*H*scale)]*64+[0.,0.],format='csc');q=np.r_[2*float(H)*reference,0.,0.]
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=300;settings.time_limit=5
    settings.tol_feas=settings.tol_gap_abs=settings.tol_gap_rel=1e-10
    record.update(assembly_wall_s=time.perf_counter()-start,solver_called=True,variables=66,input_variables=64,constraint_rows=M.shape[0],cone_dimensions=[len(rows),65,3],physical_trust_radius=str(RHO),numerical_variable_scale=str(scale),objective_divisor=str(scale),solver_version=clarabel.__version__,solver_tolerance=1e-10)
    solution=clarabel.DefaultSolver(P,q,M,b,cones,settings).solve()
    record.update(status=str(solution.status),primal_residual=solution.r_prim,dual_residual=solution.r_dual,iterations=solution.iterations,solver_wall_s=solution.solve_time,raw_solution=solution.x,raw_objective=solution.obj_val,raw_dual_objective=solution.obj_val_dual,absolute_objective_gap=abs(solution.obj_val-solution.obj_val_dual))
    if str(solution.status)!='Solved':return record|dict(failure_layer='candidate_numeric',rejection='solver status')
    raw=np.asarray(solution.x[:64],dtype=float)*float(scale);record['raw_physical_delta']=raw.tolist()
    record['raw_candidate_energy']=float(H)*float(np.sum((reference+raw)**2))
    source=restore(m,anchor['state']);old=anchor['plan'];attempts=[]
    for alpha in (F(1),F(1,2),F(1,4)):
        repair=time.perf_counter();delta=[F(round(float(alpha)*x*10**12),10**12) for x in raw]
        cone=evaluate(rows,delta);attempt=dict(alpha=str(alpha),rational_delta=list(map(str,delta)),cone=cone,repair_wall_s=time.perf_counter()-repair)
        if cone['passed']:
            commands=[tuple(F(x)+delta[2*j+k] for k,x in enumerate(u)) for j,u in enumerate(old['remaining'][:32])]
            proposal=numerical_plan(data,source,old,commands);validate_start=time.perf_counter()
            try:proof=A.validate_candidate(m,data,source,old,proposal,old['phase'])
            except ValueError as error:attempt.update(passed=False,failure_layer='nonlinear_geometry_terminal_K_energy_budget',rejection=str(error))
            else:
                attempt.update(passed=True,nonlinear_proof=proof,validation_wall_s=time.perf_counter()-validate_start);attempts.append(attempt)
                return record|dict(accepted=True,changed=any(delta),accepted_proposal=proposal,accepted_delta=list(map(str,delta)),accepted_alpha=str(alpha),attempts=attempts,accepted_prefix_energy=str(H*sum(sum(x*x for x in u) for u in commands)),candidate_source='Clarabel raw solution, fixed rationalization/backoff only')
        else:attempt.update(passed=False,failure_layer='rational_cone',rejection=cone['violations'])
        attempts.append(attempt)
    return record|dict(attempts=attempts,failure_layer=attempts[-1]['failure_layer'],rejection=attempts[-1]['rejection'])


def diagnostic(rows,dA):
    d=[F(x) for pair in dA for x in pair];full=evaluate(rows,d)
    low,high=F(0),F(1)
    if full['passed']:low=high
    else:
        for _ in range(60):
            middle=(low+high)/2
            if evaluate(rows,[middle*x for x in d])['passed']:low=middle
            else:high=middle
    check=evaluate(rows,[high*x for x in d]);limiting=sorted(check['rows'],key=lambda r:r['slack'])[:3]
    return dict(A_full_correction_accepted=full['passed'],A_full_violations=full['violations'],alpha_lower=low,alpha_upper=high,limiting_rows=limiting)
