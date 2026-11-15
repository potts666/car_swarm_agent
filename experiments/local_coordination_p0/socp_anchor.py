"""Small real-ZOH conic anchor check for one certified moving reference.

Uses the supplied theorem's global derivative bounds. Approximate Jacobians are
zero, with certified gradient-norm error budgets, so the inner approximation is
deliberately conservative. This is not the five-method online controller.
Clarabel Ax+s=b convention: https://clarabel.org/stable/python/getting_started_py/
"""
from fractions import Fraction as F
import math

import numpy as np
from scipy import sparse


class ConicSolveFailure(RuntimeError):
    def __init__(self,solution):
        self.report=dict(status=str(solution.status),solver_called=True,
            solver_time_s=solution.solve_time,iterations=solution.iterations,
            primal_residual=solution.r_prim,dual_residual=solution.r_dual,
            explicit_zero_anchor_passed=True,numerical_solution_used_for_execution=False)
        super().__init__('conic solver status: '+str(solution.status))


def derivative_bounds(n=32,time_steps=None):
    h,L,vmax,rb=F(1,5),F(27,10),F(2),F(1807,500)
    root=math.isqrt(n)
    if root*root<n:root+=1
    beta=h*root;T=h*(n if time_steps is None else time_steps);c,sec=F(11,20),F(64,49)
    P1=T/L*(beta*c+vmax*sec*beta)
    P2=T/L*(2*sec*beta*beta+2*vmax*sec*c*beta*beta)
    Hp=T*(2*beta*P1+vmax*(P2+P1*P1))
    Gp=T*(beta+vmax*P1)
    return dict(P1=P1,P2=P2,Hp=Hp,Hc=Hp+rb*(P2+P1*P1),Gp=Gp,Gc=Gp+rb*P1)


def anchor_problem(controls, v0, delta0, geometry_margin, receiver_rp, receiver_rpsi,
                   free_steps=32,stop_indices=()):
    """Full input/state rows and (16)–(21), with a conservative tube contract.

    The certified reference envelope supplies geometry_margin; a corner tube
    smaller than that margin preserves road, pair and authority contracts.
    Receiver radii must separately be certified subsets of the image domain.
    """
    import clarabel
    n=len(controls); dim=2*n+2; t,s=dim-2,dim-1; h=F(1,5)
    if n!=32 or not 1<=free_steps<=n or min(geometry_margin,receiver_rp,receiver_rpsi)<0:
        raise ValueError('expected 32 controls and nonnegative certified margins')
    u=[tuple(map(F,row)) for row in controls];v,delta=F(v0),F(delta0)
    if not 0<=v<=2 or abs(delta)>F(1,2):raise ValueError('anchor initial-state bound')
    coeff=derivative_bounds(n)
    boundary=receiver_rp==0 or receiver_rpsi==0 or geometry_margin==0
    scale=F(1) if boundary else min(receiver_rp/(coeff['Gp']+1),
        receiver_rpsi/(coeff['P1']+1),geometry_margin/(coeff['Gc']+1),F(1,100))/2
    if not boundary and scale<=0:
        raise ValueError('positive-radius optimization requires positive geometry reserve')
    rows=[];bounds=[]; exact_slacks=[]

    def row(values, bound):
        r=np.zeros(dim)
        for j,value in values.items():r[j]=float(value*scale if j<t else value*scale if j==t else value*scale*scale)
        normalizer=max(float(abs(bound)),float(np.max(abs(r))),1e-300)
        rows.append(r/normalizer);bounds.append(float(bound)/normalizer);exact_slacks.append(F(bound))

    for k,(a,w) in enumerate(u):
        for j,x in ((2*k,a),(2*k+1,w)):
            row({j:1},1-x);row({j:-1},1+x)
        v+=h*a;delta+=h*w
        row({2*j:h for j in range(k+1)},2-v)
        row({2*j:-h for j in range(k+1)},v)
        row({2*j+1:h for j in range(k+1)},F(1,2)-delta)
        row({2*j+1:-h for j in range(k+1)},F(1,2)+delta)
    row({t:1},scale if not boundary else F(1,100))
    row({t:coeff['Gc'],s:coeff['Hc']/2},geometry_margin)
    row({t:coeff['Gp'],s:coeff['Hp']/2},receiver_rp)
    row({t:coeff['P1'],s:coeff['P2']/2},receiver_rpsi)
    # Exact verification of the zero-increment anchor, before any float solve.
    if min(exact_slacks)<0:
        raise ValueError('the supplied reference is not a feasible anchor')
    for k in stop_indices:
        if not 0<k<=free_steps or F(v0)+h*sum(u[j][0] for j in range(k))!=0:
            raise ValueError('anchor misses a physical stop')
    constant=all(u[j]==u[0] for j in range(free_steps))
    if boundary or constant:
        # With positive gradient budgets and r_p=0 or r_psi=0, t=s=0,
        # and the norm cone forces Delta=0. This unique optimum is exact.
        return dict(status='exact_unique_anchor' if boundary else 'exact_energy_minimum',solver_called=False,
            explicit_zero_anchor_passed=True,boundary_receiver_forces_zero_increment=boundary,
            coefficients={k:str(v) for k,v in coeff.items()},
            minimum_exact_anchor_linear_slack=str(min(exact_slacks)),
            variables=dim,actual_ZOH_input_variables=2*n,numerical_increment_norm=0.,
            numerical_variables=0,numerical_constraint_rows=0,
            numerical_solution_used_for_execution=False,
            scope='exact boundary-case presolve, not a numeric solver success')
    matrices=[np.asarray(rows)];rhs=[np.asarray(bounds)]
    cones=[clarabel.NonnegativeConeT(len(rows))]
    # Terminal v/delta equal their certified reference values, including nonzero values.
    equations=[]
    for component in (0,1):
        r=np.zeros(dim);r[component:2*free_steps:2]=float(h);equations.append(r)
    for k in stop_indices:
        if not 0<k<=free_steps:raise ValueError('stop outside free prefix')
        if F(v0)+h*sum(u[j][0] for j in range(k))!=0:raise ValueError('anchor misses physical stop')
        r=np.zeros(dim);r[:2*k:2]=float(h);equations.append(r)
    for k in range(free_steps,n):
        for component in (0,1):
            r=np.zeros(dim);r[2*k+component]=1.;equations.append(r)
    equality=np.asarray(equations)
    matrices.append(equality);rhs.append(np.zeros(len(equations)));cones.append(clarabel.ZeroConeT(len(equations)))
    # t >= ||Delta w||.
    soc=np.zeros((2*n+1,dim));soc[0,t]=-1;soc[1:,:2*n]=-np.eye(2*n)
    matrices.append(soc);rhs.append(np.zeros(2*n+1));cones.append(clarabel.SecondOrderConeT(2*n+1))
    # (s+1, 2t, s-1) in SOC is exactly t^2 <= s.
    rotated=np.zeros((3,dim));rotated[0,s]=-1;rotated[1,t]=-2;rotated[2,s]=-1
    matrices.append(rotated);rhs.append(np.array([1.,0.,-1.]));cones.append(clarabel.SecondOrderConeT(3))
    A=sparse.csc_matrix(np.vstack(matrices));b=np.concatenate(rhs)
    zero=np.zeros(dim);slack=b-A@zero
    # Cone values at zero are (0,...,0) and (1,0,-1); both valid exactly.
    assert np.all(slack[:len(rows)]>=0) and np.all(equality@zero==0)
    P=sparse.diags([2*float(scale)]*(2*n)+[0.,0.],format='csc');q=np.r_[2*np.asarray(u,dtype=float).ravel(),0.,0.]
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=300
    # Development numeric precision, recorded separately from the exact anchor.
    # Stricter 1e-12 attempts and measured residuals are retained in 007/008.
    settings.time_limit=5.;settings.tol_feas=1e-10;settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-10
    solution=clarabel.DefaultSolver(P,q,A,b,cones,settings).solve()
    if str(solution.status)!='Solved':
        error=ConicSolveFailure(solution)
        error.report.update(numerical_variables=dim,numerical_constraint_rows=A.shape[0])
        raise error
    x=np.array(solution.x)
    return dict(status=str(solution.status), solver_version=clarabel.__version__,
        solver_called=True,variable_scale=str(scale),
        numerical_tolerance=1e-10,algorithm_parameter_frozen=False,
        primal_residual=solution.r_prim,dual_residual=solution.r_dual,
        solver_time_s=solution.solve_time,iterations=solution.iterations,
        variables=dim,actual_ZOH_input_variables=2*n,
        numerical_variables=dim,numerical_constraint_rows=A.shape[0],cone_dimensions=[len(rows),len(equations),2*n+1,3],
        explicit_zero_anchor_passed=True,minimum_exact_anchor_linear_slack=str(min(exact_slacks)),
        coefficients={k:str(v) for k,v in coeff.items()},
        geometry_margin=str(geometry_margin),receiver_rp=str(receiver_rp),receiver_rpsi=str(receiver_rpsi),
        numerical_increment_norm=float(scale)*float(np.linalg.norm(x[:2*n])),
        scaled_numerical_solution=x.tolist(),numerical_solution_used_for_execution=False,
        source_domain_or_receiver_subset_proof_supplied_by_caller=True,
        boundary_receiver_forces_zero_increment=(receiver_rp==0 or receiver_rpsi==0),
        scope='isolated cone/anchor feasibility; no controller integration or runtime plan acceptance')
