"""Four registered solves only; complete public callback and exact dyadic audit."""
from fix_socp import *
import platform,sys

def fields(obj):
    out={}
    for k in dir(obj):
        if k.startswith('_') or callable(getattr(obj,k)):continue
        v=getattr(obj,k);out[k]=v if isinstance(v,(float,int,str,bool,list)) else str(v)
    return out

def strict_interior(model):
    rows=model['assembled_rows'];T=min([RHO/2,F(1,2)]+[-F(r['g_upper'])/(2*(F(r['gradient_error'])+F(r['hessian'])/2)) for r in rows if F(r['gradient_error'])+F(r['hessian'])/2>0]);assert T>0
    slack=[F.from_float(float(b))-F.from_float(float(a[-1]))*T/F(1,10000) for a,b in zip(model['A'],model['b'])];n=model['cone_dimensions'][0];assert all(x>0 for x in slack[:n])
    for size in model['cone_dimensions'][1:]:
        block=slack[n:n+size];assert block[0]>0 and block[0]**2>sum(x*x for x in block[1:]);n+=size
    return dict(passed=True,T=str(T),all_binary64_coefficients_treated_as_exact=True)

def exact_audit(model,solution):
    cv=lambda v:[F.from_float(float(x)) for x in v]
    x,s,z=cv(solution.x),cv(solution.s),cv(solution.z);a=[cv(r) for r in model['A']];p=[cv(r) for r in model['P']];q,b=cv(model['q']),cv(model['b'])
    dot=lambda u,v:sum((t*w for t,w in zip(u,v) if t and w),F(0))
    rp=[dot(row,x)+ss-bb for row,ss,bb in zip(a,s,b)]
    rd=[dot(p[j],x)+q[j]+dot([r[j] for r in a],z) for j in range(65)]
    px=dot(x,[dot(row,x) for row in p]);qx=dot(q,x);bz=dot(b,z)
    def norm(v):return math.sqrt(float(dot(v,v)))
    return dict(primal_vector=list(map(str,rp)),dual_vector=list(map(str,rd)),primal_norm=norm(rp),dual_norm=norm(rd),normalized_primal=norm(rp)/max(1,norm(b)+norm(x)+norm(s)),normalized_dual=norm(rd)/max(1,norm(q)+norm(x)+norm(z)),primal_objective=str(px/2+qx),dual_objective=str(-px/2-bz),b_dot_z=str(bz),cancellation_ratio=float(sum(abs(t*w) for t,w in zip(b,z))/max(1,abs(bz))),objective_energy_scale='1/10000')

def main():
    dest=HERE/'results/probe';assert not dest.exists();dest.mkdir(parents=True)
    data,m=setup();levels=read(HERE/'base_c/analysis/numerical_levels.json');index=read(BASE/'results/enrichment_001/index.json');out={}
    write(dest/'environment.json',dict(python=sys.version,platform=platform.platform(),clarabel=clarabel.__version__,numpy=np.__version__,settings_source='Clarabel.rs v0.11.1 default/settings.rs'))
    for name,item in index.items():
        anchor=get(HERE/'objects',item['anchor_id']);cert=get(HERE/'objects',item['derivative_id']);original=get(HERE/'objects',levels[name+'_Q_1_10000']['assembly_id']);rows=rows_for(data,m,anchor,cert,'B-JL');inner,res=reserved_rows(rows,'Q');assert content_id(inner)==content_id(original['assembled_rows'])
        ref=np.array([[float(F(x)) for x in u] for u in anchor['plan']['remaining'][:32]]).ravel()
        for representation in ('old','centered'):
            P,q,a,b,cones,model=(old_assembly if representation=='old' else assembly)(inner,ref,F(1,10000))
            for k in ('P','q','A','b','cone_dimensions'):
                if representation=='old':assert model[k]==original[k],k
            model.update(physical_rows=rows,assembled_rows=inner,rounding_reserves=res);assembly_id=put(HERE/'objects',model);interior=strict_interior(model)
            cfg=clarabel.DefaultSettings();cfg.verbose=True;cfg.max_iter=300;cfg.time_limit=5.;cfg.tol_feas=cfg.tol_gap_abs=cfg.tol_gap_rel=1e-10
            solver=clarabel.DefaultSolver(P,q,a,b,cones,cfg);solver.print_to_buffer();trace=[]
            def callback(info):trace.append(fields(info));return False
            solver.set_termination_callback(callback);solution=solver.solve();info=fields(solver.get_info());tag=name+'_'+representation
            (dest/(tag+'.log')).write_text(solver.get_print_buffer());write(dest/(tag+'_trace.json'),trace);write(dest/(tag+'_settings.json'),fields(solver.get_settings()));write(dest/(tag+'_exact_audit.json'),exact_audit(model,solution))
            result=dict(method='B-JL',anchor_id=item['anchor_id'],derivative_id=item['derivative_id'],rows=rows,solver_called=True,variables=65,input_variables=64,assembly_id=assembly_id,numerical_variable_scale='1/10000',physical_trust_radius=str(RHO),solver_tolerance=1e-10,solver_version=clarabel.__version__,status=str(solution.status),primal_residual=solution.r_prim,dual_residual=solution.r_dual,raw_solution=solution.x,raw_slack=solution.s,raw_dual=solution.z,raw_objective=solution.obj_val,raw_dual_objective=solution.obj_val_dual,iterations=solution.iterations,solver_wall_s=solution.solve_time,info=info,pre_postprocess_status='unknown: callback does not expose a guaranteed post-termination pre-postprocess state',unavailable=['per-factorization dynamic regularization events','per-iteration refinement counts'],interior=interior,representation=representation,rounding_reserves=res)
            strict=admit(data,m,anchor,cert,result,'strict_solved_v1');strict_id=put(HERE/'objects',strict)
            chosen=strict;policy_id=None
            if representation=='centered' and solution.status==clarabel.SolverStatus.AlmostSolved:
                chosen=admit(data,m,anchor,cert,result,'certificate_suboptimal_v1');policy_id=put(HERE/'objects',chosen)
            out[tag]=dict(result_id=put(HERE/'objects',result),strict_id=strict_id,policy_id=policy_id,chosen_id=put(HERE/'objects',chosen),status=result['status'],strict_accepted=strict['accepted'],accepted=chosen['accepted'],policy=chosen['admission_policy'],info=info)
            write(dest/'summary.json',out);print(tag,result['status'],'accepted',chosen['accepted'],flush=True)
    assert len(out)==4
    return out
if __name__=='__main__':main()
