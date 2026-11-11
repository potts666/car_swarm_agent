"""M2a single-vehicle QPs. Peer data are serialized public messages, never an Agent."""
import copy
import math
import time
import numpy as np
import osqp
from scipy.linalg import block_diag
from scipy.sparse import csc_matrix,eye,vstack
from bicycle_core import rollout,audit,corners
from longitudinal_joint import Agent,response,pose,WEIGHTS
from linearized_joint import (condensed,body_support,STATE_TRUST,INPUT_TRUST,ERROR_LIMIT,
                             STATE_WEIGHT,INPUT_WEIGHT,NUMERICAL_BAND,DEADLINE_S,MAX_UPDATES)


def local_corridors(car,reference,permission,scene):
    CARS=(car,);reference={car:reference};permissions={car:permission}
    corridors={};pair=[]
    protection=[-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2]
    for c in CARS:
        corridors[c]=[]
        for z in reference[c][1:]:
            support=body_support(z[2],scene['vehicle']);low=support[:2];high=support[2:]
            candidates=[]
            for rect in scene['road_rectangles']:
                lower=np.array([rect[0],rect[2]])-low;upper=np.array([rect[1],rect[3]])-high
                margin=float(np.min(np.r_[z[:2]-lower,upper-z[:2]]))
                candidates.append((margin,lower,upper,rect))
            _,lower,upper,rect=max(candidates,key=lambda t:t[0])
            guard=None
            if not permissions[c]:
                options=[(protection[0]-z[0]-high[0],0,-1,protection[0]-high[0]),
                         (z[0]+low[0]-protection[1],0,1,protection[1]-low[0]),
                         (protection[2]-z[1]-high[1],1,-1,protection[2]-high[1]),
                         (z[1]+low[1]-protection[3],1,1,protection[3]-low[1])]
                _,axis,side,bound=max(options)
                if side==1:lower[axis]=max(lower[axis],bound)
                else:upper[axis]=min(upper[axis],bound)
                guard=dict(axis=axis,side=side,bound=bound)
            corridors[c].append(dict(lower=lower,upper=upper,support=support,road=rect,guard=guard))
    return corridors[car]


def local_subproblem(car,agent,linear_reference,linear_inputs,target,target_inputs,desired,corridors,peer,parameters,scene,end):
    CARS=(car,);agents={car:agent};parameters={car:parameters}
    linear_reference={car:linear_reference};linear_inputs={car:linear_inputs};target={car:target}
    target_inputs={car:target_inputs};desired={car:desired};corridors={car:corridors}
    N=scene['N'];n=2*N;Hs=[];qs=[];maps={};bases={};rows=[];lower=[];upper=[];bounds=[]
    def row(c,values,lo,hi):
        full=np.zeros(n);i=CARS.index(c)*2*N;full[i:i+2*N]=values
        rows.append(full);lower.append(lo);upper.append(hi)
    for c in CARS:
        a=agents[c];p=parameters[c]
        base,M,_=condensed(a.z,linear_reference[c],linear_inputs[c],scene);bases[c]=base;maps[c]=M
        weights=np.tile(STATE_WEIGHT,N);weights[-5:]*=10.
        flat=M.reshape(5*N,2*N);offset=(base-desired[c][1:]).ravel()
        D=np.eye(N)-np.eye(N,k=-1);smooth=np.kron(D,np.eye(2))
        R=np.diag(np.tile(INPUT_WEIGHT,N))+.02*smooth.T@smooth
        H=2*(flat.T@(weights[:,None]*flat)+R)
        q=2*flat.T@(weights*offset)
        Hs.append(H);qs.append(q)
        for k in range(N):
            trust_low=np.maximum(target[c][k+1]-STATE_TRUST,linear_reference[c][k+1]-STATE_TRUST)
            trust_high=np.minimum(target[c][k+1]+STATE_TRUST,linear_reference[c][k+1]+STATE_TRUST)
            trust_low[0:2]=np.maximum(trust_low[:2],corridors[c][k]['lower'])
            trust_high[0:2]=np.minimum(trust_high[:2],corridors[c][k]['upper'])
            d=a.segment['direction'] if a.mode=='MOVE' else 1
            vlo,vhi=(0.,p['cap']) if d==1 else (-p['cap'],0.)
            trust_low[3]=max(trust_low[3],vlo);trust_high[3]=min(trust_high[3],vhi)
            trust_low[4]=max(trust_low[4],-scene['vehicle']['delta_max']);trust_high[4]=min(trust_high[4],scene['vehicle']['delta_max'])
            for j in range(5):row(c,M[k,j],trust_low[j]-base[k,j],trust_high[j]-base[k,j])
            v=scene['vehicle'];lo=np.maximum([v['a_min'],-v['omega_max']],np.maximum(target_inputs[c][k]-INPUT_TRUST,linear_inputs[c][k]-INPUT_TRUST))
            hi=np.minimum([v['a_max'],v['omega_max']],np.minimum(target_inputs[c][k]+INPUT_TRUST,linear_inputs[c][k]+INPUT_TRUST))
            if a.mode!='MOVE':lo=target_inputs[c][k].copy();hi=lo.copy()
            elif p['stop_at_end']:
                K=p['stop_index']
                if K==1:lo[0]=hi[0]=target_inputs[c][k,0]
                elif k>=K:lo[0]=hi[0]=0.
                if k>=K:lo[1]=hi[1]=0.
            bounds.extend(zip(lo,hi))
        if a.mode=='MOVE':
            times=np.arange(1,N+1)*scene['h'];P,_=response(times,scene['h'],N)
            progress=np.zeros((N,2*N));progress[:,::2]=a.segment['direction']*P
            baseline=p['s0']+times*p['v0']
            for j in range(N):row(c,progress[j],-baseline[j],p['upper']-baseline[j])
            K=p['stop_index'] if p['stop_at_end'] else N
            if p['stop_at_end']:row(c,progress[K-1],p['upper']-baseline[K-1],p['upper']-baseline[K-1])
            row(c,M[K-1,3],-base[K-1,3],-base[K-1,3])
            if p['stop_at_end']:
                endpoint=np.array(a.segment['end'])
                # Stop-to-turn geometry is a small terminal box, not a pose reset.
                for j,tol in ((0,.005),(1,.005),(2,.003)):
                    row(c,M[K-1,j],endpoint[j]-tol-base[K-1,j],endpoint[j]+tol-base[K-1,j])
                row(c,M[K-1,4],a.segment['delta']-base[K-1,4],a.segment['delta']-base[K-1,4])
    separating_rows=[]
    peer_ticks=np.asarray(peer['states'])[::round(scene['h']/scene['sample_s'])]
    for k in range(N):
        body=corners([peer_ticks[k+1]],scene['vehicle'])[0]
        own=corridors[car][k]['support'];nominal=target[car][k+1]
        options=[]
        for axis in (0,1):
            if nominal[axis]<=peer_ticks[k+1,axis]:
                bound=float(body[:,axis].min()-own[2+axis]-scene['s_min'])
                options.append((bound-nominal[axis],axis,'upper',bound))
            else:
                bound=float(body[:,axis].max()-own[axis]+scene['s_min'])
                options.append((nominal[axis]-bound,axis,'lower',bound))
        _,axis,side,bound=max(options)
        separating_rows.append(dict(axis=axis,side=side,bound=bound,peer_plan_id=peer['id']))
        if side=='upper':row(car,maps[car][k,axis],-np.inf,bound-bases[car][k,axis])
        else:row(car,maps[car][k,axis],bound-bases[car][k,axis],np.inf)
    H=block_diag(*Hs);q=np.r_[tuple(qs)];A=np.array(rows);lo=np.array(lower);hi=np.array(upper)
    bounds=np.array(bounds);matrix=vstack([csc_matrix(A),eye(n)],format='csc');lo=np.r_[lo-NUMERICAL_BAND,bounds[:,0]];hi=np.r_[hi+NUMERICAL_BAND,bounds[:,1]]
    fixed=np.flatnonzero(bounds[:,0]==bounds[:,1]);free=np.flatnonzero(bounds[:,0]!=bounds[:,1]);u=np.zeros(n);u[fixed]=bounds[fixed,0]
    offset=np.asarray(matrix@u).ravel();reduced=matrix[:,free];norms=np.asarray(abs(reduced).sum(axis=1)).ravel();keep=norms>1e-12
    status='all inputs fixed';ok=True;iterations=0;raw_wall=0.
    if np.any(lo[~keep]-offset[~keep]>scene['tolerance']) or np.any(offset[~keep]-hi[~keep]>scene['tolerance']):
        ok=False;status='fixed-input constraint inconsistent'
    elif np.any(lo>hi+1e-12):ok=False;status='empty trust/corridor intersection'
    elif len(free):
        remaining=end-time.perf_counter()
        if remaining<=0:ok=False;status='deadline before solver'
        else:
            scale=max(1.,np.max(np.abs(q)),np.max(np.abs(H)));scales=np.maximum(1.,norms[keep])
            solver=osqp.OSQP();started=time.perf_counter()
            solver.setup(P=csc_matrix(H[np.ix_(free,free)]/scale),q=(q+H@u)[free]/scale,
                A=csc_matrix(reduced[keep].multiply(1./scales[:,None])),l=(lo-offset)[keep]/scales,u=(hi-offset)[keep]/scales,
                max_iter=10000,eps_abs=1e-7,eps_rel=1e-7,polishing=True,verbose=False,time_limit=max(1e-6,end-time.perf_counter()),adaptive_rho_interval=50)
            nominal=target_inputs[car].ravel();solver.warm_start(x=nominal[free])
            result=solver.solve(raise_error=False);raw_wall=time.perf_counter()-started
            status=result.info.status;iterations=int(result.info.iter);ok=result.info.status_val in (1,2)
            if result.x is not None:u[free]=result.x
    stop_adjustments=[]
    if ok:
        for i,c in enumerate(CARS):
            agent=agents[c];j=i*2*N
            if agent.mode=='MOVE' and parameters[c]['stop_at_end']:
                end_speed=agent.z[3]+scene['h']*u[j]
                if agent.segment['direction']*end_speed<0.:
                    # Correct the applied braking input before all affine and nonlinear checks.
                    stopping_acceleration=-agent.z[3]/scene['h']
                    stop_adjustments.append(dict(car=c,raw_acceleration=float(u[j]),acceleration=float(stopping_acceleration),
                        initial_speed=float(agent.z[3]),raw_end_speed=float(end_speed)))
                    u[j]=stopping_acceleration
    residual=matrix@u;violation=max(0.,float(np.max(lo-residual)),float(np.max(residual-hi)))
    eig=float(np.linalg.eigvalsh(H)[0]);ok=bool(ok and np.isfinite(u).all() and violation<=scene['tolerance'] and eig>=-1e-10)
    predicted={c:np.vstack((agents[c].z,bases[c]+np.einsum('kij,j->ki',maps[c],u[i*2*N:(i+1)*2*N]))) for i,c in enumerate(CARS)}
    controls={c:u[i*2*N:(i+1)*2*N].reshape(N,2) for i,c in enumerate(CARS)}
    return controls,predicted,dict(accepted=ok,message=status,iterations=iterations,affine_violation=violation,
        hessian_min_eigenvalue=eig,variables=n,free_variables=len(free),affine_constraints=True,solver_wall_s=raw_wall,
        objective_without_constant=float(.5*u@H@u+q@u),solution=u.tolist(),solver_called=bool(raw_wall>0),stop_input_adjustments=stop_adjustments,separating_rows=separating_rows)



def local_reference(agent,permission,peer,scene,warm):
    started=time.perf_counter();N=scene['N'];h=scene['h'];p=agent.parameters(permission)
    times=np.arange(1,N+1)*h;P,V=response(times,h,N);D=np.eye(N)-np.eye(N,k=-1);w=WEIGHTS
    H=2*(w['progress']*P.T@P+w['speed']*V.T@V+w['acceleration']*np.eye(N)+w['input_change']*D.T@D+w['terminal_progress']*np.outer(P[-1],P[-1]))
    base=p['s0']+times*p['v0'];previous=np.r_[agent.previous,np.zeros(N-1)]
    q=2*(w['progress']*P.T@(base-p['end'])+w['speed']*V.T@(np.full(N,p['v0'])-p['cap'])-w['input_change']*D.T@previous+w['terminal_progress']*P[-1]*(base[-1]-p['end']))
    rows=[];lo=[];hi=[]
    def row(mat,l,u):
        mat=np.atleast_2d(mat);rows.extend(mat);lo.extend(np.broadcast_to(l,len(mat)));hi.extend(np.broadcast_to(u,len(mat)))
    if not p['cap']:bounds=np.zeros((N,2))
    else:
        v=scene['vehicle'];amin,amax=(v['a_min'],v['a_max']) if p['direction']==1 else (-v['a_max'],-v['a_min']);K=p['stop_index']
        if p['stop_at_end'] and K==1:bounds=np.array([(-p['v0']/h,-p['v0']/h)]+[(0.,0.)]*(N-1))
        else:bounds=np.array([(amin,amax)]*K+[(0.,0.)]*(N-K))
        count=K-1 if p['stop_at_end'] else K
        row(P[:count],-base[:count],p['upper']-base[:count]);row(V[:max(0,K-1)],-p['v0'],p['cap']-p['v0'])
        if not(p['stop_at_end'] and K==1):
            row(V[K-1],-p['v0'],-p['v0'])
            if p['stop_at_end']:row(P[K-1],p['upper']-base[K-1],p['upper']-base[K-1])
    report=peer['report'];ticks=np.asarray(peer['states'])[::round(h/scene['sample_s'])][1:]
    supports=scene['vehicle']['L']+scene['vehicle']['front']
    if agent.car=='A':
        if not report['ready'] and report['index']==0:
            row(P,-np.inf,ticks[:,0]-2*supports-scene['s_min']-scene['initial']['A'][0]-base)
        elif not report['ready']:
            bound=report['entry_envelope_xmin']-scene['s_min']-supports-scene['initial']['A'][0]
            row(P,-np.inf,bound-base)
        elif report['index']>=3 and report['mode']!='WAIT':
            lower=report['return_envelope_xmax']+scene['s_min']+scene['vehicle']['rear']-scene['initial']['A'][0]
            row(P,lower-base,np.inf)
    elif not agent.ready and agent.index==0:
        row(P,-np.inf,scene['initial']['B'][0]-ticks[:,0]-2*supports-scene['s_min']-base)
    A=np.array(rows).reshape(-1,N);band=1e-6
    matrix=vstack([csc_matrix(A),eye(N)],format='csc');lower=np.r_[np.array(lo)-band,bounds[:,0]];upper=np.r_[np.array(hi)+band,bounds[:,1]]
    fixed=np.flatnonzero(bounds[:,0]==bounds[:,1]);free=np.flatnonzero(bounds[:,0]!=bounds[:,1]);x=np.zeros(N);x[fixed]=bounds[fixed,0]
    offset=np.asarray(matrix@x).ravel();reduced=matrix[:,free];norms=np.asarray(abs(reduced).sum(axis=1)).ravel();keep=norms>1e-12
    status='all inputs fixed';status_val=1;iterations=0;solver_wall=0.;warm_used=False
    if np.any(lower[~keep]-offset[~keep]>scene['tolerance']) or np.any(offset[~keep]-upper[~keep]>scene['tolerance']):status='inconsistent fixed input';status_val=-1
    elif len(free):
        scale=max(1.,np.max(abs(q)),np.max(abs(H)));scales=np.maximum(1.,norms[keep]);solver=osqp.OSQP();before=time.perf_counter()
        solver.setup(P=csc_matrix(H[np.ix_(free,free)]/scale),q=(q+H@x)[free]/scale,A=csc_matrix(reduced[keep].multiply(1/scales[:,None])),
            l=(lower-offset)[keep]/scales,u=(upper-offset)[keep]/scales,eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,polishing=True,verbose=False,adaptive_rho_interval=50)
        if warm is not None:solver.warm_start(x=np.clip(warm,bounds[:,0],bounds[:,1])[free]);warm_used=True
        result=solver.solve(raise_error=False);solver_wall=time.perf_counter()-before
        if result.x is not None:x[free]=result.x
        status=result.info.status;status_val=result.info.status_val;iterations=int(result.info.iter)
    residual=matrix@x;violation=max(0.,float(np.max(lower-residual,initial=0.)),float(np.max(residual-upper,initial=0.)))
    if p['cap'] and p['stop_at_end']:
        K=p['stop_index'];violation=max(violation,abs(float(p['s0']+K*h*p['v0']+P[K-1]@x-p['upper'])),abs(float(p['v0']+V[K-1]@x)))
    eig=float(np.linalg.eigvalsh(H)[0]);accepted=bool(status_val in (1,2) and np.isfinite(x).all() and violation<=scene['tolerance'] and eig>=-1e-10)
    return x,dict(accepted=accepted,message=status,parameters=p,solution=x.tolist(),variables=N,free_variables=len(free),iterations=iterations,
        affine_violation=violation,hessian_min_eigenvalue=eig,solver_wall_s=solver_wall,solve_wall_s=time.perf_counter()-started,warm_start_used=warm_used)


class LocalOptimizer:
    """Own Agent and own committed reference only; peer is a checked wire plan."""
    def __init__(self):self.previous=None

    def predict(self,agent,permission,peer,scene):
        started=time.perf_counter();end=started+DEADLINE_S;N=scene['N'];n=round(scene['h']/scene['sample_s']);car=agent.car
        warm=None
        if self.previous and (agent.index,agent.mode)==(self.previous['index'],self.previous['mode']):warm=self.previous['solution'][1:]+[0.]
        accel,ref=local_reference(agent,permission,peer,scene,warm)
        if not ref['accepted']:return None,None,dict(accepted=False,message='local reference rejected',reference_record=ref,subproblems=[],solve_wall_s=time.perf_counter()-started)
        p=ref['parameters'];stage=agent.segment['stage']
        if agent.mode=='MOVE':commands=[(float(p['direction']*a),0.,stage) for a in accel]
        else:
            fallback='COMPLETE' if agent.mode=='DONE' else 'MUTEX_WAIT' if agent.mode=='WAIT' else stage
            commands=agent.steering[:N]+[(0.,0.,fallback)]*max(0,N-len(agent.steering[:N]))
        dense=rollout(agent.z,commands,scene);target=dense[::n];inputs=np.array(commands,dtype=object)[:,:2].astype(float);desired=target.copy()
        if agent.mode=='MOVE':
            times=np.arange(1,N+1)*scene['h'];P,V=response(times,scene['h'],N);s=p['s0']+times*p['v0']+P@accel
            desired[1:]=np.array([pose(agent.segment['start'],z,agent.segment['delta'],agent.segment['direction'],scene['vehicle']) for z in s]);desired[1:,3]=p['direction']*(p['v0']+V@accel)
        center=target.copy();linear_inputs=inputs.copy();corridor=local_corridors(car,target,permission,scene);records=[];accepted=False;errors=None
        for _ in range(MAX_UPDATES):
            if time.perf_counter()>=end:break
            proposed,predicted,record=local_subproblem(car,agent,center,linear_inputs,target,inputs,desired,corridor,peer,p,scene,end);records.append(record)
            if not record['accepted']:break
            commands=[(float(a),float(w),commands[j][2]) for j,(a,w) in enumerate(proposed[car])];dense=rollout(agent.z,commands,scene);linear=predicted[car]
            errors=dict(position_m=float(np.max(np.linalg.norm(linear[:,:2]-dense[::n,:2],axis=1))),heading_rad=float(np.max(abs(linear[:,2]-dense[::n,2]))),
                speed_m_s=float(np.max(abs(linear[:,3]-dense[::n,3]))),steering_rad=float(np.max(abs(linear[:,4]-dense[::n,4]))))
            trust=bool(np.all(abs(dense[::n]-target)<=STATE_TRUST+scene['tolerance']));safety=audit({car:dense},scene)
            record.update(mapping_errors=errors,trust_replay_passed=trust,nonlinear_safety_passed=safety['passed'])
            if trust and safety['passed'] and all(errors[k]<=v for k,v in ERROR_LIMIT.items()):accepted=True;break
            center=dense[::n].copy();linear_inputs=proposed[car]
        elapsed=time.perf_counter()-started
        return commands,dense,dict(accepted=bool(accepted and elapsed<=DEADLINE_S),message='accepted' if accepted else 'local bicycle or replay rejected',
            reference_record=ref,subproblems=records,qp_updates=len(records),mapping_errors=errors,deadline_exceeded=elapsed>DEADLINE_S,solve_wall_s=elapsed,
            peer_plan_id=peer['id'],reference_numeric_qp_calls=int(ref['free_variables']>0),bicycle_numeric_qp_calls=sum(s['solver_called'] for s in records))

    def remember(self,agent,record):
        if not record['accepted']:raise ValueError('cannot cache rejected reference')
        self.previous=dict(index=agent.index,mode=agent.mode,solution=record['reference_record']['solution'])
