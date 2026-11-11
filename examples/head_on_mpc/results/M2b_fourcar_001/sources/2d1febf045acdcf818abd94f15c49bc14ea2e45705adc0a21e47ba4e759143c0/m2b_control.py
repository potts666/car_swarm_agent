"""Shared mathematics for true joint four-car and own-only 32/64-variable QPs."""
import copy
import itertools
import time
import numpy as np
import osqp
from scipy.linalg import block_diag
from scipy.sparse import csc_matrix,eye,vstack
from bicycle_core import rollout,corners
from longitudinal_joint import response,pose,WEIGHTS
from linearized_joint import condensed,STATE_TRUST,INPUT_TRUST,ERROR_LIMIT,STATE_WEIGHT,INPUT_WEIGHT,NUMERICAL_BAND,DEADLINE_S,MAX_UPDATES
from m2a_local import local_corridors
from m2b_scene import audit


def solve_matrix(H,q,matrix,lower,upper,bounds,warm,end,eps,max_iter,scene):
    n=len(q);fixed=np.flatnonzero(bounds[:,0]==bounds[:,1]);free=np.flatnonzero(bounds[:,0]!=bounds[:,1]);x=np.zeros(n);x[fixed]=bounds[fixed,0]
    offset=np.asarray(matrix@x).ravel();reduced=matrix[:,free];norms=np.asarray(abs(reduced).sum(axis=1)).ravel();keep=norms>1e-12
    ok=True;status='all inputs fixed';iterations=0;wall=0.;warm_used=False;residuals={}
    if np.any(lower[~keep]-offset[~keep]>scene['tolerance']) or np.any(offset[~keep]-upper[~keep]>scene['tolerance']):ok=False;status='fixed-input inconsistent'
    elif np.any(lower>upper+1e-12):ok=False;status='empty constraint intersection'
    elif len(free):
        if time.perf_counter()>=end:ok=False;status='budget before solver'
        else:
            scale=max(1.,np.max(abs(q)),np.max(abs(H)));scales=np.maximum(1.,norms[keep]);solver=osqp.OSQP();start=time.perf_counter()
            solver.setup(P=csc_matrix(H[np.ix_(free,free)]/scale),q=(q+H@x)[free]/scale,A=csc_matrix(reduced[keep].multiply(1/scales[:,None])),l=(lower-offset)[keep]/scales,u=(upper-offset)[keep]/scales,eps_abs=eps,eps_rel=eps,max_iter=max_iter,polishing=True,verbose=False,adaptive_rho_interval=50,time_limit=max(1e-6,end-time.perf_counter()))
            if warm is not None:solver.warm_start(x=np.clip(warm,bounds[:,0],bounds[:,1])[free]);warm_used=True
            result=solver.solve(raise_error=False);wall=time.perf_counter()-start;status=result.info.status;iterations=int(result.info.iter);ok=result.info.status_val in (1,2)
            if result.x is not None:x[free]=result.x
            residuals=dict(primal=float(result.info.prim_res),dual=float(result.info.dual_res))
    value=matrix@x;violation=max(0.,float(np.max(lower-value,initial=0.)),float(np.max(value-upper,initial=0.)));ok=bool(ok and np.isfinite(x).all() and violation<=scene['tolerance'])
    return x,dict(accepted=ok,message=status,iterations=iterations,solver_wall_s=wall,solver_called=bool(wall),variables=n,free_variables=len(free),warm_start_used=warm_used,affine_violation=violation,solver_residuals=residuals)


def reference_qp(agents,permissions,scene,warm,end):
    started=time.perf_counter();cars=tuple(agents);N=scene['N'];h=scene['h'];P,V=response(np.arange(1,N+1)*h,h,N);D=np.eye(N)-np.eye(N,k=-1);w=WEIGHTS
    Hs=[];qs=[];rows=[];lower=[];upper=[];allbounds=[];parameters={}
    def row(c,mat,lo,hi):
        mat=np.atleast_2d(mat);full=np.zeros((len(mat),N*len(cars)));i=cars.index(c)*N;full[:,i:i+N]=mat;rows.extend(full);lower.extend(np.broadcast_to(lo,len(mat)));upper.extend(np.broadcast_to(hi,len(mat)))
    for c,agent in agents.items():
        p=agent.parameters(permissions[c]);parameters[c]=p;times=np.arange(1,N+1)*h;base=p['s0']+times*p['v0'];previous=np.r_[agent.previous,np.zeros(N-1)]
        H=2*(w['progress']*P.T@P+w['speed']*V.T@V+w['acceleration']*np.eye(N)+w['input_change']*D.T@D+w['terminal_progress']*np.outer(P[-1],P[-1]));q=2*(w['progress']*P.T@(base-p['end'])+w['speed']*V.T@(np.full(N,p['v0'])-p['cap'])-w['input_change']*D.T@previous+w['terminal_progress']*P[-1]*(base[-1]-p['end']))
        Hs.append(H);qs.append(q)
        if not p['cap']:bounds=np.zeros((N,2))
        else:
            v=scene['vehicle'];K=p['stop_index']
            bounds=np.array([(-p['v0']/h,-p['v0']/h)]+[(0.,0.)]*(N-1)) if p['stop_at_end'] and K==1 else np.array([(v['a_min'],v['a_max'])]*K+[(0.,0.)]*(N-K))
            count=K-1 if p['stop_at_end'] else K;row(c,P[:count],-base[:count],p['upper']-base[:count]);row(c,V[:max(0,K-1)],-p['v0'],p['cap']-p['v0'])
            if not(p['stop_at_end'] and K==1):
                row(c,V[K-1],-p['v0'],-p['v0'])
                if p['stop_at_end']:row(c,P[K-1],p['upper']-base[K-1],p['upper']-base[K-1])
        allbounds.extend(bounds)
    H=block_diag(*Hs);q=np.concatenate(qs);bounds=np.array(allbounds);A=np.array(rows).reshape(-1,len(q));matrix=vstack([csc_matrix(A),eye(len(q))],format='csc');lo=np.r_[np.array(lower)-1e-6,bounds[:,0]];hi=np.r_[np.array(upper)+1e-6,bounds[:,1]]
    x,record=solve_matrix(H,q,matrix,lo,hi,bounds,warm,end,1e-8,20000,scene)
    record.update(parameters=parameters,solution=x.tolist(),solve_wall_s=time.perf_counter()-started,_details=dict(H=H,q=q,A=matrix.toarray(),lower=lo,upper=hi))
    for i,c in enumerate(cars):
        p=parameters[c]
        if p['cap'] and p['stop_at_end']:
            K=p['stop_index'];u=x[i*N:(i+1)*N];record['affine_violation']=max(record['affine_violation'],abs(float(p['s0']+K*h*p['v0']+P[K-1]@u-p['upper'])),abs(float(p['v0']+V[K-1]@u)))
    record['accepted'] &= record['affine_violation']<=scene['tolerance']
    return {c:x[i*N:(i+1)*N] for i,c in enumerate(cars)},record


def bicycle_qp(agents,linear_reference,linear_inputs,target,target_inputs,desired,corridors,peers,parameters,scene,end):
    CARS=tuple(agents);N=scene['N'];n=2*N*len(CARS);Hs=[];qs=[];maps={};bases={};rows=[];lower=[];upper=[];bounds=[];state_boxes={c:[] for c in CARS}
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
            state_boxes[c].append((trust_low.copy(),trust_high.copy()))
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
    pairs=[(a,b) for a,b in itertools.combinations(CARS,2)]+[(a,b) for a in CARS for b in peers]
    for a,b in pairs:
      other=target[b] if b in agents else np.asarray(peers[b]['states'])[::round(scene['h']/scene['sample_s'])]
      for k in range(N):
        own=corridors[a][k]['support'];body=corners([other[k+1]],scene['vehicle'])[0]
        support=corridors[b][k]['support'] if b in agents else np.r_[body.min(axis=0)-other[k+1,:2],body.max(axis=0)-other[k+1,:2]]
        options=[]
        for axis in (0,1):
            if target[a][k+1,axis]<=other[k+1,axis]:
                bound=float(support[axis]-own[2+axis]-scene['s_min']);options.append((bound-target[a][k+1,axis]+other[k+1,axis],axis,1,bound))
            else:
                bound=float(own[axis]-support[2+axis]-scene['s_min']);options.append((bound-other[k+1,axis]+target[a][k+1,axis],axis,-1,bound))
        _,axis,sign,bound=max(options);low,high=state_boxes[a][k]
        blo,bhi=state_boxes[b][k] if b in agents else (other[k+1],other[k+1])
        maximum=float(high[axis]-blo[axis] if sign==1 else bhi[axis]-low[axis]);implied=maximum<=bound
        separating_rows.append(dict(own=a,peer=b,node=k+1,axis=axis,sign=sign,bound=bound,maximum_via_boxes=maximum,redundant=implied,peer_plan_id=peers[b]['id'] if b in peers else None))
        if implied:continue
        values=np.zeros(n);i=CARS.index(a)*2*N;values[i:i+2*N]=sign*maps[a][k,axis]
        offset=sign*bases[a][k,axis]
        if b in agents:
            j=CARS.index(b)*2*N;values[j:j+2*N]=-sign*maps[b][k,axis];offset-=sign*bases[b][k,axis]
        else:offset-=sign*other[k+1,axis]
        rows.append(values);lower.append(-np.inf);upper.append(bound-offset)
    H=block_diag(*Hs);q=np.r_[tuple(qs)];A=np.array(rows);lo=np.array(lower);hi=np.array(upper)
    bounds=np.array(bounds);matrix=vstack([csc_matrix(A),eye(n)],format='csc');lo=np.r_[lo-NUMERICAL_BAND,bounds[:,0]];hi=np.r_[hi+NUMERICAL_BAND,bounds[:,1]]
    nominal=np.concatenate([target_inputs[c].ravel() for c in CARS])
    u,raw=solve_matrix(H,q,matrix,lo,hi,bounds,nominal,end,1e-7,10000,scene)
    status=raw['message'];iterations=raw['iterations'];raw_wall=raw['solver_wall_s'];ok=raw['accepted'];free=range(raw['free_variables'])
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
        objective_without_constant=float(.5*u@H@u+q@u),solution=u.tolist(),solver_called=bool(raw_wall>0),stop_input_adjustments=stop_adjustments,separating_rows=separating_rows,_details=dict(H=H,q=q,A=matrix.toarray(),lower=lo,upper=hi,raw=raw.get('_details')))



class Optimizer:
    def __init__(self):self.previous={}
    def predict(self,agents,permissions,peers,scene):
        started=time.perf_counter();end=started+DEADLINE_S;cars=tuple(agents);N=scene['N'];n=round(scene['h']/scene['sample_s']);warm=[];used={}
        for c,a in agents.items():
            last=self.previous.get(c);valid=last is not None and (last['index'],last['mode'])==(a.index,a.mode);used[c]=valid
            warm.extend(last['solution'][1:]+[0.] if valid else [0.]*N)
        accel,ref=reference_qp(agents,permissions,scene,np.array(warm) if any(used.values()) else None,end)
        ref['warm_start_per_car']=used
        commands={};dense={};target={};inputs={};desired={};records=[];errors={};accepted=False
        if ref['accepted']:
            for c,a in agents.items():
                stage=a.segment['stage'];p=ref['parameters'][c]
                commands[c]=[(float(v),0.,stage) for v in accel[c]] if a.mode=='MOVE' else a.steering[:N]+[(0.,0.,a.mode)]*max(0,N-len(a.steering[:N]))
                dense[c]=rollout(a.z,commands[c],scene);target[c]=dense[c][::n];inputs[c]=np.array(commands[c],dtype=object)[:,:2].astype(float);desired[c]=target[c].copy()
                if a.mode=='MOVE':
                    times=np.arange(1,N+1)*scene['h'];P,V=response(times,scene['h'],N);progress=p['s0']+times*p['v0']+P@accel[c]
                    desired[c][1:]=np.array([pose(a.segment['start'],z,a.segment['delta'],1,scene['vehicle']) for z in progress]);desired[c][1:,3]=p['v0']+V@accel[c]
            center={c:z.copy() for c,z in target.items()};linear_inputs={c:u.copy() for c,u in inputs.items()};corridors={c:local_corridors(c,target[c],permissions[c],scene) for c in cars}
            for _ in range(MAX_UPDATES):
                if time.perf_counter()>=end:break
                proposed,linear,sub=bicycle_qp(agents,center,linear_inputs,target,inputs,desired,corridors,peers,ref['parameters'],scene,end);records.append(sub)
                if not sub['accepted']:break
                for c,a in agents.items():
                    commands[c]=[(float(v),float(w),commands[c][i][2]) for i,(v,w) in enumerate(proposed[c])];dense[c]=rollout(a.z,commands[c],scene)
                    errors[c]=dict(position_m=float(np.max(np.linalg.norm(linear[c][:,:2]-dense[c][::n,:2],axis=1))),heading_rad=float(np.max(abs(linear[c][:,2]-dense[c][::n,2]))),speed_m_s=float(np.max(abs(linear[c][:,3]-dense[c][::n,3]))),steering_rad=float(np.max(abs(linear[c][:,4]-dense[c][::n,4]))))
                safe=audit({**{c:np.asarray(p['states']) for c,p in peers.items()},**dense},scene)
                trust=all(np.all(abs(dense[c][::n]-target[c])<=STATE_TRUST+scene['tolerance']) for c in cars)
                sub.update(mapping_errors=errors,nonlinear_safety_passed=safe['passed'],trust_replay_passed=bool(trust))
                if trust and safe['passed'] and all(e[k]<=v for e in errors.values() for k,v in ERROR_LIMIT.items()):accepted=True;break
                center={c:z[::n].copy() for c,z in dense.items()};linear_inputs=proposed
        elapsed=time.perf_counter()-started
        layer='reference' if not ref['accepted'] else 'budget' if elapsed>DEADLINE_S else 'bicycle' if records and not records[-1]['accepted'] else 'replay' if not accepted else None
        record=dict(accepted=bool(accepted and elapsed<=DEADLINE_S),failure_layer=layer,message=ref['message'] if not ref['accepted'] else records[-1]['message'] if records else 'no bicycle update',reference_record=ref,subproblems=records,mapping_errors=errors,qp_updates=len(records),solve_wall_s=elapsed,deadline_s=DEADLINE_S,deadline_exceeded=elapsed>DEADLINE_S,peer_plan_ids={c:p['id'] for c,p in peers.items()},reference_numeric_qp_calls=int(ref['solver_called']),bicycle_numeric_qp_calls=sum(r['solver_called'] for r in records))
        return commands,dense,record
    def remember(self,agents,record):
        if not record['accepted']:raise ValueError('rejected reference cannot be cached')
        N=next(iter(agents.values())).scene['N']
        for i,(c,a) in enumerate(agents.items()):self.previous[c]=dict(index=a.index,mode=a.mode,solution=record['reference_record']['solution'][i*N:(i+1)*N])


def commit_agent(agent,commands,states):
    h=agent.scene['h'];n=round(h/agent.scene['sample_s'])
    if agent.mode=='MOVE':
        acceleration=commands[0][0];agent.s+=h*agent.z[3]+.5*h*h*acceleration;agent.previous=acceleration
        if agent.stop_steps is not None:agent.stop_steps-=1
    elif agent.steering:agent.steering.pop(0)
    agent.z=np.array(states[n])


def predict_joint(optimizer,agents,permissions,scene):
    if len(agents)!=4:raise ValueError('central path requires four agents in one QP')
    return optimizer.predict(agents,permissions,{},scene)
