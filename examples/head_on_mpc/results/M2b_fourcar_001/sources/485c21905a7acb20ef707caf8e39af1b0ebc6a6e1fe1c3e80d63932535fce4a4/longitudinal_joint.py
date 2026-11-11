"""Fixed primitive geometry, affine two-car longitudinal QP and bicycle mapping."""
import copy
import math
import time
import numpy as np
from scipy.linalg import block_diag
from scipy.sparse import csc_matrix, eye, vstack
import osqp
from bicycle_core import (CARS, audit, clipped_area, corners, rollout, steer,
                          shift_advance)

WEIGHTS = dict(progress=1., speed=0.1, acceleration=0.1, input_change=0.05, terminal_progress=10.)


def pose(start, length, delta, direction, vehicle):
    q = np.array(start, float).copy()
    kappa = math.tan(delta)/vehicle['L']
    distance = direction*np.asarray(length)
    if abs(kappa)<1e-12:
        q[0] += float(distance)*math.cos(q[2]);q[1] += float(distance)*math.sin(q[2])
    else:
        yaw=q[2]+float(distance)*kappa
        q[0] += (math.sin(yaw)-math.sin(q[2]))/kappa
        q[1] += (math.cos(q[2])-math.cos(yaw))/kappa
        q[2]=yaw
    q[3]=0.;q[4]=delta
    return q


def routes(scene):
    r=scene['turn_radius_m'];d=math.atan(scene['vehicle']['L']/r)
    arc=r*math.acos(1-scene['bay_offset_local_y']/(2*r))
    advance=shift_advance(scene)
    spec={'A':[(84.,0.,'NAVIGATE')],
          'B':[(scene['initial']['B'][0]-scene['waiting_anchor_abs_x'],0.,'QUEUE_APPROACH'),
               (arc,d,'ENTRY'),(arc,-d,'ENTRY'),
               (arc,-d,'RETURN'),(arc,d,'RETURN'),
               (scene['waiting_anchor_abs_x']-2*advance-scene['goals']['B'][0],0.,'DEPART')]}
    spec['A'][0]=(scene['goals']['A'][0]-scene['initial']['A'][0],0.,'NAVIGATE')
    result={}
    for c in CARS:
        origin=np.array(scene['initial'][c],float);path=[];offset=0.
        for index,(distance,delta,stage) in enumerate(spec[c]):
            direction=1 if distance>=0 else -1;length=abs(distance)
            samples=np.linspace(0,length,max(2,math.ceil(length/.01)+1))
            reference=np.array([pose(origin,s,delta,direction,scene['vehicle']) for s in samples])
            body=corners(reference,scene['vehicle'])
            protection=[-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2]
            occupied=[j for j,b in enumerate(body) if clipped_area(b,protection)>scene['tolerance']]
            admission=float(samples[max(0,occupied[0]-1)]) if occupied else length
            cap=min(scene['vehicle']['v_fwd'] if direction==1 else scene['vehicle']['v_rev'],
                    1. if abs(delta)>1e-12 else 2.)
            check=audit({c:reference},scene)
            if not check['passed']:
                raise ValueError('fixed swept route outside road: '+c+str(index))
            path.append(dict(length=length,delta=delta,direction=direction,stage=stage,
                start=origin.tolist(),end=reference[-1].tolist(),offset=offset,
                speed_cap=cap,admission_bound=admission,
                body_xmin=float(body[:,:,0].min()),body_xmax=float(body[:,:,0].max()),
                sweep_step_m=float(samples[1]-samples[0]),sweep_check=check))
            offset+=length;origin=reference[-1]
        result[c]=path
    return result


class Agent:
    def __init__(self, car, path, scene):
        self.car=car;self.path=path;self.scene=scene
        self.z=np.array(scene['initial'][car],float)
        self.index=0;self.s=0.;self.previous=0.;self.ready=False
        self.mode='MOVE';self.steering=[];self.stop_steps=None

    @property
    def segment(self):
        return self.path[min(self.index,len(self.path)-1)]

    @property
    def finished(self):
        return self.index==len(self.path)

    def advance(self, permission):
        if self.mode=='MOVE' and self.s>=self.segment['length']-1e-6 and abs(self.z[3])<1e-6:
            if abs(self.z[3])>1e-12:
                self.stop_steps=1
                return
            old_length=self.segment['length']
            self.index+=1;self.s=old_length if self.finished else 0.;self.previous=0.;self.stop_steps=None
            if self.car=='B' and self.index==3:
                self.ready=True;self.mode='WAIT'
            else:
                self.mode='STEER'
        if self.mode=='WAIT' and permission:
            self.mode='STEER'
        if self.mode=='STEER' or (self.mode=='WAIT' and not self.steering and abs(self.z[4])>1e-7):
            target=0. if self.finished or self.mode=='WAIT' else self.segment['delta']
            if not self.steering:
                self.steering=steer(self.z[4],target,self.scene,'MUTEX_WAIT' if self.mode=='WAIT' else self.segment['stage'])
            if not self.steering and self.mode=='STEER':
                self.mode='DONE' if self.finished else 'MOVE'

    def parameters(self, g):
        if self.mode!='MOVE':
            return dict(s0=self.s,v0=0.,end=self.s,upper=self.s,cap=0.,direction=1,stop_at_end=True,stop_index=0)
        p=self.segment
        sign=1 if self.car=='A' else -1
        cleared=sign*self.z[0]-self.scene['vehicle']['rear']>=self.scene['release_plane_abs_x']+self.scene['release_margin_m']
        upper=p['length'] if g or cleared else p['admission_bound']
        T=self.scene['h']*self.scene['N'];cap=p['speed_cap']
        a=min(self.scene['vehicle']['a_max'],-self.scene['vehicle']['a_min'])
        close=upper-self.s<=cap*max(0.,T-2*cap/a)
        if close and self.stop_steps is None:
            self.stop_steps=self.scene['N']
        return dict(s0=self.s,v0=p['direction']*self.z[3],end=upper,upper=upper,
                    cap=cap,direction=p['direction'],stop_at_end=self.stop_steps is not None,stop_index=self.stop_steps or self.scene['N'])


def response(times, h, N):
    relative=np.asarray(times)[:,None]-np.arange(N)[None,:]*h
    return .5*(np.maximum(relative,0.)**2-np.maximum(relative-h,0.)**2),np.clip(relative,0.,h)


def build_qp(agents, permissions, scene):
    N=scene['N'];h=scene['h'];T=N*h
    times=np.arange(1,N+1)*h;P,V=response(times,h,N)
    # Nonnegative nodal speeds imply monotone s between ticks under constant a.
    dense_times=times;Pd,Vd=P,V
    w=WEIGHTS;D=np.eye(N)-np.eye(N,k=-1)
    Hs=[];qs=[];A=[];lo=[];hi=[];E=[];b=[];bounds=[];parameters={}
    def row(car, mat, lower, upper):
        full=np.zeros((len(mat),2*N));full[:,CARS.index(car)*N:(CARS.index(car)+1)*N]=mat
        A.extend(full);lo.extend(np.broadcast_to(lower,len(mat)));hi.extend(np.broadcast_to(upper,len(mat)))
    for c in CARS:
        agent=agents[c];p=agent.parameters(permissions[c]);parameters[c]=p
        base=p['s0']+times*p['v0'];densebase=p['s0']+dense_times*p['v0']
        H=2*(w['progress']*P.T@P+w['speed']*V.T@V+w['acceleration']*np.eye(N)+w['input_change']*D.T@D+w['terminal_progress']*np.outer(P[-1],P[-1]))
        previous=np.r_[agent.previous,np.zeros(N-1)]
        q=2*(w['progress']*P.T@(base-p['end'])+w['speed']*V.T@(np.full(N,p['v0'])-p['cap'])-w['input_change']*D.T@previous+w['terminal_progress']*P[-1]*(base[-1]-p['end']))
        Hs.append(H);qs.append(q)
        if p['cap']==0.:
            bounds.extend([(0.,0.)]*N)
        else:
            d=p['direction'];v=scene['vehicle']
            amin,amax=(v['a_min'],v['a_max']) if d==1 else (-v['a_max'],-v['a_min'])
            K=p['stop_index']
            if p['stop_at_end'] and K==1:
                bounds.extend([(-p['v0']/h,-p['v0']/h)]+[(0.,0.)]*(N-1))
            else:
                bounds.extend([(amin,amax)]*K+[(0.,0.)]*(N-K))
            count=K-1 if p['stop_at_end'] else K
            row(c,Pd[:count],-densebase[:count],p['upper']-densebase[:count])
            row(c,Vd[:max(0,K-1)],-p['v0'],p['cap']-p['v0'])
            if not (p['stop_at_end'] and K==1):
                full=np.zeros(2*N);full[CARS.index(c)*N:(CARS.index(c)+1)*N]=V[K-1]
                E.append(full);b.append(-p['v0'])
                if p['stop_at_end']:
                    full=np.zeros(2*N);full[CARS.index(c)*N:(CARS.index(c)+1)*N]=P[K-1]
                    E.append(full);b.append(p['upper']-base[K-1])
    # Fixed spatial separating envelopes avoid decision-dependent collision masks.
    aa,bb=agents['A'],agents['B'];pa,pb=parameters['A'],parameters['B']
    supports=scene['vehicle']['L']+scene['vehicle']['front']
    if not bb.ready:
        if bb.index==0:
            full=np.hstack((Pd,Pd))
            upper=scene['initial']['B'][0]-scene['initial']['A'][0]-2*supports-scene['s_min']
            base=pa['s0']+dense_times*pa['v0']+pb['s0']+dense_times*pb['v0']
            A.extend(full);lo.extend(np.full(len(full),-np.inf));hi.extend(upper-base)
        else:
            xmin=min(p['body_xmin'] for p in bb.path[1:3])
            bound=xmin-scene['s_min']-supports-scene['initial']['A'][0]
            row('A',Pd,-np.inf,bound-pa['s0']-dense_times*pa['v0'])
    elif bb.index>=3 and bb.mode not in ('WAIT',):
        xmax=max(p['body_xmax'] for p in bb.path[3:])
        lower=xmax+scene['s_min']+scene['vehicle']['rear']-scene['initial']['A'][0]
        row('A',Pd,lower-pa['s0']-dense_times*pa['v0'],np.inf)
    H=block_diag(*Hs);q=np.concatenate(qs)
    A=np.array(A).reshape(-1,2*N);lo=np.array(lo);hi=np.array(hi)
    E=np.array(E).reshape(-1,2*N);b=np.array(b)
    return H,q,A,lo,hi,E,b,bounds,parameters


def solve(agents, permissions, scene,feasibility_band=0.,warm=None):
    started=time.perf_counter();solver_wall=0.;setup_wall=0.;warm_used=False
    H,q,A,lo,hi,E,b,bounds,parameters=build_qp(agents,permissions,scene)
    scale=max(1.,np.max(np.abs(q)),np.max(np.abs(H)))
    matrix=vstack([csc_matrix(A),csc_matrix(E),eye(len(q))],format='csc')
    lower=np.r_[lo-feasibility_band,b-feasibility_band,[z[0] for z in bounds]];upper=np.r_[hi+feasibility_band,b+feasibility_band,[z[1] for z in bounds]]
    fixed=np.array([i for i,(l,u) in enumerate(bounds) if l==u],int)
    free=np.array([i for i,(l,u) in enumerate(bounds) if l!=u],int)
    x=np.zeros(len(q));x[fixed]=[bounds[i][0] for i in fixed]
    status='all inputs fixed';status_val=1;iterations=0;primal=0.;dual=0.
    if len(free):
        offset=np.asarray(matrix@x).ravel()
        reduced=matrix[:,free]
        norms=np.asarray(abs(reduced).sum(axis=1)).ravel()
        keep=norms>1e-12
        if np.any(lower[~keep]-offset[~keep]>scene['tolerance']) or np.any(offset[~keep]-upper[~keep]>scene['tolerance']):
            raise ValueError('inconsistent fixed-input affine constraint')
        row_scale=np.maximum(1.,norms[keep])
        reduced=csc_matrix(reduced[keep].multiply(1./row_scale[:,None]))
        solver=osqp.OSQP();setup_started=time.perf_counter()
        solver.setup(P=csc_matrix(H[np.ix_(free,free)]/scale),q=(q+H@x)[free]/scale,
            A=reduced,l=(lower-offset)[keep]/row_scale,u=(upper-offset)[keep]/row_scale,
            eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,polishing=True,verbose=False,
            adaptive_rho_interval=50)
        if warm is not None:
            warm=np.asarray(warm,float)
            if warm.shape!=q.shape or not np.isfinite(warm).all():
                raise ValueError('invalid reference warm-start seed')
            warm=np.clip(warm,np.array(bounds)[:,0],np.array(bounds)[:,1])
            solver.warm_start(x=warm[free]);warm_used=True
        setup_wall=time.perf_counter()-setup_started;solver_started=time.perf_counter()
        result=solver.solve(raise_error=False);solver_wall=time.perf_counter()-solver_started
        if result.x is not None:x[free]=result.x
        status=result.info.status;status_val=result.info.status_val;iterations=result.info.iter
        primal=result.info.prim_res;dual=result.info.dual_res
    finite=bool(np.isfinite(x).all())
    residual=matrix@x
    violation=max(0.,float(np.max(lower-residual,initial=0.)),float(np.max(residual-upper,initial=0.)))
    for i,c in enumerate(CARS):
        p=parameters[c]
        if p['cap'] and p['stop_at_end']:
            K=p['stop_index'];Pk,Vk=response([K*scene['h']],scene['h'],scene['N'])
            ui=x[i*scene['N']:(i+1)*scene['N']]
            violation=max(violation,abs(float(p['s0']+K*scene['h']*p['v0']+(Pk@ui)[0]-p['upper'])),abs(float(p['v0']+(Vk@ui)[0])))
    minimum=float(np.linalg.eigvalsh(H)[0])
    record=dict(solver='OSQP '+osqp.__version__,solver_success=status_val in (1,2),
        message=status,iterations=int(iterations),solve_wall_s=time.perf_counter()-started,
        affine_violation=violation,feasibility_band=feasibility_band,hessian_min_eigenvalue=minimum,affine_constraints=True,
        variables=len(q),inequality_rows=len(A),equality_rows=len(E),parameters=parameters,
        accepted=bool(status_val in (1,2) and finite and violation<=scene['tolerance'] and minimum>=-1e-10),
        objective_without_constant=float(.5*x@H@x+q@x),solution=x.tolist(),
        primal_residual=float(primal),dual_residual=float(dual),free_variables=len(free),
        setup_wall_s=setup_wall,solver_wall_s=solver_wall,warm_start_used=warm_used,
        warm_start_solution=warm.tolist() if warm_used else None)
    return {c:x[i*scene['N']:(i+1)*scene['N']] for i,c in enumerate(CARS)},record


class Controller:
    def __init__(self, scene, path, reference_warm_start=False):
        self.reference_warm_start=reference_warm_start;self.reference_previous={}
        self.scene=scene;self.agents={c:Agent(c,path[c],scene) for c in CARS}

    def prepare(self, permissions):
        for c,a in self.agents.items():a.advance(permissions[c])

    def predict(self, permissions):
        seed=None
        if self.reference_warm_start and self.reference_previous:
            seed=[]
            for c,a in self.agents.items():
                previous=self.reference_previous.get(c)
                if previous and (a.index,a.mode)==(previous['index'],previous['mode']):
                    seed.extend(previous['solution'][1:]+[0.])
                else:seed.extend([0.]*self.scene['N'])
        kwargs={'warm':seed} if seed is not None else {}
        accelerations,record=solve(self.agents,permissions,self.scene,getattr(self,'reference_feasibility_band',0.),**kwargs)
        if not record['accepted']:
            controls={c:[(0.,0.,'REJECTED')]*self.scene['N'] for c in CARS}
            predictions={c:rollout(a.z,controls[c],self.scene) for c,a in self.agents.items()}
            record['mapping_errors']={c:dict(position_m=0.,heading_rad=0.,speed_m_s=0.) for c in CARS}
            record['mapping_evaluated']=False
            return controls,predictions,record
        controls={};predictions={};errors={}
        for c,agent in self.agents.items():
            p=record['parameters'][c];stage=agent.segment['stage']
            if agent.mode=='MOVE':
                controls[c]=[(float(p['direction']*a),0.,stage) for a in accelerations[c]]
            else:
                commands=agent.steering[:self.scene['N']]
                fallback='COMPLETE' if agent.mode=='DONE' else 'MUTEX_WAIT' if agent.mode=='WAIT' else stage
                controls[c]=commands+[(0.,0.,fallback)]*(self.scene['N']-len(commands))
            predictions[c]=rollout(agent.z,controls[c],self.scene)
            n=round(self.scene['h']/self.scene['sample_s']);T=self.scene['N']*self.scene['h']
            ticks=np.arange(1,round(T/self.scene['sample_s'])+1)*self.scene['sample_s']
            P,V=response(ticks,self.scene['h'],self.scene['N'])
            sp=p['s0']+ticks*p['v0']+P@accelerations[c]
            vp=p['v0']+V@accelerations[c]
            expected=np.array([pose(agent.segment['start'],s,agent.segment['delta'],agent.segment['direction'],self.scene['vehicle']) for s in sp]) if agent.mode=='MOVE' else predictions[c][1:].copy()
            errors[c]=dict(position_m=float(np.max(np.linalg.norm(expected[:,:2]-predictions[c][1:,:2],axis=1))),
                heading_rad=float(np.max(np.abs(expected[:,2]-predictions[c][1:,2]))),
                speed_m_s=float(np.max(np.abs(p['direction']*vp-predictions[c][1:,3]))),
                path_s_end_m=float(sp[-1]))
        record['mapping_errors']=errors
        return controls,predictions,record

    def remember_reference(self, record):
        if self.reference_warm_start and record['accepted']:
            reference=record.get('reference_record',record);N=self.scene['N']
            self.reference_previous={c:dict(index=a.index,mode=a.mode,solution=reference['solution'][i*N:(i+1)*N])
                                     for i,(c,a) in enumerate(self.agents.items())}

    def commit(self, controls, predictions, record):
        self.remember_reference(record)
        n=round(self.scene['h']/self.scene['sample_s']);h=self.scene['h']
        for c,agent in self.agents.items():
            if agent.mode=='MOVE':
                p=record['parameters'][c];a=record['solution'][CARS.index(c)*self.scene['N']]
                agent.s += h*p['v0']+.5*h*h*a
                agent.previous=float(a)
                if agent.stop_steps is not None:agent.stop_steps-=1
            elif agent.steering:
                agent.steering.pop(0)
            agent.z=predictions[c][n].copy()

    def future_witness(self):
        begun=time.perf_counter();future=copy.deepcopy(self);commands={c:[] for c in CARS};records=[]
        permissions={'A':False,'B':True}
        failure=None
        for _ in range(round(self.scene['timeout_s']/self.scene['h'])):
            if time.perf_counter()-begun>getattr(self,'future_deadline_s',float('inf')):
                failure='future_wall_deadline';break
            future.prepare(permissions)
            if all(a.mode=='DONE' for a in future.agents.values()):break
            controls,predictions,record=future.predict(permissions);records.append(record)
            if not record['accepted'] or not audit(predictions,self.scene)['passed'] or any(max(e[k] for k in ('position_m','heading_rad','speed_m_s'))>getattr(self,'mapping_tolerance',self.scene['tolerance']) for e in record['mapping_errors'].values()):
                failure='future_QP_or_prediction_rejected';break
            for c in CARS:commands[c].append(controls[c][0])
            future.commit(controls,predictions,record)
        else:failure='future_timeout'
        motions={c:rollout(self.agents[c].z,commands[c],self.scene) for c in CARS}
        proof=audit(motions,self.scene)
        from bicycle_core import completed
        proof['both_future_goals']=all(completed(motions[c][-1],self.scene['goals'][c],self.scene) for c in CARS)
        proof['passed'] &= proof['both_future_goals'] and failure is None
        return dict(commands=commands,check=proof,states={c:a.z.tolist() for c,a in self.agents.items()},
                    final={c:z[-1].tolist() for c,z in motions.items()},failure=failure,
                    qp_calls=len(records),wall_s=time.perf_counter()-begun,solves=records)
