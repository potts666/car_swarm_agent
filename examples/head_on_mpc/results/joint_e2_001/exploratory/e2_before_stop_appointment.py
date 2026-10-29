"""Fixed primitive geometry, affine two-car longitudinal QP and bicycle mapping."""
import copy
import math
import time
import numpy as np
from scipy.linalg import block_diag
from scipy.optimize import minimize
from bicycle_core import (CARS, audit, clipped_area, corners, drive, rollout, steer,
                          shift_advance, stop)

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
        self.mode='MOVE';self.steering=[]

    @property
    def segment(self):
        return self.path[min(self.index,len(self.path)-1)]

    @property
    def finished(self):
        return self.index==len(self.path)

    def advance(self, permission):
        if self.mode=='MOVE' and self.s>=self.segment['length']-1e-6 and abs(self.z[3])<1e-6:
            old_length=self.segment['length']
            self.index+=1;self.s=old_length if self.finished else 0.;self.previous=0.
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
            return dict(s0=self.s,v0=0.,end=self.s,upper=self.s,cap=0.,direction=1,stop_at_end=True)
        p=self.segment
        sign=1 if self.car=='A' else -1
        cleared=sign*self.z[0]-self.scene['vehicle']['rear']>=self.scene['release_plane_abs_x']+self.scene['release_margin_m']
        upper=p['length'] if g or cleared else p['admission_bound']
        T=self.scene['h']*self.scene['N'];cap=p['speed_cap']
        a=min(self.scene['vehicle']['a_max'],-self.scene['vehicle']['a_min'])
        close=upper-self.s<=cap*max(0.,T-2*cap/a)
        return dict(s0=self.s,v0=p['direction']*self.z[3],end=upper,upper=upper,
                    cap=cap,direction=p['direction'],stop_at_end=bool(close))


def response(times, h, N):
    relative=np.asarray(times)[:,None]-np.arange(N)[None,:]*h
    return .5*(np.maximum(relative,0.)**2-np.maximum(relative-h,0.)**2),np.clip(relative,0.,h)


def build_qp(agents, permissions, scene):
    N=scene['N'];h=scene['h'];T=N*h
    times=np.arange(1,N+1)*h;P,V=response(times,h,N)
    # Subcycle s rows retain affine form and check inter-tick stopping bounds.
    dense_times=np.arange(1,round(T/scene['sample_s'])+1)*scene['sample_s']
    Pd,Vd=response(dense_times,h,N)
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
            bounds.extend([(amin,amax)]*N)
            row(c,Pd,-densebase,p['upper']-densebase)
            row(c,Vd,-p['v0'],p['cap']-p['v0'])
            full=np.zeros(2*N);full[CARS.index(c)*N:(CARS.index(c)+1)*N]=V[-1]
            E.append(full);b.append(-p['v0'])
            if p['stop_at_end']:
                full=np.zeros(2*N);full[CARS.index(c)*N:(CARS.index(c)+1)*N]=P[-1]
                E.append(full);b.append(p['upper']-base[-1])
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


def solve(agents, permissions, scene):
    started=time.perf_counter()
    H,q,A,lo,hi,E,b,bounds,parameters=build_qp(agents,permissions,scene)
    scale=max(1.,np.max(np.abs(q)),np.max(np.abs(H)))
    # SLSQP consumes explicit constant Hessian objective and affine rows only.
    finite_lo=np.isfinite(lo);finite_hi=np.isfinite(hi)
    G=np.vstack((A[finite_lo],-A[finite_hi]));rhs=np.r_[lo[finite_lo],-hi[finite_hi]]
    constraints=[dict(type='ineq',fun=lambda x:G@x-rhs,jac=lambda x:G)]
    if len(E):constraints.append(dict(type='eq',fun=lambda x:E@x-b,jac=lambda x:E))
    result=minimize(lambda x:(.5*x@H@x+q@x)/scale,np.zeros(len(q)),jac=lambda x:(H@x+q)/scale,
        bounds=bounds,constraints=constraints,method='SLSQP',options=dict(maxiter=200,ftol=1e-11))
    x=result.x
    violation=max(0.,float(np.max(rhs-G@x,initial=0.)),float(np.max(np.abs(E@x-b))) if len(E) else 0.,
                  max(max(l-z,z-u,0.) for z,(l,u) in zip(x,bounds)))
    minimum=float(np.linalg.eigvalsh(H)[0])
    record=dict(solver='SciPy SLSQP on explicit convex QP',solver_success=bool(result.success),
        message=result.message,iterations=int(result.get('nit',0)),solve_wall_s=time.perf_counter()-started,
        affine_violation=violation,hessian_min_eigenvalue=minimum,affine_constraints=True,
        variables=len(q),inequality_rows=len(G),equality_rows=len(E),parameters=parameters,
        accepted=bool(result.success and np.isfinite(x).all() and violation<=scene['tolerance'] and minimum>=-1e-10),
        objective_without_constant=float(.5*x@H@x+q@x),solution=x.tolist())
    return {c:x[i*scene['N']:(i+1)*scene['N']] for i,c in enumerate(CARS)},record


class Controller:
    def __init__(self, scene, path):
        self.scene=scene;self.agents={c:Agent(c,path[c],scene) for c in CARS}

    def prepare(self, permissions):
        for c,a in self.agents.items():a.advance(permissions[c])

    def predict(self, permissions):
        accelerations,record=solve(self.agents,permissions,self.scene)
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

    def commit(self, controls, predictions, record):
        n=round(self.scene['h']/self.scene['sample_s']);h=self.scene['h']
        for c,agent in self.agents.items():
            if agent.mode=='MOVE':
                p=record['parameters'][c];a=record['solution'][CARS.index(c)*self.scene['N']]
                agent.s += h*p['v0']+.5*h*h*a
                agent.previous=float(a)
            elif agent.steering:
                agent.steering.pop(0)
            agent.z=predictions[c][n].copy()

    def future_witness(self):
        begun=time.perf_counter();future=copy.deepcopy(self);commands={c:[] for c in CARS};records=[]
        permissions={'A':False,'B':True}
        failure=None
        for _ in range(round(self.scene['timeout_s']/self.scene['h'])):
            future.prepare(permissions)
            if all(a.mode=='DONE' for a in future.agents.values()):break
            controls,predictions,record=future.predict(permissions);records.append(record)
            if not record['accepted'] or not audit(predictions,self.scene)['passed']:
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
