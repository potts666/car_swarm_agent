"""Time-varying affine bicycle QPs with frozen corridors and bounded updates."""
import math
import time
import numpy as np
import osqp
from scipy.linalg import block_diag
from scipy.sparse import csc_matrix,eye,vstack
from bicycle_core import CARS,derivative,step,rollout,corners,audit
from longitudinal_joint import Controller,response,pose

STATE_TRUST=np.array([.15,.15,.02,.25,.03])
INPUT_TRUST=np.array([.3,.25])
ERROR_LIMIT=dict(position_m=.0001,heading_rad=.00005,speed_m_s=1e-6,steering_rad=1e-6)
STATE_WEIGHT=np.array([10.,40.,100.,1.,10.])
INPUT_WEIGHT=np.array([.05,.05])
DEADLINE_S=.5
MAX_UPDATES=3
NUMERICAL_BAND=1e-7


def continuous_jacobians(z,vehicle):
    _,_,psi,v,delta=z;L=vehicle['L']
    A=np.zeros((5,5));B=np.zeros((5,2))
    A[0,2]=-v*math.sin(psi);A[0,3]=math.cos(psi)
    A[1,2]=v*math.cos(psi);A[1,3]=math.sin(psi)
    A[2,3]=math.tan(delta)/L;A[2,4]=v/(L*math.cos(delta)**2)
    B[3,0]=1.;B[4,1]=1.
    return A,B


def rk4_jacobians(z,u,h,vehicle):
    """Differentiate all four RK4 stages by the chain rule."""
    I=np.eye(5);F,G=continuous_jacobians(z,vehicle)
    k1=derivative(z,u,vehicle);Az1=F;Bu1=G
    z2=z+h*k1/2;F,G=continuous_jacobians(z2,vehicle)
    k2=derivative(z2,u,vehicle);Az2=F@(I+h*Az1/2);Bu2=F@(h*Bu1/2)+G
    z3=z+h*k2/2;F,G=continuous_jacobians(z3,vehicle)
    k3=derivative(z3,u,vehicle);Az3=F@(I+h*Az2/2);Bu3=F@(h*Bu2/2)+G
    z4=z+h*k3;F,G=continuous_jacobians(z4,vehicle)
    Az4=F@(I+h*Az3);Bu4=F@(h*Bu3)+G
    return I+h*(Az1+2*Az2+2*Az3+Az4)/6,h*(Bu1+2*Bu2+2*Bu3+Bu4)/6


def condensed(z0,reference,inputs,scene):
    N=scene['N'];M=np.zeros((5,2*N));base=np.array(z0,float)
    maps=[];bases=[];jacobians=[]
    for k in range(N):
        A,B=rk4_jacobians(reference[k],inputs[k],scene['h'],scene['vehicle'])
        # Nominal endpoint comes from dense RK4; the defect matches it exactly.
        defect=reference[k+1]-A@reference[k]-B@inputs[k]
        M=A@M;M[:,2*k:2*k+2]+=B;base=A@base+defect
        maps.append(M.copy());bases.append(base.copy());jacobians.append((A,B,defect))
    return np.array(bases),np.array(maps),jacobians


def body_support(heading,vehicle):
    body=corners([[0.,0.,heading,0.,0.]],vehicle)[0]
    radius=math.hypot(vehicle['L']+vehicle['front'],vehicle['width']/2)
    inflation=radius*STATE_TRUST[2]
    return np.r_[body.min(axis=0)-inflation,body.max(axis=0)+inflation]


def freeze_corridors(reference,permissions,scene,agents):
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
    b=agents['B']
    for k in range(scene['N']):
        if not b.ready:axis,left,right=0,'A','B'
        elif b.mode=='WAIT':axis,left,right=1,'B','A'
        else:axis,left,right=0,'B','A'
        l=corridors[left][k]['support'][2+axis];r=corridors[right][k]['support'][axis]
        pair.append(dict(axis=axis,left=left,right=right,upper=float(r-l-scene['s_min'])))
    return corridors,pair


def subproblem(agents,linear_reference,linear_inputs,target,target_inputs,desired,corridors,pair,parameters,scene,end):
    N=scene['N'];n=4*N;Hs=[];qs=[];maps={};bases={};rows=[];lower=[];upper=[];bounds=[]
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
    for k,p in enumerate(pair):
        values=np.zeros(n);axis=p['axis'];l=p['left'];r=p['right']
        values[CARS.index(l)*2*N:(CARS.index(l)+1)*2*N]=maps[l][k,axis]
        values[CARS.index(r)*2*N:(CARS.index(r)+1)*2*N]=-maps[r][k,axis]
        rows.append(values);lower.append(-np.inf);upper.append(p['upper']-bases[l][k,axis]+bases[r][k,axis])
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
            nominal=np.r_[target_inputs['A'].ravel(),target_inputs['B'].ravel()];solver.warm_start(x=nominal[free])
            result=solver.solve(raise_error=False);raw_wall=time.perf_counter()-started
            status=result.info.status;iterations=int(result.info.iter);ok=result.info.status_val in (1,2)
            if result.x is not None:u[free]=result.x
    residual=matrix@u;violation=max(0.,float(np.max(lo-residual)),float(np.max(residual-hi)))
    eig=float(np.linalg.eigvalsh(H)[0]);ok=bool(ok and np.isfinite(u).all() and violation<=scene['tolerance'] and eig>=-1e-10)
    predicted={c:np.vstack((agents[c].z,bases[c]+np.einsum('kij,j->ki',maps[c],u[i*2*N:(i+1)*2*N]))) for i,c in enumerate(CARS)}
    controls={c:u[i*2*N:(i+1)*2*N].reshape(N,2) for i,c in enumerate(CARS)}
    return controls,predicted,dict(accepted=ok,message=status,iterations=iterations,affine_violation=violation,
        hessian_min_eigenvalue=eig,variables=n,free_variables=len(free),affine_constraints=True,solver_wall_s=raw_wall,
        objective_without_constant=float(.5*u@H@u+q@u),solution=u.tolist(),solver_called=bool(raw_wall>0))


class BicycleController(Controller):
    mapping_tolerance=.0001
    future_deadline_s=30.
    reference_feasibility_band=NUMERICAL_BAND
    def __init__(self,scene,path,updates):
        super().__init__(scene,path);self.updates=updates

    def predict(self,permissions):
        started=time.perf_counter();end=started+DEADLINE_S
        reference_controls,reference_dense,ref_record=super().predict(permissions)
        if not ref_record['accepted']:
            return reference_controls,reference_dense,{**ref_record,'accepted':False,'message':'reference QP rejected','qp_updates':0,
                'bicycle_numeric_qp_calls':0,'reference_numeric_qp_calls':int(ref_record['free_variables']>0),
                'reference_record':ref_record,'subproblems':[],'deadline_exceeded':time.perf_counter()>end}
        N=self.scene['N'];n=round(self.scene['h']/self.scene['sample_s'])
        target={c:z[::n] for c,z in reference_dense.items()}
        target_inputs={c:np.array([[u[0],u[1]] for u in reference_controls[c]]) for c in CARS}
        desired={c:z.copy() for c,z in target.items()}
        times=np.arange(1,N+1)*self.scene['h'];P,V=response(times,self.scene['h'],N)
        for i,c in enumerate(CARS):
            agent=self.agents[c];p=ref_record['parameters'][c]
            if agent.mode=='MOVE':
                acceleration=np.array(ref_record['solution'][i*N:(i+1)*N])
                progress=p['s0']+times*p['v0']+P@acceleration
                desired[c][1:]=np.array([pose(agent.segment['start'],s,agent.segment['delta'],agent.segment['direction'],self.scene['vehicle']) for s in progress])
                desired[c][1:,3]=agent.segment['direction']*(p['v0']+V@acceleration)
        center={c:z.copy() for c,z in target.items()};inputs={c:u.copy() for c,u in target_inputs.items()}
        corridors,pair=freeze_corridors(target,permissions,self.scene,self.agents)
        records=[];controls=reference_controls;dense=reference_dense;errors={};linear=target;accepted=False
        for iteration in range(self.updates):
            if time.perf_counter()>=end:break
            proposed,linear,record=subproblem(self.agents,center,inputs,target,target_inputs,desired,corridors,pair,ref_record['parameters'],self.scene,end)
            records.append(record)
            if not record['accepted']:break
            controls={c:[(float(a),float(w),reference_controls[c][j][2]) for j,(a,w) in enumerate(proposed[c])] for c in CARS}
            dense={c:rollout(self.agents[c].z,controls[c],self.scene) for c in CARS}
            errors={c:dict(position_m=float(np.max(np.linalg.norm(linear[c][:,:2]-dense[c][::n,:2],axis=1))),
                heading_rad=float(np.max(np.abs(linear[c][:,2]-dense[c][::n,2]))),
                speed_m_s=float(np.max(np.abs(linear[c][:,3]-dense[c][::n,3]))),
                steering_rad=float(np.max(np.abs(linear[c][:,4]-dense[c][::n,4])))) for c in CARS}
            error_ok=all(e[k]<=v for e in errors.values() for k,v in ERROR_LIMIT.items())
            trust_ok=all(np.all(np.abs(dense[c][::n]-target[c])<=STATE_TRUST+self.scene['tolerance']) for c in CARS)
            safety=audit(dense,self.scene)
            record.update(mapping_errors=errors,nonlinear_safety_passed=safety['passed'],trust_replay_passed=bool(trust_ok))
            if error_ok and trust_ok and safety['passed']:
                accepted=True;break
            center={c:dense[c][::n].copy() for c in CARS};inputs=proposed
        elapsed=time.perf_counter()-started
        record=records[-1] if records else dict(affine_violation=0.,hessian_min_eigenvalue=0.,free_variables=0,solution=[])
        result={**record,'parameters':ref_record['parameters'],'accepted':bool(accepted and elapsed<=DEADLINE_S),
            'mapping_errors':errors or {c:dict(position_m=0.,heading_rad=0.,speed_m_s=0.,steering_rad=0.) for c in CARS},
            'solver':'OSQP bicycle affine QP','solve_wall_s':elapsed,'qp_updates':len(records),
            'bicycle_numeric_qp_calls':sum(r['solver_called'] for r in records),'reference_numeric_qp_calls':int(ref_record['free_variables']>0),
            'reference_record':ref_record,'subproblems':records,'deadline_s':DEADLINE_S,'deadline_exceeded':elapsed>DEADLINE_S,
            'linear_states':{c:z.tolist() for c,z in linear.items()},
            'corridors':{c:[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in p.items()} for p in ps] for c,ps in corridors.items()},
            'pair_separating_rows':pair}
        if elapsed>DEADLINE_S:result['message']='planning deadline exceeded'
        elif not accepted and records and records[-1]['accepted']:result['message']='nonlinear replay/trust rejection after update limit'
        return controls,dense,result

    def commit(self,controls,predictions,record):
        n=round(self.scene['h']/self.scene['sample_s']);h=self.scene['h']
        for c,a in self.agents.items():
            if a.mode=='MOVE':
                acceleration=a.segment['direction']*controls[c][0][0]
                a.s+=h*a.segment['direction']*a.z[3]+.5*h*h*acceleration
                a.previous=acceleration
                if a.stop_steps is not None:a.stop_steps-=1
            elif a.steering:a.steering.pop(0)
            a.z=predictions[c][n].copy()
