"""Analysis only: original geometry/integrator reused, never invoke a controller."""
import itertools
import math
import numpy as np
from bicycle_core import corners,gaps,outside_area,clipped_area,rollout,steer

CARS=('A1','A2','B1','B2')
UNITS=('position_m','heading_rad','speed_m_s','steering_rad')


def cumulative_progress(report,route):
    segments=route['segments'];index=report['index']
    if index<0 or index>len(segments):raise ValueError('invalid route segment index')
    return float(segments[index]['offset']+report['progress_m']) if index<len(segments) else float(sum(q['length'] for q in segments))


def wrapped(angle):return np.arctan2(np.sin(angle),np.cos(angle))


def differences(a,b,ds):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=5:raise ValueError('unaligned state arrays')
    values=(np.linalg.norm(a[:,:2]-b[:,:2],axis=1),abs(wrapped(a[:,2]-b[:,2])),abs(a[:,3]-b[:,3]),abs(a[:,4]-b[:,4]))
    maxima={k:float(v.max()) for k,v in zip(UNITS,values)}
    indices={k:int(v.argmax()) for k,v in zip(UNITS,values)}
    return dict(maximum=maxima,argmax_index=indices,argmax_lead_s={k:i*ds for k,i in indices.items()}),dict(zip(UNITS,values))


def aligned_difference(old,new,h,ds):
    n=round(h/ds)
    if n<1 or abs(n*ds-h)>1e-10:raise ValueError('control/sample periods not aligned')
    a=np.asarray(old)[n:];b=np.asarray(new)[:len(a)]
    return differences(a,b,ds)


def command_violations(commands,scene):
    u=np.array([q[:2] for q in commands],float);p=scene['vehicle']
    return dict(acceleration_m_s2=float(max(0.,(p['a_min']-u[:,0]).max(),(u[:,0]-p['a_max']).max())),
                steering_rate_rad_s=float(max(0.,(abs(u[:,1])-p['omega_max']).max())))


def layer_safety(plans,context,scene):
    if not plans:return dict(complete=False,missing=list(CARS),cars={},pairs={},gap_margin_m=None,physical_protocol_passed=None)
    bodies={c:corners(np.asarray(p['states'],float),scene['vehicle']) for c,p in plans.items()}
    p=scene['vehicle'];tol=scene['tolerance'];ds=scene['sample_s'];car_rows={}
    for c,plan in plans.items():
        z=np.asarray(plan['states'],float)
        if not np.isfinite(z).all():raise ValueError('nonfinite prediction')
        body=bodies[c];road=outside_area(body,scene);sign=1 if c[0]=='A' else -1
        fields={'road_area_m2':road,'forward_speed_m_s':np.maximum(0.,z[:,3]-p['v_fwd']),
                'reverse_speed_m_s':np.maximum(0.,-z[:,3]-p['v_rev']),
                'steering_rad':np.maximum(0.,abs(z[:,4])-p['delta_max'])}
        intrusive=np.zeros(len(z))
        if context['owner']!=c:
            lo=body.min(axis=1);hi=body.max(axis=1)
            # Exact disjoint bounding boxes imply zero polygon intersection, not a geometric relaxation.
            relevant=np.flatnonzero((hi[:,0]>=-18)&(lo[:,0]<=18)&(hi[:,1]>=-1.4)&(lo[:,1]<=1.4))
            for i in relevant:intrusive[i]=clipped_area(body[i],[-18,18,-1.4,1.4])
        fields['unauthorized_area_m2']=intrusive
        if c in context.get('released',[]):fields['released_boundary_m']=np.maximum(0.,26.5-(sign*body[:,:,0]).min(axis=1))
        worst={k:dict(value=float(v.max()),index=int(v.argmax()),lead_s=float(v.argmax()*ds),violating_samples=int(np.sum(v>tol))) for k,v in fields.items()}
        commands=command_violations(plan['commands'],scene)
        car_rows[c]=dict(violations=worst,input_violations=commands,
                         passed=all(v['value']<=tol for v in worst.values()) and all(v<=tol for v in commands.values()))
    pairs={}
    for a,b in itertools.combinations(plans,2):
        values=gaps(bodies[a],bodies[b]);i=int(values.argmin())
        pairs[a+':'+b]=dict(clearance_m=float(values[i]),margin_m=float(values[i]-scene['s_min']),index=i,lead_s=i*ds,
                            violating_samples=int(np.sum(values<scene['s_min']-tol)))
    complete=set(plans)==set(CARS)
    margin=min((v['margin_m'] for v in pairs.values()),default=None)
    return dict(complete=complete,missing=sorted(set(CARS)-set(plans)),cars=car_rows,pairs=pairs,gap_margin_m=margin,
                physical_protocol_passed=bool(all(r['passed'] for r in car_rows.values()) and (margin is None or margin>=-tol)) if complete else None)


def stopping_distance(v,h,brake):
    speed=abs(float(v));q=math.floor(speed/(brake*h));r=speed-q*brake*h
    return q*h*speed-.5*brake*h*h*q*q+.5*h*r


def phase_diagnostic(plan,route,scene,parameters,fixed_inputs=None):
    r=plan['report'];mode=r['mode'];z=np.asarray(plan['states']);u=np.array([q[:2] for q in plan['commands']],float);h=scene['h'];n=round(h/scene['sample_s']);tol=scene['tolerance']
    result=dict(mode=mode,index=r['index'],ocp_feasibility_checked=False)
    segments=route['segments']
    if mode=='MOVE':
        segment=segments[r['index']];direction=segment['direction'];vel=direction*z[::n,3]
        progress=float(r['progress_m'])+np.r_[0.,np.cumsum(direction*(h*z[::n,3][:-1]+.5*h*h*u[:,0]))]
        margin=float(segment['length']-r['progress_m'])
        brake=-scene['vehicle']['a_min'] if r['z'][3]>=0 else scene['vehicle']['a_max']
        result.update(remaining_m=margin,one_step_stop_margin_m=margin-.5*h*abs(r['z'][3]),
                      discrete_stop_distance_m=stopping_distance(r['z'][3],h,brake),
                      discrete_stop_margin_m=margin-stopping_distance(r['z'][3],h,brake),
                      progress_violation_m=float(max(0.,-progress.min(),(progress-segment['length']).max())),
                      phase_speed_violation_m_s=float(max(0.,-vel.min(),(vel-segment['speed_cap']).max())))
        if parameters:
            k=parameters['stop_index'];result['reference_terminal_speed_m_s']=abs(float(z[k*n,3]))
            if parameters['stop_at_end']:result['reference_terminal_progress_m']=abs(float(progress[k]-parameters['upper']))
        checked=[result['progress_violation_m'],result['phase_speed_violation_m_s']]
        result['checked_phase_rows_passed']=all(v<=tol for v in checked)
        # Different units remain separate; terminal equations are partial diagnostics, not rebuilt QP feasibility.
        result['reference_terminal_rows_passed']=all(v<=tol for k,v in result.items() if k.startswith('reference_terminal_')) if parameters else None
    else:
        result['fixed_acceleration_error_m_s2']=float(abs(u[:,0]).max())
        if fixed_inputs is not None:
            expected=np.array([q[:2] for q in fixed_inputs],float)
            result['fixed_steering_rate_error_rad_s']=float(abs(u[:,1]-expected[:,1]).max())
            result['checked_phase_rows_passed']=all(result[k]<=tol for k in ['fixed_acceleration_error_m_s2','fixed_steering_rate_error_rad_s'])
            result['fixed_input_source']='accepted current final; internal steering queue not independently reconstructed'
        else:result['checked_phase_rows_passed']=None
    return result


def terminal_zero_check(plan,scene):
    z=np.asarray(plan['states'])[-1];tail=rollout(z,[(0.,0.,'DIAGNOSTIC_TAIL')],scene)
    return dict(terminal_speed_m_s=abs(float(z[3])),zero_tail_displacement_m=float(np.linalg.norm(tail[-1,:2]-z[:2])),
                checked_extension_s=scene['h'],invariant_terminal_set_defined=False)
