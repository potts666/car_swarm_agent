"""Mode/reference-aware public candidates; not a recursive-feasibility certificate."""
import copy
import hashlib
import time
import numpy as np
from bicycle_core import rollout
from m2b_control import reference_qp
from m2b_protocol import context_plan,check_plan,canonical
from m2a_logs import plan_id


class CandidateRejected(ValueError):
    def __init__(self,message,record):
        super().__init__(message);self.record=record


def contract(agent,parameters,permission):
    return dict(mode=agent.mode,index=agent.index,stop_active=parameters['stop_at_end'],
                stop_index=parameters['stop_index'],cap=parameters['cap'],upper=parameters['upper'],authorized=bool(permission))


def compatible_reference(old,new):
    if old is None:return False
    if any(old[k]!=new[k] for k in ['mode','index','stop_active','cap','upper','authorized']):return False
    if new['stop_active'] and new['mode']=='MOVE':return new['stop_index']==max(1,old['stop_index']-1)
    return True


def phase_residuals(agent,commands,parameters,scene):
    u=np.array([q[:2] for q in commands],float);N=scene['N'];h=scene['h']
    if u.shape!=(N,2) or not np.isfinite(u).all():raise ValueError('missing/nonfinite candidate inputs')
    if agent.mode!='MOVE':
        expected=agent.steering[:N]+[(0.,0.,agent.mode)]*max(0,N-len(agent.steering[:N]))
        fixed=np.array([q[:2] for q in expected],float)
        return dict(fixed_acceleration_m_s2=float(abs(u[:,0]-fixed[:,0]).max()),fixed_steering_rate_rad_s=float(abs(u[:,1]-fixed[:,1]).max()))
    velocities=agent.z[3]+h*np.r_[0.,np.cumsum(u[:,0])]
    progress=parameters['s0']+np.r_[0.,np.cumsum(h*velocities[:-1]+.5*h*h*u[:,0])]
    residuals=dict(progress_m=float(max(0.,-progress.min(),(progress-parameters['upper']).max())),
                   forward_speed_m_s=float(max(0.,-velocities.min(),(velocities-parameters['cap']).max())))
    k=parameters['stop_index']
    residuals['terminal_speed_m_s']=abs(float(velocities[k]))
    if parameters['stop_at_end']:residuals['terminal_progress_m']=abs(float(progress[k]-parameters['upper']))
    return residuals


def validate_start(plan,agent,parameters,context,identity,scene,cache=None):
    expected=contract(agent,parameters,context['owner']==identity.ids[agent.car])
    if plan.get('applicability')!=expected:raise ValueError('candidate mode/reference identity mismatch')
    if plan['context']!=context or plan['report']['car']!=identity.ids[agent.car]:raise ValueError('candidate state/context mismatch')
    z=check_plan(identity.plan_to_role(plan),identity.report_to_role(plan['report']),identity.context(context,False),scene,cache)
    residuals=phase_residuals(agent,plan['commands'],parameters,scene)
    passed=all(v<=scene['tolerance'] for v in residuals.values())
    return dict(physical_permission_passed=True,partial_phase_reference_passed=passed,residuals=residuals,
                full_ocp_checked=False,scope='physical/permission + current fixed inputs or partial progress/speed/reference terminal rows')


def build_start(agent,parameters,report,context,identity,scene,last,old_contract,cache=None):
    started=time.perf_counter();car=identity.ids[agent.car];new_contract=contract(agent,parameters,context['owner']==car)
    N=scene['N'];reason='initialization' if last is None else 'mode_or_reference_change';old_rejection=None;record=None;repairs=[]
    if last is not None and compatible_reference(old_contract,new_contract):
        commands=last['commands'][1:]+[[0.,0.,'SHIFT_TAIL']]
        n=round(scene['h']/scene['sample_s'])
        if np.max(abs(np.asarray(last['states'])[n]-agent.z))>1e-12:raise ValueError('committed shift does not match actual state')
        plan=context_plan(report,commands,rollout(agent.z,commands,scene),'seed','committed_shift',last['id'])
        plan['applicability']=new_contract;plan['id']=plan_id(plan)
        check=validate_start(plan,agent,parameters,context,identity,scene,cache)
        if check['partial_phase_reference_passed']:
            return plan,dict(source='shift',old_shift_applicable=True,validation=check,reference_record=None,build_wall_s=time.perf_counter()-started)
        old_rejection=check;reason='shift_partial_rows_not_applicable'
    if agent.mode!='MOVE':
        commands=agent.steering[:N]+[(0.,0.,agent.mode)]*max(0,N-len(agent.steering[:N]))
        source='own_fixed_queue'
    else:
        accel,record=reference_qp({car:agent},{car:context['owner']==car},scene,None,started+.5)
        if not record['accepted']:raise CandidateRejected('candidate reference rejected: '+record['message'],record)
        commands=[(float(a),0.,agent.segment['stage']) for a in accel[car]];source='own_reference'
        if parameters['stop_at_end'] and agent.z[3]+scene['h']*commands[0][0]<0.:
            acceleration=-agent.z[3]/scene['h'];repairs.append(dict(raw_acceleration=commands[0][0],applied_acceleration=float(acceleration)))
            commands[0]=(float(acceleration),commands[0][1],commands[0][2])
        if repairs:
            detail=record['_details'];x=np.array([q[0] for q in commands]);value=detail['A']@x
            violation=max(0.,float(np.max(detail['lower']-value)),float(np.max(value-detail['upper'])))
            if violation>scene['tolerance']:raise ValueError('repaired candidate reference violates original affine tolerance')
    plan=context_plan(report,commands,rollout(agent.z,commands,scene),'seed',source,last['id'] if last else None)
    plan['applicability']=new_contract;plan['id']=plan_id(plan)
    check=validate_start(plan,agent,parameters,context,identity,scene,cache)
    if not check['partial_phase_reference_passed']:raise ValueError('new candidate partial stage/reference rejection '+canonical(check['residuals']))
    elapsed=time.perf_counter()-started
    if elapsed>.5:raise TimeoutError('candidate preparation exceeds local .5s computation budget')
    return plan,dict(source=source,reason=reason,old_shift_applicable=False,old_shift_rejection=old_rejection,
                     validation=check,reference_record=record,input_repairs=repairs,raw_reference_optimum_after_repair=False if repairs else None,build_wall_s=elapsed)
