"""Finite-horizon numerical switching witnesses, never a feasibility theorem."""
import copy
import time
import numpy as np

from head_on_core import Settings, lane_y
from pass_release_core import CARS, hold_path_check, holding_bounds, neighbor_constraints, pair_check
from recovery_core import motion_cycle, propagate
from trajectory_acceptance import audit_controls


class TransitionGuard:
    def __init__(self, scene, options):
        self.scene=scene;self.options=options;self.records=[];self.solver_calls=0;self.wall_s=0.

    def verify_controls(self, checkpoint, controls, targets, hold_cars, role, reference, require_parked, mode):
        cfg=Settings(**self.scene['settings'])
        if any(np.shape(controls[c])!=(cfg.horizon,) or not np.all(np.isfinite(controls[c])) for c in CARS):
            return dict(passed=False,failed_constraints=['invalid_controls_shape_or_nonfinite'])
        ticks=np.arange(1,cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        paired=propagate(checkpoint,controls,self.scene,ticks);audits={};failed=[]
        for c,other in (('A','B'),('B','A')):
            kwargs,_=neighbor_constraints(checkpoint['states'][c][:2],paired[c][:,2],paired[other],cfg)
            audit=audit_controls(checkpoint['states'][c][:2],controls[c],targets[c],cfg,self.scene['road'],
                                 hold_bounds=holding_bounds(targets[c],self.scene) if c in hold_cars else None,**kwargs)
            audits[c]=audit
            failed.extend(c+':'+key for key,value in audit['residuals'].items() if value>cfg.tolerance)
        pair=pair_check(paired,controls,self.scene,cfg,role,np.array(reference),self.options,require_parked=require_parked)
        failed.extend(pair['reasons'])
        holds={c:hold_path_check(np.vstack([checkpoint['states'][c],paired[c]]),targets[c],self.scene) for c in hold_cars}
        failed.extend(c+':strict_hold_band' for c,h in holds.items() if not h['passed'])
        if mode=='RETURN_MERGE':
            m=paired[role]
            if np.max(np.abs(m[:,0]-reference[0]))>self.options['parked_position_tolerance_m'] or np.max(np.abs(m[:,1]))>self.options['parked_speed_tolerance_m_s']:
                failed.append('return_merge_longitudinal_hold')
        return dict(passed=bool(not failed),failed_constraints=failed,local=audits,pair=pair,holding=holds,
                    predicted_states={c:v.tolist() for c,v in paired.items()},
                    violations_by_car_and_physical_unit={c:r['residuals'] for c,r in audits.items()})

    def check(self, before_mode, after_mode, checkpoint, last_controls, targets, hold_cars, role, reference,
              *, require_parked=True, new_lanes=None, fresh_controls=None):
        started=time.perf_counter();cfg=Settings(**self.scene['settings']);cp=copy.deepcopy(checkpoint)
        if new_lanes is not None:cp['lanes']=copy.deepcopy(new_lanes)
        now=cp['absolute_time_s'];failed=[]
        for c in CARS:
            if abs(lane_y(cp['lanes'][c],[now])[0]-cp['states'][c][2])>1e-12:failed.append('lateral_state_jump')
        # Inspect the previous terminal state BEFORE permitting a zero-input tail.
        terminal={c:float(cp['states'][c][1]+cfg.dt*sum(last_controls[c][1:])) for c in CARS}
        rest=all(abs(v)<=1e-12 for v in terminal.values())
        shifted={c:np.r_[last_controls[c][1:],0.] for c in CARS} if rest else None
        shift=(self.verify_controls(cp,shifted,targets,hold_cars,role,reference,require_parked,after_mode)
               if shifted is not None and not failed else dict(passed=False,failed_constraints=failed or ['old_terminal_rest_not_established']))
        shift.update(old_terminal_velocity_m_s=terminal,zero_tail_appended=rest,
                     exact_infinite_stationarity_certified=False,rest_numeric_tolerance_m_s=1e-12,
                     zero_input_interval_s=cfg.dt,terminal_velocity_residual_m_s={c:abs(v) for c,v in terminal.items()},
                     controls={c:u.tolist() for c,u in shifted.items()} if shifted is not None else None)
        witness='shifted_zero_tail' if shift['passed'] else None;fresh=None;new_calls=0
        if not witness and not failed:
            if fresh_controls is not None:
                fresh=self.verify_controls(cp,{c:np.asarray(u) for c,u in fresh_controls.items()},targets,hold_cars,role,reference,require_parked,after_mode)
                witness='existing_hold_precheck' if fresh['passed'] else None
            else:
                plan=motion_cycle(cp,self.scene,self.options,targets,stage=after_mode,cycle=-1,role=role,
                                  reference=reference,hold_cars=hold_cars,require_parked=require_parked)
                new_calls=len(plan['solves']);self.solver_calls+=new_calls
                if plan['passed']:
                    fresh=self.verify_controls(cp,plan['controls'],targets,hold_cars,role,reference,require_parked,after_mode)
                    fresh['controls']={c:u.tolist() for c,u in plan['controls'].items()}
                else:fresh=dict(passed=False,failed_constraints=[plan['failure']])
                fresh['solves']=plan['solves'];witness='fresh_joint_plan' if fresh['passed'] else None
        elapsed=time.perf_counter()-started;self.wall_s+=elapsed
        result=dict(from_mode=before_mode,to_mode=after_mode,absolute_time_s=now,actual_checkpoint=checkpoint,
                    allowed=witness is not None,witness=witness,shifted_candidate=shift,fresh_candidate=fresh,
                    compute_wall_s=elapsed,additional_solver_calls=new_calls,
                    initialized_from_actual_state=True,joint_peer_consistency_checked=True,
                    applied_as_backup=False,not_a_recursive_feasibility_proof=True)
        self.records.append(result)
        return result
