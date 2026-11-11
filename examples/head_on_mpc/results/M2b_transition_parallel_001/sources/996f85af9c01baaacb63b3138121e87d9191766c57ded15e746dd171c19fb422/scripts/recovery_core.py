"""Chen-inspired diagnostics adapted to the existing longitudinal experiment.

No MBVC-WB variable, safe-corridor construction or deadlock theorem is claimed.
"""
import copy
from collections import deque
import time

import numpy as np

from head_on_core import Settings, integrate, lane_y, matrices, prediction, receive
from pass_release_core import (CARS, continuity, holding_bounds, hold_path_check,
                               neighbor_constraints, pair_check, snapshot)
from trajectory_acceptance import audit_controls, solve_qp


class BlockingDetector:
    def __init__(self, scene, settings):
        self.scene = scene
        self.settings = settings
        self.history = deque()
        self.accumulated = 0.

    def observe(self, now, states, predicted, accepted, mode='NAVIGATE', blocked=None):
        # Intentional role waiting is supervised by phase timeouts instead.
        if mode != 'NAVIGATE':
            self.history.clear()
            self.accumulated = 0.
            return dict(status='intentional_mode', triggered=False, mode=mode)
        if not accepted or predicted is None:
            self.history.clear()
            self.accumulated = 0.
            return dict(status='solver_failure', triggered=False, reason='no_current_accepted_prediction')
        cfg = Settings(**self.scene['settings'])
        endpoints = {c: float(predicted[c][-1, 0]) for c in CARS}
        tails = {c: abs(float(predicted[c][-1, 0]-predicted[c][-1-cfg.substeps, 0])) for c in CARS}
        unfinished = {c: not (abs(states[c][0]-self.scene['goals_x'][c]) <= self.scene['arrival_position_tolerance_m'] and
                             abs(states[c][1]) <= self.scene['arrival_speed_tolerance_m_s']) for c in CARS}
        entry = dict(time=now, positions={c: float(states[c][0]) for c in CARS},
                     endpoints=endpoints, tails=tails, blocked=blocked or dict.fromkeys(CARS, False))
        if self.history:
            old = self.history[-1]
            stable = all(unfinished.values()) and all(
                abs(entry['positions'][c]-old['positions'][c]) <= self.settings['actual_progress_m'] and
                abs(endpoints[c]-old['endpoints'][c]) <= self.settings['prediction_endpoint_change_m'] and
                tails[c] <= self.settings['prediction_tail_displacement_m'] for c in CARS)
            self.accumulated = self.accumulated+(now-old['time']) if stable else 0.
        self.history.append(entry)
        window = self.settings['window_s']
        while len(self.history)>1 and self.history[1]['time'] <= now-window+1e-8:
            self.history.popleft()
        elapsed = now-self.history[0]['time']
        progress = {c: float(np.ptp([e['positions'][c] for e in self.history])) for c in CARS}
        endpoint_change = {c: float(np.ptp([e['endpoints'][c] for e in self.history])) for c in CARS}
        tail_max = {c: max(e['tails'][c] for e in self.history) for c in CARS}
        blocked_fraction = {c: float(np.mean([e['blocked'][c] for e in self.history])) for c in CARS}
        stationary = (elapsed >= window-1e-8 and all(unfinished.values()) and
                      all(progress[c] <= self.settings['actual_progress_m'] and
                          endpoint_change[c] <= self.settings['prediction_endpoint_change_m'] and
                          tail_max[c] <= self.settings['prediction_tail_displacement_m'] for c in CARS))
        evidence = all(blocked_fraction[c] >= self.settings['minimum_blocked_fraction'] for c in CARS)
        return dict(status='mutual_blocking' if stationary and evidence else 'suspected_stall' if stationary else 'observing',
                    triggered=bool(stationary and evidence), absolute_time_s=now, mode=mode,
                    window_observed_s=elapsed, accumulated_stall_s=self.accumulated,
                    unfinished=unfinished, actual_position_span_m=progress, endpoint_span_m=endpoint_change,
                    maximum_prediction_tail_displacement_m=tail_max, blocked_fraction=blocked_fraction,
                    blocking_metric='longitudinal hard-neighbor slack near activation AND goal points toward peer; not paper w_ij',
                    thresholds=self.settings, strict_deadlock_proof=False)


def propagate(checkpoint, controls, scene, times):
    cfg = Settings(**scene['settings'])
    pmat, vmat = matrices(times, cfg)
    return {c: np.column_stack((checkpoint['states'][c][0]+times*checkpoint['states'][c][1]+pmat@controls[c],
                               checkpoint['states'][c][1]+vmat@controls[c],
                               lane_y(checkpoint['lanes'][c], checkpoint['absolute_time_s']+times))) for c in CARS}


def motion_cycle(checkpoint, scene, options, targets, *, stage, cycle, role='A', reference=None,
                 hold_cars=(), require_parked=False):
    """Same solver, model and layered checks; only stage targets/hold sets vary."""
    cfg = Settings(**scene['settings'])
    now = checkpoint['absolute_time_s']
    states = {c: np.array(checkpoint['states'][c][:2], float) for c in CARS}
    lanes = checkpoint['lanes']
    warm = {c: np.array(checkpoint['warm_controls'][c], float) for c in CARS}
    ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
    predicted, controls, records, masks = {}, {}, [], {}
    # Normal planning is decentralized previous-prediction exchange. RETURN
    # solves the holding car first, then the returning car against that plan.
    if 'packets' in checkpoint:
        peers = {c: receive(checkpoint['packets'][c], now+ticks) for c in CARS}
    else:
        peers = {c: receive(prediction(states[c], warm[c], lanes[c], now, cfg), now+ticks) for c in CARS}
    order = CARS if stage=='NAVIGATE' else (('B','A') if role=='A' else ('A','B'))
    for c in order:
        other = 'B' if c=='A' else 'A'
        peer = peers[other] if stage=='NAVIGATE' or other not in predicted else predicted[other]
        kwargs, mask = neighbor_constraints(states[c], lane_y(lanes[c], now+ticks), peer, cfg)
        bounds = holding_bounds(targets[c], scene) if c in hold_cars else None
        u, report = solve_qp(states[c], checkpoint['previous_acceleration'][c], warm[c], now,
                             targets[c], cfg, scene['road'], hold_bounds=bounds, **kwargs)
        audit = audit_controls(states[c], u, targets[c], cfg, scene['road'], hold_bounds=bounds, **kwargs)
        report.update(cycle=cycle, car=c, stage=stage, role=role, accepted=audit['passed'],
                      independent_acceptance=audit, controls=u.tolist(), neighbor_overlap_mask=mask.tolist())
        records.append(report)
        if not audit['passed']:
            return dict(passed=False, failure='solver_failure', solves=records, predictions=None, checks=[], next=None)
        controls[c] = u
        predicted[c] = propagate(checkpoint, {**controls, other: controls.get(other, warm[other])}, scene, ticks)[c]
        masks[c] = mask
    ref = np.array(reference if reference is not None else checkpoint['states'][role], float)
    check = pair_check(predicted, controls, scene, cfg, role, ref, options, require_parked=require_parked)
    check.update(kind='whole_prediction', cycle=cycle, stage=stage, absolute_time_s=now)
    for c in hold_cars:
        hold = hold_path_check(np.vstack([checkpoint['states'][c], predicted[c]]), targets[c], scene)
        check.setdefault('holding', {})[c] = hold
        check['passed'] &= hold['passed']
    if not check['passed']:
        return dict(passed=False, failure='paired_prediction_failed', solves=records, predictions=predicted, checks=[check], next=None)
    relative = np.arange(cfg.substeps+1)*cfg.dt/cfg.substeps
    motions = {c: np.column_stack((integrate(states[c], controls[c][0], relative), lane_y(lanes[c], now+relative))) for c in CARS}
    executed = pair_check(motions, {c:[u[0]] for c,u in controls.items()}, scene, cfg, role, ref, options, require_parked=require_parked)
    executed.update(kind='executed_segment', cycle=cycle, stage=stage, absolute_time_s=now)
    for c in hold_cars:
        h = hold_path_check(motions[c], targets[c], scene)
        executed.setdefault('holding', {})[c] = h
        executed['passed'] &= h['passed']
    executed['applied_to_simulation'] = bool(executed['passed'])
    if not executed['passed']:
        return dict(passed=False, failure='actual_path_failed', solves=records, predictions=predicted, checks=[check,executed], next=None)
    new_states = {c: motions[c][-1,:2] for c in CARS}
    previous = {c:float(u[0]) for c,u in controls.items()}
    shifted = {c:np.r_[u[1:],u[-1]] for c,u in controls.items()}
    result = snapshot(new_states, lanes, now+cfg.dt, previous, shifted)
    result['packets'] = {c:prediction(states[c], controls[c], lanes[c], now, cfg) for c in CARS}
    result['accepted_controls'] = {c:u.tolist() for c,u in controls.items()}
    return dict(passed=True, failure=None, solves=records, predictions=predicted, controls=controls,
                checks=[check,executed], motions=motions, next=result, masks=masks)


def append_cycle(plan, checkpoint, scene, role, passer, stage, rows, records, paths, predictions, execute=True):
    cfg = Settings(**scene['settings']); now = checkpoint['absolute_time_s']
    records.extend(plan['solves']); paths.extend(plan['checks'])
    if plan['predictions'] is not None:
        ticks = np.arange(1,cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        predictions.append(dict(stage=stage, absolute_time_s=now, times=(now+ticks).tolist(), role=role,
                                states={c:p.tolist() for c,p in plan['predictions'].items()}))
    if plan['passed'] and execute:
        for c,motion in plan['motions'].items():
            for j,s in enumerate(motion):
                rows.append(dict(absolute_time_s=round(now+j*cfg.dt/cfg.substeps,10), car=c,
                                 x=float(s[0]), vx=float(s[1]), y=float(s[2]),
                                 acceleration=float(plan['controls'][c][0]), stage=stage,
                                 exit_subphase='', role=role, passer=passer, right_holder=''))


def return_to_original_tasks(checkpoint, last_controls, scene, options, role, reference, cycle,
                             rows, records, paths, predictions, events, transition_guard=None):
    """Release-to-return continuation; the passing car stays at its original goal."""
    checkpoint = copy.deepcopy(checkpoint)
    checkpoint['accepted_controls'] = copy.deepcopy(last_controls)
    cfg = Settings(**scene['settings']); start=checkpoint['absolute_time_s']
    passer='B' if role=='A' else 'A'; stage='RETURN_MERGE'; failure=None
    before=copy.deepcopy(checkpoint)
    y=checkpoint['states'][role][2]
    new_lanes=copy.deepcopy(checkpoint['lanes'])
    new_lanes[role]=[y,0.,start,cfg.lateral_seconds]
    if transition_guard is not None:
        switch=transition_guard.check('RELEASED','RETURN_MERGE',checkpoint,last_controls,
            {role:float(reference[0]),passer:scene['goals_x'][passer]},(passer,),role,reference.tolist(),
            require_parked=False,new_lanes=new_lanes)
        events.append(dict(kind='switch_check',**switch))
        if not switch['allowed']:failure='switch_guard_rejected'
    if failure is None:checkpoint['lanes']=new_lanes
    # A new lateral schedule starts at the inherited actual y; no state jump.
    event=dict(kind='stage',to=stage,absolute_time_s=start,states=copy.deepcopy(checkpoint['states']),
               inherited=before,original_goals=scene['goals_x'])
    if failure is None:events.append(event)
    hold_target=float(reference[0]); compute=0.; calls=0
    phase_limit=options.get('return_timeout_s',100.)
    while failure is None:
        now=checkpoint['absolute_time_s']
        if now-start+cfg.dt>phase_limit+1e-9 or now-cfg.epoch+cfg.dt>options['total_timeout_s']+1e-9:
            failure='return_timeout'; break
        if stage=='RETURN_MERGE' and now>=start+cfg.lateral_seconds-1e-8:
            if transition_guard is not None:
                switch=transition_guard.check(stage,'RETURN',checkpoint,checkpoint['accepted_controls'],
                    scene['goals_x'],(passer,),role,reference.tolist(),require_parked=False)
                events.append(dict(kind='switch_check',**switch))
                if not switch['allowed']:failure='switch_guard_rejected';break
            stage='RETURN'
            events.append(dict(kind='stage',to=stage,absolute_time_s=now,states=copy.deepcopy(checkpoint['states'])))
        targets=dict(scene['goals_x'])
        if stage=='RETURN_MERGE':targets[role]=hold_target
        began=time.perf_counter()
        plan=motion_cycle(checkpoint,scene,options,targets,stage=stage,cycle=cycle,role=role,
                          reference=reference,hold_cars=(passer,))
        compute+=time.perf_counter()-began; calls+=len(plan['solves'])
        if stage=='RETURN_MERGE' and plan['passed']:
            predicted=plan['predictions'][role];actual=plan['motions'][role]
            hold_ok=all(float(np.max(np.abs(m[:,0]-hold_target)))<=options['parked_position_tolerance_m'] and
                        float(np.max(np.abs(m[:,1])))<=options['parked_speed_tolerance_m_s'] for m in (predicted,actual))
            if not hold_ok:
                plan['passed']=False;plan['failure']='return_merge_hold_failed'
                plan['checks'][-1]['applied_to_simulation']=False
        append_cycle(plan,checkpoint,scene,role,passer,stage,rows,records,paths,predictions)
        if not plan['passed']:
            failure=plan['failure']; break
        checkpoint=plan['next'];cycle+=1
        if stage=='RETURN_MERGE':
            actual=checkpoint['states'][role]
            if abs(actual[0]-hold_target)>options['parked_position_tolerance_m'] or abs(actual[1])>options['parked_speed_tolerance_m_s']:
                failure='return_merge_hold_failed';break
        if stage=='RETURN' and all(abs(checkpoint['states'][c][0]-scene['goals_x'][c])<=scene['arrival_position_tolerance_m'] and
                                  abs(checkpoint['states'][c][1])<=scene['arrival_speed_tolerance_m_s'] and
                                  abs(checkpoint['states'][c][2])<=cfg.tolerance for c in CARS):break
    now=checkpoint['absolute_time_s'];completed=failure is None
    final_stage='BOTH_TASKS_COMPLETED' if completed else 'FAILED'
    events.append(dict(kind='both_tasks_completed' if completed else 'failure',to=final_stage,
                       absolute_time_s=now,states=checkpoint['states'],failure=failure,
                       both_tasks_completed=completed,simulation_terminated=not completed))
    for c,s in checkpoint['states'].items():
        rows.append(dict(absolute_time_s=round(now,10),car=c,x=s[0],vx=s[1],y=s[2],
                         acceleration=checkpoint['previous_acceleration'][c],stage=final_stage,
                         exit_subphase='',role=role,passer=passer,right_holder=''))
    return dict(status=final_stage, failure=failure, failure_phase=None if completed else stage,
                both_tasks_completed=completed, simulation_terminated=not completed, return_simulation_s=now-start,
                initial_to_completion_simulation_s=(now-cfg.epoch) if completed else None,
                final_actual_states=checkpoint['states'],final_checkpoint=checkpoint,
                return_compute_wall_s=compute,return_primary_calls=calls)
