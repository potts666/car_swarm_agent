#!/usr/bin/env python3
"""Replay cycle 15 A unchanged; label constraints and solve Phase I separately."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent / 'reverse_parking_mpc'
sys.path.insert(0, str(BASE))
import mpc


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def verify_baseline():
    frozen = json.loads((BASE / 'frozen_baseline.json').read_text())
    for name, digest in frozen['core_sha256'].items():
        if hashlib.sha256((BASE / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Frozen baseline changed: ' + name)
    return frozen


def extract():
    frozen = verify_baseline()
    source = BASE / 'two_baseline/two_20261010.jsonl'
    rows = [json.loads(line) for line in source.read_text().splitlines()]
    row = next(row for row in rows if row['cycle'] == 15 and row['car'] == 0)
    previous_b = next(row for row in rows if row['cycle'] == 14 and row['car'] == 1)
    config = json.loads((BASE / 'config.json').read_text())
    previous_controls = np.array(previous_b['controls'])
    previous = mpc.rollout(previous_b['state'], np.vstack((previous_controls, [0., previous_controls[-1, 1]])), config)
    steps = config['time']['model_substeps']
    if not np.array_equal(previous[steps + 1:], row['peer_input']):
        raise RuntimeError('Peer prediction reconstruction differs from logged problem')
    return dict(cycle=row['cycle'],time_s=row['time_s'],state=row['state'],
                peer_input=row['peer_input'],peer_state_at_start=previous[steps].tolist(),
                reference_states=row['solver']['reference_states'],
                reference_controls=row['solver']['reference_controls'],gear=row['solver']['gear'],
                warm_input=row['warm_input'],config=config,
                logged_output=row['controls'],logged_report=row['solver'],
                source_log_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                baseline_core_sha256=frozen['core_sha256'],
                interpretation='Exact cycle 15 (16th round) local optimization; no map, peer, target, constraint or original seed changes.')


def reference_container(problem):
    # Only these window entries are read by the unmodified baseline solve().
    config = problem['config']
    steps = config['time']['model_substeps']
    cycle = problem['cycle']
    target = np.array(problem['reference_states'])
    states = np.zeros((cycle * steps + len(target) + 1, 5))
    states[cycle * steps + 1:] = target
    controls = np.vstack((np.zeros((cycle, 2)), problem['reference_controls']))
    gear = np.r_[np.zeros(cycle, dtype=int), problem['gear']]
    return dict(states=states, controls=controls, gear=gear)


def groups(problem, controls):
    config = problem['config']
    state = np.array(problem['state'])
    future = mpc.rollout(state, controls, config)[1:]
    limits = config['limits']
    steps = config['time']['model_substeps']
    dt = config['time']['control_dt_s']
    lateral = future[:, 3] ** 2 * np.tan(future[:, 4]) / config['vehicle']['wheelbase_m']
    gear = np.repeat(problem['gear'], steps)
    moving = gear != 0
    return dict(speed_lower=future[:, 3] - limits['speed_m_s'][0],
                speed_upper=limits['speed_m_s'][1] - future[:, 3],
                steering_rate=limits['steering_rate_rad_s'] * dt - abs(np.diff(np.r_[state[4], controls[:, 1]])),
                total_acceleration=limits['total_acceleration_m_s2'] - np.hypot(np.repeat(controls[:, 0], steps), lateral),
                static_body_gap=mpc.static_gaps(future[:, :2], future[:, 2], np.array(config['obstacle_boxes'])) - limits['static_sat_gap_m'],
                reference_direction=future[moving, 3] * gear[moving],
                predicted_peer_body_gap=mpc.pair_gap(future, np.array(problem['peer_input'])) - limits['pair_sat_gap_m'])


def evaluate(problem, controls):
    controls = np.asarray(controls)
    config = problem['config']
    limits = config['limits']
    labelled = groups(problem, controls)
    residual, equalities = mpc.residuals(np.array(problem['state']), controls, config,
                                       np.array(problem['gear']), np.array(problem['peer_input']))
    # This diagnostic is a decomposition, not a new implementation of safety.
    np.testing.assert_allclose(np.concatenate(list(labelled.values())), residual, atol=1e-12, rtol=0)
    bound_acc = limits['acceleration_m_s2'] - abs(controls[:, 0])
    bound_steer = limits['steering_rad'] - abs(controls[:, 1])
    labelled.update(acceleration_bounds=bound_acc, steering_bounds=bound_steer)
    details = {}
    for name, values in labelled.items():
        index = int(np.argmin(values))
        details[name] = dict(min_residual=float(values[index]),worst_index=index,
                             maximum_violation=max(0., -float(values[index])),values=values.tolist())
        if name not in ['reference_direction']:
            increment = config['time']['control_dt_s'] if name in ['steering_rate','acceleration_bounds','steering_bounds'] else config['time']['control_dt_s']/config['time']['model_substeps']
            details[name]['worst_time_s'] = problem['time_s'] + (index+1)*increment
    maximum = max([max(0.,-float(np.min(v))) for v in labelled.values()] +
                  [float(np.max(abs(equalities))) if len(equalities) else 0.])
    prediction = mpc.rollout(problem['state'], controls, config)
    return dict(feasible_same_local_constraints=maximum <= config['mpc']['constraint_tolerance'],
                maximum_violation=maximum,constraints=details,stop_equalities=equalities.tolist(),
                min_sampled_peer_sat_gap_m=float(np.min(mpc.pair_gap(prediction[1:],np.array(problem['peer_input'])))),
                controls=controls.tolist(),prediction=prediction.tolist(),
                sample_dt_s=config['time']['control_dt_s']/config['time']['model_substeps'],
                continuous_time_certified=False)


def braking(problem, magnitude, straighten=False):
    state = np.array(problem['state'])
    controls = []
    config = problem['config']
    dt = config['time']['control_dt_s']
    steering = state[4]
    for _ in range(config['mpc']['horizon_steps']):
        # Integrate bounded acceleration; do not replace speed by zero.
        acceleration = -np.sign(state[3]) * min(magnitude, abs(state[3])/dt)
        if straighten and abs(state[3]) < 1e-10:
            steering -= np.sign(steering)*min(abs(steering),config['limits']['steering_rate_rad_s']*dt)
        control = np.array([acceleration,steering])
        controls.append(control)
        state = mpc.rollout(state,[control],config)[-1]
    return np.array(controls)


def dense_check(problem, controls):
    config = problem['config']
    state = np.array(problem['state'])
    states = [state.copy()]
    per_interval = []
    for control in controls:
        motion = mpc.rollout(state,[control],config,config['time']['validation_substeps'])
        # Restart each dense interval at the same coarse-model actual state.
        per_interval.extend(motion[1:])
        state = mpc.rollout(state,[control],config)[-1]
    future = np.array(per_interval)
    dt = config['time']['control_dt_s']/config['time']['validation_substeps']
    times = dt*np.arange(1,len(future)+1)
    peer = np.vstack((problem['peer_state_at_start'],problem['peer_input']))
    sample_times = config['time']['control_dt_s']/config['time']['model_substeps']*np.arange(len(peer))
    interpolated_peer = np.column_stack([np.interp(times,sample_times,np.unwrap(peer[:,j]) if j==2 else peer[:,j]) for j in range(5)])
    gaps = mpc.pair_gap(future,interpolated_peer)
    static = mpc.static_gaps(future[:,:2],future[:,2],np.array(config['obstacle_boxes']))
    distances = mpc.body_distance(future[:,:2],future[:,2],interpolated_peer[:,:2],interpolated_peer[:,2])
    return dict(min_sampled_peer_sat_gap_m=float(np.min(gaps)),
                min_sampled_peer_euclidean_distance_m=float(np.min(distances)),
                min_sampled_static_sat_gap_m=float(np.min(static)),sample_dt_s=dt,
                peer_interpolation='linear positions / unwrapped yaw between the FIXED local 0.1 s peer samples',
                continuous_time_certified=False)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=ROOT/'results')
    parser.add_argument('--snapshot',type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    problem=json.loads(args.snapshot.read_text()) if args.snapshot else extract()
    current=verify_baseline()
    if current['core_sha256'] != problem['baseline_core_sha256']:
        raise RuntimeError('Snapshot uses a different frozen baseline')
    write(args.output/'problem.json',problem)
    ref=reference_container(problem)
    calls=[]
    original_minimize=mpc.minimize
    def capture(fun,x0,**kwargs):
        started=time.perf_counter()
        result=original_minimize(fun,x0,**kwargs)
        calls.append(dict(fun=fun,x0=np.array(x0),kwargs=kwargs,result=result,
                          seconds=time.perf_counter()-started))
        return result
    mpc.minimize=capture
    try:
        controls, report=mpc.solve(np.array(problem['state']),problem['cycle'],ref,problem['config'],
                                  np.array(problem['warm_input']),np.array(problem['peer_input']))
    finally:
        mpc.minimize=original_minimize
    write(args.output/'original_replay.json',dict(report=report,
          output_max_difference_from_logged=float(np.max(abs(controls-np.array(problem['logged_output'])))),
          exact_reference_window_match=bool(np.array_equal(mpc.window(ref,problem['cycle'],problem['config'])[0],problem['reference_states'])),
          attempts=[dict(seed=call['x0'].reshape(-1,2).tolist(),result=evaluate(problem,call['result'].x.reshape(-1,2)),
                         message=str(call['result'].message),success=bool(call['result'].success),
                         objective=float(call['result'].fun)) for call in calls]))
    original=calls[0]
    objective=original['fun']
    constraints=original['kwargs']['constraints']
    bounds=original['kwargs']['bounds']
    inequality=constraints[0]['fun']
    initializations={'signed_reference':np.array(problem['reference_controls']),
                     'shifted_solution':np.array(problem['warm_input']),
                     'max_brake_hold_steer':braking(problem,1),
                     'gentle_brake_hold_steer':braking(problem,.5),
                     'max_brake_then_straighten':braking(problem,1,True)}
    candidates={}
    for name,u in initializations.items():
        result=evaluate(problem,u)
        result.update(objective=float(objective(u.ravel())),dense_collision_check=dense_check(problem,u))
        candidates[name]=result
    write(args.output/'candidates.json',candidates)
    phase=[]
    optimized=[]
    for name,initial in initializations.items():
        z=initial.ravel()
        slack=max(0.,-float(np.min(inequality(z))))
        phase_constraints=[{'type':'ineq','fun':lambda zs:inequality(zs[:-1])+zs[-1]}]
        for constraint in constraints[1:]:
            if constraint['type']=='eq':
                phase_constraints.append({'type':'eq','fun':lambda zs,fun=constraint['fun']:fun(zs[:-1])})
        started=time.perf_counter()
        result=minimize(lambda zs:float(zs[-1]),np.r_[z,slack],method='SLSQP',
                        bounds=bounds+[(0.,None)],constraints=phase_constraints,
                        options={'maxiter':200,'ftol':1e-9,'eps':problem['config']['mpc']['finite_difference_step']})
        checked=evaluate(problem,result.x[:-1].reshape(-1,2))
        phase.append(dict(initialization=name,optimizer_success=bool(result.success),message=str(result.message),
                          iterations=int(result.nit),slack=float(result.x[-1]),solve_seconds=time.perf_counter()-started,
                          validation=checked))
        if checked['feasible_same_local_constraints']:
            started=time.perf_counter()
            refined=minimize(objective,result.x[:-1],**{**original['kwargs'],'constraints':constraints})
            validation=evaluate(problem,refined.x.reshape(-1,2))
            optimized.append(dict(initialization=name,optimizer_success=bool(refined.success),message=str(refined.message),
                                  iterations=int(refined.nit),objective=float(refined.fun),
                                  accepted=bool(refined.success and validation['feasible_same_local_constraints']),
                                  solve_seconds=time.perf_counter()-started,validation=validation,
                                  dense_collision_check=dense_check(problem,refined.x.reshape(-1,2))))
    write(args.output/'phase_one.json',phase)
    write(args.output/'objective_from_feasible.json',optimized)
    found=any(v['feasible_same_local_constraints'] for v in candidates.values()) or any(v['validation']['feasible_same_local_constraints'] for v in phase)
    gear=np.array(problem['gear'])
    stop=gear==0
    stop[:-1] |= gear[1:]==0
    stop[:-1] |= gear[:-1]*gear[1:]<0
    audit=dict(reference_direction=gear.tolist(),stop_equation_endpoints_s=(problem['time_s']+(np.flatnonzero(stop)+1)*problem['config']['time']['control_dt_s']).tolist(),
               waiting_with_zero_speed_allowed_in_reverse_mode=True,
               physical_hard_constraints=['bicycle dynamics','signed speed bounds','acceleration / steering bounds','steering rate','total acceleration','static / fixed-prediction body clearance'],
               policy_hard_constraints=['reference-dependent direction inequality','reference-dependent fixed stop equalities'],
               soft_task_progress=['timed pose / signed-speed tracking','reference control tracking','terminal POSITION COST (no terminal-goal hard constraint in this local problem)'],
               caveat='Future fixed stop/direction schedules reduce admissible controls in general. At THIS snapshot, no stop equality exists; waiting is not forbidden by reverse direction since v=0 is allowed.')
    write(args.output/'constraint_policy_audit.json',audit)
    summary=dict(local_sampled_feasibility_confirmed=found,
                 original_replay_accepted=report['accepted'],
                 objective_from_feasible_accepted=sum(v['accepted'] for v in optimized),
                 feasible_raw_candidates=[name for name,value in candidates.items() if value['feasible_same_local_constraints']],
                 original_constraints_unchanged=True,continuous_time_certified=False,
                 interpretation='A legal candidate refutes local sampled infeasibility; rejected optimization is a numerical/initialization outcome, not a scene or local impossibility proof.' if found else 'Local feasibility unconfirmed; no impossibility claim.')
    write(args.output/'summary.json',summary)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
