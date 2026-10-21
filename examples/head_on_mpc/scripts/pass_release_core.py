"""Continue an actual EXIT checkpoint through PASS and conservative release.

Offline branch simulation only. No vehicle return and no two-task completion.
"""
import copy
import csv
import json
from pathlib import Path
import time

import numpy as np

from head_on_core import (Settings, body_gap, inside_road, integrate, lane_y,
                          matrices, prediction, receive, write_json)
from trajectory_acceptance import audit_controls, road_residual, solve_qp

CARS = ('A', 'B')


def json_finite(value):
    """Preserve rejection evidence as JSON null for nonfinite numeric fields."""
    if isinstance(value, dict):
        return {k: json_finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_finite(v) for v in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value


def geometry(scene, cfg, options):
    channel = scene['channel']
    road = scene['road']
    region = dict(xmin=channel['xmin'], xmax=channel['xmax'], ymin=road['ymin'], ymax=road['ymax'])
    targets = dict(scene['goals_x'])
    result = dict(channel=channel, conflict_region=region, release_margin_m=options['release_margin_m'],
                  exit_boundaries={'A_positive_x': region['xmax'], 'B_negative_x': region['xmin']},
                  passing_stop_targets_x=targets, arrival_position_tolerance_m=scene['arrival_position_tolerance_m'],
                  arrival_speed_tolerance_m_s=scene['arrival_speed_tolerance_m_s'],
                  road_unchanged=road)
    for car in CARS:
        target = np.array([[targets[car], 0., 0.]])
        if not np.all(inside_road(target, road, cfg)) or not body_cleared(target[0], car, region, cfg, options):
            raise ValueError(f'geometry_insufficient:{car}: no valid existing passing stop target')
    return result


def body_cleared(state, car, region, cfg, options):
    # No tolerance allows early release at the geometric boundary.
    if car == 'A':
        return bool(state[0]-cfg.length/2 >= region['xmax']+options['release_margin_m'])
    return bool(state[0]+cfg.length/2 <= region['xmin']-options['release_margin_m'])


def bay_contains(state, role, scene, cfg):
    target = scene['retreat_x'][role]
    matching = [b for b in scene['road']['bays'] if b['xmin'] <= target <= b['xmax']]
    x, _, y = state
    return any(x-cfg.length/2 >= b['xmin']-1e-8 and x+cfg.length/2 <= b['xmax']+1e-8 and
               y-cfg.width/2 >= scene['road']['ymin']-1e-8 and y+cfg.width/2 <= b['ymax']+1e-8
               for b in matching) and bool(inside_road(np.array([state]), scene['road'], cfg)[0])


def parked_check(motion, reference, role, scene, cfg, options):
    finite = bool(np.all(np.isfinite(motion)))
    if not finite:
        return {'passed': False, 'finite': False, 'residuals': {}}
    residuals = dict(position_from_actual_reference_m=float(np.max(np.abs(motion[:, 0]-reference[0]))),
                     speed_m_s=float(np.max(np.abs(motion[:, 1]))),
                     speed_from_actual_reference_m_s=float(np.max(np.abs(motion[:, 1]-reference[1]))),
                     lateral_from_actual_reference_m=float(np.max(np.abs(motion[:, 2]-reference[2]))),
                     # y=3 parked above main road. Keep the whole body outside main strip.
                     main_strip_intrusion_m=float(max(0., scene['road']['ymax']-(np.min(motion[:, 2])-cfg.width/2))))
    passed = (residuals['position_from_actual_reference_m'] <= options['parked_position_tolerance_m'] and
              residuals['speed_m_s'] <= options['parked_speed_tolerance_m_s'] and
              residuals['speed_from_actual_reference_m_s'] <= options['parked_speed_tolerance_m_s'] and
              residuals['lateral_from_actual_reference_m'] <= options['parked_lateral_tolerance_m'] and
              residuals['main_strip_intrusion_m'] == 0. and
              all(bay_contains(s, role, scene, cfg) for s in motion))
    return {'passed': bool(passed), 'finite': finite, 'residuals': residuals}


def pair_check(motions, controls, scene, cfg, role, reference, options):
    finite = all(np.all(np.isfinite(m)) for m in motions.values())
    if not finite:
        return {'passed': False, 'finite': False, 'reasons': ['nonfinite'], 'residuals': {}, 'parked': {'passed': False}}
    gap = float(np.min(body_gap(motions['A'], motions['B'], cfg)))
    residuals = dict(road_body_m=max(road_residual(m, scene['road'], cfg) for m in motions.values()),
                     neighbor_gap_m=max(0., cfg.margin-gap),
                     speed_m_s=max(float(max(0., np.max(np.abs(m[:, 1]))-cfg.max_speed)) for m in motions.values()),
                     acceleration_m_s2=max(float(max(0., np.max(np.abs(u))-cfg.max_acceleration)) for u in controls.values()))
    reasons = []
    if not all(np.all(inside_road(m, scene['road'], cfg)) for m in motions.values()):
        reasons.append('body_road')
    for key in ('neighbor_gap_m', 'speed_m_s', 'acceleration_m_s2'):
        if residuals[key] > cfg.tolerance:
            reasons.append(key)
    parked = parked_check(motions[role], reference, role, scene, cfg, options)
    if not parked['passed']:
        reasons.append('parked_hold')
    return dict(passed=not reasons, finite=True, reasons=reasons, residuals=residuals,
                minimum_gap_m=gap, parked=parked)


def snapshot(states, lanes, now, previous, warm):
    return dict(absolute_time_s=float(now), states={c: [*map(float, s), float(lane_y(lanes[c], [now])[0])] for c, s in states.items()},
                previous_acceleration={c: float(a) for c, a in previous.items()},
                warm_controls={c: u.tolist() for c, u in warm.items()}, lanes=copy.deepcopy(lanes))


def continuity(before, after):
    residuals = dict(time_s=abs(after['absolute_time_s']-before['absolute_time_s']),
                     position_m=max(abs(after['states'][c][0]-before['states'][c][0]) for c in CARS),
                     velocity_m_s=max(abs(after['states'][c][1]-before['states'][c][1]) for c in CARS),
                     lateral_m=max(abs(after['states'][c][2]-before['states'][c][2]) for c in CARS),
                     previous_acceleration_m_s2=max(abs(after['previous_acceleration'][c]-before['previous_acceleration'][c]) for c in CARS),
                     warm_control_m_s2=max(float(np.max(np.abs(np.array(after['warm_controls'][c])-before['warm_controls'][c]))) for c in CARS),
                     lateral_schedule_endpoints_m=max(float(np.max(np.abs(np.array(after['lanes'][c][:2])-before['lanes'][c][:2]))) for c in CARS),
                     lateral_schedule_stamp_s=max(abs(after['lanes'][c][2]-before['lanes'][c][2]) for c in CARS),
                     lateral_schedule_duration_s=max(abs(after['lanes'][c][3]-before['lanes'][c][3]) for c in CARS))
    return dict(passed=all(v <= 1e-12 for v in residuals.values()), residuals=residuals)


def load_checkpoint(exit_directory, scene):
    """Reconstruct only from the executed log and last applied/solved controls."""
    directory = Path(exit_directory)
    summary = json.loads((directory/'summary.json').read_text())
    with (directory/'actual.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    solves = json.loads((directory/'solves.json').read_text())
    last = {c: next(r for r in reversed(rows) if r['car'] == c) for c in CARS}
    last_solves = {c: next(r for r in reversed(solves) if r['car'] == c) for c in CARS}
    now = float(last['A']['absolute_time_s'])
    assert abs(now-float(last['B']['absolute_time_s'])) < 1e-9
    states = {c: np.array([float(last[c]['x']), float(last[c]['vx'])]) for c in CARS}
    previous = {c: float(last[c]['acceleration']) for c in CARS}
    warm = {c: np.r_[last_solves[c]['controls'][1:], last_solves[c]['controls'][-1]] for c in CARS}
    cfg = Settings(**scene['settings'])
    # Reconstruct the original completed lateral schedule from the first
    # actually executed enter_bay sample, not an idealized terminal lane reset.
    role = summary['role']
    entered = next(float(r['absolute_time_s']) for r in rows if r['car'] == role and r['phase'] == 'enter_bay')
    lanes = {c: [0., scene['bay_y'], entered, cfg.lateral_seconds] if c == role else
                [0., 0., cfg.epoch, cfg.lateral_seconds] for c in CARS}
    for c in CARS:
        assert abs(lane_y(lanes[c], [now])[0]-float(last[c]['y'])) <= 1e-12
    return summary, states, lanes, now, previous, warm, rows, solves


def ready_check(exit_summary, checkpoint, scene, cfg, role, options):
    waiter = 'B' if role == 'A' else 'A'
    s = checkpoint['states']
    end_lane = checkpoint['lanes'][role][2]+checkpoint['lanes'][role][3]
    checks = dict(full_exit=exit_summary['feasible'], lateral_finished=checkpoint['absolute_time_s'] >= end_lane-1e-8,
                  retreat_target_position=abs(s[role][0]-scene['retreat_x'][role]) <= scene['arrival_position_tolerance_m'],
                  retreat_speed=abs(s[role][1]) <= scene['arrival_speed_tolerance_m_s'],
                  retreat_lateral=abs(s[role][2]-scene['bay_y']) <= cfg.tolerance,
                  bay_contains=bay_contains(s[role], role, scene, cfg),
                  waiter_position=abs(s[waiter][0]-scene['initial_states_x_v'][waiter][0]) <= .01,
                  waiter_speed=abs(s[waiter][1]) <= .01,
                  waiter_lateral=abs(s[waiter][2]) <= cfg.tolerance,
                  body_road=all(inside_road(np.array([v]), scene['road'], cfg)[0] for v in s.values()),
                  pair_gap=float(body_gap(np.array([s['A']]), np.array([s['B']]), cfg)[0]) >= cfg.margin-cfg.tolerance)
    return dict(passed=bool(all(checks.values())), checks={k: bool(v) for k, v in checks.items()})


def release_check(actual, passer, holder, target, region, scene, cfg, role, reference, options,
                  latest_prediction_passed, no_failure, hold_seconds, latest_prediction_stays_clear=False):
    pair = pair_check({c: np.array([s]) for c, s in actual.items()}, {'A': [0.], 'B': [0.]}, scene, cfg, role, reference, options)
    checks = dict(actual_body_cleared=body_cleared(actual[passer], passer, region, cfg, options),
                  actual_at_target=abs(actual[passer][0]-target) <= scene['arrival_position_tolerance_m'],
                  actual_low_speed=abs(actual[passer][1]) <= scene['arrival_speed_tolerance_m_s'],
                  held_long_enough=hold_seconds+1e-9 >= options['cleared_hold_s'],
                  holder_matches=holder == passer, latest_prediction=latest_prediction_passed,
                  predicted_stays_clear=latest_prediction_stays_clear,
                  no_failure=no_failure, current_pair=pair['passed'], parked_hold=pair['parked']['passed'])
    return dict(passed=bool(all(checks.values())), checks={k: bool(v) for k, v in checks.items()}, pair_check=pair)


def neighbor_constraints(state, own_y, peer, cfg):
    front = peer[0, 0] >= state[0]
    mask = np.abs(own_y-peer[:, 2]) < cfg.width+cfg.margin
    neighbor = peer.copy()
    neighbor[~mask, 0] = 1e4 if front else -1e4
    return {('front' if front else 'rear'): neighbor}, mask


def holding_bounds(target, scene):
    return dict(speed_max_m_s=scene['arrival_speed_tolerance_m_s'],
                xmin=target-scene['arrival_position_tolerance_m'], xmax=target+scene['arrival_position_tolerance_m'])


def hold_path_check(motion, target, scene):
    error = float(np.max(np.abs(motion[:, 0]-target)))
    speed = float(np.max(np.abs(motion[:, 1])))
    return dict(passed=bool(error <= scene['arrival_position_tolerance_m'] and speed <= scene['arrival_speed_tolerance_m_s']),
                target_position_error_m=error, speed_m_s=speed,
                position_tolerance_m=scene['arrival_position_tolerance_m'], speed_limit_m_s=scene['arrival_speed_tolerance_m_s'])


def precheck_hold(states, lanes, now, previous, warm, packets, role, passer, target, reference, scene, cfg, options, region, cycle):
    """No execution or warm mutation. Check a full paired constrained horizon."""
    ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
    pmat, vmat = matrices(ticks, cfg)
    predicted, controls, records = {}, {}, []
    bounds = holding_bounds(target, scene)
    before = snapshot(states, lanes, now, previous, warm)
    for c in (role, passer):
        peer = receive(packets[passer], now+ticks) if c == role else predicted[role]
        kwargs, mask = neighbor_constraints(states[c], lane_y(lanes[c], now+ticks), peer, cfg)
        local_target = reference[0] if c == role else target
        hb = bounds if c == passer else None
        u, report = solve_qp(states[c], previous[c], warm[c], now, local_target, cfg, scene['road'], hold_bounds=hb, **kwargs)
        audit = audit_controls(states[c], u, local_target, cfg, scene['road'], hold_bounds=hb, **kwargs)
        report.update(cycle=cycle, stage='PASS', car=c, role=role, diagnostic=True, purpose='hold_mode_precheck',
                      accepted=audit['passed'], independent_acceptance=audit, controls=u.tolist(),
                      neighbor_overlap_samples=int(np.sum(mask)), neighbor_overlap_mask=mask.tolist())
        records.append(report)
        if not audit['passed']:
            return dict(passed=False, reason='independent_acceptance', records=records, stop_candidate=None,
                        before=before, after=snapshot(states, lanes, now, previous, warm))
        controls[c] = u
        predicted[c] = np.column_stack((states[c][0]+ticks*states[c][1]+pmat@u,
                                       states[c][1]+vmat@u, lane_y(lanes[c], now+ticks)))
    pair = pair_check(predicted, controls, scene, cfg, role, reference, options)
    hold = hold_path_check(np.vstack([[*states[passer], lane_y(lanes[passer], [now])[0]], predicted[passer]]), target, scene)
    clear = all(body_cleared(s, passer, region, cfg, options) for s in predicted[passer])
    # Independent explicit stop candidate: one ZOH interval to zero, then zero.
    stop_u = np.zeros(cfg.horizon)
    stop_u[0] = -states[passer][1]/cfg.dt
    peer_kwargs, _ = neighbor_constraints(states[passer], lane_y(lanes[passer], now+ticks), predicted[role], cfg)
    stop_audit = audit_controls(states[passer], stop_u, target, cfg, scene['road'], hold_bounds=bounds, **peer_kwargs)
    stop_predicted = {role: predicted[role], passer: np.column_stack((states[passer][0]+ticks*states[passer][1]+pmat@stop_u,
                      states[passer][1]+vmat@stop_u, lane_y(lanes[passer], now+ticks)))}
    stop_pair = pair_check(stop_predicted, {role: controls[role], passer: stop_u}, scene, cfg, role, reference, options)
    stop_hold = hold_path_check(np.vstack([[*states[passer], 0.], stop_predicted[passer]]), target, scene)
    stop_clear = all(body_cleared(s, passer, region, cfg, options) for s in stop_predicted[passer])
    stop = dict(passed=bool(stop_audit['passed'] and stop_pair['passed'] and stop_hold['passed'] and stop_clear),
                controls=stop_u.tolist(), first_acceleration_m_s2=float(stop_u[0]),
                first_interval_displacement_m=float(states[passer][1]*cfg.dt/2),
                independent_acceptance=stop_audit, pair=stop_pair, hold=stop_hold, stays_clear=stop_clear,
                applied=False, note='diagnostic candidate only; never a fallback control')
    return dict(passed=bool(pair['passed'] and hold['passed'] and clear), reason=None if pair['passed'] and hold['passed'] and clear else 'paired_or_hold_path',
                pair=pair, hold=hold, stays_clear=clear, records=records, stop_candidate=stop,
                before=before, after=snapshot(states, lanes, now, previous, warm))


def run_branch(directory, exit_directory, scene, policy, options, selection_wait, hold_constraints=False):
    """Role is fixed before this call. No access to the other role's PASS."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    branch_started = time.perf_counter()
    cfg = Settings(**scene['settings'])
    geo = geometry(scene, cfg, options)
    (exit_summary, states, lanes, now, previous, warm, exit_rows, exit_solves) = load_checkpoint(exit_directory, scene)
    role = exit_summary['role']
    passer = 'B' if role == 'A' else 'A'
    start = now
    before = snapshot(states, lanes, now, previous, warm)
    # This snapshot is the source of the actual parked reference, including v.
    reference = np.array(before['states'][role], float)
    gate = ready_check(exit_summary, before, scene, cfg, role, options)
    stage = 'PARKED_READY'
    failure, failure_phase = None, None
    holder = None
    grant = pass_start = first_clear = stop_candidate = stop_confirm = release_time = None
    cleared_since = None
    complete = False
    records = [{**r, 'stage': 'EXIT'} for r in exit_solves]
    paths = [{**r, 'stage': 'EXIT'} for r in json.loads((Path(exit_directory)/'path_checks.json').read_text())]
    predictions = [{**r, 'stage': 'EXIT', 'role': role,
                    'lateral_occupancy': {c: [s[2] for s in r['states'][c]] for c in CARS}}
                   for r in (json.loads(line) for line in (Path(exit_directory)/'predictions.jsonl').read_text().splitlines())]
    rows = []
    for r in exit_rows:
        rows.append(dict(absolute_time_s=float(r['absolute_time_s']), car=r['car'], x=float(r['x']), vx=float(r['vx']),
                         y=float(r['y']), acceleration=float(r['acceleration']), stage='EXIT', exit_subphase=r['phase'],
                         role=role, passer=passer, right_holder=''))
    events = [dict(kind='stage', to='EXIT', absolute_time_s=cfg.epoch, role=role),
              dict(kind='stage', to='PARKED_READY', absolute_time_s=now, checkpoint=before, ready_check=gate)]
    current = lambda: {c: [*map(float, states[c]), float(lane_y(lanes[c], [now])[0])] for c in CARS}
    after = snapshot(states, lanes, now, previous, warm)
    continuity_check = continuity(before, after)
    if not gate['passed'] or not continuity_check['passed']:
        failure, failure_phase = 'parked_ready_or_continuity_failed', 'PARKED_READY'
    if not failure:
        stage, holder, grant, pass_start = 'PASS', passer, now, now
        events.append(dict(kind='grant', to='PASS', absolute_time_s=now, holder=holder, states=current(), ready_check=gate,
                           continuity=continuity_check, before=before, after=after))
    for boundary_stage in ('PARKED_READY', stage):
        for c, value in current().items():
            rows.append(dict(absolute_time_s=round(now, 10), car=c, x=value[0], vx=value[1], y=value[2],
                             acceleration=previous[c], stage=boundary_stage, exit_subphase='', role=role,
                             passer=passer, right_holder=holder or ''))
    packets = {c: prediction(states[c], warm[c], lanes[c], now, cfg) for c in CARS}
    next_cycle = max(r['cycle'] for r in records)+1
    pass_records_start = len(records)
    pass_paths_start = len(paths)
    stage_computation = {'PASS': 0., 'CLEARED_HOLD': 0.}
    stage_calls = {'PASS': 0, 'CLEARED_HOLD': 0}
    passed_prediction = False
    hold_prechecks = []
    diagnostic_calls = 0
    attempt_started = time.perf_counter()
    while not failure and not complete:
        cycle = next_cycle
        next_cycle += 1
        if now-cfg.epoch+cfg.dt > options['total_timeout_s']+1e-9:
            failure, failure_phase = 'total_timeout', stage
            break
        if now-start+cfg.dt > options['pass_timeout_s']+1e-9:
            failure, failure_phase = 'pass_stage_timeout', stage
            break
        cycle_started = time.perf_counter()
        cycle_stage = stage
        ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        pmat, vmat = matrices(ticks, cfg)
        peers = {c: receive(packets[c], now+ticks) for c in CARS}
        predicted, controls = {}, {}
        cycle_ok = True
        for c in (role, passer):
            peer = peers[passer] if c == role else predicted[role]
            kwargs, mask = neighbor_constraints(states[c], lane_y(lanes[c], now+ticks), peer, cfg)
            target = reference[0] if c == role else geo['passing_stop_targets_x'][c]
            hb = holding_bounds(target, scene) if hold_constraints and stage == 'CLEARED_HOLD' and c == passer else None
            u, report = solve_qp(states[c], previous[c], warm[c], now, target, cfg, scene['road'], hold_bounds=hb, **kwargs)
            audit = audit_controls(states[c], u, target, cfg, scene['road'], hold_bounds=hb, **kwargs)
            report.update(cycle=cycle, car=c, stage=stage, role=role, target=target, controls=u.tolist(),
                          accepted=audit['passed'], independent_acceptance=audit,
                          neighbor_overlap_samples=int(np.sum(mask)), neighbor_overlap_mask=mask.tolist())
            records.append(report)
            stage_calls[stage] += 1
            controls[c] = u
            if not audit['passed']:
                cycle_ok = False
                failure, failure_phase = 'primary_independent_acceptance_failed', stage
                break
            predicted[c] = np.column_stack((states[c][0]+ticks*states[c][1]+pmat@u,
                                            states[c][1]+vmat@u, lane_y(lanes[c], now+ticks)))
        if not cycle_ok:
            stage_computation[cycle_stage] += time.perf_counter()-cycle_started
            break
        check = pair_check(predicted, controls, scene, cfg, role, reference, options)
        if hold_constraints and stage == 'CLEARED_HOLD':
            check['passing_hold'] = hold_path_check(np.vstack([[*states[passer], 0.], predicted[passer]]), geo['passing_stop_targets_x'][passer], scene)
            if not check['passing_hold']['passed']:
                check['passed'] = False
                check['reasons'].append('passing_hold_prediction')
        check.update(cycle=cycle, stage=stage, kind='whole_prediction', absolute_time_s=now)
        paths.append(check)
        predictions.append(dict(cycle=cycle, stage=stage, absolute_time_s=now, role=role, holder=holder,
                                times=(now+ticks).tolist(), states={c: p.tolist() for c, p in predicted.items()},
                                lateral_occupancy={c: lane_y(lanes[c], now+ticks).tolist() for c in CARS}))
        passed_prediction = check['passed']
        if not check['passed']:
            failure = 'parked_hold_failed' if not check['parked']['passed'] else 'paired_prediction_failed'
            failure_phase = stage
            stage_computation[cycle_stage] += time.perf_counter()-cycle_started
            break
        relative = np.arange(cfg.substeps+1)*cfg.dt/cfg.substeps
        motions = {c: np.column_stack((integrate(states[c], controls[c][0], relative), lane_y(lanes[c], now+relative))) for c in CARS}
        check = pair_check(motions, {c: [u[0]] for c, u in controls.items()}, scene, cfg, role, reference, options)
        # Independently check the exact dynamics for every executed substep.
        dynamic = {}
        step = cfg.dt/cfg.substeps
        for c, motion in motions.items():
            dx = motion[1:, 0]-(motion[:-1, 0]+step*motion[:-1, 1]+.5*step**2*controls[c][0])
            dv = motion[1:, 1]-(motion[:-1, 1]+step*controls[c][0])
            dynamic[c] = dict(position_m=float(np.max(np.abs(dx))), velocity_m_s=float(np.max(np.abs(dv))))
        check['dynamics_residuals'] = dynamic
        if any(v > cfg.tolerance for r in dynamic.values() for v in r.values()):
            check['passed'] = False
            check['reasons'].append('executed_dynamics')
        if stage == 'CLEARED_HOLD':
            passer_motion = motions[passer]
            signed_clearance = (passer_motion[:, 0]-cfg.length/2-geo['conflict_region']['xmax']-options['release_margin_m']
                                if passer == 'A' else geo['conflict_region']['xmin']-options['release_margin_m']-passer_motion[:, 0]-cfg.length/2)
            check['cleared_hold_residuals'] = dict(
                target_position_error_m=float(np.max(np.abs(passer_motion[:, 0]-geo['passing_stop_targets_x'][passer]))),
                actual_segment_speed_m_s=float(np.max(np.abs(passer_motion[:, 1]))),
                clearance_deficit_m=float(max(0., -np.min(signed_clearance))))
            check['cleared_hold_all_samples'] = bool(all(
                body_cleared(s, passer, geo['conflict_region'], cfg, options) and
                abs(s[0]-geo['passing_stop_targets_x'][passer]) <= scene['arrival_position_tolerance_m'] and
                abs(s[1]) <= scene['arrival_speed_tolerance_m_s'] for s in motions[passer]))
            if not check['cleared_hold_all_samples']:
                check['passed'] = False
                check['reasons'].append('cleared_hold_lost')
        check['applied_to_simulation'] = check['passed']
        check.update(cycle=cycle, stage=stage, kind='executed_segment', absolute_time_s=now)
        paths.append(check)
        # Audit before advancing the offline model. Unchecked controls are never applied.
        if not check['passed']:
            failure = ('parked_hold_failed' if not check['parked']['passed'] else
                       'cleared_hold_actual_condition_lost' if 'cleared_hold_lost' in check['reasons'] else 'actual_path_failed')
            failure_phase = stage
            stage_computation[cycle_stage] += time.perf_counter()-cycle_started
            break
        outgoing = {c: prediction(states[c], controls[c], lanes[c], now, cfg) for c in CARS}
        for c, motion in motions.items():
            for j, sample in enumerate(motion):
                rows.append(dict(absolute_time_s=round(now+relative[j], 10), car=c, x=float(sample[0]), vx=float(sample[1]),
                                 y=float(sample[2]), acceleration=float(controls[c][0]), stage=stage, exit_subphase='',
                                 role=role, passer=passer, right_holder=holder))
        for j, sample in enumerate(motions[passer]):
            if first_clear is None and body_cleared(sample, passer, geo['conflict_region'], cfg, options):
                first_clear = float(now+relative[j])
                events.append(dict(kind='first_actual_body_cleared', absolute_time_s=first_clear,
                                   states={c: motions[c][j].tolist() for c in CARS}, holder=holder))
        for c in CARS:
            states[c] = motions[c][-1, :2]
            previous[c] = float(controls[c][0])
            warm[c] = np.r_[controls[c][1:], controls[c][-1]]
        now += cfg.dt
        packets = outgoing
        actual = current()
        release_gate = release_check(actual, passer, holder, geo['passing_stop_targets_x'][passer], geo['conflict_region'],
                                     scene, cfg, role, reference, options, passed_prediction, not failure,
                                     0. if cleared_since is None else now-cleared_since,
                                     all(body_cleared(s, passer, geo['conflict_region'], cfg, options) for s in predicted[passer]))
        stop_ok = all(v for k, v in release_gate['checks'].items() if k != 'held_long_enough')
        if stop_ok:
            if cleared_since is None:
                hold_entry_ok = True
                if hold_constraints:
                    precheck = precheck_hold(states, lanes, now, previous, warm, packets, role, passer,
                        geo['passing_stop_targets_x'][passer], reference, scene, cfg, options, geo['conflict_region'], cycle)
                    hold_prechecks.append(dict(absolute_time_s=now, passer=passer, **precheck))
                    records.extend(precheck['records'])
                    diagnostic_calls += len(precheck['records'])
                    hold_entry_ok = precheck['passed'] and precheck['before'] == precheck['after']
                    events.append(dict(kind='hold_mode_precheck', absolute_time_s=now, passed=hold_entry_ok,
                                       precheck_index=len(hold_prechecks)-1, states=actual))
                if hold_entry_ok:
                    cleared_since, stop_candidate, stage = now, now, 'CLEARED_HOLD'
                    events.append(dict(kind='stage', to=stage, absolute_time_s=now, states=actual, check=release_gate))
                else:
                    failure, failure_phase = 'hold_mode_precheck_failed', 'PASS'
            elif release_gate['passed']:
                stop_confirm, release_time, complete, stage = now, now, True, 'RELEASED'
                events.append(dict(kind='release', absolute_time_s=now, holder=holder, states=actual,
                                   check=release_gate, scope=geo['conflict_region'], both_tasks_completed=False))
                holder = None
        elif cleared_since is not None:
            failure, failure_phase = 'cleared_hold_condition_lost', 'CLEARED_HOLD'
        stage_computation[cycle_stage] += time.perf_counter()-cycle_started
    if failure:
        stage = 'FAILED'
        events.append(dict(kind='failure', absolute_time_s=now, failure=failure, failure_phase=failure_phase,
                           states=current(), simulation_terminated=True, verified_physical_safe_stop=False))
    suffix_wall = time.perf_counter()-attempt_started
    for c, s in current().items():
        rows.append(dict(absolute_time_s=round(now, 10), car=c, x=s[0], vx=s[1], y=s[2], acceleration=previous[c],
                         stage=stage, exit_subphase='', role=role, passer=passer, right_holder=holder or ''))
    # Minimum executed and predicted gaps are derived separately, including EXIT.
    executed_gaps, predicted_gaps = [], []
    for a in paths:
        if a.get('kind') == 'whole_prediction':
            predicted_gaps.append(a.get('minimum_gap_m', a.get('minimum_sampled_gap_m')))
        elif (a.get('kind') == 'executed_segment' and a.get('applied_to_simulation', True)) or a.get('kind') is None:
            executed_gaps.append(a.get('minimum_gap_m', a.get('minimum_sampled_gap_m')))
    summary = dict(policy=policy, role=role, passer=passer, status=stage, failure=failure, failure_phase=failure_phase,
                   exit_success=exit_summary['feasible'], pass_success=stop_confirm is not None,
                   released=complete, overall_success=complete and exit_summary['feasible'], both_tasks_completed=False,
                   simulation_terminated=bool(failure), verified_physical_safe_stop=False,
                   exit_simulation_s=start-cfg.epoch, pass_simulation_s=(stop_confirm-start) if stop_confirm else None,
                   pass_plus_hold_simulation_s=now-start, initial_to_release_simulation_s=(release_time-cfg.epoch) if release_time else None,
                   retreat_distance_m=exit_summary['retreat_distance_m'],
                   passing_distance_m=abs(states[passer][0]-before['states'][passer][0]),
                   grant_time_s=grant-cfg.epoch if grant else None, pass_start_time_s=pass_start-cfg.epoch if pass_start else None,
                   first_actual_body_cleared_time_s=first_clear-cfg.epoch if first_clear else None,
                   first_stop_candidate_time_s=stop_candidate-cfg.epoch if stop_candidate else None,
                   stopped_confirmation_time_s=stop_confirm-cfg.epoch if stop_confirm else None,
                   release_time_s=release_time-cfg.epoch if release_time else None,
                   minimum_executed_gap_m=min(executed_gaps), minimum_predicted_gap_m=min(predicted_gaps),
                   continuity_check=continuity_check, ready_check=gate, parked_reference_actual=reference.tolist(),
                   selection_wait_wall_s=selection_wait,
                   phase_compute_wall_s={'EXIT': exit_summary['generation_compute_wall_seconds'], **stage_computation},
                   suffix_total_wall_s=suffix_wall, phase_primary_calls={'EXIT': exit_summary['primary_solve_calls'], **stage_calls},
                   phase_diagnostic_calls={'EXIT': 0, 'PASS': diagnostic_calls, 'CLEARED_HOLD': 0},
                   phase_retry_counts={'EXIT': 0, 'PASS': 0, 'CLEARED_HOLD': 0},
                   diagnostic_calls=diagnostic_calls, retries=0, role_switch_count=0, geometry=geo,
                   hold_constraints_enabled=hold_constraints, hold_precheck_count=len(hold_prechecks),
                   hold_precheck_failures=sum(not p['passed'] for p in hold_prechecks),
                   final_actual_states=current(), continuous_safety_certified=False, steering_feasibility_tested=False)
    write_json(directory/'config.json', dict(scene=scene, options=options, geometry=geo, policy=policy, role=role))
    write_json(directory/'summary.json', summary)
    write_json(directory/'solves.json', json_finite(records))
    write_json(directory/'path_checks.json', paths)
    write_json(directory/'hold_prechecks.json', json_finite(hold_prechecks))
    with (directory/'actual.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for name, data in (('predictions.jsonl', predictions), ('events.jsonl', events)):
        with (directory/name).open('w') as stream:
            for entry in data:
                stream.write(json.dumps(entry, allow_nan=False)+'\n')
    summary['branch_total_wall_s_including_log_write'] = time.perf_counter()-branch_started
    write_json(directory/'summary.json', summary)
    return summary
