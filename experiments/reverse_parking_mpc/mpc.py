#!/usr/bin/env python3
"""Signed-speed, timed-reference MPC. Independent of the forward-only baseline."""
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

PREVIOUS = Path(__file__).resolve().parent.parent / 'parking_conflict'
sys.path.insert(0, str(PREVIOUS / 'frozen'))
from optimize_spatial_temporal import static_gaps, rectangle_gap, body_distance


def rollout(state, controls, config, substeps=None):
    steps = substeps or config['time']['model_substeps']
    h = config['time']['control_dt_s'] / steps
    state = np.array(state, dtype=float).copy()
    states = [state.copy()]
    for acceleration, steering in np.asarray(controls):
        for _ in range(steps):
            mid_speed = state[3] + acceleration * h / 2
            omega = mid_speed * np.tan(steering) / config['vehicle']['wheelbase_m']
            mid_yaw = state[2] + omega * h / 2
            state[:2] += h * mid_speed * np.array([np.cos(mid_yaw), np.sin(mid_yaw)])
            state[2] += h * omega
            state[3] += acceleration * h
            state[4] = steering
            states.append(state.copy())
    return np.asarray(states)


def pair_gap(first, second):
    center = second[:, :2] + 1.25 * np.column_stack((np.cos(second[:, 2]), np.sin(second[:, 2])))
    return rectangle_gap(first[:, :2], first[:, 2], center, second[:, 2],
                         np.tile([2.35, 1.0], (len(first), 1)))


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def reference(start, controls, config):
    states = rollout(start, controls, config)
    substeps = config['time']['model_substeps']
    endpoint_speed = states[::substeps, 3]
    gear = np.sign(endpoint_speed[:-1] + endpoint_speed[1:]).astype(int)
    gear[np.abs(endpoint_speed[:-1]) + np.abs(endpoint_speed[1:]) < 1e-8] = 0
    return dict(states=states, controls=np.asarray(controls), gear=gear)


def window(ref, cycle, config):
    n = config['mpc']['horizon_steps']
    substeps = config['time']['model_substeps']
    state_indices = np.minimum(np.arange(cycle * substeps + 1, (cycle + n) * substeps + 1),
                               len(ref['states']) - 1)
    u = np.zeros((n, 2))
    gear = np.zeros(n, dtype=int)
    count = max(0, min(n, len(ref['controls']) - cycle))
    u[:count] = ref['controls'][cycle:cycle + count]
    gear[:count] = ref['gear'][cycle:cycle + count]
    return ref['states'][state_indices], u, gear


def residuals(state, controls, config, gear, peer=None):
    expected_geometry = (4.5, 1.8, 1.0, .1)
    geometry = config['vehicle']
    actual_geometry = tuple(geometry[key] for key in ('length_m', 'width_m', 'rear_overhang_m', 'padding_per_side_m'))
    if actual_geometry != expected_geometry:
        raise ValueError('Frozen SAT helper geometry does not match vehicle configuration')
    future = rollout(state, controls, config)[1:]
    limits = config['limits']
    dt = config['time']['control_dt_s']
    steps = config['time']['model_substeps']
    lateral = future[:, 3] ** 2 * np.tan(future[:, 4]) / config['vehicle']['wheelbase_m']
    checks = [future[:, 3] - limits['speed_m_s'][0], limits['speed_m_s'][1] - future[:, 3],
              limits['steering_rate_rad_s'] * dt - np.abs(np.diff(np.r_[state[4], controls[:, 1]])),
              limits['total_acceleration_m_s2'] - np.hypot(np.repeat(controls[:, 0], steps), lateral),
              static_gaps(future[:, :2], future[:, 2], np.array(config['obstacle_boxes'])) - limits['static_sat_gap_m']]
    repeated_gear = np.repeat(gear, steps)
    moving = repeated_gear != 0
    checks.append(future[moving, 3] * repeated_gear[moving])
    # Gear-zero stages are explicit stopped intervals, including steering setup
    # and reference gear transitions. They are equalities, not sign clipping.
    stop_mask = gear == 0
    stop_mask[:-1] |= gear[1:] == 0
    stop_mask[:-1] |= gear[:-1] * gear[1:] < 0
    endpoint_speeds = state[3] + dt * np.cumsum(controls[:, 0])
    stopped = endpoint_speeds[stop_mask]
    if peer is not None:
        checks.append(pair_gap(future, peer) - limits['pair_sat_gap_m'])
    return np.concatenate(checks), stopped


def solve(state, cycle, ref, config, warm=None, peer=None):
    target, reference_controls, gear = window(ref, cycle, config)
    n = config['mpc']['horizon_steps']
    weights = config['mpc']['weights']
    limits = config['limits']

    def objective(z):
        controls = z.reshape(n, 2)
        future = rollout(state, controls, config)[1:]
        position = future[:, :2] - target[:, :2]
        heading = wrap(future[:, 2] - target[:, 2])
        changes = np.diff(np.vstack(([0, state[4]], controls)), axis=0)
        return float(weights['position'] * np.sum(position ** 2) +
                     weights['yaw'] * np.sum(heading ** 2) +
                     weights['signed_speed'] * np.sum((future[:, 3] - target[:, 3]) ** 2) +
                     weights['control'] * np.sum(controls ** 2) +
                     weights['change'] * np.sum(changes ** 2) +
                     weights['reference_control'] * np.sum((controls - reference_controls) ** 2) +
                     weights['terminal_position'] * np.sum(position[-1] ** 2))

    def inequality(z):
        return residuals(state, z.reshape(n, 2), config, gear, peer)[0]

    def equality(z):
        return residuals(state, z.reshape(n, 2), config, gear, peer)[1]

    stop_mask = gear == 0
    stop_mask[:-1] |= gear[1:] == 0
    stop_mask[:-1] |= gear[:-1] * gear[1:] < 0
    equality_jacobian = np.zeros((int(np.sum(stop_mask)), 2 * n))
    for row, endpoint in enumerate(np.flatnonzero(stop_mask)):
        equality_jacobian[row, :2 * (endpoint + 1):2] = config['time']['control_dt_s']
    constraints = [{'type': 'ineq', 'fun': inequality}]
    if len(equality(reference_controls.ravel())):
        constraints.append({'type': 'eq', 'fun': equality, 'jac': lambda z: equality_jacobian})
    guesses = [('signed_reference', reference_controls)]
    if warm is not None:
        guesses.append(('shifted_solution', warm))
    start = time.perf_counter()
    attempts = []
    accepted = False
    for name, guess in guesses:
        result = minimize(objective, guess.ravel(), method='SLSQP',
                          bounds=[(-limits['acceleration_m_s2'], limits['acceleration_m_s2']),
                                  (-limits['steering_rad'], limits['steering_rad'])] * n,
                          constraints=constraints,
                          options={'maxiter': config['mpc']['max_iterations'],
                                   'ftol': config['mpc']['ftol'], 'eps': config['mpc']['finite_difference_step']})
        inequality_error = max(0., -float(np.min(inequality(result.x))))
        equalities = equality(result.x)
        equality_error = float(np.max(abs(equalities))) if len(equalities) else 0.
        violation = max(inequality_error, equality_error)
        accepted = bool(result.success and np.all(np.isfinite(result.x)) and
                        violation <= config['mpc']['constraint_tolerance'])
        attempts.append(dict(initialization=name, success=bool(result.success), accepted=accepted,
                             message=str(result.message), iterations=int(result.nit),
                             violation=violation, objective=float(result.fun)))
        if accepted:
            break
    return result.x.reshape(n, 2), dict(accepted=accepted, solve_seconds=time.perf_counter() - start,
                                       attempts=attempts, gear=gear.tolist(),
                                       reference_controls=reference_controls.tolist(),
                                       reference_states=target.tolist(), violation=violation)
