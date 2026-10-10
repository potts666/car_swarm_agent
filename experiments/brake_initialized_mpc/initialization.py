#!/usr/bin/env python3
"""Fresh model-consistent braking seeds; never a fallback controller."""
import numpy as np


def braking_seed(state, config, rollout, magnitude, straighten=False):
    actual = np.array(state, dtype=float).copy()
    dt = config['time']['control_dt_s']
    steering = actual[4]
    controls = []
    for _ in range(config['mpc']['horizon_steps']):
        acceleration = -np.sign(actual[3]) * min(magnitude, abs(actual[3]) / dt)
        if straighten and abs(actual[3]) < 1e-10:
            steering -= np.sign(steering) * min(abs(steering), config['limits']['steering_rate_rad_s'] * dt)
        control = np.array([acceleration, steering])
        controls.append(control)
        actual = rollout(actual, [control], config)[-1]
    return np.array(controls)


def braking_seeds(state, config, rollout):
    maximum = config['limits']['acceleration_m_s2']
    return [('max_brake_hold_steer', braking_seed(state, config, rollout, maximum)),
            ('gentle_brake_hold_steer', braking_seed(state, config, rollout, maximum / 2)),
            ('max_brake_then_straighten', braking_seed(state, config, rollout, maximum, True))]


def inspect(state, controls, config, gear, peer, residuals):
    inequality, equalities = residuals(np.array(state), controls, config, gear, peer)
    bound_acc = config['limits']['acceleration_m_s2'] - abs(controls[:, 0])
    bound_steer = config['limits']['steering_rad'] - abs(controls[:, 1])
    violation = max(0., -float(np.min(inequality)), -float(np.min(bound_acc)),
                    -float(np.min(bound_steer)), float(np.max(abs(equalities))) if len(equalities) else 0.)
    finite = bool(np.all(np.isfinite(controls)) and np.all(np.isfinite(inequality)) and np.all(np.isfinite(equalities)))
    return dict(feasible_against_old_peer_prediction=finite and violation <= config['mpc']['constraint_tolerance'],
                max_violation=violation, inequality_residuals=inequality.tolist(),
                stop_equation_residuals=equalities.tolist(),
                candidate_controls=controls.tolist(), regenerated_from_state=np.asarray(state).tolist(),
                joint_current_execution_safety_certified=False, fallback_executed=False)
