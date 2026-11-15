"""Isolated P0 checks; no changes to the historical controller or its contracts."""
from fractions import Fraction as F
import math

import numpy as np
from scipy.integrate import solve_ivp


def rational(value):
    return F(str(value))


def maximum_distance(n, h, acceleration, vmax):
    """Exact rest-to-rest upper bound for uniform ZOH acceleration intervals."""
    velocities = [min(k*h*acceleration, (n-k)*h*acceleration, vmax)
                  for k in range(n+1)]
    return h*sum(velocities), velocities


def shortest_straight(distance, scene):
    h = rational(scene['h'])
    p = scene['vehicle']
    if rational(p['a_max']) != -rational(p['a_min']):
        raise ValueError('shortest-time bound currently requires symmetric acceleration limits')
    acceleration = min(rational(p['a_max']), -rational(p['a_min']))
    vmax = rational(p['v_fwd'])
    distance = rational(distance)
    if distance <= 0 or min(h, acceleration, vmax) <= 0:
        raise ValueError('positive forward distance and limits required')
    n = 1
    while maximum_distance(n, h, acceleration, vmax)[0] < distance:
        n += 1
    bound, velocities = maximum_distance(n, h, acceleration, vmax)
    velocities = [v*distance/bound for v in velocities]
    inputs = [(b-a)/h for a, b in zip(velocities, velocities[1:])]
    return dict(steps=n, seconds=n*h, accelerations=inputs,
                velocities=velocities, distance=distance,
                shorter_distance_bound=maximum_distance(n-1, h, acceleration, vmax)[0])


def straight_source_certificate(source, accelerations, road, scene):
    """Exact rational enclosure, ONLY for psi=delta=omega=0 and one road box.

    Covers every x/y/v in the declared source box and every continuous time.
    This is not a curved connector, permission, or multi-car certificate.
    """
    box = [[rational(x) for x in interval] for interval in source]
    if len(box) != 5 or any(lo > hi for lo, hi in box):
        raise ValueError('invalid five-state box')
    if box[2] != [0, 0] or box[4] != [0, 0]:
        raise ValueError('only exact zero heading and steering supported')
    road = list(map(rational, road))
    p = scene['vehicle']; h = rational(scene['h'])
    xmin, xmax = box[0]; ymin, ymax = box[1]; vlo, vhi = box[3]
    all_x = [xmin, xmax]; all_v = [vlo, vhi]
    inputs = list(map(rational, accelerations))
    if not inputs:
        raise ValueError('empty connector')
    for a in inputs:
        if not rational(p['a_min']) <= a <= rational(p['a_max']):
            raise ValueError('acceleration bound')
        for x, v in ((xmin, vlo), (xmax, vhi)):
            times = [F(0), h]
            if a and 0 < -v/a < h:
                times.append(-v/a)
            all_x.extend(x+t*v+a*t*t/2 for t in times)
        xmin += h*vlo+a*h*h/2
        xmax += h*vhi+a*h*h/2
        vlo += h*a; vhi += h*a
        all_v.extend([vlo, vhi])
    body_bounds = [min(all_x)-rational(p['rear']),
                   max(all_x)+rational(p['L'])+rational(p['front']),
                   ymin-rational(p['width'])/2, ymax+rational(p['width'])/2]
    margins = [body_bounds[0]-road[0], road[1]-body_bounds[1],
               body_bounds[2]-road[2], road[3]-body_bounds[3]]
    speed_margin = min(min(all_v)+rational(p['v_rev']), rational(p['v_fwd'])-max(all_v))
    return dict(passed=min(margins+[speed_margin]) >= 0,
                terminal_box=[[xmin, xmax], box[1], box[2], [vlo, vhi], box[4]],
                swept_body_box=body_bounds, road_margins=margins,
                speed_margin=speed_margin, arithmetic='exact rational',
                integration_error=F(0), source_dimension='x,y,v; psi=delta=0',
                covers_six_pairs=False, covers_permissions=False)


def check_seam(end, start, receiver, tail, required_tail, context, expected, tolerance):
    """Provisional full-state/default-tail guard, not the controller protocol."""
    end, start, receiver = np.asarray(end), np.asarray(start), np.asarray(receiver)
    if end.shape != (5,) or start.shape != (5,) or receiver.shape != (5, 2):
        raise ValueError('invalid full-state shape')
    if not all(np.isfinite(v).all() for v in (end, start, receiver)) or not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('nonfinite state or invalid tolerance')
    if np.any(receiver[:, 0] > receiver[:, 1]):
        raise ValueError('empty receiver')
    if context != expected:
        raise ValueError('stale version, deadline, or permission')
    if np.max(abs(end-start)) > tolerance:
        raise ValueError('full-state seam discontinuity')
    if np.any(end < receiver[:, 0]-tolerance) or np.any(end > receiver[:, 1]+tolerance):
        raise ValueError('outside receiver')
    tail, required_tail = np.asarray(tail), np.asarray(required_tail)
    if tail.shape != (2,) or required_tail.shape != (2,) or not np.isfinite(tail).all() or not np.isfinite(required_tail).all():
        raise ValueError('invalid continuation input')
    if np.max(abs(tail-required_tail)) > tolerance:
        raise ValueError('default continuation input mismatch')


def independent_rollout(start, commands, scene):
    """DOP853 comparison: numerical calibration, without a rigorous error tube."""
    h = scene['h']; wheelbase = scene['vehicle']['L']
    z = np.array(start, dtype=float); states = [z.copy()]
    for a, omega, _ in commands:
        def rhs(_, q):
            return [q[3]*math.cos(q[2]), q[3]*math.sin(q[2]),
                    q[3]*math.tan(q[4])/wheelbase, a, omega]
        result = solve_ivp(rhs, (0, h), z, method='DOP853', rtol=1e-12, atol=1e-13)
        if not result.success:
            raise RuntimeError(result.message)
        z = result.y[:, -1]; states.append(z.copy())
    return np.asarray(states)
