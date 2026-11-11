"""Offline rear-axle bicycle execution and shared stop/steer motion primitives."""
from collections import deque
import math
import numpy as np

CARS = ('A', 'B')
SIGN = {'A': 1, 'B': -1}


def derivative(z, u, vehicle):
    x, y, psi, v, delta = z
    return np.array([v*math.cos(psi), v*math.sin(psi),
                     v*math.tan(delta)/vehicle['L'], u[0], u[1]])


def step(z, u, dt, vehicle):
    k1 = derivative(z, u, vehicle)
    k2 = derivative(z+dt*k1/2, u, vehicle)
    k3 = derivative(z+dt*k2/2, u, vehicle)
    k4 = derivative(z+dt*k3, u, vehicle)
    return z+dt*(k1+2*k2+2*k3+k4)/6


def rollout(z, commands, scene, cycles=None):
    h, ds = scene['h'], scene['sample_s']
    count = round(h/ds)
    assert abs(count*ds-h) < 1e-10
    controls = list(commands)
    p = scene['vehicle']; tolerance = scene['tolerance']
    if not np.isfinite(z).all() or any(not np.isfinite([a,w]).all() or
            a < p['a_min']-tolerance or a > p['a_max']+tolerance or
            abs(w) > p['omega_max']+tolerance for a,w,_ in controls):
        raise ValueError('non-finite state or actuator command outside configured bounds')
    if cycles is not None:
        controls = controls[:cycles]+[(0., 0., 'HOLD')]*max(0, cycles-len(controls))
    states = [np.array(z, float)]
    for a, omega, _ in controls:
        for _ in range(count):
            states.append(step(states[-1], (a, omega), ds, scene['vehicle']))
    return np.array(states)


def drive(distance, scene, stage, limit=None):
    """Exact-distance rest-to-rest trapezoid; identical time scaling in both policies."""
    if abs(distance) < 1e-9:
        return []
    h = scene['h']; p = scene['vehicle']; direction = np.sign(distance)
    vmax = (p['v_fwd'] if direction > 0 else p['v_rev']) if limit is None else limit
    amax = min(p['a_max'], -p['a_min'])
    n = max(1, math.ceil(min(vmax/amax, math.sqrt(abs(distance)/amax))/h))
    cruise = max(0, math.ceil(abs(distance)/(vmax*h)-n))
    acceleration = abs(distance)/(h*h*n*(n+cruise))
    while acceleration > amax+1e-12 or acceleration*n*h > vmax+1e-12:
        cruise += 1
        acceleration = abs(distance)/(h*h*n*(n+cruise))
    a = float(direction*acceleration)
    return [(a, 0., stage)]*n+[(0., 0., stage)]*cruise+[(-a, 0., stage)]*n


def steer(old, new, scene, stage):
    if abs(new-old) < 1e-10:
        return []
    n = math.ceil(abs(new-old)/(scene['vehicle']['omega_max']*scene['h']))
    return [(0., (new-old)/(n*scene['h']), stage)]*n


def shift(scene, returning=False, stage='ENTRY'):
    """Two tangent circular arcs; curvature changes only at a full stop."""
    radius = scene['turn_radius_m']; offset = scene['bay_offset_local_y']
    angle = math.acos(1-offset/(2*radius))
    delta = math.atan(scene['vehicle']['L']/radius)*(-1 if returning else 1)
    return (steer(0., delta, scene, stage)+drive(radius*angle, scene, stage, limit=1.)+
            steer(delta, -delta, scene, stage)+drive(radius*angle, scene, stage, limit=1.)+
            steer(-delta, 0., scene, stage))


def shift_advance(scene):
    r = scene['turn_radius_m']
    return 2*r*math.sin(math.acos(1-scene['bay_offset_local_y']/(2*r)))


def stop(z, scene, stage='QUEUE_BRAKE'):
    n = math.ceil(abs(z[3])/(min(-scene['vehicle']['a_min'], scene['vehicle']['a_max'])*scene['h'])-1e-10)
    return [] if n == 0 else [(-z[3]/(n*scene['h']), 0., stage)]*n


def bay_entry(z, car, scene, recovery=False):
    commands = stop(z, scene)
    stopped = rollout(z, commands, scene)[-1]
    distance = SIGN[car]*(-SIGN[car]*scene['waiting_anchor_abs_x']-stopped[0])
    commands += drive(distance, scene, 'RETREAT' if recovery else 'QUEUE_APPROACH')
    commands += shift(scene)
    return commands


def return_plan(z, car, scene):
    commands = shift(scene, returning=True, stage='RETURN')
    end = rollout(z, commands, scene)[-1]
    commands += drive(SIGN[car]*(scene['goals'][car][0]-end[0]), scene, 'DEPART')
    return commands


def corners(states, vehicle):
    states = np.atleast_2d(states)
    local = np.array([[-vehicle['rear'], -vehicle['width']/2],
                      [vehicle['L']+vehicle['front'], -vehicle['width']/2],
                      [vehicle['L']+vehicle['front'], vehicle['width']/2],
                      [-vehicle['rear'], vehicle['width']/2]])
    c, s = np.cos(states[:, 2]), np.sin(states[:, 2])
    x = states[:, 0, None]+c[:, None]*local[:, 0]-s[:, None]*local[:, 1]
    y = states[:, 1, None]+s[:, None]*local[:, 0]+c[:, None]*local[:, 1]
    return np.stack([x, y], axis=-1)


def clipped_area(poly, rectangle):
    """Sutherland-Hodgman clipping, including a body straddling road pieces."""
    points = list(poly)
    for axis, bound, side in ((0,rectangle[0],1),(0,rectangle[1],-1),
                              (1,rectangle[2],1),(1,rectangle[3],-1)):
        out = []
        for a, b in zip(points, points[1:]+points[:1]):
            ina, inb = side*(a[axis]-bound) >= 0, side*(b[axis]-bound) >= 0
            if ina:
                out.append(a)
            if ina != inb:
                out.append(a+(b-a)*(bound-a[axis])/(b[axis]-a[axis]))
        points = out
        if not points:
            return 0.
    a = np.array(points)
    return abs(np.sum(a[:,0]*np.roll(a[:,1],-1)-a[:,1]*np.roll(a[:,0],-1)))/2


def road_cells(rectangles):
    xs = sorted({x for r in rectangles for x in r[:2]})
    ys = sorted({y for r in rectangles for y in r[2:]})
    return [[a,b,c,d] for a,b in zip(xs,xs[1:]) for c,d in zip(ys,ys[1:])
            if any(r[0] <= (a+b)/2 <= r[1] and r[2] <= (c+d)/2 <= r[3] for r in rectangles)]


def outside_area(polygons, scene):
    rectangles = scene['road_rectangles']; cells = road_cells(rectangles)
    lo, hi = polygons.min(axis=1), polygons.max(axis=1)
    contained = np.zeros(len(polygons), bool)
    for a,b,c,d in rectangles:
        contained |= (lo[:,0] >= a-1e-10)&(hi[:,0] <= b+1e-10)&(lo[:,1] >= c-1e-10)&(hi[:,1] <= d+1e-10)
    residual = np.zeros(len(polygons))
    for i in np.flatnonzero(~contained):
        residual[i] = max(0., scene['vehicle']['length']*scene['vehicle']['width']-
                          sum(clipped_area(polygons[i], r) for r in cells))
    return residual


def gaps(pa, pb):
    """Euclidean distance between oriented rectangles, zero if SAT overlaps."""
    separated = np.zeros(len(pa), bool)
    best = np.full(len(pa), np.inf)
    for poly in (pa, pb):
        for edge in (poly[:,1]-poly[:,0], poly[:,2]-poly[:,1]):
            axis = np.stack([-edge[:,1], edge[:,0]], axis=-1)
            a = np.einsum('nki,ni->nk', pa, axis)
            b = np.einsum('nki,ni->nk', pb, axis)
            separated |= (a.max(1) < b.min(1)) | (b.max(1) < a.min(1))
    for source, target in ((pa,pb),(pb,pa)):
        for i in range(4):
            e0, e1 = target[:,i], target[:,(i+1)%4]
            edge = e1-e0
            q = source-e0[:,None,:]
            t = np.clip(np.einsum('nki,ni->nk',q,edge)/np.sum(edge*edge,axis=1)[:,None],0.,1.)
            d = np.linalg.norm(q-t[:,:,None]*edge[:,None,:],axis=2).min(1)
            best = np.minimum(best,d)
    return np.where(separated,best,0.)


def audit(motions, scene):
    if any(not np.isfinite(z).all() for z in motions.values()):
        return dict(passed=False, residuals={'non_finite_state': True}, minimum_gap_m=None)
    residuals = {}; bodies = {c:corners(z,scene['vehicle']) for c,z in motions.items()}
    p = scene['vehicle']; minimum = None
    for c,z in motions.items():
        residuals[c] = dict(outside_road_area_m2=float(outside_area(bodies[c],scene).max()),
            forward_speed_m_s=float(max(0.,z[:,3].max()-p['v_fwd'])),
            reverse_speed_m_s=float(max(0.,-z[:,3].min()-p['v_rev'])),
            steering_rad=float(max(0.,np.abs(z[:,4]).max()-p['delta_max'])))
    if len(bodies) == 2:
        minimum = float(gaps(bodies['A'],bodies['B']).min())
        residuals['pair_gap_m'] = max(0.,scene['s_min']-minimum)
    values = [v for r in residuals.values() for v in (r.values() if isinstance(r,dict) else [r])]
    return dict(passed=all(v <= scene['tolerance'] for v in values), residuals=residuals, minimum_gap_m=minimum)


def completed(z, goal, scene):
    tol = scene['terminal']; error = z-np.array(goal)
    yaw = math.atan2(math.sin(error[2]),math.cos(error[2]))
    return bool(np.linalg.norm(error[:2]) <= tol['position_m'] and abs(yaw) <= tol['yaw_rad'] and
                abs(error[3]) <= tol['speed_m_s'] and abs(error[4]) <= tol['steering_rad'])


def arbitrate(owner, requests):
    """One authoritative owner; timestamp then ID breaks simultaneous requests."""
    winner = min(requests,key=lambda item:(item[0],item[1]))[1] if owner is None and requests else owner
    return winner, {car:car == winner for _,car in requests}


def validate_scene(scene):
    p = scene['vehicle']; h = scene['h']; ds = scene['sample_s']
    if (h <= 0 or ds <= 0 or abs(round(h/ds)*ds-h)>1e-10 or scene['N']<2 or
            min(p['length'],p['width'],p['L'],p['v_fwd'],p['v_rev'],p['omega_max'])<=0 or
            min(p['front'],p['rear'])<0 or abs(p['length']-p['rear']-p['L']-p['front'])>1e-10 or
            not 0<p['delta_max']<math.pi/2 or not p['a_min']<0<p['a_max'] or
            scene['turn_radius_m']<p['L']/math.tan(p['delta_max'])):
        raise ValueError('inconsistent vehicle geometry, actuator limits or integration grid')
    if not 0<scene['d_A']<scene['d_B'] or scene['g']<=p['rear']:
        raise ValueError('invalid arrival offset or goal clearance')
    for car in CARS:
        z=np.array(scene['initial'][car]); expected=-SIGN[car]*(scene['D']/2+scene['d_'+car]+p['L']+p['front'])
        if (np.max(np.abs(z-[expected,0.,0. if car=='A' else math.pi,0.,0.]))>scene['tolerance'] or
                scene['previous_input'][car]!=[0.,0.]):
            raise ValueError('initial state disagrees with full-body distance convention')
        goal=[SIGN[car]*(scene['D']/2+scene['g']),0.,z[2],0.,0.]
        if np.max(np.abs(np.array(scene['goals'][car])-goal))>scene['tolerance']:
            raise ValueError('goal is not the configured symmetric original task')
    if not audit({c:np.array([scene['initial'][c]]) for c in CARS},scene)['passed']:
        raise ValueError('initial full bodies violate road or separation')
