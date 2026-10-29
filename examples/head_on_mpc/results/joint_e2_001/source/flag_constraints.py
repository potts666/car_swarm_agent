"""E1 fixed-reference progress bounds; no QP solver is claimed here."""
import numpy as np
from bicycle_core import corners, clipped_area, rollout


def construct(car, g, state, commands, phase, scene):
    if car not in ('A', 'B') or type(g) is not int or g not in (0, 1):
        raise ValueError('invalid vehicle or received permission')
    reference = rollout(state, list(commands), scene)
    # Progress is cumulative rear-axle arc length on this fixed remaining reference.
    progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reference[:, :2], axis=0), axis=1))]
    region = [-scene['admission_protection_abs_x'], scene['admission_protection_abs_x'],
              -scene['W']/2, scene['W']/2]
    overlap = np.array([clipped_area(b, region) for b in corners(reference, scene['vehicle'])])
    bad = np.flatnonzero(overlap > scene['tolerance'])
    upper = float(progress[max(0, bad[0]-1)] if len(bad) else progress[-1]) if not g else float(progress[-1])
    return dict(g_i=g, phase=phase, mode='PASS' if g else 'WAIT',
                s_origin_m=0., s_min_m=0., s_max_m=upper,
                reference_end_s_m=float(progress[-1]), terminal_stop=not g,
                forbidden_body_rectangle=None if g else region,
                road_rectangles=scene['road_rectangles'], vehicle_limits=scene['vehicle'],
                pair_minimum_gap_m=scene['s_min'], sample_s=scene['sample_s'],
                qp_implemented=False)


def accepts(parameters, prediction, scene):
    progress = float(np.linalg.norm(np.diff(prediction[:, :2], axis=0), axis=1).sum())
    if progress > parameters['s_max_m'] + scene['tolerance']:
        return False
    region = parameters['forbidden_body_rectangle']
    return region is None or all(clipped_area(b, region) <= scene['tolerance']
                                 for b in corners(prediction, scene['vehicle']))
