"""Rational small-case certificates from the supplied motion theory, sections 3–5.

These are isolated four-car fixtures, not the current task's full connector graph.
Nominal receiver centers are defined by the exact ZOH flow and saved inputs;
floating-point coordinates are only auxiliary observations, never certified centers.
"""
from fractions import Fraction as F
import itertools


def radius_step(rp, rpsi, rv, rdelta):
    """Motion equations (22)–(25), with rational tan/sec^2 upper bounds."""
    if min(rp, rpsi, rv, rdelta) < 0:
        raise ValueError('negative radius')
    h, wheelbase, vmax = F(1, 5), F(27, 10), F(2)
    q = F(11, 20)/wheelbase*rv + vmax*F(64, 49)/wheelbase*rdelta
    return (rp+h*rv/2+vmax*h*rpsi+vmax*h*h*q/3,
            rpsi+h*q/2)


def curved_fixture_certificate():
    h, rb, safety = F(1, 5), F(1807, 500), F(1, 2)
    if rb*rb < F(7, 2)**2+F(9, 10)**2:
        raise ValueError('invalid corner-radius bound')
    # cos(.1)>=199/200, tan(.1)<=20/199, log(1+x)<=x.
    nominal_displacement = F(1, 10)+rb*F(10, 5373)+F(13, 100)*(1+rb*F(200, 5373))
    reference_inputs = [(F(0), F(1, 2)), (F(-1), F(0)),
                        (F(-1), F(0)), (F(-1, 2), F(0)), (F(0), F(-1, 2))]
    rp, rpsi, rv, rd = F(5, 1000), F(1, 1000), F(2, 100), F(1, 1000)
    initial_radii = dict(position_ball=rp, heading=rpsi, speed=rv, steering=rd)
    v, delta = F(1, 2), F(0)
    stages = []
    for j, (a, omega) in enumerate(reference_inputs):
        correction_a, correction_omega = rv/h, rd/h
        input_margin = min(1-abs(a)-correction_a, 1-abs(omega)-correction_omega)
        next_v, next_delta = v+h*a, delta+h*omega
        speed_margin = min(v-rv, 2-v-rv, next_v, 2-next_v)
        steering_margin = min(F(1, 2)-abs(delta)-rd, F(1, 2)-abs(next_delta))
        if min(input_margin, speed_margin, steering_margin) < 0:
            raise ValueError('invalid feedback domain')
        next_rp, next_rpsi = radius_step(rp, rpsi, rv, rd)
        stages.append(dict(stage=j, reference_v_delta=[v, delta],
            reference_next_v_delta=[next_v, next_delta], reference_input=[a, omega],
            source_radii=dict(position_ball=rp, heading=rpsi, speed=rv, steering=rd),
            receiver_radii=dict(position_ball=next_rp, heading=next_rpsi, speed=F(0), steering=F(0)),
            input_margin=input_margin, speed_margin=speed_margin, steering_margin=steering_margin))
        rp, rpsi, rv, rd = next_rp, next_rpsi, F(0), F(0)
        v, delta = next_v, next_delta
    if v or delta:
        raise ValueError('terminal zero input would not give a stationary receiver')
    # Radius bounds are monotone over each holding interval. Use the final
    # radii for the entire chain, and add nominal motion by the triangle inequality.
    tube = rp+rb*rpsi
    displacement = nominal_displacement+tube
    pairs = {'A1:A2':F(69, 2)-displacement, 'A1:B1':F(30)-displacement,
             'A1:B2':F(33)-displacement, 'A2:B1':F(69),
             'A2:B2':F(72), 'B1:B2':F(21, 5)}
    assert len(pairs) == len(list(itertools.combinations(range(4), 2)))
    road_margin = min(F(1, 2)-displacement, F(8, 5))
    pair_margin = min(pairs.values())-safety
    return dict(passed=min(road_margin, pair_margin) >= 0,
        scope='isolated source domain around A1=(0,0,0,.5,0); other three cars fixed',
        source_reference=[F(0), F(0), F(0), F(1, 2), F(0)], source_radii=initial_radii,
        model=dict(wheelbase=F(27,10), speed=[F(0),F(2)], acceleration=[F(-1),F(1)],
                   steering=[F(-1,2),F(1,2)], steering_rate=[F(-1),F(1)]),
        road_rectangles=[[-52,52,F(-7,5),F(7,5)],[-52,-10,F(-11,2),F(11,2)],
                         [10,52,F(-11,2),F(11,2)]],
        stationary_cars={'A2':dict(position=[-39,-3], heading='0'),
                         'B1':dict(position=[37,3], heading='pi'),
                         'B2':dict(position=[40,-3], heading='pi')},
        reference_inputs=reference_inputs, dt=h, corner_radius_bound=rb,
        nominal_body_displacement_bound=nominal_displacement,
        whole_source_body_error_bound=tube, whole_source_displacement_bound=displacement,
        minimum_road_margin=road_margin, pair_lower_bounds=pairs,
        minimum_pair_margin=pair_margin, stages=stages,
        receiver_centers='exact ZOH flow of source_reference under reference_inputs prefixes',
        feedback='a=reference_a-(v-reference_v)/h; omega=reference_omega-(delta-reference_delta)/h',
        heading_branch='continuous real branch around the nominal reference',
        authority=dict(owner='A1', other_body_x_protection_margins=[F(35,2),F(31,2),F(37,2)]),
        nominal_node_integration_error='no numerical center used in the proof; exact-flow definition',
        arithmetic='exact rational bound evaluation; analytic inequalities from motion §§3,5',
        source_set_inclusion='position ball and heading interval radii from (24),(25); v/delta reset by (19)',
        terminal_continuation=[F(0),F(0)], terminal_invariant=True,
        full_task_goal=False, full_task_route_certified=False,
        same_nominal_source_as_current_task=False)
