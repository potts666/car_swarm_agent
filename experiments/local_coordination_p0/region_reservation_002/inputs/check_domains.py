"""Exact rational domain, timing and overlap checks for the analytic construction."""
import json
from fractions import Fraction as F
from pathlib import Path
from interval_source import I, Q, sincos, tan
from construct_certificate import SQ7, BODY_ERROR

ROOT = Path(__file__).resolve().parent


def lower(x):
    return F(x.l, Q)


def upper(x):
    return F(x.u, Q)


def main():
    h = F(1, 5)
    rB = F(1807, 500)
    alpha_lo, alpha_hi = F('0.722734247813'), F('0.722734247814')
    assert lower(sincos(I(alpha_lo))[1]) > F(3, 4)
    assert upper(sincos(I(alpha_hi))[1]) < F(3, 4)
    delta_lo, delta_hi = F('0.422853926132'), F('0.422853926134')
    assert upper(tan(I(delta_lo))) < F(9, 20)
    assert lower(tan(I(delta_hi))) > F(9, 20)
    assert delta_hi < F(1, 2) and 2 * delta_hi / (5 * h) < 1
    # tan(d) <= d/(1-d^2/2), since sin(d)<=d and cos(d)>=1-d^2/2.
    k_wait = F(1, 100) / (1 - F(1, 100)**2 / 2) / F(27, 10)
    brake_S = F(1, 500)
    wait_theta = F(1, 10000) + k_wait * brake_S
    wait_pose = F(1, 100) + brake_S
    assert wait_theta < F(108, 1000000)
    assert wait_pose <= F(12, 1000)
    full_wait_body = wait_pose + (80 + rB) * wait_theta
    entry_from_wait = wait_pose + 11 * wait_theta
    assert full_wait_body < BODY_ERROR
    assert entry_from_wait < F(2, 100) and wait_theta < F(2, 10000)
    k_entry = F(1, 1000) / (1 - F(1, 1000)**2 / 2) / F(27, 10)
    theta = F(2, 10000) + F(51, 100) * h * k_entry
    pose_after_reset = F(2, 100) + h * F(1, 100) / 2 + F(51, 100) * h * theta
    exit_pose = pose_after_reset + 56 * theta
    assert theta < F(24, 100000) and exit_pose < F(35, 1000)
    exit_tail_body = F(35, 1000) + (13 + rB) * F(24, 100000)
    assert exit_tail_body < BODY_ERROR
    assert F(35, 1000) + 13 * F(24, 100000) < F(2, 25)
    # Arc length 6*alpha; trapezoidal ZOH profiles have M=5 ramp steps.
    Dlo, Dhi = lower(3 * SQ7), upper(3 * SQ7)
    staging = (Dlo - 7, Dhi - 7)
    exit_line = (12 - Dhi, 12 - Dlo)
    arc = (6 * alpha_lo, 6 * alpha_hi)
    ticks = []
    for length_lo, length_hi, expected in ((staging[0], staging[1], 10),
                                          (arc[0], arc[1], 27),
                                          (exit_line[0], exit_line[1], 26)):
        ceil_lo = -(-(length_lo / h).numerator // (length_lo / h).denominator)
        ceil_hi = -(-(length_hi / h).numerator // (length_hi / h).denominator)
        assert ceil_lo == ceil_hi
        n = 5 + max(5, ceil_hi)
        assert n == expected
        ticks.append(n)
    staging_n, arc_n, exit_n = ticks
    approach = 2 + staging_n + 2 * arc_n + 3 * 5 + 20
    crossing = 1 + 557 + 4
    tail = exit_n + 2 * arc_n + 3 * 5
    assert approach == 101 and crossing == 562 and tail == 95
    assert approach > tail
    overlap_gap = 33 - (24 + Dhi) - 2 * BODY_ERROR
    assert overlap_gap > F(1, 2)
    # Entry reset: .1m nominal; 557 cruise ticks plus four braking ticks.
    assert F(1, 10) + 557 * h / 2 + 4 * h / 4 == 56
    prep = 55
    prep_profiles = {}
    for car, length in (('A1', 5), ('A2', 9), ('B1', 7), ('B2', 10)):
        M = 5
        n = M + max(M, 5 * length)
        V = F(length, 1) / (h * (n - M))
        speed = distance = F(0)
        for j in range(n):
            a = V / (M * h) if j < M else -V / (M * h) if j >= n - M else F(0)
            nxt = speed + h * a
            assert 0 <= nxt <= 1 and abs(a) <= 1
            distance += h * (speed + nxt) / 2
            speed = nxt
        assert speed == 0 and distance == length
        prep_profiles[car] = n
    assert max(prep_profiles.values()) == prep
    assert 6 - F(9, 5) - 2 * BODY_ERROR >= F(1, 2)
    serial = prep + 4 * (approach + crossing + tail)
    overlapped = prep + 4 * (approach + crossing) + tail
    assert serial == 3087 and overlapped == 2802
    values = {'passed': True, 'scope': 'Analytic controls and geometric tube, no rational-input controller replay',
              'domain_bounds': {k: str(v) for k, v in dict(wait_yaw=wait_theta,
                  wait_after_brake_position=wait_pose, wait_full_body=full_wait_body,
                  entry_position_from_wait=entry_from_wait, entry_reset_yaw=theta,
                  exit_position=exit_pose, exit_tail_body=exit_tail_body,
                  simultaneous_tail_approach_gap=overlap_gap).items()},
              'ticks': dict(preparation=prep, approach=approach, crossing=crossing,
                  tail=tail, per_car=approach+crossing+tail, serial=serial, overlapped=overlapped),
              'preparation_profiles': prep_profiles,
              'geometry_certificate_sha256': __import__('hashlib').sha256((ROOT/'geometry_certificate.json').read_bytes()).hexdigest()}
    (ROOT / 'domain_certificate.json').write_text(json.dumps(values, indent=2), encoding='utf-8')
    print(json.dumps(values, indent=2))


if __name__ == '__main__':
    main()
